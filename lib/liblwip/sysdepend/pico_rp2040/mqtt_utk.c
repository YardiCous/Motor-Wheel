/** @file mqtt_utk.c
 * @brief Binary robot telemetry and commands on the existing network owner.
 * NOTE: Only cyw43_utk_task and its lwIP callbacks execute this module.
 */
#include "mqtt_utk.h"

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "cyw43_utk.h"
#include "wifi_secrets.h"
#include "../../../../app_program/data_formats.h"
#include "../../../../app_program/robot_os.h"
#include "../../../../app_program/subsystem1/telemetry.h"

#include <string.h>

#define MQTT_UTK_PERIOD_MS 250u
#define MQTT_UTK_RETRY_MAX_MS 30000u
#define MQTT_UTK_ACK_LIMIT 8u
#define MQTT_UTK_CONNECTING 1u
#define MQTT_UTK_SUBSCRIBING 2u
#define MQTT_UTK_ONLINE 3u
#define MQTT_UTK_RETRY 4u
#define MQTT_UTK_TELEMETRY "robot/telemetry"
#define MQTT_UTK_STATUS "robot/status"
#define MQTT_UTK_COMMAND "robot/command"
#define MQTT_UTK_ACK "robot/ack"

/** @brief Retry state has five values and an exponent bounded to five. */
typedef struct
{
    uint32_t state : 3;
    uint32_t retry_exponent : 3;
    bool     b_timer_started : 1;
    uint32_t retry_at;
} mqtt_utk_state_t;

static mqtt_client_t *  gp_client = NULL;
static ip_addr_t        g_broker_address;
static mqtt_utk_state_t g_connection = {0};
static telemetry_t      g_receiver   = {0};

static void schedule_retry(void);
static void connect_client(void);
static void incoming_publish(void * p_argument, const char * p_topic,
                             u32_t total);
static void incoming_data(void * p_argument, const u8_t * p_data, u16_t length,
                          u8_t flags);
static void subscribe_done(void * p_argument, err_t result);
static void publish_done(void * p_argument, err_t result);
static void connection_changed(mqtt_client_t * p_client, void * p_argument,
                               mqtt_connection_status_t status);
static void network_tick(void * p_argument);
static void send_ack(uint8_t ack);

/** @brief Failed attempts use a bounded exponential retry deadline. */
void mqtt_utk_start (void)
{
    if (!g_connection.b_timer_started)
    {
        sys_timeout(MQTT_UTK_PERIOD_MS, network_tick, NULL);
        g_connection.b_timer_started = true;
    }

    bool b_busy = (MQTT_UTK_CONNECTING == g_connection.state) ||
                  (MQTT_UTK_SUBSCRIBING == g_connection.state) ||
                  (MQTT_UTK_ONLINE == g_connection.state);
    bool b_waiting =
        (MQTT_UTK_RETRY == g_connection.state) &&
        (0u != ((sys_now() - g_connection.retry_at) & 0x80000000u));

    if ((!b_busy) && (!b_waiting))
    {
        connect_client();
    }
}

/** @brief Allocate at most one client and validate the configured broker. */
static void connect_client (void)
{
    if (NULL == gp_client)
    {
        gp_client = mqtt_client_new();
    }

    bool b_valid =
        (NULL != gp_client) &&
        (0 != ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&g_broker_address)));

    if (b_valid)
    {
        struct mqtt_connect_client_info_t configuration = {0};
        IP_SET_TYPE(&g_broker_address, IPADDR_TYPE_V4);
        configuration.client_id  = MQTT_CLIENT_ID;
        configuration.keep_alive = 15u;
        g_connection.state       = MQTT_UTK_CONNECTING;
        err_t result =
            mqtt_client_connect(gp_client, &g_broker_address, MQTT_BROKER_PORT,
                                connection_changed, NULL, &configuration);

        if (ERR_OK != result)
        {
            schedule_retry();
        }
    }
    else
    {
        schedule_retry();
    }
}

/** @brief Exactly one timer chain is created for the lifetime of the client.
 */
static void schedule_retry (void)
{
    uint32_t delay = 1000u << g_connection.retry_exponent;
    delay = (delay > MQTT_UTK_RETRY_MAX_MS) ? MQTT_UTK_RETRY_MAX_MS : delay;
    g_connection.retry_at = sys_now() + delay;
    g_connection.state    = MQTT_UTK_RETRY;
    g_connection.retry_exponent +=
        (g_connection.retry_exponent < 5u) ? 1u : 0u;
    robot_os_network(true, false);
}

/** @brief No topic or payload buffer can grow with an untrusted publish. */
static void incoming_publish (void * p_argument, const char * p_topic,
                             u32_t total)
{
    (void)p_argument;
    telemetry_begin(&g_receiver, total,
                    0 == strcmp(p_topic, MQTT_UTK_COMMAND));

    if (0u == total)
    {
        send_ack(128u);
    }
}

/** @brief Fragment checks preserve all binary values, including zero. */
static void incoming_data (void * p_argument, const u8_t * p_data, u16_t length,
                          u8_t flags)
{
    (void)p_argument;
    bool b_last  = (0u != (flags & MQTT_DATA_FLAG_LAST));
    bool b_ready = telemetry_fragment(&g_receiver, p_data, length, b_last);

    if (b_ready)
    {
        /* A validated command has exactly six meaningful payload bits. */
        bool b_queued = robot_os_submit((uint8_t)g_receiver.payload);

        if (!b_queued)
        {
            send_ack(128u);
        }
    }
    else if (b_last)
    {
        send_ack(128u);
    }
    else
    {
        /* Wait for the last fragment without blocking the service. */
    }
}

/** @brief Online status requires successful restoration of the subscription.
 */
static void subscribe_done (void * p_argument, err_t result)
{
    (void)p_argument;

    if (ERR_OK == result)
    {
        g_connection.state          = MQTT_UTK_ONLINE;
        g_connection.retry_exponent = 0u;
        robot_os_network(true, true);
    }
    else
    {
        mqtt_disconnect(gp_client);
        schedule_retry();
    }
}

/** @brief Failed telemetry is replaced by the next fresh snapshot. */
static void publish_done (void * p_argument, err_t result)
{
    (void)p_argument;

    if (ERR_OK != result)
    {
        robot_os_network(true, false);
    }
}

/** @brief Every accepted connection installs callbacks before subscribing. */
static void connection_changed (mqtt_client_t * p_client, void * p_argument,
                               mqtt_connection_status_t status)
{
    (void)p_client;
    (void)p_argument;

    if (MQTT_CONNECT_ACCEPTED == status)
    {
        g_connection.state = MQTT_UTK_SUBSCRIBING;
        mqtt_set_inpub_callback(gp_client, incoming_publish, incoming_data,
                                NULL);
        err_t result = mqtt_subscribe(gp_client, MQTT_UTK_COMMAND, 0u,
                                      subscribe_done, NULL);

        if (ERR_OK != result)
        {
            mqtt_disconnect(gp_client);
            schedule_retry();
        }
    }
    else
    {
        schedule_retry();
    }
}

/** @brief Publishing runs at 4 Hz entirely within the network service task. */
static void network_tick (void * p_argument)
{
    (void)p_argument;
    T_CYW43_UTK_STATUS radio = {0};
    cyw43_utk_get_status(&radio);
    bool b_wifi   = (2 == radio.link_status);
    bool b_online = b_wifi && (MQTT_UTK_ONLINE == g_connection.state) &&
                    (0u != mqtt_client_is_connected(gp_client));
    robot_os_network(b_wifi, b_online);

    if (b_online)
    {
        uint8_t packet[DATA_FORMATS_TELEMETRY_BYTES] = {0u};

        if (robot_os_telemetry(packet))
        {
            (void)mqtt_publish(gp_client, MQTT_UTK_TELEMETRY, packet,
                               DATA_FORMATS_TELEMETRY_BYTES, 0u, 0u,
                               publish_done, NULL);
        }

        uint8_t status = DATA_FORMATS_VERSION | 4u | 8u;
        (void)mqtt_publish(gp_client, MQTT_UTK_STATUS, &status, 1u, 0u, 0u,
                           publish_done, NULL);

        for (uint8_t slot = 0u; slot < MQTT_UTK_ACK_LIMIT; ++slot)
        {
            uint8_t ack = 0u;

            if (robot_os_ack(&ack))
            {
                send_ack(ack);
            }
        }
    }

    sys_timeout(MQTT_UTK_PERIOD_MS, network_tick, NULL);
}

/** @brief No acknowledgement is retained for a later session. */
static void send_ack (uint8_t ack)
{
    if ((NULL != gp_client) && (0u != mqtt_client_is_connected(gp_client)))
    {
        (void)mqtt_publish(gp_client, MQTT_UTK_ACK, &ack, 1u, 0u, 0u,
                           publish_done, NULL);
    }
}

/*** End of file. ***/

