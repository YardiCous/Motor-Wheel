#include "lwip/apps/mqtt.h"
#include "pico/cyw43_arch.h"
#include "robot.h"
/* Needed to reject retained/DUP STARTs; this adapter is verified against SDK 2.3.1. */
#include "lwip/apps/mqtt_priv.h"
#include "lwip/dns.h"
#include <string.h>

#if __has_include("robot_secrets.h")
#include "robot_secrets.h"
#endif
#ifndef ROBOT_WIFI_SSID
#define ROBOT_WIFI_SSID ""
#define ROBOT_WIFI_PASSWORD ""
#endif
#ifndef ROBOT_MQTT_HOST
#define ROBOT_MQTT_HOST "192.168.1.10"
#endif
#ifndef ROBOT_MQTT_PORT
#define ROBOT_MQTT_PORT 1883
#endif
#ifndef ROBOT_MQTT_USER
#define ROBOT_MQTT_USER NULL
#define ROBOT_MQTT_PASSWORD NULL
#endif
static mqtt_client_t *client;
static ip_addr_t broker;
static struct {
    unsigned state : 3;
    unsigned initialized : 1;
    unsigned subscribed : 1;
    unsigned dns_busy : 1;
    unsigned dns_ready : 1;
    unsigned connecting : 1;
    unsigned rx_valid : 1;
    unsigned rx_unsafe_start : 1;
    unsigned seen_sequence : 1;
    unsigned last_sequence : 6;
    unsigned sequence : 6;
    unsigned backoff_seconds : 5;
    uint16_t entered_at, published_at, status_at;
} network;
static void retry(void) {
    network.state = NET_RETRY;
    network.subscribed = 0;
    network.entered_at = robot_now();
    unsigned next = network.backoff_seconds ? network.backoff_seconds * 2u : 1u;
    network.backoff_seconds = next > 30 ? 30 : next;
    network.connecting = 0;
    robot_network_status(false, false);
}
static void incoming_topic(void *arg, const char *topic, u32_t length) {
    (void)arg;
    network.rx_valid = length == 1 && strcmp(topic, "robot/command") == 0;
    network.rx_unsafe_start = (client->rx_buffer[0] & 0x09u) != 0;
}
static void incoming_data(void *arg, const u8_t *data, u16_t length, u8_t flags) {
    (void)arg;
    if (!network.rx_valid || length != 1 || !(flags & MQTT_DATA_FLAG_LAST)) {
        network.rx_valid = 0;
        return;
    }
    network.rx_valid = 0;
    command_t command;
    uint8_t sequence;
    if (!command_decode(data, length, &command, &sequence))
        return;
    unsigned delta = (sequence - network.last_sequence) & 63u;
    bool fresh = !network.seen_sequence || (delta > 0 && delta < 32);
    bool accepted = false;
    if (command == CMD_STOP || command == CMD_ESTOP ||
        (fresh && !(command == CMD_START && network.rx_unsafe_start))) {
        accepted = robot_command(command, sequence);
        if (accepted) {
            network.last_sequence = sequence;
            network.seen_sequence = 1;
        }
    }
    /* START/RESET get a later ACK from Navigation after checking vehicle state. */
    if (!accepted || command == CMD_STOP || command == CMD_ESTOP) {
        uint8_t ack = (uint8_t)(sequence | (accepted ? 64u : 0u));
        (void)tk_snd_mbf(q_ack, &ack, 1, TMO_POL);
    }
}
static void subscribed(void *arg, err_t result) {
    (void)arg;
    if (result == ERR_OK)
        network.subscribed = 1;
    else
        retry();
}
static void connection(mqtt_client_t *connection_client, void *arg,
                       mqtt_connection_status_t status) {
    (void)arg;
    if (status != MQTT_CONNECT_ACCEPTED) {
        retry();
        return;
    }
    network.state = NET_ONLINE;
    network.entered_at = robot_now();
    network.backoff_seconds = 1;
    mqtt_set_inpub_callback(connection_client, incoming_topic, incoming_data, NULL);
    if (mqtt_subscribe(connection_client, "robot/command", 1, subscribed, NULL) != ERR_OK)
        retry();
}
static void dns_result(const char *name, const ip_addr_t *address, void *arg) {
    (void)name;
    (void)arg;
    network.dns_busy = 0;
    if (network.state != NET_MQTT)
        return;
    if (address) {
        broker = *address;
        network.dns_ready = 1;
    } else
        retry();
}
void network_prepare(void) {
    /* Radio power-up, firmware upload and station setup include SDK sleeps.
     * Complete them before the robot's deadlines and 30 ms watchdog start. */
    memset(&network, 0, sizeof(network));
    network.backoff_seconds = 1;
    if (!ROBOT_WIFI_SSID[0])
        return;
    if (cyw43_arch_init() != 0)
        return;
    network.initialized = 1;
    cyw43_arch_enable_sta_mode();
}
void network_init(void) {
    /* Keep the driver prepared by main; shared robot state now exists. */
    robot_network_status(false, false);
}
static void publish_snapshot(void) {
    robot_state_t state;
    robot_snapshot(&state);
    telemetry_msg_t payload = {{0}};
    uint32_t left = 250, right = 250, distance = 0;
    if (state.motion_valid) {
        bits_get(state.motion.bytes, 8, 0, 9, &left);
        bits_get(state.motion.bytes, 8, 9, 9, &right);
        bits_get(state.motion.bytes, 8, 18, 24, &distance);
    }
    telemetry_set(&payload, T_VERSION, 1);
    telemetry_set(&payload, T_VEHICLE, state.control.vehicle);
    telemetry_set(&payload, T_FAULTS, state.control.faults);
    telemetry_set(&payload, T_LEFT_SPEED, (int)left - 250);
    telemetry_set(&payload, T_RIGHT_SPEED, (int)right - 250);
    telemetry_set(&payload, T_LEFT_ENCODER, state.encoder_left);
    telemetry_set(&payload, T_RIGHT_ENCODER, state.encoder_right);
    telemetry_set(&payload, T_DISTANCE, distance);
    telemetry_set(&payload, T_IR_LEFT, state.adc_left);
    telemetry_set(&payload, T_IR_CENTER, state.adc_center);
    telemetry_set(&payload, T_IR_RIGHT, state.adc_right);
    telemetry_set(&payload, T_BARCODE, state.barcode);
    telemetry_set(&payload, T_HUMP, state.hump_peak);
    telemetry_set(&payload, T_RANGE, state.control.range_mm);
    telemetry_set(&payload, T_WIFI, state.wifi);
    telemetry_set(&payload, T_MQTT, state.mqtt);
    telemetry_set(&payload, T_SEQUENCE, network.sequence++);
    /* QoS0 snapshot drops on ring-buffer pressure instead of blocking control. */
    (void)mqtt_publish(client, "robot/telemetry", payload.bytes, sizeof(payload), 0, 0, NULL, NULL);
    if (elapsed_ms(robot_now(), network.status_at) >= 1000) {
        network.status_at = robot_now();
        (void)mqtt_publish(client, "robot/status", payload.bytes, sizeof(payload), 0, 0, NULL,
                           NULL);
    }
}
void network_step(void) {
    if (!ROBOT_WIFI_SSID[0])
        return; /* Local/autonomous operation does not require WiFi. */
    if (!network.initialized) {
        /* A failed hardware initialization must not block the running safety
         * tasks. A reboot retries preparation; association/MQTT retry below. */
        return;
    }
    uint16_t now = robot_now();
    cyw43_arch_poll();
    switch ((network_state_t)network.state) {
    case NET_INIT:
        if (!client)
            client = mqtt_client_new();
        if (!client) {
            retry();
            return;
        }
        if (mqtt_client_is_connected(client))
            mqtt_disconnect(client);
        network.dns_ready = 0;
        if (cyw43_arch_wifi_connect_async(ROBOT_WIFI_SSID, ROBOT_WIFI_PASSWORD,
                                          CYW43_AUTH_WPA2_AES_PSK) != 0) {
            retry();
            return;
        }
        network.state = NET_WIFI;
        network.entered_at = now;
        break;
    case NET_WIFI: {
        int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        if (status == CYW43_LINK_UP) {
            network.state = NET_MQTT;
            network.entered_at = now;
        } else if (status < 0 || elapsed_ms(now, network.entered_at) > 15000)
            retry();
        break;
    }
    case NET_MQTT:
        if (elapsed_ms(now, network.entered_at) > 10000) {
            mqtt_disconnect(client);
            retry();
            break;
        }
        if (!network.dns_ready && !network.dns_busy && !network.connecting) {
            err_t result = dns_gethostbyname(ROBOT_MQTT_HOST, &broker, dns_result, NULL);
            if (result == ERR_OK)
                network.dns_ready = 1;
            else if (result == ERR_INPROGRESS)
                network.dns_busy = 1;
            else
                retry();
        }
        if (network.dns_ready) {
            static const struct mqtt_connect_client_info_t info = {.client_id = "CSC20-Motor-Wheel",
                                                                   .client_user = ROBOT_MQTT_USER,
                                                                   .client_pass =
                                                                       ROBOT_MQTT_PASSWORD,
                                                                   .keep_alive = 15};
            network.dns_ready = 0;
            network.connecting = 1;
            err_t result =
                mqtt_client_connect(client, &broker, ROBOT_MQTT_PORT, connection, NULL, &info);
            if (result != ERR_OK) {
                network.connecting = 0;
                retry();
            }
        }
        break;
    case NET_ONLINE:
        network.connecting = 0;
        if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP ||
            !mqtt_client_is_connected(client)) {
            mqtt_disconnect(client);
            retry();
            break;
        }
        robot_network_status(true, true);
        if (!network.subscribed && elapsed_ms(now, network.entered_at) > 10000) {
            mqtt_disconnect(client);
            retry();
            break;
        }
        for (unsigned i = 0; i < 4; ++i) {
            uint8_t ack;
            if (tk_rcv_mbf(q_ack, &ack, TMO_POL) != 1)
                break;
            (void)mqtt_publish(client, "robot/ack", &ack, 1, 1, 0, NULL, NULL);
        }
        if (elapsed_ms(now, network.published_at) >= TELEMETRY_PERIOD_MS) {
            network.published_at = now;
            publish_snapshot();
        }
        break;
    case NET_RETRY:
        if (elapsed_ms(now, network.entered_at) >= network.backoff_seconds * 1000u) {
            network.state = NET_INIT;
        }
        break;
    }
}
