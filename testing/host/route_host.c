/** @file route_host.c
 * @brief Host test: GP28 scanner, letter memory and junction actions.
 * Build: gcc -std=c99 -Wall -Wextra -Werror testing/host/route_host.c
 *        app_program/subsystem3/scanner.c app_program/subsystem3/code39.c
 *        app_program/subsystem5/junction.c app_program/subsystem5/navigation.c
 *        app_program/subsystem5/obstacle.c app_program/subsystem2/motion.c
 *        app_program/data_formats.c -o route_host && ./route_host
 */
#include "route_host.h"

#include "../../app_program/data_formats.h"
#include "../../app_program/subsystem2/motion.h"
#include "../../app_program/subsystem3/scanner.h"
#include "../../app_program/subsystem5/junction.h"
#include "../../app_program/subsystem5/navigation.h"

#include <assert.h>
#include <stdio.h>

#define HOST_WHITE 2000u
#define HOST_BLACK 3500u
#define HOST_THRESHOLD 2700u
/* About 70 mm/s: 4 encoder edges every 5-ms sample. */
#define HOST_EDGES_STEP 4u
#define HOST_NARROW 80u
#define HOST_WIDE 240u

/* "*Z*": bit 8 is the first bar, set bits are wide; 0x094 '*', 0x0d0 'Z'. */
static const uint16_t g_text[3] = {0x094u, 0x0d0u, 0x094u};

static char drive(scanner_t * p_scanner, uint32_t * p_edges, bool b_black,
                  uint32_t width, bool b_moving);
static char drive_barcode(scanner_t * p_scanner, uint32_t * p_edges);

/** @brief Scanner first, then the letter memory and navigation. */
int main(void)
{
    scanner_t scanner = {0};
    uint32_t  edges   = 0u;
    char      letter  = '\0';
    scanner_init(&scanner);

    /* Parked: flicker to black and back starts nothing. */
    (void)drive(&scanner, &edges, false, 40u, false);
    assert('\0' == drive(&scanner, &edges, true, 40u, false));
    (void)drive(&scanner, &edges, false, 40u, false);
    assert(!scanner.b_scanning);

    /* Moving: a blip shorter than a millimetre is dropped. */
    (void)drive(&scanner, &edges, true, 4u, true);
    (void)drive(&scanner, &edges, false, 40u, true);
    assert(!scanner.b_scanning);

    /* A whole "*Z*" then white: Z once, at the end of the quiet zone. */
    (void)drive(&scanner, &edges, false, 400u, true);
    letter = drive_barcode(&scanner, &edges);
    assert('\0' == letter);
    letter = drive(&scanner, &edges, false, 1000u, true);
    assert('Z' == letter);
    assert(!scanner.b_scanning);

    /* A junction straight after the barcode: flushing still gives Z. */
    (void)drive(&scanner, &edges, false, 400u, true);
    (void)drive_barcode(&scanner, &edges);
    (void)drive(&scanner, &edges, false, 100u, true);
    assert(scanner.b_scanning);
    assert('Z' == scanner_flush(&scanner));
    assert('\0' == scanner_flush(&scanner));

    /* Junction memory: '$' and the default table both carry straight on. */
    junction_t memory = {0};
    junction_init(&memory);
    assert(JUNCTION_NONE == memory.pending);
    assert(!junction_store(&memory, '1'));
    assert(junction_store(&memory, 'K'));
    assert('K' == memory.pending);
    assert(JUNCTION_FORWARD == junction_take(&memory));
    assert(JUNCTION_NONE == memory.pending);

    for (char each = 'A'; each <= 'Z'; ++each)
    {
        assert(JUNCTION_FORWARD == junction_action(each));
    }

    /* Navigation: not following keeps the letter; following uses it once. */
    navigation_t manager = {0};
    navigation_init(&manager);
    assert(JUNCTION_NONE == manager.junction.pending);
    assert(navigation_letter(&manager, 'Q'));
    assert(JUNCTION_IGNORED == navigation_junction(&manager));
    assert('Q' == manager.junction.pending);
    assert(navigation_supervise(&manager, DATA_FORMATS_START, false));
    assert(JUNCTION_FORWARD == navigation_junction(&manager));
    assert(JUNCTION_NONE == manager.junction.pending);
    assert(DATA_FORMATS_FOLLOW == manager.state);
    assert(JUNCTION_FORWARD == navigation_junction(&manager));

    (void)puts("PASS: GP28 scanner (parked, blip, *Z*, junction flush), "
               "letter memory, junction actions");
    return 0;
}

/** @brief Hold one colour for this many edges; return any letter read. */
static char drive(scanner_t * p_scanner, uint32_t * p_edges, bool b_black,
                  uint32_t width, bool b_moving)
{
    char     letter = '\0';
    uint16_t raw    = b_black ? HOST_BLACK : HOST_WHITE;

    for (uint32_t moved = 0u; moved < width; moved += HOST_EDGES_STEP)
    {
        *p_edges += b_moving ? HOST_EDGES_STEP : 0u;
        char now = scanner_step(p_scanner, raw, HOST_THRESHOLD, *p_edges,
                                *p_edges, true);
        letter   = ('\0' != now) ? now : letter;
    }

    return letter;
}

/** @brief Three characters with one narrow gap between them. */
static char drive_barcode(scanner_t * p_scanner, uint32_t * p_edges)
{
    char letter = '\0';

    for (uint8_t index = 0u; index < 3u; ++index)
    {
        if (index > 0u)
        {
            (void)drive(p_scanner, p_edges, false, HOST_NARROW, true);
        }

        for (uint8_t element = 0u; element < 9u; ++element)
        {
            bool     b_wide = (0u != ((g_text[index] >> (8u - element)) & 1u));
            uint32_t width  = b_wide ? HOST_WIDE : HOST_NARROW;
            char     now =
                drive(p_scanner, p_edges, (0u == (element & 1u)), width, true);
            letter = ('\0' != now) ? now : letter;
        }
    }

    return letter;
}

/*** End of file. ***/

