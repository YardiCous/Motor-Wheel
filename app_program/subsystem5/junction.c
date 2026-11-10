/** @file junction.c
 * @brief Barcode letter to junction action table, used once per junction.
 */
#include "junction.h"

#define JUNCTION_LETTERS 26u

/* Edit this table to give each letter its action at the next junction:
 * JUNCTION_FORWARD, JUNCTION_LEFT, JUNCTION_RIGHT or JUNCTION_UTURN. */
static const uint8_t g_actions[JUNCTION_LETTERS] = {
    JUNCTION_FORWARD, /* A */
    JUNCTION_FORWARD, /* B */
    JUNCTION_FORWARD, /* C */
    JUNCTION_FORWARD, /* D */
    JUNCTION_FORWARD, /* E */
    JUNCTION_UTURN,   /* F */
    JUNCTION_FORWARD, /* G */
    JUNCTION_FORWARD, /* H */
    JUNCTION_FORWARD, /* I */
    JUNCTION_FORWARD, /* J */
    JUNCTION_FORWARD, /* K */
    JUNCTION_FORWARD, /* L */
    JUNCTION_FORWARD, /* M */
    JUNCTION_FORWARD, /* N */
    JUNCTION_FORWARD, /* O */
    JUNCTION_FORWARD, /* P */
    JUNCTION_FORWARD, /* Q */
    JUNCTION_FORWARD, /* R */
    JUNCTION_FORWARD, /* S */
    JUNCTION_FORWARD, /* T */
    JUNCTION_FORWARD, /* U */
    JUNCTION_FORWARD, /* V */
    JUNCTION_FORWARD, /* W */
    JUNCTION_FORWARD, /* X */
    JUNCTION_FORWARD, /* Y */
    JUNCTION_RIGHT,   /* Z */
};

static bool is_letter(char letter);

/** @brief '$' means no barcode has been read since the last junction. */
void junction_init(junction_t * p_junction)
{
    p_junction->pending = JUNCTION_NONE;
}

/** @brief A newer barcode replaces an unused older one. */
bool junction_store(junction_t * p_junction, char letter)
{
    bool b_stored = is_letter(letter);

    if (b_stored)
    {
        p_junction->pending = letter;
    }

    return b_stored;
}

/** @brief '$' carries straight on; a letter is used once. */
uint8_t junction_take(junction_t * p_junction)
{
    uint8_t action      = junction_action(p_junction->pending);
    p_junction->pending = JUNCTION_NONE;
    return action;
}

/** @brief Unknown or '$' is FORWARD, the same as having no barcode. */
uint8_t junction_action(char letter)
{
    uint8_t action = JUNCTION_FORWARD;

    if (is_letter(letter))
    {
        action = g_actions[(uint8_t)(letter - 'A')];
    }

    return action;
}

/** @brief Console text for the action. */
const char * junction_name(uint8_t action)
{
    const char * p_name = "IGNORED";

    switch (action)
    {
        case JUNCTION_FORWARD:
            p_name = "FORWARD";
            break;

        case JUNCTION_LEFT:
            p_name = "LEFT";
            break;

        case JUNCTION_RIGHT:
            p_name = "RIGHT";
            break;

        case JUNCTION_UTURN:
            p_name = "U-TURN";
            break;

        default:
            /* JUNCTION_IGNORED keeps the default text. */
            break;
    }

    return p_name;
}

/** @brief Capital A..Z only. */
static bool is_letter(char letter)
{
    return (letter >= 'A') && (letter <= 'Z');
}

/*** End of file. ***/

