/** @file junction.h
 * @brief What to do at the next junction, decided by the last barcode.
 *
 * A barcode letter is remembered until the line sensors reach a junction
 * (left and centre both black). There the letter's action from the A..Z
 * table is used once and the memory goes back to JUNCTION_NONE ('$'). With
 * no letter remembered the robot carries straight on.
 */
#ifndef JUNCTION_H
#define JUNCTION_H

#include <stdbool.h>
#include <stdint.h>

/** @brief "No barcode since the last junction". */
#define JUNCTION_NONE '$'
/** @brief Queue byte meaning "the line sensors have reached a junction". */
#define JUNCTION_EVENT '#'

#define JUNCTION_FORWARD 0u
#define JUNCTION_LEFT 1u
#define JUNCTION_RIGHT 2u
#define JUNCTION_UTURN 3u
/** @brief navigation_junction() result when the robot was not following. */
#define JUNCTION_IGNORED 4u

/** @brief The remembered letter, or JUNCTION_NONE. */
typedef struct
{
    char pending;
} junction_t;

/** @brief Start with nothing remembered ('$'). */
void junction_init(junction_t * p_junction);
/** @brief Remember an A..Z letter; anything else is refused. */
bool junction_store(junction_t * p_junction, char letter);
/** @brief The action for the remembered letter; memory goes back to '$'. */
uint8_t junction_take(junction_t * p_junction);
/** @brief Table lookup: A..Z gives its action, anything else FORWARD. */
uint8_t junction_action(char letter);
/** @brief "FORWARD", "LEFT", "RIGHT", "U-TURN" or "IGNORED". */
const char * junction_name(uint8_t action);

#endif /* JUNCTION_H */

/*** End of file. ***/

