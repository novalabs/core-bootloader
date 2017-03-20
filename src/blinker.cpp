/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <nil.h>
#include <hal.h>

#include <core/bootloader/blinker.hpp>

static const uint8_t led_default[] = {
    LED_ON(10), LED_OFF(950), LED_LOOP()
};

static const uint8_t* _pattern = led_default;

static bool _active = true;

void
blinkerForce(
    bool on
)
{
    if (on) {
        palSetPad(LED_GPIO, LED_PIN);
    } else {
        palClearPad(LED_GPIO, LED_PIN);
    }
}

void
blinkerSetActive(
    bool active
)
{
    _active = active;
}

void
blinkerSetPattern(
    const uint8_t* pattern
)
{
    _pattern = pattern;
}

THD_WORKING_AREA(blinkerThreadWorkingArea, 128);
THD_FUNCTION(blinkerThread, arg) {
    (void)arg;
    static const uint8_t* oldPattern = 0;
    static uint8_t        id = 0;

    while (1) {
        if (_pattern != oldPattern) {
            oldPattern = _pattern;
            id = 0;
        }

        uint8_t x = oldPattern[id];

        if (x == 0) {
            id = 0;
        } else {
            if (x & 0x80) {
                if (_active) {
                    palSetPad(LED_GPIO, LED_PIN);
                }

                chThdSleepMilliseconds(BLINKER_GRANULARITY * (x & 0x7F));
            } else {
                if (_active) {
                    palClearPad(LED_GPIO, LED_PIN);
                }

                chThdSleepMilliseconds(BLINKER_GRANULARITY * (x & 0x7F));
            }

            id++;
        }
    }
}
