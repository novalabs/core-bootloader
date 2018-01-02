/* COPYRIGHT (c) 2016-2018 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <algorithm>

/*******************************/
/* A quick & dirty led blinker */
/*******************************/

/* Sequences are specified as array of uint8_t. e.g.: */
/* static const uint8_t led_default[] = { LED_ON(10), LED_OFF(950), LED_ON(100), LED_OFF(50), LED_LOOP() }; */

static const uint32_t BLINKER_GRANULARITY = 10; // [ms]

template <typename T, T min, T max>
constexpr T
saturate(
    T x
)
{
    return std::min(std::max(x, min), max);
}

// BLINKER_GRANULARITY ms minimum, 127 * BLINKER_GRANULARITY ms maximum
constexpr uint8_t
LED_ON(
    uint32_t ms
)
{
    return 0x80 | saturate<uint8_t, 0x01, 0x7F>(ms / BLINKER_GRANULARITY);
}

constexpr uint8_t
LED_OFF(
    uint32_t ms
)
{
    return 0x00 | saturate<uint8_t, 0x01, 0x7F>(ms / BLINKER_GRANULARITY);
}

constexpr uint8_t
LED_LOOP()
{
    return 0x00;
}

extern THD_WORKING_AREA(blinkerThreadWorkingArea, 128);
THD_FUNCTION(blinkerThread, arg);

// Changes the sequence
void
blinkerSetPattern(
    const uint8_t* pattern
);

void
blinkerSetActive(
    bool active
);

void
blinkerForce(
    bool on
);
