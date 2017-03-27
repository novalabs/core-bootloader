/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <stdint.h>
#include <core/Array.hpp>

namespace hw {
typedef enum {
    HARDWARE, WATCHDOG, SOFTWARE, OTHER
} ResetSource;

ResetSource
getResetSource();

void
reset();


using UID = Array<uint8_t, 12>;

const UID&
getUID();

uint32_t
getNVR();

void
setNVR(
    uint32_t value
);


enum WatchdogReason : uint32_t {
    NO_APPLICATION   = 0xCAFEBABE,
    USER_REQUEST     = 0xB0BAFE77,
    BOOT_APPLICATION = 0xBAADF00D,
    TRANSPORT_FAIL   = 0xACABACAB
};


enum watchdogPeriod {
    PERIOD_800_MS,
    PERIOD_1600_MS,
    PERIOD_6400_MS
};

void
watchdogEnable(
    watchdogPeriod period
);

void
watchdogReload();


typedef void (* pFunction)(
    void
);

int32_t
jumptoapp(
    uint32_t addr
);
}
