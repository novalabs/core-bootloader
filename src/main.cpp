/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <nil.h>
#include <hal.h>

#include <core/bootloader/bootloader.hpp>
#include <core/bootloader/hw/hw_utils.hpp>
#include <core/stm32_flash/FlashSegment.hpp>
#include <core/bootloader/blinker.hpp>

#include <stm32f091xc.h>

THD_TABLE_BEGIN
THD_TABLE_ENTRY(
    bootloaderThreadWorkingArea,
    "Bootloader",
    bootloaderThread,
    NULL
)
THD_TABLE_ENTRY(blinkerThreadWorkingArea, "Blinker", blinkerThread, NULL)
THD_TABLE_END

int
main(
    void
)
{
    //#ifdef STOP_IWDG_ON_DEBUG
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
    //#endif

//	hw::watchdogEnable(hw::watchdogPeriod::PERIOD_800_MS);

    halInit();
    chSysInit();

    if (hw::getResetSource() == hw::ResetSource::WATCHDOG) {
        if ((hw::getNVR() != hw::WatchdogReason::NO_APPLICATION) && (hw::getNVR() != hw::WatchdogReason::USER_REQUEST) && (hw::getNVR() != hw::WatchdogReason::BOOT_APPLICATION)) {
            while (1) {
                // port_disable();
                //  palSetPad(LED_GPIO, LED_PIN);
                //  palClearPad(LED_GPIO, LED_PIN);

                //osalSysHalt("FAULT!!!");
                // WE HAVE A REAL WATCHDOG FAULT... DEAL WITH IT!!!
            }
        }
    }

    while (true) {
        // Leggi, fatti le pippe, fai qualunque cosa MA NON DORMIRE!
    }
} // main
