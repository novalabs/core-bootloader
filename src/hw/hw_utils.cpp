/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <core/bootloader/hw/hw_utils.hpp>

#include <nil.h>
#include <hal.h>

#if defined(STM32F091xC)
uint32_t VectorTable[48] __attribute__((section(".vector_table")));
#endif

namespace hw {
constexpr uint8_t*
CPU_UID()
{
    return reinterpret_cast<uint8_t*>(0x1FFFF7AC);
}

//uint8_t UID[12] = {CPU_UID()[0], CPU_UID()[1], CPU_UID()[2], CPU_UID()[3], CPU_UID()[4], CPU_UID()[5], CPU_UID()[6], CPU_UID()[7], CPU_UID()[8], CPU_UID()[9], CPU_UID()[10], CPU_UID()[11] };

//uint8_t* UID = CPU_UID();

static const UID __attribute__((aligned(4))) _UID({CPU_UID()[0], CPU_UID()[1], CPU_UID()[2], CPU_UID()[3], CPU_UID()[4], CPU_UID()[5], CPU_UID()[6], CPU_UID()[7], CPU_UID()[8], CPU_UID()[9], CPU_UID()[10], CPU_UID()[11]});

const UID&
getUID()
{
    return _UID;
}

ResetSource
getResetSource()
{
    static const uint32_t CSR = RCC->CSR; // save it for later...

    if ((CSR & RCC_CSR_IWDGRSTF) == RCC_CSR_IWDGRSTF) {
        RCC->CSR |= RCC_CSR_RMVF;
        return ResetSource::WATCHDOG;
    } else if ((CSR & RCC_CSR_SFTRSTF) == RCC_CSR_SFTRSTF) {
        RCC->CSR |= RCC_CSR_RMVF;
        return ResetSource::SOFTWARE;
    } else {
        RCC->CSR |= RCC_CSR_RMVF;
        return ResetSource::HARDWARE;
    }
}

void
reset()
{
    chThdSleepMilliseconds(100);

    NVIC_SystemReset();

    while (1) {}
} // reset

uint32_t
getNVR()
{
    return RTC->BKP0R;
}

void
setNVR(
    uint32_t value
)
{
    RTC->BKP0R = value;
}

void
watchdogEnable(
    watchdogPeriod period
)
{
#ifndef OVERRIDE_WATCHDOG
    switch (period) {
      case watchdogPeriod::PERIOD_0_MS:
          IWDG->KR  = 0x5555;   // enable access
          IWDG->PR  = 1;    // /8
          IWDG->RLR = 0x1;   // maximum (circa 800 ms)
          IWDG->KR  = 0xCCCC;   // start watchdog
          break;
      case watchdogPeriod::PERIOD_800_MS:
          IWDG->KR  = 0x5555; // enable access
          IWDG->PR  = 1;  // /8
          IWDG->RLR = 0xFFF; // maximum (circa 800 ms)
          IWDG->KR  = 0xCCCC; // start watchdog
          break;
      case watchdogPeriod::PERIOD_1600_MS:
          IWDG->KR  = 0x5555; // enable access
          IWDG->PR  = 2;  // /16
          IWDG->RLR = 0xFFF; // maximum (circa 1600 ms)
          IWDG->KR  = 0xCCCC; // start watchdog
          break;
      default:
          IWDG->KR  = 0x5555; // enable access
          IWDG->PR  = 4;  // /64
          IWDG->RLR = 0xFFF; // maximum (circa 6400 ms)
          IWDG->KR  = 0xCCCC; // start watchdog
          break;
    } // switch
#endif // ifndef OVERRIDE_WATCHDOG
} // watchdogEnable

void
watchdogReload()
{
#ifndef OVERRIDE_WATCHDOG
    IWDG->KR = 0xAAAA; // reload
#endif
}

int32_t
jumptoapp(
    uint32_t addr
)
{
#if defined(STM32F091xC)
    // Copy vector table from image to RAM
    for (std::size_t i = 0; i < 48; i++) {
        VectorTable[i] = *(uint32_t*)(addr + (i << 2));
    }

    // Enable the SYSCFG peripheral clock
    rccEnableAPB2(RCC_APB2ENR_SYSCFGCOMPEN, FALSE);
    // Remap SRAM at 0x00000000
    SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_MEM_MODE;
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE;
#endif

    pFunction JumpToApp;
    uint32_t  JumpAddress;

    // The second entry of the vector table contains the reset_handler function
    JumpAddress = *(uint32_t*)(addr + 4);

    // Assign the function pointer
    JumpToApp = (pFunction)JumpAddress;

    // Initialize user application's Stack Pointer
    __set_MSP(*(uint32_t*)addr);

    chSysDisable();

    // Jump!
    JumpToApp();

    return 0;
} // jumptoapp
}
