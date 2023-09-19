#include "Drv_Sys.h"

static uint64_t SysRunTimeMs = 0;

void SysTick_Init(void )
{
	MAP_SysTick_setPeriod(48000);
    MAP_SysTick_enableModule();
    MAP_SysTick_enableInterrupt();
}
void SysTick_Handler(void)
{
	SysRunTimeMs++;
}
uint32_t GetSysRunTimeMs(void)
{
	return SysRunTimeMs;
}
uint32_t GetSysRunTimeUs(void)
{
	return SysRunTimeMs*1000 + (SysTick->LOAD - SysTick->VAL) * 1000 / SysTick->LOAD;
}

void MyDelayUs ( uint32_t us )
{
    uint32_t now = GetSysRunTimeUs();
    while ( GetSysRunTimeUs() - now < us );
}
void MyDelayMs(u32 time)
{
	while ( time-- )
        MyDelayUs ( 1000 );
}

void DrvSysInit(void)
{
	/* Halting WDT and disabling master interrupts */
    MAP_WDT_A_holdTimer();
    MAP_Interrupt_disableMaster();
	/* Seed the pseudo random num generator */
    srand(TLV->RANDOM_NUM_1);
    /* Set the core voltage level to VCORE1 */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    /* Set 2 flash wait states for Flash bank 0 and 1*/
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
	/* Initializes Clock System */
	//MAP_CS_enableDCOExternalResistor();
	MAP_CS_disableDCOExternalResistor();
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
	/* Enabling the FPU for floating point operation */
    MAP_FPU_enableModule();
    MAP_FPU_enableLazyStacking();
	
	MAP_Interrupt_enableMaster();
	
	SysTick_Init();
}
