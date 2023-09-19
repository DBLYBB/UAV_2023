#include "Drv_Paramter.h"
#include "Ano_Parameter.h"

void DvrParamterInit(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0); 
	ROM_EEPROMInit();
}

void DvrParamterRead(void)
{
	ROM_EEPROMRead((uint32_t *)Ano_Parame.byte,0X00,2048);
}

void DvrParamterSave(void)
{
	ROM_EEPROMProgram((uint32_t *)Ano_Parame.byte,0X00,2048);
}
