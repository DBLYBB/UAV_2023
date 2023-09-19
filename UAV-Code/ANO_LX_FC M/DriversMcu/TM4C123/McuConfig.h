#ifndef _MCUCONFIG_H_
#define _MCUCONFIG_H_
#include "TM4C123G.h"

typedef	uint32_t 	u32;
typedef	uint16_t 	u16;
typedef	uint8_t 	u8;
typedef	int32_t 	s32;
typedef	int16_t 	s16;
typedef	int8_t 		s8;

#include "rom.h"
#include "rom_map.h"
#include "sysctl.h"
#include "gpio.h"
#include "pin_map.h"

/*中断号越小优先级越高*/
#define  USER_INT0  0x00
#define  USER_INT1  0x20
#define  USER_INT2  0x40
#define  USER_INT3  0x60
#define  USER_INT4  0x80
#define  USER_INT5  0xA0
#define  USER_INT6  0xD0
#define  USER_INT7  0xE0
/*================IO配置======================*/
#define LED1_SYSCTL			SYSCTL_PERIPH_GPIOD
#define LED2_SYSCTL			SYSCTL_PERIPH_GPIOD
#define LED3_SYSCTL			SYSCTL_PERIPH_GPIOA
#define LEDS_SYSCTL			SYSCTL_PERIPH_GPIOF
#define LED1_PORT			GPIOD_BASE
#define LED2_PORT			GPIOD_BASE
#define LED3_PORT			GPIOA_BASE
#define LEDS_PORT			GPIOF_BASE
#define LED1_PIN			GPIO_PIN_0
#define LED2_PIN			GPIO_PIN_1
#define LED3_PIN			GPIO_PIN_6
#define LEDS_PIN			GPIO_PIN_4
///**********PWM输出引脚配置*************/
//#define M0TO_PWM1_FUNCTION	GPIO_PB6_M0PWM0
//#define M0TO_PWM2_FUNCTION	GPIO_PB7_M0PWM1
//#define M0TO_PWM3_FUNCTION	GPIO_PB4_M0PWM2
//#define M0TO_PWM4_FUNCTION	GPIO_PB5_M0PWM3
/*油门行程为[0:1000]*/
//#define MINTHROTTLE	    	0				//最小油门值           
//#define MAXTHROTTLE 		1000            //最大油门值
//#define PWM_PERIOD_MAX		3125			//周期为2.5ms(400hz)
/**********spi引脚配置*************/
/*传感器用*/
#define SPI0_SYSCTL			SYSCTL_PERIPH_GPIOA
#define SPI0_PROT			GPIOA_BASE
#define SPI0_CLK_PIN		GPIO_PIN_2
#define SPI0_RX_PIN			GPIO_PIN_4
#define SPI0_TX_PIN			GPIO_PIN_5
#define SPI0_CLK			GPIO_PA2_SSI0CLK
#define SPI0_RX				GPIO_PA4_SSI0RX
#define SPI0_TX				GPIO_PA5_SSI0TX
#define SPI0_BASE			SSI0_BASE
/**********BMI088的引脚配置*************/
#define BMI088ACC_CSPIN_SYSCTL	SYSCTL_PERIPH_GPIOD
#define BMI088ACC_CS_PORT		GPIOD_BASE
#define BMI088ACC_CS_PIN		GPIO_PIN_2
#define BMI088GYR_CSPIN_SYSCTL	SYSCTL_PERIPH_GPIOC
#define BMI088GYR_CS_PORT		GPIOC_BASE
#define BMI088GYR_CS_PIN		GPIO_PIN_3
#define BMI088_READYPIN_SYSCTL	SYSCTL_PERIPH_GPIOB
#define BMI088_READY_PORT	    GPIOB_BASE
#define BMI088_READY_INT_PORT 	INT_GPIOB
#define BMI088_READY_PIN	    GPIO_PIN_2
/**********AK8975的CS使能引脚配置*************/
#define AK_CSPIN_SYSCTL		SYSCTL_PERIPH_GPIOD
#define AK8975_CS_PORT		GPIOD_BASE
#define AK8975_CS_PIN		GPIO_PIN_3
/**********spl06的CS使能引脚配置*************/
#define SPL_CSPIN_SYSCTL	SYSCTL_PERIPH_GPIOC
#define SPL06_CS_PORT		GPIOC_BASE
#define SPL06_CS_PIN		GPIO_PIN_2
/**********Flash的CS使能引脚配置*************/
#define FLASH_CSPIN_SYSCTL	SYSCTL_PERIPH_GPIOE
#define FLASH_CS_PORT		GPIOE_BASE
#define FLASH_CS_PIN		GPIO_PIN_2
/**********PWM输出引脚配置*************/
#define M0TO_PWM1_FUNCTION	GPIO_PB6_M0PWM0
#define M0TO_PWM2_FUNCTION	GPIO_PB7_M0PWM1
#define M0TO_PWM3_FUNCTION	GPIO_PB4_M0PWM2
#define M0TO_PWM4_FUNCTION	GPIO_PB5_M0PWM3
#define M0TO_PWM5_FUNCTION	GPIO_PF0_M1PWM4
#define M0TO_PWM6_FUNCTION	GPIO_PF1_M1PWM5
#define M0TO_PWM7_FUNCTION	GPIO_PF2_M1PWM6
#define M0TO_PWM8_FUNCTION	GPIO_PF3_M1PWM7
#define HEAT_PWM_FUNCTION	GPIO_PA7_M1PWM3
/*油门行程为[0:1000]*/
#define MINTHROTTLE	    	0				//最小油门值           
#define MAXTHROTTLE 		1000            //最大油门值
#define PWM_PERIOD_MAX		3125			//周期为2.5ms(400hz)
///**********Uart引脚配置*************/
//#define	UART0_RX			GPIO_PA0_U0RX
//#define	UART0_TX			GPIO_PA1_U0TX
//#define	UART0_PORT			GPIOA_BASE
//#define	UART0_PIN_RX		GPIO_PIN_0
//#define	UART0_PIN_TX		GPIO_PIN_1
/**********PPM定时器配置*************/
#define PPM_SYSCTL			SYSCTL_PERIPH_GPIOC
#define PPM_FUNCTION		GPIO_PC6_WT1CCP0
#define PPM_PORTS			GPIOC_BASE
#define PPM_PIN				GPIO_PIN_6
#define PULSE_MIN   		800
#define PULSE_MAX   		2200
/**********ADC引脚配置*************/
#define ADC_PORT			GPIOE_BASE
#define ADC_PIN				GPIO_PIN_3
/**********串口引脚配置*************/
#define SBUS_SYSCTL			SYSCTL_PERIPH_GPIOC
#define SBUS_UART			UART3_BASE
#define SBUS_BAUDRATE		100000
#define	UART3_RX			GPIO_PC6_U3RX
#define	UART3_PORT			GPIOC_BASE
#define	UART3_PIN_RX		GPIO_PIN_6 

#define	UART0_RX			GPIO_PA0_U0RX
#define	UART0_TX			GPIO_PA1_U0TX
#define	UART0_PORT			GPIOA_BASE
#define	UART0_PIN_RX		GPIO_PIN_0
#define	UART0_PIN_TX		GPIO_PIN_1

#define	UART2_RX			GPIO_PD6_U2RX
#define	UART2_TX			GPIO_PD7_U2TX
#define	UART2_PORT			GPIOD_BASE
#define	UART2_PIN_RX		GPIO_PIN_6
#define	UART2_PIN_TX		GPIO_PIN_7

#define	UART4_RX			GPIO_PC4_U4RX
#define	UART4_TX			GPIO_PC5_U4TX
#define	UART4_PORT			GPIOC_BASE
#define	UART4_PIN_RX		GPIO_PIN_4
#define	UART4_PIN_TX		GPIO_PIN_5

#define	UART5_RX			GPIO_PE4_U5RX
#define	UART5_TX			GPIO_PE5_U5TX
#define	UART5_PORT			GPIOE_BASE
#define	UART5_PIN_RX		GPIO_PIN_4
#define	UART5_PIN_TX		GPIO_PIN_5

#define	UART7_RX			GPIO_PE0_U7RX
#define	UART7_TX			GPIO_PE1_U7TX
#define	UART7_PORT			GPIOE_BASE
#define	UART7_PIN_RX		GPIO_PIN_0
#define	UART7_PIN_TX		GPIO_PIN_1
//#define USE_HEAT


#endif
