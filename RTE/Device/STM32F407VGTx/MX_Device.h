/******************************************************************************
 * File Name   : MX_Device.h
 * Date        : 29/12/2022 22:32:53
 * Description : STM32Cube MX parameter definitions
 * Note        : This file is generated by STM32CubeMX (DO NOT EDIT!)
 ******************************************************************************/

#ifndef __MX_DEVICE_H
#define __MX_DEVICE_H

/*---------------------------- Clock Configuration ---------------------------*/

#define MX_LSI_VALUE                            32000
#define MX_LSE_VALUE                            32768
#define MX_HSI_VALUE                            16000000
#define MX_HSE_VALUE                            25000000
#define MX_EXTERNAL_CLOCK_VALUE                 12288000
#define MX_PLLCLKFreq_Value                     96000000
#define MX_SYSCLKFreq_VALUE                     16000000
#define MX_HCLKFreq_Value                       16000000
#define MX_FCLKCortexFreq_Value                 16000000
#define MX_CortexFreq_Value                     16000000
#define MX_AHBFreq_Value                        16000000
#define MX_APB1Freq_Value                       16000000
#define MX_APB2Freq_Value                       16000000
#define MX_APB1TimFreq_Value                    16000000
#define MX_APB2TimFreq_Value                    16000000
#define MX_48MHZClocksFreq_Value                48000000
#define MX_EthernetFreq_Value                   16000000
#define MX_I2SClocksFreq_Value                  96000000
#define MX_RTCFreq_Value                        32000
#define MX_WatchDogFreq_Value                   32000
#define MX_MCO1PinFreq_Value                    16000000
#define MX_MCO2PinFreq_Value                    16000000

/*-------------------------------- DMA        --------------------------------*/

#define MX_DMA                                  1

/* NVIC Configuration */

/* NVIC DMA1_Stream6_IRQn */
#define MX_DMA1_Stream6_IRQn_interruptPremptionPriority 0
#define MX_DMA1_Stream6_IRQn_PriorityGroup      NVIC_PRIORITYGROUP_4
#define MX_DMA1_Stream6_IRQn_Subriority         0

/* NVIC DMA1_Stream0_IRQn */
#define MX_DMA1_Stream0_IRQn_interruptPremptionPriority 0
#define MX_DMA1_Stream0_IRQn_PriorityGroup      NVIC_PRIORITYGROUP_4
#define MX_DMA1_Stream0_IRQn_Subriority         0

/*-------------------------------- I2C1       --------------------------------*/

#define MX_I2C1                                 1

/* GPIO Configuration */

/* Pin PB6 */
#define MX_I2C1_SCL_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_I2C1_SCL_Pin                         PB6
#define MX_I2C1_SCL_GPIOx                       GPIOB
#define MX_I2C1_SCL_GPIO_Pin                    GPIO_PIN_6
#define MX_I2C1_SCL_GPIO_AF                     GPIO_AF4_I2C1
#define MX_I2C1_SCL_GPIO_Pu                     GPIO_NOPULL
#define MX_I2C1_SCL_GPIO_Mode                   GPIO_MODE_AF_OD

/* Pin PB7 */
#define MX_I2C1_SDA_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_I2C1_SDA_Pin                         PB7
#define MX_I2C1_SDA_GPIOx                       GPIOB
#define MX_I2C1_SDA_GPIO_Pin                    GPIO_PIN_7
#define MX_I2C1_SDA_GPIO_AF                     GPIO_AF4_I2C1
#define MX_I2C1_SDA_GPIO_Pu                     GPIO_NOPULL
#define MX_I2C1_SDA_GPIO_Mode                   GPIO_MODE_AF_OD

/* DMA Configuration */

/* DMA I2C1_TX */
#define MX_I2C1_TX_DMA_DMA_Handle
#define MX_I2C1_TX_DMA_Instance                 DMA1_Stream6
#define MX_I2C1_TX_DMA_FIFOMode                 DMA_FIFOMODE_DISABLE
#define MX_I2C1_TX_DMA_Priority                 DMA_PRIORITY_LOW
#define MX_I2C1_TX_DMA_Channel                  DMA_CHANNEL_1
#define MX_I2C1_TX_DMA_PeriphDataAlignment      DMA_PDATAALIGN_BYTE
#define MX_I2C1_TX_DMA_MemDataAlignment         DMA_MDATAALIGN_BYTE
#define MX_I2C1_TX_DMA_Mode                     DMA_NORMAL
#define MX_I2C1_TX_DMA_Direction                DMA_MEMORY_TO_PERIPH
#define MX_I2C1_TX_DMA_PeriphInc                DMA_PINC_DISABLE
#define MX_I2C1_TX_DMA_MemInc                   DMA_MINC_ENABLE
#define MX_I2C1_TX_DMA_IpInstance

/* DMA I2C1_RX */
#define MX_I2C1_RX_DMA_DMA_Handle
#define MX_I2C1_RX_DMA_Instance                 DMA1_Stream0
#define MX_I2C1_RX_DMA_FIFOMode                 DMA_FIFOMODE_DISABLE
#define MX_I2C1_RX_DMA_Priority                 DMA_PRIORITY_LOW
#define MX_I2C1_RX_DMA_Channel                  DMA_CHANNEL_1
#define MX_I2C1_RX_DMA_PeriphDataAlignment      DMA_PDATAALIGN_BYTE
#define MX_I2C1_RX_DMA_MemDataAlignment         DMA_MDATAALIGN_BYTE
#define MX_I2C1_RX_DMA_Mode                     DMA_NORMAL
#define MX_I2C1_RX_DMA_Direction                DMA_PERIPH_TO_MEMORY
#define MX_I2C1_RX_DMA_PeriphInc                DMA_PINC_DISABLE
#define MX_I2C1_RX_DMA_MemInc                   DMA_MINC_ENABLE
#define MX_I2C1_RX_DMA_IpInstance

/* NVIC Configuration */

/* NVIC I2C1_EV_IRQn */
#define MX_I2C1_EV_IRQn_interruptPremptionPriority 0
#define MX_I2C1_EV_IRQn_PriorityGroup           NVIC_PRIORITYGROUP_4
#define MX_I2C1_EV_IRQn_Subriority              0

/* NVIC I2C1_ER_IRQn */
#define MX_I2C1_ER_IRQn_interruptPremptionPriority 0
#define MX_I2C1_ER_IRQn_PriorityGroup           NVIC_PRIORITYGROUP_4
#define MX_I2C1_ER_IRQn_Subriority              0

/*-------------------------------- SYS        --------------------------------*/

#define MX_SYS                                  1

/* GPIO Configuration */

/*-------------------------------- USART2     --------------------------------*/

#define MX_USART2                               1

#define MX_USART2_VM                            VM_ASYNC

/* GPIO Configuration */

/* Pin PA2 */
#define MX_USART2_TX_GPIO_ModeDefaultPP         GPIO_MODE_AF_PP
#define MX_USART2_TX_GPIO_Speed                 GPIO_SPEED_FREQ_VERY_HIGH
#define MX_USART2_TX_Pin                        PA2
#define MX_USART2_TX_GPIOx                      GPIOA
#define MX_USART2_TX_GPIO_PuPd                  GPIO_NOPULL
#define MX_USART2_TX_GPIO_Pin                   GPIO_PIN_2
#define MX_USART2_TX_GPIO_AF                    GPIO_AF7_USART2

/* Pin PA3 */
#define MX_USART2_RX_GPIO_ModeDefaultPP         GPIO_MODE_AF_PP
#define MX_USART2_RX_GPIO_Speed                 GPIO_SPEED_FREQ_VERY_HIGH
#define MX_USART2_RX_Pin                        PA3
#define MX_USART2_RX_GPIOx                      GPIOA
#define MX_USART2_RX_GPIO_PuPd                  GPIO_NOPULL
#define MX_USART2_RX_GPIO_Pin                   GPIO_PIN_3
#define MX_USART2_RX_GPIO_AF                    GPIO_AF7_USART2

/* NVIC Configuration */

/* NVIC USART2_IRQn */
#define MX_USART2_IRQn_interruptPremptionPriority 0
#define MX_USART2_IRQn_PriorityGroup            NVIC_PRIORITYGROUP_4
#define MX_USART2_IRQn_Subriority               0

/*-------------------------------- NVIC       --------------------------------*/

#define MX_NVIC                                 1

/*-------------------------------- GPIO       --------------------------------*/

#define MX_GPIO                                 1

/* GPIO Configuration */

#endif  /* __MX_DEVICE_H */

