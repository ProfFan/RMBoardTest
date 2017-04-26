#![allow(dead_code)]

/* FLASH(up to 2 MB) base address in the alias region                         */
pub const FLASH_BASE        :u32 = 0x08000000;
/* CCM(core coupled memory) data RAM(64 KB) base address in the alias region  */
pub const CCMDATARAM_BASE   :u32 = 0x10000000;
/* SRAM1(112 KB) base address in the alias region                              */
pub const SRAM1_BASE        :u32 = 0x20000000;
/* SRAM2(16 KB) base address in the alias region                              */
pub const SRAM2_BASE        :u32 = 0x2001C000; 
/* Peripheral base address in the alias region                                */
pub const PERIPH_BASE       :u32 = 0x40000000; 
/* Backup SRAM(4 KB) base address in the alias region                         */
pub const BKPSRAM_BASE      :u32 = 0x40024000; 
/* FMC registers base address                                                 */
pub const FMC_R_BASE        :u32 = 0xA0000000; 
/* SRAM1(112 KB) base address in the bit-band region                          */
pub const SRAM1_BB_BASE     :u32 = 0x22000000; 
/* SRAM2(16 KB) base address in the bit-band region                           */
pub const SRAM2_BB_BASE     :u32 = 0x22380000; 
/* Peripheral base address in the bit-band region                             */
pub const PERIPH_BB_BASE    :u32 = 0x42000000; 
/* Backup SRAM(4 KB) base address in the bit-band region                      */
pub const BKPSRAM_BB_BASE   :u32 = 0x42480000; 
/* FLASH end address                                                          */
pub const FLASH_END         :u32 = 0x081FFFFF; 
/* Base address of : (up to 528 Bytes) embedded FLASH OTP Area                */
pub const FLASH_OTP_BASE    :u32 = 0x1FFF7800; 
/* End address of : (up to 528 Bytes) embedded FLASH OTP Area                 */
pub const FLASH_OTP_END     :u32 = 0x1FFF7A0F; 
/* CCM data RAM end address                                                   */
pub const CCMDATARAM_END    :u32 = 0x1000FFFF; 

/* Legacy defines */
pub const SRAM_BASE         :u32 = SRAM1_BASE;
pub const SRAM_BB_BASE      :u32 = SRAM1_BB_BASE;

/* Peripheral memory map */
pub const APB1PERIPH_BASE   :u32 = PERIPH_BASE;
pub const APB2PERIPH_BASE     :u32 = (PERIPH_BASE + 0x00010000);
pub const AHB1PERIPH_BASE     :u32 = (PERIPH_BASE + 0x00020000);
pub const AHB2PERIPH_BASE     :u32 = (PERIPH_BASE + 0x10000000);

/* APB1 peripherals */
pub const TIM2_BASE           :u32 = (APB1PERIPH_BASE + 0x0000);
pub const TIM3_BASE           :u32 = (APB1PERIPH_BASE + 0x0400);
pub const TIM4_BASE           :u32 = (APB1PERIPH_BASE + 0x0800);
pub const TIM5_BASE           :u32 = (APB1PERIPH_BASE + 0x0C00);
pub const TIM6_BASE           :u32 = (APB1PERIPH_BASE + 0x1000);
pub const TIM7_BASE           :u32 = (APB1PERIPH_BASE + 0x1400);
pub const TIM12_BASE          :u32 = (APB1PERIPH_BASE + 0x1800);
pub const TIM13_BASE          :u32 = (APB1PERIPH_BASE + 0x1C00);
pub const TIM14_BASE          :u32 = (APB1PERIPH_BASE + 0x2000);
pub const RTC_BASE            :u32 = (APB1PERIPH_BASE + 0x2800);
pub const WWDG_BASE           :u32 = (APB1PERIPH_BASE + 0x2C00);
pub const IWDG_BASE           :u32 = (APB1PERIPH_BASE + 0x3000);
pub const I2S2ext_BASE        :u32 = (APB1PERIPH_BASE + 0x3400);
pub const SPI2_BASE           :u32 = (APB1PERIPH_BASE + 0x3800);
pub const SPI3_BASE           :u32 = (APB1PERIPH_BASE + 0x3C00);
pub const I2S3ext_BASE        :u32 = (APB1PERIPH_BASE + 0x4000);
pub const USART2_BASE         :u32 = (APB1PERIPH_BASE + 0x4400);
pub const USART3_BASE         :u32 = (APB1PERIPH_BASE + 0x4800);
pub const UART4_BASE          :u32 = (APB1PERIPH_BASE + 0x4C00);
pub const UART5_BASE          :u32 = (APB1PERIPH_BASE + 0x5000);
pub const I2C1_BASE           :u32 = (APB1PERIPH_BASE + 0x5400);
pub const I2C2_BASE           :u32 = (APB1PERIPH_BASE + 0x5800);
pub const I2C3_BASE           :u32 = (APB1PERIPH_BASE + 0x5C00);
pub const CAN1_BASE           :u32 = (APB1PERIPH_BASE + 0x6400);
pub const CAN2_BASE           :u32 = (APB1PERIPH_BASE + 0x6800);
pub const PWR_BASE            :u32 = (APB1PERIPH_BASE + 0x7000);
pub const DAC_BASE            :u32 = (APB1PERIPH_BASE + 0x7400);
pub const UART7_BASE          :u32 = (APB1PERIPH_BASE + 0x7800);
pub const UART8_BASE          :u32 = (APB1PERIPH_BASE + 0x7C00);

/* APB2 peripherals */
pub const TIM1_BASE           :u32 = (APB2PERIPH_BASE + 0x0000);
pub const TIM8_BASE           :u32 = (APB2PERIPH_BASE + 0x0400);
pub const USART1_BASE         :u32 = (APB2PERIPH_BASE + 0x1000);
pub const USART6_BASE         :u32 = (APB2PERIPH_BASE + 0x1400);
pub const ADC1_BASE           :u32 = (APB2PERIPH_BASE + 0x2000);
pub const ADC2_BASE           :u32 = (APB2PERIPH_BASE + 0x2100);
pub const ADC3_BASE           :u32 = (APB2PERIPH_BASE + 0x2200);
pub const ADC123_COMMON_BASE  :u32 = (APB2PERIPH_BASE + 0x2300);
/* Legacy define */
pub const ADC_BASE            :u32 = ADC123_COMMON_BASE;
pub const SDIO_BASE           :u32 = (APB2PERIPH_BASE + 0x2C00);
pub const SPI1_BASE           :u32 = (APB2PERIPH_BASE + 0x3000);
pub const SPI4_BASE           :u32 = (APB2PERIPH_BASE + 0x3400);
pub const SYSCFG_BASE         :u32 = (APB2PERIPH_BASE + 0x3800);
pub const EXTI_BASE           :u32 = (APB2PERIPH_BASE + 0x3C00);
pub const TIM9_BASE           :u32 = (APB2PERIPH_BASE + 0x4000);
pub const TIM10_BASE          :u32 = (APB2PERIPH_BASE + 0x4400);
pub const TIM11_BASE          :u32 = (APB2PERIPH_BASE + 0x4800);
pub const SPI5_BASE           :u32 = (APB2PERIPH_BASE + 0x5000);
pub const SPI6_BASE           :u32 = (APB2PERIPH_BASE + 0x5400);
pub const SAI1_BASE           :u32 = (APB2PERIPH_BASE + 0x5800);
pub const SAI1_Block_A_BASE   :u32 = (SAI1_BASE + 0x004);
pub const SAI1_Block_B_BASE   :u32 = (SAI1_BASE + 0x024);

/* AHB1 peripherals */
pub const GPIOA_BASE          :u32 = (AHB1PERIPH_BASE + 0x0000);
pub const GPIOB_BASE          :u32 = (AHB1PERIPH_BASE + 0x0400);
pub const GPIOC_BASE          :u32 = (AHB1PERIPH_BASE + 0x0800);
pub const GPIOD_BASE          :u32 = (AHB1PERIPH_BASE + 0x0C00);
pub const GPIOE_BASE          :u32 = (AHB1PERIPH_BASE + 0x1000);
pub const GPIOF_BASE          :u32 = (AHB1PERIPH_BASE + 0x1400);
pub const GPIOG_BASE          :u32 = (AHB1PERIPH_BASE + 0x1800);
pub const GPIOH_BASE          :u32 = (AHB1PERIPH_BASE + 0x1C00);
pub const GPIOI_BASE          :u32 = (AHB1PERIPH_BASE + 0x2000);
pub const GPIOJ_BASE          :u32 = (AHB1PERIPH_BASE + 0x2400);
pub const GPIOK_BASE          :u32 = (AHB1PERIPH_BASE + 0x2800);
pub const CRC_BASE            :u32 = (AHB1PERIPH_BASE + 0x3000);
pub const RCC_BASE            :u32 = (AHB1PERIPH_BASE + 0x3800);
pub const FLASH_R_BASE        :u32 = (AHB1PERIPH_BASE + 0x3C00);
pub const DMA1_BASE           :u32 = (AHB1PERIPH_BASE + 0x6000);
pub const DMA1_Stream0_BASE   :u32 = (DMA1_BASE + 0x010);
pub const DMA1_Stream1_BASE   :u32 = (DMA1_BASE + 0x028);
pub const DMA1_Stream2_BASE   :u32 = (DMA1_BASE + 0x040);
pub const DMA1_Stream3_BASE   :u32 = (DMA1_BASE + 0x058);
pub const DMA1_Stream4_BASE   :u32 = (DMA1_BASE + 0x070);
pub const DMA1_Stream5_BASE   :u32 = (DMA1_BASE + 0x088);
pub const DMA1_Stream6_BASE   :u32 = (DMA1_BASE + 0x0A0);
pub const DMA1_Stream7_BASE   :u32 = (DMA1_BASE + 0x0B8);
pub const DMA2_BASE           :u32 = (AHB1PERIPH_BASE + 0x6400);
pub const DMA2_Stream0_BASE   :u32 = (DMA2_BASE + 0x010);
pub const DMA2_Stream1_BASE   :u32 = (DMA2_BASE + 0x028);
pub const DMA2_Stream2_BASE   :u32 = (DMA2_BASE + 0x040);
pub const DMA2_Stream3_BASE   :u32 = (DMA2_BASE + 0x058);
pub const DMA2_Stream4_BASE   :u32 = (DMA2_BASE + 0x070);
pub const DMA2_Stream5_BASE   :u32 = (DMA2_BASE + 0x088);
pub const DMA2_Stream6_BASE   :u32 = (DMA2_BASE + 0x0A0);
pub const DMA2_Stream7_BASE   :u32 = (DMA2_BASE + 0x0B8);
pub const ETH_BASE            :u32 = (AHB1PERIPH_BASE + 0x8000);
pub const ETH_MAC_BASE        :u32 = (ETH_BASE);
pub const ETH_MMC_BASE        :u32 = (ETH_BASE + 0x0100);
pub const ETH_PTP_BASE        :u32 = (ETH_BASE + 0x0700);
pub const ETH_DMA_BASE        :u32 = (ETH_BASE + 0x1000);
pub const DMA2D_BASE          :u32 = (AHB1PERIPH_BASE + 0xB000);

/* AHB2 peripherals */
pub const DCMI_BASE           :u32 = (AHB2PERIPH_BASE + 0x50000);
pub const RNG_BASE            :u32 = (AHB2PERIPH_BASE + 0x60800);

/* FMC Bankx registers base address */
pub const FMC_Bank1_R_BASE    :u32 = (FMC_R_BASE + 0x0000);
pub const FMC_Bank1E_R_BASE   :u32 = (FMC_R_BASE + 0x0104);
pub const FMC_Bank2_3_R_BASE  :u32 = (FMC_R_BASE + 0x0060);
pub const FMC_Bank4_R_BASE    :u32 = (FMC_R_BASE + 0x00A0);
pub const FMC_Bank5_6_R_BASE  :u32 = (FMC_R_BASE + 0x0140);


/* Debug MCU registers base address */
pub const DBGMCU_BASE       :u32 = 0xE0042000;
/* USB registers base address */
pub const USB_OTG_HS_PERIPH_BASE           :u32 = 0x40040000;
pub const USB_OTG_FS_PERIPH_BASE           :u32 = 0x50000000;

pub const USB_OTG_GLOBAL_BASE              :u32 = 0x000;
pub const USB_OTG_DEVICE_BASE              :u32 = 0x800;
pub const USB_OTG_IN_ENDPOINT_BASE         :u32 = 0x900;
pub const USB_OTG_OUT_ENDPOINT_BASE        :u32 = 0xB00;
pub const USB_OTG_EP_REG_SIZE              :u32 = 0x20;
pub const USB_OTG_HOST_BASE                :u32 = 0x400;
pub const USB_OTG_HOST_PORT_BASE           :u32 = 0x440;
pub const USB_OTG_HOST_CHANNEL_BASE        :u32 = 0x500;
pub const USB_OTG_HOST_CHANNEL_SIZE        :u32 = 0x20;
pub const USB_OTG_PCGCCTL_BASE             :u32 = 0xE00;
pub const USB_OTG_FIFO_BASE                :u32 = 0x1000;
pub const USB_OTG_FIFO_SIZE                :u32 = 0x1000;

/* Unique device ID register base address */
pub const UID_BASE                 :u32 = 0x1FFF7A10; 
/* FLASH Size register base address       */
pub const FLASHSIZE_BASE           :u32 = 0x1FFF7A22; 
/* Package size register base address     */
pub const PACKAGE_BASE             :u32 = 0x1FFF7BF0; 
