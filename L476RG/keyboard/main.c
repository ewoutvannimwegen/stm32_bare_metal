// ToDo: HSI groter dan 48MHz maken

#include "stm32l476xx.h"

#include "main.h"

#define USBx_PCGCCTL *(__IO uint32_t*)((uint32_t)USBx_BASE + USB_OTG_PCGCCTL_BASE)
#define USBx_INEP(i) ((USB_OTG_INEndpointTypeDef*)(USBx_BASE + USB_OTG_IN_ENDPOINT_BASE + ((i)*USB_OTG_EP_REG_SIZE)))
#define USBx_OUTEP(i) ((USB_OTG_OUTEndpointTypeDef*)(USBx_BASE + USB_OTG_OUT_ENDPOINT_BASE + ((i)*USB_OTG_EP_REG_SIZE)))

typedef struct {
    uint32_t devEndpoints; // Device endpoint number
} USB_OTG_CfgTypeDef;

USB_OTG_GlobalTypeDef* USBx;
USB_OTG_DeviceTypeDef* USBx_DEVICE;
USB_OTG_INEndpointTypeDef* USBx_INEP;
USB_OTG_CfgTypeDef cfg;

void clkInit(void);
void ioInit(void);
void usbCoreInit(void);
void usbDeviceInit(void);
void usbDeviceConnect(void);

int main(void)
{
    clkInit();
    ioInit();
    usbCoreInit();
    usbDeviceInit();
    usbDeviceConnect();

    while (1) {
        for (uint32_t i = 0; i < (SystemCoreClock / 8); i++)
            ;                       // Sleep 1 sec
        GPIOA->ODR ^= GPIO_ODR_OD5; // Toggle LD2
    }
}

void clkInit(void)
{
    /* HSI */
    RCC->CR |= RCC_CR_HSION; // HSI clock enabled
    while (!(RCC->CR & RCC_CR_HSIRDY))
        ; // Wait for HSI ready

    /* PWR */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; // Power interface clock enabled
    PWR->CR1 &= ~PWR_CR1_VOS_1;
    PWR->CR1 |= PWR_CR1_VOS_0; // Voltage scaling range 1
    PWR->CR2 |= PWR_CR2_USV;   // USB supply valid
    
    /* Flash */
    FLASH->ACR |= FLASH_ACR_ICEN;   // Instruction cache enabled
    FLASH->ACR |= FLASH_ACR_PRFTEN; // Prefetch enabled
    FLASH->ACR |= FLASH_ACR_DCEN;   // Data cache enabled
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_3WS; // Flash latency 4 CPU cycles

    /* PLL */
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI; // HSI -> PLL
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_1 | RCC_PLLCFGR_PLLM_0); // PLLM/1
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_3; // PLLN*8
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_5 |
    RCC_PLLCFGR_PLLN_4 | RCC_PLLCFGR_PLLN_2 | RCC_PLLCFGR_PLLN_1 | 
    RCC_PLLCFGR_PLLN_0);
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLR_1 | RCC_PLLCFGR_PLLR_0); // PLLR/2
    RCC->CR |= RCC_CR_PLLON;                // PLL enabled PLLOUT = 64MHz
    while (!(RCC->CR & RCC_CR_PLLON))
        ;                               // Wait till PLL is locked
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN; // PLLCLK enabled
    RCC->CFGR |= RCC_CFGR_SW_PLL;       // PLLCLK -> SYSCLK
    while (!(RCC->CFGR & RCC_CFGR_SWS_PLL))
        ; // Wait till PLL is configured as SYSCLK
    RCC->CFGR |= RCC_CFGR_HPRE_0; // AHB/1
    
    SystemCoreClockUpdate();
    
    /* MSI */
    RCC->CR &= ~RCC_CR_MSION; // MSI off
    while (!(RCC->CR & RCC_CR_MSIRDY))
        ; // Wait for MSI ready
    
    /* LSE */
    PWR->CR1 |= PWR_CR1_DBP; // Access the RTC and backup registers enabled
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53; // RTC write protection unlocked
    RCC->BDCR |= RCC_BDCR_LSEON; // LSE oscillator enabled
    while (!(RCC->BDCR & RCC_BDCR_LSERDY))
        ; // Wait till LSE ready

    /* MSI */
    RCC->CR |= RCC_CR_MSIPLLEN; // MSI PLL enabled
    RCC->CR |= RCC_CR_MSIRGSEL;    // MSI range is provided by MSIRANGE[3:0] in RCC_CR
    while(!(RCC->CR & RCC_CR_MSIRDY));
    RCC->CR |= RCC_CR_MSION;       // MSI clock enabled
    while (!(RCC->CR & RCC_CR_MSIRDY))
        ; // Wait for MSI ready
    RCC->CR |= RCC_CR_MSIRANGE_11; // MSI 48MHz
    RCC->CCIPR |= RCC_CCIPR_CLK48SEL_1 | RCC_CCIPR_CLK48SEL_0; // MSI->USB
    
    SystemCoreClockUpdate();

    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN; // OTG FS clock enabled
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // GPIOA clock enabled
}

void ioInit(void)
{
    GPIOA->AFR[1] = 0; // Reset high alternate function register

    /* LD2 [PA5] */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;                                    // Initial state LOW
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5_1 | GPIO_OSPEEDR_OSPEED5_0); // Low speed
    GPIOA->MODER &= ~GPIO_MODER_MODE5_1;
    GPIOA->MODER |= GPIO_MODER_MODE5_0; // Output

    /* OTG_FS_DM [PA11] */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT11;    // Output push-pull
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD11_1 | GPIO_PUPDR_PUPD11_0); // No pull                             
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED11_1 | GPIO_OSPEEDR_OSPEED11_0; // Very high speed
    GPIOA->MODER |= GPIO_MODER_MODE11_1;
    GPIOA->MODER &= ~GPIO_MODER_MODE11_0; // Alternate function
    GPIOA->AFR[1] |= GPIO_AFRH_AFSEL11_3 |  GPIO_AFRH_AFSEL11_1; // AF10 OTG_FS_DM

    /* OTG_FS_DP [PA12] */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT12;                                  // Output push-pull
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD12_1 | GPIO_PUPDR_PUPD12_0); // No pull  
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED12_1 | GPIO_OSPEEDR_OSPEED12_0; // Very high speed
    GPIOA->MODER |= GPIO_MODER_MODE12_1;
    GPIOA->MODER &= ~GPIO_MODER_MODE12_0; // Alternate function
    GPIOA->AFR[1] |= GPIO_AFRH_AFSEL12_3 | GPIO_AFRH_AFSEL12_1; // AF10 OTG_FS_DP

    NVIC_EnableIRQ(OTG_FS_IRQn);
}

void usbCoreInit(void)
{
    USBx->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT; // USB interrupts disabled
    USBx->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL; // FS serial transceiver selected
    while (!(USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));
    USBx->GRSTCTL |= USB_OTG_GRSTCTL_CSRST; // Core soft reset
    while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST);                                // Wait till reset completed
    USBx->GCCFG |= USB_OTG_GCCFG_PWRDWN; // USB FS transceiver enabled
}

void usbDeviceInit(void)
{
    uint32_t USBx_BASE = (uint32_t)USBx;
    USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD); // Reset both modes
    USBx->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD; // Force device mode
    
    for (uint32_t i = 0U; i < 15U; i++)
        USBx->DIEPTXF[i] = 0U;              // Reset Tx FIFO IN endpoints
    USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS; // Core generates device disconnect from host
    USBx->GCCFG &= ~USB_OTG_GCCFG_VBDEN;    // VBUS sensing B disabled

    /* B-peripheral session valid override enabled */
    USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
    USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;

    USBx_PCGCCTL = 0U; // Restart physical clock
    USBx_DEVICE->DCFG |= 0U;                                        // Device mode configuration
    USBx_DEVICE->DCFG |= USB_OTG_DCFG_DSPD_1 | USB_OTG_DCFG_DSPD_0; // Full speed

    USBx->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (0x10U << 6));
    while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH)
        ;
    USBx->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
    while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH)
        ;

    USBx_DEVICE->DIEPMSK = 0U;
    USBx_DEVICE->DOEPMSK = 0U;
    USBx_DEVICE->DAINTMSK = 0U;

    /* Configure IN endpoints */
    for (uint32_t i = 0U; i < cfg.devEndpoints; i++) {
        if ((USBx_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA) {
            if (i == 0U)
                USBx_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_SNAK;
            else
                USBx_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
        } else
            USBx_INEP(i)->DIEPCTL = 0U;
        USBx_INEP(i)->DIEPTSIZ = 0U;
        USBx_INEP(i)->DIEPINT = 0xFB7FU;
    }

    /* Configure OUT endpoints */
    for (uint32_t i = 0U; i < cfg.devEndpoints; i++) {
        if ((USBx_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA) {
            if (i == 0U)
                USBx_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_SNAK;
            else
                USBx_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
        } else
            USBx_OUTEP(i)->DOEPCTL = 0U;
        USBx_OUTEP(i)->DOEPTSIZ = 0U;
        USBx_OUTEP(i)->DOEPINT = 0xFB7FU;
    }

    USBx_DEVICE->DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);
    USBx->GINTMSK = 0U;                       // Reset interrupts
    USBx->GINTSTS = 0xBFFFFFFFU;              // Clear pending interrupts
    USBx->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM; // Common interrupts enabled

    /* Device mode interrupts enabled */
    USBx->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_IISOIXFRM | USB_OTG_GINTMSK_PXFRM_IISOOXFRM | USB_OTG_GINTMSK_WUIM;
}

void usbDeviceConnect(void)
{
    uint32_t USBx_BASE = (uint32_t)USBx;
    USBx_PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
    USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS; // Core issues connect after this bit is cleared
}
