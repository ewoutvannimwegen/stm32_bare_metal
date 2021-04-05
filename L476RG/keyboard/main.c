#include "stm32l476xx.h"

#include "main.h"

#define MAX_EP          6
#define MAX_RX_PACKET   128
#define MAX_CONTROL_EP  1
#define MAX_FIFO_SZ     320  /*in 32-bit chunks */
#define RX_FIFO_SZ      ((4 * MAX_CONTROL_EP + 6) + ((MAX_RX_PACKET / 4) + 1) + (MAX_EP * 2) + 1)
#define STATUS_VAL(x)   (USBD_HW_BC | USBD_HW_ADDRFST | (x))

static USB_OTG_GlobalTypeDef * const OTG  = (void*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_GLOBAL_BASE);
static USB_OTG_DeviceTypeDef * const OTGD = (void*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
static volatile uint32_t * const OTGPCTL  = (void*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE);

inline static uint32_t* EPFIFO(uint32_t ep) {
    return (uint32_t*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + (ep << 12));
}

inline static USB_OTG_INEndpointTypeDef* EPIN(uint32_t ep) {
    return (void*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (ep << 5));
}

inline static USB_OTG_OUTEndpointTypeDef* EPOUT(uint32_t ep) {
    return (void*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (ep << 5));
}

void clkInit(void);
void ioInit(void);
void usbCoreInit(void);
void usbDeviceInit(void);

int main(void)
{
    clkInit();
    ioInit();
    usbCoreInit();
    usbDeviceInit();
    while (1);
}

void clkInit(void)
{
    /* MSI */
    RCC->CR |= RCC_CR_MSION; // MSI enabled
    RCC->CFGR &= ~RCC_CFGR_SW; // MSI -> SYSCLK
    while(!(RCC->CR & RCC_CR_MSIRDY));

    /* PWR */
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; // Power interface clock enabled
    PWR->CR1 &= ~PWR_CR1_VOS_1;
    PWR->CR1 |= PWR_CR1_VOS_0; // Voltage scaling range 1
    while(PWR->SR2 & PWR_SR2_VOSF); // Regulator output is changing
    
    /* FLASH */
    FLASH->ACR |= FLASH_ACR_ICEN;   // Instruction cache enabled
    FLASH->ACR |= FLASH_ACR_PRFTEN; // Prefetch enabled
    FLASH->ACR |= FLASH_ACR_DCEN;   // Data cache enabled
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS; // Flash latency 3 CPU cycles
    
    /* MSI RANGE */
    RCC->CR &= ~RCC_CR_MSIRANGE;
    RCC->CR |= RCC_CR_MSIRANGE_11 | RCC_CR_MSIRGSEL; // MSI 48MHz
    while(!(RCC->CR & RCC_CR_MSIRDY));
    RCC->CCIPR |= RCC_CCIPR_CLK48SEL_1 | RCC_CCIPR_CLK48SEL_0; // MSI->USB
    SystemCoreClockUpdate();
}

void ioInit(void)
{
    /* Clocks */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // GPIOA clock enabled

    /* OTG_FS_DM [PA11] */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT11;    // Output push-pull
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD11_1 | GPIO_PUPDR_PUPD11_0); // No pull                             
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED11_1 | GPIO_OSPEEDR_OSPEED11_0; // Very high speed
    GPIOA->MODER |= GPIO_MODER_MODE11_1;
    GPIOA->MODER &= ~GPIO_MODER_MODE11_0; // Alternate function
    GPIOA->AFR[1] = 0; // Reset high alternate function register
    GPIOA->AFR[1] |= GPIO_AFRH_AFSEL11_3 |  GPIO_AFRH_AFSEL11_1; // AF10 OTG_FS_DM

    /* OTG_FS_DP [PA12] */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT12;                                  // Output push-pull
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD12_1 | GPIO_PUPDR_PUPD12_0); // No pull  
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED12_1 | GPIO_OSPEEDR_OSPEED12_0; // Very high speed
    GPIOA->MODER |= GPIO_MODER_MODE12_1;
    GPIOA->MODER &= ~GPIO_MODER_MODE12_0; // Alternate function
    GPIOA->AFR[1] |= GPIO_AFRH_AFSEL12_3 | GPIO_AFRH_AFSEL12_1; // AF10 OTG_FS_DP
}

// https://github.com/dmitrystu/libusb_stm32/blob/master/src/usbd_stm32l476_otgfs.c
// https://www.usbmadesimple.co.uk/index.html
void usbCoreInit(void)
{
    /* Clocks */
    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN; // OTG FS clock enabled
    PWR->CR2 |= PWR_CR2_USV; // USB power enabled

    /* Core soft reset */
    OTG->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL; // FS serial transceiver
    while(!(OTG->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL)); // AHB master idle
    OTG->GRSTCTL |= USB_OTG_GRSTCTL_CSRST; // Core soft reset
    while(OTG->GRSTCTL & USB_OTG_GRSTCTL_CSRST); // Wait for self clear 
    
    /* Device mode */
    OTG->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL | // Force device mode
    USB_OTG_GUSBCFG_TRDT_2 | USB_OTG_GUSBCFG_TRDT_1; // Frequency

    /* Vbus */
    OTG->GCCFG &= ~USB_OTG_GCCFG_VBDEN; // Vbus sensing off
    OTG->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN | USB_OTG_GOTGCTL_BVALOVAL;
    
    OTG->GCCFG = USB_OTG_GCCFG_PWRDWN; // USB FS transceiver enabled
}

void usbDeviceInit(void) {
    *OTGPCTL = 0; // Restart PHY
    OTGD->DCTL |= USB_OTG_DCTL_SDIS; // Soft disconnect
    OTGD->DCFG |= USB_OTG_DCFG_DSPD_1 | USB_OTG_DCFG_DSPD_0; // Full speed
    OTGD->DCFG &= ~(USB_OTG_DCFG_PERSCHIVL_1 | USB_OTG_DCFG_PERSCHIVL_0); // 80% of the frame interval

    /* Interrupts */
    OTGD->DIEPMSK |= USB_OTG_DIEPMSK_XFRCM; // Transfer completed interrupt unmasked
    OTG->GINTMSK |= USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM |
    USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_WUIM |
    USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_RXFLVLM;
    OTG->GINTSTS = 0xFFFFFFFF; // Clear pending interrupts
    OTG->GAHBCFG |= USB_OTG_GAHBCFG_GINT; // Unmask global interrupt

    /* FIFO */
    OTG->GRXFSIZ = RX_FIFO_SZ; // Max FIFO size
    OTG->DIEPTXF0_HNPTXFSIZ = RX_FIFO_SZ | (0x10 << 16); // EP0 Tx FIFO 64 byte
}



