#define WEAK __attribute__((weak, alias("Default_Handler")))
#define WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))

void __StackTop(void);  // Defined in link.ld
void __end__(void);     // Defined in link.ld

void Reset_Handler(void);  // Defined below
void SystemInit(void);
void Default_Handler(void) {
  for (;;) (void) 0;
}

WEAK void NMI_Handler(void);
WEAK void HardFault_Handler(void);
WEAK void MemManage_Handler(void);
WEAK void BusFault_Handler(void);
WEAK void UsageFault_Handler(void);
WEAK void SecureFault_Handler(void);
WEAK void SVC_Handler(void);
WEAK void DebugMon_Handler(void);
WEAK void PendSV_Handler(void);
WEAK void SysTick_Handler(void);
WEAK void IntDefaultHandler(void);

WEAK void CTI_IRQHandler(void);
WEAK void CMC0_IRQHandler(void);
WEAK void DMA0_CH0_IRQHandler(void);
WEAK void DMA0_CH1_IRQHandler(void);
WEAK void DMA0_CH2_IRQHandler(void);
WEAK void DMA0_CH3_IRQHandler(void);
WEAK void DMA0_CH4_IRQHandler(void);
WEAK void DMA0_CH5_IRQHandler(void);
WEAK void DMA0_CH6_IRQHandler(void);
WEAK void DMA0_CH7_IRQHandler(void);
WEAK void DMA0_CH8_IRQHandler(void);
WEAK void DMA0_CH9_IRQHandler(void);
WEAK void DMA0_CH10_IRQHandler(void);
WEAK void DMA0_CH11_IRQHandler(void);
WEAK void DMA0_CH12_IRQHandler(void);
WEAK void DMA0_CH13_IRQHandler(void);
WEAK void DMA0_CH14_IRQHandler(void);
WEAK void DMA0_CH15_IRQHandler(void);
WEAK void EWM0_IRQHandler(void);
WEAK void MCM0_IRQHandler(void);
WEAK void MSCM0_IRQHandler(void);
WEAK void SPC0_IRQHandler(void);
WEAK void WUU0_IRQHandler(void);
WEAK void WDOG0_IRQHandler(void);
WEAK void WDOG1_IRQHandler(void);
WEAK void SCG0_IRQHandler(void);
WEAK void SFA0_IRQHandler(void);
WEAK void FMU0_IRQHandler(void);
WEAK void ELE_CMD_IRQHandler(void);
WEAK void ELE_SECURE_IRQHandler(void);
WEAK void ELE_NONSECURE_IRQHandler(void);
WEAK void TRDC0_IRQHandler(void);
WEAK void RTC_Alarm_IRQHandler(void);
WEAK void RTC_Seconds_IRQHandler(void);
WEAK void LPTMR0_IRQHandler(void);
WEAK void LPTMR1_IRQHandler(void);
WEAK void LPIT0_IRQHandler(void);
WEAK void TPM0_IRQHandler(void);
WEAK void TPM1_IRQHandler(void);
WEAK void LPI2C0_IRQHandler(void);
WEAK void LPI2C1_IRQHandler(void);
WEAK void I3C0_IRQHandler(void);
WEAK void LPSPI0_IRQHandler(void);
WEAK void LPSPI1_IRQHandler(void);
WEAK void LPUART0_IRQHandler(void);
WEAK void LPUART1_IRQHandler(void);
WEAK void FLEXIO0_IRQHandler(void);
WEAK void CAN0_IRQHandler(void);
WEAK void RF_IMU0_IRQHandler(void);
WEAK void RF_IMU1_IRQHandler(void);
WEAK void RF_NBU_IRQHandler(void);
WEAK void RF_FMU_IRQHandler(void);
WEAK void RF_WOR_IRQHandler(void);
WEAK void RF_802_15_4_IRQHandler(void);
WEAK void RF_Generic_IRQHandler(void);
WEAK void RF_BRIC_IRQHandler(void);
WEAK void RF_LANT_SW_IRQHandler(void);
WEAK void RFMC_IRQHandler(void);
WEAK void DSB_IRQHandler(void);
WEAK void GPIOA_INT0_IRQHandler(void);
WEAK void GPIOA_INT1_IRQHandler(void);
WEAK void GPIOB_INT0_IRQHandler(void);
WEAK void GPIOB_INT1_IRQHandler(void);
WEAK void GPIOC_INT0_IRQHandler(void);
WEAK void GPIOC_INT1_IRQHandler(void);
WEAK void GPIOD_INT0_IRQHandler(void);
WEAK void GPIOD_INT1_IRQHandler(void);
WEAK void PORTA_EFT_IRQHandler(void);
WEAK void PORTB_EFT_IRQHandler(void);
WEAK void PORTC_EFT_IRQHandler(void);
WEAK void PORTD_EFT_IRQHandler(void);
WEAK void ADC0_IRQHandler(void);
WEAK void LPCMP0_IRQHandler(void);
WEAK void LPCMP1_IRQHandler(void);
WEAK void VBAT_IRQHandler(void);
WEAK void Reserved91_IRQHandler(void);

__attribute__((section(".vectors"))) void (*const tab[16 + 76])(void) = {
    &__StackTop,          // The initial stack pointer
    Reset_Handler,        // The reset handler
    NMI_Handler,          // NMI Handler
    HardFault_Handler,    // Hard Fault Handler
    MemManage_Handler,    // MPU Fault Handler
    BusFault_Handler,     // Bus Fault Handler
    UsageFault_Handler,   // Usage Fault Handler
    SecureFault_Handler,  // Secure Fault Handler
    0,                    // Image size
    0,                    // Image type
    0,                    // Reserved
    SVC_Handler,          // SVCall Handler
    DebugMon_Handler,     // Debug Monitor Handler
    0,                    // Image load address
    PendSV_Handler,       // PendSV Handler
    SysTick_Handler,      // SysTick Handler

    // Chip Level - MCXW71
    CTI_IRQHandler,
    CMC0_IRQHandler,
    DMA0_CH0_IRQHandler,
    DMA0_CH1_IRQHandler,
    DMA0_CH2_IRQHandler,
    DMA0_CH3_IRQHandler,
    DMA0_CH4_IRQHandler,
    DMA0_CH5_IRQHandler,
    DMA0_CH6_IRQHandler,
    DMA0_CH7_IRQHandler,
    DMA0_CH8_IRQHandler,
    DMA0_CH9_IRQHandler,
    DMA0_CH10_IRQHandler,
    DMA0_CH11_IRQHandler,
    DMA0_CH12_IRQHandler,
    DMA0_CH13_IRQHandler,
    DMA0_CH14_IRQHandler,
    DMA0_CH15_IRQHandler,
    EWM0_IRQHandler,
    MCM0_IRQHandler,
    MSCM0_IRQHandler,
    SPC0_IRQHandler,
    WUU0_IRQHandler,
    WDOG0_IRQHandler,
    WDOG1_IRQHandler,
    SCG0_IRQHandler,
    SFA0_IRQHandler,
    FMU0_IRQHandler,
    ELE_CMD_IRQHandler,
    ELE_SECURE_IRQHandler,
    ELE_NONSECURE_IRQHandler,
    TRDC0_IRQHandler,
    RTC_Alarm_IRQHandler,
    RTC_Seconds_IRQHandler,
    LPTMR0_IRQHandler,
    LPTMR1_IRQHandler,
    LPIT0_IRQHandler,
    TPM0_IRQHandler,
    TPM1_IRQHandler,
    LPI2C0_IRQHandler,
    LPI2C1_IRQHandler,
    I3C0_IRQHandler,
    LPSPI0_IRQHandler,
    LPSPI1_IRQHandler,
    LPUART0_IRQHandler,
    LPUART1_IRQHandler,
    FLEXIO0_IRQHandler,
    CAN0_IRQHandler,
    RF_IMU0_IRQHandler,
    RF_IMU1_IRQHandler,
    RF_NBU_IRQHandler,
    RF_FMU_IRQHandler,
    RF_WOR_IRQHandler,
    RF_802_15_4_IRQHandler,
    RF_Generic_IRQHandler,
    RF_BRIC_IRQHandler,
    RF_LANT_SW_IRQHandler,
    RFMC_IRQHandler,
    DSB_IRQHandler,
    GPIOA_INT0_IRQHandler,
    GPIOA_INT1_IRQHandler,
    GPIOB_INT0_IRQHandler,
    GPIOB_INT1_IRQHandler,
    GPIOC_INT0_IRQHandler,
    GPIOC_INT1_IRQHandler,
    GPIOD_INT0_IRQHandler,
    GPIOD_INT1_IRQHandler,
    PORTA_EFT_IRQHandler,
    PORTB_EFT_IRQHandler,
    PORTC_EFT_IRQHandler,
    PORTD_EFT_IRQHandler,
    ADC0_IRQHandler,
    LPCMP0_IRQHandler,
    LPCMP1_IRQHandler,
    VBAT_IRQHandler,
    Reserved91_IRQHandler,
};

extern unsigned char _end[];  // End of data section, start of heap. See link.ld
__attribute__((naked, noreturn)) void Reset_Handler(void) {
  asm("cpsid i");  // Disable interrupts
#if 1
  // Config VTOR & MSPLIM register
  asm("ldr r0, =0Xe000ed08  \n"
      "str %0, [r0]         \n"
      "ldr r1, [%0]         \n"
      "msr msp, r1          \n"
      "msr msplim, %1       \n"
      :
      : "r"(tab), "r"(__end__)
      : "r0", "r1");
#endif
  extern long __bss_start__, __bss_end__, __data_start__, __data_end__, __etext;
  for (long *dst = &__bss_start__; dst < &__bss_end__;) *dst++ = 0;
  for (long *dst = &__data_start__, *src = &__etext; dst < &__data_end__;)
    *dst++ = *src++;
  // extern long _siram, _siiram, _eiram;
  // for (long *dst = &_siram, *src = &_siiram; dst < &_eiram;) *dst++ = *src++;
  SystemInit();
  __asm("cpsie i");  // Reenable interrupts
  extern void main(void);
  main();             // Call main()
  for (;;) (void) 0;  // Infinite loop
}

__attribute__((weak)) void SystemCoreClockUpdate(void) {
}
