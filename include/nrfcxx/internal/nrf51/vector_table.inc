/* SPDX-License-Identifier: CC-BY-SA-4.0 */
/* Copyright 2017-2019 Peter A. Bigot */

/* Vector table declaration for nRF51 microcontrollers */
/* Reset Handler -15 */
/* NMI Handler -14 */
/* Hard Fault Handler -13 */
/* Reserved -12 */
/* Reserved -11 */
/* Reserved -10 */
/* Reserved -9 */
/* Reserved -8 */
/* Reserved -7 */
/* Reserved -6 */
/* SVCall Handler -5 */
/* Reserved -4 */
/* Reserved -3 */
/* PendSV Handler -2 */
/* SysTick Handler -1 */
EXPAND_IRQ_REFERENCE(POWER_CLOCK) /*  0 : POWER_CLOCK */
EXPAND_IRQ_REFERENCE(RADIO)       /*  1 : RADIO */
EXPAND_IRQ_REFERENCE(UART0)       /*  2 : UART0 */
EXPAND_IRQ_REFERENCE(SPI0_TWI0)   /*  3 : SPI0_TWI0 */
EXPAND_IRQ_REFERENCE(SPI1_TWI1)   /*  4 : SPI1_TWI1 */
EXPAND_RESERVED(5)
EXPAND_IRQ_REFERENCE(GPIOTE)      /*  6 : GPIOTE */
EXPAND_IRQ_REFERENCE(ADC)         /*  7 : ADC */
EXPAND_IRQ_REFERENCE(TIMER0)      /*  8 : TIMER0 */
EXPAND_IRQ_REFERENCE(TIMER1)      /*  9 : TIMER1 */
EXPAND_IRQ_REFERENCE(TIMER2)      /* 10 : TIMER2 */
EXPAND_IRQ_REFERENCE(RTC0)        /* 11 : RTC0 */
EXPAND_IRQ_REFERENCE(TEMP)        /* 12 : TEMP */
EXPAND_IRQ_REFERENCE(RNG)         /* 13 : RNG */
EXPAND_IRQ_REFERENCE(ECB)         /* 14 : ECB */
EXPAND_IRQ_REFERENCE(CCM_AAR)     /* 15 : CCM_AAR */
EXPAND_IRQ_REFERENCE(WDT)         /* 16 : WDT */
EXPAND_IRQ_REFERENCE(RTC1)        /* 17 : RTC1 */
EXPAND_IRQ_REFERENCE(QDEC)        /* 18 : QDEC */
EXPAND_IRQ_REFERENCE(LPCOMP)      /* 19 : LPCOMP */
EXPAND_IRQ_REFERENCE(SWI0)        /* 20 : SWI0 */
EXPAND_IRQ_REFERENCE(SWI1)        /* 21 : SWI1 */
EXPAND_IRQ_REFERENCE(SWI2)        /* 22 : SWI2 */
EXPAND_IRQ_REFERENCE(SWI3)        /* 23 : SWI3 */
EXPAND_IRQ_REFERENCE(SWI4)        /* 24 : SWI4 */
EXPAND_IRQ_REFERENCE(SWI5)        /* 25 : SWI5 */
EXPAND_RESERVED(26)
EXPAND_RESERVED(27)
EXPAND_RESERVED(28)
EXPAND_RESERVED(29)
EXPAND_RESERVED(30)
EXPAND_RESERVED(31)

/*
 * Local Variables:
 * mode:c
 * End:
 */
