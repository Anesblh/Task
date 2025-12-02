#include "stm32f7xx.h"

#define LEDG_PIN 0   // PB0
#define LEDB_PIN 7   // PB7
#define LEDR_PIN 14  // PB14
#define BTN_PIN 13   // PC13

#define BUTTON_PRESSED 1
#define BUTTON_RELEASED 0

volatile uint32_t msTicks = 0;
volatile uint8_t ledIndex = 0;
volatile uint8_t autoMode = 0;
volatile uint8_t buttonState = 0;
volatile uint32_t btnPressStart = 0;
volatile int32_t encoder_position = 50;
volatile uint32_t led_blink_period = 500;
volatile uint32_t last_led_toggle = 0;
volatile uint32_t encoder_interrupt_count = 0;

// SysTick Handler
void SysTick_Handler(void) {
    msTicks++;
}

// GPIO Init 
void GPIO_Init(void) {
    // Enable clocks for GPIOA, GPIOB, and GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // Configure PB0, PB7, PB14 as outputs (LEDs)
    GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER7 | GPIO_MODER_MODER14);
    GPIOB->MODER |= (1 << (2 * LEDG_PIN)) | (1 << (2 * LEDB_PIN)) | (1 << (2 * LEDR_PIN));

    // Configure PA0 (CLK) and PA1 (DT) as inputs with pull-up 
    GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1);
}

// Timer2: LED Blink Timer
void TIM2_Init(uint32_t ms) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 10800 - 1;
    TIM2->ARR = ms * 10;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);
}

// Timer3: Debounce Timer
void TIM3_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 10800 - 1;
    TIM3->ARR = 100;
    TIM3->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM3_IRQn);
}

// EXTI Configuration 
void EXTI_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure EXTI13 for button (PC13)
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
    EXTI->IMR |= EXTI_IMR_MR13;
    EXTI->RTSR |= EXTI_RTSR_TR13;
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    // Configure EXTI0 for encoder CLK (PA0)
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
    EXTI->IMR |= EXTI_IMR_MR0;
    EXTI->FTSR |= EXTI_FTSR_TR0;   // Falling edge
    EXTI->RTSR |= EXTI_RTSR_TR0;   // Also enable rising edge for testing
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_SetPriority(EXTI0_IRQn, 2);
}

// TIM2 IRQ Handler: LED Blinker
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;

        GPIOB->ODR &= ~((1 << LEDG_PIN) | (1 << LEDB_PIN) | (1 << LEDR_PIN));

        switch (ledIndex) {
            case 0:
                GPIOB->ODR |= (1 << LEDG_PIN);
                break;
            case 1:
                GPIOB->ODR |= (1 << LEDB_PIN);
                break;
            case 2:
                GPIOB->ODR |= (1 << LEDR_PIN);
                break;
        }

        if (autoMode) {
            ledIndex = (ledIndex + 1) % 3;
        }
    }
}

// TIM3 IRQ Handler: Debounce Timer
void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF;
        TIM3->CR1 &= ~TIM_CR1_CEN;

        if (!(GPIOC->IDR & (1 << BTN_PIN))) {
            if (buttonState == BUTTON_RELEASED) {
                buttonState = BUTTON_PRESSED;
                ledIndex = (ledIndex + 1) % 3;
                btnPressStart = msTicks;
            }
        }

        EXTI->IMR |= EXTI_IMR_MR13;
    }
}

// EXTI Handler for Button (PC13)
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR13) {
        EXTI->PR = EXTI_PR_PR13;
        EXTI->IMR &= ~EXTI_IMR_MR13;
        TIM3->CNT = 0;
        TIM3->CR1 |= TIM_CR1_CEN;
    }
}

// EXTI Handler for Encoder (PA0) - FIXED FOR STM32F767xx
void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR |= EXTI_PR_PR0;

        // Debug: Count interrupts and blink red LED
        encoder_interrupt_count++;
        GPIOB->ODR ^= (1 << LEDR_PIN);  // Toggle red LED on each interrupt

        // Small delay for signal stability
        for(volatile int i = 0; i < 100; i++);

        // Read current state of DT pin (PA1) - CORRECTED REGISTER NAME
        uint32_t dt_state = (GPIOA->IDR & GPIO_IDR_ID1) ? 1 : 0;

        // Determine rotation direction
        if (dt_state == 0) {
            encoder_position++;
            if (encoder_position > 100) {
                encoder_position = 100;
            }
        } else {
            encoder_position--;
            if (encoder_position < 0) {
                encoder_position = 0;
            }
        }

        // Update LED blink frequency
        led_blink_period = 1000 - (encoder_position * 9);
        TIM2->ARR = led_blink_period * 10;
    }
}

// Clock Init: 216 MHz
void clock_init(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    FLASH->ACR |= FLASH_ACR_LATENCY_7WS;

    RCC->PLLCFGR = (25 << RCC_PLLCFGR_PLLM_Pos) |
                   (432 << RCC_PLLCFGR_PLLN_Pos) |
                   (0 << RCC_PLLCFGR_PLLP_Pos) |
                   RCC_PLLCFGR_PLLSRC_HSE |
                   (9 << RCC_PLLCFGR_PLLQ_Pos);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    SystemCoreClockUpdate();
}

// Main Program
int main(void) {
    clock_init();
    SysTick_Config(SystemCoreClock / 1000);
    GPIO_Init();
    TIM2_Init(500);
    TIM3_Init();
    EXTI_Init();

    while (1) {
        if ((buttonState == BUTTON_PRESSED) && (GPIOC->IDR & (1 << BTN_PIN))) {
            uint32_t pressDuration = msTicks - btnPressStart;

            if (pressDuration >= 2000) {
                autoMode = 1;
            }

            buttonState = BUTTON_RELEASED;
        }

        if ((buttonState == BUTTON_PRESSED) && autoMode) {
            autoMode = 0;
        }
    }
}

