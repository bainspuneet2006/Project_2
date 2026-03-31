//  STM32 IR Transmitter - as simple as possible
//  Protocol: send N pulses = command N, then 50ms gap, repeat forever

#include "../Common/Include/stm32l051xx.h"
#include "adc.h"
#include <stdio.h>

#define F_CPU          32000000L
#define IR_FREQ        38000L
#define HIGH_THRESHOLD 3000
#define LOW_THRESHOLD  1000

// Command numbers - just how many pulses to send
#define CMD_STOP              1
#define CMD_FORWARD           2
#define CMD_BACKWARD          3
#define CMD_LEFT              4
#define CMD_RIGHT             5
#define CMD_ROTATE            6
#define CMD_SLIGHT_RIGHT      7
#define CMD_SLIGHT_LEFT       8
#define CMD_SLIGHT_BACK_RIGHT 9
#define CMD_SLIGHT_BACK_LEFT  10
#define CMD_PATH1             11
#define CMD_PATH2             12
#define CMD_PATH3             13
#define CMD_CRUISE            14

int joy_x, joy_y, press;
int sw1, sw2, sw3, cruise_btn;
int cmd;

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
void wait_1ms(void)
{
    SysTick->LOAD = (F_CPU / 1000L) - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while ((SysTick->CTRL & BIT16) == 0);
    SysTick->CTRL = 0x00;
}

void delayms(int len)
{
    while (len--) wait_1ms();
}

// ---------------------------------------------------------------------------
// UART for debug prints
// ---------------------------------------------------------------------------
void UART1_Init(void)
{
    RCC->IOPENR  |= BIT0;
    RCC->APB2ENR |= BIT14;
    GPIOA->MODER &= ~(BIT19 | BIT18);
    GPIOA->MODER |=   BIT19;
    GPIOA->AFR[1] &= ~(0xF << 4);
    GPIOA->AFR[1] |=  (4   << 4);
    USART1->BRR   = F_CPU / 115200;
    USART1->CR1  |= BIT3 | BIT0;
}

int _write(int file, char *ptr, int len)
{
    int i;
    for (i = 0; i < len; i++)
    {
        while (!(USART1->ISR & BIT7));
        USART1->TDR = ptr[i];
    }
    return len;
}

// ---------------------------------------------------------------------------
// GPIO - joystick, buttons
// ---------------------------------------------------------------------------
void GPIO_Init(void)
{
    RCC->IOPENR |= BIT0 | BIT1;

    // PA2, PA3 = analog (joystick axes)
    GPIOA->MODER |= (BIT4 | BIT5 | BIT6 | BIT7);

    // PA4=press, PA5=sw1, PA6=sw2, PA7=sw3 - inputs with pull-up
    GPIOA->MODER &= ~(BIT8|BIT9|BIT10|BIT11|BIT12|BIT13|BIT14|BIT15);
    GPIOA->PUPDR |=  (BIT8|BIT10|BIT12|BIT14);

    // PB0 = cruise button - input with pull-up
    GPIOB->MODER &= ~(BIT0 | BIT1);
    GPIOB->PUPDR |=   BIT0;
}

// ---------------------------------------------------------------------------
// IR 38kHz carrier on PA15 via TIM2_CH1
// ---------------------------------------------------------------------------
void IR_PWM_Init(void)
{
    RCC->IOPENR  |= BIT0;
    RCC->APB1ENR |= BIT0;

    // PA15 alternate function 5 = TIM2_CH1
    GPIOA->MODER  = (GPIOA->MODER  & ~(BIT30 | BIT31)) | BIT31;
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xF << 28))     | (5 << 28);

    TIM2->PSC   = 0;
    TIM2->ARR   = (F_CPU / IR_FREQ) - 1;
    TIM2->CCR1  = (TIM2->ARR + 1) / 2;   // 50% duty cycle
    TIM2->CCMR1 = (6 << 4) | BIT3;       // PWM mode 1, preload enable
    TIM2->CCER &= ~BIT0;                  // output off initially
    TIM2->CR1  |= BIT7 | BIT0;
    TIM2->EGR  |= BIT0;
}

void carrier_on(void)  { TIM2->CCER |=  BIT0; }
void carrier_off(void) { TIM2->CCER &= ~BIT0; }

// ---------------------------------------------------------------------------
// Read all inputs
// ---------------------------------------------------------------------------
void read_inputs(void)
{
    joy_x      = readADC(ADC_CHSELR_CHSEL3);
    joy_y      = readADC(ADC_CHSELR_CHSEL2);
    press      = (GPIOA->IDR & BIT4) ? 0 : 1;
    sw1        = (GPIOA->IDR & BIT5) ? 0 : 1;
    sw2        = (GPIOA->IDR & BIT6) ? 0 : 1;
    sw3        = (GPIOA->IDR & BIT7) ? 0 : 1;
    cruise_btn = (GPIOB->IDR & BIT0) ? 0 : 1;
}

// ---------------------------------------------------------------------------
// Pick command from inputs
// ---------------------------------------------------------------------------
void assign_command(void)
{
    if      (sw1)                                                 cmd = CMD_PATH1;
    else if (sw2)                                                 cmd = CMD_PATH2;
    else if (sw3)                                                 cmd = CMD_PATH3;
    else if (cruise_btn)                                          cmd = CMD_CRUISE;
    else if (press)                                               cmd = CMD_ROTATE;
    else if (joy_y >= HIGH_THRESHOLD && joy_x >= HIGH_THRESHOLD) cmd = CMD_SLIGHT_RIGHT;
    else if (joy_y >= HIGH_THRESHOLD && joy_x <= LOW_THRESHOLD)  cmd = CMD_SLIGHT_LEFT;
    else if (joy_y <= LOW_THRESHOLD  && joy_x >= HIGH_THRESHOLD) cmd = CMD_SLIGHT_BACK_RIGHT;
    else if (joy_y <= LOW_THRESHOLD  && joy_x <= LOW_THRESHOLD)  cmd = CMD_SLIGHT_BACK_LEFT;
    else if (joy_y >= HIGH_THRESHOLD)                             cmd = CMD_FORWARD;
    else if (joy_y <= LOW_THRESHOLD)                              cmd = CMD_BACKWARD;
    else if (joy_x >= HIGH_THRESHOLD)                             cmd = CMD_RIGHT;
    else if (joy_x <= LOW_THRESHOLD)                              cmd = CMD_LEFT;
    else                                                          cmd = CMD_STOP;
}

// ---------------------------------------------------------------------------
// Send command: N pulses (5ms on, 5ms off), then 50ms end gap
// Receiver just counts pulses before the long gap
// ---------------------------------------------------------------------------
void send_command(int n)
{
    int i;
    for (i = 0; i < n; i++)
    {
        carrier_on();
        delayms(5);
        carrier_off();
        delayms(5);
    }
    delayms(50);  // long gap = end of message
}

// ---------------------------------------------------------------------------
// Main - read inputs, send command, repeat forever
// ---------------------------------------------------------------------------
void main(void)
{
    int last_cmd = -1;

    GPIO_Init();
    initADC();
    UART1_Init();
    setbuf(stdout, NULL);
    IR_PWM_Init();

    printf("\r\nSTM32 IR TX READY\r\n");

    while (1)
    {
        read_inputs();
        assign_command();

        // Only print when command changes
        if (cmd != last_cmd)
        {
            printf("TX: %d\r\n", cmd);
            last_cmd = cmd;
        }

        send_command(cmd);  // sends then waits 50ms, then loops back
    }
}
