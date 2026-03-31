//  STM32 IR Transmitter
//  Simple pulse-count protocol: send N pulses = command N
//  Each pulse: 5ms ON, 5ms OFF
//  End of message: 50ms silence

#include "../Common/Include/stm32l051xx.h"
#include "adc.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define F_CPU 32000000L
#define IR_FREQ 38000L

#define HIGH_THRESHOLD 3000
#define LOW_THRESHOLD  1000

// Commands are just numbers 1-14
// The number itself is how many pulses we send
#define CMD_STOP             1
#define CMD_FORWARD          2
#define CMD_BACKWARD         3
#define CMD_LEFT             4
#define CMD_RIGHT            5
#define CMD_ROTATE           6
#define CMD_SLIGHT_RIGHT     7
#define CMD_SLIGHT_LEFT      8
#define CMD_SLIGHT_BACK_RIGHT 9
#define CMD_SLIGHT_BACK_LEFT 10
#define CMD_PATH1            11
#define CMD_PATH2            12
#define CMD_PATH3            13
#define CMD_CRUISE           14

// Global variables
int joy_x;
int joy_y;
int press;
int sw1;
int sw2;
int sw3;
int cruise;
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
// UART for debug
// ---------------------------------------------------------------------------
void UART1_Init(void)
{
    RCC->IOPENR  |= BIT0;
    RCC->APB2ENR |= BIT14;

    GPIOA->MODER &= ~(BIT19 | BIT18);
    GPIOA->MODER |=  BIT19;

    GPIOA->AFR[1] &= ~(0xF << 4);
    GPIOA->AFR[1] |=  (4   << 4);

    USART1->BRR  = 32000000 / 115200;
    USART1->CR1 |= BIT3;
    USART1->CR1 |= BIT0;
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
// GPIO
// ---------------------------------------------------------------------------
void GPIO_Init(void)
{
    RCC->IOPENR |= BIT0; // GPIOA
    RCC->IOPENR |= BIT1; // GPIOB

    // PA2, PA3 analog (joystick)
    GPIOA->MODER = (GPIOA->MODER & ~(BIT4 | BIT5)) | (BIT4 | BIT5);
    GPIOA->MODER = (GPIOA->MODER & ~(BIT6 | BIT7)) | (BIT6 | BIT7);

    // PA4 joystick press input, pull-up
    GPIOA->MODER &= ~(BIT8 | BIT9);
    GPIOA->PUPDR  = (GPIOA->PUPDR & ~(BIT8 | BIT9)) | BIT8;

    // PA5 SW1
    GPIOA->MODER &= ~(BIT10 | BIT11);
    GPIOA->PUPDR  = (GPIOA->PUPDR & ~(BIT10 | BIT11)) | BIT10;

    // PA6 SW2
    GPIOA->MODER &= ~(BIT12 | BIT13);
    GPIOA->PUPDR  = (GPIOA->PUPDR & ~(BIT12 | BIT13)) | BIT12;

    // PA7 SW3
    GPIOA->MODER &= ~(BIT14 | BIT15);
    GPIOA->PUPDR  = (GPIOA->PUPDR & ~(BIT14 | BIT15)) | BIT14;

    // PB0 cruise
    GPIOB->MODER &= ~(BIT0 | BIT1);
    GPIOB->PUPDR  = (GPIOB->PUPDR & ~(BIT0 | BIT1)) | BIT0;
}

// ---------------------------------------------------------------------------
// IR 38kHz carrier on PA15 via TIM2_CH1
// ---------------------------------------------------------------------------
void IR_PWM_Init(void)
{
    RCC->IOPENR  |= BIT0;
    RCC->APB1ENR |= BIT0;

    GPIOA->OTYPER  &= ~BIT15;
    GPIOA->OSPEEDR  = (GPIOA->OSPEEDR & ~(BIT30 | BIT31)) | BIT30;
    GPIOA->MODER    = (GPIOA->MODER   & ~(BIT30 | BIT31)) | BIT31;
    GPIOA->AFR[1]   = (GPIOA->AFR[1]  & ~(0xF << 28))     | (5 << 28);

    TIM2->CR1   = 0;
    TIM2->PSC   = 0;
    TIM2->ARR   = (F_CPU / IR_FREQ) - 1;
    TIM2->CCR1  = (TIM2->ARR + 1) / 2;

    TIM2->CCMR1 &= ~(0xFF);
    TIM2->CCMR1 |=  (6 << 4);
    TIM2->CCMR1 |=  BIT3;

    TIM2->CCER &= ~BIT0;   // start OFF
    TIM2->CR1  |=  BIT7;
    TIM2->EGR  |=  BIT0;
    TIM2->CR1  |=  BIT0;
}

void carrier_on(void)  { TIM2->CCER |=  BIT0; }
void carrier_off(void) { TIM2->CCER &= ~BIT0; }

// ---------------------------------------------------------------------------
// Input reading
// ---------------------------------------------------------------------------
void read_joystick(void)
{
    joy_x = readADC(ADC_CHSELR_CHSEL3);
    joy_y = readADC(ADC_CHSELR_CHSEL2);
    press = (GPIOA->IDR & BIT4) ? 0 : 1;
}

void read_path_control(void)
{
    sw1   = (GPIOA->IDR & BIT5) ? 0 : 1;
    sw2   = (GPIOA->IDR & BIT6) ? 0 : 1;
    sw3   = (GPIOA->IDR & BIT7) ? 0 : 1;
    cruise = (GPIOB->IDR & BIT0) ? 0 : 1;
}

// ---------------------------------------------------------------------------
// Command assignment from inputs
// ---------------------------------------------------------------------------
void assign_command(void)
{
    cmd = CMD_STOP;

    if (sw1)        { cmd = CMD_PATH1;  return; }
    if (sw2)        { cmd = CMD_PATH2;  return; }
    if (sw3)        { cmd = CMD_PATH3;  return; }
    if (cruise)     { cmd = CMD_CRUISE; return; }

    if (press)
    {
        cmd = CMD_ROTATE;
        return;
    }

    if      (joy_y >= HIGH_THRESHOLD && joy_x >= HIGH_THRESHOLD) cmd = CMD_SLIGHT_RIGHT;
    else if (joy_y >= HIGH_THRESHOLD && joy_x <= LOW_THRESHOLD)  cmd = CMD_SLIGHT_LEFT;
    else if (joy_y <= LOW_THRESHOLD  && joy_x >= HIGH_THRESHOLD) cmd = CMD_SLIGHT_BACK_RIGHT;
    else if (joy_y <= LOW_THRESHOLD  && joy_x <= LOW_THRESHOLD)  cmd = CMD_SLIGHT_BACK_LEFT;
    else if (joy_y >= HIGH_THRESHOLD)                            cmd = CMD_FORWARD;
    else if (joy_y <= LOW_THRESHOLD)                             cmd = CMD_BACKWARD;
    else if (joy_x >= HIGH_THRESHOLD)                            cmd = CMD_RIGHT;
    else if (joy_x <= LOW_THRESHOLD)                             cmd = CMD_LEFT;
    else                                                         cmd = CMD_STOP;
}

// ---------------------------------------------------------------------------
// IR send: just send cmd number of pulses, then long gap
// Each pulse = 5ms carrier ON, 5ms OFF
// End of message = 50ms silence
// ---------------------------------------------------------------------------
void send_command(int cmd_num)
{
    int i;

    for (i = 0; i < cmd_num; i++)
    {
        carrier_on();
        delayms(5);
        carrier_off();
        delayms(5);
    }

    // Long gap marks end of message
    delayms(50);
}

const char* cmd_to_string(int c)
{
    switch (c)
    {
        case CMD_STOP:              return "STOP";
        case CMD_FORWARD:           return "FORWARD";
        case CMD_BACKWARD:          return "BACKWARD";
        case CMD_LEFT:              return "LEFT";
        case CMD_RIGHT:             return "RIGHT";
        case CMD_ROTATE:            return "ROTATE";
        case CMD_SLIGHT_RIGHT:      return "SLIGHT_RIGHT";
        case CMD_SLIGHT_LEFT:       return "SLIGHT_LEFT";
        case CMD_SLIGHT_BACK_RIGHT: return "SLIGHT_BACK_RIGHT";
        case CMD_SLIGHT_BACK_LEFT:  return "SLIGHT_BACK_LEFT";
        case CMD_PATH1:             return "PATH1";
        case CMD_PATH2:             return "PATH2";
        case CMD_PATH3:             return "PATH3";
        case CMD_CRUISE:            return "CRUISE";
        default:                    return "UNKNOWN";
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
void main(void)
{
    int last_cmd = -1;

    GPIO_Init();
    initADC();
    UART1_Init();
    setbuf(stdout, NULL);
    IR_PWM_Init();

    printf("\r\nSTM32 IR TRANSMITTER READY\r\n");
    printf("Protocol: pulse count, 5ms on/off, 50ms gap\r\n");

    while (1)
    {
        read_path_control();
        read_joystick();
        assign_command();

        // Only print on change to avoid UART spam
        if (cmd != last_cmd)
        {
            printf("TX: %s (%d pulses)\r\n", cmd_to_string(cmd), cmd);
            last_cmd = cmd;
        }

        send_command(cmd);
        // No extra delay needed - the 50ms gap + pulse time is enough
    }
}
