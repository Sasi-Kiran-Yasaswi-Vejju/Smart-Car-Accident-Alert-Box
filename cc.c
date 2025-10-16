/*

  STM32F401CCU6 Integrated System

  - HC-SR04: TRIG = PA1, ECHO = PA2

  - HC-05 (Bluetooth): PA9 (TX), PA10 (RX)

  - MPU6050 (I2C1): PB6 (SCL), PB7 (SDA)

  - Sound sensor (analog): PA4

  - MQ-2 Engine Failure (digital input): PA6

  - Buzzer: PB0 (active-high)

  - Onboard LED: PC13 (active-low)

  - Uses SysTick (1 ms)

*/



#include "stm32f4xx.h"

#include <stdio.h>

#include <stdint.h>

#include <math.h>

#include <stdlib.h>

#ifndef M_PI

#define M_PI 3.14159265358979323846

#endif



volatile uint32_t ticks = 0;



/* ---------- SysTick ---------- */

void SysTick_Handler(void){ ticks++; }

void delay_ms(uint32_t ms){ uint32_t s = ticks; while((ticks - s) < ms); }

void SysTick_enable(void){

    SysTick->LOAD = (SystemCoreClock / 1000U) - 1U; // 1 ms tick

    SysTick->VAL  = 0;

    SysTick->CTRL = 7; // enable, tickint, use processor clock

}



/* ---------- UART (USART1) ---------- */

static void usart1_set_baud(uint32_t baud){

    uint32_t pclk = SystemCoreClock;

    float usartdiv = (float)pclk / (16.0f * (float)baud);

    uint32_t mantissa = (uint32_t)usartdiv;

    uint32_t fraction = (uint32_t)((usartdiv - (float)mantissa) * 16.0f + 0.5f);

    if(fraction >= 16){ mantissa += 1; fraction = 0; }

    USART1->BRR = (mantissa << 4) | (fraction & 0xF);

}



void usart1_init(uint32_t baud){

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;



    GPIOA->MODER &= ~((3U << (9*2)) | (3U << (10*2)));

    GPIOA->MODER |=  (2U << (9*2)) | (2U << (10*2));

    GPIOA->AFR[1] &= ~((0xFU<<((9-8)*4)) | (0xFU<<((10-8)*4)));

    GPIOA->AFR[1] |=  (7U<<((9-8)*4)) | (7U<<((10-8)*4));



    usart1_set_baud(baud);

    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

}



void send_bt(const char *s){

    while(*s){

        while(!(USART1->SR & USART_SR_TXE));

        USART1->DR = *s++;

    }

}



/* ---------- I2C1 (MPU6050) ---------- */

static uint32_t get_pclk1_freq_hz(void){

    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7U;

    uint32_t prescaler = 1;

    if ((ppre1 & 0x4U) == 0) prescaler = 1;

    else {

        switch(ppre1){

            case 0x4: prescaler = 2; break;

            case 0x5: prescaler = 4; break;

            case 0x6: prescaler = 8; break;

            case 0x7: prescaler = 16; break;

            default: prescaler = 1; break;

        }

    }

    return SystemCoreClock / prescaler;

}



void i2c1_init(void){

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;



    GPIOB->MODER &= ~((3U<<(6*2))|(3U<<(7*2)));

    GPIOB->MODER |=  (2U<<(6*2))|(2U<<(7*2));

    GPIOB->OTYPER |= (1U<<6)|(1U<<7);

    GPIOB->PUPDR &= ~((3U<<(6*2))|(3U<<(7*2)));

    GPIOB->PUPDR |=  (1U<<(6*2))|(1U<<(7*2));

    GPIOB->AFR[0] &= ~((0xFU<<(6*4))|(0xFU<<(7*4)));

    GPIOB->AFR[0] |= (4U<<(6*4)) | (4U<<(7*4));



    uint32_t pclk1 = get_pclk1_freq_hz();

    uint32_t pclk1_mhz = (pclk1 / 1000000U);

    if(pclk1_mhz < 2) pclk1_mhz = 2;

    I2C1->CR2 = (pclk1_mhz & 0x3F);

    uint32_t ccr = pclk1 / (2U * 100000U);

    if(ccr < 1) ccr = 1;

    if(ccr > 0xFFF) ccr = 0xFFF;

    I2C1->CCR = ccr & 0xFFF;

    I2C1->TRISE = pclk1_mhz + 1U;

    I2C1->CR1 |= I2C_CR1_ACK | I2C_CR1_PE;

}



void i2c1_write_reg(uint8_t addr, uint8_t reg, uint8_t data){

    while(I2C1->SR2 & I2C_SR2_BUSY);

    I2C1->CR1 |= I2C_CR1_START;

    while(!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = addr & ~1U;

    while(!(I2C1->SR1 & I2C_SR1_ADDR));

    (void)I2C1->SR2;

    while(!(I2C1->SR1 & I2C_SR1_TXE));

    I2C1->DR = reg;

    while(!(I2C1->SR1 & I2C_SR1_TXE));

    I2C1->DR = data;

    while(!(I2C1->SR1 & I2C_SR1_BTF));

    I2C1->CR1 |= I2C_CR1_STOP;

}



uint8_t i2c1_read_reg(uint8_t addr, uint8_t reg){

    uint8_t val = 0;

    while(I2C1->SR2 & I2C_SR2_BUSY);

    I2C1->CR1 |= I2C_CR1_START;

    while(!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = addr & ~1U;

    while(!(I2C1->SR1 & I2C_SR1_ADDR));

    (void)I2C1->SR2;

    while(!(I2C1->SR1 & I2C_SR1_TXE));

    I2C1->DR = reg;

    while(!(I2C1->SR1 & I2C_SR1_BTF));

    I2C1->CR1 |= I2C_CR1_START;

    while(!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = addr | 1U;

    while(!(I2C1->SR1 & I2C_SR1_ADDR));

    I2C1->CR1 &= ~I2C_CR1_ACK;

    (void)I2C1->SR2;

    I2C1->CR1 |= I2C_CR1_STOP;

    while(!(I2C1->SR1 & I2C_SR1_RXNE));

    val = I2C1->DR;

    I2C1->CR1 |= I2C_CR1_ACK;

    return val;

}



/* ---------- MPU6050 helpers ---------- */

#define MPU_ADDR (0x68 << 1)

void mpu6050_wake(void){ i2c1_write_reg(MPU_ADDR, 0x6B, 0x00); }

void mpu6050_read_accel(float *Ax, float *Ay, float *Az){

    uint8_t d[6];

    for(int i=0;i<6;i++) d[i] = i2c1_read_reg(MPU_ADDR, 0x3B + i);

    int16_t ax = (d[0]<<8) | d[1];

    int16_t ay = (d[2]<<8) | d[3];

    int16_t az = (d[4]<<8) | d[5];

    *Ax = ax / 16384.0f; *Ay = ay / 16384.0f; *Az = az / 16384.0f;

}



/* ---------- GPIO init ---------- */

void gpio_init(void){

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;



    // PA1 TRIG output, PA2 ECHO input

    GPIOA->MODER &= ~((3U << (1*2)) | (3U << (2*2)));

    GPIOA->MODER |=  (1U << (1*2));



    // PA4 analog for sound

    GPIOA->MODER |= (3U << (4*2));



    // PA6 digital input for MQ-2 engine failure detection

    GPIOA->MODER &= ~(3U << (6*2)); // input mode

    GPIOA->PUPDR &= ~(3U << (6*2));

    GPIOA->PUPDR |=  (1U << (6*2)); // pull-up



    // PB0 Buzzer output

    GPIOB->MODER &= ~(3U << (0*2)); GPIOB->MODER |= (1U << (0*2)); GPIOB->ODR &= ~(1U<<0);



    // PC13 LED output

    GPIOC->MODER &= ~(3U << (13*2)); GPIOC->MODER |= (1U << (13*2)); GPIOC->ODR |= (1U<<13);

}



/* ---------- ADC1 init ---------- */

void adc1_init(void){

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    ADC1->SQR3 = 4; // channel 4 (PA4)

    ADC1->SMPR2 |= (7U << (4*3)); // sample time

    ADC1->CR2 |= ADC_CR2_ADON;

}



uint16_t read_adc(void){

    ADC1->CR2 |= ADC_CR2_SWSTART;

    while(!(ADC1->SR & ADC_SR_EOC));

    return ADC1->DR;

}



/* ---------- Ultrasonic ---------- */

uint32_t measure_distance(void){

    GPIOA->ODR &= ~(1U<<1);

    delay_ms(2);

    GPIOA->ODR |= (1U<<1);

    for(volatile int i=0;i<200;i++);

    GPIOA->ODR &= ~(1U<<1);



    uint32_t t0 = ticks;

    while( (GPIOA->IDR & (1U<<2)) == 0 ){

        if((ticks - t0) > 30) return 0xFFFFFFFFU;

    }



    uint32_t count = 0;

    uint32_t tstart = ticks;

    while(GPIOA->IDR & (1U<<2)){

        count++;

        if((ticks - tstart) > 30) break;

    }

    return (count / 58U);

}



/* ---------- Main ---------- */

int main(void){

    SysTick_enable();

    gpio_init();

    adc1_init();

    usart1_init(9600);

    i2c1_init();



    mpu6050_wake();

    delay_ms(50);



    char buf[80];

    uint8_t who = i2c1_read_reg(MPU_ADDR, 0x75);

    sprintf(buf, "MPU WHO_AM_I=0x%02X\r\n", who);


    send_bt(buf);

    send_bt("System Ready: ultrasonic + MPU6050 + sound sensor (40dB) + HC-05 + MQ-2 engine failure\r\n");



    while(1){

        // Ultrasonic detection

        uint32_t dist = measure_distance();

        if(dist != 0xFFFFFFFFU){

            sprintf(buf,"Distance: %lu cm\r\n",dist);

            send_bt(buf);

            if(dist <= 20U) { GPIOB->ODR |= (1U<<0); send_bt("Object detected!\r\n"); }

            else GPIOB->ODR &= ~(1U<<0);

        }



        // MPU6050 tilt

        float Ax, Ay, Az;

        mpu6050_read_accel(&Ax,&Ay,&Az);

        float pitch = atan2f(Ax, sqrtf(Ay*Ay + Az*Az)) * 180.0f/M_PI;

        float roll  = atan2f(Ay, sqrtf(Ax*Ax + Az*Az)) * 180.0f/M_PI;

        float g_total = sqrtf(Ax*Ax + Ay*Ay + Az*Az);



        if(pitch > 45.0f || pitch < -45.0f || roll > 45.0f || roll < -45.0f || g_total > 2.5f){

            send_bt("Accident detected! Reply within 15s if OK.\r\n");

            uint32_t start = ticks; uint8_t responded = 0;

            while((ticks-start)<15000U){

                if(USART1->SR & USART_SR_RXNE){ (void)USART1->DR; responded=1; send_bt("Response received. Cancelled.\r\n"); break; }

            }

            if(!responded){ GPIOB->ODR |= (1U<<0); GPIOC->ODR &= ~(1U<<13); delay_ms(15000); GPIOB->ODR &= ~(1U<<0); GPIOC->ODR |= (1U<<13); send_bt("No response -> alarm ON (MPU).\r\n"); }

        }



        // Sound sensor detection (~40dB)

        uint16_t adc_val = read_adc();

        if(adc_val > 300){ // adjust this after calibration for 40dB

            send_bt("Loud crash sound detected! Reply within 15s if OK.\r\n");

            uint32_t start = ticks; uint8_t responded = 0;

            while((ticks-start)<15000U){

                if(USART1->SR & USART_SR_RXNE){ (void)USART1->DR; responded=1; send_bt("Response received. Cancelled.\r\n"); break; }

            }

            if(!responded){ GPIOB->ODR |= (1U<<0); GPIOC->ODR &= ~(1U<<13); send_bt("No response -> alarm ON (sound).\r\n"); delay_ms(15000); GPIOB->ODR &= ~(1U<<0); GPIOC->ODR |= (1U<<13); }

        }



        // MQ-2 Engine failure detection (PA6, digital input)

        if((GPIOA->IDR & (1U<<6)) == 0){ // active-low detection

            send_bt("Engine failure detected! Buzzer ON immediately!\r\n");

            GPIOB->ODR |= (1U<<0);  // buzzer ON

            GPIOC->ODR &= ~(1U<<13); // LED ON

        } else {

            GPIOB->ODR &= ~(1U<<0);  // buzzer OFF

            GPIOC->ODR |= (1U<<13);  // LED OFF

        }



        delay_ms(800);

    }

}

