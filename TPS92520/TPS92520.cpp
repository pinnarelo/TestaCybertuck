/***************************************************************************//**
*   @file    TPS92520.c
*   @brief   TPS92520 implementation file.
*   @devices TPS92520-Q1
*
********************************************************************************
* Copyright 2018(c) Automotive Lighting
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Automotive Lighting nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Automotive Lighting component.
*
* THIS SOFTWARE IS PROVIDED BY AUTOMOTIVE LIGHTING "AS IS" AND ANY EXPRESS OR 
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, 
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL AUTOMOTIVE LIGHTING BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "mbed.h"
#include "TPS92520.h"

#include <stdint.h>
#include <iostream>
#include <string.h>
#include <string>
#include <cstring>



/* Error codes */
#define INVALID_VAL -1 /* Invalid argument */
#define COMM_ERR    -2 /* Communication error on receive */
#define TIMEOUT     -3 /* A timeout has occured */

/* SPI CLK Frequency */
#if not definated
    #define SPI_CLK 100000
#endif

// Current sense resistor value [mOhm]
#define R_sense 100

TPS92520::TPS92520(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName pwr_en) : my_spi(mosi, miso, sclk), my_pin( cs), pwr_en_pin(pwr_en) { }

TPS92520::~TPS92520() { }


/***************************************************************************//**
* @brief Reads the junction temperature 
*
* @return pointer to string message.
*******************************************************************************/
char* TPS92520::temp(void) {
    int ret;
    int low,high = 0;
    char* buffer = new char[22];

    
    /* TEMPL Register */                        
    ret = readRegs(TEMPL,&low);
    if(ret<0) {
        sprintf(buffer, "COMM ERROR\n\r");
        return buffer; }
    low = low & 0x0003;
    
    /* TEMPH Register */                        
    ret = readRegs(TEMPH,&high);
    if(ret<0) {
        sprintf(buffer, "COMM ERROR\n\r");
        return buffer; }
    high = high & 0x00FF;
    
    high = high << 2;
    high = high + low;
    
    float temp = 0.7168*high-274.51;
    int tempd = (int)temp;

    sprintf(buffer, "Temp Junction: %dC\n\r", tempd);

    char dest[50] = "Learning C++ is fun";
    char src[50] = " and easy";

    strcat(dest, src);
        
    return buffer;
}

/***************************************************************************//**
* @brief Reads value from all status registers and create message with all 
*       errors 
*
* @return pointer to string message.
*******************************************************************************/
char* TPS92520::status(void) {
    int ret;
    int raw_data,c = 0;
    char* buffer = new char[200];
    char* bufferLF = new char[4];

    sprintf(bufferLF, "\n\r");

    
    const char *reports1[8] = {"CH2LSILIM", "CH2HSILIM", "CH2SHORT", "CH2COMPOV",
                             "CH1LSILIM", "CH1HSILIM", "CH1SHORT", "CH1COMPOV"};
                             
    const char *reports2[6] = {"CH2TP", "CH2BSTUV", "CH2TOFFMIN",
                            "CH1TP", "CH1BSTUV", "CH1TOFFMIN"};
    
    const char *reports3[7] = {"V5AUV", "Watchdog Expired", "Watchdog Expired",
                            "TW", "PC", "CH2STATUS","CH1STATUS"};
    
    readRegs(STATUS3,&raw_data);
    
    /* STATUS 1 Register */                        
    ret = readRegs(STATUS1,&raw_data);
    if(ret<0) {
        sprintf(buffer, "COMM ERROR\n\r");
        return buffer; }
    raw_data = raw_data & 0x00FF;
    for(int i = 0; i<8; i++) {
        c = raw_data & 1 <<i;
        if(c>0) {
            strcat(buffer, reports1[7-i]);
            strcat(buffer, bufferLF);
            }
        }
    
    /* STATUS 2 Register */                        
    ret = readRegs(STATUS2,&raw_data);
    if(ret<0) {
        sprintf(buffer, "COMM ERROR\n\r");
        return buffer; }
    raw_data = raw_data & 0x00FF;
    for(int i = 0; i<6; i++) {
        c = raw_data & 1 <<i;
        if(c>0) {
            strcat(buffer, reports2[5-i]);
            strcat(buffer, bufferLF);
            }
        }
        
    /* STATUS 3 Register */                        
    ret = readRegs(STATUS3,&raw_data);
    if(ret<0) {
        sprintf(buffer, "COMM ERROR\n\r");
        return buffer; }
    raw_data = raw_data & 0x00FF;
    for(int i = 0; i<7; i++) {
        c = raw_data & 1 <<i;
        if(c>0) {
            strcat(buffer, reports3[6-i]);
            strcat(buffer, bufferLF);
            }
        }
    
    if(buffer[0] == '\0') sprintf(buffer, "OK\r\n");
        
    return buffer;
}

/***************************************************************************//**
* @brief Init SPI periphery 
*
*******************************************************************************/
void TPS92520::init(void) {
    // Setup the spi for 16 bit data, Mode 0 CLK polarity 0, Phase 0
    my_spi.format(16,0);
    my_spi.frequency(SPI_CLK);
    // Enable 5VA PWR supply
    pwr_en_pin = 1;
    
    int data; 
    /* Check STATUS3 Register */                        
    readRegs(STATUS3,&data);
    
    
}


/***************************************************************************//**
* @brief Read register value.
*
* @param addr  - The address of the register.
* @param pData - Pointer to the data which hold readed register data
*
* @return Returns 0 for success or negative error code. 
*
*******************************************************************************/
int TPS92520::readRegs(int addr, int *pData) {
    uint8_t CMD = 0;
    uint16_t frame = 0;
    
    frame = CMD<<15 | addr<<9 | 0x00;
    frame = frame | parity(frame)<<8;
    
    /* First Frame */
    my_pin = 0;
    my_spi.write(frame);
    my_pin = 1;
    wait_us(10);
    
    /* Second Frame */
    my_pin = 0;
    uint16_t rx_buffer = my_spi.write(frame);
    my_pin = 1;
    
    /* Response is valid in the second frame @ Datasheet */
    *pData = rx_buffer;
    
    if((rx_buffer & SPI_ERROR) == TRUE)
    {
        return COMM_ERR;
    }
    else if((rx_buffer & 0x7C00)>>10 != 24)
    {
        return COMM_ERR;
    }
    else
    {
        return 0;
    }

}

/***************************************************************************//**
* @brief Write register value.
*
* @param addr  - The address of the register.
* @param data  - The data to be writen.
* @param response - Pointer to the data which hold writen register data
*
* @return Returns 0 for success or negative error code. 
*
*******************************************************************************/    
int TPS92520::writeRegs(int addr, int data, int *response) {
    uint8_t CMD = 1;
    uint16_t frame = 0;
    
    frame = CMD<<15 | addr<<9 | data;
    frame = frame | parity(frame)<<8;
    
    /* First Frame */
    my_pin = 0;
    my_spi.write(frame);
    my_pin = 1;
    wait_us(10);
    
    /* Second Frame */
    my_pin = 0;
    uint16_t rx_buffer = my_spi.write(frame);
    my_pin = 1;
    
    /* Response is valid in the second frame @ Datasheet */
    *response = rx_buffer;
    
    if((rx_buffer & SPI_ERROR) == TRUE) return COMM_ERR;
    
    if((rx_buffer & 0x00FF) != data) return COMM_ERR;
    
    if((rx_buffer & 0x3F00)>>8 != addr) return COMM_ERR;
      
    return 0;

    }

/***************************************************************************//**
* @brief Reset device. This function clear all writeable register to their 
*                   default value.
*
* @return Returns 0. 
*
*******************************************************************************/     
int TPS92520::reset() {
    int data;
  
    // Disable 5VA PWR supply
    pwr_en_pin = 0;
    //wait_ms(350);
    
    // Enable 5VA PWR supply
    pwr_en_pin = 1;
    
    //wait_ms(50);
    /* Check STATUS3 Register */                        
    readRegs(STATUS3,&data);

    return 0;   
    }    

/***************************************************************************//**
* @brief Calculate parity bit.
*
* @param frame  - The frame for which we want to calculate parity bit.
*
* @return Returns parity bit value. 
*
*******************************************************************************/  
int TPS92520::parity(uint16_t frame) {
    uint8_t XNOR = 0;
    
    for(int i = 15; i >= 0; i--){
        if((frame & (1 << i)) != 0){
            XNOR = !(XNOR^1);
            if(i == 15){
                XNOR = 1; }
            }
        else{
            XNOR = !(XNOR^0);
            if(i == 15){
                XNOR = 0; }
           } 
        }
    return XNOR;
    }

/***************************************************************************//**
* @brief Set the PWM duty.
*
* @param 
*
* @return  
*
*******************************************************************************/     
int TPS92520::setPWMduty(uint8_t channel,uint16_t duty) {
    uint8_t CMD = 1;
    
    if(duty> 1023 == 1) return INVALID_VAL;
    
    uint16_t data = (uint16_t)duty/100*1023;
    uint16_t frame = 0;
    
    // First SPI frame
    uint8_t addr = CH1PWMH;
    if (channel == 2) addr +=2; 
    
    frame = CMD<<15 | addr<<9 | data&0x0300>>8;
    frame = frame | parity(frame)<<8;
    
    my_pin = 0;
    uint16_t rx_buffer = my_spi.write(frame);
    my_pin = 1;
    
    // Second SPI frame
    addr = CH1PWML;
    if (channel == 2) addr +=2; 
    
    frame = CMD<<15 | addr<<9 | data&0x00FF;
    frame = frame | parity(frame)<<8;

    my_pin = 0;
    rx_buffer = my_spi.write(frame);
    my_pin = 1;
        
    return 0;

    }
    
/***************************************************************************//**
* @brief Set the output current.
*   Rsense has same value for both channels 
*   need to definate Rsense value before use @ main definition
*
* @param channel - Number of the channel
* @param current_mA - Output current in mA  
*
* @return Returns pointer to string message. 
*
*******************************************************************************/     
char* TPS92520::setCurrent(uint8_t channel,uint16_t current_mA) {
    char* buffer = new char[20];
    
    float ad = current_mA*14*1024/2.48/1000*R_sense/1000;
    uint16_t CHxIADJ = ceil(ad);
     
    if(CHxIADJ>1023){
        sprintf(buffer, "INVALID VALUE\n\r");
        return buffer; }
        
    int ret, response; 
    
    // First SPI frame
    uint8_t addr = CH1IADJL;
    if (channel == 2) addr +=2; 
    
    /* CHxIADJL Register */  
    ret = writeRegs(addr, CHxIADJ&0x0003, &response);
    if(ret<0) {
        sprintf(buffer, "COMM ERROR\n\r");
        return buffer; }
        
    addr +=1; 
       
    /* CHxIADJH Register */  
    ret = writeRegs(addr, (CHxIADJ >> 2)&0x00FF, &response);
    if(ret<0) {
        sprintf(buffer, "COMM ERROR\n\r");
        return buffer; }
    //sprintf(buffer, "OK A: %d,H: %d, L: %d\n\r",CHxIADJ, (CHxIADJ >> 2)&0x00FF, CHxIADJ&0x0003);  
    return buffer;
} 
    
/***************************************************************************//**
* @brief Set the PWM duty cycle of the output current.
* 
* @param channel - Number of the channel
* @param duty_percentage - duty_percentage 0 - 100 % 
*
* @return Returns pointer to string message. 
*
*******************************************************************************/     
char* TPS92520::setPWMDuty(uint8_t channel,uint16_t duty_percentage) {
    char* buffer = new char[20];
    
    float ad = 10.23*duty_percentage;
    uint16_t CHxPWM = ceil(ad);
     
    if(CHxPWM>1023){
        sprintf(buffer, "INVALID VALUE\n\r");
        return buffer; }
        
    int ret, response, response2, data; 
    
    uint8_t addr = CH1PWMH;
    if (channel == 2) addr +=2;
    
    /* CHxPWMH Register */  
    ret = writeRegs(addr, (CHxPWM >> 8)&0x0003, &response);
    if(ret<0) {
        sprintf(buffer, "COMM ERROR\n\r");
        return buffer; } 
        
    addr -=1;
    
    /* CHxPWML Register */  
    ret = writeRegs(addr, CHxPWM&0x00FF, &response);
    if(ret<0) {
        sprintf(buffer, "COMM ERROR\n\r");
        return buffer; }
        
    ret = readRegs(SYSCFG1,&response);
    if(ret<0) {
        sprintf(buffer, "COMM ERROR\n\r");
        return buffer; }
        
    if(channel == 1) {
        response &=0x00FD;
        data = response+2; }
    else {
        response &=0x00F7;
        data = response+8; }
        
    //wait_ms(10);
     
    ret = writeRegs(SYSCFG1, data, &response2);
    if(ret<0) {
        sprintf(buffer, "COMM ERROR\n\r");
        return buffer; }
    
    sprintf(buffer, "OK\n\r");  
    return buffer;

    }
    
/***************************************************************************//**
* @brief Set the switching frequency.
*
* @param channel - Number of the channel
* @param fsw_kHz - switching frequency [kHz] set in range 100 - 2400 kHz
*
* @return Returns pointer to string message. 
*
*******************************************************************************/     
char* TPS92520::setFreq(uint8_t channel,uint16_t fsw_kHz) {
    char* buffer = new char[20];
    
    float ad = (uint16_t)fsw_kHz/50;
    uint16_t CHxTON = ceil(ad) - 1;
     
    if(CHxTON>48){
        sprintf(buffer, "INVALID VALUE\n\r");
        return buffer; }
        
    int ret, response; 
    
    uint8_t addr = CH1TON;
    if (channel == 2) addr +=1;
    
    /* CHxTON Register */  
    ret = writeRegs(addr, CHxTON&0x003F, &response);
    if(ret<0) {
        sprintf(buffer, "COMM ERROR\n\r");
        return buffer; } 
        
    ad = (CHxTON+1)/(.02);
    CHxTON = ceil(ad);
    sprintf(buffer, "OK Fsw: %d kHz\n\r", CHxTON);  
    return buffer;

}
    
/***************************************************************************//**
* @brief Set the switching frequency.
*
* @param channel - Number of the channel
* @param clk_div - coresponding Datasheet value
*                   0 @ 1395 Hz
*                   1 @ 1221 Hz
*                   2 @ 977 Hz
*                   -
*                   5   407 Hz
*                   6 @ 199 Hz
*   `               7 @ 100 Hz
*
* @return Returns pointer to string message. 
*
*******************************************************************************/     
char* TPS92520::setPWMClkDiv(uint16_t clk_div) {
    char* buffer = new char[20];
     
    if(clk_div>7){
        sprintf(buffer, "INVALID VALUE\n\r");
        return buffer; }
        
    int ret, response; 
    
    uint8_t addr = PWMDIV;
    
    /* PWMDIV Register */  
    ret = writeRegs(addr, clk_div&0x0007, &response);
    if(ret<0) {
        sprintf(buffer, "COMM ERROR\n\r");
        return buffer; } 
        
    sprintf(buffer, "OK\n\r");  
    return buffer;

}