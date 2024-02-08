/***************************************************************************//**
*   @file    TPS92520.h
*   @brief   TPS92520 header file.
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

#ifndef __TPS92520_H__
#define __TPS92520_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "mbed.h"
#include <stdint.h>
#include <iostream>
#include <string.h>


/** TPS92520 class
 *
 *  TPS92520: A library
 *
 *  TPS92520 is an LED Driver
 *  @endcode
 */
 
class TPS92520
{
public:

    /** Create a TPS92520 instance
     *  which is connected to specified SPI pins
     *
     * @param mosi SPI MOSI pin
     * @param miso SPI MISO pin
     * @param sclk SPI SCLK pin
     */
    TPS92520(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName pwr_en);

    /** Destructor of BME280
     */
    virtual ~TPS92520();

    void init(void);
    
    int readRegs(int addr, int *pData);
    
    int writeRegs(int addr, int data, int *response);
    
    int reset(void);
    
    int setPWMduty(uint8_t channel,uint16_t duty);
    
    char* setCurrent(uint8_t channel, uint16_t current_mA);
    
    char* status(void);
    
    char* temp(void);
    
    char* setPWMDuty(uint8_t channel,uint16_t duty_percentage);
    
    char* setFreq(uint8_t channel,uint16_t fsw_kHz);
    
    char* setPWMClkDiv(uint16_t clk_div);

private:

    SPI         my_spi;
    DigitalOut  my_pin;
    DigitalOut  pwr_en_pin;
    
    int parity(uint16_t frame);
};

/* TPS92520-Q1 Register Map */
#define SYSCFG1     0x00
#define SYSCFG2     0x01
#define CMWTAP      0x02
#define STATUS1     0x03
#define STATUS2     0x04
#define STATUS3     0x05
#define TWLMT       0x06
#define SLEEP       0x07
#define CH1IADJL    0x08
#define CH1IADJH    0x09
#define CH2IADJL    0x0A
#define CH2IADJH    0x0B
#define PWMDIV      0x0C
#define CH1PWML     0x0D
#define CH1PWMH     0x0E
#define CH2PWML     0x0F
#define CH2PWMH     0x10
#define CH1TON      0x11
#define CH2TON      0x12
#define CH1VIN      0x13
#define CH1VLED     0x14
#define CH1VLEDON   0x15
#define CH1VLEDOFF  0x16
#define CH2VIN      0x17
#define CH2VLED     0x18
#define CH2VLEDON   0x19
#define CH2VLEDOFF  0x1A
#define TEMPL       0x1B
#define TEMPH       0x1C
#define V5D         0x1D
#define LHCFG1      0x1E
#define LHCFG2      0x1F
#define LHIL        0x20
#define LHIH        0x21
#define LH1IADJL    0x22
#define LH1IADJH    0x23
#define LH2IADJL    0x24
#define LH2IADJH    0x25
#define LHCH1PWML   0x26
#define LHCH1PWMH   0x27
#define LHCH2PWML   0x28
#define LHCH2PWMH   0x29
#define LH1TON      0x2A
#define LH2TON      0x2B
#define RESET       0x2C 

/* System Config Register bits */
#define FPINSRT    (0 << 7)

/* Status Register bits */


#define SPI_ERROR     (1 << 15)

#define TRUE 1
#define FALSE 0


#endif /* __TPS92520_H__ */