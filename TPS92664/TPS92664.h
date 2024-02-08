/***************************************************************************//**
*   @file    TPS92664.h
*   @brief   TPS92664 header file.
*   @devices TPS92664-Q1
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

#ifndef __TPS92664_H__
#define __TPS92664_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "mbed.h"
#include <stdint.h>


#define BUFF_LEN    32
#define MSG_LEN     64
#define DATA_LEN    MSG_LEN - 2


#ifdef DEBUG_TPS92664
#define DEBUG_TPS92664(...) printf(__VA_ARGS__)
#else
#define DEBUG_TPS92664(...)
#endif

#define TPS92664_USE_ACK 1
#define TPS92664_DISABLE_ACK 0

#define TPS92664_RW 1   /* Read and Write */
#define TPS92664_R  2   /* Read only */
#define TPS92664_W  3   /* Write only */

#ifndef TRUE
    #define TRUE 1
#endif
#ifndef FALSE
    #define FALSE 0
#endif


/* TPS92664-Q1 Macros */
#define TPS92664_SLEW_RATE_MODE(x)          (((x) & 0x07) << 3) | ((x) & 0x07)
#define TPS92664_CS_GAIN(x)                 ((x) << 2)
#define TPS92664_ADCEN(x)                   ((x) << 6)
#define TPS92664_OVERVOLTAGE_MODE(x)        ((((x) & 0x03) << 6) | (((x) & 0x03) << 4)| (((x) & 0x03) << 2) | ((x) & 0x03))
#define TPS92664_REG_WIDTH 10;
#define TPS92664_REG_SIZE 1024;
#define TPS92664_WCNT 1023;


/* Polynomic coeficient CRC */
#define POLY    0xA001 // CRC-16-MAXIM (IBM) (or 0xA001)
#define TPS92664_INIT   /* We want to use the TPS92664_regs array defined in TPS92664.h */


/* Error codes */
#define INVALID_VAL -1 /* Invalid argument */
#define COMM_ERR    -2 /* Communication error on receive */
#define TIMEOUT     -3 /* A timeout has occured */

#define DEVID_Byte          0x08
//#define DEVID_Byte          0x61
#define Broadcast_Write_CMD 0xBF
//#define Broadcast_Write_CMD 0x1F - do not work - DO NOT USE

/* TPS92664-Q1 Frame Initialization Byte*/
#define WR_1Bite            0x87
#define WR_2Bite            0x99
#define WR_3Bite            0x1E
#define WR_4Bite            0xAA
#define WR_5Bite            0xAD
#define WR_8Bite            0xB5
#define WR_12Bite           0x2D
#define WR_16Bite           0x33
#define WR_20Bite           0xB5
#define WR_32Bite           0xB4
#define RD_1Bite            0x4B
#define RD_2Bite            0xCC
#define RD_3Bite            0xD2
#define RD_4Bite            0x55
#define RD_5Bite            0xE5
#define RD_8Bite            0x26
#define RD_12Bite           0xE1
#define RD_16Bite           0x66
#define RD_20Bite           0x7A
#define RD_32Bite           0x78


// Circular buffer pointers 
// volatile makes read-modify-write atomic 
extern volatile int rx_in;
extern volatile int rx_out;

extern char rx_buffer[256];


extern char rx_line[40];        // Line buffers for sprintf and sscanf





/*! Device register info */

typedef struct 
{   
    uint8_t positionIndex;
    uint8_t address;
    uint8_t value;
    uint8_t mask;
    uint8_t rw;
}tps92664_st_reg;


typedef struct
{   
    uint16_t width;
    uint16_t phase;
}tps92664_leds;


enum TPS92664_ADC_t
{
    ADC_CH1 = 0,               
    ADC_CH2 = 1
};

enum TPS92664_SlewRate_t
{
    SLOWEST_RATE = 0,    
    SLOW_RATE = 1,           
    MEDIUM_RATE = 2,
    FAST_RATE = 3,
    FASTEST_RATE = 4
};

enum TPS92664_CSGain_t
{
    x1_GAIN = 0,               
    x10_GAIN = 1
};

enum TPS92664_ADCEN_t
{
    LED_ADC_DISABLE = 0,               
    LED_ADC_ENABLE = 1
};

enum TPS92664_Overvoltage_t
{
    V_TH_OPEN1 = 0,             //6V typ
    V_TH_OPEN2 = 1,             //12V typ
    V_TH_OPEN3_DEFAULT  = 2,    //18V typ
    V_TH_OPEN3 = 3              //18V typ
};


/*! TPS92664 Phase-Shift and Width Registers*/

typedef enum tps92664RegisterName
{
	MTPCFG = 0x00,
    OUTCTRL,
    TWLMT,
    SLEWL,
    SLEWH,
	DEFWIDTH01,
	DEFWIDTH02,
	DEFWIDTH03,
	DEFWIDTH04,
	DEFWIDTH05,
	DEFWIDTH06,
	DEFWIDTH07,
	DEFWIDTH08,
	DEFWIDTH09,
	DEFWIDTH10,
	DEFWIDTH11,
	DEFWIDTH12,
	DEFWIDTH13,
	DEFWIDTH14,
	DEFWIDTH15,
	DEFWIDTH16,
	PHASE01H,
	PHASE02H,
	PHASE03H,
	PHASE04H,
	PHASE05H,
	PHASE06H,
	PHASE07H,
	PHASE08H,
	PHASE09H,
	PHASE10H,
	PHASE11H,
	PHASE12H,
	PHASE13H,
	PHASE14H,
	PHASE15H,
	PHASE16H,
	PHASE04_01L,
	PHASE08_05L,
	PHASE12_09L,
	PHASE16_13L,
    WIDTH01H,
	WIDTH02H,
	WIDTH03H,
	WIDTH04H,
	WIDTH05H,
	WIDTH06H,
	WIDTH07H,
	WIDTH08H,
	WIDTH09H,
	WIDTH10H,
	WIDTH11H,
	WIDTH12H,
	WIDTH13H,
	WIDTH14H,
	WIDTH15H,
	WIDTH16H,
	WIDTH04_01L,
	WIDTH08_05L,
	WIDTH12_09L,
	WIDTH16_13L,
	BANK_PHASE01L,
	BANK_PHASE02L,
	BANK_PHASE03L,
	BANK_PHASE04L,
	BANK_PHASE04_01H,
	BANK_PHASE05L,
	BANK_PHASE06L,
	BANK_PHASE07L,
	BANK_PHASE08L,
	BANK_PHASE08_05H,
	BANK_PHASE09L,
	BANK_PHASE10L,
	BANK_PHASE11L,
	BANK_PHASE12L,
	BANK_PHASE12_09H,
	BANK_PHASE13L,
	BANK_PHASE14L,
	BANK_PHASE15L,
	BANK_PHASE16L,
	BANK_PHASE16_13H,
	BANK_WIDTH01L,
	BANK_WIDTH02L,
	BANK_WIDTH03L,
	BANK_WIDTH04L,
	BANK_WIDTH04_01H,
	BANK_WIDTH05L,
	BANK_WIDTH06L,
	BANK_WIDTH07L,
	BANK_WIDTH08L,
	BANK_WIDTH08_05H,
	BANK_WIDTH09L,
	BANK_WIDTH10L,
	BANK_WIDTH11L,
	BANK_WIDTH12L,
	BANK_WIDTH12_09H,
	BANK_WIDTH13L,
	BANK_WIDTH14L,
	BANK_WIDTH15L,
	BANK_WIDTH16L,
	BANK_WIDTH16_13H,
	LEDONTH01,
	LEDONTH02,
	LEDONTH03,
	LEDONTH04,
	LEDONTH05,
	LEDONTH06,
	LEDONTH07,
	LEDONTH08,
	LEDONTH09,
	LEDONTH10,
	LEDONTH11,
	LEDONTH12,
	LEDONTH13,
	LEDONTH14,
	LEDONTH15,
	LEDONTH16,
    TPS_SYSCFG,
    TPS_CMWTAP,
    TPS_PWMTICK,
    TPS_ADCIN,          
    TPS_CLK_SYNC,
    TPS_STATUS,        
    FLT_OPEN_OR_DRVL,
    FLT_OPEN_OR_DRVH,
    FAULT_SHORTL,
    FAULT_SHORTH,        
    FAULT_RESFETL,
    FAULT_RESFETH,
    CERRCNT,
    TPS_ADC1,
    TPS_ADC2,
    TPS_DIETEMP,
	TPS_CS,
	VLEDON01,
	VLEDON02,
	VLEDON03,
	VLEDON04,
	VLEDON05,
	VLEDON06,
	VLEDON07,
	VLEDON08,
	VLEDON09,
	VLEDON10,
	VLEDON11,
	VLEDON12,
	VLEDON13,
	VLEDON14,
	VLEDON15,
	VLEDON16,
	VLEDOFF01,
	VLEDOFF02,
	VLEDOFF03,
	VLEDOFF04,
	VLEDOFF05,
	VLEDOFF06,
	VLEDOFF07,
	VLEDOFF08,
	VLEDOFF09,
	VLEDOFF10,
	VLEDOFF11,
	VLEDOFF12,
	VLEDOFF13,
	VLEDOFF14,
	VLEDOFF15,
	VLEDOFF16,
	PWM_MISCOUNTL,
	PWM_MISCOUNTH,
	LOTINFO1,
	LOTINFO2,
	LOTINFO3,
	LOTINFO4,
	MTP_PROG1,
	MTP_PROG2,
	MTP_PROG3,
	MTP_PROG4,
    TPS_ICID
} tps92664RegisterName;





/*
 * The structure describes the device and is used with the TPS92664 driver.
 * @slave_select_id: The ID of the Slave Select to be passed to the UART calls.
 * @regs: A reference to the register list of the device that the user must
 *       provide when calling the Setup() function.
 * @userCRC: Whether to do or not a cyclic redundancy check on UART transfers.
 * @check_ready: When enabled all register read and write calls will first wait
 *               until the device is ready to accept user requests.
 * @spi_rdy_poll_cnt: Number of times the driver should read the Error register
 *                    to check if the device is ready to accept user requests,
 *                    before a timeout error will be issued.
 */
typedef struct 
{
    int device_id;
    tps92664_st_reg *regs;
    bool useACK;
    tps92664_leds *leds;
} tps92664_device;



// Circular buffers for serial TX and RX data - used by interrupt routines
const int buffer_size = 255;
// might need to increase buffer size for high baud rates

char* TPS92664_Get_Status(tps92664_device *device);

int TPS92664_BroadcastSetLeds(tps92664_device *device, tps92664_leds *pLeds, uint8_t nLEDs);

int TPS92664_SetLeds(tps92664_device *device, tps92664_leds *pLeds, uint8_t nLEDs);

int TPS92664_Calculate_Phase_Width(tps92664_leds *pLeds, float Duty[16], int blank_time);


/***************************************************************************//**
* @brief Get ADC Values
*
* @param device     - The handler of the instance of the driver.
*
* @return - return ADC value or negative failure code.
*******************************************************************************/    
int TPS92664_Get_ADC(tps92664_device *device, enum TPS92664_ADC_t channel);



/***************************************************************************//**
* @brief TPS92664_TPS92664_SingleDeviceRead
*
* @param device  - The handler of the instance of the driver.
* @param pReg    - The handler of the instance of the registers.
* @param nBytes  - The number of bytes to be read. Allowable values are
* 1, 2, 3, 4, 5, 8, 12, 16, 20 or 32 bytes
*
* @return Returns 0 for success or negative error code. 
*
*******************************************************************************/
int32_t TPS92664_SingleDeviceRead(tps92664_device *device, tps92664_st_reg* pReg, uint8_t nBytes);


int32_t TPS92664_init(tps92664_device *device, tps92664_st_reg *regs);


int TPS92664_Set_SlewRate(tps92664_device *device, enum TPS92664_SlewRate_t slewrate);
int TPS92664_Set_CSGain(tps92664_device *device, enum TPS92664_CSGain_t csgain);
int TPS92664_Set_ADCmeasurement(tps92664_device *device, enum TPS92664_ADCEN_t adcen);




int TPS92664_BroadcastWriteRegs(tps92664_st_reg *pReg, uint8_t nBytes);
int TPS92664_SingleDeviceWriteRegs(tps92664_device *device, tps92664_st_reg *pReg, uint8_t nBytes);

void TPS92664_Lmm_IRQ(void);
unsigned TPS92664_crc16(char *buf, size_t len);

/***************************************************************************//**
* @brief reset the device UART and protocol state machine. 
* Causes the TPS92664 devices on the network to reset
* communications reset does not reset the registers and does not halt normal LED PWM operation.
*******************************************************************************/    
int32_t TPS92664_communicationReset(void);

void onSigioUARToverCAN(void);


#endif /* __TPS92664_H__*/