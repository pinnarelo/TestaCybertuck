#include "mbed.h"
#include "iostream"
#include "stdio.h"
#include "string"
#include "algorithm"

#include "cstring"
#include <string.h>
#include <cstring>

#include "TPS92664.h"
using namespace  std;


//#define DEBUG
#include "hl_debug.h"

BufferedSerial UARToverCAN(PA_15, PB_7);

Mutex               mutex2;
ConditionVariable   cond2(mutex2);
Thread              thread10;
Thread              UARTprint_thread;
EventQueue          eventQueue2;


void UARTPrint(void);

// Circular buffer pointers 
// volatile makes read-modify-write atomic 
volatile int rx_in;
volatile int rx_out;

char rx_buffer[256];

// Line buffers for sprintf and sscanf
char rx_line[40];

char                recvBuff2[BUFF_LEN] = { 0 };
size_t              recvLen2;
char                message2[MSG_LEN] = { 0 };

// The following variable is protected by locking the mutex
char                data2[DATA_LEN] = { 0 };






/***************************************************************************//**
* @brief Reads value from all status registers and create message with all 
*       errors 
*
* @return pointer to string message.
*******************************************************************************/
char* TPS92664_Get_Status(tps92664_device *device) {

    char* buffer = new char[200];
    char* bufferLF = new char[4];

     if(!device) {
        sprintf(buffer, "COMM ERROR\n\r");
        return buffer;
     }
    

    tps92664_st_reg tps92664_regs[] =
    {
        {0x7A, 0x85, 0x00, 0xFF, 1}, /* STATUS */
    };
    
    int ret = TPS92664_SingleDeviceRead(device, &tps92664_regs[0], 1);
    
    if (ret < 0) {
        sprintf(buffer, "COMM ERROR\n\r");
        return buffer;
     }

    

    int raw_data,c = 0;

    sprintf(bufferLF, "\r\n");

    
    const char *reports1[8] = {"PWM_ERR", "CHPMP_ERR", "VLED_ERR", "LIMP_HOME",
                             "TRIM_ERR", "TW", "MTP_ERR", "PWR"};
                             

    raw_data = device->regs[TPS_STATUS].value;
    

    raw_data = raw_data & 0x00FF;
    for(int i = 0; i<8; i++) {
        c = raw_data & 1 <<i;
        if(c>0) {
            strcat(buffer, reports1[7-i]);
            strcat(buffer, bufferLF);
        }
    }
    
    if(buffer[0] == '\0') {
        sprintf(buffer, "OK\r\n");
    }
    else {
        tps92664_st_reg *reg;
        reg = device->regs;
        reg[TPS_STATUS].value = 0;
        ret = TPS92664_SingleDeviceWriteRegs(device, &reg[TPS_STATUS], 1);
    }
        
    return buffer;
}



/***************************************************************************//**
* @brief Calculate Phase and Width values for requested Duty
*       Use only if PSON bit is set 
*
* @param pLeds          - The handler of the instance of the Leds structure.
* @param Duty           - The LEDs duty cycle.
* @param blank_time     - The offset for switch-ON/OFF.
*******************************************************************************/    
int TPS92664_Calculate_Phase_Width(tps92664_leds *pLeds, float Duty[16], int blank_time)
{
   // INFO("TPS92664_Calculate_Phase_Width function.\r");
    int ret = 0;
    for(int a = 0; a<16; a++)
    {
        pLeds[a].width = uint16_t(0);
        pLeds[a].phase = uint16_t(0);
    }
    
    int N_segments = 16;
    
   // INFO("Segments: %d\r\n", N_segments);
    
    // Calculate LEDs width
    
    for(int i = 0;i<N_segments;i++){
      //      INFO("Duty: %d\r", Duty[i]);
            //pLeds[i].width = (round(Duty[i]/100*TPS92664_REG_SIZE));
            
            if (Duty[i] != 0) {
                pLeds[i].width = uint16_t(Duty[i]*1024/100);
            }
            else
            {
                pLeds[i].width = 0;
            }
            
            if (pLeds[i].width>1023){
                pLeds[i].width = 1023;}
            
            INFO("my_leds[%d].width: %d\r", i, pLeds[i].width);

    }
    
    // Calculate LEDs phase

    int start = 0;
    for(int i = 0;i<N_segments;i++){
        pLeds[i].phase = start;
        if (pLeds[i].width < 1023 && pLeds[i].width > 0) 
        {
            start = pLeds[i].phase+1023-pLeds[i].width+blank_time;
            if (start > 1023) {
                start = start-1023; }
        }
        INFO("Start: %d\r", start);
        INFO("my_leds[%d].phase: %d\r", i, pLeds[i].phase);
        
    }
    
    
    return ret;
}


int TPS92664_SetLeds(tps92664_device *device, tps92664_leds *pLeds, uint8_t nLEDs)
{
    if(!device)
        return INVALID_VAL;
    
    tps92664_st_reg *reg;
    reg = device->regs;
    
    reg[WIDTH01H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH04_01L].value = reg[WIDTH04_01L].value & 0xFC | (pLeds->width & 0x03);
    reg[PHASE01H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE04_01L].value = reg[PHASE04_01L].value & 0xFC | (pLeds->phase & 0x03);
    pLeds++;
	reg[WIDTH02H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH04_01L].value = reg[WIDTH04_01L].value & 0xF3 | ((pLeds->width & 0x03) << 2);
    reg[PHASE02H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE04_01L].value = reg[PHASE04_01L].value & 0xF3 | ((pLeds->phase & 0x03) << 2);
    pLeds++;
	reg[WIDTH03H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH04_01L].value = reg[WIDTH04_01L].value & 0xCF | ((pLeds->width & 0x03) << 4);
    reg[PHASE03H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE04_01L].value = reg[PHASE04_01L].value & 0xCF | ((pLeds->phase & 0x03) << 4);
    pLeds++;
	reg[WIDTH04H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH04_01L].value = reg[WIDTH04_01L].value & 0x3F | ((pLeds->width & 0x03) << 6);
    reg[PHASE04H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE04_01L].value = reg[PHASE04_01L].value & 0x3F | ((pLeds->phase & 0x03) << 6);
    pLeds++;

	reg[WIDTH05H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH08_05L].value = reg[WIDTH08_05L].value & 0xFC | (pLeds->width & 0x03);
    reg[PHASE05H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE08_05L].value = reg[PHASE08_05L].value & 0xFC | (pLeds->phase & 0x03);
    pLeds++;
	reg[WIDTH06H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH08_05L].value = reg[WIDTH08_05L].value & 0xF3 | ((pLeds->width & 0x03) << 2);
    reg[PHASE06H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE08_05L].value = reg[PHASE08_05L].value & 0xF3 | ((pLeds->phase & 0x03) << 2);
    pLeds++;
	reg[WIDTH07H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH08_05L].value = reg[WIDTH08_05L].value & 0xCF | ((pLeds->width & 0x03) << 4);
    reg[PHASE07H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE08_05L].value = reg[PHASE08_05L].value & 0xCF | ((pLeds->phase & 0x03) << 4);
    pLeds++;
	reg[WIDTH08H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH08_05L].value = reg[WIDTH08_05L].value & 0x3F | ((pLeds->width & 0x03) << 6);
    reg[PHASE08H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE08_05L].value = reg[PHASE08_05L].value & 0x3F | ((pLeds->phase & 0x03) << 6);
    pLeds++;

	reg[WIDTH09H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH12_09L].value = reg[WIDTH12_09L].value & 0xFC | (pLeds->width & 0x03);
    reg[PHASE09H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE12_09L].value = reg[PHASE12_09L].value & 0xFC | (pLeds->phase & 0x03);
    pLeds++;
	reg[WIDTH10H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH12_09L].value = reg[WIDTH12_09L].value & 0xF3 | ((pLeds->width & 0x03) << 2);
    reg[PHASE10H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE12_09L].value = reg[PHASE12_09L].value & 0xF3 | ((pLeds->phase & 0x03) << 2);
    pLeds++;
	reg[WIDTH11H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH12_09L].value = reg[WIDTH12_09L].value & 0xCF | ((pLeds->width & 0x03) << 4);
    reg[PHASE11H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE12_09L].value = reg[PHASE12_09L].value & 0xCF | ((pLeds->phase & 0x03) << 4);
    pLeds++;
	reg[WIDTH12H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH12_09L].value = reg[WIDTH12_09L].value & 0x3F | ((pLeds->width & 0x03) << 6);
    reg[PHASE12H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE12_09L].value = reg[PHASE12_09L].value & 0x3F | ((pLeds->phase & 0x03) << 6);
    pLeds++;

	reg[WIDTH13H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH16_13L].value = reg[WIDTH16_13L].value & 0xFC | (pLeds->width & 0x03);
    reg[PHASE13H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE16_13L].value = reg[PHASE16_13L].value & 0xFC | (pLeds->phase & 0x03);
    pLeds++;
	reg[WIDTH14H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH16_13L].value = reg[WIDTH16_13L].value & 0xF3 | ((pLeds->width & 0x03) << 2);
    reg[PHASE14H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE16_13L].value = reg[PHASE16_13L].value & 0xF3 | ((pLeds->phase & 0x03) << 2);
    pLeds++;
	reg[WIDTH15H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH16_13L].value = reg[WIDTH16_13L].value & 0xCF | ((pLeds->width & 0x03) << 4);
    reg[PHASE15H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE16_13L].value = reg[PHASE16_13L].value & 0xCF | ((pLeds->phase & 0x03) << 4);
    pLeds++;
	reg[WIDTH16H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH16_13L].value = reg[WIDTH16_13L].value & 0x3F | ((pLeds->width & 0x03) << 6);
    reg[PHASE16H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE16_13L].value = reg[PHASE16_13L].value & 0x3F | ((pLeds->phase & 0x03) << 6); 


    int ret = TPS92664_SingleDeviceWriteRegs(device, &reg[PHASE01H], 20);
    if (ret<0)
       return ret;
        
    ret = TPS92664_SingleDeviceWriteRegs(device, &reg[WIDTH01H], 20);
    if (ret<0)
        return ret;

    return ret;
}


int TPS92664_BroadcastSetLeds(tps92664_device *device, tps92664_leds *pLeds, uint8_t nLEDs)
{
  //  INFO("TPS92664_BroadcastSetLeds function.\n");
    
    tps92664_st_reg *reg;
    reg = device->regs;
    
    reg[WIDTH01H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH04_01L].value = reg[WIDTH04_01L].value & 0xFC | (pLeds->width & 0x03);
    reg[PHASE01H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE04_01L].value = reg[PHASE04_01L].value & 0xFC | (pLeds->phase & 0x03);
    pLeds++;
	reg[WIDTH02H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH04_01L].value = reg[WIDTH04_01L].value & 0xF3 | ((pLeds->width & 0x03) << 2);
    reg[PHASE02H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE04_01L].value = reg[PHASE04_01L].value & 0xF3 | ((pLeds->phase & 0x03) << 2);
    pLeds++;
	reg[WIDTH03H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH04_01L].value = reg[WIDTH04_01L].value & 0xCF | ((pLeds->width & 0x03) << 4);
    reg[PHASE03H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE04_01L].value = reg[PHASE04_01L].value & 0xCF | ((pLeds->phase & 0x03) << 4);
    pLeds++;
	reg[WIDTH04H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH04_01L].value = reg[WIDTH04_01L].value & 0x3F | ((pLeds->width & 0x03) << 6);
    reg[PHASE04H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE04_01L].value = reg[PHASE04_01L].value & 0x3F | ((pLeds->phase & 0x03) << 6);
    pLeds++;

	reg[WIDTH05H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH08_05L].value = reg[WIDTH08_05L].value & 0xFC | (pLeds->width & 0x03);
    reg[PHASE05H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE08_05L].value = reg[PHASE08_05L].value & 0xFC | (pLeds->phase & 0x03);
    pLeds++;
	reg[WIDTH06H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH08_05L].value = reg[WIDTH08_05L].value & 0xF3 | ((pLeds->width & 0x03) << 2);
    reg[PHASE06H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE08_05L].value = reg[PHASE08_05L].value & 0xF3 | ((pLeds->phase & 0x03) << 2);
    pLeds++;
	reg[WIDTH07H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH08_05L].value = reg[WIDTH08_05L].value & 0xCF | ((pLeds->width & 0x03) << 4);
    reg[PHASE07H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE08_05L].value = reg[PHASE08_05L].value & 0xCF | ((pLeds->phase & 0x03) << 4);
    pLeds++;
	reg[WIDTH08H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH08_05L].value = reg[WIDTH08_05L].value & 0x3F | ((pLeds->width & 0x03) << 6);
    reg[PHASE08H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE08_05L].value = reg[PHASE08_05L].value & 0x3F | ((pLeds->phase & 0x03) << 6);
    pLeds++;

	reg[WIDTH09H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH12_09L].value = reg[WIDTH12_09L].value & 0xFC | (pLeds->width & 0x03);
    reg[PHASE09H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE12_09L].value = reg[PHASE12_09L].value & 0xFC | (pLeds->phase & 0x03);
    pLeds++;
	reg[WIDTH10H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH12_09L].value = reg[WIDTH12_09L].value & 0xF3 | ((pLeds->width & 0x03) << 2);
    reg[PHASE10H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE12_09L].value = reg[PHASE12_09L].value & 0xF3 | ((pLeds->phase & 0x03) << 2);
    pLeds++;
	reg[WIDTH11H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH12_09L].value = reg[WIDTH12_09L].value & 0xCF | ((pLeds->width & 0x03) << 4);
    reg[PHASE11H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE12_09L].value = reg[PHASE12_09L].value & 0xCF | ((pLeds->phase & 0x03) << 4);
    pLeds++;
	reg[WIDTH12H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH12_09L].value = reg[WIDTH12_09L].value & 0x3F | ((pLeds->width & 0x03) << 6);
    reg[PHASE12H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE12_09L].value = reg[PHASE12_09L].value & 0x3F | ((pLeds->phase & 0x03) << 6);
    pLeds++;

	reg[WIDTH13H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH16_13L].value = reg[WIDTH16_13L].value & 0xFC | (pLeds->width & 0x03);
    reg[PHASE13H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE16_13L].value = reg[PHASE16_13L].value & 0xFC | (pLeds->phase & 0x03);
    pLeds++;
	reg[WIDTH14H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH16_13L].value = reg[WIDTH16_13L].value & 0xF3 | ((pLeds->width & 0x03) << 2);
    reg[PHASE14H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE16_13L].value = reg[PHASE16_13L].value & 0xF3 | ((pLeds->phase & 0x03) << 2);
    pLeds++;
	reg[WIDTH15H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH16_13L].value = reg[WIDTH16_13L].value & 0xCF | ((pLeds->width & 0x03) << 4);
    reg[PHASE15H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE16_13L].value = reg[PHASE16_13L].value & 0xCF | ((pLeds->phase & 0x03) << 4);
    pLeds++;
	reg[WIDTH16H].value = ((pLeds->width & 0x3FC) >> 2);  
    reg[WIDTH16_13L].value = reg[WIDTH16_13L].value & 0x3F | ((pLeds->width & 0x03) << 6);
    reg[PHASE16H].value = ((pLeds->phase & 0x3FC) >> 2);  
    reg[PHASE16_13L].value = reg[PHASE16_13L].value & 0x3F | ((pLeds->phase & 0x03) << 6); 
 
    int ret = TPS92664_BroadcastWriteRegs(&reg[PHASE01H], 20);
    if (ret<0)
        return ret;
        
    ret = TPS92664_BroadcastWriteRegs(&reg[WIDTH01H], 20);
    if (ret<0)
        return ret;

    return ret;
}


/***************************************************************************//**
* @brief Set SlewRate for all switches
*
* @param device     - The handler of the instance of the driver.
* @param slewrate   - The Slew rate option.
*******************************************************************************/    
int TPS92664_Set_SlewRate(tps92664_device *device, enum TPS92664_SlewRate_t slewrate)
{
    if(!device)
        return INVALID_VAL;
    
    tps92664_st_reg *reg;
    
    reg = device->regs;
    
    reg[SLEWL].value = TPS92664_SLEW_RATE_MODE(slewrate);   
    reg[SLEWH].value = TPS92664_SLEW_RATE_MODE(slewrate); 
    
    //int ret = TPS92664_SingleDeviceWriteRegs(device, &reg[SLEWL], 2);
    int ret = TPS92664_BroadcastWriteRegs(&reg[SLEWL], 2);
    
    return ret;
}   

/***************************************************************************//**
* @brief Set CS Gain 
*
* @param device     - The handler of the instance of the driver.
* @param csgain     - The Current Sense gain rate option.
*******************************************************************************/    
int TPS92664_Set_CSGain(tps92664_device *device, enum TPS92664_CSGain_t csgain)
{
    INFO("TPS92664_Set_CSGain function\n");  
    if(!device)
        return INVALID_VAL;
    
    tps92664_st_reg *reg;
    
    reg = device->regs;
    INFO("TPS_SYSCFG register value: %d\n", reg[TPS_SYSCFG].value);  
    INFO("TPS92664_CS_GAIN(csgain): %d\n", TPS92664_CS_GAIN(csgain));  
    INFO("reg[TPS_SYSCFG].value & 0xFB: %d\n", reg[TPS_SYSCFG].value & 0xFB); 
    reg[TPS_SYSCFG].value = reg[TPS_SYSCFG].value & 0xFB | TPS92664_CS_GAIN(csgain);
    INFO("TPS_SYSCFG register value: %d\n", reg[TPS_SYSCFG].value);   
    
    int ret = TPS92664_SingleDeviceWriteRegs(device, &reg[TPS_SYSCFG], 1);
    
    return ret;
}


/***************************************************************************//**
* @brief Set LED ADC Enable 
*
* @param device     - The handler of the instance of the driver.
* @param adcen      - The ADC enable option.
*******************************************************************************/    
int TPS92664_Set_ADCmeasurement(tps92664_device *device, enum TPS92664_ADCEN_t adcen)
{
    INFO("TPS92664_Set_ADCmeasurement function\n");  
    if(!device)
        return INVALID_VAL;
    
    tps92664_st_reg *reg;
    
    reg = device->regs;
    INFO("TPS_SYSCFG register value: %d\n", reg[TPS_SYSCFG].value);  
    INFO("TPS92664_ADCEN(adcen): %d\n", TPS92664_ADCEN(adcen));  
    INFO("reg[TPS_SYSCFG].value & 0xBF: %d\n", reg[TPS_SYSCFG].value & 0xBF); 
    reg[TPS_SYSCFG].value = reg[TPS_SYSCFG].value & 0xBF | TPS92664_ADCEN(adcen);
    INFO("TPS_SYSCFG register value: %d\n", reg[TPS_SYSCFG].value);   
    
    int ret = TPS92664_SingleDeviceWriteRegs(device, &reg[TPS_SYSCFG], 1);
    
    return ret;
}


/***************************************************************************//**
* @brief Get ADC Values
*
* @param device     - The handler of the instance of the driver.
*
* @return - return ADC value or negative failure code.
*******************************************************************************/    
int TPS92664_Get_ADC(tps92664_device *device, enum TPS92664_ADC_t channel)
{
    if(!device)
        return INVALID_VAL;
    
    
    tps92664_st_reg tps92664_regs[] =
    {
        {0x82, 0x8D, 0x00, 0xFF, 2}, /* ADC1 */
        {0x83, 0x8E, 0x00, 0xFF, 2}, /* ADC2 */
    };
    
    int32_t ret = TPS92664_SingleDeviceRead(device, &tps92664_regs[0+channel], 1);
    
    if (ret < 0) {
        return ret;
    } 
    
    return device->regs[TPS_ADC1+channel].value;
} 


/***************************************************************************//**
* @brief Init Serial periphery 
*
* @param device     - The handler of the instance of the driver.
* @param regs       - The handler of the registers.
* @return - return 0.
*******************************************************************************/
int32_t TPS92664_init(tps92664_device *device, tps92664_st_reg *regs) 
{

    thread10.start(callback(&eventQueue2, &EventQueue::dispatch_forever));
    UARTprint_thread.start(UARTPrint);
    // Setup the Serial Port
    UARToverCAN.set_baud(1000000);

    UARToverCAN.sigio(callback(onSigioUARToverCAN));
    
    
    rx_in=0;
    rx_out=0;
    

    if(!device || !regs)
        return -1;
    
    device->regs = regs;
    
    
    
    return 0;
}


/***************************************************************************//**
* @brief Calculate parity bytes CRCL and CRCH.
*
* @param *buf  - Pointer to data buffer.
*
* @return CRC16 result. 
*
*******************************************************************************/  
unsigned TPS92664_crc16(char *buf, size_t len)
{
    unsigned crc = 0;
    for (int j = 0; j < len; j++)
    {
        char b = buf[j];
        for (int i = 0; i < 8; i++)
        {
            crc = ((b ^ (char)crc) & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);
            b >>= 1;
        }
    }
    return crc;
}



/***************************************************************************//**
* @brief reset the device UART and protocol state machine. 
* Causes the TPS92664 devices on the network to reset
* communications reset does not reset the registers and does not halt normal LED PWM operation.
*******************************************************************************/    
int32_t TPS92664_communicationReset(void)
{
    // Application buffer to send the data
    char buf[13] = {0};
    
    char reset = 0x00;
  
    UARToverCAN.write (buf, 13); // holding the RX input low for
                                // a period of at least 16 * 11 (=176) system clock periods
    
    rx_in=0;
    rx_out=0;
    
    return 0;
}


/***************************************************************************//**
* @brief Write registers. This write data to the registers multiple byte
*
* Alwais use ACK otherwise function not work properly
* @param device  - The handler of the instance of the driver.
* @param addr    - The handler of the instance of the registers.
* @param nBytes  - The number of bytes to be read. Allowable values are
*  1,2,3,4,12,16,32
*
* @return Returns 0 for success or negative error code. 
*
*******************************************************************************/    
int32_t TPS92664_SingleDeviceRead(tps92664_device *device, tps92664_st_reg* pReg, uint8_t nBytes)
{
    //INFO("TPS92664_SingleDeviceRead function\n");  
    char *pPacket;
    char cmd;
    switch(nBytes)
    {
        case 1:
            cmd = RD_1Bite;
            break;
        case 2:
            cmd = RD_2Bite;
            break;
        case 3:
            cmd = RD_3Bite;
            break;
        case 4:
            cmd = RD_4Bite;
            break;
        case 5:
            cmd = RD_5Bite;
            break;
        case 8:
            cmd = RD_8Bite;
            break;
        case 12:
            cmd = RD_12Bite;
            break;
        case 16:
            cmd = RD_16Bite;
            break;
        case 20:
            cmd = RD_20Bite;
            break;
        case 32:
            cmd = RD_32Bite;
            break;
        default:
            return NULL;
    }

    char packet[5] = {cmd, char(device->device_id)};
    packet[2] = pReg->address;
   
   // INFO("packet[0]: %d\n", packet[0]);  
    
   // INFO("packet[1]: %d\n", packet[1]);
    //INFO("device->device_id: %d\n", device->device_id);  
   
  //  INFO("packet[2]: %d\n", packet[2]); 
    //INFO("pReg->address: %d\n", pReg->address);   
    uint16_t crc = TPS92664_crc16(packet, 3);
    packet[3] = crc &0xFF;
    packet[4] = (crc &0xFF00)>>8;
    
 //   INFO("packet[3]: %d\n", packet[3]);  
     
 //   INFO("packet[4]: %d\n", packet[4]); 
    
    UARToverCAN.write (packet, 5);
    

    ThisThread::sleep_for(5ms);
    
    int index_data = rx_in-2-nBytes;

    int index_crc = rx_in-2;

    if (index_data < 0) {
        index_data = buffer_size + index_data;
    }
    if (index_crc < 0) {
        index_crc = buffer_size + index_crc;
    }

    char rx_data[nBytes];
    for(int b=0; b<nBytes; b++){
        rx_data[b] = rx_buffer[(index_data+b)%buffer_size];
      //  INFO("rx_data[%d]: %d\n", b, rx_data[b]);
    }
    
    
    pPacket = &rx_data[0];
    crc = TPS92664_crc16(pPacket, nBytes);
 
    // check validity of received data 
    if( (crc&0xFF) == rx_buffer[(index_crc)%buffer_size] && 
        ((crc&0xFF00)>>8) == rx_buffer[(index_crc+1)%buffer_size]) {
        
        // store data if valid 
        for(int i = 0;i<nBytes;i++){

            //device->regs[pReg->positionIndex+i].value = rx_buffer[(index_data+i)%buffer_size];
            device->regs[pReg->positionIndex].value = rx_buffer[(index_data+i)%buffer_size];
            //INFO("device->regs %d\n", device->regs[pReg->positionIndex+i].value);
            pReg->value = rx_buffer[(index_data+i)%buffer_size];
            pReg++;
        }
        return 0;
    }
    else {
        //INFO("CRC: FAULT\n");
        return COMM_ERR;
    }
}


/***************************************************************************//**
* @brief Write registers. This write data to the registers multiple byte
*
* Alwais use ACK otherwise function not work properly
* @param device     - The handler of the instance of the driver.
* @param addr    - The address of the first register.
* @param nBytes  - The number of bytes to be written. Allowable values are
*  1,2,3,4,12,16,32
* @param pData   - Pointer to the data buffer
*
* @return Returns 0 for success or negative error code. 
*
*******************************************************************************/    
int TPS92664_SingleDeviceWriteRegs(tps92664_device *device, tps92664_st_reg *pReg, uint8_t nBytes)
{
    char cmd;
    char packet[37];
    switch(nBytes)
    {
        case 1:
            cmd = WR_1Bite;
            break;
        case 2:
            cmd = WR_2Bite;
            break;
        case 3:
            cmd = WR_3Bite;
            break;
        case 4:
            cmd = WR_4Bite;
            break;
        case 5:
            cmd = WR_5Bite;
            break;
        case 8:
            cmd = WR_8Bite;
            break;
        case 12:
            cmd = WR_12Bite;
            break;
        case 16:
            cmd = WR_16Bite;
            break;
        case 20:
            cmd = WR_20Bite;
            break;
        case 32:
            cmd = WR_32Bite;
            break;
        default:
            return INVALID_VAL;
    }
    
     
    packet[0] = cmd;
    packet[1] = char(device->device_id);
    packet[2] = pReg->address;
    int firstAddress = pReg->address;


    INFO("packet[0]: %d\n", packet[0]);
    INFO("packet[1]: %d\n", packet[1]);
    INFO("packet[2]: %d\n", packet[2]);

    /*
    for(int i=3;i<3+nBytes;i++){
        
        if( pReg->value > device->regs[pReg->positionIndex].mask){
            packet[i] = pReg->value & device->regs[pReg->positionIndex].mask;
            
        }
        else {
            packet[i] = pReg->value;
        } 
        
        pReg++;
    } 
    */
    for(int i=3;i<3+nBytes;i++)
    {
        INFO("pReg->value: %d\n", pReg->value);
        packet[i] = pReg->value;
        INFO("packet[%d]: %d\n", i, packet[i]);
        pReg++;
    }
    // restore address of fist data
    for(int i=0;i<nBytes;i++)
    {
        pReg--;
    }

    uint16_t crc = TPS92664_crc16(packet, 3+nBytes);
    packet[3+nBytes] = crc &0xFF;
    packet[4+nBytes] = (crc &0xFF00)>>8;

    
    int ret = UARToverCAN.write (packet, 5+nBytes);
    if (ret<0)
    {
        return ret;
    }

    ThisThread::sleep_for(1ms);
    
    int index_data = rx_in-1-nBytes;
    int index_ack = rx_in-1;


    if(device->useACK == true) {
        
        if(rx_buffer[index_ack%buffer_size] != 0x7F)
        {
            return COMM_ERR;    
        }
        else 
        {/*
            for(int i = 0;i<nBytes;i++)
            {
               //  Store data 
              //  INFO("DATA WRITE OK\n");
                INFO("device->regs->address %d\n", pReg->address);
                INFO("device->regs->value %d\n", pReg->value);
                //device->regs[pReg->positionIndex+i].value = packet[3+i];
                device->regs[pReg->address+i].value = packet[3+i];
                pReg++;
            }
            */
        }
        
    }
    return 0;
}




/***************************************************************************//**
* @brief TPS92664_BroadcastWriteRegs
*
* @param pReg    - Register structure holding info about the registers to be written
* @param nBytes  - The number of bytes to be written. Allowable values are
*  1,2,3,4,5,8,12,16,20,32
*
* @return Returns 0 for success or negative error code. 
*
*******************************************************************************/    
int TPS92664_BroadcastWriteRegs(tps92664_st_reg *pReg, uint8_t nBytes)
{
    char cmd;
    char packet[37];
    switch(nBytes)
    {
        case 1:
            cmd = WR_1Bite;
            break;
        case 2:
            cmd = WR_2Bite;
            break;
        case 3:
            cmd = WR_3Bite;
            break;
        case 4:
            cmd = WR_4Bite;
            break;
        case 5:
            cmd = WR_5Bite;
            break;
        case 8:
            cmd = WR_8Bite;
            break;
        case 12:
            cmd = WR_12Bite;
            break;
        case 16:
            cmd = WR_16Bite;
            break;
        case 20:
            cmd = WR_20Bite;
            break;
        case 32:
            cmd = WR_32Bite;
            break;
        default:
            return INVALID_VAL;
    }
     
    packet[0] = cmd;
    packet[1] = Broadcast_Write_CMD;
    packet[2] = pReg->address;
    
    for(int i=3;i<3+nBytes;i++)
    {
        packet[i] = pReg->value;
        pReg++;
    }
    
    uint16_t crc = TPS92664_crc16(packet, 3+nBytes);
    packet[3+nBytes] = crc &0xFF;
    packet[4+nBytes] = (crc &0xFF00)>>8;

    return UARToverCAN.write (packet, 5+nBytes);
}

void UARTPrint(void)
{
    mutex2.lock();

    while (1) {
        // Wait for a condition to change
        cond2.wait();

        // Now it is safe to access data in this thread
        //printf("Data length received: %d\r\n", rx_in);
    }
}


void TPS92664_Lmm_IRQ(void)
{
    while (UARToverCAN.readable()) {
        // Read UARToverCAN
        //recvLen2 = UARToverCAN.read(recvBuff2, BUFF_LEN);
        recvLen2 = UARToverCAN.read(rx_buffer, BUFF_LEN);
        rx_in = recvLen2;
        
        mutex2.lock();

        // Signal for other threads that data has been received
        cond2.notify_all();
        mutex2.unlock();
    }
}

void onSigioUARToverCAN(void)
{
    eventQueue2.call(TPS92664_Lmm_IRQ);
}