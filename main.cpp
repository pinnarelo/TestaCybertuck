#include "mbed.h"
#include <cstdint>
#include <iostream>
#include "PCBA.h"


#define BUFF_LEN    32
#define MSG_LEN     64
#define DATA_LEN    MSG_LEN - 2

#define T1_LAMP
//#define T2_LAMP


InterruptIn button(BUTTON1);

DigitalOut          led2(LED1);
BufferedSerial      serial(USBTX, USBRX);
Thread              thread1;
Thread              thread2;
Thread              thread3;
Thread              thread4;
Thread              thread5;
Thread              LMMStatusThread;
Thread              threadButton;
Thread              LMMMInit;
EventQueue          VirtualSerialeventQueue;
EventQueue          CANeventQueue;
Mutex               mutex;
Mutex               CANmutex;
ConditionVariable   cond(mutex);
ConditionVariable   Serialcond(CANmutex);
char                recvBuff[BUFF_LEN] = { 0 };
size_t              recvLen;
char                message[MSG_LEN] = { 0 };

// The following variable is protected by locking the mutex
char                data[DATA_LEN] = { 0 };


// Circular buffers for serial TX and RX data - used by interrupt routines
const int serial_buffer_size = 255;    
    
// Circular buffer pointers 
// volatile makes read-modify-write atomic 
volatile int serial_rx_in;
volatile int serial_rx_out;

volatile bool LMMStatusrunning = true;
volatile bool PWRONresetrunning = true;
    
// Line buffers for sprintf and sscanf
char serial_rx_line[40];
    
// might need to increase buffer size for high baud rates
char serial_rx_buffer[serial_buffer_size+1];

volatile int LMMMODE=0;

int hex2int(char ch);
void taskPrintData(void);
void printmenu(void);
void onSigio(void);
void AllLightOff(void);
void TailLightOn(void);
void TailLightOff(void);
void StopLightOn(void);
void StopLightOff(void);

void T1stopturnLightOn(void);
void T1stopturnLightOff(void);
void T2stopLightOn(void);
void T2stopLightOff(void);
void buttonIRQrise(void);
void T1tailLightOn(void);
void T1tailLightOff(void);
void T2tailLightOn(void);
void T2tailLightOff(void);
void T1LightOff(void);
void T2LightOff(void);
void RHTurnLightOn(void);
void SetPWMTICK(void);
void T2tailAnymationOn(void);
void LMMInitLoop(void);
void LMMStatus(void);
void buttonIRQfall(void);
void T1stopturnLightOnPercentage(int FluxPer);

#include "TPS92664.h"
#include "TPS92520.h"

TPS92520 LedDriver(PB_15, PB_14, PB_13, PC_4, PB_12);
//DigitalOut CH1_PWM(PA_1);
//DigitalOut CH2_PWM(PA_3);

tps92664_device my_tps92664_device;                      /* A new driver instance */
tps92664_device *LMM1_handler = &my_tps92664_device; /* A driver handle to pass around */

tps92664_leds my_leds[16];
tps92664_leds *p_my_leds;

//uint8_t LEDsDuty[16] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
float LEDsDuty[16] = {10,10,10,10,100,0,0,0,0,0,0,0,0,0,0,0};
int blank_time = 0;
int PWMtick = 0;

tps92664_st_reg *preg;
    tps92664_st_reg tps92664_regs[] =
    {
        {0x00, 0x00, 0x00, 0xBF, 1}, /* MTPCFG */		
        {0x01, 0x01, 0x00, 0xFF, 1}, /* OUTCTRL */
        {0x02, 0x02, 0x00, 0xFF, 1}, /* TWLMT */
        {0x03, 0x03, 0x00, 0x3F, 1}, /* SLEWL */
        {0x04, 0x04, 0x00, 0x3F, 1}, /* SLEWH */
        {0x05, 0x05, 0x00, 0xFF, 1}, /* DWIDTH01 */
        {0x06, 0x06, 0x00, 0xFF, 1}, /* DWIDTH02 */
        {0x07, 0x07, 0x00, 0xFF, 1}, /* DWIDTH03 */
        {0x08, 0x08, 0x00, 0xFF, 1}, /* DWIDTH04 */
        {0x09, 0x09, 0x00, 0xFF, 1}, /* DWIDTH05 */
        {0x0A, 0x0A, 0x00, 0xFF, 1}, /* DWIDTH06 */
        {0x0B, 0x0B, 0x00, 0xFF, 1}, /* DWIDTH07 */
        {0x0C, 0x0C, 0x00, 0xFF, 1}, /* DWIDTH08 */
        {0x0D, 0x0D, 0x00, 0xFF, 1}, /* DWIDTH09 */
        {0x0E, 0x0E, 0x00, 0xFF, 1}, /* DWIDTH10 */
        {0x0F, 0x0F, 0x00, 0xFF, 1}, /* DWIDTH11 */       
        {0x10, 0x10, 0x00, 0xFF, 1}, /* DWIDTH12 */   
        {0x11, 0x11, 0x00, 0xFF, 1}, /* DWIDTH13 */   
        {0x12, 0x12, 0x00, 0xFF, 1}, /* DWIDTH14 */   
        {0x13, 0x13, 0x00, 0xFF, 1}, /* DWIDTH15 */   
        {0x14, 0x14, 0x00, 0xFF, 1}, /* DWIDTH16 */   
        {0x15, 0x15, 0x00, 0xFF, 1}, /* PHASE01H */   
        {0x16, 0x16, 0x00, 0xFF, 1}, /* PHASE02H */   
        {0x17, 0x17, 0x00, 0xFF, 1}, /* PHASE03H */   
        {0x18, 0x18, 0x00, 0xFF, 1}, /* PHASE04H */   
        {0x19, 0x19, 0x00, 0xFF, 1}, /* PHASE05H */   
        {0x1A, 0x1A, 0x00, 0xFF, 1}, /* PHASE06H */   
        {0x1B, 0x1B, 0x00, 0xFF, 1}, /* PHASE07H */   
        {0x1C, 0x1C, 0x00, 0xFF, 1}, /* PHASE08H */   
        {0x1D, 0x1D, 0x00, 0xFF, 1}, /* PHASE09H */   
        {0x1E, 0x1E, 0x00, 0xFF, 1}, /* PHASE10H */   
        {0x1F, 0x1F, 0x00, 0xFF, 1}, /* PHASE11H */   
        {0x20, 0x20, 0x00, 0xFF, 1}, /* PHASE12H */
        {0x21, 0x21, 0x00, 0xFF, 1}, /* PHASE13H */
        {0x22, 0x22, 0x00, 0xFF, 1}, /* PHASE14H */
        {0x23, 0x23, 0x00, 0xFF, 1}, /* PHASE15H */
        {0x24, 0x24, 0x00, 0xFF, 1}, /* PHASE16H */
        {0x25, 0x25, 0x00, 0xFF, 1}, /* PHASE04_01L */
        {0x26, 0x26, 0x00, 0xFF, 1}, /* PHASE08_05L */
        {0x27, 0x27, 0x00, 0xFF, 1}, /* PHASE12_09L */
        {0x28, 0x28, 0x00, 0xFF, 1}, /* PHASE16_13L */
        {0x29, 0x29, 0x00, 0xFF, 1}, /* WIDTH01H */
        {0x2A, 0x2A, 0x00, 0xFF, 1}, /* WIDTH02H */
        {0x2B, 0x2B, 0x00, 0xFF, 1}, /* WIDTH03H */
        {0x2C, 0x2C, 0x00, 0xFF, 1}, /* WIDTH04H */
        {0x2D, 0x2D, 0x00, 0xFF, 1}, /* WIDTH05H */
        {0x2E, 0x2E, 0x00, 0xFF, 1}, /* WIDTH06H */
        {0x2F, 0x2F, 0x00, 0xFF, 1}, /* WIDTH07H */
        {0x30, 0x30, 0x00, 0xFF, 1}, /* WIDTH08H */
        {0x31, 0x31, 0x00, 0xFF, 1}, /* WIDTH09H */
        {0x32, 0x32, 0x00, 0xFF, 1}, /* WIDTH10H */
        {0x33, 0x33, 0x00, 0xFF, 1}, /* WIDTH11H */
        {0x34, 0x34, 0x00, 0xFF, 1}, /* WIDTH12H */
        {0x35, 0x35, 0x00, 0xFF, 1}, /* WIDTH13H */
        {0x36, 0x36, 0x00, 0xFF, 1}, /* WIDTH14H */
        {0x37, 0x37, 0x00, 0xFF, 1}, /* WIDTH15H */
        {0x38, 0x38, 0x00, 0xFF, 1}, /* WIDTH16H */
        {0x39, 0x39, 0x00, 0xFF, 1}, /* WIDTH04_01L */
        {0x3A, 0x3A, 0x00, 0xFF, 1}, /* WIDTH08_05L */
        {0x3B, 0x3B, 0x00, 0xFF, 1}, /* WIDTH12_09L */
        {0x3C, 0x3C, 0x00, 0xFF, 1}, /* WIDTH16_13L */
        {0x3D, 0x3D, 0x00, 0xFF, 1}, /* BANK_PHASE01L */
        {0x3E, 0x3E, 0x00, 0xFF, 1}, /* BANK_PHASE02L */
        {0x3F, 0x3F, 0x00, 0xFF, 1}, /* BANK_PHASE03L */
        {0x40, 0x40, 0x00, 0xFF, 1}, /* BANK_PHASE04L */
        {0x41, 0x41, 0x00, 0xFF, 1}, /* BANK_PHASE04_01H */
        {0x42, 0x42, 0x00, 0xFF, 1}, /* BANK_PHASE05L */
        {0x43, 0x43, 0x00, 0xFF, 1}, /* BANK_PHASE06L */
        {0x44, 0x44, 0x00, 0xFF, 1}, /* BANK_PHASE07L */
        {0x45, 0x45, 0x00, 0xFF, 1}, /* BANK_PHASE08L */
        {0x46, 0x46, 0x00, 0xFF, 1}, /* BANK_PHASE08_05H */
        {0x47, 0x47, 0x00, 0xFF, 1}, /* BANK_PHASE09L */
        {0x48, 0x48, 0x00, 0xFF, 1}, /* BANK_PHASE10L */
        {0x49, 0x49, 0x00, 0xFF, 1}, /* BANK_PHASE11L */
        {0x4A, 0x4A, 0x00, 0xFF, 1}, /* BANK_PHASE12L */
        {0x4B, 0x4B, 0x00, 0xFF, 1}, /* BANK_PHASE12_09H */
        {0x4C, 0x4C, 0x00, 0xFF, 1}, /* BANK_PHASE13L */
        {0x4D, 0x4D, 0x00, 0xFF, 1}, /* BANK_PHASE14L */
        {0x4E, 0x4E, 0x00, 0xFF, 1}, /* BANK_PHASE15L */
        {0x4F, 0x4F, 0x00, 0xFF, 1}, /* BANK_PHASE16L */
        {0x50, 0x50, 0x00, 0xFF, 1}, /* BANK_PHASE16_13H */
        {0x51, 0x51, 0x00, 0xFF, 1}, /* BANK_WIDTH01L */
        {0x52, 0x52, 0x00, 0xFF, 1}, /* BANK_WIDTH02L */
        {0x53, 0x53, 0x00, 0xFF, 1}, /* BANK_WIDTH03L */
        {0x54, 0x54, 0x00, 0xFF, 1}, /* BANK_WIDTH04L */
        {0x55, 0x55, 0x00, 0xFF, 1}, /* BANK_WIDTH04_01H */
        {0x56, 0x56, 0x00, 0xFF, 1}, /* BANK_WIDTH05L */
        {0x57, 0x57, 0x00, 0xFF, 1}, /* BANK_WIDTH06L */
        {0x58, 0x58, 0x00, 0xFF, 1}, /* BANK_WIDTH07L */
        {0x59, 0x59, 0x00, 0xFF, 1}, /* BANK_WIDTH08L */
        {0x5A, 0x5A, 0x00, 0xFF, 1}, /* BANK_WIDTH08_05H */
        {0x5B, 0x5B, 0x00, 0xFF, 1}, /* BANK_WIDTH09L */
        {0x5C, 0x5C, 0x00, 0xFF, 1}, /* BANK_WIDTH10L */
        {0x5D, 0x5D, 0x00, 0xFF, 1}, /* BANK_WIDTH11L */
        {0x5E, 0x5E, 0x00, 0xFF, 1}, /* BANK_WIDTH12L */
        {0x5F, 0x5F, 0x00, 0xFF, 1}, /* BANK_WIDTH12_09H */
        {0x60, 0x60, 0x00, 0xFF, 1}, /* BANK_WIDTH13L */
        {0x61, 0x61, 0x00, 0xFF, 1}, /* BANK_WIDTH14L */
        {0x62, 0x62, 0x00, 0xFF, 1}, /* BANK_WIDTH15L */
        {0x63, 0x63, 0x00, 0xFF, 1}, /* BANK_WIDTH16L */
        {0x64, 0x64, 0x00, 0xFF, 1}, /* BANK_WIDTH16_13H */
        {0x65, 0x70, 0x00, 0xFF, 1}, /* LEDONTH01 */
        {0x66, 0x71, 0x01, 0xFF, 1}, /* LEDONTH02 */
        {0x67, 0x72, 0x01, 0xFF, 1}, /* LEDONTH03 */
        {0x68, 0x73, 0x01, 0xFF, 1}, /* LEDONTH04 */
        {0x69, 0x74, 0x01, 0xFF, 1}, /* LEDONTH05 */
        {0x6A, 0x75, 0x01, 0xFF, 1}, /* LEDONTH06 */
        {0x6B, 0x76, 0x01, 0xFF, 1}, /* LEDONTH07 */
        {0x6C, 0x77, 0x01, 0xFF, 1}, /* LEDONTH08 */
        {0x6D, 0x78, 0x01, 0xFF, 1}, /* LEDONTH09 */
        {0x6E, 0x79, 0x01, 0xFF, 1}, /* LEDONTH10 */
        {0x6F, 0x7A, 0x01, 0xFF, 1}, /* LEDONTH11 */
		{0x70, 0x7B, 0x01, 0xFF, 1}, /* LEDONTH12 */
        {0x71, 0x7C, 0x01, 0xFF, 1}, /* LEDONTH13 */
        {0x72, 0x7D, 0x01, 0xFF, 1}, /* LEDONTH14 */
        {0x73, 0x7E, 0x01, 0xFF, 1}, /* LEDONTH15 */
		{0x74, 0x7F, 0x01, 0xFF, 1}, /* LEDONTH16 */     
        {0x75, 0x80, 0x00, 0xFF, 1}, /* SYSCFG */
        {0x76, 0x81, 0x05, 0x05, 1}, /* CMWTAP */
        {0x77, 0x82, 0x0C, 0xFF, 1}, /* PWMTICK */
        {0x78, 0x83, 0x00, 0x03, 1}, /* ADCID */
        {0x79, 0x84, 0x00, 0x03, 1}, /* CLK_SYNC */
        {0x7A, 0x85, 0x00, 0xFF, 1}, /* STATUS */
        {0x7B, 0x86, 0x00, 0xFF, 1}, /* FLT_OPEN_OR_DRVL */
        {0x7C, 0x87, 0x00, 0xFF, 1}, /* FLT_OPEN_OR_DRVH */
        {0x7D, 0x88, 0x00, 0xFF, 1}, /* FAULT_SHORTL */
		{0x7E, 0x89, 0x00, 0xFF, 1}, /* FAULT_SHORTH */
		{0x7F, 0x8A, 0x00, 0xFF, 1}, /* FAULT_RESFETL */
		{0x80, 0x8B, 0x00, 0xFF, 1}, /* FAULT_RESFETH */    
        {0x81, 0x8C, 0x00, 0xFF, 2}, /* CERRCNT */	
        {0x82, 0x8D, 0x00, 0xFF, 2}, /* ADC1 */
        {0x83, 0x8E, 0x00, 0xFF, 2}, /* ADC2 */
        {0x84, 0x8F, 0x00, 0xFF, 2}, /* DIETEMP */
        {0x85, 0x90, 0x00, 0xFF, 2}, /* CS */
        {0x86, 0x91, 0x00, 0xFF, 2}, /* VLEDON01 */
        {0x87, 0x92, 0x00, 0xFF, 2}, /* VLEDON02 */
        {0x88, 0x93, 0x00, 0xFF, 2}, /* VLEDON03 */
        {0x89, 0x94, 0x00, 0xFF, 2}, /* VLEDON04 */
        {0x8A, 0x95, 0x00, 0xFF, 2}, /* VLEDON05 */
        {0x8B, 0x96, 0x00, 0xFF, 2}, /* VLEDON06 */
        {0x8C, 0x97, 0x00, 0xFF, 2}, /* VLEDON07 */
        {0x8D, 0x98, 0x00, 0xFF, 2}, /* VLEDON08 */
        {0x8E, 0x99, 0x00, 0xFF, 2}, /* VLEDON09 */
        {0x90, 0x9A, 0x00, 0xFF, 2}, /* VLEDON10 */
        {0x91, 0x9B, 0x00, 0xFF, 2}, /* VLEDON11 */
        {0x92, 0x9C, 0x00, 0xFF, 2}, /* VLEDON12 */
        {0x93, 0x9D, 0x00, 0xFF, 2}, /* VLEDON13 */
        {0x94, 0x9E, 0x00, 0xFF, 2}, /* VLEDON14 */
		{0x95, 0x9F, 0x00, 0xFF, 2}, /* VLEDON15 */
        {0x96, 0xA0, 0x00, 0xFF, 2}, /* VLEDON16 */
		{0x97, 0xA1, 0x00, 0xFF, 2}, /* VLEDOFF01 */
        {0x98, 0xA2, 0x00, 0xFF, 2}, /* VLEDOFF02 */
        {0x99, 0xA3, 0x00, 0xFF, 2}, /* VLEDOFF03 */
        {0x9A, 0xA4, 0x00, 0xFF, 2}, /* VLEDOFF04 */
        {0x9B, 0xA5, 0x00, 0xFF, 2}, /* VLEDOFF05 */
        {0x9C, 0xA6, 0x00, 0xFF, 2}, /* VLEDOFF06 */
        {0x9D, 0xA7, 0x00, 0xFF, 2}, /* VLEDOFF07 */
        {0x9E, 0xA8, 0x00, 0xFF, 2}, /* VLEDOFF08 */
        {0x9F, 0xA9, 0x00, 0xFF, 2}, /* VLEDOFF09 */
        {0xA0, 0xAA, 0x00, 0xFF, 2}, /* VLEDOFF10 */
        {0xA1, 0xAB, 0x00, 0xFF, 2}, /* VLEDOFF11 */
        {0xA2, 0xAC, 0x00, 0xFF, 2}, /* VLEDOFF12 */
        {0xA3, 0xAD, 0x00, 0xFF, 2}, /* VLEDOFF13 */
        {0xA4, 0xAE, 0x00, 0xFF, 2}, /* VLEDOFF14 */
		{0xA5, 0xAF, 0x00, 0xFF, 2}, /* VLEDOFF15 */
        {0xA6, 0xB0, 0x00, 0xFF, 2}, /* VLEDOFF16 */	
        {0xA7, 0xB1, 0x00, 0xFF, 2}, /* PWM_MISCOUNTL */
        {0xA8, 0xB2, 0x00, 0xFF, 2}, /* PWM_MISCOUNTH */
        {0xA9, 0xF7, 0x00, 0xFF, 2}, /* LOTINFO1 */
		{0xAA, 0xF8, 0x00, 0xFF, 2}, /* LOTINFO2 */
		{0xAB, 0xF9, 0x00, 0xFF, 2}, /* LOTINFO3 */
		{0xAC, 0xFA, 0x00, 0xFF, 2}, /* LOTINFO4 */
		{0xAD, 0xFB, 0x00, 0xFF, 2}, /* MTP_PROG1 */
		{0xAE, 0xFC, 0x00, 0xFF, 2}, /* MTP_PROG2 */
		{0xAF, 0xFD, 0x00, 0xFF, 2}, /* MTP_PROG3 */
		{0xB0, 0xFE, 0x00, 0xFF, 2}, /* MTP_PROG4 */
        {0xB1, 0xFF, 0x00, 0xFF, 2}, /* TPS_ICID */
    };


void buttonIRQrise(void)
{

    CANeventQueue.call(T1stopturnLightOff);
}

void buttonIRQfall(void)
{

    CANeventQueue.call(T1stopturnLightOn);
}


void T1stopturnLightOnPercentage(int FluxPer)
{
    CANmutex.lock();
    //printf("T1stopturnLightOnPercentage\n");
    if (FluxPer>100)
    {
        FluxPer = 100;
    }
    else if (FluxPer<0)
    {
        FluxPer = 0;
    }
    float FluxPerU = (int)FluxPer;
    //printf("FluxPer = %d\n",FluxPerU);

    float LEDsDutyStop[16] = {FluxPerU,FluxPerU,FluxPerU,FluxPerU,100,100,100,100,FluxPerU,100,100,100,100,100,100,100};

    //printf("TPS92664_Calculate_Phase_Width\n");
    TPS92664_Calculate_Phase_Width(p_my_leds, LEDsDutyStop, blank_time);

    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    CANmutex.unlock();
}


void T1stopturnLightOn(void)
{
    CANmutex.lock();
    printf("T1 Stop Light On\n");
    //float LEDsDutyStop[16] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100}; //origin B2-Sample, C0-Sample

    float LEDsDutyStop[16] = {100,100,100,100,
                                100,100,100,100,
                                0,100,100,100,100,
                                100,100,100}; //Option 1E

    printf("TPS92664_Calculate_Phase_Width\n");
    TPS92664_Calculate_Phase_Width(p_my_leds, LEDsDutyStop, blank_time);


    my_tps92664_device.device_id = T1_RH_LMM_DEVICE_ID;
    int ret = TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    if (ret < 0)
    {
        printf("TPS92664_SetLeds FAIL\n");
    }
    CANmutex.unlock();


}

void T1stopturnLightOff(void)
{
    CANmutex.lock();
    printf("T1 Stop Light Off\n");
    float LEDsDutyStop[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    printf("TPS92664_Calculate_Phase_Width\n");
    TPS92664_Calculate_Phase_Width(p_my_leds, LEDsDutyStop, blank_time);

    my_tps92664_device.device_id = T1_RH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    CANmutex.unlock();
}

void T2stopLightOn(void)
{
    CANmutex.lock();
    printf("T2 Stop Light On\n");
    float LEDsDutyStop[16] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100}; //origin B2-Sample, C0-Sample


    printf("TPS92664_Calculate_Phase_Width\n");
    TPS92664_Calculate_Phase_Width(p_my_leds, LEDsDutyStop, blank_time);
    

    my_tps92664_device.device_id = T2_CHMSL_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T1_RH_LMM_DEVICE_ID;
    CANmutex.unlock();
}

void T2stopLightOff(void)
{
    CANmutex.lock();
    printf("T2 Stop Light Off\n");
    float LEDsDutyStop[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    printf("TPS92664_Calculate_Phase_Width\n");
    TPS92664_Calculate_Phase_Width(p_my_leds, LEDsDutyStop, blank_time);
    

    my_tps92664_device.device_id = T2_CHMSL_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T1_RH_LMM_DEVICE_ID;
    CANmutex.unlock();
}

void T1tailLightOn(void)
{
    CANmutex.lock();
    printf("T1 Tail Light On\n");
    //float LEDsDutyTail[16] = {10.93,10.93,10.93,12.7,100,0,0,0,0,0,0,0,0,0,0,0};     // first segment is longer
    //float LEDsDutyTail[16] = {10,10,10,10,100,0,0,0,0,0,0,0,0,0,0,0};     // origin setup
    //float LEDsDutyTail[16] = {10.1,10.1,10.1,10.1,100,0,0,0,0,0,0,0,0,0,0,0};     // origin setup
    float LEDsDutyTail[16] = {10.42,10.42,10.42,10.71,0,0,0,0,0,0,0,0,0,0,0,0};     // origin setup
    printf("TPS92664_Calculate_Phase_Width\n");
    TPS92664_Calculate_Phase_Width(p_my_leds, LEDsDutyTail, blank_time);


    my_tps92664_device.device_id = T1_RH_LMM_DEVICE_ID;
    int ret = TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    if (ret < 0)
    {
        printf("TPS92664_SetLeds FAIL\n");
    }
    CANmutex.unlock();
    
}

void T1tailLightOff(void)
{
    CANmutex.lock();
    printf("T1 Tail Light Off\n");
    float LEDsDuty[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    printf("TPS92664_Calculate_Phase_Width\n");
    TPS92664_Calculate_Phase_Width(p_my_leds, LEDsDuty, blank_time);


    my_tps92664_device.device_id = T1_RH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    CANmutex.unlock();
}

void T2tailLightOn(void)
{
    CANmutex.lock();
    printf("T2 Tail Light On\n");
    float LEDsDuty[16] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};

    printf("TPS92664_Calculate_Phase_Width\n");
    TPS92664_Calculate_Phase_Width(p_my_leds, LEDsDuty, blank_time);
    

    my_tps92664_device.device_id = T2_OUTBOARD_RH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T2_OUTBOARD_LH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T2_CENTRAL_LH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T2_CENTRAL_RH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T1_RH_LMM_DEVICE_ID;
    CANmutex.unlock();
}

void T2tailAnymationOn(void)
{
    CANmutex.lock();
    printf("T2 Tail Light On\n");
    float LEDsDuty[16] = {6,6,6,6,6,6,6,0,6,6,6,6,6,6,6,0}; // visible missmatch of LED brightness;

    printf("TPS92664_Calculate_Phase_Width\n");
    TPS92664_Calculate_Phase_Width(p_my_leds, LEDsDuty, blank_time);
    

    my_tps92664_device.device_id = T2_OUTBOARD_RH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T2_OUTBOARD_LH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T2_CENTRAL_LH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T2_CENTRAL_RH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T1_RH_LMM_DEVICE_ID;
    CANmutex.unlock();
}

void T2tailLightOff(void)
{
    CANmutex.lock();
    printf("T2 Tail Light On\n");
    float LEDsDuty[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    printf("TPS92664_Calculate_Phase_Width\n");
    TPS92664_Calculate_Phase_Width(p_my_leds, LEDsDuty, blank_time);


    my_tps92664_device.device_id = T2_OUTBOARD_RH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T2_OUTBOARD_LH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T2_CENTRAL_LH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T2_CENTRAL_RH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T1_RH_LMM_DEVICE_ID;
    CANmutex.unlock();
}

void T2LightOff(void)
{
    CANmutex.lock();
    printf("T2 Tail Light On\n");
    float LEDsDuty[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    printf("TPS92664_Calculate_Phase_Width\n");
    TPS92664_Calculate_Phase_Width(p_my_leds, LEDsDuty, blank_time);


    my_tps92664_device.device_id = T2_OUTBOARD_RH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T2_OUTBOARD_LH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T2_CENTRAL_LH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T2_CENTRAL_RH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T2_CHMSL_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    my_tps92664_device.device_id = T1_RH_LMM_DEVICE_ID;
    CANmutex.unlock();
}


void T1LightOff(void)
{
    CANmutex.lock();
    printf("T1 Light Off\n");
    float LEDsDutyStop[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    printf("TPS92664_Calculate_Phase_Width\n");
    TPS92664_Calculate_Phase_Width(p_my_leds, LEDsDutyStop, blank_time);

    my_tps92664_device.device_id = T1_RH_LMM_DEVICE_ID;
    TPS92664_SetLeds(LMM1_handler, p_my_leds, 16);
    CANmutex.unlock();
}

void SetPWMTICK(void)
{
    CANmutex.lock();
    printf("SetPWMTICK\n");
    

    my_tps92664_device.device_id = T1_RH_LMM_DEVICE_ID;
    printf("TPS Set PWMTICK\n");
    tps92664_regs[TPS_PWMTICK].value = PWMtick;  
    int ret = TPS92664_BroadcastWriteRegs(&tps92664_regs[TPS_PWMTICK],1);  



    CANmutex.unlock();
}


void AllLightOff(void)
{
    
    #if defined(T1_LAMP)
    CANeventQueue.call(T1LightOff);
    #endif
    #if defined(T2_LAMP)
    CANeventQueue.call(T2LightOff);
    #endif
}

void TailLightOn(void)
{
    #if defined(T1_LAMP)
    CANeventQueue.call(T1tailLightOn);
    #endif
    #if defined(T2_LAMP)
    CANeventQueue.call(T2tailLightOn);
    #endif
    
}

void TailLightOff(void)
{
    #if defined(T1_LAMP)
    CANeventQueue.call(T1tailLightOff);
    #endif
    #if defined(T2_LAMP)
    CANeventQueue.call(T2tailLightOff);
    #endif
}

void StopLightOff(void)
{
    
    #if defined(T1_LAMP)
    CANeventQueue.call(T1stopturnLightOff);
    #endif
    #if defined(T2_LAMP)
    CANeventQueue.call(T2stopLightOff);
    #endif
}

void StopLightOn(void)
{
    #if defined(T1_LAMP)
    CANeventQueue.call(T1stopturnLightOn);
    #endif
    #if defined(T2_LAMP)
    CANeventQueue.call(T2stopLightOn);
    #endif
    
}

void RHTurnLightOn(void)
{
    CANeventQueue.call(T1stopturnLightOn);
    
}

void led2_thread()
{
    while (true) {
        led2 = !led2;
        ThisThread::sleep_for(1s);
    }
}

void LMMStatusRead(void)
{
    while (true) {
        CANmutex.lock();
        my_tps92664_device.device_id = T1_RH_LMM_DEVICE_ID;

        int ret = TPS92664_SingleDeviceRead(LMM1_handler, &tps92664_regs[TPS_STATUS], 1);
        if (ret < 0) {
            LMMMODE = -1;
            ThisThread::sleep_for(10ms);
        }
        else {
            if (LMMMODE < 0) {
                //printf("LMM REINIT\n\r");
                CANmutex.unlock();
                CANeventQueue.call(LMMInitLoop);
                LMMMODE = 0;
                //printf("LMMMODE %d\n\r", LMMMODE);
            }
            else{
                LMMMInit.terminate();
            }
            ThisThread::sleep_for(10ms);
            CANmutex.unlock();
        }

    }
}



void LMMStatus(void)
{
    while (LMMStatusrunning) {
    /*  printf("LMMStatusThread.get_state()\n");
        uint8_t test =  LMMStatusThread.get_state();
        printf("test %d\n",test);
        printf("LMMStatusThread.get_state() %d\n",LMMStatusThread.get_state());*/
        CANmutex.lock();
        printf("LMMStatus\n");
        my_tps92664_device.device_id = T1_RH_LMM_DEVICE_ID;

        printf("%s",TPS92664_Get_Status(LMM1_handler));

        int ret = TPS92664_SingleDeviceRead(LMM1_handler, &preg[TPS_ADC1], 4);
        if (ret < 0)
        {
            //break;
        }
        else
        {
            printf("TPS92664 ADC1 : %d\n", tps92664_regs[TPS_ADC1].value);
            printf("TPS92664 ADC2 : %d\n", my_tps92664_device.regs[TPS_ADC2].value);
            printf("TPS92664 TPS_DIETEMP : %d\n", my_tps92664_device.regs[TPS_DIETEMP].value);
            printf("TPS92664 TPS_CS : %d\n", my_tps92664_device.regs[TPS_CS].value);
        }



        ret = TPS92664_SingleDeviceRead(LMM1_handler, &preg[TPS_ICID], 1);
        if (ret < 0)
        {

        }
        else
        {
         
            printf("TPS92664 TPS ICID : %d\n", my_tps92664_device.regs[TPS_ICID].value);
            printf("TPS92664 device ID : %d\n", my_tps92664_device.device_id);
        }

        ret = TPS92664_Get_ADC(LMM1_handler, ADC_CH1);
        printf("TPS92664 TPS ADC1 : %d\n", ret);

        ret = TPS92664_SingleDeviceRead(LMM1_handler, &tps92664_regs[TPS_STATUS],1);
        if (ret < 0)
        {

        }
        else
        {
            printf("TPS92664 TPS_STATUS : %d\n", my_tps92664_device.regs[TPS_STATUS].value);
        }

        ret = TPS92664_SingleDeviceRead(LMM1_handler, &tps92664_regs[TPS_SYSCFG],1);
        if (ret < 0)
        {

        }
        else
        {
            printf("TPS92664 TPS_SYSCFG : %d\n", my_tps92664_device.regs[TPS_SYSCFG].value);
        }
/*
        ret = TPS92664_SingleDeviceRead(LMM1_handler, &tps92664_regs[TPS_CMWTAP],1);
        if (ret < 0)
        {
            break;
        }
        else
        {
            printf("my_tps92664_device %d TPS92664 TPS_CMWTAP : %d\n",my_tps92664_device.device_id , my_tps92664_device.regs[TPS_CMWTAP].value);
        }

        ret = TPS92664_SingleDeviceRead(LMM1_handler, &preg[WIDTH01H], 1);
        if (ret < 0)
        {

        }
        else
        {
            printf("TPS92664 WIDTH01H : %d\n", my_tps92664_device.regs[WIDTH01H].value);
        }

        ret = TPS92664_SingleDeviceRead(LMM1_handler, &preg[MTPCFG], 2);
        if (ret < 0)
        {
            printf("Read Fault\n");
        }
        else
        {
            printf("TPS92664 MTPCFG : %d\n", my_tps92664_device.regs[MTPCFG].value);
            printf("TPS92664 OUTCTRL : %d\n", my_tps92664_device.regs[OUTCTRL].value);
        }

        ret = TPS92664_SingleDeviceRead(LMM1_handler, &preg[TPS_PWMTICK], 1);
        if (ret < 0)
        {
            printf("Read Fault\n");
        }
        else
        {
            printf("TPS92664 PWMTICK : %d\n", my_tps92664_device.regs[TPS_PWMTICK].value);
        }
        
        ret = TPS92664_SingleDeviceRead(LMM1_handler, &preg[SLEWL], 2);
        if (ret < 0)
        {
            printf("Read Fault\n");
        }
        else
        {
            printf("TPS92664 SLEWL : %d\n", my_tps92664_device.regs[SLEWL].value);
            printf("TPS92664 SLEWH : %d\n", my_tps92664_device.regs[SLEWH].value);
        }
        

        ret = TPS92664_SingleDeviceRead(LMM1_handler, &preg[VLEDON01], 16);
        if (ret < 0)
        {
            printf("Read Fault\n");
        }
        else
        {
            printf("TPS92664 VLEDON01 : %d\n", my_tps92664_device.regs[VLEDON01].value);
            printf("TPS92664 VLEDON02 : %d\n", my_tps92664_device.regs[VLEDON02].value);
            printf("TPS92664 VLEDON03 : %d\n", my_tps92664_device.regs[VLEDON03].value);
            printf("TPS92664 VLEDON04 : %d\n", my_tps92664_device.regs[VLEDON04].value);
            printf("TPS92664 VLEDON05 : %d\n", my_tps92664_device.regs[VLEDON05].value);
            printf("TPS92664 VLEDON09 : %d\n", my_tps92664_device.regs[VLEDON09].value);
            printf("TPS92664 VLEDON13 : %d\n", my_tps92664_device.regs[VLEDON13].value);
          
        }

        ret = TPS92664_SingleDeviceRead(LMM1_handler, &preg[VLEDOFF01], 16);
        if (ret < 0)
        {
            printf("Read Fault\n");
        }
        else
        {
            printf("TPS92664 VLEDOFF01 : %d\n", my_tps92664_device.regs[VLEDOFF01].value);
            printf("TPS92664 VLEDOFF02 : %d\n", my_tps92664_device.regs[VLEDOFF02].value);
            printf("TPS92664 VLEDOFF03 : %d\n", my_tps92664_device.regs[VLEDOFF03].value);
            printf("TPS92664 VLEDOFF04 : %d\n", my_tps92664_device.regs[VLEDOFF04].value);
            printf("TPS92664 VLEDOFF05 : %d\n", my_tps92664_device.regs[VLEDOFF05].value);
            printf("TPS92664 VLEDOFF09 : %d\n", my_tps92664_device.regs[VLEDOFF09].value);
            printf("TPS92664 VLEDOFF13 : %d\n", my_tps92664_device.regs[VLEDOFF13].value);
        }
        */
         
        ret = TPS92664_SingleDeviceRead(LMM1_handler, &preg[FAULT_SHORTH], 1);
        if (ret < 0)
        {
           // break;
           // printf("Read Fault\n");
        }
        else
        {
            printf("TPS92664 FAULT_SHORTH : %d\n", my_tps92664_device.regs[FAULT_SHORTH].value);
        }

        if (my_tps92664_device.regs[FAULT_SHORTH].value > 0)
        {
            // mask
            uint8_t FAULT_SHORTH_MASK= 0b00010000;
            uint8_t faulth = tps92664_regs[FAULT_SHORTH].value & FAULT_SHORTH_MASK;
            //printf("fault High : %d\n", faulth);
            
            if((tps92664_regs[FAULT_SHORTH].value & FAULT_SHORTH_MASK) > 0)
            {
                printf("stop/turn LED string Short/Open Circuit Fault detected\n");
            }

            // write to clear
            tps92664_regs[FAULT_SHORTH].value = 0;  
            ret = TPS92664_SingleDeviceWriteRegs(LMM1_handler, &tps92664_regs[FAULT_SHORTH],1);
                if (ret < 0)
                {
                   // break;
                }
                else
                {
                    //printf("my_tps92664_device TPS92664 FAULT_SHORTH : %d\n", my_tps92664_device.regs[FAULT_SHORTH].value);
                }
            
        }

        ret = TPS92664_SingleDeviceRead(LMM1_handler, &preg[FAULT_SHORTL], 1);
        if (ret < 0)
        {
          //  break;
           // printf("Read Fault\n");
        }
        else
        {
            printf("TPS92664 FAULT_SHORTL : %d\n", my_tps92664_device.regs[FAULT_SHORTL].value);
        }

        if (my_tps92664_device.regs[FAULT_SHORTL].value > 0)
        {
            // mask
            uint8_t FAULT_SHORTL_MASK= 0b00011111;
            uint8_t faultl = tps92664_regs[FAULT_SHORTH].value & FAULT_SHORTL_MASK;
            //printf("fault Low : %d\n", faultl);
            FAULT_SHORTL_MASK= 0b00010000;
            if((tps92664_regs[FAULT_SHORTL].value & FAULT_SHORTL_MASK) > 0)
            {
                printf("Tail (aux.stop/turn) LED string Short/Open Circuit Fault detected\n");
            }
            FAULT_SHORTL_MASK= 0b00001111;
            if((tps92664_regs[FAULT_SHORTL].value & FAULT_SHORTL_MASK) > 0)
            {
                printf("Tail (aux.stop/turn) single LED Short/Open Circuit Fault detected\n");
            }

            // write to clear
            tps92664_regs[FAULT_SHORTL].value = 0;  
            ret = TPS92664_SingleDeviceWriteRegs(LMM1_handler, &tps92664_regs[FAULT_SHORTL],1);
                if (ret < 0)
                {
                 //   break;
                }
                else
                {
                    //printf("my_tps92664_device TPS92664 FAULT_SHORTL : %d\n", my_tps92664_device.regs[FAULT_SHORTL].value);
                }
            
        }

        printf("\r\n");
        CANmutex.unlock();

        ThisThread::sleep_for(800ms);
    }

}

int main()
{
    int response, ret;
    printf("Starting..\r\n");

    printmenu();

   
  //  thread1.set_priority(osPriorityLow);
  //  thread2.set_priority(osPriorityBelowNormal);
  //  thread3.set_priority(osPriorityNormal);
   // thread4.set_priority(osPriorityLow);
  //  thread5.set_priority(osPriorityNormal);
  //  LMMStatusThread.set_priority(osPriorityLow);


    thread1.start(taskPrintData);
    thread2.start(callback(&VirtualSerialeventQueue, &EventQueue::dispatch_forever));
    thread5.start(callback(&CANeventQueue, &EventQueue::dispatch_forever));
   // LMMStatusThread.start(callback(&CANeventQueue, &EventQueue::dispatch_forever));
    thread4.start(led2_thread);
    serial.sigio(callback(onSigio));

    LMMInitLoop();
    TPS92664_communicationReset();

    thread3.start(LMMStatusRead);

    button.rise(&buttonIRQrise);  // attach the address of the flip function to the rising edge
    button.fall(&buttonIRQfall);  // attach the address of the flip function to the rising edge

    while (1) 
    {
        

    }
}

void LMMInitLoop(void)
{
    printf("LMMInitLoop\n");

    int ret;
    preg = tps92664_regs;
    p_my_leds = my_leds;

    ret = TPS92664_init(LMM1_handler, (tps92664_st_reg *)&tps92664_regs);
    if (ret < 0)
    {
        /* TPS92664 initialization failed, check the value of ret! */
       // printf("TPS92664 Init Fail\n");
    }
    else
    {
        /* TPS92664 initialization OK */
      //  printf("TPS92664 Init OK\n");
    } 

    my_tps92664_device.regs = tps92664_regs;
    


    TPS92664_communicationReset();

    tps92664_regs[MTPCFG].value = 0x60;           //Vopalensky
    ret = TPS92664_BroadcastWriteRegs(&tps92664_regs[MTPCFG],1);  
    

    my_tps92664_device.device_id = T1_RH_LMM_DEVICE_ID;
   // printf("TPS92664 device address : %d\n", my_tps92664_device.device_id);
    tps92664_regs[TPS_SYSCFG].value = int(0b00100000);           //Broadcast set ACKEN && CMWEN
    TPS92664_BroadcastWriteRegs(&tps92664_regs[TPS_SYSCFG],1);   
    my_tps92664_device.useACK = true;



    tps92664_regs[DEFWIDTH01].value = 0x1A;
    tps92664_regs[DEFWIDTH02].value = 0x1A;
    tps92664_regs[DEFWIDTH03].value = 0x1A;
    tps92664_regs[DEFWIDTH04].value = 0x1A;
    tps92664_regs[DEFWIDTH05].value = 0xFF;
    tps92664_regs[DEFWIDTH06].value = 0x00;
    tps92664_regs[DEFWIDTH07].value = 0x00;
    tps92664_regs[DEFWIDTH08].value = 0x00;
    tps92664_regs[DEFWIDTH09].value = 0x00;
    tps92664_regs[DEFWIDTH10].value = 0x00;
    tps92664_regs[DEFWIDTH11].value = 0x00;
    tps92664_regs[DEFWIDTH12].value = 0x00;
    tps92664_regs[DEFWIDTH13].value = 0x00;
    tps92664_regs[DEFWIDTH14].value = 0x00;
    tps92664_regs[DEFWIDTH15].value = 0x00;
    tps92664_regs[DEFWIDTH16].value = 0x00;


   // printf("TPS Set default width registers\n");
    ret = TPS92664_BroadcastWriteRegs(&tps92664_regs[DEFWIDTH01],16);  

    if (ret < 0)
    {
      //  printf("TPS92664_BroadcastWriteRegs FAIL\n");
    }
    else
    {
      //  printf("TPS92664_BroadcastWriteRegs OK\n");
    }

    tps92664_regs[LEDONTH01].value = 51;
    tps92664_regs[LEDONTH02].value = 51;
    tps92664_regs[LEDONTH03].value = 51;
    tps92664_regs[LEDONTH04].value = 51;
    tps92664_regs[LEDONTH05].value = 55;
    tps92664_regs[LEDONTH06].value = 3;
    tps92664_regs[LEDONTH07].value = 3;
    tps92664_regs[LEDONTH08].value = 3;
    tps92664_regs[LEDONTH09].value = 55;    // origin
    tps92664_regs[LEDONTH09].value = 3;    // option 1E
    tps92664_regs[LEDONTH10].value = 3;
    tps92664_regs[LEDONTH11].value = 3;
    tps92664_regs[LEDONTH12].value = 3;
    tps92664_regs[LEDONTH13].value = 55;
    tps92664_regs[LEDONTH14].value = 3;
    tps92664_regs[LEDONTH15].value = 3;
    tps92664_regs[LEDONTH16].value = 3;


    //printf("TPS Set LED ON tresholdsregisters\n");
    ret = TPS92664_BroadcastWriteRegs(&tps92664_regs[LEDONTH01],16);  

    if (ret < 0)
    {
        //printf("TPS92664_BroadcastWriteRegs FAIL\n");
    }
    else
    {
       // printf("TPS92664_BroadcastWriteRegs OK\n");
    }

   // printf("Setup CMWTAB Watchdog\n");
    tps92664_regs[TPS_CMWTAP].value = 7;           
    ret = TPS92664_BroadcastWriteRegs(&tps92664_regs[TPS_CMWTAP],1);   
    if (ret < 0)
    {
      //  printf("Setup CMWTABFAIL\n");
    }
    else
    {
      //  printf("Setup CMWTAB OK\n");
    }



    //printf("Enable Watchdog\n");
    //tps92664_regs[TPS_SYSCFG].value = int(0b00101000);           //Broadcast set ACKEN && CMWEN
    tps92664_regs[TPS_SYSCFG].value = int(0b01101101);           //Broadcast set LEDADCEN && ACKEN && CMWEN && CSEN && CSGAIN
    //tps92664_regs[TPS_SYSCFG].value = int(0b01100101);           //Broadcast set LEDADCEN && ACKEN  && CSEN && CSGAIN
    ret = TPS92664_BroadcastWriteRegs(&tps92664_regs[TPS_SYSCFG],1);   
    if (ret < 0)
    {
      //  printf("Watchdog FAIL\n");
    }
    else
    {
      //  printf("Watchdog OK\n");
    }

    tps92664_regs[TPS_STATUS].value = int(0b00000001);           //At startup, set bit to 1, then read periodically:
    ret = TPS92664_BroadcastWriteRegs(&tps92664_regs[TPS_STATUS],1);   
    if (ret < 0)
    {
      //  printf("Status set-up FAIL\n");
    }
    else
    {
      //  printf("Status set-up OK\n");
    }

    ret = TPS92664_Set_CSGain(LMM1_handler, x10_GAIN);
    if (ret < 0)
    {
      //  printf("TPS92664_Set_CSGain FAIL\n");
    }
    else
    {
      //  printf("TPS92664_Set_CSGain OK\n");
    }

    //printf("TPS Set PWMTICK\n");
    tps92664_regs[TPS_PWMTICK].value = 13;  
    ret = TPS92664_BroadcastWriteRegs(&tps92664_regs[TPS_PWMTICK],1);  

    if (ret < 0)
    {
      //  printf("TPS92664_BroadcastWriteRegs FAIL\n");
    }
    else
    {
      //  printf("TPS92664_BroadcastWriteRegs OK\n");
    }
   // printf("TPS Set SlewRate\n");
    ret = TPS92664_Set_SlewRate(LMM1_handler, FASTEST_RATE);
    if (ret < 0)
    {
      //  printf("TPS92664_Set_SlewRate FAIL\n");
    }
    else
    {
      //  printf("TPS92664_Set_SlewRates OK\n");
    }

   // printf("TPS92664_SetLeds\n");
    ret = TPS92664_BroadcastSetLeds(LMM1_handler, p_my_leds, 16);

    if (ret < 0)
    {
        //   printf("TPS92664_BroadcastSetLeds FAIL\n");
    }
    else
    {
      //  printf("TPS92664_BroadcastSetLeds OK\n");
    }
    tps92664_regs[TPS_STATUS].value = 1;  
    ret = TPS92664_BroadcastWriteRegs(&tps92664_regs[TPS_STATUS],1);   
    if (ret < 0)
    {
      //  printf("Status set-up FAIL\n");
    }
    else
    {
      //  printf("Status set-up OK\n");
    }
    
    ret = TPS92664_SingleDeviceWriteRegs(LMM1_handler, &tps92664_regs[TPS_STATUS],1);   
    if (ret < 0)
    {
      //  printf("Status set-up FAIL\n");
    }
    else
    {
      //  printf("Status set-up OK\n");
    }
    LMMMODE = 0;
    printf("REINIT DONE\n");
    LMMStatusThread.start(LMMStatus);
}


void printmenu(void){
    mutex.lock();
    printf("Available Commands:\n\r");
    printf("TPS92664 [+/-]     - Setup LMM TPS92664blank time\n\r");
    printf("TEMP [percentage]     \n\r");
    printf("ANYM    \n\r");
    printf("devid    \n\r");
    printf("STOP    \n\r");
    printf("TAIL    \n\r");
    printf("TURN    \n\r");
    printf("DIR    \n\r");
    printf("POS    \n\r");
    printf("OFF    \n\r");
    printf("TICK    \n\r");
    mutex.unlock();
}


void taskPrintData()
{
    mutex.lock();

    while (1) {
        // Wait for a condition to change
        cond.wait();
        int response, ret;
        int raw_data = 0;
        // Now it is safe to access data in this thread
        printf("Data received: %s\r\n", data);

        if(strncmp(data,"TPS92664",8) == 0) {
            int value = data[9];
            if (value == 43) {
                blank_time = blank_time+1;}
            if (value == 45) {
                blank_time = blank_time-1;}
            
            printf("LMM blank_time:%d\r\n", blank_time);
            CANeventQueue.call(TailLightOn);
        }        
        else if(strncmp(data,"ANYM",4) == 0) {
            CANeventQueue.call(T2tailAnymationOn);
        }
        else if(strncmp(data,"devid",5) == 0) {
            int len = strlen(data)-1;
            int devid = 0;
            int count=1;
            while(len > 6) {
                devid += count*hex2int(data[len-1]);
                count = count*10;
                len--; 
            }
            printf("devid: %d \r\n", devid);
            my_tps92664_device.device_id = devid;

        }
        else if(strncmp(data,"TICK",4)== 0) {
            int len = strlen(data);
           // printf("lenght1 : %d \r\n", len);
            len--; 
            int tick = 0;
            int count=1;
            while(len > 5) { 
               // printf("data[len-1] : %d \r\n", data[len-1]);
               // printf("hex2int : %d \r\n", hex2int(data[len-1]));
                tick += count*hex2int(data[len-1]);
               // printf("PWM tick : %d \r\n", tick);
                count = count*10;
                len--; 
            }
            PWMtick = tick;
            printf("PWM TICK : %d \r\n", PWMtick);
            CANeventQueue.call(SetPWMTICK);

        }
        else if(strncmp(data,"STOP",4) == 0) {
           CANeventQueue.call(StopLightOn);

        }
        else if(strncmp(data,"TAIL",3) == 0) {
           CANeventQueue.call(TailLightOn);
        }
        else if(strncmp(data,"TURN",4) == 0) {
           CANeventQueue.call(RHTurnLightOn);
        }
        else if(strncmp(data,"DIR",3) == 0) {
           CANeventQueue.call(RHTurnLightOn);
        }
        else if(strncmp(data,"POS",3) == 0) {
           CANeventQueue.call(TailLightOn);
        }
        else if(strncmp(data,"OFF",3) == 0) {
           CANeventQueue.call(AllLightOff);

        }
        else if(strncmp(data,"TEMP",4)== 0) {
            int len = strlen(data)-4;
            int duty = 0;
            int count=1;
            while(len > 2) { 
                duty += count*hex2int(data[len-1]);
                count = count*10;
                len--; 
            }
            printf("Temp Derating: %d \r\n", duty);
            CANeventQueue.call(T1stopturnLightOnPercentage, duty);
            
            //T1stopturnLightOnPercentage(duty);
        }

        else {
            printf("Invalid Command\r\n");
            printmenu();
        }

        memset(data, 0, DATA_LEN);  // empty data to make space for new data
    }
}

void onSerialReceived(void)
{

    while (serial.readable()) {
        // Read serial
        recvLen = serial.read(recvBuff, BUFF_LEN);

        if ((strlen(message) + recvLen) > MSG_LEN) {
            // too much data -> something went wrong
            memset(message, 0, MSG_LEN);
            break;
        }

        strcat(message, recvBuff);          // append received chars to message

       
        if (message[strlen(message) - 1] == '\r') {
            // message complete
            mutex.lock();

            // copy the chars from the message to the data storage
           // strcat(data, &message[1]);      // omit first char (which is 's')
           strcat(data, message);
            //data[strlen(data) - 1] = '\0';  // delete last char (which is '#')
            memset(message, 0, MSG_LEN);

            // Signal for other threads that data has been received
            cond.notify_all();
            mutex.unlock();
            break;
        }
    }
    memset(recvBuff, 0, BUFF_LEN);
}

void onSigio(void)
{
    VirtualSerialeventQueue.call(onSerialReceived);
}

int hex2int(char ch)
{
    if (ch >= '0' && ch <= '9')
        return ch - '0';
    if (ch >= 'A' && ch <= 'F')
        return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f')
        return ch - 'a' + 10;
    return -1;
}