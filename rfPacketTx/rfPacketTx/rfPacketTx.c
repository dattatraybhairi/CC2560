/*
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>
#include <unistd.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Board Header files */
#include "Board.h"
#include "smartrf_settings/smartrf_settings.h"

/*********************************************************************
* INCLUDES modified by Prem
*/
#include <stdio.h>
//ADC driver 
#include <ti/drivers/ADC.h>
//Battery monitor driver
#include "ti/devices/cc13x2_cc26x2/driverlib/aon_batmon.h"
#include <string.h>

/***** Defines *****/

#define PACKET_INTERVAL     5000000  /* Set packet interval to 500000us or 500ms */

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

//static uint8_t packet[PAYLOAD_LENGTH];
static uint16_t seqNumber;

/*--------------------------------edited by Prem------------------------------*/
typedef unsigned char BYTE;
//char ascii_str[] = "Hello 12.34!";
static uint8_t PAYLOAD_LENGTH = 30;
static BYTE packet[30]; 
static char ambient[] = "Temp: ";
static char battery[] = " Bat: ";
static char pir[] = " PIR: ";
static char data[30] = {0};
/*--------------------------------edited by Prem------------------------------*/

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
/*--------------------------------edited by Prem------------------------------*/
//Power supply pin config for the thermal sensor.
static const PIN_Config thermalPins[] = {
  IOID_25          | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH   | PIN_PUSHPULL | PIN_DRVSTR_MAX, /*DIO1 THERMAL IN*/
  PIN_TERMINATE
};
//register pin handle Thermal. 
static PIN_Handle thermalpinHandle;
//register pin state Thermal.
static PIN_State  thermalpinState;

//Power supply pin config for the pir sensor.
static const PIN_Config PIRPins[] = {
  IOID_24 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,       /*DIO24 PIR IN*/
  PIN_TERMINATE
};
//register pin handle PIR.
static PIN_Handle PIRpinHandle;
//register pin state PIR.
static PIN_State  PIRpinState;

// ADC handle
static ADC_Handle adc;
// ADC params
static ADC_Params params;

// count the pir hits
static uint8_t pirCount=0;
// reference is obj + 2.
static float reference_temp=0;
// variable that stores the ambient temp
static uint32_t surface_temp=0;

static float temp_calibration=0;       //this parameter was used to calibrate the temperature

static float temperature_range=10;    //we make a map of temperature-voltage according to sensor datasheet. 10 is the temperature step when sensor and 
//object distance is 9CM.
static float offset_vol=0.014;        //this parameter was used to set the mid level voltage,when put the sensor in normal environment after 10 min,
//the sensor output 0.For example,the surrounding temperature is 29?,but the result is 27? via the sensor,
//you should set the reerence to 0.520 or more,according to your sensor to change.
//the unit is V
static float tempValue = 0; 
static float objtValue= 0;  
static float current_temp=0;
static float temp=0;
//static float temp1=0;
//static float temp2=0;
static unsigned int temp3=0;
static const float reference_vol=0.500;
//static unsigned char clear_num=0;//when use lcd to display
static float R=0;
static float voltage=0;

static long res[100]={
  318300,302903,288329,274533,261471,249100,237381,226276,215750,205768,
  196300,187316,178788,170691,163002,155700,148766,142183,135936,130012,
  124400,119038,113928,109059,104420,100000,95788,91775,87950,84305,
  80830,77517,74357,71342,68466,65720,63098,60595,58202,55916,
  53730,51645,49652,47746,45924,44180,42511,40912,39380,37910,
  36500,35155,33866,32631,31446,30311,29222,28177,27175,26213,
  25290,24403,23554,22738,21955,21202,20479,19783,19115,18472,
  17260,16688,16138,15608,15098,14608,14135,13680,13242,12819,
  12412,12020,11642,11278,10926,10587,10260,9945,9641,9347,
  9063,8789,8525,8270,8023,7785,7555,7333,7118,6911};

static float obj [13][12]={
  /*0*/             { 0,-0.274,-0.58,-0.922,-1.301,-1.721,-2.183,-2.691,-3.247,-3.854,-4.516,-5.236}, //
  /*1*/             { 0.271,0,-0.303,-0.642,-1.018,-1.434,-1.894,-2.398,-2.951,-3.556,-4.215,-4.931},  //?surrounding temperature,from -10,0,10,...100
  /*2*/             { 0.567,0.3,0,-0.335,-0.708,-1.121,-1.577,-2.078,-2.628,-3.229,-3.884,-4.597},   //?object temperature,from -10,0,10,...110
  /*3*/             { 0.891,0.628,0.331,0,-0.369,-0.778,-1.23,-1.728,-2.274,-2.871,-3.523,-4.232},
  /*4*/             { 1.244,0.985,0.692,0.365,0,-0.405,-0.853,-1.347,-1.889,-2.482,-3.13,-3.835},
  /*5*/             { 1.628,1.372,1.084,0.761,0.401,0,-0.444,-0.933,-1.47,-2.059,-2.702,-3.403},
  /*6*/             { 2.043,1.792,1.509,1.191,0.835,0.439,0,-0.484,-1.017,-1.601,-2.24,-2.936},
  /*7*/             { 2.491,2.246,1.968,1.655,1.304,0.913,0.479,0,-0.528,-1.107,-1.74,-2.431},
  /*8*/             { 2.975,2.735,2.462,2.155,1.809,1.424,0.996,0.522,0,-0.573,-1.201,-1.887},
  /*9*/             { 3.495,3.261,2.994,2.692,2.353,1.974,1.552,1.084,0.568,0,-0.622,-1.301},
  /*10*/            { 4.053,3.825,3.565,3.27,2.937,2.564,2.148,1.687,1.177,0.616,0,-0.673},
  /*11*/            { 4.651,4.43,4.177,3.888,3.562,3.196,2.787,2.332,1.829,1.275,0.666,0},
  /*12*/            { 5.29,5.076,4.83,4.549,4.231,3.872,3.47,3.023,2.527,1.98,1.379,0.72}
};

/* Pin driver handles */
static PIN_Handle pirPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State pirPinState;

/*
 * Application PIR input pin configuration table:
 *   - PIR interrupts are configured to trigger on both edge.
 */
PIN_Config pirPinTable[] = {
    IOID_23  | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_BOTHEDGES,
    PIN_TERMINATE
};

/***** Prototypes *****/
//edited by Prem
void string2ByteArray(char* input, BYTE* output);
static void Delay(uint32_t duration);
static float measureSurTemp( void );
static float measureObjectTemp( void );
static float binSearch(long x);
static float arraysearch(float x,float y);
static void Sensor_initialization( void );
static uint32_t measureBattery( void );
static void pirCallbackFxn(PIN_Handle handle, PIN_Id pinId);

/***** Function definitions *****/
/*--------------------------------edited by Prem------------------------------*/
void *mainThread(void *arg0)
{   
    
    /*Initialize PIR and Thermal sensors edited by Prem*/
    Sensor_initialization();
//    printf("%d\n", measureBattery());
//    printf("O %d\n", (int)measureObjectTemp());
//    printf("S %d\n", (int)measureSurTemp());
//    char result[2];  
//    sprintf(result, "%d", surface_temp); 
//    printf("%s\n", result);       
    RF_Params rfParams;
    RF_Params_init(&rfParams);
    
    pirPinHandle = PIN_open(&pirPinState, pirPinTable);
    if(!pirPinHandle) {
        /* Error initializing pir input pins */
        while(1);
    }
    
    /* Setup callback for pir input pins */
    if (PIN_registerIntCb(pirPinHandle, &pirCallbackFxn) != 0) {
        /* Error registering button callback function */
        while(1);
    }
    
   /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (ledPinHandle == NULL)
    {
        while(1);
    }

    RF_cmdIeeeTx.payloadLen = PAYLOAD_LENGTH;
    RF_cmdIeeeTx.pPayload = packet;
    RF_cmdIeeeTx.startTrigger.triggerType = TRIG_NOW;

    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdRadioSetup, &rfParams);

    /* Set the frequency */
//    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    while(1)
    {
        memset(data, 0, sizeof data);
        char temp[3];
        sprintf(temp, "%d", (int)measureSurTemp());
        char bat[4];
        sprintf(bat, "%d", (int)measureBattery());
        char count[3];
        sprintf(count, "%d", (int)pirCount);
        strcat(data, ambient); 
        strcat(data, temp);
        strcat(data, battery);
        strcat(data, bat);
        strcat(data, pir);
        strcat(data, count);
        //converting string to BYTE[]
        string2ByteArray(data, packet);
        
        RF_ScheduleCmdParams scheduleParam;
        RF_ScheduleCmdParams_init(&scheduleParam);

        RF_CmdHandle txCmd = RF_runScheduleCmd(rfHandle, (RF_Op*)&RF_cmdIeeeTx, &scheduleParam, NULL, 0);


#ifndef POWER_MEASUREMENT
        PIN_setOutputValue(ledPinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
#endif        
        /* Power down the radio */
        RF_yield(rfHandle);

#ifdef POWER_MEASUREMENT
        /* Sleep for PACKET_INTERVAL s */
        sleep(PACKET_INTERVAL);
#else
        /* Sleep for PACKET_INTERVAL us */
        usleep(PACKET_INTERVAL);
#endif

    }
}

/*--------------------------------edited by Prem------------------------------*/
//function to convert string to byte array
void string2ByteArray(char* input, BYTE* output)
{
    int loop;
    int i;
    /* Create packet with incrementing sequence number and random payload */
    output[0] = (uint8_t)(seqNumber >> 8);
    output[1] = (uint8_t)(seqNumber++);
    
    loop = 0;
    i = 2;
    
    while(input[loop] != '\0')
    {
        output[i++] = input[loop++];
    }
}
/*--------------------------------edited by Prem------------------------------*/
//--------------------edited by prem Sensor_initialization--------------------//
/*******************************************************************************
* @fn          Sensor_initialization
*
* @brief       Initialize the Sensors
*
* @param       none
*
* @return      none
*/

static void Sensor_initialization(void)
{ 
  //Power on the Thermal Sensor.
  thermalpinHandle = PIN_open(&thermalpinState, thermalPins);
  //Power On PIR Sensor.
  PIRpinHandle = PIN_open(&PIRpinState, PIRPins);
  
  //Delay of 400 milliseconds for Thermal Sensor to initialize.
  Delay(400);
  //Measure Surface(ambient) Temperature 50 times.
  for(uint8_t i=0;i<100;i++)
  {
    measureSurTemp();
  }
  //Set reference temperature.  
  //  reference_temp =  (measureSurTemp() + 1.00);
  //Saving the Surface(ambient) Temperature convert float to int.
  surface_temp = (int)measureSurTemp();
  //Measure Object Temperature 100 times.
  for(uint8_t i=0;i<100;i++)
  {
    measureObjectTemp();
  }
  //Set dynamic reference temp
  //  reference_temp =  (measureObjectTemp() + 2.00);
  //if dynamic reference temp is less than ambient temp them..
  if(reference_temp<surface_temp)
  {
    reference_temp=surface_temp+2.0;
  }
  //    //Debug code.    
//      printf("O %f\n", measureObjectTemp());
//      printf("S %d\n", surface_temp);
  //    printf("R %f\n", reference_temp);
  
  //Power off the Thermal sensor.
//  PIN_close(thermalpinHandle);
  
  
}
//--------------------edited by prem Sensor_initialization--------------------//

//--------------------------edited by prem delay------------------------------//
/*******************************************************************************
* @fn          delay
*
* @brief       Delays execution
*
* @param       Duration of Delay in millisec
*
* @return      none
*/
static void Delay(uint32_t duration)
{
  //iterator
  uint32_t i=0;
  //enter while loop and execute NOP cycles
  while(i<(1000*duration))
  {
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");i++;
  }
}
//--------------------------edited by prem delay------------------------------//

//------------------------edited by prem measureSurTemp-----------------------//
/*******************************************************************************
* @fn          measureSurTemp
*
* @brief       measurres the object temperature.
*
* @param       none
*
* @return      none
*/

static float measureSurTemp()
{ 
  //variable that store the ADC values from the Thermal sensor.
  uint16_t adcValue;
  unsigned char i=0;
  //float current_temp1=0;	  
  int signal=0;	  
  tempValue=0;
  //initializes the ADC driver (sets params to default)
  ADC_init();
  //initialize the ADC peripheral specified by the particular index value.
  //Board_ADC0 implies DIO26/A3
  adc = ADC_open(Board_ADC0, &params);
  //Opened successfully
  if (adc != NULL) 
  {
    for(i=0;i<10;i++)       //	  
    {
      //Set adcValue to NULL
      adcValue=NULL;
      //perform ADC single channel single sample conversion
      if (!ADC_convert(adc, &adcValue)) 
      {
        //printf("%d\n",adcValue);
        tempValue+= adcValue;  
      }
    }
  }
  //close ADC driver.
  ADC_close(adc);
  //Averaging processing
  tempValue=tempValue/10;	  
  temp = tempValue*4.3/4095;	  
  R=2000000*temp/(2.50-temp);	  
  signal=binSearch(R);	  
  current_temp=signal-1+temp_calibration+(res[signal-1]-R)/(res[signal-1]-res[signal]);
  return current_temp;
  
}
//------------------------edited by prem measureSurTemp-----------------------//

//------------------------edited by prem measureObjectTemp--------------------//
/*******************************************************************************
* @fn          measureObjectTemp
*
* @brief       measurres the object temperature.
*
* @param       none
*
* @return      none
*/

static float measureObjectTemp()
{
  //variable that store the ADC values from the Thermal sensor.
  uint16_t adcValue;
  unsigned char i=0;  
  //unsigned char j=0;  
  float sur_temp=0;  
  unsigned int array_temp=0;  
  float temp1,temp2; 
  float final_temp=0;
  objtValue=0;	
  //initializes the ADC driver (sets params to default)
  ADC_init();
  //initialize the ADC peripheral specified by the particular index value.
  //Board_ADC1 implies DIO27/A4
  adc = ADC_open(Board_ADC1, &params);
  //Opened successfully
  if (adc != NULL) 
  {
    for(i=0;i<10;i++)
    {
      adcValue=NULL;
      //perform ADC single channel single sample conversion
      if (!ADC_convert(adc, &adcValue)) 
      {
        //printf("%d\n",adcValue);
        objtValue+= adcValue;
      }
      
    }
  }
  //close ADC driver.
  ADC_close(adc);
  //Averaging processing
  objtValue=objtValue/10;     
  temp1=objtValue*4.2/4095;//+objt_calibration; 
  sur_temp=temp1-(reference_vol+offset_vol);             
  array_temp=arraysearch(current_temp,sur_temp*1000);        
  temp2=current_temp;        
  temp1=(temperature_range*voltage)/(obj[array_temp+1][(int)(temp2/10)+1]-obj[array_temp][(int)(temp2/10)+1]);        
  final_temp=temp2+temp1;        
  return final_temp;
  
}
//------------------------edited by prem measureObjectTemp--------------------//

//------------------------edited by prem binSearch----------------------------//
/*******************************************************************************
* @fn          binSearch
*
* @brief       
*
* @param       long x
*
* @return      float
*/
static float binSearch(long x)// this function used for measure the surrounding temperature
{
  int low,mid,high;
  low=0;
  //mid=0;
  high=100;
  while (low<=high)
  {
    mid=(low+high)/2;
    if(x<res[mid])
      low= mid+1;
    else//(x>res[mid])
      high=mid-1;
  }
  return (int)mid;
}
//------------------------edited by prem binSearch----------------------------//

//------------------------edited by prem arraysearch--------------------------//
/*******************************************************************************
* @fn          arraysearch
*
* @brief       
*
* @param       float x,float y
*
* @return      float
*/
static float arraysearch(float x,float y)//x is the surrounding temperature,y is the object temperature
{
  int i=0;
  float tem_coefficient=100;//Magnification of 100 times	
  i=(x/10)+1;//Ambient temperature			
  voltage=(float)y/tem_coefficient;//the original voltage		
  //Serial.print("sensor voltage:\t");		
  //Serial.print(voltage,5);	
  //Serial.print("V");			
  for(temp3=0;temp3<13;temp3++)		
  {			
    if((voltage>obj[temp3][i])&&(voltage<obj[temp3+1][i]))				
    {			
      return temp3;					
    }			
  }
}
//------------------------edited by prem arraysearch--------------------------//

//---------------------edited by prem measureBattery--------------------------//
static uint32_t measureBattery( void )
{
  //Measure Battery Voltage.
  uint32_t percent = AONBatMonBatteryVoltageGet();
  percent = (percent * 125) >> 5;    
  percent = (uint32_t)(((percent-2500)* 100) / 850);
  return percent;
}
//---------------------edited by prem measureBattery--------------------------//

/*
 *  ======== pirCallbackFxn ========
 *  Pin interrupt Callback function board buttons configured in the pinTable.
 *  If Board_PIN_LED3 and Board_PIN_LED4 are defined, then we'll add them to the PIN
 *  callback function.
 */
static void pirCallbackFxn(PIN_Handle handle, PIN_Id pinId) {
    uint32_t currVal = 0;

    /* Debounce logic, only toggle if the button is still pushed (low) */
    CPUdelay(8000*50);
    if (!PIN_getInputValue(pinId)) {
        /* Toggle LED based on the button pressed */
        switch (pinId) {
            case IOID_23:
                //increment pirCount
                pirCount++;
                break;

            default:
                /* Do nothing */
                break;
        }
    }
}