/*
 * Copyright (c) 2015-2018, Texas Instruments Incorporated
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

#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/net/wifi/slnetifwifi.h>
/* For usleep() */
#include <unistd.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <ctype.h>

/* POSIX Header files */
#include <pthread.h>

/* Driver Header files */
#include <ti/drivers/PWM.h>
#include <ti/drivers/ADC.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART.h>

#include <third_party/fatfs/ffcio.h>
#include <ti/drivers/SD.h>
#include <ti/drivers/SDFatFS.h>

#include <ti/display/Display.h>

#include <ti/drivers/SPI.h>

// TI-Driver includes
#include "ti_drivers_config.h"
#include <file.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include<unistd.h>

/* Buffer size used for the file copy process */
#ifndef CPY_BUFF_SIZE
#define CPY_BUFF_SIZE       2048
#endif

/* String conversion macro */
#define STR_(n)             #n
#define STR(n)              STR_(n)

/* Drive number used for FatFs */
#define DRIVE_NUM           1

const char pcycle_file[] = "fat:"STR(DRIVE_NUM)":pcycle.txt";
//char temp_file[] = "fat:"STR(DRIVE_NUM)":temp00001_10000.txt";
//char temp_file[20] ;
char temp_file[50];
const char outputfile[] = "fat:"STR(DRIVE_NUM)":output.txt";


#define APPLICATION_NAME                      ("TCP Echo")
#define APPLICATION_VERSION                   ("1.0.0.0")
#define DEVICE_ERROR                          ("Device error, please refer \"DEVICE ERRORS CODES\" section in errors.h")
#define WLAN_ERROR                            ("WLAN error, please refer \"WLAN ERRORS CODES\" section in errors.h")
#define SL_STOP_TIMEOUT                       (200)

#define TCPPORT 							  (8080)
#define SPAWN_TASK_PRIORITY                   (9)
#define TASK_STACK_SIZE                       (2048)
#define SLNET_IF_WIFI_PRIO                    (5)
#define SLNET_IF_WIFI_NAME                    "CC32xx"

#define SSID_NAME                             "One"                  /* AP SSID */
#define SECURITY_TYPE                         SL_WLAN_SEC_TYPE_OPEN
#define SECURITY_KEY                          ""

//#define SSID_NAME                             "DemoAP"                  /* AP SSID */
//#define SECURITY_TYPE                         SL_WLAN_SEC_TYPE_WPA_WPA2 /* Security type could be SL_WLAN_SEC_TYPE_OPEN */
//#define SECURITY_KEY                          "12345678"                /* Password of the secured AP */

#define THREADSTACKSIZE   (768)

//PWM & Duty cycle parameters

double duty = 50,dutyv,pwmv,boost_d;//boost duty fixed
double swf=100,d_val=100; //swf =100 = default value
double dmin = 10;
float a=1;
double p=1;



/* ADC conversion result variables */
uint16_t adcValue0;
uint32_t adcValue0MicroVolt;
uint16_t adcValue1;
uint32_t adcValue1MicroVolt;
uint16_t adcValue2;
uint32_t adcValue2MicroVolt;
uint16_t adcValue3;
uint32_t adcValue3MicroVolt;

//ADC Source and Load Global variables
double adc_iload,adc_isrc,adc_vsrc,adc_vload;

//Global variables
double src_i=0,load_v=0,load_i=0,src_v=0;
double source_i=0,old_d,pdiff,PWM_Temp = 50;
double idiff,ddiff,vdiff;
double old_load_v=0;
double old_load_i=0;
double new_p,old_p,load_p,wless_p;
char hello[10];
char str[10];
static Display_Handle display;

//Global mode control parameters
int buck_boost = 0; // 0 - charging & 1 - discharging
int store = 0;  // 0- don't store, 1 - store
int store_rst=0;
int mppf=1;
int mpc=0; // MPP cycle flag
int fstop=0;
int X=0;

//Count variable to identify which loop is running
double ct=0,ct1=0,ct2=0,ct3=0,ct4=0,ct5=0,ct6=0,ct7=0,ct8=0,ct9=0,ct0=0;


//Global wireless variables
extern double wload_v;
double wless_v=0;
double rip_v_ref = 1.5,temp_v_ref = 1.5; // Wireless voltage reference closed loop

// PID wireless side
double kd=0.00002,der,kd_temp=0.00002,KK2_temp=3.1,KK2_temp2=5.1,pi_gain_temp=2;

// PID Buck mode
float err_rip=0, err1_rip=0, u_rip=100, u1_rip=100; // swf =200KHZ Defined for Discharging Mode Ripple Voltage PID.
float KK1=3,KK2=5.1,kd1=1,kd2=1.1;
float PIgain_rip=2;

//Temp debug variables
double load_it;


// PID boost side
float e0=0, e1=0, u0=0, u1=50;
float k1 = 1, k2=1.1;
double boost_ref = 20;
float boost_gain = 0.2;

// PID constant voltage charging
//float e0=0, e1=0, u0=0, u1=0;
//float k1 = 2, k2=3.1;
//double boost_Ref = 35;
//float boost_gain = 5;

//ARRAYS definition
double loadi[1000]={0},loadv[1000]={0},dut[1000]={0},wles_p[1000]={0},wles_v[1000]={0},sw[1000]={0},srci[1000]={0},srcv[1000]={0},srcp[1000]={0},loadp[1000]={0},eff[1000]={0},timestamp[1000]={0};
int n=0,k=0;

// Global PWM and ADC handle
PWM_Handle pwm1 = NULL;
ADC_Handle   adc,adc1,adc2,adc3;

//SD Storage
unsigned int totalmbCopied = 0;
int arr=1;
int p_count=0,newp_count=0;
int limit_count=1;
double file1mb;
unsigned int file1size;

typedef struct TaskArgs {
    void (*arg0)();
    uintptr_t arg1;
    uintptr_t arg2;
} TaskArgs;

pthread_t tcpThread = (pthread_t)NULL;
pthread_t spawn_thread = (pthread_t)NULL;
pthread_t tcpworker_thread = (pthread_t)NULL;
int32_t mode;
Display_Handle display;

extern void tcpHandler(uint32_t arg0, uint32_t arg1);
extern void tcpWorker(uint32_t arg0, uint32_t arg1);
extern int32_t ti_net_SlNet_initConfig();





/* Callback used for updating PWM */
void timerCallback(Timer_Handle myHandle, int_fast16_t status);
void timerCallback1(Timer_Handle myHandle, int_fast16_t status);


/* Set this to the current UNIX time in seconds */
const struct timespec ts = {
    .tv_sec = 1632150000,  // Fri Aug 20 2021 10:00:00 GMT-0500 (Central Daylight Time)  (value offset by a month - Mon Sep 20 2021 10:00:00 GMT-0500 (Central Daylight Time))
    .tv_nsec = 0
};

/* File name prefix for this filesystem for use with TI C RTS */
char fatfsPrefix[] = "fat";

unsigned char cpy_buff[CPY_BUFF_SIZE];


/*
 *  ======== fatfs_getFatTime ========
 */
int32_t fatfs_getFatTime(void)
{
    time_t seconds;
    uint32_t fatTime;
    struct tm *pTime;

    /*
     *  TI time() returns seconds elapsed since 1900, while other tools
     *  return seconds from 1970.  However, both TI and GNU localtime()
     *  sets tm tm_year to number of years since 1900.
     */
    seconds = time(NULL);

    pTime = localtime(&seconds);

    /*
     *  localtime() sets pTime->tm_year to number of years
     *  since 1900, so subtract 80 from tm_year to get FAT time
     *  offset from 1980.
     */
    fatTime = ((uint32_t)(pTime->tm_year - 80) << 25) |
        ((uint32_t)(pTime->tm_mon) << 21) |
        ((uint32_t)(pTime->tm_mday) << 16) |
        ((uint32_t)(pTime->tm_hour) << 11) |
        ((uint32_t)(pTime->tm_min) << 5) |
        ((uint32_t)(pTime->tm_sec) >> 1);

    return ((int32_t)fatTime);
}



void data_update(void)
{

    int_fast16_t res,res1,res2,res3;
    double adc_v,adc_v1;
    double i=0,i1=0,v=0,v1=0;
            //double v=0,i=0;

                 //sum_of_ADC_samples_Array = 0;
                // double adcsum=0,adcsum1=0;
                int numberOfSamples = 128;
                int j = 0;

        double adcsum=0,adcsum1=0,adcsum2=0,adcsum3=0;
                for (j = 0; j < numberOfSamples; j++) {
                    res = ADC_convert(adc, &adcValue0);
                    res1 = ADC_convert(adc1, &adcValue1);
                    res2 = ADC_convert(adc2, &adcValue2);
                    res3 = ADC_convert(adc3, &adcValue3);
                    while (res3 != ADC_STATUS_SUCCESS) {}
                   // c++;
                    adcValue0MicroVolt = ADC_convertRawToMicroVolts(adc, adcValue0);
                    adcValue1MicroVolt = ADC_convertRawToMicroVolts(adc1, adcValue1);
                    adcValue2MicroVolt = ADC_convertRawToMicroVolts(adc2, adcValue2);
                    adcValue3MicroVolt = ADC_convertRawToMicroVolts(adc3, adcValue3);

                    adcsum+=adcValue0MicroVolt;
                    adcsum1+=adcValue1MicroVolt;
                    adcsum2+=adcValue2MicroVolt;
                    adcsum3+=adcValue3MicroVolt;
                    // PWM_setDuty(pwm1, duty);
                    // load_p = load_v*load_i;
                      }

                // divide by the number of samples to find the average value
                //  ADC_A0 = sum_of_ADC_samples_Array/ 64;


                adc_isrc = adcsum3/numberOfSamples;
                adc_iload = adcsum2/numberOfSamples;
                adc_v = adcsum/numberOfSamples;   //Source Voltage
                adc_v1 = adcsum1/numberOfSamples;   //Load Voltage


                i = adc_iload/1000000 ;
                i1 = adc_isrc/1000000;
                v = adc_v/1000000 ;
                v1 = adc_v1/1000000 ;

                //ADC CALIBRATION
                src_v = 0.8994*(64.742*v - 0.9598)+0.7545;


                 load_v = v1 *2.4667*3.2 + 0.1;
                 load_it = i*2.4667;  //(load_i-1.5)/0.1
                 src_i = i1*2.4667;

                 src_v = 0.9732*src_v+0.9694;
                 //load_v = 2.3891*load_v-3.6051;
                 //load_i = 11.299*load_i-5.5392;
                 //load_i = (i*2.4667-1.5+.09)/0.1;
                 //src_i = 8.1825*src_i-4.1137;
                 //wless_v = 9.075*wload_v + .0005;
                 //load_i=(load_it-1.5)/0.1;
                 load_i=(load_it-1.5-0.059)/0.199;
                 wless_v = 2.4667*wload_v*2.4667*1.0645;
                 load_p = load_v*load_i;
                 wless_p = (wless_v*wless_v)/3.3;


                /*if(n<5000){
                loadi[n]= load_i;
                loadv[n]= load_v;
                srcv[n++]= src_v;
                }
                else
                    n=0;*/

                //Safety condition
                /*    if(load_v > 4.5 || load_i > 5 || wless_v > 10 )
                        {
                            /* Turn oFF ENABLE
                             GPIO_write(CONFIG_GPIO_ena, 0);
                             GPIO_write(CONFIG_GPIO_LED_0,1);
                             GPIO_write(CONFIG_GPIO_LED_1,1);
                             GPIO_write(CONFIG_GPIO_LED_2,1);
                        }*/
                if (load_v >= 4.1)
                {
                     //Turn oFF ENABLE
                    // GPIO_write(CONFIG_GPIO_ena, 0);
                    buck_boost = 1;
                    //mode = 1;

                }
                else if (load_v <=3.1)
                {
                    buck_boost = 0;
                    //mode = 0;

                }

    }


void boost_pwm(void)
{
    //*** Vboost PID ***//
                        e1 = e0;
                        //u1 = u0;
                        u1 = PWM_Temp;
                        e0 = boost_ref - src_v;

                        //u0 = (u1 + boost_gain*(k1*e0 - k2*e1));
                        PWM_Temp = (u1 + boost_gain*(k1*e0 - k2*e1));
                   /*     if (u0>= 95) //fmax=380Khz
                        u0=95;
                        else if (u0<=5) //fmin=20Khz
                        u0=5;
*/
                        if (PWM_Temp>= 95) //fmax=380Khz
                          PWM_Temp=95;
                        else if (PWM_Temp<=5) //fmin=20Khz
                            PWM_Temp=5;
                       // dutyv = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * u0) / 100);
                        dmin = PWM_Temp;
                      //  pwmv = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * u0) / 100);
                        pwmv = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * PWM_Temp) / 100);
                    //    store_update();
                        //PWM_setPeriod(pwm1,swf*1000);
                        PWM_setDutyAndPeriod(pwm1,pwmv,swf*1000);
                       // PWM_setDutyAndPeriod(pwm1,dutyv,swf*1000);
                        ct8++;

}

void pwm_f(void)
{
    //*** Vripple PID ***//
                       // soft_start_ref =wless_v+soft_start_p;
                        err1_rip = err_rip;
                        u1_rip = u_rip;

                        //err_rip = rip_v_ref - wless_v;
                /*        if (soft_start_ref!=rip_v_ref)
                        {
                            if(soft_start_ref - rip_v_ref >0)
                            {
                                soft_start_p= -(soft_start_ref - rip_v_ref)/10;
                            }
                        }*/
                        err_rip = (temp_v_ref-wless_v);
                        //der = (err_rip – err1_rip)/0.000001;
                   /*     if(soft_start!=1)
                        {
                            soft_start--;
                        }
                        else
                            KK2 = 3.1;
                        if(temp_v_ref<rip_v_ref){
                            temp_v_ref+=0.05;

                        }*/
                        //else
                            //buck_boost=0;

                      /* if(err_rip > -0.05 && err_rip <0.05)
                        {
                             KK2 = KK2_temp;
                             PIgain_rip = 1;
                             kd=0;

                         }
                         else
                          {
                             KK2 = KK2_temp2;
                             kd = kd_temp;
                             PIgain_rip = pi_gain_temp;
                          }*/

                        u_rip = u1_rip + PIgain_rip*(KK1*err_rip - KK2*err1_rip)+kd*((err_rip - err1_rip)*1000000);
                        //u_rip = (u1_rip + PIgain_rip*(KK1*err_rip - KK2*err1_rip))*a;

                        if (u_rip> 380) //fmax=380Khz
                        u_rip=380;
                        else if (u_rip<20) //fmin=20Khz
                        u_rip=20;

                        swf=u_rip;
                        //swf = 125;

                 /*      if (k<=5000){
                        sw[k++]=swf;
                        }*/
                        //PWM_setPeriod(pwm1,swf*1000);
                        PWM_setDutyAndPeriod(pwm1,pwmv,swf*1000);
                        //PWM_setDutyAndPeriod(pwm1,dutyv,swf*1000);

}

        void Adj_PWM(void)
        {

                          // Assign the current duty cycle value of PWM1 CMPA to PWM_Temp
if(load_i < 0){
    load_i = 0;
}
            new_p = load_i*load_v;
            vdiff = load_v - old_load_v;
            ddiff = PWM_Temp - old_d;
            pdiff = new_p - old_p;
            idiff = load_i - old_load_i;




           // if (vdiff==0) {
           // if (pdiff==0){
              if (idiff==0){
                old_d = PWM_Temp;
                if(X == 1)
                {
                    goto inc;
                }
                else
                    goto dec;
            }
            else{
                old_load_v = load_v;                  // Assign new input volts value to old input volts value
                old_load_i = load_i;
                old_p = new_p;
                old_d = PWM_Temp;
                //if ((vdiff > 0 && ddiff > 0)||(vdiff < 0 && ddiff < 0))
               // if ((pdiff > 0 && ddiff > 0)||(pdiff < 0 && ddiff < 0))
                if ((idiff > 0 && ddiff > 0)||(idiff < 0 && ddiff < 0))
                {
          inc:          PWM_Temp += p;       // Increase Duty Cycle
                    ct1++;
                    X=1;
                    if (PWM_Temp > 90) {
                            PWM_Temp = 90;                 // Necessary to prevent too greater PWM value
                        }
                }
                else
                {
          dec:          PWM_Temp -= p;       // Decrease Duty Cycle
                        ct2++;
                        X=0;
                        if (PWM_Temp < dmin) {
                               PWM_Temp = dmin;                     // Necessary to prevent too smaller PWM value
                           }
                }
            }

              pwmv = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * PWM_Temp) / 100);

             // store_update();

              //dutyv = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * duty) / 100);
             // PWM_setDuty(pwm1, pwmv);    // Adjust PWM1 duty cycle
             PWM_setDutyAndPeriod(pwm1,pwmv,swf*1000);
             // PWM_setDutyAndPeriod(pwm1,dutyv,swf*1000);



        }
/*
 *  ======== printError ========
 */
void printError(char *errString, int code)
{
    Display_printf(display, 0, 0, "Error! code = %d, Description = %s\n", code,
            errString);
    while(1);
}

/*!
    \brief          SimpleLinkNetAppEventHandler

    This handler gets called whenever a Netapp event is reported
    by the host driver / NWP. Here user can implement he's own logic
    for any of these events. This handler is used by 'network_terminal'
    application to show case the following scenarios:

    1. Handling IPv4 / IPv6 IP address acquisition.
    2. Handling IPv4 / IPv6 IP address Dropping.

    \param          pNetAppEvent     -   pointer to Netapp event data.

    \return         void

    \note           For more information, please refer to: user.h in the porting
                    folder of the host driver and the  CC31xx/CC32xx NWP programmer's
                    guide (SWRU455) section 5.7

*/
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    int32_t             status = 0;
    pthread_attr_t      pAttrs;
    struct sched_param  priParam;

    if(pNetAppEvent == NULL)
    {
        return;
    }

    switch(pNetAppEvent->Id)
    {
        case SL_NETAPP_EVENT_IPV4_ACQUIRED:
        case SL_NETAPP_EVENT_IPV6_ACQUIRED:

            /* Initialize SlNetSock layer with CC3x20 interface                      */
            status = ti_net_SlNet_initConfig();
            if(0 != status)
            {
                Display_printf(display, 0, 0, "Failed to initialize SlNetSock\n\r");
            }

            if(mode != ROLE_AP)
            {
                Display_printf(display, 0, 0,"[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                            "Gateway=%d.%d.%d.%d\n\r",
                            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,3),
                            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,2),
                            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,1),
                            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,0),
                            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,3),
                            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,2),
                            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,1),
                            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,0));

                pthread_attr_init(&pAttrs);
                priParam.sched_priority = 1;
                status = pthread_attr_setschedparam(&pAttrs, &priParam);
                status |= pthread_attr_setstacksize(&pAttrs, TASK_STACK_SIZE);
                status = pthread_create(&tcpThread, &pAttrs, (void *(*)(void *))tcpHandler, (void*)TCPPORT);
                if(status)
                {
                    printError("Task create failed", status);
                }
            }
            break;
        default:
            break;
   }
}

/*!
    \brief          SimpleLinkFatalErrorEventHandler

    This handler gets called whenever a socket event is reported
    by the NWP / Host driver. After this routine is called, the user's
    application must restart the device in order to recover.

    \param          slFatalErrorEvent    -   pointer to fatal error event.

    \return         void

    \note           For more information, please refer to: user.h in the porting
                    folder of the host driver and the  CC31xx/CC32xx NWP programmer's
                    guide (SWRU455) section 17.9.

*/
void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *slFatalErrorEvent)
{
    /* Unused in this application */
}
/*!
    \brief          SimpleLinkNetAppRequestMemFreeEventHandler

    This handler gets called whenever the NWP is done handling with
    the buffer used in a NetApp request. This allows the use of
    dynamic memory with these requests.

    \param          pNetAppRequest     -   Pointer to NetApp request structure.

    \param          pNetAppResponse    -   Pointer to NetApp request Response.

    \note           For more information, please refer to: user.h in the porting
                    folder of the host driver and the  CC31xx/CC32xx NWP programmer's
                    guide (SWRU455) section 17.9.

    \return         void

*/
void SimpleLinkNetAppRequestMemFreeEventHandler(uint8_t *buffer)
{
    /* Unused in this application */
}

/*!
    \brief          SimpleLinkNetAppRequestEventHandler

    This handler gets called whenever a NetApp event is reported
    by the NWP / Host driver. User can write he's logic to handle
    the event here.

    \param          pNetAppRequest     -   Pointer to NetApp request structure.

    \param          pNetAppResponse    -   Pointer to NetApp request Response.

    \note           For more information, please refer to: user.h in the porting
                    folder of the host driver and the  CC31xx/CC32xx NWP programmer's
                    guide (SWRU455) section 17.9.

    \return         void

*/
void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t *pNetAppRequest, SlNetAppResponse_t *pNetAppResponse)
{
    /* Unused in this application */
}

/*!
    \brief          SimpleLinkHttpServerEventHandler

    This handler gets called whenever a HTTP event is reported
    by the NWP internal HTTP server.

    \param          pHttpEvent       -   pointer to http event data.

    \param          pHttpEvent       -   pointer to http response.

    \return         void

    \note           For more information, please refer to: user.h in the porting
                    folder of the host driver and the  CC31xx/CC32xx NWP programmer's
                    guide (SWRU455) chapter 9.

*/
void SimpleLinkHttpServerEventHandler(SlNetAppHttpServerEvent_t *pHttpEvent,
                                      SlNetAppHttpServerResponse_t *pHttpResponse)
{
    /* Unused in this application */
}

/*!
    \brief          SimpleLinkWlanEventHandler

    This handler gets called whenever a WLAN event is reported
    by the host driver / NWP. Here user can implement he's own logic
    for any of these events. This handler is used by 'network_terminal'
    application to show case the following scenarios:

    1. Handling connection / Disconnection.
    2. Handling Addition of station / removal.
    3. RX filter match handler.
    4. P2P connection establishment.

    \param          pWlanEvent       -   pointer to Wlan event data.

    \return         void

    \note           For more information, please refer to: user.h in the porting
                    folder of the host driver and the  CC31xx/CC32xx NWP programmer's
                    guide (SWRU455) sections 4.3.4, 4.4.5 and 4.5.5.

    \sa             cmdWlanConnectCallback, cmdEnableFilterCallback, cmdWlanDisconnectCallback,
                    cmdP2PModecallback.

*/
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    /* Unused in this application */
}
/*!
    \brief          SimpleLinkGeneralEventHandler

    This handler gets called whenever a general error is reported
    by the NWP / Host driver. Since these errors are not fatal,
    application can handle them.

    \param          pDevEvent    -   pointer to device error event.

    \return         void

    \note           For more information, please refer to: user.h in the porting
                    folder of the host driver and the  CC31xx/CC32xx NWP programmer's
                    guide (SWRU455) section 17.9.

*/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /* Unused in this application */
}

/*!
    \brief          SimpleLinkSockEventHandler

    This handler gets called whenever a socket event is reported
    by the NWP / Host driver.

    \param          SlSockEvent_t    -   pointer to socket event data.

    \return         void

    \note           For more information, please refer to: user.h in the porting
                    folder of the host driver and the  CC31xx/CC32xx NWP programmer's
                    guide (SWRU455) section 7.6.

*/
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    /* Unused in this application */
}

void Connect(void)
{
    SlWlanSecParams_t   secParams = {0};
    int16_t ret = 0;
    secParams.Key = (signed char*)SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;
    Display_printf(display, 0, 0, "Connecting to : %s.\r\n",SSID_NAME);
    ret = sl_WlanConnect((signed char*)SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    if (ret)
    {
        printError("Connection failed", ret);
    }
}

void hostnet(void)
{
 _i16 Status;
_u8 Ssid[] = "Test_AP";
 Status = sl_WlanSet(SL_WLAN_CFG_AP_ID, SL_WLAN_AP_OPT_SSID, strlen(Ssid), Ssid);
if( Status )
{
    printError("Connection failed to start", Status);
}
}

void *TaskCreate(void (*pFun)(void *), char *Name, int Priority, uint32_t StackSize,uintptr_t Arg1, uintptr_t Arg2, uintptr_t argReserved)
{
    int32_t             status = 0;
    pthread_attr_t      pAttrs_tcp;
    struct sched_param  priParam;


    /* Start the TCP Worker  */
    pthread_attr_init(&pAttrs_tcp);
    priParam.sched_priority = Priority;
    status = pthread_attr_setschedparam(&pAttrs_tcp, &priParam);
    status |= pthread_attr_setstacksize(&pAttrs_tcp, StackSize);

    status = pthread_create(&tcpworker_thread, &pAttrs_tcp, (void *(*)(void *))pFun, (void *)Arg1);

    if (status < 0) {
        return (NULL);
    }

    return ((void *)!status);
}

void mainThread(void *pvParameters)
{

    int32_t             status = 0;
    pthread_attr_t      pAttrs_spawn;
    struct sched_param  priParam;

    SDFatFS_Handle sdfatfsHandle;
    /* Variables for the CIO functions */
    FILE *src, *dst, *tmp;
    /* Variables to keep track of the file copy progress */
    unsigned int bytesRead = 0;
    unsigned int bytesWritten = 0;
    unsigned int filesize;
    //double file1size;
    unsigned int totalBytesCopied = 0;


    PWM_Params paramss;

    Timer_Handle timer0,timer1;
    Timer_Params params2,params22;

    /* Call driver init functions */

                    Timer_init();
                    SDFatFS_init();

        /* Call driver init functions */
            GPIO_init();

            /* Configure the LED and button pins */
                GPIO_setConfig(CONFIG_GPIO_ena, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
            /* add_device() should be called once and is used for all media types */
                add_device(fatfsPrefix, _MSA, ffcio_open, ffcio_close, ffcio_read,
                    ffcio_write, ffcio_lseek, ffcio_unlink, ffcio_rename);

                /* Call driver init functions */
                    ADC_init();
                    Display_init();

                    /* Open the display for output */
                        display = Display_open(Display_Type_UART, NULL);
                        if (display == NULL) {
                            /* Failed to open display driver */
                            while (1);
                        }
                    /* Initialize real-time clock */
                        clock_settime(CLOCK_REALTIME, &ts);

        /* Call driver init functions. */
        PWM_init();

        ADC_Params   params,params1,params12,params3;
                       //int_fast16_t res,res1;

                       ADC_Params_init(&params);
                       adc = ADC_open(CONFIG_ADC_0, &params);

                       if (adc == NULL) {
                           Display_printf(display, 0, 0, "Error initializing CONFIG_ADC_0\n");
                           while (1);
                       }

                       ADC_Params_init(&params1);
                              adc1 = ADC_open(CONFIG_ADC_1, &params1);

                              if (adc1 == NULL) {
                                  Display_printf(display, 0, 0, "Error initializing CONFIG_ADC_1\n");
                                  while (1);
                              }

                              ADC_Params_init(&params12);
                                   adc2 = ADC_open(CONFIG_ADC_2, &params12);

                                   if (adc2 == NULL) {
                                       Display_printf(display, 0, 0, "Error initializing CONFIG_ADC_2\n");
                                       while (1);
                                   }
                                   ADC_Params_init(&params3);
                                   adc3 = ADC_open(CONFIG_ADC_3, &params3);
                                   if (adc3 == NULL) {
                                       Display_printf(display, 0, 0, "Error initializing CONFIG_ADC_3\n");
                                       while (1);
                                   }

                                   Display_printf(display, 0, 0, "Starting the fatsd example\n");
                                   Display_printf(display, 0, 0,"This example requires a FAT filesystem on the SD card.\n");
                                   Display_printf(display, 0, 0,"You will get errors if your SD card is not formatted with a filesystem.\n");

                                   /* Mount and register the SD Card */
                                   sdfatfsHandle = SDFatFS_open(CONFIG_SDFatFS_0, DRIVE_NUM);
                                   if (sdfatfsHandle == NULL) {
                                   Display_printf(display, 0, 0, "Error starting the SD card\n");
                                   while (1);
                                   }
                                   else {
                                   Display_printf(display, 0, 0, "Drive %u is mounted\n", DRIVE_NUM);
                                   }

                                   /* Try to open the source file */
                                   src = fopen(pcycle_file, "r");
                                   if (!src) {
                                   Display_printf(display, 0, 0, "Creating a new file \"%s\"...",pcycle_file);

                                   /* Open file for both reading and writing */
                                   src = fopen(pcycle_file, "w+");
                                   if (!src) {
                                   Display_printf(display, 0, 0,"Error: \"%s\" could not be created.\nPlease check the "
                                                  "Board.html if additional jumpers are necessary.\n",
                                                   pcycle_file);
                                   Display_printf(display, 0, 0, "Aborting...\n");
                                   while (1);
                                   }
                                   fprintf(src,"%d",arr);
                                   fflush(src);
                                   /* Reset the internal file pointer */
                                   rewind(src);
                                   Display_printf(display, 0, 0, "done\n");
                                  //fwrite(textarray, 1, strlen(textarray), src);
                                  //fwrite(&arr, sizeof(arr), 1, src);
                                   }
                                   else {
                                   Display_printf(display, 0, 0, "Using existing copy of \"%s\"\n",
                                                                              pcycle_file);
                                   src = fopen(pcycle_file, "r+");
                                   if (!src) {
                                   Display_printf(display, 0, 0,"Error: \"%s\" could not be created.\nPlease check the "
                                                                               "Board.html if additional jumpers are necessary.\n",
                                                                               pcycle_file);
                                   Display_printf(display, 0, 0, "Aborting...\n");
                                   while (1);
                                   }
                                   fscanf(src,"%d",&p_count);
                                   fflush(src);
                                   /* Reset the internal file pointer */
                                   rewind(src);
                                   newp_count = p_count+1;
                                   memset(temp_file, 0, 50);
                                   sprintf(temp_file, "fat:"STR(DRIVE_NUM)":T%d_%d.txt", newp_count, limit_count++); // puts string into buffer                                   fprintf(src,"%d",newp_count);
                                   fflush(src);
                                   /* Reset the internal file pointer */
                                   rewind(src);
                                   fprintf(src,"%d",newp_count);
                                   //fwrite(newp_count, sizeof(double), 1, pcycle_file);
                                   Display_printf(display, 0, 0, "done\n");
                                   }

                                   fseek(src, 0, SEEK_END);
                                   filesize = ftell(src);
                                   rewind(src);
                                   /* Close both inputfile[] and outputfile[] */
                                   fclose(src);
                                   // fclose(dst);
                                   Display_printf(display, 0, 0, "File \"%s\" size is (%u B) \n",pcycle_file,filesize);
        /** Setting up the timer in continuous callback mode that calls the callback
                     * function every 1,000,000 microseconds, or 1 second.*/

                    Timer_Params_init(&params2);
                    params2.periodUnits = Timer_PERIOD_US;
                    params2.period = 100 ; // Default 100usec to store
                    params2.timerMode = Timer_CONTINUOUS_CALLBACK;
                    params2.timerCallback = timerCallback;

                    timer0 = Timer_open(CONFIG_TIMER_3, &params2);

                    if (timer0 == NULL) {
                        /* Failed to initialized timer*/
                        while (1) {}
                    }

                    /*
                                        * Setting up the timer in continuous callback mode that calls the callback
                                        * function every 1,000,000 microseconds, or 1 second.*/

                                       Timer_Params_init(&params22);
                                       params22.periodUnits = Timer_PERIOD_US;
                                       params22.period = 20000 ; // Default 20msec to do mppt
                                       params22.timerMode = Timer_CONTINUOUS_CALLBACK;
                                       params22.timerCallback = timerCallback1;

                                       timer1 = Timer_open(CONFIG_TIMER_0, &params22);

                                       if (timer1 == NULL) {
                                           /* Failed to initialized timer*/
                                           while (1) {}
                                       }

                  dutyv = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * duty) / 100);

        PWM_Params_init(&paramss);
        paramss.dutyUnits = PWM_DUTY_FRACTION;
        paramss.dutyValue = dutyv;
        paramss.periodUnits = PWM_PERIOD_HZ;
        paramss.periodValue = swf*1000;
        pwm1 = PWM_open(CONFIG_PWM_0, &paramss);
        if (pwm1 == NULL) {
            /* CONFIG_PWM_0 did not open*/
            while (1);
        }

        PWM_start(pwm1);

        /* Turn on ENABLE */
          GPIO_write(CONFIG_GPIO_ena, 1);


                  data_update();  // initial read of current power in
                    old_load_v = load_v;
                    old_load_i = load_i;
                    old_p = old_load_v*old_load_i;

    SPI_init();



    /* Start the SimpleLink Host */
    pthread_attr_init(&pAttrs_spawn);
    priParam.sched_priority = SPAWN_TASK_PRIORITY;
    status = pthread_attr_setschedparam(&pAttrs_spawn, &priParam);
    status |= pthread_attr_setstacksize(&pAttrs_spawn, TASK_STACK_SIZE);

    status = pthread_create(&spawn_thread, &pAttrs_spawn, sl_Task, NULL);
    if(status)
    {
        printError("Task create failed", status);
    }

	/* Turn NWP on - initialize the device*/
    mode = sl_Start(0, 0, 0);
    if (mode < 0)
    {
        Display_printf(display, 0, 0,"\n\r[line:%d, error code:%d] %s\n\r", __LINE__, mode, DEVICE_ERROR);
    }

    if(mode != ROLE_AP)
    {
        /* Set NWP role as STA */
        mode = sl_WlanSetMode(ROLE_AP);
        if (mode < 0)
        {
            Display_printf(display, 0, 0,"\n\r[line:%d, error code:%d] %s\n\r", __LINE__, mode, WLAN_ERROR);
        }

        /* For changes to take affect, we restart the NWP */
        status = sl_Stop(SL_STOP_TIMEOUT);
        if (status < 0)
        {
            Display_printf(display, 0, 0,"\n\r[line:%d, error code:%d] %s\n\r", __LINE__, status, DEVICE_ERROR);
        }

        mode = sl_Start(0, 0, 0);
        if (mode < 0)
        {
            Display_printf(display, 0, 0,"\n\r[line:%d, error code:%d] %s\n\r", __LINE__, mode, DEVICE_ERROR);
        }
    }

    if(mode != ROLE_AP)
    {
        printError("Failed to configure device to it's default state", mode);
    }
    else
                Display_printf(display, 0, 0, "ROLE AP");

   // Connect();
    hostnet();
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
                               /* Failed to start timer*/
                               while (1) {}
        }
    if (Timer_start(timer1) == Timer_STATUS_ERROR) {
                                   /* Failed to start timer*/
                                   while (1) {}
            }

    while(1)
    {
        if(store_rst==1){
            n=0;
            store_rst=0;
            /* Stopping the SDCard */
            SDFatFS_close(sdfatfsHandle);
            Display_printf(display, 0, 0, "Drive %u unmounted\n", DRIVE_NUM);
        }


        if(store==1){
            if(n<1000){

                        loadi[n]= load_i;
                        loadv[n]= load_v;
                        loadp[n]=load_i*load_v;
                        dut[n]= PWM_Temp;
                        wles_p[n]= wless_p;
                        wles_v[n]= sqrtf(3.3*wless_p);
                        sw[n]=swf;
                        srci[n]=src_i;
                        srcp[n]=src_i*src_v;
                        timestamp[n]=fatfs_getFatTime();
                        if (buck_boost == 0)
                        eff[n] = (loadp[n]+wles_p[n])/srcp[n];
                        else
                        eff[n] = wles_p[n]/loadp[n];
                        srcv[n++]= src_v;

                        }
            else{
                /* Try to open the source file */
                tmp = fopen(temp_file, "r");
                if (!tmp) {
                Display_printf(display, 0, 0, "Creating a new file \"%s\"...",temp_file);
                /* Open file for both reading and writing */
                tmp = fopen(temp_file, "w+");
                if (!tmp) { Display_printf(display, 0, 0,
                "Error: \"%s\" could not be created.\nPlease check the "
                "Board.html if additional jumpers are necessary.\n",
                    temp_file);
                Display_printf(display, 0, 0, "Aborting...\n");
                while (1);
                 }

                        }
                /* Get the filesize of the source file */
                fseek(tmp, 0, SEEK_END);
                file1size = ftell(tmp);
                fclose(tmp);
                file1mb = file1size/1048218.0293;
                Display_printf(display, 0, 0, "File \"%s\" size is (%f MB) \n",temp_file,file1mb);
                /* Open file for both reading and writing */
                tmp = fopen(temp_file, "a+");
                if (!tmp) {Display_printf(display, 0, 0,
                           "Error: \"%s\" could not be created.\nPlease check the "
                           "Board.html if additional jumpers are necessary.\n",
                           temp_file);
                Display_printf(display, 0, 0, "Aborting...\n");
                while (1);
                }
                while(n!=0){
                fprintf(tmp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %ld \n",srcv[1000-n],srci[1000-n],srcp[1000-n],loadv[1000-n],loadi[1000-n],loadp[1000-n],dut[1000-n],sw[1000-n],wles_v[1000-n],wles_p[1000-n],eff[1000-n],timestamp[1000-n]);
                n--;
                }
                ct0++;
                fclose(tmp);
                if(file1mb >=100){
                memset(temp_file, 0, 50);
                sprintf(temp_file, "fat:"STR(DRIVE_NUM)":T%d_%d.txt", newp_count, limit_count++); // puts string into buffer
                //Display_printf(display, 0, 0, "Filename \"%s\" size of name is (%d Bytes) \n",temp_file,sizeof(temp_file));
                }

            }
            store=0;
        }


        if (mppf == 1){

            if(wload_v>0 && fstop==0)
                                            {
                                               data_update();
                                               //mpp_stop=0;
                                               pwm_f();
                                               //GPIO_toggle(CONFIG_GPIO_LED_2);
                                               ct3++;
                                               //usleep(50);
                                            }
                                           else
                                               swf = d_val; // default value 100 or changeable during runtime
            if (mpc == 1)
                 {
                     if (buck_boost == 0)
                      {

                          data_update();
                          Adj_PWM();
                          mpc=0;
                          //usleep(2000);
                          ct4++;
                      //   PWM_setPeriod(pwm1,swf*1000);
                        // usleep(100);  // 10ms Delay

                     }
                  else
                      {
                          data_update();
                          boost_pwm();
                          mpc=0;
                          ct5++;
                         // boost_d = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * 5) / 100);

                        //

                        //  usleep(100);  // 10ms Delay
                      }
                 }
                     //mppf=0;
          }
        else {
            data_update();
            dutyv = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * duty) / 100);
            PWM_setDutyAndPeriod(pwm1,dutyv,swf*1000);
            ct6++;

        }
                    ct++;
                    usleep(1);


                  }

}

/*
 * This callback is called every 1,000,000 microseconds, or 1 second. Because
 * the LED is toggled each time this function is called, the LED will blink at
 * a rate of once every 2 seconds.*/

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{

store=1;
ct7++;


}

void timerCallback1(Timer_Handle myHandle, int_fast16_t status)
{


mpc=1;
ct9++;


}



