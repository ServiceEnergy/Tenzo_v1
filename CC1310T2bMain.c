// �������� ����������: �������� ���������� �� ADC1 � �������� �� ����������� ���������
// � ���� ��������� ������ � ������  MASTER_ID_DEBUG � ������� RF ������
// ������������ ��� ������� ������������� ����� � ������ � SIM800
//05.06.24 �������� ����� ��� HX711 30 ���������, ������������ ������ Easylink � �������� ������ �� �����������
// 4.04.24  ��������� �������� �� GSM ������
// 17.04.24 ��������� �� Callback ��� ������ UART, ������� �� UART_MODE_BLOCKING
// 18.04.24 ������������� ������ ADC � GSM
// 19.04.24 �������� �� 3 �����, ����� ������ � 128 ���� payload (����������� - ������)
// 23.04.24 ������ ��� ������ � ��� ����� ��� ������ � ���������

#include <CC1310T2b.h>                          //�������� ��� ���� enum'��
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <ti/drivers/GPIO.h>                   //���������� �� ����� interrupts.c �������
#include <ti/sysbios/BIOS.h>

#include <ti/devices/cc13x0/inc/hw_memmap.h>   //��� ������� ������� � ���������
#include <ti/devices/cc13x0/inc/hw_types.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/devices/cc13x0/driverlib/sys_ctrl.h>
#include <ti/devices/cc13x0/driverlib/systick.h>
#include <ti/sysbios/knl/Semaphore.h>          //�������� ��� ����������� Task'��
#include <ti/sysbios/knl/Task.h>

//#include <ti/drivers/ADCBuf.h>                 //����� ADCBuf
//#include <ti/drivers/adcbuf/ADCBufCC26XX.h>    //��� ��������� adcBufParamsExtension
#include <ti/drivers/ADC.h>                    //����� ���������� ��������������
#include <ti/drivers/UART.h>                   //��� ����� � SIM800
//#include <ti/drivers/UART/UARTCC26XX.h>      //��� ����� � SIM800 //����� ������ ��� ������� RX ������

#include "easylink/EasyLink.h"                 //���������� ������� RF

//����� ������ ������ ������
#define MASTER_ID_SC                 0xF1 //��� ��
#define MASTER_ID_BASE               0xFF //��� BASE
#define MASTER_ID_FLASH              0xFA //��������
#define MASTER_ID_DEBUG_T            0xDD //Debug ��������� payload[] � UART ��� ��������� ������
#define MASTER_ID_DEBUG_A            0xDE //Debug ��������� payload[] � UART ��� ���������� ���������
#define MASTER_ID_DEBUG_S            0xDC //Debug ��������� payload[] � UART ��� ���������� ���������
#define GROUP_CODE                   0xA1 //������ ���� ����� ���������. �1,�2,�3,�4 � ������ �������� 4 ���������� ���������� ��������� ������ ������ �� ������ ������
#define UART_MAXLEN                  50 //������������ ����� ������ �� �������
#define ADCBUFFERSIZE               (40)

#define F_SAMPLE                    800000
bool flag = FALSE;
union uint64_as_uint8
{
        uint64_t uint;
        uint8_t  buf[8];
        uint16_t buf_int[4];
 };

void IrqD_OUT(uint_least8_t index);
void I_Time(GPTimerCC26XX_Handle handle_I,GPTimerCC26XX_IntMask interruptMask_I);




// ����������� ���������� ����������
//---------------------------------------------------------------------------------------
void HWI_init(void)
{
    //����� ������� ���������� �� ���� DATA ���������� HX711 luck26 29.05.24
    GPIO_setConfig(D_OUT,GPIO_CFG_IN_PU|GPIO_CFG_IN_INT_FALLING);
        GPIO_setCallback(D_OUT,IrqD_OUT);
        GPIO_enableInt(D_OUT);



}

//----------Callback GPTimer
//#pragma FUNCTION_OPTIONS(I_Time, "--opt_level=4 --opt_for_speed=5")

GPTimerCC26XX_Handle hTimer_I;
GPTimerCC26XX_Value I_TimeoutVal,current_tVal;


//Task Variables
Task_Struct TaskStructErrorHandler;  //���������� ���� ���������� ������
Char ErrorHandlerTaskStack[TASKSTACKSIZE];
Task_Struct TaskStructRFTransmitA;    //RF Transmitter A
Char RFTransmitTaskStackA[TASKSTACKSIZE];
Task_Struct hxTaskStruct;   //������ � ��� HX711
Char hxTaskStack[TASKSTACKSIZE];
Task_Struct blinkTaskStruct;    //������� �����������
Char blinkTaskStack[TASKSTACKSIZE];
//���������� ��� ������ getTemp ����� ������� ����������� TMP35
Task_Struct getTempTaskStruct;
Char getTempTaskStack[TASKSTACKSIZE];

//Semaphore Variable (Struct � Handl ����������, Params ��� �������������)
Semaphore_Struct ReadyForTransmitStructA;
Semaphore_Handle ReadyForTransmitA;
Semaphore_Struct GeneralErrorStruct;
Semaphore_Handle GeneralError;
Semaphore_Handle ReadyForHX;//������� ���������� ������ ��� HX711
Semaphore_Struct ReadyForHXStruct;
Semaphore_Handle ReadyForBlink;//������� �������� � ������ �������
Semaphore_Struct ReadyForBlinkStruct;
Semaphore_Handle ReadyForTemp;//������� �������� � ������ ������ ������������
Semaphore_Struct ReadyForTempStruct;


char ErrorMsg[50];                                 //��������� �� ���������� �������

//UART Variables
UART_Params uartParams;
UART_Handle uart;
uint8_t UARTBuff[UART_MAXLEN];                       //����� UART ������ �� 1 ����� ����� ���������
uint8_t GSMAnswer[UART_MAXLEN];                      //������ SIM800
uint16_t UARTCnt;                                    //������� ���� GSM ������

//���������� ���
ADC_Handle ADC_INA125,ADC_TMP_35,ADC_Vin;
ADC_Params params_adc;
uint16_t adcValue2,adcValue3,adcValue4;
uint32_t microVoltBuffer;

//Easy Link Variables
EasyLink_TxPacket txPacket =  { {0}, 0, 0, {0} };     // ����� ������ ��� �������� �� �����������
uint32_t DeviceID;                                    // ������������� � ��������� GSM_GetSIMNumber() ������ sim800_hub.c
volatile RF_Op* RFCommand;                            // ������ �� ������ ��������� �� �������� RF ��������
extern RF_Handle rfHandle;                            // ���������� � EasyLink.c ������ static � �������� rfHandle (!!!)
RF_InfoVal info;                                      // ���������� � RF.h
uint16_t RFStrCnt;                                    // ������� �������� ��������� ������ Buf_T[]
uint8_t Buf_T[EASYLINK_MAX_DATA_LENGTH];              // ����� ��� ��������� ����� ����� ������ RFStrCnt
uint8_t Buf_A[EASYLINK_MAX_DATA_LENGTH];              // ����� ��� ���������� ��������� ������ �������� 40 �����������

//GSM Variable
//union uint64_as_uint8 eccid; //�������� ���������� ���� CCID ������ � DeviceID ��������� ������ CCID ��� 2� ������ ���� ��� ����������� 89

//GLOBAL Variable
unsigned int ADCValue[40],ADCValue2[40],ADCValue3[3];
int Temp;
unsigned int adc_value1, adc_value2,adc_value4, VDDio;
uint16_t size_of_print;
char str[400];
char count_bits=0;


void RFModule_init(void);
void UARTModule_init(void);
void HWI_init(void);
void ErrorHandlerTask(UArg arg0, UArg arg1);
void RFTransmitTaskA(UArg arg0, UArg arg1);
void hxTask(void);
void blinkTask(void);
void getTempTask(void);
void IrqD_OUT(uint_least8_t index);

//void I_TimeoutCb(GPTimerCC26XX_Handle handle_I,GPTimerCC26XX_IntMask interruptMask_I);

//void GSM_SendCommand(char *command);
//uint8_t GSM_GetData(char* data);
//void GSMSessionStart(void);

//  ======================================== main ======================================
//-----------------------------------------------------------------------------------------
int main(void)
{
    Task_Params hxTaskParams;
    Task_Params blinkTaskParams;
    Task_Params getTempTaskParams;
    Task_Params TaskParamsErrorHandler;
    Task_Params TaskParamsRFTransmitA;

    Semaphore_Params ReadyForTransmitParamsA;
    Semaphore_Params ReadyForBlinkParams;
    Semaphore_Params ReadyForHXParams;
    Semaphore_Params ReadyForTempParams;

    CC1310_initGeneral();        //��������� ����� ������������ ������������� Board_init()
   GPIO_init();                 //��������� GPIO_PinConfig (?) � ������� GPIO_init ���������� �� ������

    GPIO_toggle(LED1);
    GPIO_toggle(LED2);

    // ��������� ���������� DCDC
    HWREG(AON_SYSCTL_BASE) = 0x00000005;  //������� PWRCTL - ������ � ����� AON_SYSCTL, ������� �������� 00

    /*
    // ���������� ������ � ������� �������� ���������� ���� ���������� ������
    Task_Params_init(&TaskParamsErrorHandler);
    TaskParamsErrorHandler.stackSize = TASKSTACKSIZE;
    TaskParamsErrorHandler.stack = &ErrorHandlerTaskStack;
    TaskParamsErrorHandler.priority = 3;
    Task_construct(&TaskStructErrorHandler, (Task_FuncPtr)ErrorHandlerTask, &TaskParamsErrorHandler, NULL);
*/
    /*
     //============������������� ���===========================================
     */
    ADC_Params_init(&params_adc);
    ADC_init();
    ADC_INA125  =     ADC_open(CC1310_LAUNCHXL_ADC0,&params_adc);
    ADC_TMP_35  =     ADC_open(CC1310_LAUNCHXL_ADC1,&params_adc);
    ADC_Vin     =     ADC_open(CC1310_LAUNCHXL_ADCVDDS,&params_adc);





    //������������� �������
    GPTimerCC26XX_Params params_I;
    params_I.width = GPT_CONFIG_32BIT;
    params_I.mode = GPT_MODE_PERIODIC;
    params_I.debugStallMode = GPTimerCC26XX_DEBUG_STALL_OFF;
    hTimer_I = GPTimerCC26XX_open(CC1310_LAUNCHXL_GPTIMER0A,&params_I);
   // if(hTimer_I == NULL)while(1);
    I_TimeoutVal = SysCtrlClockGet()/F_SAMPLE-1UL;
    GPTimerCC26XX_setLoadValue(hTimer_I,I_TimeoutVal);
    GPTimerCC26XX_registerInterrupt(hTimer_I,I_Time,GPT_INT_TIMEOUT);



    //��������� ������ ����� ��� HX711
    Task_Params_init(&hxTaskParams);
    hxTaskParams.stackSize = TASKSTACKSIZE;
    hxTaskParams.priority = 3;
    hxTaskParams.stack = &hxTaskStack;
    Task_construct(&hxTaskStruct, (Task_FuncPtr)hxTask,&hxTaskParams,NULL);

    // ���������� ������ ������� ���������� ������ �� �����
    Task_Params_init(&TaskParamsRFTransmitA);
    TaskParamsRFTransmitA.stackSize = TASKSTACKSIZE;
    TaskParamsRFTransmitA.stack = &RFTransmitTaskStackA;
    TaskParamsRFTransmitA.priority = 3;
    Task_construct(&TaskStructRFTransmitA, (Task_FuncPtr)RFTransmitTaskA, &TaskParamsRFTransmitA, NULL);

    //��������� ������ ������� �����������
    Task_Params_init(&blinkTaskParams);
    blinkTaskParams.stackSize = TASKSTACKSIZE;
    blinkTaskParams.priority = 3;
    blinkTaskParams.stack = &blinkTaskStack;
    Task_construct(&blinkTaskStruct, (Task_FuncPtr)blinkTask,&blinkTaskParams,NULL);

    //��������� ������ ����������� �����������
    Task_Params_init(&getTempTaskParams);
    getTempTaskParams.stackSize = TASKSTACKSIZE;
    getTempTaskParams.priority = 3;
    getTempTaskParams.stack = &getTempTaskStack;
    Task_construct(&getTempTaskStruct, (Task_FuncPtr)getTempTask,&getTempTaskParams,NULL);

    Semaphore_Params_init(&ReadyForHXParams);   //������� ���������� ������ ���
    ReadyForHXParams.mode=Semaphore_Mode_BINARY;
    Semaphore_construct(&ReadyForHXStruct, 0, &ReadyForHXParams);
    ReadyForHX = Semaphore_handle(&ReadyForHXStruct);

    Semaphore_Params_init(&ReadyForBlinkParams);   //������� ���������� � ������� �����������
    ReadyForBlinkParams.mode=Semaphore_Mode_BINARY;
    Semaphore_construct(&ReadyForBlinkStruct, 0, &ReadyForBlinkParams);
    ReadyForBlink = Semaphore_handle(&ReadyForBlinkStruct);

    Semaphore_Params_init(&ReadyForTempParams);   //������� ���������� � ������ ������������
    ReadyForTempParams.mode=Semaphore_Mode_BINARY;
    Semaphore_construct(&ReadyForTempStruct, 0, &ReadyForTempParams);
    ReadyForTemp = Semaphore_handle(&ReadyForTempStruct);

    Semaphore_Params_init(&ReadyForTransmitParamsA);   //������� ���������� �������� �� RF ������
    ReadyForTransmitParamsA.mode=Semaphore_Mode_BINARY;
    Semaphore_construct(&ReadyForTransmitStructA, 0, &ReadyForTransmitParamsA);
    ReadyForTransmitA = Semaphore_handle(&ReadyForTransmitStructA);

 /*
    Semaphore_Params_init(&GeneralErrorParams); //������� ����������� ����
    GeneralErrorParams.mode=Semaphore_Mode_BINARY;
    Semaphore_construct(&GeneralErrorStruct, 0, &GeneralErrorParams);
    GeneralError = Semaphore_handle(&GeneralErrorStruct);
*/










    BIOS_start();

    return (0);
}


// ������ blink
//--------------------------------------------------------------------------------------
void blinkTask(void)
{
    UARTModule_init();   //��������� UART
    HWI_init();          //��������� ���������� ����������
    RFModule_init();     //������������� ������ ����� ������ BIOS, ������, ��� ���������� ����������

uint8_t i=0,k=0;
    while(1){
        Semaphore_pend(ReadyForBlink, BIOS_WAIT_FOREVER);
      //������� ����
      for(i=0;i<10;i++)
      {
          GPIO_write(LED1,1);
          Task_sleep(1000);
          GPIO_write(LED1,0);
          Task_sleep(1000);

      }
     /*
      //����� � ����� ������� � UART ��� �������� ������� DCValue2
      for(k=0;k<30;k++)
      {
          size_of_print = sprintf (str, "ADCValue[%d] = %d\r\n",k,ADCValue2[k]);
          UART_write(uart,&str,size_of_print);
      }

     // UART_write(uart,&txPacket,txPacket.len);
      size_of_print = sprintf (str, "\r\n");
                UART_write(uart,&str,size_of_print);
    */
      Semaphore_post(ReadyForTemp);
      //Task_sleep(10000);
      //GPIO_enableInt(D_OUT);//������� ���������� �� ���� D_OUT
      //Task_sleep(10000);
    }
}

//������ � ������� ������������ ���
void getTempTask(void)
{

    //��������� ����������
    int_fast16_t ADCStatus;
    unsigned int f,c,v1,v2;

    while(1){

        //����������� ����
        //��� ��������
        Semaphore_pend(ReadyForTemp, BIOS_WAIT_FOREVER);
        //��������� �������������� ��� � ������ ������� ����������� TMP35
        ADCStatus = ADC_convert(ADC_TMP_35, &adcValue2);
        //��������� ����������� � ������� �������
        microVoltBuffer = (int) ADC_convertToMicroVolts(ADC_TMP_35, adcValue2);
        f = microVoltBuffer/1000;
        c = ((f-320)*50)/90;
        Temp = c;
        //v1 = microVoltBuffer/1000;
        //������� � UART
        //size_of_print = sprintf (str, "\r\nTemp = %d 'C\r\n\r\n",c);
        //UART_write(uart,&str,size_of_print);
        Task_sleep(1000);
        //����� �������� �������������� � ������� ���������� �������
        ADCStatus = ADC_convert(ADC_Vin, &adcValue3);
        microVoltBuffer = (int) ADC_convertToMicroVolts(ADC_Vin, adcValue3);
        v2 = microVoltBuffer/1000;
        VDDio = v2;
        //size_of_print = sprintf (str, "\r\nVDDext = %d mV\r\n\r\n",v2);
        //UART_write(uart,&str,size_of_print);
        Task_sleep(1000);
        ADCStatus = ADC_convert(ADC_INA125, &adcValue3);
        microVoltBuffer = (int) ADC_convertToMicroVolts(ADC_INA125, adcValue3);
        v1=microVoltBuffer/1000;
        ADCValue[2] = v1;
        size_of_print = sprintf (str, "\r\n%d %d %d\r\n",c,v2,v1);
        UART_write(uart,&str,size_of_print);
        Task_sleep(1000);
        //GPIO_enableInt(D_OUT);//������� ���������� �� ���� D_OUT
        Semaphore_post(ReadyForHX);//��������� ������� ���������� ������ ��� HX711
        Task_sleep(1000);

    }

}


void I_Time(GPTimerCC26XX_Handle handle_I,GPTimerCC26XX_IntMask interruptMask_I)
{
    int_fast16_t ADCStatus;
    //���������� ���������� ������� 5000 ��
    GPIO_toggle(LED1);

    //GPIO_disableInt(D_OUT);//��������� ������� ����������

    GPTimerCC26XX_setLoadValue(hTimer_I, I_TimeoutVal);//��������� �������� � ������� ������� �������
    if(count_bits<24)
    {//�������� ��� �� ���������
        if(GPIO_read(PD_SCK))//���� ���� ������ � "1"
        {
         GPIO_write(PD_SCK,0);//������� ���� � "0" � ������ ��������� ���� ������
         if(GPIO_read(D_OUT)>0) {adc_value1++;}
         count_bits++;
        }
        else
        {
            GPIO_write(PD_SCK,1);
            adc_value1<<=1;
        }


    }
    else
    {
        if(GPIO_read(PD_SCK))
        {
            GPIO_write(PD_SCK,0);
            //������������� ������
            GPTimerCC26XX_stop(hTimer_I);
            //��������� ������� ����������
           //GPIO_enableInt(D_OUT); //���������� �������� ����� � ������ HX
            count_bits=0;
            //Semaphore_post(ReadyForHX);//��������� ������� ���������� ������ ��� HX711
            //������� ��� INA125 � �������� ��������� � ���������� adc_value4
            ADCStatus = ADC_convert(ADC_INA125, &adcValue3);
            microVoltBuffer = (int) ADC_convertToMicroVolts(ADC_INA125, adcValue3);
            adc_value4=microVoltBuffer/1000;
           // ADCValue[2] = v1;
            //�������� ������� ���������� ������ ��� �����������
            Semaphore_post(ReadyForTemp);
        }
        else{
            GPIO_write(PD_SCK,1);
        }
    }
}



//������ HX711
// -------------------------------------------------------------------------------------
void hxTask(void)
{
    uint8_t count=0;//����� ���������� ���������
    unsigned char *ptr;
    uint_fast16_t i,j;

    while(1){
        GPIO_write(PD_SCK,0);
      //��� ��������
        Semaphore_pend(ReadyForHX, BIOS_WAIT_FOREVER);

        //������ �� ������� �� �������� �������� count
        if(count>10)
        {
            GPIO_write(PD_SCK,0);
            count=0;
            txPacket.payload[0] = MASTER_ID_DEBUG_S;
            txPacket.payload[1] = DeviceID >> 16;
            txPacket.payload[2] = (DeviceID >> 8) & 0xFF;
            txPacket.payload[3] = DeviceID & 0xFF;
            txPacket.dstAddr[0] = GROUP_CODE;
            txPacket.absTime = 0;
            j=4;
            for(i=0; i<10; i++)
                {
                    ptr = (unsigned char*) &ADCValue2[i];  //���� ����� �������� �������� ������� ADCValue2
                    txPacket.payload[j++]=*(ptr);                               //��������� ������ ����
                    txPacket.payload[j++]=*(ptr+1);                             //��������� ������ ����
                    txPacket.payload[j++]=*(ptr+2);                             //��������� ������ ����
                    txPacket.payload[j++]=*(ptr+3);                             //��������� �������� ����
                }//for ��� �������� � ��� HX711
           // j ++;
            for(i=0; i<10; i++)
                {
                    ptr = (unsigned char*) &ADCValue[i];  //���� ����� �������� �������� ������� ADCValue
                    txPacket.payload[j++]=*(ptr);                               //��������� ������ ����
                    txPacket.payload[j++]=*(ptr+1);                             //��������� ������ ����
                    txPacket.payload[j++]=*(ptr+2);                             //��������� ������ ����
                    txPacket.payload[j++]=*(ptr+3);                             //��������� �������� ����
                 }//for ��� �������� � ��� INA125

            ptr = (unsigned char*) &Temp;  //���� ����� ���������� Temp (� ��� �������� �������� �����������)
            txPacket.payload[j++]=*(ptr);                               //��������� ������ ����
            txPacket.payload[j++]=*(ptr+1);                             //��������� ������ ����
            txPacket.payload[j++]=*(ptr+2);                             //��������� ������ ����
            txPacket.payload[j++]=*(ptr+3);                             //��������� �������� ����

            ptr = (unsigned char*) &VDDio;  //���� ����� ���������� VDDio (� ��� �������� �������� ���������� ������� � �����������)
            txPacket.payload[j++]=*(ptr);                               //��������� ������ ����
            txPacket.payload[j++]=*(ptr+1);                             //��������� ������ ����
            txPacket.payload[j++]=*(ptr+2);                             //��������� ������ ����
            txPacket.payload[j++]=*(ptr+3);                             //��������� �������� ����

            txPacket.len = 128;
            Semaphore_post(ReadyForTransmitA);//���������� ������� ���������� �������� �� �����������
            //GPIO_enableInt(D_OUT);//������� ���������� �� ���� D_OUT
            //flag = FALSE;
            Task_sleep(1000);//�������

        }
        else
        {
            //GPIO_write(PD_SCK,0);
            //��������� ����� �������. ��������� ��������
             ADCValue2[count]=adc_value1;//�������� � HX711
             ADCValue[count]=adc_value4;//�������� � INA125
             count++;//��������� �������� ���������
             //adc_value1=0;
             //�������� ������� ����������
             //������� ����������
             /*
             adc_value2 = ((adc_value1 - 32123)*98)/106080-1218;
             size_of_print = sprintf (str, "\r\n%d %d %d %d\r\n",adc_value2,ADCValue3[0],ADCValue3[1],ADCValue3[2]);
             UART_write(uart,&str,size_of_print);
             */
             adc_value1=0;
             adc_value2=0;
             adc_value4=0;
             GPIO_enableInt(D_OUT);//������� ���������� �� ���� D_OUT
             Task_sleep(1000);
        }

    }
}





/*
// ������ BIOS, �������� ����������, ������� �� �������� GeneralError
// ��� ������ � ���������� �������� ���������� RTOS ����� Semaphore_pend()
//---------------------------------------------------------------------------------------
void ErrorHandlerTask(UArg arg0, UArg arg1)
{
int i,l;
  while(1){ //���� �������� ������ ��� ����������� �������
     Semaphore_pend(GeneralError, BIOS_WAIT_FOREVER); //���� ����������� ��������
     txPacket.payload[0] = MASTER_ID_DEBUG_T;
     txPacket.payload[1] = DeviceID >> 16;
     txPacket.payload[2] = (DeviceID >> 8) & 0xFF;
     txPacket.payload[3] = DeviceID & 0xFF;
     txPacket.dstAddr[0] = GROUP_CODE;
     txPacket.absTime = 0;
     l = strlen(&ErrorMsg[0]);
     for(i=0; i<l; i++) txPacket.payload[i] = ErrorMsg[i];
     txPacket.len = l + 4;
     EasyLink_transmit(&txPacket); //������� ���������� EasyLink
  }
}
*/



// ������ BIOS, �������� ����������, �� �������� ReadyForTransmit �������� ����� 868 MHz
// ��������� ������ �������� � ������ Buf_A[], ����� ������ RFStrCnt
//---------------------------------------------------------------------------------------
void RFTransmitTaskA(UArg arg0, UArg arg1)
{
int i;
   while(1){
       Semaphore_pend(ReadyForTransmitA, BIOS_WAIT_FOREVER); //���� ����������� �������� (�������� ���������� RTOS)
       txPacket.payload[0] = MASTER_ID_DEBUG_S;
       txPacket.payload[1] = DeviceID >> 16;
       txPacket.payload[2] = (DeviceID >> 8) & 0xFF;
       txPacket.payload[3] = DeviceID & 0xFF;
       txPacket.len        = 128; //
       txPacket.dstAddr[0] = GROUP_CODE;
       txPacket.absTime = 0;

       EasyLink_transmit(&txPacket); //������� ���������� EasyLink

       Semaphore_post(ReadyForBlink);
       Task_sleep(1000);



   }
}


// Init RF Module
// C:/ti/simplelink_cc13x0_sdk_4_20_02_07/docs/proprietary-rf/proprietary-rf-users-guide/easylink/easylink-api-reference-cc13x0.html
//---------------------------------------------------------------------------------------
void RFModule_init(void)
{
    //uint8_t addrFilter[EASYLINK_MAX_ADDR_SIZE * EASYLINK_MAX_ADDR_FILTERS] = {0xAA,0xA1};
    //uint8_t addrFilter[EASYLINK_MAX_ADDR_SIZE * EASYLINK_MAX_ADDR_FILTERS] = {0xAA, 0xE9,0xA1,0xA2,0xA3,0xA4,0xA5};
    EasyLink_init(EasyLink_Phy_Custom);
    EasyLink_setRfPwr(14);
    //EasyLink_enableRxAddrFilter(addrFilter, 1, 2); //��� ������ ������� 2, ��� ������ 7
}


// Init UART Module
//---------------------------------------------------------------------------------------
void UARTModule_init(void)
{
    UART_init();
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.stopBits = UART_STOP_ONE;
    uartParams.baudRate = 115200;
    uartParams.readMode = UART_MODE_BLOCKING;
    //uartParams.writeCallback = writeCallback; //����������
    uart = UART_open(0, &uartParams);
    if (uart == NULL) { while (1); }
}


/*
// Init UART
// Unblocking Callback mode
//---------------------------------------------------------------------------------------
void UARTModule_init(void)
{
    UART_init();
    UART_Params_init(&uartParams); //������� ������� ����� ���������:(��������� gsm UART ������ � MDD_BASE_hub.c)
    uartParams.writeDataMode = UART_DATA_BINARY;    //UART_DATA_TEXT  UART_DATA_BINARY
    uartParams.readDataMode =  UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL ;  // UART_RETURN_FULL UART_RETURN_NEWLINE
    uartParams.readEcho = UART_ECHO_OFF;            // UART_ECHO_ON
    uartParams.dataLength = UART_LEN_8;
    uartParams.stopBits = UART_STOP_ONE;
    uartParams.baudRate = 115200;
    uartParams.readMode = UART_MODE_BLOCKING;   // UART_MODE_CALLBACK UART_MODE_BLOCKING
    //uartGSMParams.readCallback = UART_RxFxn;     //���������� ����������
    //uartParams.writeCallback = writeCallback;    //���������� ����������
    uartParams.readTimeout = 1000000;//500000;   //No timeout in UART_MODE_CALLBACK
    uartParams.writeTimeout= 1000000;            //UART_WAIT_FOREVER
}

*/



/*
 *  ============== ���������� ����������
 */
void IrqD_OUT(uint_least8_t index)
{

    GPIO_disableInt(D_OUT);//��������� ������� ����������
    count_bits = 0;//�������� ������� ���
    adc_value1=0;//�������� ����������
    GPIO_write(PD_SCK,0);
    GPTimerCC26XX_start(hTimer_I);
    GPIO_toggle(LED2);
    //flag = TRUE;

   /*
    count_bits = 0;//�������� ������� ���
    GPIO_write(PD_SCK,0);
    GPTimerCC26XX_start(hTimer_I);//��������� ������
    GPIO_enableInt(D_OUT);//��������� ������� ����������
    */
    /*
 uint8_t i=0;
    //��������� ���������� D_OUT
    GPIO_disableInt(D_OUT);
    //��������� ����
    for( i=0;i<24;i++)
    {
        //������� �����
        GPIO_write(PD_SCK,1);
        //�������� �������� ADC �� 1 �����
        adc_value1<<=1;
        Task_sleep(1);
        //�������� �����
        GPIO_write(PD_SCK,0);
        //��������� ������� �� ���� D_OUT
        //���� ��� "1" �� �������������� �������� ADC
        if(GPIO_read(D_OUT)>0)
        {
            adc_value1++;
        }
        Task_sleep(1);
    }

    //������� �����
     GPIO_write(PD_SCK,1);
     //���������
     Task_sleep(1);
     //�������� �����
     GPIO_write(PD_SCK,0);
     Task_sleep(1);
     //���������� �������
     Semaphore_post(ReadyForHX);
     GPIO_enableInt(D_OUT);//������� ���������� �� ���� D_OUT
     Task_sleep(1000);
*/

}

