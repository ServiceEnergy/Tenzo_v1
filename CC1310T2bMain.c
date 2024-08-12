// Тестовый передатчик: измеряет напряжение на ADC1 и передает по радиоканалу измерение
// в виде текстовой строки с меткой  MASTER_ID_DEBUG в составе RF пакета
// используется для отладки измерительных цепей и обмена с SIM800
//05.06.24 Добавлен опрос АЦП HX711 30 измерений, формирование пакета Easylink и отправка пакета по радиоканалу
// 4.04.24  Добавлена передача по GSM каналу
// 17.04.24 Отказался от Callback при чтении UART, перешел на UART_MODE_BLOCKING
// 18.04.24 Одновременная работа ADC и GSM
// 19.04.24 Передача по 3 байта, чтобы влезть в 128 байт payload (увеличивать - нельзя)
// 23.04.24 Сделал два буфера и два таска для модема и измерений

#include <CC1310T2b.h>                          //Добавить для всех enum'ов
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <ti/drivers/GPIO.h>                   //Перенесено из файла interrupts.c примера
#include <ti/sysbios/BIOS.h>

#include <ti/devices/cc13x0/inc/hw_memmap.h>   //Для прямого доступа к регистрам
#include <ti/devices/cc13x0/inc/hw_types.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/devices/cc13x0/driverlib/sys_ctrl.h>
#include <ti/devices/cc13x0/driverlib/systick.h>
#include <ti/sysbios/knl/Semaphore.h>          //Добавить для определения Task'ов
#include <ti/sysbios/knl/Task.h>

//#include <ti/drivers/ADCBuf.h>                 //Режим ADCBuf
//#include <ti/drivers/adcbuf/ADCBufCC26XX.h>    //Для структуры adcBufParamsExtension
#include <ti/drivers/ADC.h>                    //Режим одиночного преобразования
#include <ti/drivers/UART.h>                   //Для связи с SIM800
//#include <ti/drivers/UART/UARTCC26XX.h>      //Для связи с SIM800 //Нужна только для очистки RX буфера

#include "easylink/EasyLink.h"                 //Библиотека функций RF

//Режим приема пакета данных
#define MASTER_ID_SC                 0xF1 //МДД КЗ
#define MASTER_ID_BASE               0xFF //МДД BASE
#define MASTER_ID_FLASH              0xFA //Прошивка
#define MASTER_ID_DEBUG_T            0xDD //Debug пересылка payload[] в UART как текстовой строки
#define MASTER_ID_DEBUG_A            0xDE //Debug пересылка payload[] в UART как аналоговые измерения
#define MASTER_ID_DEBUG_S            0xDC //Debug пересылка payload[] в UART как аналоговые измерения
#define GROUP_CODE                   0xA1 //вводим коды групп устройств. А1,А2,А3,А4 в группе максимум 4 устройства устройства принимают пакеты только от членов группы
#define UART_MAXLEN                  50 //максимальная длина ответа на команду
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




// Настраиваем аппаратные прерывания
//---------------------------------------------------------------------------------------
void HWI_init(void)
{
    //Здесь добавим прерывание от ноги DATA микросхемы HX711 luck26 29.05.24
    GPIO_setConfig(D_OUT,GPIO_CFG_IN_PU|GPIO_CFG_IN_INT_FALLING);
        GPIO_setCallback(D_OUT,IrqD_OUT);
        GPIO_enableInt(D_OUT);



}

//----------Callback GPTimer
//#pragma FUNCTION_OPTIONS(I_Time, "--opt_level=4 --opt_for_speed=5")

GPTimerCC26XX_Handle hTimer_I;
GPTimerCC26XX_Value I_TimeoutVal,current_tVal;


//Task Variables
Task_Struct TaskStructErrorHandler;  //Обработчик всех аппаратных ошибок
Char ErrorHandlerTaskStack[TASKSTACKSIZE];
Task_Struct TaskStructRFTransmitA;    //RF Transmitter A
Char RFTransmitTaskStackA[TASKSTACKSIZE];
Task_Struct hxTaskStruct;   //Работа с АЦП HX711
Char hxTaskStack[TASKSTACKSIZE];
Task_Struct blinkTaskStruct;    //Мигалка светодиодом
Char blinkTaskStack[TASKSTACKSIZE];
//Переменные для задачи getTemp опрос датчика температуры TMP35
Task_Struct getTempTaskStruct;
Char getTempTaskStack[TASKSTACKSIZE];

//Semaphore Variable (Struct и Handl глобальные, Params при инициализации)
Semaphore_Struct ReadyForTransmitStructA;
Semaphore_Handle ReadyForTransmitA;
Semaphore_Struct GeneralErrorStruct;
Semaphore_Handle GeneralError;
Semaphore_Handle ReadyForHX;//Семафор готовности данных АЦП HX711
Semaphore_Struct ReadyForHXStruct;
Semaphore_Handle ReadyForBlink;//Семафор перехода в задачу мигания
Semaphore_Struct ReadyForBlinkStruct;
Semaphore_Handle ReadyForTemp;//Семафор перехода в задачу опроса термодатчика
Semaphore_Struct ReadyForTempStruct;


char ErrorMsg[50];                                 //Сообщения об аппаратных ошибках

//UART Variables
UART_Params uartParams;
UART_Handle uart;
uint8_t UARTBuff[UART_MAXLEN];                       //Буфер UART читаем по 1 байту можно уменьшить
uint8_t GSMAnswer[UART_MAXLEN];                      //Ответы SIM800
uint16_t UARTCnt;                                    //Счетчик байт GSM ответа

//Переменные АЦП
ADC_Handle ADC_INA125,ADC_TMP_35,ADC_Vin;
ADC_Params params_adc;
uint16_t adcValue2,adcValue3,adcValue4;
uint32_t microVoltBuffer;

//Easy Link Variables
EasyLink_TxPacket txPacket =  { {0}, 0, 0, {0} };     // Пакет данных для передачи по радиоканалу
uint32_t DeviceID;                                    // Присваивается в процедуре GSM_GetSIMNumber() модуля sim800_hub.c
volatile RF_Op* RFCommand;                            // Ссылка на статус структуру со статусом RF операции
extern RF_Handle rfHandle;                            // Определена в EasyLink.c УБРАТЬ static в описании rfHandle (!!!)
RF_InfoVal info;                                      // Определено в RF.h
uint16_t RFStrCnt;                                    // Счетчик символов текстовой строки Buf_T[]
uint8_t Buf_T[EASYLINK_MAX_DATA_LENGTH];              // Буфер для текстовых строк длина строки RFStrCnt
uint8_t Buf_A[EASYLINK_MAX_DATA_LENGTH];              // Буфер для аналоговых измерений всегда заполнен 40 измерениями

//GSM Variable
//union uint64_as_uint8 eccid; //значение оставшихся цифр CCID вместе с DeviceID формирует полный CCID без 2х первых цифр они стандартные 89

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

    CC1310_initGeneral();        //Системный вызов генерируется автоматически Board_init()
   GPIO_init();                 //Обработка GPIO_PinConfig (?) В примере GPIO_init вызывается из потока

    GPIO_toggle(LED1);
    GPIO_toggle(LED2);

    // ВЫключить встроенный DCDC
    HWREG(AON_SYSCTL_BASE) = 0x00000005;  //Регистр PWRCTL - первый в блоке AON_SYSCTL, поэтому смещение 00

    /*
    // Определяем Задачу в которой крутится обработчик всех аппаратных ошибок
    Task_Params_init(&TaskParamsErrorHandler);
    TaskParamsErrorHandler.stackSize = TASKSTACKSIZE;
    TaskParamsErrorHandler.stack = &ErrorHandlerTaskStack;
    TaskParamsErrorHandler.priority = 3;
    Task_construct(&TaskStructErrorHandler, (Task_FuncPtr)ErrorHandlerTask, &TaskParamsErrorHandler, NULL);
*/
    /*
     //============Инициализация АЦП===========================================
     */
    ADC_Params_init(&params_adc);
    ADC_init();
    ADC_INA125  =     ADC_open(CC1310_LAUNCHXL_ADC0,&params_adc);
    ADC_TMP_35  =     ADC_open(CC1310_LAUNCHXL_ADC1,&params_adc);
    ADC_Vin     =     ADC_open(CC1310_LAUNCHXL_ADCVDDS,&params_adc);





    //Инициализация таймера
    GPTimerCC26XX_Params params_I;
    params_I.width = GPT_CONFIG_32BIT;
    params_I.mode = GPT_MODE_PERIODIC;
    params_I.debugStallMode = GPTimerCC26XX_DEBUG_STALL_OFF;
    hTimer_I = GPTimerCC26XX_open(CC1310_LAUNCHXL_GPTIMER0A,&params_I);
   // if(hTimer_I == NULL)while(1);
    I_TimeoutVal = SysCtrlClockGet()/F_SAMPLE-1UL;
    GPTimerCC26XX_setLoadValue(hTimer_I,I_TimeoutVal);
    GPTimerCC26XX_registerInterrupt(hTimer_I,I_Time,GPT_INT_TIMEOUT);



    //Определим задачу опрса АЦП HX711
    Task_Params_init(&hxTaskParams);
    hxTaskParams.stackSize = TASKSTACKSIZE;
    hxTaskParams.priority = 3;
    hxTaskParams.stack = &hxTaskStack;
    Task_construct(&hxTaskStruct, (Task_FuncPtr)hxTask,&hxTaskParams,NULL);

    // Определяем Задачу которая пересылает данные по радио
    Task_Params_init(&TaskParamsRFTransmitA);
    TaskParamsRFTransmitA.stackSize = TASKSTACKSIZE;
    TaskParamsRFTransmitA.stack = &RFTransmitTaskStackA;
    TaskParamsRFTransmitA.priority = 3;
    Task_construct(&TaskStructRFTransmitA, (Task_FuncPtr)RFTransmitTaskA, &TaskParamsRFTransmitA, NULL);

    //Определим задачу мигания светодиодом
    Task_Params_init(&blinkTaskParams);
    blinkTaskParams.stackSize = TASKSTACKSIZE;
    blinkTaskParams.priority = 3;
    blinkTaskParams.stack = &blinkTaskStack;
    Task_construct(&blinkTaskStruct, (Task_FuncPtr)blinkTask,&blinkTaskParams,NULL);

    //Определим задачу определения температуры
    Task_Params_init(&getTempTaskParams);
    getTempTaskParams.stackSize = TASKSTACKSIZE;
    getTempTaskParams.priority = 3;
    getTempTaskParams.stack = &getTempTaskStack;
    Task_construct(&getTempTaskStruct, (Task_FuncPtr)getTempTask,&getTempTaskParams,NULL);

    Semaphore_Params_init(&ReadyForHXParams);   //Семафор готовности данных АЦП
    ReadyForHXParams.mode=Semaphore_Mode_BINARY;
    Semaphore_construct(&ReadyForHXStruct, 0, &ReadyForHXParams);
    ReadyForHX = Semaphore_handle(&ReadyForHXStruct);

    Semaphore_Params_init(&ReadyForBlinkParams);   //Семафор готовности к миганию светодиодом
    ReadyForBlinkParams.mode=Semaphore_Mode_BINARY;
    Semaphore_construct(&ReadyForBlinkStruct, 0, &ReadyForBlinkParams);
    ReadyForBlink = Semaphore_handle(&ReadyForBlinkStruct);

    Semaphore_Params_init(&ReadyForTempParams);   //Семафор готовности к опроса термодатчика
    ReadyForTempParams.mode=Semaphore_Mode_BINARY;
    Semaphore_construct(&ReadyForTempStruct, 0, &ReadyForTempParams);
    ReadyForTemp = Semaphore_handle(&ReadyForTempStruct);

    Semaphore_Params_init(&ReadyForTransmitParamsA);   //Семафор готовности передачи по RF каналу
    ReadyForTransmitParamsA.mode=Semaphore_Mode_BINARY;
    Semaphore_construct(&ReadyForTransmitStructA, 0, &ReadyForTransmitParamsA);
    ReadyForTransmitA = Semaphore_handle(&ReadyForTransmitStructA);

 /*
    Semaphore_Params_init(&GeneralErrorParams); //Семафор Аппаратного сбоя
    GeneralErrorParams.mode=Semaphore_Mode_BINARY;
    Semaphore_construct(&GeneralErrorStruct, 0, &GeneralErrorParams);
    GeneralError = Semaphore_handle(&GeneralErrorStruct);
*/










    BIOS_start();

    return (0);
}


// Задача blink
//--------------------------------------------------------------------------------------
void blinkTask(void)
{
    UARTModule_init();   //Настройка UART
    HWI_init();          //Настройка аппаратных прерываний
    RFModule_init();     //Инициализация Только после старта BIOS, потому, что пользуется семафорами

uint8_t i=0,k=0;
    while(1){
        Semaphore_pend(ReadyForBlink, BIOS_WAIT_FOREVER);
      //Напишем цикл
      for(i=0;i<10;i++)
      {
          GPIO_write(LED1,1);
          Task_sleep(1000);
          GPIO_write(LED1,0);
          Task_sleep(1000);

      }
     /*
      //Здесь в цикле выведем в UART все элементы массива DCValue2
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
      //GPIO_enableInt(D_OUT);//Включим прерывание по ноге D_OUT
      //Task_sleep(10000);
    }
}

//Задача в которой опрашиваются АЦП
void getTempTask(void)
{

    //Объявляем переменные
    int_fast16_t ADCStatus;
    unsigned int f,c,v1,v2;

    while(1){

        //Бесконечный цикл
        //Ждём семафора
        Semaphore_pend(ReadyForTemp, BIOS_WAIT_FOREVER);
        //Запускаем преобразование АЦП с выхода датчика температуры TMP35
        ADCStatus = ADC_convert(ADC_TMP_35, &adcValue2);
        //Переводим мокровольты в градусы цельсия
        microVoltBuffer = (int) ADC_convertToMicroVolts(ADC_TMP_35, adcValue2);
        f = microVoltBuffer/1000;
        c = ((f-320)*50)/90;
        Temp = c;
        //v1 = microVoltBuffer/1000;
        //Выводим в UART
        //size_of_print = sprintf (str, "\r\nTemp = %d 'C\r\n\r\n",c);
        //UART_write(uart,&str,size_of_print);
        Task_sleep(1000);
        //Тепеь запустим преобразование и измерим напряжение питания
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
        //GPIO_enableInt(D_OUT);//Включим прерывание по ноге D_OUT
        Semaphore_post(ReadyForHX);//Выставили семафор готовности данных АЦП HX711
        Task_sleep(1000);

    }

}


void I_Time(GPTimerCC26XX_Handle handle_I,GPTimerCC26XX_IntMask interruptMask_I)
{
    int_fast16_t ADCStatus;
    //Обработчик прерывания таймера 5000 Гц
    GPIO_toggle(LED1);

    //GPIO_disableInt(D_OUT);//Отключили внешнее прерывание

    GPTimerCC26XX_setLoadValue(hTimer_I, I_TimeoutVal);//Загрузили значение в счётный регистр таймера
    if(count_bits<24)
    {//Значение АЦП не прочитано
        if(GPIO_read(PD_SCK))//Если нога тактов в "1"
        {
         GPIO_write(PD_SCK,0);//Опустим ногу в "0" и прочтём состояние ноги данных
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
            //Останавливаем таймер
            GPTimerCC26XX_stop(hTimer_I);
            //Запускаем внешнее прерывание
           //GPIO_enableInt(D_OUT); //Прерывание включать будем в задаче HX
            count_bits=0;
            //Semaphore_post(ReadyForHX);//Выставили семафор готовности данных АЦП HX711
            //Опросим АЦП INA125 и сохраним результат в переменной adc_value4
            ADCStatus = ADC_convert(ADC_INA125, &adcValue3);
            microVoltBuffer = (int) ADC_convertToMicroVolts(ADC_INA125, adcValue3);
            adc_value4=microVoltBuffer/1000;
           // ADCValue[2] = v1;
            //Выставим семафор готовности опроса АЦП температуры
            Semaphore_post(ReadyForTemp);
        }
        else{
            GPIO_write(PD_SCK,1);
        }
    }
}



//Задача HX711
// -------------------------------------------------------------------------------------
void hxTask(void)
{
    uint8_t count=0;//Подсёт количества измерений
    unsigned char *ptr;
    uint_fast16_t i,j;

    while(1){
        GPIO_write(PD_SCK,0);
      //Ждём семафора
        Semaphore_pend(ReadyForHX, BIOS_WAIT_FOREVER);

        //Теперь всё зависит от значения счётчика count
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
                    ptr = (unsigned char*) &ADCValue2[i];  //Берём адрес текущего элемента массива ADCValue2
                    txPacket.payload[j++]=*(ptr);                               //Сохраняем первый байт
                    txPacket.payload[j++]=*(ptr+1);                             //Сохраняем второй байт
                    txPacket.payload[j++]=*(ptr+2);                             //Сохраняем третий байт
                    txPacket.payload[j++]=*(ptr+3);                             //Сохраняем четвёртый байт
                }//for для значений с АЦП HX711
           // j ++;
            for(i=0; i<10; i++)
                {
                    ptr = (unsigned char*) &ADCValue[i];  //Берём адрес текущего элемента массива ADCValue
                    txPacket.payload[j++]=*(ptr);                               //Сохраняем первый байт
                    txPacket.payload[j++]=*(ptr+1);                             //Сохраняем второй байт
                    txPacket.payload[j++]=*(ptr+2);                             //Сохраняем третий байт
                    txPacket.payload[j++]=*(ptr+3);                             //Сохраняем четвёртый байт
                 }//for для значений с АЦП INA125

            ptr = (unsigned char*) &Temp;  //Берём адрес переменной Temp (в ней хранится значение температуры)
            txPacket.payload[j++]=*(ptr);                               //Сохраняем первый байт
            txPacket.payload[j++]=*(ptr+1);                             //Сохраняем второй байт
            txPacket.payload[j++]=*(ptr+2);                             //Сохраняем третий байт
            txPacket.payload[j++]=*(ptr+3);                             //Сохраняем четвёртый байт

            ptr = (unsigned char*) &VDDio;  //Берём адрес переменной VDDio (в ней хранится значение напряжения питания в миливольтах)
            txPacket.payload[j++]=*(ptr);                               //Сохраняем первый байт
            txPacket.payload[j++]=*(ptr+1);                             //Сохраняем второй байт
            txPacket.payload[j++]=*(ptr+2);                             //Сохраняем третий байт
            txPacket.payload[j++]=*(ptr+3);                             //Сохраняем четвёртый байт

            txPacket.len = 128;
            Semaphore_post(ReadyForTransmitA);//Выставляем семафор готовности передачи по радиоканалу
            //GPIO_enableInt(D_OUT);//Включим прерывание по ноге D_OUT
            //flag = FALSE;
            Task_sleep(1000);//Дрыхнем

        }
        else
        {
            //GPIO_write(PD_SCK,0);
            //Заполняем буфер данными. Инкремент счётчика
             ADCValue2[count]=adc_value1;//Значения с HX711
             ADCValue[count]=adc_value4;//Значения с INA125
             count++;//Инкремент счётчика измерений
             //adc_value1=0;
             //Включаем внешние прерывания
             //Добавим математику
             /*
             adc_value2 = ((adc_value1 - 32123)*98)/106080-1218;
             size_of_print = sprintf (str, "\r\n%d %d %d %d\r\n",adc_value2,ADCValue3[0],ADCValue3[1],ADCValue3[2]);
             UART_write(uart,&str,size_of_print);
             */
             adc_value1=0;
             adc_value2=0;
             adc_value4=0;
             GPIO_enableInt(D_OUT);//Включим прерывание по ноге D_OUT
             Task_sleep(1000);
        }

    }
}





/*
// Задача BIOS, работает непрерывно, оживает по семафору GeneralError
// При работе с семаформаи передаем управление RTOS через Semaphore_pend()
//---------------------------------------------------------------------------------------
void ErrorHandlerTask(UArg arg0, UArg arg1)
{
int i,l;
  while(1){ //Сюда попадаем только при критических ошибках
     Semaphore_pend(GeneralError, BIOS_WAIT_FOREVER); //Ждем наступления семафора
     txPacket.payload[0] = MASTER_ID_DEBUG_T;
     txPacket.payload[1] = DeviceID >> 16;
     txPacket.payload[2] = (DeviceID >> 8) & 0xFF;
     txPacket.payload[3] = DeviceID & 0xFF;
     txPacket.dstAddr[0] = GROUP_CODE;
     txPacket.absTime = 0;
     l = strlen(&ErrorMsg[0]);
     for(i=0; i<l; i++) txPacket.payload[i] = ErrorMsg[i];
     txPacket.len = l + 4;
     EasyLink_transmit(&txPacket); //Функция библиотеки EasyLink
  }
}
*/



// Задача BIOS, работает непрерывно, по семафору ReadyForTransmit передает пакет 868 MHz
// Структура данных хранится в буфере Buf_A[], длина буфера RFStrCnt
//---------------------------------------------------------------------------------------
void RFTransmitTaskA(UArg arg0, UArg arg1)
{
int i;
   while(1){
       Semaphore_pend(ReadyForTransmitA, BIOS_WAIT_FOREVER); //Ждем наступления семафора (передаем управление RTOS)
       txPacket.payload[0] = MASTER_ID_DEBUG_S;
       txPacket.payload[1] = DeviceID >> 16;
       txPacket.payload[2] = (DeviceID >> 8) & 0xFF;
       txPacket.payload[3] = DeviceID & 0xFF;
       txPacket.len        = 128; //
       txPacket.dstAddr[0] = GROUP_CODE;
       txPacket.absTime = 0;

       EasyLink_transmit(&txPacket); //Функция библиотеки EasyLink

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
    //EasyLink_enableRxAddrFilter(addrFilter, 1, 2); //Для первой строчки 2, для второй 7
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
    //uartParams.writeCallback = writeCallback; //Прерывание
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
    UART_Params_init(&uartParams); //Сначала создать потом заполнить:(Настройка gsm UART сисдит в MDD_BASE_hub.c)
    uartParams.writeDataMode = UART_DATA_BINARY;    //UART_DATA_TEXT  UART_DATA_BINARY
    uartParams.readDataMode =  UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL ;  // UART_RETURN_FULL UART_RETURN_NEWLINE
    uartParams.readEcho = UART_ECHO_OFF;            // UART_ECHO_ON
    uartParams.dataLength = UART_LEN_8;
    uartParams.stopBits = UART_STOP_ONE;
    uartParams.baudRate = 115200;
    uartParams.readMode = UART_MODE_BLOCKING;   // UART_MODE_CALLBACK UART_MODE_BLOCKING
    //uartGSMParams.readCallback = UART_RxFxn;     //Обработчик прерывания
    //uartParams.writeCallback = writeCallback;    //Обработчик прерывания
    uartParams.readTimeout = 1000000;//500000;   //No timeout in UART_MODE_CALLBACK
    uartParams.writeTimeout= 1000000;            //UART_WAIT_FOREVER
}

*/



/*
 *  ============== Обработчик прерывания
 */
void IrqD_OUT(uint_least8_t index)
{

    GPIO_disableInt(D_OUT);//Отключили внешнее прерывание
    count_bits = 0;//Обнулили счётчик бит
    adc_value1=0;//Обнулили переменную
    GPIO_write(PD_SCK,0);
    GPTimerCC26XX_start(hTimer_I);
    GPIO_toggle(LED2);
    //flag = TRUE;

   /*
    count_bits = 0;//Обнулили счётчик бит
    GPIO_write(PD_SCK,0);
    GPTimerCC26XX_start(hTimer_I);//запускаем таймер
    GPIO_enableInt(D_OUT);//Отключили внешнее прерывание
    */
    /*
 uint8_t i=0;
    //Отключили прерывание D_OUT
    GPIO_disableInt(D_OUT);
    //Запустили цикл
    for( i=0;i<24;i++)
    {
        //Подняли строб
        GPIO_write(PD_SCK,1);
        //Сдвинули значение ADC на 1 влево
        adc_value1<<=1;
        Task_sleep(1);
        //Опустили строб
        GPIO_write(PD_SCK,0);
        //Проверяем уровень на пине D_OUT
        //Если там "1" то инкрементируем значение ADC
        if(GPIO_read(D_OUT)>0)
        {
            adc_value1++;
        }
        Task_sleep(1);
    }

    //Подняли строб
     GPIO_write(PD_SCK,1);
     //Подождали
     Task_sleep(1);
     //Опустили строб
     GPIO_write(PD_SCK,0);
     Task_sleep(1);
     //Выставляем симафор
     Semaphore_post(ReadyForHX);
     GPIO_enableInt(D_OUT);//Включим прерывание по ноге D_OUT
     Task_sleep(1000);
*/

}

