#include <rtthread.h>
#include <stdlib.h>
#include "serial.h"
#include "lib_Misc.h"
#include "tad_LCD.h"
#include "tad_Downloader.h"
#include "I2C_Eeprom.h"
#include "tad_lcd.h"

#define   SOH    0x01
#define   ACK    0x06
#define   EOT    0x04
#define   CAN    0x18
#define   STX    0x02
#define   NAK    0x15
#define   PAK    0x88
#define   CCE    0x99

#pragma pack(1)
typedef struct _TaDownFile_
{
    uint32_t MagicNum;
    uint8_t  FileName [64];
    uint8_t FileType;
    uint32_t FileLen;
} TaDownFile;

enum _Tad_Status_
{
    Status_Idle,
    Status_Menu,
    Status_WaitingConnect,
    Status_Burning,
};

typedef enum _Tad_FileType_
{
    Type_Fw = 1,
    Type_Menu,
    Type_Osd,
} File_Type;

enum _Tad_KeyValue_
{
    TadKey_Start = 0x01,
    TadKey_Cancel = 0x02,
    TadKey_Open = 0x04,	//0x04,
    TadKey_Close = 0x08,
};

typedef struct _FLASH_ObjInfo_
{
    File_Type FileType;
    uint32_t FontFileOs;
    uint8_t FileName[64];
    uint32_t TargetOs;	/*需要被烧写到目标板的位置*/
} ObjInfo;

#pragma pack()
ObjInfo flashobj[10];	/*flash中可存放最多10个对象*/
ObjInfo SelectedObj;	/*当前被选中的对象*/
ObjInfo* theMenuObj = NULL;
ObjInfo* theHzkobj = NULL;
ObjInfo* thePinyinObj = NULL;
ObjInfo* theASCIIObj = NULL;

rt_device_t pCom232;
rt_device_t pCom485;
rt_device_t pSpiFlash;
rt_device_t pLcd1602;

enum _Tad_Status_ tad_status = Status_Idle;

uint32_t CurBps = 2;
uint8_t AutoUpdateEn = 0;
rt_timer_t ComTimer;

#ifdef _BEEP_THREAD_
rt_thread_t BeepThread;
rt_timer_t BeepTimeOut;
#endif
uint8_t FwDownLoadFlag = 0;

//const char OsStr[8][4] = {{"00 "}, {"04 "}, {"08 "}, {"0C "}, {"10 "}, {"14 "}, {"18 "}, {"1C"}};
const char OsStr[10][4] = {{"01 "}, {"02 "}, {"03 "}, {"04 "}, {"05 "}, {"06 "}, {"07 "}, {"08"}, {"09"}, {"10"}};

const uint8_t taReboot[7] = {0x99, 0x00, 0x07, 0xAA, 0x00, 0x00, 0x4A};

const uint8_t taConnectPassW[5] = {'T', 'U', 'O', 'A', 'N'};

const char BpsStr[3][20] = {{"57600-N-1"}, {"115200-N-1"}, {"230400-N-1"}};

#define GPIO_PORT_BEEP	GPIOB
#define GPIO_PIN_BEEP	GPIO_Pin_12

void TaBeepThread_entry();

void taD_KeyPortInit()
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_GPIOB , ENABLE);

    /* Configure SPI1 pins: DO SCK  and DI */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU;	//Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_PIN_BEEP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//Out_PP;
    GPIO_Init(GPIO_PORT_BEEP, &GPIO_InitStructure);
}

uint8_t taD_GetKeyValue()
{
    uint16_t PortVal;
    PortVal = GPIO_ReadInputData(GPIOC);

    return (uint8_t)(((~PortVal) & 0x003c) >> 2);
}

uint8_t taD_CheckKey(uint8_t Key)
{
    if (taD_GetKeyValue() == Key)
    {
        rt_thread_delay(300);
        if (taD_GetKeyValue() == Key)
        {
            return 1;
        }
    }
    return 0;
}

#ifdef _BEEP_THREAD_
void BeepTimeOutProc(void *para)
{
    //if ((BeepThread->stat == RT_THREAD_READY) || (BeepThread->stat == RT_THREAD_RUNNING))
    {
        rt_thread_delete(BeepThread);
        rt_thread_delay(100);
    }
    GPIO_ResetBits(GPIO_PORT_BEEP, GPIO_PIN_BEEP);
}
#endif


void taD_IdleHandle()
{
    rt_size_t RsBytes = 0;
    uint8_t RcvData = 0;
    uint8_t Key = taD_GetKeyValue();
    if (Key == TadKey_Start)
    {
        rt_thread_delay(150);
        if (taD_GetKeyValue() == TadKey_Start)
        {
            tad_status = Status_WaitingConnect;
            ComTimeOutHandle((void *)0);
            while (taD_GetKeyValue() & TadKey_Start);
            //rt_thread_delay(1000);
            return;
        }
    }
    else if (Key == TadKey_Open)
    {
        rt_thread_delay(1000);
        if (taD_GetKeyValue() == TadKey_Open)
        {
            tad_status = Status_Menu;
            while (taD_GetKeyValue() & TadKey_Open);
            return;
        }
    }
    RsBytes = pCom485->read(pCom485, 0, &RcvData, 1);		//用485烧写固件
    if (RsBytes)
    {
        ta_CmdPaser(RcvData);
        rt_timer_start(ComTimer);
    }
    else
    {
        rt_thread_delay(20);
    }
}

uint8_t WaitKey()
{
    uint8_t CurKey = 0;
    while (1)
    {
        CurKey = taD_GetKeyValue();
        if (CurKey)
        {
            rt_thread_delay(80);
            if (CurKey == taD_GetKeyValue())
            {
                return CurKey;
            }
        }
    }
}

uint8_t usart_GetData(rt_device_t Dev, uint8_t *Buffer, uint8_t Size, uint32_t TimeOut)
{
    uint32_t  i;
    uint8_t RxByte = Size;
    uint8_t *pBuf = Buffer;

    for (i = 0; i < TimeOut; i++)
    {
        if (pCom485->read(pCom485, 0, pBuf, 1))
        {
            Size--;
            if (!Size)
            {
                return 1;
            }
            pBuf++;
        }
        else
        {
            rt_thread_delay(1);
        }
    }
    return 0;
}


const uint16_t SucessBeep[] = {500, 200, 1000, 200, 200, 100, 200, 100};
const uint16_t FailBeep[] = {2000, 100, 2000, 100, 2000, 100, 2000, 100};

void TaBeepThread_entry()
{
    uint8_t i, Flag = 1;
    for (i = 0; i < 8; i++)
    {
        if (Flag)
        {
            GPIO_SetBits(GPIO_PORT_BEEP, GPIO_PIN_BEEP);
            Flag = 0;
        }
        else
        {
            GPIO_ResetBits(GPIO_PORT_BEEP, GPIO_PIN_BEEP);
            Flag = 1;
        }
        if (FwDownLoadFlag)
        {
            rt_thread_delay(SucessBeep[i]);
        }
        else
        {
            rt_thread_delay(FailBeep[i]);
        }
    }
}

void TaDownLoaderInit()
{
    uint8_t FileNum, i;
    uint32_t tmpFileOs;

    I2c_Soft_Init(1);
    I2c_Soft_E2IcTypeSet(USE_24C256);

    tad_status = Status_Idle;
    taD_KeyPortInit();

    pSpiFlash = rt_device_find("flash");
    pCom232 = rt_device_find("uart1");
    pCom485 = rt_device_find("485");
    pLcd1602 = rt_device_find("Lcd1602");

    FileNum = ScanFlash();

    CurBps = ReadByte(BPS_SETTING_OS);
    if(CurBps > 2)
    {
        CurBps = 2;
    }
    AutoUpdateEn = ReadByte(AutoUpdate_SET_OS);
    if(AutoUpdateEn >= 2)
    {
        AutoUpdateEn = 0;
    }

    if (CurBps > 2)
    {
        CurBps = 2;
        WriteByte(BPS_SETTING_OS, CurBps);
    }
    pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[CurBps]);

    tmpFileOs = ReadDword(FILE_SEL_OS);

    for (i = 0; i < 10; i++)
    {
        if (tmpFileOs == flashobj[i].FontFileOs)
        {
            //SelectedObj.FontFileOs= tmpFileOs;
            memcpy(&SelectedObj, &flashobj[i], sizeof(ObjInfo));
            break;
        }
    }
    if (i >= 10)
    {
        //SelectedObj.FontFileOs = flashobj[0].FontFileOs;
        memcpy(&SelectedObj, &flashobj[i], sizeof(ObjInfo));
        WriteDword(FILE_SEL_OS, SelectedObj.FontFileOs);
    }
    //memcpy(SelectedObj.FileName, flashobj[i].FileName, 64);
    pLcd1602->write(pLcd1602, -1, SelectedObj.FileName, 32);


#if 0
    FwDownLoadFlag = 1;
    rt_thread_startup(BeepThread);
    rt_thread_delay(9000);
    FwDownLoadFlag = 0;
    rt_thread_delay(9000);
    BeepTimeOutProc((void *)0);
#endif

    ComTimer = rt_timer_create("ComTO", ComTimeOutHandle, (void *)0, 2000, 1);		//3秒钟后若无串口数据则清除接收标记位
    //ComTimer = rt_timer_create("ComTimeOut", ComTimeOutHandle, (void *)0, 5000, 1);
}

extern struct rt_mutex SysInit;

void rt_TaDownLoader_thread_entry(void* parameter)
{
    rt_thread_delay(1000);
#if 1
    TaDownLoaderInit();

    while (1)
    {
        switch (tad_status)
        {
        case Status_Menu:
            taD_MenuHandle();
            break;
        case Status_WaitingConnect:
            taD_ConnectHandle();
            break;
        default:
            taD_IdleHandle();
            break;
        }
    }
#endif
}

