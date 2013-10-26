#include "tad_update.h"


uint8_t taCmdMatchIndex;
uint8_t taCmdMatchFlag;
uint8_t taFileBuffer[138];
uint8_t taCmdLen = 0;

const uint8_t taCmd[] = {0x99, 0x00, 0x07, 0xAD};

void taD_ConnectHandle()
{
    if (SelectedObj.FileType == Type_Menu)//(taD_GetFileType() == Type_Menu)//FONT_FILE_T)
    {
        pCom485->control(pCom485, RT_SERIAL_CMD_ClearRxBuff, (void *)0);
        if(taD_FontDown(SelectedObj))
        {
            FwDownLoadFlag = 1;
            pLcd1602->control(pLcd1602, DispEnd, (void *)0);
            rt_thread_delay(500);
            pLcd1602->write(pLcd1602, 16, "Sucess! ++++++++", 16);
        }
        else
        {
            FwDownLoadFlag = 0;
            pLcd1602->control(pLcd1602, DispEnd, (void *)0);
            rt_thread_delay(500);
            pLcd1602->write(pLcd1602, 16, "Fail!!!!--------", 16);
        }
        pCom485->control(pCom485, RT_SERIAL_CMD_ClearRxBuff, (void *)0);
    }
    else if(SelectedObj.FileType == Type_Osd)
    {
        if(taD_FontDown(SelectedObj))
        {
            FwDownLoadFlag = 1;
            pLcd1602->control(pLcd1602, DispEnd, (void *)0);
            rt_thread_delay(500);
            pLcd1602->write(pLcd1602, 16, "Sucess! ++++++++", 16);
        }
        else
        {
            FwDownLoadFlag = 0;
            pLcd1602->control(pLcd1602, DispEnd, (void *)0);
            rt_thread_delay(500);
            pLcd1602->write(pLcd1602, 16, "Fail!!!!--------", 16);
        }
    }
    else
    {
        //taD_AutoUpdateOsd();
        #if 1
        if (taD_ConnectBootLoader(2400, 115200, 200))
        {

            if(taD_FwDownloader(SelectedObj.FontFileOs))
            {
                FwDownLoadFlag = 1;
                pLcd1602->control(pLcd1602, DispEnd, (void *)0);
                rt_thread_delay(500);
                pLcd1602->write(pLcd1602, 16, "Sucess! ++++++++", 16);
                rt_thread_delay(2000);  /*等待球机烧完程序之后重启*/
                pCom485->control(pCom485, RT_SERIAL_CMD_ClearRxBuff, (void *)0);
                pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, "2400-N-1");

                if(AutoUpdateEn)
                {
                    uint8_t query[7] = {0xff, 0x00, 0x00, 0x51, 0x00, 0x00, 0x00};
                    uint8_t reply = 0;

                    pLcd1602->control(pLcd1602, DispEnd, (void *)0);
                    rt_thread_delay(500);
                    pLcd1602->write(pLcd1602, 0, "Auto update ....", 16);
                    pLcd1602->write(pLcd1602, 16, "Waiting ........", 16);

                    while(!usart_GetData(pCom485, &reply, 1, 300))
                    {
                        rt_thread_delay(1000);
                        pCom485->write(pCom485, 0, query, 7);
                    }
                    pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[CurBps]);

                    rt_thread_delay(50);
                    if(taD_AutoUpdateOsd())
                    {
                        pLcd1602->control(pLcd1602, DispEnd, (void *)0);
            		rt_thread_delay(500);
                        pLcd1602->write(pLcd1602, 16, "Sucess! ++++++++", 16);
                    }
                    else
                    {
                        pLcd1602->control(pLcd1602, DispEnd, (void *)0);
            		rt_thread_delay(500);
                        pLcd1602->write(pLcd1602, 16, "Fail!!!!--------", 16);
                    }
                }
            }
            else
            {
                FwDownLoadFlag = 0;
                pLcd1602->control(pLcd1602, DispEnd, (void *)0);
                rt_thread_delay(500);
                pLcd1602->write(pLcd1602, 16, "Fail!!!!--------", 16);
            }
        }
        else
        {
            FwDownLoadFlag = 0;
            pLcd1602->control(pLcd1602, DispEnd, (void *)0);
            rt_thread_delay(500);
            pLcd1602->write(pLcd1602, 16, "Fail!!!!--------", 16);
        }
        #endif
    }
    TaBeepThread_entry();
    tad_status = Status_Idle;
    pLcd1602->control(pLcd1602, DispEnd, (void *)0);
    rt_thread_delay(2000);
    pLcd1602->write(pLcd1602, -1, SelectedObj.FileName, 32);
    return;
}

void ComTimeOutHandle(void *paramter)
{
    taCmdMatchIndex = 0;
    pCom485->control(pCom485, RT_DEVICE_CTRL_SUSPEND, (void *)RT_NULL);
    pCom485->control(pCom485, RT_DEVICE_CTRL_RESUME, (void *)RT_NULL);
    //pCom232->control(pCom232, RT_SERIAL_CMD_ClearRxBuff, (void *)RT_NULL);
    rt_timer_stop(ComTimer);
}

uint8_t taD_ConnectBootLoader(uint32_t preBps, uint32_t DwnBps, uint32_t RetryTimes)
{
    uint32_t i, j;
    uint8_t tmpRxBuffer[5];
    uint8_t *pBuffer;
    pLcd1602->write(pLcd1602, -1, SelectedObj.FileName, 32);
    rt_thread_delay(200);
    //pLcd1602->control(pLcd1602, DispConnecting, (void *)0);
    for (i = 0; i < RetryTimes; i++)
    {
        pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, "2400-N-1");
        pCom485->write(pCom485, 0, taReboot, 7);
        rt_thread_delay(80);

#if 1
        pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[CurBps]);
        for (j = 0; j < 40; j++)
        {
            uint8_t SynData[2] = {0x55, 0xAA};
            pCom485->write(pCom485, 0, SynData, 1);
            Delay_1us(50);
            pCom485->write(pCom485, 0, (SynData + 1), 1);
            rt_thread_delay(1);
            if (taD_CheckKey(TadKey_Cancel))
            {
                pLcd1602->control(pLcd1602, DispEnd, (void *)0);
                pLcd1602->write(pLcd1602, -1, SelectedObj.FileName, 32);
                tad_status = Status_Idle;
                return 0;
            }
        }

        pCom485->control(pCom485, RT_SERIAL_CMD_ClearRxBuff, (void *)0);
        for (j = 0; j < 5; j++)
        {
            pCom485->write(pCom485, 0, taConnectPassW, 5);

            if (usart_GetData(pCom485, tmpRxBuffer, 1, 150))
            {
                if (0x43 == tmpRxBuffer[0])
                {
                    uint8_t percent = 0;
                    //pLcd1602->control(pLcd1602, DispProcessing, (void *)&percent);
                    return 1;
                }
                pCom485->control(pCom485, RT_SERIAL_CMD_ClearRxBuff, (void *)RT_NULL);
            }
            if (taD_CheckKey(TadKey_Cancel))
            {
                pLcd1602->control(pLcd1602, DispEnd, (void *)0);
                pLcd1602->write(pLcd1602, -1, SelectedObj.FileName, 32);
                tad_status = Status_Idle;
                return 0;
            }
        }
#endif
        rt_thread_delay(100);
    }
    return 0;
}

uint16_t xmodem_CheckSumAdd(uint8_t *pData, uint8_t Size)
{
    uint16_t i, cs = 0;
    if (-1 == Size)
        return 0;
    for (i = 0; i < Size; i++)
    {
        cs += *pData;
        pData++;
    }
    return cs;
}

void ta_CmdPaser(uint8_t Data)
{
    if (taCmdMatchIndex < 3)
    {
        if (taCmdMatchIndex < 2)
        {
            if (Data == taCmd[taCmdMatchIndex])
            {
                taFileBuffer[taCmdMatchIndex] = Data;
                taCmdMatchIndex++;
                return;
            }
            else
            {
                taCmdMatchIndex = 0;
                return;
            }
        }
        else
        {
            taFileBuffer[taCmdMatchIndex] = Data;
            taCmdMatchIndex++;
            taCmdLen = taFileBuffer[2];
        }
    }
    else
    {
        taFileBuffer[taCmdMatchIndex] = Data;
        taCmdMatchIndex++;
        if (taCmdMatchIndex >= taCmdLen)
        {
            uint8_t abyTmp[3];
            uint32_t offset;
            if (lib_ChecksumAdd(taFileBuffer, taCmdLen - 1) == taFileBuffer[taCmdLen - 1])
            {
                switch (taFileBuffer[3])
                {
                case 0xAD:
                    rt_thread_delay(80);
                    abyTmp[0] = 0x06;
                    pCom485->write(pCom485, 0, abyTmp, 1);
                    break;
                case 0xAB:
                    //if (taCmdLen == 41)
                {
                    //offset = *((uint32_t *)&taFileBuffer[4]);
                    offset = (((uint32_t)taFileBuffer[4])<<24) | (((uint32_t)taFileBuffer[5])<<16) | (((uint32_t)taFileBuffer[6])<<8) | ((uint32_t)taFileBuffer[7]);
                    pSpiFlash->write(pSpiFlash, offset, &taFileBuffer[8], 32);
                    //rt_thread_delay(10);
                    abyTmp[0] = 0x06;
                    pCom485->write(pCom485, 0, abyTmp, 1);
                }
                break;
                }
            }
            ComTimeOutHandle((void *)0);
            //taCmdMatchIndex = 0;
#if 0
            if (lib_ChecksumAdd(taFileBuffer, 6) == taFileBuffer[6])
            {
                rt_size_t RecByte = 0;
                uint8_t abyTmp[3];
                abyTmp[0] = 0x06;
                pCom232->write(pCom232, 0, abyTmp, 1);
                while (1)
                {
                    rt_thread_delay(500);
                    RecByte = pCom232->read(pCom232, 0, taFileBuffer, 41);
                    if (RecByte)
                    {
                        if
                    }
            }
        }
        if (lib_ChecksumAdd(taFileBuffer, 40) == taFileBuffer[40])
            {
                uint32_t offset;
                uint8_t abyTmp[3];
                offset = *((uint32_t *)&taFileBuffer[4]);
                pSpiFlash->write(pSpiFlash, offset, &taFileBuffer[8], 32);
                abyTmp[0] = 0x06;
                pCom232->write(pCom232, 0, abyTmp, 1);
            }
            taCmdMatchIndex = 0;
#endif
        }
    }
}

uint8_t taD_FwDownloader(uint32_t FwFileOs)
{
    uint8_t tmpData[133];
    uint8_t Index = 1;
    uint8_t ErrTryS = 0;
    uint32_t CurDataOs;
    uint32_t TranferedBytes = 0;
    TaDownFile CurFile;
    uint8_t RxBuf[3];
    uint32_t     Promp = 200;
    uint8_t Flag = 1;
    //uint8_t str[16] = {'U', 'p', 'd', 'a', 't', 'i', 'n', 'g','.', '.', '.', '.', ' ', ' ', ' ', '%'};
    
    pSpiFlash->read(pSpiFlash, FwFileOs, (uint8_t *)&CurFile, sizeof(TaDownFile));

    CurDataOs = FwFileOs + sizeof(TaDownFile);
    if (CurFile.FileLen)
    {
        do
        {
            uint32_t ReadBytes;
            uint16_t CheckSum;
            //uint8_t str[16] = {'U', 'p', 'd', 'a', 't', 'i', 'n', 'g','.', '.', '.', '.', ' ', ' ', ' ', '%'};

            if (taD_CheckKey(TadKey_Cancel))
            {
                tad_status = Status_Idle;
                return 0;
            }
            tmpData [0] = SOH;
            tmpData [1] = Index;
            tmpData [2] = ~Index;
            ReadBytes = CurFile.FileLen - TranferedBytes;
            if (ReadBytes > 128)
            {
                pSpiFlash->read(pSpiFlash, CurDataOs, tmpData + 3, 128);
            }
            else
            {
                pSpiFlash->read(pSpiFlash, CurDataOs, tmpData + 3, ReadBytes);
                memset (tmpData + 3 + ReadBytes, 0x00, 128 - ReadBytes);
            }
            CheckSum = xmodem_CheckSumAdd(tmpData + 3, 128);
            tmpData[131] = CheckSum >> 8;
            tmpData[132] = CheckSum & 0xff;

            for (ErrTryS = 0; ErrTryS < 3; ErrTryS++)
            {
                pCom485->control(pCom485, RT_SERIAL_CMD_ClearRxBuff, (void *)RT_NULL);
                pCom485->write(pCom485, 0, tmpData, 133);

                if (usart_GetData(pCom485, RxBuf, 1, 300))	//700
                {
                    if (ACK == RxBuf[0])
                    {
                        uint8_t Percent;
                        uint8_t temp = 0;

                        Index++;
                        CurDataOs += 128;
                        ErrTryS = 0;
                        TranferedBytes += 128;
                        Percent =(uint8_t)((float)TranferedBytes * 100 / CurFile.FileLen);
                        pLcd1602->control(pLcd1602, DispProcessing, (void *)Percent);
#if 0
                        if(TranferedBytes == 128)
                        {
                            pLcd1602->write(pLcd1602, 16,str, 16);
                        }
                        if(Percent % 10 == 0)
                        {
                            taD_Display(Percent);
                        }
#endif
                        if (Promp)
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
                            Promp--;
                        }
                        break;

                    }
                    else
                    {
                        rt_thread_delay(50);
                    }
                }
                else
                {
                    rt_thread_delay(50);
                }
            }
            if (3 <= ErrTryS)
            {
                return 0;
            }
        }
        while(TranferedBytes < CurFile.FileLen);
        tmpData[0] = EOT;
        pCom485->write(pCom485, 0, tmpData, 1);
        return 1;
    }
}

/*根据烧写方式(bootloader烧写和程序中烧写)，将bin文件分成两类*/
File_Type taD_GetFileType(uint8_t objname[])
{
    if(objname[0] == 'T' && objname[2] == 'D')
    {
        return Type_Fw;
    }
    else if(objname[0] == 'T' && objname[2] == 'M')
    {
        return Type_Menu;
    }
    else
        return Type_Osd;
}

/*
  *	功能	: 查询球机主板中的flash 中文件信息
  *    备注	: 菜单0 ~ 200, 0xFF为flash 初始值，说明无菜单包
  */
uint8_t taD_GetCurFontStatus(uint8_t RxBuf[])
{
    uint8_t Reply = 0;
    uint8_t ver_query[8] = {0x99, 0x00, 0x08, 0xF1, 0x0D, 0x00, 0x00, 0x9F};

    pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, "2400-N-1");

#if 0
    uint16_t i = 50;
    while(i--)
    {
        Delay_1ms(10);
        readNum = pCom485->read(pCom485, 0, &menu_verinfo, 1);
        if(readNum)
        {
            //printf("OK\n");
            break;
        }

    }
#endif

    uint8_t num = 50;
    while(--num)
    {
        pCom485->write(pCom485, 0, ver_query, 8);
        usart_GetData(pCom485, RxBuf, 11, 300);
        if(RxBuf[0] == 0x99 && RxBuf[3] == 0xF1)
        {
            //printf("Query OK\n");
            break;
        }
    }
    if(!num)
    {
        pLcd1602->control(pLcd1602, DispEnd, (void *)0);
        rt_thread_delay(500);
        pLcd1602->write(pLcd1602, 16, "AutoUpdateFail!!", 16);
        return 0;
    }
    rt_thread_delay(50);

    return 1;
}

uint8_t taD_FontDown(ObjInfo Object)
{

    uint8_t tmpData[41];
    uint8_t connectHead[7] = {0x99, 0x00, 0x07, 0xAD, 0x00};
    uint16_t Index = 1;
    uint8_t ErrTryS = 0;
    uint32_t CurDataOs;
    uint32_t TranferedBytes = 0;
    TaDownFile CurFile;
    uint8_t RxBuf[3];
    uint32_t     Promp = 200;
    uint8_t Flag = 1;
#if 1
    switch(CurBps)
    {
    case 0:
        connectHead[5] = 0x1B;
        connectHead[6] = 0x68;
        break;
    case 1:
    case 2:	/*230400 波特率烧写不稳定，屏蔽*/
        connectHead[5] = 0x1C;
        connectHead[6] = 0x69;
        break;
#if 0
    case 2:
        connectHead[5] = 0xFF;
        connectHead[6] = 0x4C;
        break;
#endif
    default:
        break;
    }
#endif
    pSpiFlash->read(pSpiFlash, Object.FontFileOs, (uint8_t *)&CurFile, sizeof(TaDownFile));
    uint32_t tmpOs = Object.TargetOs;	/*获取目标文件的烧写地址*/

#if 1
    pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, "2400-N-1");
    rt_thread_delay(50);

    /*首先发送波特率设置信息*/
    uint8_t num = 100;
    while(--num)
    {
        pCom485->write(pCom485, 0, connectHead, 7);
        //Delay_1ms(50);
        usart_GetData(pCom485, RxBuf, 1, 300);
        if(RxBuf[0] == ACK)
        {
            //printf("OK\n");
            break;
        }
    }
    if(!num)
    {
        //printf("Error\n");
        return 0;
    }
#endif

    rt_thread_delay(50);
    //pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[CurBps]);
    if(CurBps == 0)
    {
        pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[0]);
    }
    else
    {
        pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[1]);
    }
    rt_thread_delay(50);

    /*开始发送数据*/
    CurDataOs = Object.FontFileOs+ sizeof(TaDownFile);

#if 0
    uint8_t test[500] = {0};
    pSpiFlash->read(pSpiFlash, 0x0010000, test, 500);
    //st[0] = 0;
    SPI_FLASH_SectorErase(0x10000);
    pSpiFlash->read(pSpiFlash, 0x0010000, test, 500);
#endif

    if (CurFile.FileLen)
    {
        do
        {
            uint32_t ReadBytes;
            uint16_t CheckSum;
            //uint8_t str[16] = {'U', 'p', 'd', 'a', 't', 'i', 'n', 'g','.', '.', '.', '.', ' ', ' ', ' ', '%'};

            if (taD_CheckKey(TadKey_Cancel))
            {
                tad_status = Status_Idle;
                return 0;
            }
            /*4位数据头*/
            tmpData [0] = 0x99;
            tmpData [1] = 0;
            tmpData [2] = 0x29;
            tmpData [3] = 0xAB;

            /*烧写地址*/
            tmpData [4] = (uint8_t)(tmpOs >> 24);
            tmpData [5] = (uint8_t)(tmpOs >> 16);
            tmpData [6] = (uint8_t)(tmpOs >> 8);
            tmpData [7] = (uint8_t)tmpOs;
            tmpOs += 0x20;

            ReadBytes = CurFile.FileLen - TranferedBytes;
            if (ReadBytes > 32)
            {
                pSpiFlash->read(pSpiFlash, CurDataOs, tmpData + 8, 32);
            }
            else
            {
                pSpiFlash->read(pSpiFlash, CurDataOs, tmpData + 8, ReadBytes);
                memset (tmpData + 3 + ReadBytes, 0x00, 32 - ReadBytes);
            }
            CheckSum = xmodem_CheckSumAdd(tmpData, 40);
            //tmpData[40] = CheckSum >> 8;
            tmpData[40] = (uint8_t)(CheckSum & 0xff);

            for (ErrTryS = 0; ErrTryS < 3; ErrTryS++)
            {
                pCom485->control(pCom485, RT_SERIAL_CMD_ClearRxBuff, (void *)RT_NULL);
                pCom485->write(pCom485, 0, tmpData, 41);

                if (usart_GetData(pCom485, RxBuf, 1, 300))	//700
                {
                    if (ACK == RxBuf[0])
                    {
                        uint8_t Percent;
                        uint8_t temp;
#if 0
                        if(Index % 1000 == 0)
                        {
                            printf("%d\n",Index);
                        }
                        Index++;
#endif
                        CurDataOs += 32;
                        ErrTryS = 0;
                        TranferedBytes += 32;
                        Percent =(uint8_t)((float)TranferedBytes * 100 / CurFile.FileLen);
                        pLcd1602->control(pLcd1602, DispProcessing, (void *)Percent);
#if 0
                        if(TranferedBytes == 32)
                        {
                            pLcd1602->write(pLcd1602, 16,str, 16);
                        }
                        if(Percent != temp)
                        {
                            taD_Display(Percent);
                        }
                        temp = Percent;
#endif
                        if (Promp)
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
                            Promp--;
                        }
                        break;

                    }
                    else
                    {
                        rt_thread_delay(50);
                    }
                }
                else
                {
                    rt_thread_delay(50);
                }
            }
            if (3 <= ErrTryS)
            {
                return 0;
            }
        }
        while(TranferedBytes < CurFile.FileLen);
        tmpData[0] = EOT;
        pCom485->write(pCom485, 0, tmpData, 1);
        return 1;
    }

}

uint8_t taD_AutoUpdateOsd()
{
    uint8_t Updateflag = 0;
    uint8_t reply[11] = {0};
    //Updateflag = taD_GetCurFontStatus(reply);
    if(!taD_GetCurFontStatus(reply))
    {
        return 0;
    }
    else
    {
        if(!reply[5])	/*球机flash中无pinyin，更新*/
        {
            pLcd1602->write(pLcd1602, 0, "pinyin_tol.bin..", 16);
            if(!taD_FontDown(*thePinyinObj))
            {
                return 0;
            }
        }
        if(!reply[6])
        {
            pLcd1602->write(pLcd1602, 0, "asc16_spec.bin..", 16);
            if(!taD_FontDown(*theASCIIObj))
            {
                return 0;
            }
        }
        if(!reply[7])
        {
            pLcd1602->write(pLcd1602, 0, "hzk14.bin......", 16);
            if(!taD_FontDown(*theHzkobj))
            {
                return 0;
            }
        }
        if(!reply[8])
        {
            pLcd1602->write(pLcd1602, 0, "TaMenuFile......", 16);
            if(!taD_FontDown(*theMenuObj))
            {
                return 0;
            }
        }
        else if(reply[9] < (theMenuObj->FileName[19] - 48))
        {
            pLcd1602->write(pLcd1602, 0, "TaMenuFile......", 16);
            if(!taD_FontDown(*theMenuObj))
            {
                return 0;
            }
        }
    }
    return 1;
}

uint8_t lib_ChecksumAdd(uint8_t *pArray, uint8_t byNum)
{
    uint8_t    i;
    uint8_t    sum = 0;

    for (i = 0; i < byNum; i++)
    {
        sum += *(pArray + i);
    }
    return sum;
}
