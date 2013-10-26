#include "taD_Menu.h"


const uint8_t FirstLevelStr[][16] =
{
    {"->Select File   "},
    {"->Setting Bps   "},
    {"->AutoUpdate Y/N"}
};

const uint8_t BpsStrSetting[][16] =
{
    {"->57600Bps   "},
    {"->115200Bps   "},
    {"->230400Bps   "},
};

const uint8_t AutoUpdateSetting[][16] =
{
    {"->No"},
    {"->Yes"}
};

struct _MenuStruct_
{
    uint8_t MenuId;
    uint8_t Menu;
};

struct _FlashFile_
{
    uint8_t Flag;
};

uint8_t lib_TraverseItem(uint8_t byMaxItemNo, uint8_t byCurItemNo, uint8_t byFlag) //0~MaxItemNo-1; Flag Ture: 加  False: 减
{
    if (byFlag == 1)
    {
        if (byCurItemNo < (byMaxItemNo - 1))
        {
            byCurItemNo++;
        }
    }
    else
    {
        if (0 < byCurItemNo)
        {
            byCurItemNo--;
        }
    }
    return byCurItemNo;
}

uint8_t ScanFlash()
{
    uint8_t FileNum = 0;
    uint8_t i;
    uint8_t FileHead[sizeof(TaDownFile)];
    struct _TaDownFile_ *pFileHead;
#if 1
    for (i = 0; i < 7; i++)
    {
        memset(FileHead, 0, sizeof(TaDownFile));
        pSpiFlash->read(pSpiFlash, i*0x40000, FileHead, sizeof(TaDownFile));
        pFileHead = (TaDownFile *)FileHead;
        if (pFileHead->MagicNum == 0x55AA)
        {
            memcpy(flashobj[FileNum].FileName, (const void *)&(pFileHead->FileName), 64);
            flashobj[FileNum].FontFileOs=  i*0x40000;
            flashobj[FileNum].FileType = taD_GetFileType(flashobj[FileNum].FileName);
            if(flashobj[FileNum].FileType == Type_Menu)
            {
                theMenuObj = &flashobj[FileNum];	/*选中菜单这个对象*/
                flashobj[FileNum].TargetOs = 0x80000;
            }
            FileNum++;
        }
    }

    /*将固定的三个文件加到对象数组中*/
    strcpy(flashobj[FileNum].FileName, "hzk14.bin");/*汉字库*/
    flashobj[FileNum].FontFileOs = 0x1c0000;
    flashobj[FileNum].FileType = Type_Osd;
    flashobj[FileNum].TargetOs = 0x20000;
    theHzkobj = &flashobj[FileNum];

    strcpy(flashobj[FileNum + 1].FileName, "ASC16_spec.bin");	/*ASCII*/
    flashobj[FileNum + 1].FontFileOs = 0x1f9000;
    flashobj[FileNum + 1].FileType = Type_Osd;
    flashobj[FileNum + 1].TargetOs = 0x10000;
    theASCIIObj = &flashobj[FileNum + 1];

    strcpy(flashobj[FileNum + 2].FileName, "pinyin_tol.bin");	/*拼音*/
    flashobj[FileNum + 2].FontFileOs = 0x1fb000;
    flashobj[FileNum + 2].FileType = Type_Osd;
    flashobj[FileNum + 2].TargetOs = 0x0;
    thePinyinObj = &flashobj[FileNum + 2];


#else
    {
        const uint8_t CFileName [][64] =
        {
            {"TAD-AH_01.09.20.bin"},
            {"TAD-RH.IR10_01.09.20.bin"},
            {"TAD-RJH.IR10_01.09.20.bin"},
            {"TAD-RJHL.IR10_01.09.20.bin"}
        };
        for (i = 0; i < 4; i++)
        {
            memcpy(FileName[FileNum], CFileName[i], 64);
            FileOs[FileNum] =  i*0x40000;
            FileNum++;
        }
    }
#endif
    return (FileNum + 3);
}

void taD_MenuHandle()
{
    uint8_t MenuLevel = 0;
    uint8_t LevelOneIndex = 0;
    uint8_t LevelTwoIndex = 0;
    uint8_t LevelTwoNum = 3;
    uint8_t MenuId = 0;
    uint8_t Key = 0;
    uint8_t tmpOs;

    pLcd1602->write(pLcd1602, -1, FirstLevelStr[LevelOneIndex], 16);
    while (1)
    {
        Key = WaitKey();
        if (Key == TadKey_Open)
        {
            if (0 == MenuId)
            {
                if (0 == LevelOneIndex)
                {
                    LevelTwoNum = ScanFlash();

                    //tmpOs = FileOs[LevelTwoIndex]/0x40000;
                    tmpOs = LevelTwoIndex;

                    pLcd1602->write(pLcd1602, -1, "->", 32);
                    pLcd1602->write(pLcd1602, 2, OsStr[tmpOs], 3);
                    pLcd1602->write(pLcd1602, 5, flashobj[LevelTwoIndex].FileName, 32);
                    MenuId = 1;
                }
                else if(1 == LevelOneIndex)
                {
                    LevelTwoIndex = CurBps;
                    pLcd1602->write(pLcd1602, -1, BpsStrSetting[LevelTwoIndex], 16);
                    MenuId = 2;
                }
                else
                {
                    LevelTwoIndex = AutoUpdateEn;
                    pLcd1602->write(pLcd1602, -1, AutoUpdateSetting[LevelTwoIndex], 16);
                    MenuId = 3;
                }
            }
            else if(1 == MenuId)
            {
                memcpy(&SelectedObj, &flashobj[LevelTwoIndex], sizeof(ObjInfo));
                //SelectedObj.FontFileOs = flashobj[LevelTwoIndex].FontFileOs;
                WriteDword(FILE_SEL_OS, SelectedObj.FontFileOs);		//记住选择
                //memcpy(SelectedObj.FileName, flashobj[LevelTwoIndex].FileName, 64);
                tad_status = Status_Idle;
                break;
            }
            else if(2 == MenuId)
            {
                CurBps = LevelTwoIndex;
                WriteByte(BPS_SETTING_OS, CurBps);		//记住选择
                pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)&BpsStr[CurBps]);
                tad_status = Status_Idle;
                break;
            }
            else if(3 == MenuId)
            {
                AutoUpdateEn = LevelTwoIndex;
                WriteByte(AutoUpdate_SET_OS, AutoUpdateEn);
                tad_status = Status_Idle;
                break;
            }
        }
        else if (Key == TadKey_Close)
        {
            tad_status = Status_Idle;
            break;
        }
        else if (Key == TadKey_Start)
        {
            if (0 == MenuId)
            {
                LevelOneIndex = lib_TraverseItem(3, LevelOneIndex, 1);
                pLcd1602->write(pLcd1602, -1, FirstLevelStr[LevelOneIndex], 16);
            }
            else if(1 == MenuId)
            {
                LevelTwoIndex = lib_TraverseItem(LevelTwoNum, LevelTwoIndex, 1);
                //tmpOs = FileOs[LevelTwoIndex]/0x40000;
                tmpOs = LevelTwoIndex;
                pLcd1602->write(pLcd1602, -1, "->", 32);
                pLcd1602->write(pLcd1602, 2, OsStr[tmpOs], 3);
                pLcd1602->write(pLcd1602, 5, flashobj[LevelTwoIndex].FileName, 32);
            }
            else if(2 == MenuId)
            {
                LevelTwoIndex = lib_TraverseItem(LevelTwoNum, LevelTwoIndex, 1);
                pLcd1602->write(pLcd1602, -1, BpsStrSetting[LevelTwoIndex], 16);
            }
            else if(3 == MenuId)
            {
                LevelTwoIndex = lib_TraverseItem(2, LevelTwoIndex, 1);
                pLcd1602->write(pLcd1602, -1, AutoUpdateSetting[LevelTwoIndex], 16);
            }
        }
        else if (Key == TadKey_Cancel)
        {
            if (0 == MenuId)
            {
                LevelOneIndex = lib_TraverseItem(3, LevelOneIndex, 0);
                pLcd1602->write(pLcd1602, -1, FirstLevelStr[LevelOneIndex], 16);
            }
            else if(1 == MenuId)
            {
                LevelTwoIndex = lib_TraverseItem(LevelTwoNum, LevelTwoIndex, 0);
                //tmpOs = FileOs[LevelTwoIndex]/0x40000;
                tmpOs = LevelTwoIndex;
                pLcd1602->write(pLcd1602, -1, "->", 32);
                pLcd1602->write(pLcd1602, 2, OsStr[tmpOs], 3);
                pLcd1602->write(pLcd1602, 5, flashobj[LevelTwoIndex].FileName, 32);
            }
            else if(2 == MenuId)
            {
                LevelTwoIndex = lib_TraverseItem(LevelTwoNum, LevelTwoIndex, 0);
                pLcd1602->write(pLcd1602, -1, BpsStrSetting[LevelTwoIndex], 16);
            }
            else if(3 == MenuId)
            {
                LevelTwoIndex = lib_TraverseItem(2, LevelTwoIndex, 0);
                pLcd1602->write(pLcd1602, -1, AutoUpdateSetting[LevelTwoIndex], 16);
            }
        }
        Key = 0;
        rt_thread_delay(150);
    }
    pLcd1602->write(pLcd1602, -1, SelectedObj.FileName, 32);
    rt_thread_delay(1000);
}
