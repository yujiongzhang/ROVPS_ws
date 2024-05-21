#include "define_sts1000.h"



//初始化命令包
void init_cmd(CMD_STS1000 *cmd_ptr)
{
    cmd_ptr->ucID = 0x00;
    cmd_ptr->ucWorkStatus = 0x2b;
    cmd_ptr->ucRange = 0x03;
    cmd_ptr->ucDataType = 0x09;
    cmd_ptr->ucTrainAngleLo = 0x00;
    cmd_ptr->ucTrainAngleHi = 0x00;
    cmd_ptr->ucSectorWidthLo = 0x10;
    cmd_ptr->ucSectorWidthHi = 0x1c;
    cmd_ptr->ucStartGain = 0xe5;
    cmd_ptr->ucAbsorption = 0x11;
    cmd_ptr->ucStepSize = 0x02;
    cmd_ptr->ucPulseType = 0x00;
    cmd_ptr->ucFreqS = 0x0a;
    cmd_ptr->ucFreqE = 0x0f;
    cmd_ptr->ucSoundSpeed = 0x7d;
    cmd_ptr->ucCheckSum = 0x00;
}

void updateCheckSum(CMD_STS1000 *cmd_ptr)
{
    cmd_ptr->ucCheckSum = 0x00;
    unsigned char sum = 0;
//    unsigned char * p = (unsigned char *)&cmd_sts1000;
    unsigned char *p = reinterpret_cast<unsigned char*>(cmd_ptr);
    for(unsigned int i = 0; i<sizeof(CMD_STS1000); i++)
    {
        sum += *p++;
    }
    cmd_ptr->ucCheckSum = -sum;
}


