#pragma once
#ifndef DEFINE_STS1000_H
#define DEFINE_STS1000_H



#define RECVBUFSIZE 200000    //实时数据流接收缓冲区大小
#define DATAARYSIZE 200000  //数据大数组大小
#define ANSPKTSIZE 516 //一个应答包大小
#define HMSPPIDATASIZE 500 //sts1000 PPI模式数据量500

#define SCANRANGE 10.0 //扫描范围
#define STARTGAIN 20 //
#define DIFFUSION 5 //


//*******************命令包******************************
typedef struct	// Command To STS1000 Sonar Head
{
    unsigned char ucID;				// 0x10 to 0x1f
    unsigned char ucWorkStatus;		// Bit 0 - 1 = Transmit if Bit 6 = 1 (Slave Mode)
    unsigned char ucRange; 			// 1 to 50
    unsigned char ucDataType; 		// D3:0 - D50点，以50点步进
    unsigned char ucTrainAngleLo; 	// 0 - 359°
    unsigned char ucTrainAngleHi; 	// 0 - 359°
    unsigned char ucSectorWidthLo; 	// 0 - 360°(0表示固定不转，360表示极坐标)
    unsigned char ucSectorWidthHi; 	// 0 - 360°(0表示固定不转，360表示极坐标)
    unsigned char ucStartGain; 		// 0 - 40dB
    unsigned char ucAbsorption; 	// dB/100m
    unsigned char ucStepSize; 		// Bit3:0 - 步距，1 ~ 15
    unsigned char ucPulseType; 		//
    unsigned char ucFreqS;
    unsigned char ucFreqE;
    unsigned char ucSoundSpeed;		// Sound speed, 0-250, c = 1375+ucSpeed
    unsigned char ucCheckSum;		// Check sum in this version
} CMD_STS1000;


//**************************应答包帧头***************************
typedef struct // Sonar Echo Data
{
    unsigned char ucHeader;// 0xFE
    unsigned char ucID; // 0x1 to 0xfD
    unsigned char ucStatus; // Bit1:0 - 00 8bits数据，一字节代表一数据点
    unsigned char ucTemp; // 温度，100+T，T∈(-99,152)，单位0.5℃
    unsigned char ucDataLo; // Echo Package length =
    unsigned char ucDataHi; // (((ucDataHi&0x7f)<<7)|(ucDataLo&0x7f))
    unsigned char ucAngleLo; // nAngle =
    unsigned char ucAngleHi; // (((ucAngleHi&0x7f)<<7)|(ucAngleLo&0x7f))
    unsigned char ucCPSLo; // 罗经=
    unsigned char ucCPSHi; // (((ucCPSHi&0x7f)<<7)|(ucCPSLo&0x7f))
    unsigned char ucPRSLo; // 压力=
    unsigned char ucPRSHi; // (((ucPRSHi&0x7f)<<7)|(ucPRSLo&0x7f))
    unsigned char ucPitchLo; // 纵倾=
    unsigned char ucPitchHi; // (((ucPitchHi&0x7f)<<7)|(ucPitchLo&0x7f))
    unsigned char ucRollLo; // 横摇=
    unsigned char ucRollHi; // (((ucRollHi&0x7f)<<7)|(ucRollLo&0x7f))
}ANSHEAD_STS1000;



void init_cmd(CMD_STS1000 *cmd_ptr);
void updateCheckSum(CMD_STS1000 *cmd_ptr);

#endif // DEFINE_STS1000_H
