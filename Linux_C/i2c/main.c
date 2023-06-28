#include "i2c.h"
#include "wit_c_sdk.h"
#include <stdio.h>

#define TIMEOUT	3
#define RETRY	3

static int fd;
char *i2c_dev = "/dev/i2c-1";



#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0;

static void AutoScanSensor(void);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
static int i2c_read(u8 addr, u8 reg, u8 *data, u32 len);
static int i2c_write(u8 addr, u8 reg, u8 *data, u32 len);

int main(int argc,char* argv[]){
	if(argc < 2)
	{
		printf("please input dev name\n");
		return 0;
	}
	
	float fAcc[3], fGyro[3], fAngle[3];
	int i;
	fd = i2c_open(argv[1], 3, 3);
	if( fd < 0)printf("open %s fail\n", argv[1]);
	else printf("open %s success\n", argv[1]);
	WitInit(WIT_PROTOCOL_I2C, 0x50);
	WitI2cFuncRegister(i2c_write, i2c_read);
	WitRegisterCallBack(CopeSensorData);
	WitDelayMsRegister(Delayms);
	printf("\r\n********************** wit-motion IIC example  ************************\r\n");
	AutoScanSensor();
	while (1)
	{
		WitReadReg(AX, 12);
		usleep(500000);
		if(s_cDataUpdate)
		{
			printf("\r\n");
			for(i = 0; i < 3; i++)
			{
				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			}
			if(s_cDataUpdate & ACC_UPDATE)
			{
				printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
				s_cDataUpdate &= ~ACC_UPDATE;
			}
			if(s_cDataUpdate & GYRO_UPDATE)
			{
				printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
				s_cDataUpdate &= ~GYRO_UPDATE;
			}
			if(s_cDataUpdate & ANGLE_UPDATE)
			{
				printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
				s_cDataUpdate &= ~ANGLE_UPDATE;
			}
			if(s_cDataUpdate & MAG_UPDATE)
			{
				printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
				s_cDataUpdate &= ~MAG_UPDATE;
			}
		}
	}
	
	
		
	close(fd);
	return 0;
}

static int i2c_read(u8 addr, u8 reg, u8 *data, u32 len)
{
	if(i2c_read_data(addr>>1, reg, data, len) < 0)return 0;
	return 1;
}
static int i2c_write(u8 addr, u8 reg, u8 *data, u32 len)
{
	if(i2c_write_data(addr>>1, reg, data, len) < 0)return 0;
	return 1;
}
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void Delayms(uint16_t ucMs)
{
	usleep(ucMs*1000);
}

static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 0; i < 0x7F; i++)
	{
		WitInit(WIT_PROTOCOL_I2C, i);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			usleep(5);
			if(s_cDataUpdate != 0)
			{
				printf("find %02X addr sensor\r\n", i);
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}

