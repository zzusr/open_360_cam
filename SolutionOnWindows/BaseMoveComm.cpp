#include "BaseMoveComm.h"
#include "USART.h"

unsigned char Get_Crc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while (len--)
	{
		crc ^= *ptr++;
		for (i = 0; i < 8; i++)
		{
			if (crc & 0x01) crc = (crc >> 1) ^ 0x8C;
			else crc >>= 1;
		}
	}
	return crc;
}

//机器人运动控制参数：v, w, Ɵ
//v：机器人移动速度; 单位mm / s，有效值 - 2147483647~+ 2147483647。
//范围[-900, 900], 即 - 900mm / s - 900mm / s
//w：机器人旋转速度，单位(rad / s * 1000)，有效值 - 2147483647~+ 2147483647。
//范围[-1000, 1000], , 即 - 1rad / s - 1rad / s
//Ɵ：机器人移动角度，有效值0~360，有效值[0, 360)，逆时针方向, 右手法则。
//
//四种控制方式：
//当机器人作为差动移动时：v, w, Ɵ = 0; （ROS）
//当机器人原地旋转时：v = 0, w, Ɵ = 0;
//当机器人横向移动时：v, w = 0, Ɵ;
//全向移动时；v, w, Ɵ；(vx = vsinƟ, vy = vcosƟ, w)
//
//波特率115200，8个数据位，无校验位，1个停止位
//数据格式：帧头 + 标志位 + 长度 + 数据 + 校验和 + 帧尾
//帧头：两个字节，0x55  0xAA
//标志位：电脑发底盘0xA5  0x5A
//底盘发电脑0XAA  0Xaa


//机器人手动或编程运动控制指令：
//长度：一个字节，目前固定为12
//数据：包含速度，角速度和偏航角
//速度共4个字节，低字节在前，高字节在后，单位mm/s，有效值 -2147483647~+ 2147483647
//角速度4个字节，低字节在前，高字节在后，单位(rad / s * 1000)，有效值 - 2147483647~+ 2147483647		（逆时针为正，顺时针为负）
//偏航角4个字节，低字节在前，高字节在后，单位度，有效值0~360，逆时针方向。
//校验和：一个字节，前17个字节的CRC


//电脑发送底盘指令
//55 AA A5 5A 0C F4 01 00 00 58 02 00 00 00 00 00 00 06 0D 0A
//帧头：55 AA
//标志位：A5 5A
//数据位长度：0x0C
//速度为（红色字体）：0x000001F4（500），以500mm / s的速度向前移动；
//角速度（绿色字体）：0x00000258（600），为0.6rad / s的角速度旋转
//偏航角（蓝色字体）：0x00000000（0），以0°移动。
//CRC	校验和：0x06
//帧尾：0D 0A

void BaseMoveControl(unsigned char Send_Buff[16],int m_velocity, int m_rotatespeed, int m_orientation)
{
	BYTE CRC = 0;
	Send_Buff[0] = 0x55;
	Send_Buff[1] = 0xAA;
	Send_Buff[2] = 0xA5;
	Send_Buff[3] = 0x5A;
	Send_Buff[4] = 0x0C;
	Send_Buff[5] = *(BYTE*)(&m_velocity);
	Send_Buff[6] = *((BYTE*)(&m_velocity) + 1);

	Send_Buff[7] = *(BYTE*)(&m_rotatespeed);
	Send_Buff[8] = *((BYTE*)(&m_rotatespeed) + 1);

	Send_Buff[9] = *(BYTE*)(&m_orientation);
	Send_Buff[10] = *((BYTE*)(&m_orientation) + 1);

	CRC = Get_Crc8(Send_Buff, 9);
	Send_Buff[11] = CRC;	
}

//BaseMoveControl(Send_Buff, 0, 262, 0);//15度/s
//com1.WriteChar(Send_Buff, 16);