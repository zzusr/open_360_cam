#pragma once

unsigned char Get_Crc8(unsigned char *ptr, unsigned short len);
void BaseMoveControl(unsigned char Send_Buff[16], int m_velocity, int m_rotatespeed, int m_orientation);