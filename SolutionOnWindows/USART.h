
#pragma once


#include <windows.h>
class Comm
{
public:

	HANDLE hComm;
	COMSTAT comstat;
	OVERLAPPED m_ov;
	int rev_byte_count;
	Comm();
	~Comm() {
		CloseHandle(hComm);
	}
	bool openport(LPCTSTR portname, int baudrate);//�򿪴���

	bool setupdcb(int rate_arg);//����DCB

	bool setuptimeout(DWORD ReadInterval, DWORD ReadTotalMultiplier, DWORD ReadTotalconstant, DWORD WriteTotalMultiplier, DWORD WriteTotalconstant);

	bool ReceiveChar(BYTE *m_RevBuff, int m_Count);

	bool WriteChar(BYTE* m_szWriteBuffer, DWORD m_nToSend);


};