#include <iostream>
#include "USART.h"
using namespace std;

Comm::Comm()
{
	hComm = NULL;
	rev_byte_count = 0;
}
bool Comm::openport(LPCTSTR portname, int baudrate)//�򿪴���
{
	hComm = CreateFile(portname, //���ں�
		GENERIC_READ | GENERIC_WRITE, //�����д
		0, //ͨѶ�豸�����Զ�ռ��ʽ��
		0, //�ް�ȫ����
		OPEN_EXISTING, //ͨѶ�豸�Ѵ���
		FILE_FLAG_OVERLAPPED, //�첽I/O
		0); //ͨѶ�豸������ģ���
	if (hComm == INVALID_HANDLE_VALUE)
	{
		CloseHandle(hComm);
		cout << "com open fail" << endl;
		return FALSE;
	}
	else
	{
		if (setupdcb(baudrate) &&
			setuptimeout(0, 0, 0, 0, 0) &&
			SetupComm(hComm, 1024, 1024))
		{
			cout << "com open sucess" << endl;
			return true;
		}

	}

}

bool Comm::setupdcb(int rate_arg)//����DCB
{
	DCB dcb;
	int rate = rate_arg;
	memset(&dcb, 0, sizeof(dcb));
	if (!GetCommState(hComm, &dcb))//��ȡ��ǰDCB����
		return FALSE;
	// set DCB to configure the serial port
	dcb.DCBlength = sizeof(dcb);
	/* ---------- Serial Port Config ------- */

	dcb.BaudRate = rate;      //������
	dcb.Parity = NOPARITY;    //����żУ��
	dcb.fParity = 0;          //�Ƿ������żУ��  ������
	dcb.StopBits = ONESTOPBIT;//1λֹͣλ
	dcb.ByteSize = 8;         //8λ������
	dcb.fOutxCtsFlow = 0;
	dcb.fOutxDsrFlow = 0;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fDsrSensitivity = 0;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fOutX = 0;
	dcb.fInX = 0;
	/* ----------------- misc parameters ----- */
	dcb.fErrorChar = 0;
	dcb.fBinary = 1;
	dcb.fNull = 0;
	dcb.fAbortOnError = 0;
	dcb.wReserved = 0;
	dcb.XonLim = 2;
	dcb.XoffLim = 4;
	dcb.XonChar = 0x13;
	dcb.XoffChar = 0x19;
	dcb.EvtChar = 0;
	// set DCB
	if (!SetCommState(hComm, &dcb))
	{
		CloseHandle(hComm);
		return false;
	}
	else
		return true;
}

bool Comm::setuptimeout(DWORD ReadInterval, DWORD ReadTotalMultiplier, DWORD ReadTotalconstant, DWORD WriteTotalMultiplier, DWORD WriteTotalconstant)
{
	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = ReadInterval;
	timeouts.ReadTotalTimeoutConstant = ReadTotalconstant;
	timeouts.ReadTotalTimeoutMultiplier = ReadTotalMultiplier;
	timeouts.WriteTotalTimeoutConstant = WriteTotalconstant;
	timeouts.WriteTotalTimeoutMultiplier = WriteTotalMultiplier;
	if (!SetCommTimeouts(hComm, &timeouts))
	{
		CloseHandle(hComm);
		return false;
	}
	else
		return true;
}


bool Comm::ReceiveChar(BYTE *m_RevBuff, int m_Count)
{
	BOOL bRead = TRUE;
	BOOL bResult = TRUE;
	DWORD BytesRead = 0;
	DWORD dwError = 0;
	BYTE RXBuff = 0;
	unsigned int Count = 0;
	for (;;)
	{
		Sleep(10);//��ʱ��Ŀ����Ϊ�˽���CPU��ʹ����
		bResult = ClearCommError(hComm, &dwError, &comstat);
		if (comstat.cbInQue == 0)//���뻺�����ֽ���Ϊ0
					continue;

			if (bRead)
			{
				bResult = ReadFile(hComm,  // Handle to COMM port
					&RXBuff,               // RX Buffer Pointer
					1,                     // Read one byte
					&BytesRead,            // Stores number of bytes readʵ�ʶ������ֽڸ���
					&m_ov);                // pointer to the m_ov structure

				
				m_RevBuff[Count] = RXBuff;
				Count++;
			
				cout << "0x" << hex << (int)RXBuff << endl;
				printf(" Count:%d\n", Count);
				
				if (Count == m_Count)
				{
					cout << "over" << endl;
					break;
				}

				if (!bResult)
				{
					switch (dwError = GetLastError())
					{
					case ERROR_IO_PENDING:
					{
						bRead = FALSE;
						break;
					}
					default:
					{
						break;
					}
					}
				}
				else
				{
					bRead = TRUE;

				}
			} // close if (bRead)
		if (!bRead)
		{
			bRead = TRUE;
			bResult = GetOverlappedResult(hComm,  // Handle to COMM port
				&m_ov,  // Overlapped structure
				&BytesRead,     // Stores number of bytes read
				TRUE);          // Wait flag
		}
	}
	return true;
}




bool Comm::WriteChar(BYTE* m_szWriteBuffer, DWORD m_nToSend)
{
	BOOL bWrite = TRUE;
	BOOL bResult = TRUE;
	DWORD BytesSent = 0;
	if (bWrite)
	{
		m_ov.Offset = 0;
		m_ov.OffsetHigh = 0;
		// Clear buffer
		bResult = WriteFile(hComm, // Handle to COMM Port
			m_szWriteBuffer, // Pointer to message buffer in calling finction
			m_nToSend,      // Length of message to send
			&BytesSent,     // Where to store the number of bytes sent
			&m_ov);        // Overlapped structure
		if (!bResult)
		{
			DWORD dwError = GetLastError();
			switch (dwError)
			{
			case ERROR_IO_PENDING:
			{
				// continue to GetOverlappedResults()
				BytesSent = 0;
				bWrite = FALSE;
				break;
			}
			default:
			{
				// all other error codes
				break;
			}
			}
		}
	} // end if(bWrite)
	if (!bWrite)
	{
		bWrite = TRUE;
		bResult = GetOverlappedResult(hComm,   // Handle to COMM port
			&m_ov,     // Overlapped structure
			&BytesSent,    // Stores number of bytes sent
			TRUE);         // Wait flag

						   // deal with the error code
		if (!bResult)
		{
			printf("GetOverlappedResults() in WriteFile()");
		}
	} // end if (!bWrite)

	  // Verify that the data size send equals what we tried to send
	if (BytesSent != m_nToSend)
	{
		printf("WARNING: WriteFile() error.. Bytes Sent: %d; Message Length: %d\n", BytesSent, strlen((char*)m_szWriteBuffer));
	}
	return true;
}
