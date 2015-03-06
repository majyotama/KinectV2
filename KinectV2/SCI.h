
#include "stdafx.h"
#include <Windows.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>


#define _CRT_SECURE_NO_WARNINGS
#define SCI_PORT "\\\\.\\COM7"

std::string SCI_pBufferRecieved;                        //��M���������̃o�b�t�@
HANDLE hComPort;                                        //�V���A���|�[�g�̃n���h��
OVERLAPPED sendOverlapped, recieveOverlapped;			//�uReadFile()�v�uWriteFile()�v�����s���鎞�ɁuOVERLAPPED�v���w�肷�邱�Ƃ��K�{�Ȃ̂Œ�`����

typedef struct {
	cv::Point3i txPos;
}SCIpara;

void SCIInit(){	//�V���A���ʐM�������֐�
	//�O�̂��ߏ��������Ă���(���������Ȃ���������ƌ����ē��ɃG���[�͊m�F���Ă��Ȃ����c�c)
	ZeroMemory(&sendOverlapped, sizeof(OVERLAPPED));
	ZeroMemory(&recieveOverlapped, sizeof(OVERLAPPED));

	// �V���A���|�[�g���J��
	//HANDLE hComPort;//�O���[�o���ϐ��Ƃ��Ē�`����
	hComPort = CreateFile(_T(SCI_PORT)
		, GENERIC_READ | GENERIC_WRITE
		, 0                             //�I�u�W�F�N�g�����L���@���Ȃ�
		, NULL							// �Z�L�����e�B�����f�t�H���g//�n���h�����q�v���Z�X�֌p�����邱�Ƃ������邩�ǂ���//NULL�͌p���ł��Ȃ�
		, OPEN_EXISTING					//�t�@�C�����J���܂��B�w�肵���t�@�C�������݂��Ă��Ȃ��ꍇ�A���̊֐��͎��s���܂��B
		, FILE_FLAG_OVERLAPPED//0//FILE_FLAG_OVERLAPPED�́A�񓯊��ł��邱�Ƃ��Ӗ�����B//�����Ƃ́A���̊֐������s����Ə�������������܂ő҂������B�񓯊��Ƃ�CPU�̋󂫎��Ԃ��g���Ċ֐������������
		//�ǂ�������Ȃ��̂�����ǁAFILE_FLAG_OVERLAPPED��ݒ肷��Ƒ��M����肭�s���Ă��Ȃ��C������c�c�B
		//�ˁuFILE_FLAG_OVERLAPPED�v���w�肵���ꍇ�́uReadFile()�v�uWriteFile()�v�����s���鎞�ɁuOVERLAPPED�v���w�肷�邱�Ƃ��K�{�B���Ȃ��ƃo�O��B
		, NULL
		); // �V���A���|�[�g���J��

	if (hComPort == INVALID_HANDLE_VALUE){
		OutputDebugString(_T("�|�[�g���J���܂���"));
	}

	/*//�K�v������Ύw�肷��//�w�肵�Ȃ������ꍇ�̓f�t�H���g�̗e�ʂ��w�肳���
	//����M�o�b�t�@�̗e�ʂ̎w��
	SetupComm(        hComPort,        // �ʐM�f�o�C�X�̃n���h���FCreateFile()�Ŏ擾�����n���h�����w��
	1024,                // ��M�o�b�t�@�̃T�C�Y  �F��M�̃o�b�t�@�[�T�C�Y���o�C�g�P�ʂŎw��
	1024                // ���M�o�b�t�@�̃T�C�Y  �F���M�̃o�b�t�@�[�T�C�Y���o�C�g�P�ʂŎw��
	);
	*/

	//�o�b�t�@�̏�����
	//�w�肵���ʐM�����̏o�̓o�b�t�@�܂��͓��̓o�b�t�@�ɂ��邷�ׂĂ̕�����j�����܂��B�������̓ǂݎ�葀��܂��͏������ݑ���𒆎~���邱�Ƃ��ł��܂��B
	PurgeComm(hComPort, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);//�����Ă��������ǁA�܂��ꉞ���������Ă����B


	//�ʐM�̐ݒ�
	DCB dcb; // �V���A���|�[�g�̍\����񂪓���\����
	GetCommState(hComPort, &dcb); // ���݂̐ݒ�l��ǂݍ��� // DCB �\���̂̃����o�̈ꕔ������ݒ肷��ꍇ�ł��A���̃����o���K�؂Ȓl�ɂ��邽�߁AGetCommState �֐����g���Ă������� DCB �\���̂�ݒ肵�Ă���A�Y�����郁���o�̒l���C������悤�ɂ��܂��B(msdn) // �܂�S�Đݒ肷��ꍇ�͕s�v�ł͂��邪�A���S�̂��߂Ɉ�x�ǂݍ���ł����H(����͈ꕔ�����ݒ肵�Ȃ��̂ŕK�v)

	dcb.BaudRate = 19200; // ���x
	dcb.ByteSize = 8; // �f�[�^��
	dcb.Parity	 = NOPARITY; // �p���e�B
	dcb.StopBits = ONESTOPBIT; // �X�g�b�v�r�b�g��
	dcb.fOutxCtsFlow = FALSE; // ���M��CTS�t���[
	dcb.fRtsControl	 = RTS_CONTROL_DISABLE;//RTS_CONTROL_ENABLE; // RTS�t���[ // �Ȃ�
	dcb.EvtChar		 = NULL;//�܂�u\0�v�ɓ���//�����Ŏw�肵�����������M�������uEV_RXFLAG�v�C�x���g����������B

	SetCommState(hComPort, &dcb); // �ύX�����ݒ�l����������

	/*//�K�v������ΐݒ肷��B//�w�肵�Ȃ������ꍇ�̓f�t�H���g�̗e�ʂ��w�肳���
	// �V���A���|�[�g�̃^�C���A�E�g��ԑ���
	COMMTIMEOUTS cto;
	GetCommTimeouts( hComPort, &cto ); // �^�C���A�E�g�̐ݒ��Ԃ��擾
	cto.ReadIntervalTimeout = 1000;
	cto.ReadTotalTimeoutMultiplier = 0;
	cto.ReadTotalTimeoutConstant = 1000;
	cto.WriteTotalTimeoutMultiplier = 0;
	cto.WriteTotalTimeoutConstant = 0;
	SetCommTimeouts( hComPort, &cto ); // �^�C���A�E�g�̏�Ԃ�ݒ�
	*/
}

#define SETEND '|'
#define SETEND2 '/'
unsigned char crc_calc_t(void* data, int byte){	//crc�v�Z
	static unsigned char *data2, crc, temp, inv;
	int i, j;

	data2 = (unsigned char*)malloc(byte);
	for (i = 0; i < byte; i++){
		data2[i] = ((unsigned char*)data)[i];
	}
	crc = 0;
	for (j = 0; j < byte; j++){
		for (i = 0; i < 8; i++){
			inv = (data2[j] >> (7 - i)) ^ (crc >> 7);
			inv = inv & 0x01;
			inv += inv * 6;
			crc = crc << 1;
			temp = crc;
			temp = temp ^ inv;
			temp = temp & 0x06;
			crc = crc & 0xf8;
			crc += temp;
			temp = inv & 0x01;
			crc += temp;
			//crc = ((crc << 1) & 0xf8) + ((crc << 1) ^ (inv & 0x06)) + (inv & 0x01);
		}
	}
	free(data2);

	if (crc == SETEND || crc == SETEND2)	//��؂蕶���Ƃ̍�����h��
		crc++;
	return crc;
}

unsigned int __stdcall   DataSendThread(void* sci){
	SCIpara* scipara = (SCIpara*)sci;
	UINT16 x = (UINT16)scipara->txPos.x, y = (UINT16)scipara->txPos.y;
	unsigned char sendChar[10];
	sendChar[0] = x >> 8;
	sendChar[1] = (unsigned char)x;
	sendChar[2] = y >> 8;
	sendChar[3] = (unsigned char)y;
	sendChar[4] = crc_calc_t((unsigned char*)sendChar, 4);
	sendChar[5] = '|';
	sendChar[6] = '|';
	DWORD SCI_LengthOfPutOrRecieved;
	WriteFile(hComPort, &sendChar, sizeof(char) * 7, &SCI_LengthOfPutOrRecieved, &sendOverlapped); // �|�[�g�֑��M//�ulength()�v�͕����̏I����\��NULL�u\0�v�𕶎��̒����Ƃ��Ĉ���Ȃ��̂Łu+1�v���Ă���
	return 0;
}

void CloseSCIHandle(void){
	CloseHandle(hComPort);
}

void   DataSend(cv::Point3i pos){
	UINT16 x = (UINT16)pos.x, y = (UINT16)&pos.y;
	unsigned char sendChar[10];
	sendChar[0] = x >> 8;
	sendChar[1] = (unsigned char)x;
	sendChar[2] = y >> 8;
	sendChar[3] = (unsigned char)y;
	sendChar[4] = crc_calc_t((unsigned char*)sendChar, 4);
	sendChar[5] = '|';
	sendChar[6] = '|';
	DWORD SCI_LengthOfPutOrRecieved;
	WriteFile(hComPort, &sendChar, sizeof(char) * 7, &SCI_LengthOfPutOrRecieved, &sendOverlapped); // �|�[�g�֑��M//�ulength()�v�͕����̏I����\��NULL�u\0�v�𕶎��̒����Ƃ��Ĉ���Ȃ��̂Łu+1�v���Ă���
}