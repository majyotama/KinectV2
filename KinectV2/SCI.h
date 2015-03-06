
#include "stdafx.h"
#include <Windows.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>


#define _CRT_SECURE_NO_WARNINGS
#define SCI_PORT "\\\\.\\COM7"

std::string SCI_pBufferRecieved;                        //受信した文字のバッファ
HANDLE hComPort;                                        //シリアルポートのハンドル
OVERLAPPED sendOverlapped, recieveOverlapped;			//「ReadFile()」「WriteFile()」を実行する時に「OVERLAPPED」を指定することが必須なので定義する

typedef struct {
	cv::Point3i txPos;
}SCIpara;

void SCIInit(){	//シリアル通信初期化関数
	//念のため初期化しておく(初期化しなかったからと言って特にエラーは確認していないが……)
	ZeroMemory(&sendOverlapped, sizeof(OVERLAPPED));
	ZeroMemory(&recieveOverlapped, sizeof(OVERLAPPED));

	// シリアルポートを開く
	//HANDLE hComPort;//グローバル変数として定義する
	hComPort = CreateFile(_T(SCI_PORT)
		, GENERIC_READ | GENERIC_WRITE
		, 0                             //オブジェクトを共有方法しない
		, NULL							// セキュリティ属性デフォルト//ハンドルを子プロセスへ継承することを許可するかどうか//NULLは継承できない
		, OPEN_EXISTING					//ファイルを開きます。指定したファイルが存在していない場合、この関数は失敗します。
		, FILE_FLAG_OVERLAPPED//0//FILE_FLAG_OVERLAPPEDは、非同期であることを意味する。//同期とは、この関数を実行すると処理が完了するまで待たされる。非同期とはCPUの空き時間を使って関数が処理される
		//良く分からないのだけれど、FILE_FLAG_OVERLAPPEDを設定すると送信が上手く行われていない気がする……。
		//⇒「FILE_FLAG_OVERLAPPED」を指定した場合は「ReadFile()」「WriteFile()」を実行する時に「OVERLAPPED」を指定することが必須。しないとバグる。
		, NULL
		); // シリアルポートを開く

	if (hComPort == INVALID_HANDLE_VALUE){
		OutputDebugString(_T("ポートが開きません"));
	}

	/*//必要があれば指定する//指定しなかった場合はデフォルトの容量が指定される
	//送受信バッファの容量の指定
	SetupComm(        hComPort,        // 通信デバイスのハンドル：CreateFile()で取得したハンドルを指定
	1024,                // 受信バッファのサイズ  ：受信のバッファーサイズをバイト単位で指定
	1024                // 送信バッファのサイズ  ：送信のバッファーサイズをバイト単位で指定
	);
	*/

	//バッファの初期化
	//指定した通信資源の出力バッファまたは入力バッファにあるすべての文字を破棄します。未処理の読み取り操作または書き込み操作を中止することもできます。
	PurgeComm(hComPort, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);//無くても動くけど、まあ一応初期化しておく。


	//通信の設定
	DCB dcb; // シリアルポートの構成情報が入る構造体
	GetCommState(hComPort, &dcb); // 現在の設定値を読み込み // DCB 構造体のメンバの一部だけを設定する場合でも、他のメンバも適切な値にするため、GetCommState 関数を使っていったん DCB 構造体を設定してから、該当するメンバの値を修正するようにします。(msdn) // つまり全て設定する場合は不要ではあるが、安全のために一度読み込んでおく？(今回は一部しか設定しないので必要)

	dcb.BaudRate = 19200; // 速度
	dcb.ByteSize = 8; // データ長
	dcb.Parity	 = NOPARITY; // パリティ
	dcb.StopBits = ONESTOPBIT; // ストップビット長
	dcb.fOutxCtsFlow = FALSE; // 送信時CTSフロー
	dcb.fRtsControl	 = RTS_CONTROL_DISABLE;//RTS_CONTROL_ENABLE; // RTSフロー // なし
	dcb.EvtChar		 = NULL;//つまり「\0」に同じ//ここで指定した文字列を受信した時「EV_RXFLAG」イベントが発生する。

	SetCommState(hComPort, &dcb); // 変更した設定値を書き込み

	/*//必要があれば設定する。//指定しなかった場合はデフォルトの容量が指定される
	// シリアルポートのタイムアウト状態操作
	COMMTIMEOUTS cto;
	GetCommTimeouts( hComPort, &cto ); // タイムアウトの設定状態を取得
	cto.ReadIntervalTimeout = 1000;
	cto.ReadTotalTimeoutMultiplier = 0;
	cto.ReadTotalTimeoutConstant = 1000;
	cto.WriteTotalTimeoutMultiplier = 0;
	cto.WriteTotalTimeoutConstant = 0;
	SetCommTimeouts( hComPort, &cto ); // タイムアウトの状態を設定
	*/
}

#define SETEND '|'
#define SETEND2 '/'
unsigned char crc_calc_t(void* data, int byte){	//crc計算
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

	if (crc == SETEND || crc == SETEND2)	//区切り文字との混同を防ぐ
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
	WriteFile(hComPort, &sendChar, sizeof(char) * 7, &SCI_LengthOfPutOrRecieved, &sendOverlapped); // ポートへ送信//「length()」は文字の終わりを表すNULL「\0」を文字の長さとして扱わないので「+1」しておく
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
	WriteFile(hComPort, &sendChar, sizeof(char) * 7, &SCI_LengthOfPutOrRecieved, &sendOverlapped); // ポートへ送信//「length()」は文字の終わりを表すNULL「\0」を文字の長さとして扱わないので「+1」しておく
}