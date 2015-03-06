
#define _CRT_SECURE_NO_WARNINGS

#include "stdafx.h"
#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>

#include <process.h>
#include <stdio.h>

#include "OutputDebug.h"
#include "Shuttle.h"
#include "SCI.h"


int depthWidth = 0;	//Depthカメラ横ピクセル数
int depthHeight = 0;	//Depthカメラ縦ピクセル数
                                   //受信スレッドのID

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}



//メイン関数
int _tmain(int argc, _TCHAR* argv[])
{
	#define SAVE
	SCIInit();

#pragma region 変数宣言

	#pragma region Reader 
	cv::setUseOptimized(true);

//	SCIInit();	//シリアル通信初期化

	// Sensor
	IKinectSensor* pSensor;

	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult)){
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	hResult = pSensor->Open();
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	// Source
	IDepthFrameSource* pDepthSource;
	hResult = pSensor->get_DepthFrameSource(&pDepthSource);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	IDepthFrameReader* pDepthReader;
	hResult = pDepthSource->OpenReader(&pDepthReader);
	if (FAILED(hResult)){
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	IFrameDescription* pDepthDescription;
	hResult = pDepthSource->get_FrameDescription(&pDepthDescription);
	if (FAILED(hResult)){
		std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	#pragma endregion

	unsigned int	depthBufferSize;

	pDepthDescription->get_Width(&depthWidth); //depth横ピクセル数取得
	pDepthDescription->get_Height(&depthHeight); //depth縦ピクセル数取得
	depthBufferSize = depthWidth * depthHeight * sizeof(unsigned short);

	cv::Mat depthBufferMat(depthHeight, depthWidth, CV_16UC1);	//depth 16bit生値
	cv::Mat beforeMat(depthWidth+100, depthHeight+100, CV_16UC1);	//depth 回転画像
	cv::Mat rotateBufMat(depthWidth, depthHeight, CV_16UC1);	//depth 回転画像

#pragma endregion

	//cv::namedWindow("Color");


	// 中心：画像中心
	cv::Point2f center(depthWidth/2, depthHeight/2);
	// 以上の条件から2次元の回転行列を計算
	const cv::Mat affine_matrix = cv::getRotationMatrix2D(center, -90.0, 1.0);

	SERCHPARA serchPara;
	serchPara.src = new cv::Mat(depthWidth, depthHeight, CV_16UC1);
	serchPara.flag = false;

	debugPara para;
	para.src = new cv::Mat(depthWidth, depthHeight, CV_8U);

	filePara filePara;
	filePara.src = new cv::Mat(depthWidth, depthHeight, CV_8U);
	filePara.counter = 0;

	SCIpara scipara;

	while (1){

		//Depthデータの処理
		IDepthFrame* pDepthFrame = nullptr;
		hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);

		if (SUCCEEDED(hResult)){
			#pragma region Depth2Mat
			hResult = pDepthFrame->AccessUnderlyingBuffer(&depthBufferSize, reinterpret_cast<UINT16**>(&depthBufferMat.data));
			/* アフィン変換で回転 */
			//depthBufferMat.copyTo();
			cv::warpAffine(depthBufferMat, rotateBufMat, affine_matrix, rotateBufMat.size());
			#pragma endregion



			cv::Mat cleanMat(rotateBufMat.cols, rotateBufMat.rows, CV_8U);
			cv::Mat testMat(depthBufferMat.cols, depthBufferMat.rows, CV_8U);
		//	NoiseCancel(rotateBufMat, cleanMat);
			depthBufferMat.convertTo(testMat, CV_8U, -255.0f / 8000.0f, 255.0f);
			cv::imshow("Color", testMat);

			/*
			
			#pragma region SerchThread
			rotateBufMat.copyTo(*(serchPara.src));
			HANDLE hHandle1 = (HANDLE)_beginthreadex(NULL, 0, threadSerchShuttle, &serchPara, 0, NULL);
			WaitForSingleObject(hHandle1, INFINITE);
			#pragma endregion
		
			#pragma region DebugThread
			cleanMat.copyTo(*(para.src));
			para.ballpos = serchPara.debugPos +cv::Point(depthWidth / 2, depthHeight / 2);
			para.range = serchPara.range;
			HANDLE hHandle = (HANDLE)_beginthreadex(NULL, 0, threadOutput, &para, 0, NULL);

			#pragma endregion
			
			#pragma region File IO Thread
			#ifdef SAVE
			if (serchPara.flag){
				cleanMat.copyTo(*(filePara.src));
				filePara.counter++;
				HANDLE hHandle2 = (HANDLE)_beginthreadex(NULL, 0, threadFileIO, &filePara, 0, NULL);
				CloseHandle(hHandle2);
			}
			#endif
			#pragma endregion

			#pragma region SCIThread
			if (serchPara.tx_flag){
				scipara.txPos = serchPara.txpos;
				HANDLE hHandle3 = (HANDLE)_beginthreadex(NULL, 0, DataSendThread, &scipara, 0, NULL);
				CloseHandle(hHandle3);
			}
			#pragma endregion

//			if(serchPara.tx_flag)DataSend(serchPara.txpos);

			CloseHandle(hHandle);
			CloseHandle(hHandle1);
			*/
			
		}
		if (cv::waitKey(1) == VK_ESCAPE){
			break;
		}
		SafeRelease(pDepthFrame);
	}

	//後片付け
	CloseSCIHandle();
	SafeRelease(pDepthSource);
	SafeRelease(pDepthReader);
	SafeRelease(pDepthDescription);
	if (pSensor){
		pSensor->Close();
	}
	SafeRelease(pSensor);
	cv::destroyAllWindows();
	return 0;
}