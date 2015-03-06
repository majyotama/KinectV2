﻿#include <iostream>
#include <sstream>

#include <Kinect.h>
#include <opencv2\opencv.hpp>

// Visual Studio Professional以上を使う場合はCComPtrの利用を検討してください。
//#include "ComPtr.h"
//#include <atlbase.h>

// 次のように使います
// ERROR_CHECK( ::GetDefaultKinectSensor( &kinect ) );
// 書籍での解説のためにマクロにしています。実際には展開した形で使うことを検討してください。
#define ERROR_CHECK( ret )  \
    if ( (ret) != S_OK ) {    \
        std::stringstream ss;	\
        ss << "failed " #ret " " << std::hex << ret << std::endl;			\
        throw std::runtime_error( ss.str().c_str() );			\
    }

class KinectApp
{
private:

    IKinectSensor* kinect = nullptr;
	int colorWidth=1920;
	int colorHeight=1080;
	int depthWidth=520;
	int depthHeight=424;
    IColorFrameReader* colorFrameReader = nullptr;
	cv::Mat colorBuffer;

    IDepthFrameReader* depthFrameReader = nullptr;
	cv::Mat depthBuffer;
	cv::Mat depthMat;

    const char* ColorWindowName = "Color Image";
    const char* DepthWindowName = "Depth Image";

    
    const int ColorBytesPerPixel = 4;

    int minDepth;
    int maxDepth;

    UINT16 minDepthReliableDistance;
    UINT16 maxDepthReliableDistance;

   

    int depthPointX;
    int depthPointY;

	UINT colorSize;
	UINT depthSize;

public:

    // 初期化
    void initialize()
    {
        // デフォルトのKinectを取得する
        ERROR_CHECK( ::GetDefaultKinectSensor( &kinect ) );

        // Kinectを開く
        ERROR_CHECK( kinect->Open() );

        BOOLEAN isOpen = false;
        ERROR_CHECK( kinect->get_IsOpen( &isOpen ) );
        if ( !isOpen ){
            throw std::runtime_error( "Kinectが開けません" );
        }

        // カラーリーダーを取得する
        IColorFrameSource* colorFrameSource;
        ERROR_CHECK( kinect->get_ColorFrameSource( &colorFrameSource ) );
        ERROR_CHECK( colorFrameSource->OpenReader( &colorFrameReader ) );

        // カラー画像のサイズを取得する
        IFrameDescription* colorFrameDescription;
        ERROR_CHECK( colorFrameSource->get_FrameDescription( &colorFrameDescription ) );
        ERROR_CHECK( colorFrameDescription->get_Width( &colorWidth ) );
        ERROR_CHECK( colorFrameDescription->get_Height( &colorHeight ) );

        // Depthリーダーを取得する
        IDepthFrameSource* depthFrameSource;
        ERROR_CHECK( kinect->get_DepthFrameSource( &depthFrameSource ) );
        ERROR_CHECK( depthFrameSource->OpenReader( &depthFrameReader ) );

        // Depth画像のサイズを取得する
        IFrameDescription* depthFrameDescription;
        ERROR_CHECK( depthFrameSource->get_FrameDescription( &depthFrameDescription ) );
        ERROR_CHECK( depthFrameDescription->get_Width( &depthWidth ) );
        ERROR_CHECK( depthFrameDescription->get_Height( &depthHeight ) );

        depthPointX = depthWidth / 2;
        depthPointY = depthHeight / 2;

        // Depthの最大値、最小値を取得する
        ERROR_CHECK( depthFrameSource->get_DepthMinReliableDistance( &minDepthReliableDistance ) );
        ERROR_CHECK( depthFrameSource->get_DepthMaxReliableDistance( &maxDepthReliableDistance ) );

        std::cout << "Depthデータの幅   : " << depthWidth << std::endl;
        std::cout << "Depthデータの高さ : " << depthHeight << std::endl;

        std::cout << "Depth最小値       : " << minDepthReliableDistance << std::endl;
        std::cout << "Depth最大値       : " << maxDepthReliableDistance << std::endl;

        // バッファーを作成する
		colorBuffer = cv::Mat(colorHeight, colorWidth, CV_8UC4);
		depthBuffer = cv::Mat(depthHeight, depthWidth, CV_16UC1);
		depthMat = cv::Mat(depthHeight, depthWidth, CV_8UC1);
       // colorBuffer.resize( colorWidth * colorHeight * ColorBytesPerPixel );
       // depthBuffer.resize( depthWidth * depthHeight );
		colorSize = colorWidth * colorHeight * ColorBytesPerPixel;
		depthSize = depthWidth * depthHeight;
        // 画面を作成
        cv::namedWindow( ColorWindowName );
		cv::namedWindow(DepthWindowName);

        // 表示範囲距離のトラックバーを作成
        minDepth = minDepthReliableDistance;
        maxDepth = maxDepthReliableDistance;
        cv::createTrackbar( "Min Depth", ColorWindowName, &minDepth, maxDepthReliableDistance );
        cv::createTrackbar( "Max Depth", ColorWindowName, &maxDepth, maxDepthReliableDistance );
    }

    void run()
    {
        while ( 1 ) {
            if(update())
				draw();

            auto key = cv::waitKey( 10 );
            if ( key == 'q' ){
                break;
            }
        }
    }

private:

    // データの更新処理
    bool update()
    {
		if (updateColor() && updateDepth())
			return true;
		return false;
    }

    bool updateColor()
    {
        // フレームを取得する
        IColorFrame* colorFrame;
        auto ret = colorFrameReader->AcquireLatestFrame( &colorFrame );
        if ( ret == S_OK ){
            // BGRAの形式でデータを取得する
            ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray(
                colorSize, reinterpret_cast<BYTE*>( colorBuffer.data ), ColorImageFormat_Bgra ) );

            // 自動解放を使わない場合には、フレームを解放する
             colorFrame->Release();
			 return true;
        }
		return false;
    }

    bool updateDepth()
    {
        // フレームを取得する
        IDepthFrame* depthFrame;
        auto ret = depthFrameReader->AcquireLatestFrame( &depthFrame );
        if ( ret == S_OK ){
            // データを取得する
			ERROR_CHECK(depthFrame->CopyFrameDataToArray(depthSize, reinterpret_cast<UINT16*>(depthBuffer.data)));
			//ERROR_CHECK(depthFrame->AccessUnderlyingBuffer(&depthSize, reinterpret_cast<UINT16**>(&depthBuffer.data)));
			depthBuffer.convertTo(depthMat, CV_8U, -255.0f / 8000.0f, 255.0f);
            // 自動解放を使わない場合には、フレームを解放する
             depthFrame->Release();
			 return true;
        }
		return false;
    }

    void draw()
    {
        drawColorMap();
        //drawDepthMap();
		//drawTest();
    }

	void drawTest()
	{
		cv::imshow(ColorWindowName, colorBuffer);
		cv::imshow(DepthWindowName, depthMat);
	}

    void drawColorMap()
    {
        ICoordinateMapper* mapper;
        ERROR_CHECK( kinect->get_CoordinateMapper( &mapper ) );

		std::vector<ColorSpacePoint> colorSpacePoints(depthSize);
		ERROR_CHECK(mapper->MapDepthFrameToColorSpace(depthWidth*depthHeight, reinterpret_cast<UINT16*>(depthBuffer.data),
			colorSpacePoints.size(), &colorSpacePoints[0]));

        // カラーデータを表示する
        cv::Mat colorImage( depthHeight, depthWidth, CV_8UC4 );

        for ( int i = 0; i < colorImage.total(); ++i ){
            int x = (int)colorSpacePoints[i].X;
            int y = (int)colorSpacePoints[i].Y;

            int srcIndex = ((y * colorWidth) + x) * ColorBytesPerPixel;
            int destIndex = i * ColorBytesPerPixel;

            if ( isValidColorRange( x, y ) && isValidDepthRange( i ) ){
                colorImage.data[destIndex + 0] = colorBuffer.data[srcIndex + 0];
                colorImage.data[destIndex + 1] = colorBuffer.data[srcIndex + 1];
                colorImage.data[destIndex + 2] = colorBuffer.data[srcIndex + 2];
            }
            else {
                colorImage.data[destIndex + 0] = 255;
                colorImage.data[destIndex + 1] = 255;
                colorImage.data[destIndex + 2] = 255;
            }
        }

        cv::imshow( ColorWindowName, colorImage );
    }

    void drawDepthMap()
    {
        ICoordinateMapper* mapper;
        ERROR_CHECK( kinect->get_CoordinateMapper( &mapper ) );

       std::vector<DepthSpacePoint> depthSpacePoints( colorWidth * colorHeight );
        ERROR_CHECK( mapper->MapColorFrameToDepthSpace( depthWidth*depthHeight, reinterpret_cast<UINT16*>( depthBuffer.data ),
           depthSpacePoints.size(), &depthSpacePoints[0] ) );

        // カラーデータを表示する
        cv::Mat colorImage( colorHeight, colorWidth, CV_8UC4 );

        for ( int i = 0; i < colorImage.total(); ++i ){
            int x = (int)depthSpacePoints[i].X;
            int y = (int)depthSpacePoints[i].Y;

            int depthIndex = (y * depthWidth) + x;
            int colorIndex = i * ColorBytesPerPixel;

            if ( isValidColorRange( x, y ) && isValidDepthRange( depthIndex ) ){
                colorImage.data[colorIndex + 0] = colorBuffer.data[colorIndex + 0];
                colorImage.data[colorIndex + 1] = colorBuffer.data[colorIndex + 1];
                colorImage.data[colorIndex + 2] = colorBuffer.data[colorIndex + 2];
            }
            else {
                colorImage.data[colorIndex + 0] = 255;
                colorImage.data[colorIndex + 1] = 255;
                colorImage.data[colorIndex + 2] = 255;
            }
        }

        cv::imshow( ColorWindowName, colorImage );
    }

    bool isValidColorRange( int x, int y )
    {
        return ((0 <= x) && (x < colorWidth)) && ((0 <= y) && (y < colorHeight));
    }

    bool isValidDepthRange( int index )
    {
        return (minDepth <= depthBuffer.data[index]) && (depthBuffer.data[index] <= maxDepth);
    }
};

void main()
{
    try {
        KinectApp app;
        app.initialize();
        app.run();
    }
    catch ( std::exception& ex ){
        std::cout << ex.what() << std::endl;
		cv::waitKey(5000);
    }
}
