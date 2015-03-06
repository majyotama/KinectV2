
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <time.h> // to calculate time needed


typedef struct{
	 cv::Mat *src;
	 unsigned short maxdepth;
	 unsigned short mindepth;
	 cv::Point		ballpos;
	 unsigned int	range;
	 std::vector<cv::Vec3f> cpoint;
}debugPara;

typedef struct{
	cv::Mat *src;
	int counter;
}filePara;

#pragma region ColorScaleBCGYR カラースケールに変換
void ColorScaleBCGYR(double in_value,uchar* out_val) // 0.0～1.0 の範囲の値をサーモグラフィみたいな色にする
{
	// 0.0                    1.0
	// 青    水    緑    黄    赤
	// 最小値以下 = 青
	// 最大値以上 = 赤
	int a = 255;    // alpha値
	int r, g, b;    // RGB値
	double  value = in_value;
	double  tmp_val = cos(4 * 3.1418 * value);
	int     col_val = (int)((-tmp_val / 2 + 0.5) * 255);
	if (value >= (4.0 / 4.0)) { r = 255;     g = 0;       b = 0; }   // 赤
	else if (value >= (3.0 / 4.0)) { r = 255;     g = col_val; b = 0; }   // 黄～赤
	else if (value >= (2.0 / 4.0)) { r = col_val; g = 255;     b = 0; }   // 緑～黄
	else if (value >= (1.0 / 4.0)) { r = 0;       g = 255;     b = col_val; }   // 水～緑
	else if (value >= (0.0 / 4.0)) { r = 0;       g = col_val; b = 255; }   // 青～水
	else { r = 0;       g = 0;       b = 255; }   // 青
	out_val[0] = b; out_val[1] = g; out_val[2] = r;
	/*ret = (a & 0x000000FF) << 24
		| (r & 0x000000FF) << 16
		| (g & 0x000000FF) << 8
		| (b & 0x000000FF);
	return ret;*/
}
#pragma endregion

#pragma region DepthMat2ColorMat デプスMatをカラーMatに変換
//srcは8ビット１チャンネル
//dstは8ビット4チャンネル

void DepthMat2ColorMat(cv::Mat &src,cv::Mat &dst,const unsigned short &maxdepth,const unsigned short &mindepth){
	
	UINT8 depth8;
	UINT16 depth, depthX = 0, depthY = 0;
	uchar bgr[3];

	dst = cv::Scalar(0, 0, 0, 0);

	for (UINT32 i = 0; i < src.total(); ++i){
		int destIndex = i * 4;
		depthX++;
		if (depthX == src.cols){
			depthX = 0;
			depthY++;
		}
		depth = reinterpret_cast<UINT16*>(src.data)[i];
		ColorScaleBCGYR((double)(depth - mindepth) / (maxdepth - mindepth), bgr);	//depthに応じてグラデーション画像生成

		if (depth <= maxdepth && depth >= mindepth){	//depthが取得範囲内か

			dst.data[destIndex + 0] = bgr[2];
			dst.data[destIndex + 1] = bgr[1];
			dst.data[destIndex + 2] = bgr[0];
		}
	}
}


#pragma endregion


#pragma region threadOutput マルチスレッドで表示
unsigned int __stdcall  threadOutput(void *p){
	
	#pragma region 変数宣言
	debugPara para = *(debugPara*)p;
	//fps計算
	clock_t  endms = 0;
	static clock_t oldms = 0;
	#pragma endregion
	
	endms = clock();

	//画像の回転

	// Green，太さ5，8近傍連結
	cv::rectangle((*para.src), para.ballpos-cv::Point(para.range/2,para.range/2), para.ballpos + cv::Point(para.range, para.range), cv::Scalar(0, 0, 0), 5, 8);

	//FPS表示
	//std::cout << "@" << ((float)1000 / (float)(endms - oldms)) << "FPS   " << debugPos << std::endl;	//前フレーム処理終了から現在フレーム処理終了までの時間、処理開始から処理終了までの時間表示
	
	std::string time = std::to_string(((float)1000 / (float)(endms - oldms))) + " ms ";
	std::string pos = std::to_string(para.ballpos.x) + " , " + std::to_string(para.ballpos.y) ;
	cv::putText((*para.src), time, cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0, 0, 0), 2);
	//cv::putText((*para.src), pos, cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0, 0, 0), 2);
	cv::imshow("Color", (*para.src)); 
	oldms = endms;

	return 0;
}
#pragma endregion


#pragma region threadFileIO 画像の保存
unsigned int __stdcall  threadFileIO(void *p){

#pragma region 変数宣言
	filePara para = *(filePara*)p;

	std::string name =  std::to_string(para.counter) + ".jpg";
	cv::imwrite(name, (*para.src));
	para.counter++;

	return 0;
}
#pragma endregion

