#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <String.h>

#define MAX_DEPTH		8000
#define MIN_DEPTH		500
#define DEPTH_THRETH	1750
#define SERCH_MARGIN    4

typedef struct{
	cv::Mat *src;
	cv::Point3d findpos;
	cv::Point3i txpos;
	cv::Point	debugPos;
	unsigned int range;
	bool tx_flag;
	bool flag;
}SERCHPARA;

#pragma region NoiseCancel ノイズ除去

void NoiseCancel(cv::Mat &src,cv::Mat &dst){
	int opening_num = 0;
	cv::Mat inputMat(src.cols, src.rows, CV_16U);
	cv::Mat blurMat(src.cols,src.rows,CV_16U);
	cv::Mat convertMat(src.cols, src.rows, CV_16U);
	cv::Mat outputMat(src.cols, src.rows, CV_8UC1);

	src.copyTo(inputMat);
	cv::medianBlur(inputMat, blurMat, 5);
	cv::morphologyEx(blurMat, convertMat, cv::MORPH_CLOSE, cv::Mat(17, 17, CV_16U), cv::Point(-1, -1), opening_num);
	//
	convertMat.copyTo(src);
	convertMat.convertTo(outputMat, CV_8U, -255.0f / 8000.0f, 255.0f);
	outputMat.copyTo(dst);
}
#pragma endregion


//#define FX 363.640
//#define FY 363.332
#define FX 363.332	//xy反転させているから
#define FY 363.640
#define PX 264.143
#define PY 207.633
cv::Point3d CalcWorldPosition(cv::Point3i rawPos) //キネクトの3次元極座標系をマシンのシャトルレシーブ位置を原点とした直交座標系に
{
	cv::Point3d worldPos;
	double Msum, Psum;
	Msum = sqrt(rawPos.x*rawPos.x / FX / FX + rawPos.y*rawPos.y / FY / FY + 1);
	Psum = rawPos.z;
	worldPos.x = Psum / Msum * 1;
	worldPos.y = Psum / Msum*rawPos.x / FX;
	worldPos.z = -Psum / Msum*rawPos.y / FY;
	return worldPos;
}

#define G (9797.4)
#define M (6.4)
#define K (0.002266)
#define dT (0.002)
#define fvr(t,vr,vz,r,z) (-K*(vr)*sqrt((vr)*(vr)+(vz)*(vz))/M)
#define fvz(t,vr,vz,r,z) (-G-K*(vz)*sqrt((vr)*(vr)+(vz)*(vz))/M)
#define fr(t,vr,vz,r,z) (vr)
#define fz(t,vr,vz,r,z) (vz)
cv::Point3i yosoku1(cv::Point3d p1, cv::Point3d p2, double deltat, double dt, double m, double k){ //シャトル落下位置予測関数
	double theta, r, or, z, oz, vr, vz, st;
	double vzk[4], vrk[4], rk[4], zk[4];
	static UINT8 cnt = 0;
	char fn[20];
	cv::Point3i point;

	vz = (p2.z - p1.z) / deltat;
	vr = sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y)) / deltat;
	theta = atan2(p2.y - p1.y, p2.x - p1.x);
	z = oz = p2.z;
	r = or = 0;
	st = 0;
//	sprintf(fn, "yosoku%d.csv", cnt);
	//fopen_s(&fp,fn, "w");
	while (1){
		//fprintf(fp, "%f,%f,%f,%f,%f\n", st, vr, vz, r, z);

		vrk[0] = dt*fvr(st, vr, vz, r, z);
		vzk[0] = dt*fvz(st, vr, vz, r, z);
		rk[0] = dt*fr(st, vr, vz, r, z);
		zk[0] = dt*fz(st, vr, vz, r, z);
		vrk[1] = dt*fvr(st + dt / 2, vr + vrk[0] / 2, vz + vzk[0] / 2, r + rk[0] / 2, z + zk[0] / 2);
		vzk[1] = dt*fvz(st + dt / 2, vr + vrk[0] / 2, vz + vzk[0] / 2, r + rk[0] / 2, z + zk[0] / 2);
		rk[1] = dt*fr(st + dt / 2, vr + vrk[0] / 2, vz + vzk[0] / 2, r + rk[0] / 2, z + zk[0] / 2);
		zk[1] = dt*fz(st + dt / 2, vr + vrk[0] / 2, vz + vzk[0] / 2, r + rk[0] / 2, z + zk[0] / 2);
		vrk[2] = dt*fvr(st + dt / 2, vr + vrk[1] / 2, vz + vzk[1] / 2, r + rk[1] / 2, z + zk[1] / 2);
		vzk[2] = dt*fvz(st + dt / 2, vr + vrk[1] / 2, vz + vzk[1] / 2, r + rk[1] / 2, z + zk[1] / 2);
		rk[2] = dt*fr(st + dt / 2, vr + vrk[1] / 2, vz + vzk[1] / 2, r + rk[1] / 2, z + zk[1] / 2);
		zk[2] = dt*fz(st + dt / 2, vr + vrk[1] / 2, vz + vzk[1] / 2, r + rk[1] / 2, z + zk[1] / 2);
		vrk[3] = dt*fvr(st + dt, vr + vrk[2], vz + vzk[2], r + rk[2], z + zk[2]);
		vzk[3] = dt*fvz(st + dt, vr + vrk[2], vz + vzk[2], r + rk[2], z + zk[2]);
		rk[3] = dt*fr(st + dt, vr + vrk[2], vz + vzk[2], r + rk[2], z + zk[2]);
		zk[3] = dt*fz(st + dt, vr + vrk[2], vz + vzk[2], r + rk[2], z + zk[2]);
		vr += (vrk[0] + 2 * vrk[1] + 2 * vrk[2] + vrk[3]) / 6;
		vz += (vzk[0] + 2 * vzk[1] + 2 * vzk[2] + vzk[3]) / 6;
		r += (rk[0] + 2 * rk[1] + 2 * rk[2] + rk[3]) / 6;
		z += (zk[0] + 2 * zk[1] + 2 * zk[2] + zk[3]) / 6;

		st = st + dt;
		if (z <= 0){
			//fprintf(fp, "%f,%f,%f,%f,%f\n", st, vr, vz, r, z);
			r = or - oz*(or - r) / (oz - z);
			z = 0;

			point.x = (int)(r*cos(theta) + p2.x);
			point.y = (int)(r*sin(theta) + p2.y);
			point.z = 0;
			break;
		}
		oz = z;
		or = r;
	}
	cnt++;
	//fclose(fp);
	return point;
}

#pragma region //スレッド シャトル探知

	unsigned int __stdcall  threadSerchShuttle(void *p){

		SERCHPARA* para = (SERCHPARA*)p;

		cv::Mat inputMat(*(para->src));
		cv::Point3d findp = 0;
		cv::Point3f txPos = 0;
		static int Lo= 30.0;	//画像のx方向y方向の分割数

		UINT32 count = 0;
		UINT16 depth1 = 0, depth2 = 0, cc = 0, xx, yy;
		double sumX = 0, sumY = 0, sumD = 0;
		static int dx;
		static int dy;
		float threthold;

		char fn[20];
		para->tx_flag = false;

		//3次元座標関係
		static cv::Point3d ballPos[2];		//XYテーブルを原点とした座標 [0]=現在、[1]=1フレーム前
		static clock_t  frameTime[2] = { 0 };	//処理速度計算用
		clock_t  startms = clock();	//処理開始時間をミリ単位で取得

		dx = (int)((float)(inputMat.cols - SERCH_MARGIN*2) / (float)Lo);
		dy = (int)((float)(inputMat.rows - SERCH_MARGIN*2) / (float)Lo);
		threthold = ((float)dx * (float)dy)* 0.5	;
	
		//データ出力
		FILE *fp;	//デバッグ用
		sprintf(fn, "pos.csv");
		fopen_s(&fp, fn, "a");

		for (int y = SERCH_MARGIN; y < inputMat.rows - SERCH_MARGIN; y += ((int)dy / 2)){
			for (int x = SERCH_MARGIN; x < inputMat.cols - SERCH_MARGIN; x += ((int)dx / 2)){

				depth1 = reinterpret_cast<UINT16*>(inputMat.data)[(y * inputMat.cols) + x];
				//ピクセルのカウント
				if (MIN_DEPTH <depth1 && depth1 < DEPTH_THRETH){
					cc = 0;
					for (yy = y; yy < y+(int)dy; ++yy){
						for (xx = x; xx < x+(int)dx; ++xx){
							depth2 = reinterpret_cast<UINT16*>(inputMat.data)[(yy * inputMat.cols) + xx];
							cc += ((MIN_DEPTH <depth2 && depth2 < DEPTH_THRETH) ? 1 : 0);
						}
					}
					if (cc > threthold){
						sumX += (x - inputMat.cols/2);
						sumY += (y - inputMat.rows / 2);
						sumD += depth1;
						count++;
					}
				}

			}
		}

		if (count > 0){	//シャトルが見つかったら
			//シャトルと思われるピクセルの中心位置を取得
			Lo = Lo - (Lo >10 ? 3 : 0);
			findp.x = (int)(sumX / (double)count);
			findp.y = (int)(sumY / (double)count);
			findp.z = (int)(sumD / (double)count);

			ballPos[1] = ballPos[0];	//過去の位置を保持
			ballPos[0] = CalcWorldPosition(findp);	//カメラ座標をXYテーブルを原点とする座標へ変換

			frameTime[1] = frameTime[0];//過去の観測時間を保持
			frameTime[0] = startms;		//このフレームの観測時間を保存

			para->flag = true;

			if (frameTime[1] != 0 && frameTime[0] - frameTime[1] < 100){	//1フレーム目じゃなくて、フレーム飛びしていなかったら
				txPos = yosoku1(ballPos[1], ballPos[0], (double)(frameTime[0] - frameTime[1]) / 1000.0, dT, M, K);	//着弾位置を予測
				std::cout << ballPos[0] << "→" << txPos << std::endl;	//前フレーム処理終了から現在フレーム処理終了までの時間、処理開始から処理終了までの時間表示
				if (fp != NULL)fprintf(fp, "%d , %lf,%lf,%lf,%d,%d\n", frameTime[0], ballPos[0].x, ballPos[0].y, ballPos[0].z,txPos.x,txPos.y);
				para->tx_flag = true;
			}
			else{
				txPos.x = 0; 
				txPos.y = 0;
				std::cout << std::endl;	//前フレーム処理終了から現在フレーム処理終了までの時間、処理開始から処理終了までの時間表示
				if (fp != NULL)fprintf(fp, "%\n");
			}

		//	if (txPos.x != 0 && txPos.y != 0){}
				//DataSend(txPos);	//予測が正しく行えたらシリアルで位置を送る*/

//			if(fp!=NULL)fprintf(fp, "%d , %lf,%lf,%lf\n",frameTime[0], ballPos[0].x, ballPos[0].y, ballPos[0].z);
		}
		else{	//シャトル見つからなかったら各変数初期化
			Lo = 30.0;
			findp.x = 0;//inputMat.cols / 2;
			findp.y = 0;//inputMat.rows / 2;
			findp.z = 0;

			para->flag = false;
		}
		if(para->flag)std::cout << ballPos[0] <<"→"<< txPos << std::endl;	//前フレーム処理終了から現在フレーム処理終了までの時間、処理開始から処理終了までの時間表示

		para->findpos = (cv::Point3d)ballPos[0];	//画面上の座標
		para->txpos   = (cv::Point3i)txPos;			//予測座標
		para->debugPos.x = findp.x;
		para->debugPos.y = findp.y;
		para->range = dx;

		if(fp!=NULL)fclose(fp);
		return 0;
	}
#pragma endregion