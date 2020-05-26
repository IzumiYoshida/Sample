#define _CRT_SECURE_NO_WARNINGS
#include "Display.h"
#include <stdlib.h>
#include "glut.h"
#include "Robot.h"
#include "Kalmanfilter.h"
#include "Inline.h"
#include "Main_Header.h"
#include "GLFunction.h"
#include "File.h"
#include "MovingObjectDetection.h"
#include "MovingObjectTracking.h"
#include "GlobalVariable.h"
#include "Joint.h"
#include "ObservedValueEpresentativePoint.h"
#include "IncrementalAlgorithm.h"
#include "Constant.h"
#include <pcl/registration/ndt.h>

int g_loopcount = 0;					/* ループの実行回数，GLの関数間での共有が分からないため */
int timestamp = 0;						/* GNSS/INS複合装置のタイムスタンプを無理やり合わせる */
int mlspointsum = 0;					/* 読み込んだ観測点数 */
int movingobjects_num = 0;      /* 検出によって得られた移動物体の数 */
int trackingobjects_total = 0;								/* 移動物体追跡数 */
double pre_robotx, pre_roboty, pre_robotz, pre_robotroll, pre_robotpitch, pre_robotyaw,pre_imutime;
double pre_imux, pre_imuy, pre_imuz, pre_imuroll, pre_imupitch, pre_imuyaw;
int imudataflag;							/* ImuDataRead()のデータのフラグ．0：通常の処理，-1：データ抜け，1：データ終了 */

Imudata imudata;
ObservedValueEpresentativePoint observed_value_epresentative_point[UPPERLIMIT_TRACK];			/* 観測値代表点 */
extern int jointobnum[UPPERLIMIT_MOVING];																				/* 統合された際の観測値の数 */
extern double jointmove[UPPERLIMIT_MOVING][UPPERLIMIT_CELL][X_AND_Y_AND_Z + 1];			/* 統合された移動物体観測値(地上固定) */
const int STOP_DRAWING = 5000;									/* 描画を停止する時間 */
pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

/*/////////////////////////////////////////////////////////////////////
OpenGLの中心となる処理関数
引数
なし
返り値
なし
/////////////////////////////////////////////////////////////////////*/
void Display(void) {

	/* 描画の時間調整のためのスリープ */
	AdjustTime();

	/* ループカウンタ表示 */
	std::cout << "---------------ループ回数 " << g_loopcount << "---------------" << std::endl;

	/* microEV用データ読み込み関数 */
	imudataflag = imudata.ImuDataRead();

	/* データ抜けのとき */
	if (imudataflag == -1) {
		std::cout << g_loopcount << "回目でデータ抜け発生!!" << std::endl;
	}

	/* データが終了していないとき */
	if (imudataflag != 1) {
		/* 観測点の座標変換～追跡処理 */
		StartTracking();
		/* 描画開始 */
		if (g_loopcount == 0) {
			ReturnLook();
			Drawing(g_robot.GetRobotX(), g_robot.GetRobotY(), g_robot.GetRobotZ(), g_robot.GetRobotRoll(), g_robot.GetRobotPitch(), g_robot.GetRobotYaw(), mlspointsum);
		}
		else if (g_loopcount > 0) {
			/* 描画関数の呼び出し */
			Drawing(g_robot.GetRobotX(), g_robot.GetRobotY(), g_robot.GetRobotZ(), g_robot.GetRobotRoll(), g_robot.GetRobotPitch(), g_robot.GetRobotYaw(), mlspointsum);
		}
		/* 描画停止回数のとき */
		if (g_loopcount == STOP_DRAWING) {
			/* 描画停止 */
			glutIdleFunc(0);
		}
	}
	/* データ終了のとき */
	else {
		/* 描画停止 */
		glutIdleFunc(0);
	}
	/* ループカウンタ加算 */
	g_loopcount++;
}

/*/////////////////////////////////////////////////////////////////////
観測点の座標変換～追跡処理
引数
imudata
返り値
なし
/////////////////////////////////////////////////////////////////////*/
void StartTracking() {
	int i, j;

	/* タイムスタンプのズレを合わせる．現状ではIMUのタイムスタンプが100msではなく50msや150msとなるときがある */
	if (g_loopcount == 0) {
		timestamp = imudata.GetImudataTimestamp();
	}
	else {
		timestamp = timestamp + LOCALIZATION_INTERVAL;
		imudata.SetImudataTimestamp(timestamp);
	}

	/* mlsのデータ読み込み */
	mlspointsum = MlsDataRead(imudata.GetImudataTimestamp());

	/*IAのために観測点をMLSの垂直角が大きい順に並べ替える*/
	Point_Sort(mlspointsum);

	if (imudataflag == -1) {
		imudata.SetImudataX(pre_imux);
		imudata.SetImudataY(pre_imuy);
		imudata.SetImudataZ(pre_imuz);
		imudata.SetImudataRoll(pre_imuroll);
		imudata.SetImudataPitch(pre_imupitch);
		imudata.SetImudataYaw(pre_imuyaw);
	}

	/*世界座標系原点指定*/
	//Initialize_Position();

	/* 自車位置のデータを格納 */
	g_robot.SetRobotX(imudata.GetImudataX());
	g_robot.SetRobotY(imudata.GetImudataY());
	g_robot.SetRobotZ(imudata.GetImudataZ());
	g_robot.SetRobotRoll(imudata.GetImudataRoll());
	g_robot.SetRobotPitch(imudata.GetImudataPitch());
	g_robot.SetRobotYaw(imudata.GetImudataYaw());

	/*ロボット座標系原点算出*/
	RobotPosition(imudata.GetImudataX(), imudata.GetImudataY(), imudata.GetImudataZ(), imudata.GetImudataRoll(), imudata.GetImudataPitch(), imudata.GetImudataYaw());

	//自車速度計算
	g_robot.SetRobotV(3.6*sqrt(((g_robot.GetRobotX() - pre_robotx)*(g_robot.GetRobotX() - pre_robotx)) + ((g_robot.GetRobotY() - pre_roboty)*(g_robot.GetRobotY() - pre_roboty))) / 0.1);

	/*pre_imux = imudata.GetImudataX();
	pre_imuy = imudata.GetImudataY();
	pre_imuz = imudata.GetImudataZ();
	pre_imuroll = imudata.GetImudataRoll();
	pre_imupitch = imudata.GetImudataPitch();
	pre_imuyaw = imudata.GetImudataYaw();

	pre_robotx = g_robot.GetRobotX();
	pre_roboty = g_robot.GetRobotY();
	pre_robotz = g_robot.GetRobotZ();
	pre_robotroll = g_robot.GetRobotRoll();
	pre_robotpitch = g_robot.GetRobotPitch();
	pre_robotyaw = g_robot.GetRobotYaw();
*/

	std::cout << "x:" << g_robot.GetRobotX() << ", y:" << g_robot.GetRobotY() << ", z:" << g_robot.GetRobotZ() <<
		", roll:" << g_robot.GetRobotRoll() << ", pitch:" << g_robot.GetRobotPitch() << ", yaw:" << g_robot.GetRobotYaw() << ", speed:" << g_robot.GetRobotV() << std::endl;

	/* 座標変換 */
	if (LINEAR_INTERPOLATION_SELECT == 0) {
		/* 二次元（x，y，yaw）の座標変換 */
		LinearInterpolationTwoDimention(imudata.GetImudataTimestamp(), g_robot.GetRobotX(), g_robot.GetRobotY(), g_robot.GetRobotYaw(), mlspointsum);
	}
	else if (LINEAR_INTERPOLATION_SELECT == 1) {
		double now_position[NUM_USE_LINEAR_INTERPOLATION_VARIABLES] = { 0 };    /* 現時刻におけるロボット位置，1：x，2：y，3：z，4：roll，5：pitch，6：yaw */

		/* eigenの行列に値を代入 */
		now_position[0] = imudata.GetImudataX();			/* x */
		now_position[1] = imudata.GetImudataY();			/* y */
		now_position[2] = imudata.GetImudataZ();			/* z */
		now_position[3] = imudata.GetImudataRoll();		/* roll */
		now_position[4] = imudata.GetImudataPitch();		/* pitch */
		now_position[5] = imudata.GetImudataYaw();		/* yaw */

		/* 三次元（x，y，z，roll，pitch，yaw）の座標変換 */
		LinearInterpolationThreeDimention(imudata.GetImudataTimestamp(), now_position, mlspointsum);
	}

	/*インクリメンタルアルゴリズム*/
	LayersIncrementalAlgorithm(mlspointsum, g_robot.GetRobotX(), g_robot.GetRobotY(), g_robot.GetRobotZ());

	/*環境地図差分*/
	if (Map_subtraction_Method == 1) {

		//ndt初期値＝前時刻ndt姿勢
		double pre_position[NUM_USE_LINEAR_INTERPOLATION_VARIABLES] = { 0 };
		pre_position[0] = pre_imux;			/* x */
		pre_position[1] = pre_imuy;			/* y */
		pre_position[2] = pre_imuz;			/* z */
		pre_position[3] = pre_imuroll;		/* roll */
		pre_position[4] = pre_imupitch;		/* pitch */
		pre_position[5] = pre_imuyaw;		/* yaw */
		LinearInterpolationThreeDimention_pre(pre_imutime, pre_position, mlspointsum);

		if (g_loopcount > START_TRACKING + 1) {
			difference_point(g_robot.GetRobotX(), g_robot.GetRobotY(), imudata.GetImudataZ());
		}
	}
	else {
		for (int i = 0; i < mlspointsum; ++i) {
			for (int j = 0; j < LAYER; ++j) {
				g_laserpoint[j][i].difference_point = 1;
			}
		}
	}

	if (g_loopcount > START_TRACKING + 1) {

		/* 移動物体検出 */
		movingobjects_num = MovingObjectDetection(g_robot.GetRobotX(), g_robot.GetRobotY(), g_robot.GetRobotZ(), g_robot.GetRobotYaw(), mlspointsum);
		if (g_loopcount > START_TRACKING + 10) {
			for (i = 0; i < movingobjects_num; i++) {
				/* 大きさ推定でセルから大きさの観測値を求めるため必要 */
				jointobnum[i] = movingobjects[i].total_cells;
				for (j = 0; j < movingobjects[i].total_cells; j++) {
					/* 移動物体の構成セルを地上固定座標系にして格納 */
					jointmove[i][j][0] = movingobjects[i].constituent_cells[j].x * CELL_SIZE + (CELL_SIZE / 2.0) + (robotcell_x - PROCESSING_RANGE);
					jointmove[i][j][1] = movingobjects[i].constituent_cells[j].y * CELL_SIZE + (CELL_SIZE / 2.0) + (robotcell_y - PROCESSING_RANGE);
					jointmove[i][j][2] = movingobjects[i].height_from_road;
					jointmove[i][j][3] = movingobjects[i].constituent_cells[j].z_road;
				}

				/* 観測値代表点から追跡を行う */
				observed_value_epresentative_point[i].value[0] = movingobjects[i].center[0];
				observed_value_epresentative_point[i].value[1] = movingobjects[i].center[1];
				observed_value_epresentative_point[i].matchflag = 0;
				for (j = 0; j < NUM_VERTEX; j++) {
					observed_value_epresentative_point[i].vertex[j][0] = movingobjects[i].apx_x[j];
					observed_value_epresentative_point[i].vertex[j][1] = movingobjects[i].apx_y[j];
				}
			}
			/* 移動物体追跡 */
			trackingobjects_total = MovingObjectTracking(movingobjects_num, trackingobjects_total);
			std::cout << "trackingobjects_total " << trackingobjects_total << std::endl;

			/* 追跡管理法 */
			trackingobjects_total = ArrangeTrack(trackingobjects_total, g_robot.GetRobotV());

			/* 追跡結果出力 */
			OutputRobot(g_loopcount, imudata);
			OutputTrack(trackingobjects_total, g_loopcount, imudata);

			/*環境地図生成*/
			CreateMap();
		}
	}
	pre_imux = imudata.GetImudataX();
	pre_imuy = imudata.GetImudataY();
	pre_imuz = imudata.GetImudataZ();
	pre_imuroll = imudata.GetImudataRoll();
	pre_imupitch = imudata.GetImudataPitch();
	pre_imuyaw = imudata.GetImudataYaw();
	pre_imutime = imudata.GetImudataTimestamp();

	pre_robotx = g_robot.GetRobotX();
	pre_roboty = g_robot.GetRobotY();
	pre_robotz = g_robot.GetRobotZ();
	pre_robotroll = g_robot.GetRobotRoll();
	pre_robotpitch = g_robot.GetRobotPitch();
	pre_robotyaw = g_robot.GetRobotYaw();
}

/*/////////////////////////////////////////////////////////////////////
環境地図生成（STOP_DRAWING指定する必要あり）
引数：なし
返り値：なし
/////////////////////////////////////////////////////////////////////*/
void CreateMap() {

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PassThrough<pcl::PointXYZI> pass;

	int point_count = 0;
	cloud->clear();
	cloud->width = mlspointsum * 32;
	cloud->height = 1;
	cloud->points.resize(mlspointsum * 32);

	for (int i = 0; i < mlspointsum; ++i)
	{
		for (int j = 0; j < 32; ++j)
		{
			if (g_laserpoint[j][i].ver_classification != Laserpoint::V_ROAD) {
				if (g_laserpoint[j][i].distance > 2.0) {
					cloud->points[point_count].x = g_laserpoint[j][i].x;
					cloud->points[point_count].y = g_laserpoint[j][i].y;
					cloud->points[point_count].z = g_laserpoint[j][i].z;
					cloud->points[point_count].intensity = g_laserpoint[j][i].ref_intensity;
					point_count++;
				}
			}
		}
	}

	cloud->height = 1;
	cloud->width = point_count;
	cloud->points.resize(cloud->height * cloud->width);
	if (cloud->size() > 0)
	{
		char file_name_pcd[50];
		{
			sprintf(file_name_pcd, "OutputMapData/map_fragment/cloud_%.d.pcd", g_loopcount);
		}
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("intensity");
		pass.setFilterLimits(0, FLT_MAX);
		pass.filter(*cloud);
		pcl::io::savePCDFileBinary(file_name_pcd, *cloud);
	}

	if (g_loopcount == STOP_DRAWING) {
		for (int main_count = 1; main_count <= STOP_DRAWING; main_count++) {
			std::cout << "ループ数:" << main_count << std::endl;

			//点群読み込み
			std::string file_str = "OutputMapData/map_fragment/cloud_" + std::to_string(main_count) + ".pcd";
			pcl::io::loadPCDFile(file_str, *cloud);
			if (cloud->size() == 0) {
				continue;
			}
			*map_cloud += *cloud;
		}

		pcl::io::savePCDFileBinary("OutputMapData/outputmap.pcd", *map_cloud);

		std::cout << "終了" << std::endl;
	}
}

/*/////////////////////////////////////////////////////////////////////
緯度経度をECEF直交座標系に変換
引数：緯度,経度,高度
返り値：なし
/////////////////////////////////////////////////////////////////////*/
Eigen::Vector3d to_rectangular_coordinate(double latitude_degree, double longitude_degree, double height_meter) {
	const double A = 6378137.0;
	const double F = 1.0 / 298.257223563;
	double latitude_rad = latitude_degree * M_PI / 180.0;
	double longitude_rad = longitude_degree * M_PI / 180.0;
	double E = 2.0*F - F * F;
	double N = A / sqrt(1.0 - E * sin(latitude_rad)*sin(latitude_rad));

	return Eigen::Vector3d(
		(N + height_meter)*cos(latitude_rad)*cos(longitude_rad),
		(N + height_meter)*cos(latitude_rad)*sin(longitude_rad),
		(N*(1.0 - E) + height_meter)*sin(latitude_rad)
	);
}

/*/////////////////////////////////////////////////////////////////////
世界座標系原点指定
引数：なし
返り値：なし
/////////////////////////////////////////////////////////////////////*/
void Initialize_Position() {
	double origin_point[3];	//緯度経度高度
	double latitude_degree = imudata.GetImudataLatitude();
	double longitude_degree = imudata.GetImudataLongitude();
	double height_meter = imudata.GetImudataHeight();

	//世界座標系原点の緯度経度高度
	origin_point[0] = 34.79998705;
	origin_point[1] = 135.7672586;
	origin_point[2] = 122.0927;
	double origin_latitude_rad = origin_point[0] * M_PI / 180.0;
	double origin_longitude_rad = origin_point[1] * M_PI / 180.0;

	//緯度経度高度を世界座標系に変換
	Eigen::Vector3d rectangular_position
		= to_rectangular_coordinate(latitude_degree, longitude_degree, height_meter)
		- to_rectangular_coordinate(origin_point[0], origin_point[1], origin_point[2]);
	Eigen::Vector3d result = Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(-(0.5*M_PI - origin_latitude_rad), Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(-origin_longitude_rad, Eigen::Vector3d::UnitZ())
		* rectangular_position;

	imudata.SetImudataX(result[0]);
	imudata.SetImudataY(result[1]);
	imudata.SetImudataZ(result[2]);
}