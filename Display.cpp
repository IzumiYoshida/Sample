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

int g_loopcount = 0;					/* ���[�v�̎��s�񐔁CGL�̊֐��Ԃł̋��L��������Ȃ����� */
int timestamp = 0;						/* GNSS/INS�������u�̃^�C���X�^���v�𖳗���荇�킹�� */
int mlspointsum = 0;					/* �ǂݍ��񂾊ϑ��_�� */
int movingobjects_num = 0;      /* ���o�ɂ���ē���ꂽ�ړ����̂̐� */
int trackingobjects_total = 0;								/* �ړ����̒ǐՐ� */
double pre_robotx, pre_roboty, pre_robotz, pre_robotroll, pre_robotpitch, pre_robotyaw,pre_imutime;
double pre_imux, pre_imuy, pre_imuz, pre_imuroll, pre_imupitch, pre_imuyaw;
int imudataflag;							/* ImuDataRead()�̃f�[�^�̃t���O�D0�F�ʏ�̏����C-1�F�f�[�^�����C1�F�f�[�^�I�� */

Imudata imudata;
ObservedValueEpresentativePoint observed_value_epresentative_point[UPPERLIMIT_TRACK];			/* �ϑ��l��\�_ */
extern int jointobnum[UPPERLIMIT_MOVING];																				/* �������ꂽ�ۂ̊ϑ��l�̐� */
extern double jointmove[UPPERLIMIT_MOVING][UPPERLIMIT_CELL][X_AND_Y_AND_Z + 1];			/* �������ꂽ�ړ����̊ϑ��l(�n��Œ�) */
const int STOP_DRAWING = 5000;									/* �`����~���鎞�� */
pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

/*/////////////////////////////////////////////////////////////////////
OpenGL�̒��S�ƂȂ鏈���֐�
����
�Ȃ�
�Ԃ�l
�Ȃ�
/////////////////////////////////////////////////////////////////////*/
void Display(void) {

	/* �`��̎��Ԓ����̂��߂̃X���[�v */
	AdjustTime();

	/* ���[�v�J�E���^�\�� */
	std::cout << "---------------���[�v�� " << g_loopcount << "---------------" << std::endl;

	/* microEV�p�f�[�^�ǂݍ��݊֐� */
	imudataflag = imudata.ImuDataRead();

	/* �f�[�^�����̂Ƃ� */
	if (imudataflag == -1) {
		std::cout << g_loopcount << "��ڂŃf�[�^��������!!" << std::endl;
	}

	/* �f�[�^���I�����Ă��Ȃ��Ƃ� */
	if (imudataflag != 1) {
		/* �ϑ��_�̍��W�ϊ��`�ǐՏ��� */
		StartTracking();
		/* �`��J�n */
		if (g_loopcount == 0) {
			ReturnLook();
			Drawing(g_robot.GetRobotX(), g_robot.GetRobotY(), g_robot.GetRobotZ(), g_robot.GetRobotRoll(), g_robot.GetRobotPitch(), g_robot.GetRobotYaw(), mlspointsum);
		}
		else if (g_loopcount > 0) {
			/* �`��֐��̌Ăяo�� */
			Drawing(g_robot.GetRobotX(), g_robot.GetRobotY(), g_robot.GetRobotZ(), g_robot.GetRobotRoll(), g_robot.GetRobotPitch(), g_robot.GetRobotYaw(), mlspointsum);
		}
		/* �`���~�񐔂̂Ƃ� */
		if (g_loopcount == STOP_DRAWING) {
			/* �`���~ */
			glutIdleFunc(0);
		}
	}
	/* �f�[�^�I���̂Ƃ� */
	else {
		/* �`���~ */
		glutIdleFunc(0);
	}
	/* ���[�v�J�E���^���Z */
	g_loopcount++;
}

/*/////////////////////////////////////////////////////////////////////
�ϑ��_�̍��W�ϊ��`�ǐՏ���
����
imudata
�Ԃ�l
�Ȃ�
/////////////////////////////////////////////////////////////////////*/
void StartTracking() {
	int i, j;

	/* �^�C���X�^���v�̃Y�������킹��D����ł�IMU�̃^�C���X�^���v��100ms�ł͂Ȃ�50ms��150ms�ƂȂ�Ƃ������� */
	if (g_loopcount == 0) {
		timestamp = imudata.GetImudataTimestamp();
	}
	else {
		timestamp = timestamp + LOCALIZATION_INTERVAL;
		imudata.SetImudataTimestamp(timestamp);
	}

	/* mls�̃f�[�^�ǂݍ��� */
	mlspointsum = MlsDataRead(imudata.GetImudataTimestamp());

	/*IA�̂��߂Ɋϑ��_��MLS�̐����p���傫�����ɕ��בւ���*/
	Point_Sort(mlspointsum);

	if (imudataflag == -1) {
		imudata.SetImudataX(pre_imux);
		imudata.SetImudataY(pre_imuy);
		imudata.SetImudataZ(pre_imuz);
		imudata.SetImudataRoll(pre_imuroll);
		imudata.SetImudataPitch(pre_imupitch);
		imudata.SetImudataYaw(pre_imuyaw);
	}

	/*���E���W�n���_�w��*/
	//Initialize_Position();

	/* ���Ԉʒu�̃f�[�^���i�[ */
	g_robot.SetRobotX(imudata.GetImudataX());
	g_robot.SetRobotY(imudata.GetImudataY());
	g_robot.SetRobotZ(imudata.GetImudataZ());
	g_robot.SetRobotRoll(imudata.GetImudataRoll());
	g_robot.SetRobotPitch(imudata.GetImudataPitch());
	g_robot.SetRobotYaw(imudata.GetImudataYaw());

	/*���{�b�g���W�n���_�Z�o*/
	RobotPosition(imudata.GetImudataX(), imudata.GetImudataY(), imudata.GetImudataZ(), imudata.GetImudataRoll(), imudata.GetImudataPitch(), imudata.GetImudataYaw());

	//���ԑ��x�v�Z
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

	/* ���W�ϊ� */
	if (LINEAR_INTERPOLATION_SELECT == 0) {
		/* �񎟌��ix�Cy�Cyaw�j�̍��W�ϊ� */
		LinearInterpolationTwoDimention(imudata.GetImudataTimestamp(), g_robot.GetRobotX(), g_robot.GetRobotY(), g_robot.GetRobotYaw(), mlspointsum);
	}
	else if (LINEAR_INTERPOLATION_SELECT == 1) {
		double now_position[NUM_USE_LINEAR_INTERPOLATION_VARIABLES] = { 0 };    /* �������ɂ����郍�{�b�g�ʒu�C1�Fx�C2�Fy�C3�Fz�C4�Froll�C5�Fpitch�C6�Fyaw */

		/* eigen�̍s��ɒl���� */
		now_position[0] = imudata.GetImudataX();			/* x */
		now_position[1] = imudata.GetImudataY();			/* y */
		now_position[2] = imudata.GetImudataZ();			/* z */
		now_position[3] = imudata.GetImudataRoll();		/* roll */
		now_position[4] = imudata.GetImudataPitch();		/* pitch */
		now_position[5] = imudata.GetImudataYaw();		/* yaw */

		/* �O�����ix�Cy�Cz�Croll�Cpitch�Cyaw�j�̍��W�ϊ� */
		LinearInterpolationThreeDimention(imudata.GetImudataTimestamp(), now_position, mlspointsum);
	}

	/*�C���N�������^���A���S���Y��*/
	LayersIncrementalAlgorithm(mlspointsum, g_robot.GetRobotX(), g_robot.GetRobotY(), g_robot.GetRobotZ());

	/*���n�}����*/
	if (Map_subtraction_Method == 1) {

		//ndt�����l���O����ndt�p��
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

		/* �ړ����̌��o */
		movingobjects_num = MovingObjectDetection(g_robot.GetRobotX(), g_robot.GetRobotY(), g_robot.GetRobotZ(), g_robot.GetRobotYaw(), mlspointsum);
		if (g_loopcount > START_TRACKING + 10) {
			for (i = 0; i < movingobjects_num; i++) {
				/* �傫������ŃZ������傫���̊ϑ��l�����߂邽�ߕK�v */
				jointobnum[i] = movingobjects[i].total_cells;
				for (j = 0; j < movingobjects[i].total_cells; j++) {
					/* �ړ����̂̍\���Z����n��Œ���W�n�ɂ��Ċi�[ */
					jointmove[i][j][0] = movingobjects[i].constituent_cells[j].x * CELL_SIZE + (CELL_SIZE / 2.0) + (robotcell_x - PROCESSING_RANGE);
					jointmove[i][j][1] = movingobjects[i].constituent_cells[j].y * CELL_SIZE + (CELL_SIZE / 2.0) + (robotcell_y - PROCESSING_RANGE);
					jointmove[i][j][2] = movingobjects[i].height_from_road;
					jointmove[i][j][3] = movingobjects[i].constituent_cells[j].z_road;
				}

				/* �ϑ��l��\�_����ǐՂ��s�� */
				observed_value_epresentative_point[i].value[0] = movingobjects[i].center[0];
				observed_value_epresentative_point[i].value[1] = movingobjects[i].center[1];
				observed_value_epresentative_point[i].matchflag = 0;
				for (j = 0; j < NUM_VERTEX; j++) {
					observed_value_epresentative_point[i].vertex[j][0] = movingobjects[i].apx_x[j];
					observed_value_epresentative_point[i].vertex[j][1] = movingobjects[i].apx_y[j];
				}
			}
			/* �ړ����̒ǐ� */
			trackingobjects_total = MovingObjectTracking(movingobjects_num, trackingobjects_total);
			std::cout << "trackingobjects_total " << trackingobjects_total << std::endl;

			/* �ǐՊǗ��@ */
			trackingobjects_total = ArrangeTrack(trackingobjects_total, g_robot.GetRobotV());

			/* �ǐՌ��ʏo�� */
			OutputRobot(g_loopcount, imudata);
			OutputTrack(trackingobjects_total, g_loopcount, imudata);

			/*���n�}����*/
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
���n�}�����iSTOP_DRAWING�w�肷��K�v����j
�����F�Ȃ�
�Ԃ�l�F�Ȃ�
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
			std::cout << "���[�v��:" << main_count << std::endl;

			//�_�Q�ǂݍ���
			std::string file_str = "OutputMapData/map_fragment/cloud_" + std::to_string(main_count) + ".pcd";
			pcl::io::loadPCDFile(file_str, *cloud);
			if (cloud->size() == 0) {
				continue;
			}
			*map_cloud += *cloud;
		}

		pcl::io::savePCDFileBinary("OutputMapData/outputmap.pcd", *map_cloud);

		std::cout << "�I��" << std::endl;
	}
}

/*/////////////////////////////////////////////////////////////////////
�ܓx�o�x��ECEF�������W�n�ɕϊ�
�����F�ܓx,�o�x,���x
�Ԃ�l�F�Ȃ�
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
���E���W�n���_�w��
�����F�Ȃ�
�Ԃ�l�F�Ȃ�
/////////////////////////////////////////////////////////////////////*/
void Initialize_Position() {
	double origin_point[3];	//�ܓx�o�x���x
	double latitude_degree = imudata.GetImudataLatitude();
	double longitude_degree = imudata.GetImudataLongitude();
	double height_meter = imudata.GetImudataHeight();

	//���E���W�n���_�̈ܓx�o�x���x
	origin_point[0] = 34.79998705;
	origin_point[1] = 135.7672586;
	origin_point[2] = 122.0927;
	double origin_latitude_rad = origin_point[0] * M_PI / 180.0;
	double origin_longitude_rad = origin_point[1] * M_PI / 180.0;

	//�ܓx�o�x���x�𐢊E���W�n�ɕϊ�
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