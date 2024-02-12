// PointCloudViewer.hpp
//
/********************************************************************************
* Viewer for Lidar point cloud.
* 
* PointCloudViewerʵ��3D���ڡ���ҪPCL֧�֡�
* 
* �����������裺
* 1. ����PointCloudViewer����ͬʱָ���������ơ��������Ʋ����޸ģ�����������޸ġ�
* 2. ��OpenViewer()�򿪴��ڡ����ڿ������ò��������⡢λ�á���С������ɫ��
* 3. �򿪴��ں���PointCloud����Ȼ����ViewerAddPointXYZ()��ViewerAddPointXYZRGBA()�����Ƽ���3D���ڡ�
*    ����3D���ڵĵ��ƿ��Ըı���ɫ�ʹ�С������ɾ����
* 
* �ļ���д��
* 1. ��loadPCDFile()��loadPLYFile()��ȡ�����ļ�������PointCloud����
* 2. ��savePCDFile()��savePLYFile()����PointCloud����
*
* ����������
* 1. ������۲���λ�ˣ�����ViewerSetCamera()ֱ���޸�����������ı���ʾЧ������ViewerGetCamera()��ȡ��ǰ���������
* 2. ������ת����ViewerSetRotatingCamera()ʹ�����������ĵ���ת��
* 3. �����᣺����ViewerShowCoordinates()�������������ʾ��
* 4. ������λ������SetDefaultParameters()�ָ�����ȱʡֵ��Ȼ�����OpenViewer()��Ч��
*
* �������
* �����ơ�ָ�����״�����ת���õ����޹���ռ�㼯��ͨ������ɫ����ʾʱ����ָ����ɫ��
*         �����״�������ںϺ�ÿ��������ɫ����Ϊ��ɫ���ơ�
*         ��ʾʱ��������һ���ַ�����ʶ���Ѿ���ʾ�ĵ��ƿ���ɾ����������ݡ�
* �����ڡ�������ʾ���ơ��������ת���ƣ��ü��̷����ƽ�Ƶ��ơ�������ʾ�����ᣬxyz��ֱ�Ϊ���������߶Ρ�
*         �����ᶨ����������ϵ���ռ�������������ϵ�¶��塣
* �����������۲���λ�ˣ����������ϵ���������ϵ��ԭ��Ϊ��Ļ�����룬x��y��z��Ϊ��Ļ���ҡ����Ϻ����⡣
*         ��˾�����������Ļ����ʾЧ������������ϵ���������ϵ�ı任����
* ���������ϵ����Ҫ���ĸ��������壺
*        ��1�����㣨focal�������ߣ��������ϵz�ᣩ�ľ۽��㣬��ʾʱλ����Ļ�����룬���������תʱ�Ĳ����㡣
*             ��ʼֵΪ��������ϵԭ�㣬��(0,0,0)��
*        ��2�����λ�ã�pos�������������ϵ��ԭ�㡣��������ķ������������ϵz��ķ��򣬾��������ʾ����С������
*        ��3�����Ϸ���up�������������ϵ��y�ᡣ
*             ��ʼֵΪ��������ϵz�᷽�򣬼�(0,0,1)����ʱ��������ϵz��Ϊ��Ļ���ϡ�
*        ��4��y�����ӽǣ�fovy�������ͶӰ����Ļ�Ŀռ䷶Χ(0��~180��)��
* 
* 
* Writen by Lin, Jingyu, linjy02@hotmail.com, 2021.05
* Revised by Lin, Jingyu, linjy02@hotmail.com, 2022.03
* Revised by Lin, Jingyu, linjy02@hotmail.com, 2023.05
* Revised by Lin, Jingyu, linjy02@hotmail.com, 2023.07
*
********************************************************************************/

#ifndef PointCloudViewer_HPP
#define PointCloudViewer_HPP

//#include <windows.h>
//#include <memory> // for smart pointer
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
//#include "..\LidarDataContainer\LidarDataBlock.hpp"
//#include "..\LidarDataContainer\PatchGeometry.hpp"

namespace Lidar3D
{
	// 3D����������۲��ߣ����ݽṹ
	struct ViewerCamera
	{
		//pcl::visualization::Camera Cam;
		double Focal[3];   // view point (fixed in rotation)
		double Pos[3];     // camera position
		double Up[3];      // camera's up direction
		double Clip[2];    // clipping planes depths (near & far)
		double Fovy;       // field of view in y direction (0~180 deg)
		double WinSize[2]; // size of the window on the screen
		double WinPos[2];  // position of the window on the screen
		double ViewMatrix[4][4]; // transformation from  world coordinates to camera coordinates
	};

	// ������ʾ���ƵĲ���
	struct PointCloudParams
	{
		std::int32_t point_id;		// pointcloud id
		std::int32_t point_radius;	// size of a single point
		float point_rgb[4];			// RGB of points 
		void* point_ptr;			// pointer to a PointCloud<>
		void* curve_ptr;			// pointer to a curvature PointCloud<>
	};

	//class LidarDataBlock;
	//class PatchGeometry;

	class __declspec(dllexport) PointCloudViewer
	{
	public:
		PointCloudViewer(const char* name);
		~PointCloudViewer();

		/************************ �������� ************************/
		// ���ܣ��� / �رմ��ڡ����ش��ڴ�״̬��0��1����
		// ˵�����رմ��ں����е��Ʊ�ɾ���������ı䴰�ں����������
		int OpenViewer();
		int CloseViewer();

		// ���ܣ��õ����ڴ�״̬��
		inline int IsViewing();

		// ���ܣ����ô��ں��������Ϊ��ʼֵ������PointCloudViewer����ʱ��ֵ��������OpenViewer()�򿪴��ں���Ч��
		void SetDefaultParameters();

		// ���ܣ������÷�����ƶ����ʱ�Ĳ�����ȱʡֵΪ0.2��
		void SetCamTransStep(double step=-1);
		inline int GetCamTransStep();

		/************************ ���ô��� ************************/
		// ���ܣ����ô��ڱ��⡣��ʼ����Ϊ�������ơ�
		// ����ֵ��-2 - ָ��δִ�С�-1 - ָ��ִ�к󷵻�ֵ��ʱ��0 - ָ��ִ�д���1 - ִ��ִ�гɹ�
		int ViewerSetTitle(char* title);

		// ���ܣ����ô���λ�úʹ�С��
		// ����ֵ��-2 - ָ��δִ�С�-1 - ָ��ִ�к󷵻�ֵ��ʱ��0 - ָ��ִ�д���1 - ִ��ִ�гɹ�
		// ˵������GetSystemMetrics(SM_CXFULLSCREEN)��GetSystemMetrics(SM_CYFULLSCREEN)�õ���Ļ�Ŀ�͸ߡ�
		int ViewerSetRect(int left, int top, int width, int height);

		// ���ܣ����ô��ڱ���ɫ����ɫȡֵ0~1��
		// ����ֵ��-2 - ָ��δִ�С�-1 - ָ��ִ�к󷵻�ֵ��ʱ��0 - ָ��ִ�д���1 - ִ��ִ�гɹ�
		int ViewerSetBgColor(double bgRed, double bgGreen, double bgBlue);

		// ���ܣ���ʾ�����ᣨbShowΪ1������ʾ�����ᣨbShowΪ0����
		// ����ֵ��-2 - ָ��δִ�С�-1 - ָ��ִ�к󷵻�ֵ��ʱ��0 - ָ��ִ�д���1 - ִ��ִ�гɹ�
		int ViewerShowCoordinates(int bShow);

		// ���ܣ��л������ᵱǰ��ʾ״̬��
		// ����ֵ��-2 - ָ��δִ�С�-1 - ָ��ִ�к󷵻�ֵ��ʱ��0 - ָ��ִ�д���1 - ִ��ִ�гɹ�
		int ViewerShowCoordinates();

		/************************ ������� ************************/
		// ���ܣ��������������
		// ���룺Focal[3] - ���㣨�̶��ӵ㣩�� Pos[3] - ���ԭ��λ�á�Up[3] - ����Ϸ����������ϵy���򣩡�
		//       Fovy - y�����ӽǷ�Χ(0��~180��)��
		// ����ֵ��-2 - ָ��δִ�С�-1 - ָ��ִ�к󷵻�ֵ��ʱ��0 - ָ��ִ�д���1 - ִ��ִ�гɹ�
		int ViewerSetCamera(double Focal[3], double Pos[3], double Up[3], double Fovy);

		// ���ܣ���ȡ��ǰ����Ĳ����͹۲�����Լ�����λ�ô�С��
		void ViewerGetCamera(ViewerCamera& view_cam);

		// ���ܣ����������ת����ת������������㣨�̶��ӵ㣩�����������ġ�
		//       ��ת���������������ϵ�ж��壨�ϵ��ӽǣ���Ҳ�������������ϵ�ж��壨�۲����ӽǣ���
		// ���룺speed - ת�٣�deg/s���� (axisX,axisY,axisZ) - ��ת�ᡣ
		//       axisCoordinates - ��ת����������ϵ��0 - �������ϵ��1 - ��������ϵ����������������ϵ����
		// ˵�����������ϵ��x��y��z��ֱ�����Ļ���ҡ����Ϻ����⣬��˿ɼ��ĵ��ƶ��ڸ�z�ᡣ
		//       ��ת��������������������Ϊ�������ת�����Ե��ƿ���ȥ�Ƿ�����ת��
		void ViewerSetRotatingCamera(double speed, double axisX, double axisY, double axisZ, int axisCoordinates = 0);

		/************************ ��ʾ���� ************************/
		// ���ܣ���ʾ�µ��ƻ���µ��� - ��ɫ���ơ�
		// ���룺id - ���Ʊ�ʶ������������ͬ���Ƶĵ��ƿ�ͬʱ��ʾ����ͬ���Ƶĵ��Ʊ��滻��
		//       pc - �������ݡ�pntRadius - ��Ĵ�С����ֵ��Ч����
		//       (pntR, pntG, pntB) - ��ʾ����ɫ��0~1����
		// ����ֵ��-2 - ָ��δִ�С�-1 - ָ��ִ�к󷵻�ֵ��ʱ��0 - ָ��ִ�д���1 - ִ��ִ�гɹ�
		//int ViewerAddPointXYZ(int id, pcl::PointCloud<pcl::PointXYZ>::Ptr pc, int pntRadius = 1,
		//	float pntR = 1.0f, float pntG = 1.0f, float pntB = 1.0f);
		int ViewerAddPointXYZ(int id, pcl::PointCloud<pcl::PointXYZ>::Ptr pc);

		// ���ܣ���ʾ�µ��ƻ���µ��� - ��ɫ���ơ�
		// ���룺id - ���Ʊ�ʶ������������ͬ���Ƶĵ��ƿ�ͬʱ��ʾ����ͬ���Ƶĵ��Ʊ��滻��
		//       pc - �������ݡ�pntRadius - ��Ĵ�С����ֵ��Ч����
		// ����ֵ��-2 - ָ��δִ�С�-1 - ָ��ִ�к󷵻�ֵ��ʱ��0 - ָ��ִ�д���1 - ִ��ִ�гɹ�
		//int ViewerAddPointXYZRGBA(int id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc, int pntRadius = 1);
		int ViewerAddPointXYZRGBA(int id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc);

		// ���ܣ���ʾ�µ��ƻ���µ��� - ���ߡ�
		// ���룺id - ���Ʊ�ʶ������������ͬ���Ƶĵ��ƿ�ͬʱ��ʾ����ͬ���Ƶĵ��Ʊ��滻��
		//       pc - �������ݡ�
		// ����ֵ��-2 - ָ��δִ�С�-1 - ָ��ִ�к󷵻�ֵ��ʱ��0 - ָ��ִ�д���1 - ִ��ִ�гɹ�
		int ViewerAddNormals(int id, pcl::PointCloud<pcl::PointNormal>::Ptr pc);

		// ���ܣ���ʾ�µ��ƻ���µ��� - ���ʡ�
		// ���룺id - ���Ʊ�ʶ������������ͬ���Ƶĵ��ƿ�ͬʱ��ʾ����ͬ���Ƶĵ��Ʊ��滻��
		//       pc - �������ݡ�curv - �������ݡ�
		// ����ֵ��-2 - ָ��δִ�С�-1 - ָ��ִ�к󷵻�ֵ��ʱ��0 - ָ��ִ�д���1 - ִ��ִ�гɹ�
		int ViewerAddCurvatures(int id, pcl::PointCloud<pcl::PointNormal>::Ptr pc,
			pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curv);

		// ���ܣ�ɾ��ָ�����ƻ�ȫ�����ơ�
		// ���룺id - ���Ʊ�ʶ��-1��ʾȫ������
		// ����ֵ��-2 - ָ��δִ�С�-1 - ָ��ִ�к󷵻�ֵ��ʱ��0 - ָ��ִ�д���1 - ִ��ִ�гɹ�
		int ViewerRemovePointCloud(int id = -1);

		// ���ܣ��ı���Ƶĵ�Ĵ�С���л����ƴ�С���Բ���ѡ�л���˸����Ч��
		// ���룺id - ���Ʊ�ʶ������������ͬ���Ƶĵ��ƿ�ͬʱ��ʾ����ͬ���Ƶĵ��Ʊ��滻��
		//       pntRadius - ��Ĵ�С����ֵ��Ч����
		// ����ֵ��-2 - ָ��δִ�С�-1 - ָ��ִ�к󷵻�ֵ��ʱ��0 - ָ��ִ�д���1 - ִ��ִ�гɹ�
		int ViewerSetPointRadius(int id, int pntRadius = 1);

		// ���ܣ��ı���Ƶĵ����ɫ���л�������ɫ���Բ���ѡ�л���˸����Ч����ע���ɫ���Ƶ���ɫ�ᱻ���ǡ�
		// ���룺id - ���Ʊ�ʶ������������ͬ���Ƶĵ��ƿ�ͬʱ��ʾ����ͬ���Ƶĵ��Ʊ��滻��
		//       (pntR, pntG, pntB) - ��ʾ����ɫ��0~1����
		// ����ֵ��-2 - ָ��δִ�С�-1 - ָ��ִ�к󷵻�ֵ��ʱ��0 - ָ��ִ�д���1 - ִ��ִ�гɹ�
		int ViewerSetPointColor(int id, float pntR = 1.0f, float pntG = 1.0f, float pntB = 1.0f);

		/************************ ���� ************************/
		// ���ܣ����ڲ��ԡ�
		// ����ֵ��-2 - ָ��δִ�С�-1 - ָ��ִ�к󷵻�ֵ��ʱ��0 - ָ��ִ�д���1 - ִ��ִ�гɹ�
		int ViewerTester(int id = -1); 

		/************************ ������ƣ���̬������ ************************/
		//// ���ܣ���������ת��ΪPointCloud���� - ��ɫ����
		//// ���룺point_cloud - �յ���ָ�롣block - �ں��������顣
		////       r0, nPntRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		//// �����point_cloud - ת���õ���PointCloud����
		//// ����ֵ��ת���ĵ��Ƶĵ�����
		//static int PointsetToPointXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, LidarDataBlock& block,
		//	int r0 = 0, int nPntRow = 0, int c0 = 0, int nCol = 0);

		//// ���ܣ���������ת��ΪPointCloud���� - ����ǿ�ĵ���
		//// ���룺point_cloud - �յ���ָ�롣block - �ں��������顣
		////       r0, nPntRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		//// �����point_cloud - ת���õ���PointCloud����
		//// ����ֵ��ת���ĵ��Ƶĵ�����
		//static int PointsetToPointXYZI(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, LidarDataBlock& block,
		//	int r0 = 0, int nPntRow = 0, int c0 = 0, int nCol = 0);

		//// ���ܣ���������ת��ΪPointCloud���� - ��ɫ����
		//// ���룺point_cloud - �յ���ָ�롣block - �ں��������顣
		////       r0, nPntRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		//// �����point_cloud - ת���õ���PointCloud����
		//// ����ֵ��ת���ĵ��Ƶĵ�����
		//static int PointsetToPointXYZRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, LidarDataBlock& block,
		//	int r0 = 0, int nPntRow = 0, int c0 = 0, int nCol = 0);

		//// ���ܣ����ε���ת��ΪPointCloud���� - ��������
		//// ���룺point_cloud - �յ���ָ�롣block - ���ε��ơ�minSlope - ȥ����б��С�ĵ㡣
		////       r0, nRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		//// �����point_cloud - ת���õ���PointCloud����
		//// ����ֵ��ת���ĵ��Ƶĵ�����
		//static int PointGeoToPointXYZI(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, PatchGeometry& block, double minSlope,
		//	int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		//// ���ܣ����ε���ת��ΪPointCloud���� - ������
		//// ���룺point_cloud - �յ���ָ�롣block - ���ε��ơ�minSlope - ȥ����б��С�ĵ㡣
		////       r0, nRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		//// �����point_cloud - ת���õ���PointCloud����
		//// ����ֵ��ת���ĵ��Ƶĵ�����
		//static int PointGeoToPointNormal(pcl::PointCloud<pcl::PointNormal>::Ptr& point_cloud, PatchGeometry& block, double minSlope,
		//	int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		//// ���ܣ����ε���ת��ΪPointCloud���� - ����
		//// ���룺point_cloud - �յ���ָ�롣block - ���ε��ơ�minSlope - ȥ����б��С�ĵ㡣
		////       r0, nRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		//// �����point_cloud - ת���õ���PointCloud����
		//// ����ֵ��ת���ĵ��Ƶĵ�����
		//static int PointGeoToCurvatures(pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr& point_cloud, PatchGeometry& block, double minSlope,
		//	int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		/************************ ��д���ƣ���̬������ ************************/
		// ���ܣ���PCD�ļ���ȡPointCloud����
		// ���룺file_name - �ļ�����point_cloud - �յ���ָ�롣
		// �����point_cloud - �õ����ݵ�PointCloud����
		// ����ֵ��0��ʾ�ɹ���<0��ʾʧ�ܡ�
		static int loadPCDFile(const std::string& file_name, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud);
		static int loadPCDFile(const std::string& file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud);
		static int loadPCDFile(const std::string& file_name, pcl::PointCloud<pcl::PointNormal>::Ptr& point_cloud);

		// ���ܣ���PointCloud����д��PCD�ļ�
		// ���룺file_name - �ļ�����point_cloud - �����̵�PointCloud����binary_mode - �ö����Ƹ�ʽ����ASCII��ʽ
		// ����ֵ��0��ʾ�ɹ���<0��ʾʧ�ܡ�
		static int savePCDFile(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, bool binary_mode = false);
		static int savePCDFile(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, bool binary_mode = false);
		static int savePCDFile(const std::string& file_name, const pcl::PointCloud<pcl::PointNormal>::Ptr& point_cloud, bool binary_mode = false);

		// ���ܣ���PLY�ļ���ȡPointCloud����
		// ���룺file_name - �ļ�����point_cloud - �յ���ָ�롣
		// �����point_cloud - �õ����ݵ�PointCloud����
		// ����ֵ��0��ʾ�ɹ���<0��ʾʧ�ܡ�
		static int loadPLYFile(const std::string& file_name, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud);
		static int loadPLYFile(const std::string& file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud);
		static int loadPLYFile(const std::string& file_name, pcl::PointCloud<pcl::PointNormal>::Ptr& point_cloud);

		// ���ܣ���PointCloud����д��PLY�ļ�
		// ���룺file_name - �ļ�����point_cloud - �����̵�PointCloud����binary_mode - �ö����Ƹ�ʽ����ASCII��ʽ
		// ����ֵ��0��ʾ�ɹ���<0��ʾʧ�ܡ�
		static int savePLYFile(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, bool binary_mode = false);
		static int savePLYFile(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, bool binary_mode = false);
		static int savePLYFile(const std::string& file_name, const pcl::PointCloud<pcl::PointNormal>::Ptr& point_cloud, bool binary_mode = false);


	private:
		//	char privatedata[728];
	/******** recover above and remove below in the exported header file ********/
	public:
		// system variables
		std::int32_t bViewing;				// switch of the viewer. 
		double CamTransStep;		// step when translating camera by keyborad
		// for command 
		int CommandIn();			// enter command critical section. 1: success, -1: timeout
		int CommandOut();			// exit command critical section. 1: return success, 0: return failure, -1: timeout
		std::int32_t cmdCode;				// viewer command (enum CmdID)
		std::int32_t cmdRet;				// return code of viewer command
		// window command parameters
		char strViewerTitle[256];	// viewer title. 
		std::int32_t viewerLoc[2];			// position of viewer in screen coordinates
		std::int32_t viewerSize[2];		// size of viewer in screen coordinates
		double viewerRGB[3];		// background color of viewer (0.0~1.0)
		std::int32_t bCoordinates;			// switch for coordinates
		// camera command parameters
		std::int32_t RotWorldAxis;			// world axis or not for camera rotation
		double RotAxis[3];			// axis of camera rotation
		double RotSpeed;			// speed of camera rotation
		double camFocal[3];			// camera viewpoint (fixed in rotation)
		double camPos[3];			// camera position
		double camUp[3];			// camera's up direction
		double camFovy;				// camera field of view (0~180 deg)
		ViewerCamera currCam;		// used to record current camera
		// pointcloud command parameters
		PointCloudParams pc_params;
	};
}
#endif // #ifndef PointCloudViewer_HPP
