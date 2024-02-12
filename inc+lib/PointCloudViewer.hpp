// PointCloudViewer.hpp
//
/********************************************************************************
* Viewer for Lidar point cloud.
* 
* PointCloudViewer实现3D窗口。需要PCL支持。
* 
* 基本操作步骤：
* 1. 创建PointCloudViewer对象，同时指定窗口名称。窗口名称不能修改，但标题可以修改。
* 2. 用OpenViewer()打开窗口。窗口可以设置参数：标题、位置、大小、背景色。
* 3. 打开窗口后构造PointCloud对象，然后用ViewerAddPointXYZ()和ViewerAddPointXYZRGBA()将点云加入3D窗口。
*    加入3D窗口的点云可以改变颜色和大小，可以删除。
* 
* 文件读写：
* 1. 用loadPCDFile()、loadPLYFile()读取点云文件，构造PointCloud对象。
* 2. 用savePCDFile()、savePLYFile()存盘PointCloud对象。
*
* 其它操作：
* 1. 相机（观察者位姿）：用ViewerSetCamera()直接修改相机参数，改变显示效果；用ViewerGetCamera()读取当前相机参数。
* 2. 点云旋转：用ViewerSetRotatingCamera()使点云绕着中心点旋转。
* 3. 坐标轴：调用ViewerShowCoordinates()开关坐标轴的显示。
* 4. 参数复位：调用SetDefaultParameters()恢复参数缺省值，然后调用OpenViewer()生效。
*
* 基本概念：
* 【点云】指激光雷达数据转换得到的无规则空间点集。通常无颜色，显示时可以指定颜色。
*         激光雷达与相机融合后每个点获得颜色，成为彩色点云。
*         显示时，点云用一个字符串标识。已经显示的点云可以删除或更新数据。
* 【窗口】用于显示点云。用鼠标旋转点云，用键盘方向键平移点云。可以显示坐标轴，xyz轴分别为红绿蓝短线段。
*         坐标轴定义世界坐标系。空间点均在世界坐标系下定义。
* 【相机】定义观察者位姿，即相机坐标系。相机坐标系的原点为屏幕正中央，x、y、z轴为屏幕向右、向上和向外。
*         因此决定点云在屏幕的显示效果（世界坐标系到相机坐标系的变换）。
* 【相机坐标系】主要由四个参数定义：
*        （1）焦点（focal）是视线（相机坐标系z轴）的聚焦点，显示时位于屏幕正中央，是用鼠标旋转时的不动点。
*             初始值为世界坐标系原点，即(0,0,0)。
*        （2）相机位置（pos）定义相机坐标系的原点。它到焦点的方向是相机坐标系z轴的反向，距离决定显示的缩小倍数。
*        （3）向上方向（up）定义相机坐标系的y轴。
*             初始值为世界坐标系z轴方向，即(0,0,1)。这时世界坐标系z轴为屏幕向上。
*        （4）y方向视角（fovy）定义可投影到屏幕的空间范围(0°~180°)。
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
	// 3D窗口相机（观察者）数据结构
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

	// 用于显示点云的参数
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

		/************************ 基本函数 ************************/
		// 功能：打开 / 关闭窗口。返回窗口打开状态（0或1）。
		// 说明：关闭窗口后所有点云被删除，但不改变窗口和相机参数。
		int OpenViewer();
		int CloseViewer();

		// 功能：得到窗口打开状态。
		inline int IsViewing();

		// 功能：设置窗口和相机参数为初始值（创建PointCloudViewer对象时的值）。调用OpenViewer()打开窗口后生效。
		void SetDefaultParameters();

		// 功能：管理用方向键移动相机时的步长。缺省值为0.2。
		void SetCamTransStep(double step=-1);
		inline int GetCamTransStep();

		/************************ 设置窗口 ************************/
		// 功能：设置窗口标题。初始标题为窗口名称。
		// 返回值：-2 - 指令未执行。-1 - 指令执行后返回值超时。0 - 指令执行错误。1 - 执行执行成功
		int ViewerSetTitle(char* title);

		// 功能：设置窗口位置和大小。
		// 返回值：-2 - 指令未执行。-1 - 指令执行后返回值超时。0 - 指令执行错误。1 - 执行执行成功
		// 说明：用GetSystemMetrics(SM_CXFULLSCREEN)和GetSystemMetrics(SM_CYFULLSCREEN)得到屏幕的宽和高。
		int ViewerSetRect(int left, int top, int width, int height);

		// 功能：设置窗口背景色。颜色取值0~1。
		// 返回值：-2 - 指令未执行。-1 - 指令执行后返回值超时。0 - 指令执行错误。1 - 执行执行成功
		int ViewerSetBgColor(double bgRed, double bgGreen, double bgBlue);

		// 功能：显示坐标轴（bShow为1）或不显示坐标轴（bShow为0）。
		// 返回值：-2 - 指令未执行。-1 - 指令执行后返回值超时。0 - 指令执行错误。1 - 执行执行成功
		int ViewerShowCoordinates(int bShow);

		// 功能：切换坐标轴当前显示状态。
		// 返回值：-2 - 指令未执行。-1 - 指令执行后返回值超时。0 - 指令执行错误。1 - 执行执行成功
		int ViewerShowCoordinates();

		/************************ 设置相机 ************************/
		// 功能：设置相机参数。
		// 输入：Focal[3] - 焦点（固定视点）。 Pos[3] - 相机原点位置。Up[3] - 相机上方（相机坐标系y方向）。
		//       Fovy - y方向视角范围(0°~180°)。
		// 返回值：-2 - 指令未执行。-1 - 指令执行后返回值超时。0 - 指令执行错误。1 - 执行执行成功
		int ViewerSetCamera(double Focal[3], double Pos[3], double Up[3], double Fovy);

		// 功能：读取当前相机的参数和观察矩阵以及窗口位置大小。
		void ViewerGetCamera(ViewerCamera& view_cam);

		// 功能：设置相机旋转。旋转中心是相机焦点（固定视点），即窗口中心。
		//       旋转轴可以在世界坐标系中定义（上帝视角），也可以在相机坐标系中定义（观察者视角）。
		// 输入：speed - 转速（deg/s）。 (axisX,axisY,axisZ) - 旋转轴。
		//       axisCoordinates - 旋转轴所属坐标系。0 - 相机坐标系，1 - 世界坐标系（坐标轴所在坐标系）。
		// 说明：相机坐标系的x、y、z轴分别是屏幕向右、向上和向外，因此可见的点云都在负z轴。
		//       旋转正向按照右手螺旋定则。因为是相机旋转，所以点云看上去是反向旋转。
		void ViewerSetRotatingCamera(double speed, double axisX, double axisY, double axisZ, int axisCoordinates = 0);

		/************************ 显示点云 ************************/
		// 功能：显示新点云或更新点云 - 无色点云。
		// 输入：id - 点云标识，正整数。不同名称的点云可同时显示，相同名称的点云被替换。
		//       pc - 点云数据。pntRadius - 点的大小（正值有效）。
		//       (pntR, pntG, pntB) - 显示的颜色（0~1）。
		// 返回值：-2 - 指令未执行。-1 - 指令执行后返回值超时。0 - 指令执行错误。1 - 执行执行成功
		//int ViewerAddPointXYZ(int id, pcl::PointCloud<pcl::PointXYZ>::Ptr pc, int pntRadius = 1,
		//	float pntR = 1.0f, float pntG = 1.0f, float pntB = 1.0f);
		int ViewerAddPointXYZ(int id, pcl::PointCloud<pcl::PointXYZ>::Ptr pc);

		// 功能：显示新点云或更新点云 - 彩色点云。
		// 输入：id - 点云标识，正整数。不同名称的点云可同时显示，相同名称的点云被替换。
		//       pc - 点云数据。pntRadius - 点的大小（正值有效）。
		// 返回值：-2 - 指令未执行。-1 - 指令执行后返回值超时。0 - 指令执行错误。1 - 执行执行成功
		//int ViewerAddPointXYZRGBA(int id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc, int pntRadius = 1);
		int ViewerAddPointXYZRGBA(int id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc);

		// 功能：显示新点云或更新点云 - 法线。
		// 输入：id - 点云标识，正整数。不同名称的点云可同时显示，相同名称的点云被替换。
		//       pc - 法线数据。
		// 返回值：-2 - 指令未执行。-1 - 指令执行后返回值超时。0 - 指令执行错误。1 - 执行执行成功
		int ViewerAddNormals(int id, pcl::PointCloud<pcl::PointNormal>::Ptr pc);

		// 功能：显示新点云或更新点云 - 曲率。
		// 输入：id - 点云标识，正整数。不同名称的点云可同时显示，相同名称的点云被替换。
		//       pc - 法线数据。curv - 曲率数据。
		// 返回值：-2 - 指令未执行。-1 - 指令执行后返回值超时。0 - 指令执行错误。1 - 执行执行成功
		int ViewerAddCurvatures(int id, pcl::PointCloud<pcl::PointNormal>::Ptr pc,
			pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curv);

		// 功能：删除指定点云或全部点云。
		// 输入：id - 点云标识。-1表示全部点云
		// 返回值：-2 - 指令未执行。-1 - 指令执行后返回值超时。0 - 指令执行错误。1 - 执行执行成功
		int ViewerRemovePointCloud(int id = -1);

		// 功能：改变点云的点的大小。切换点云大小可以产生选中或闪烁等特效。
		// 输入：id - 点云标识，正整数。不同名称的点云可同时显示，相同名称的点云被替换。
		//       pntRadius - 点的大小（正值有效）。
		// 返回值：-2 - 指令未执行。-1 - 指令执行后返回值超时。0 - 指令执行错误。1 - 执行执行成功
		int ViewerSetPointRadius(int id, int pntRadius = 1);

		// 功能：改变点云的点的颜色。切换点云颜色可以产生选中或闪烁等特效，但注意彩色点云的颜色会被覆盖。
		// 输入：id - 点云标识，正整数。不同名称的点云可同时显示，相同名称的点云被替换。
		//       (pntR, pntG, pntB) - 显示的颜色（0~1）。
		// 返回值：-2 - 指令未执行。-1 - 指令执行后返回值超时。0 - 指令执行错误。1 - 执行执行成功
		int ViewerSetPointColor(int id, float pntR = 1.0f, float pntG = 1.0f, float pntB = 1.0f);

		/************************ 测试 ************************/
		// 功能：用于测试。
		// 返回值：-2 - 指令未执行。-1 - 指令执行后返回值超时。0 - 指令执行错误。1 - 执行执行成功
		int ViewerTester(int id = -1); 

		/************************ 构造点云（静态函数） ************************/
		//// 功能：点云数组转换为PointCloud对象 - 无色点云
		//// 输入：point_cloud - 空点云指针。block - 内含点云数组。
		////       r0, nPntRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		//// 输出：point_cloud - 转换得到的PointCloud对象。
		//// 返回值：转换的点云的点数。
		//static int PointsetToPointXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, LidarDataBlock& block,
		//	int r0 = 0, int nPntRow = 0, int c0 = 0, int nCol = 0);

		//// 功能：点云数组转换为PointCloud对象 - 含光强的点云
		//// 输入：point_cloud - 空点云指针。block - 内含点云数组。
		////       r0, nPntRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		//// 输出：point_cloud - 转换得到的PointCloud对象。
		//// 返回值：转换的点云的点数。
		//static int PointsetToPointXYZI(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, LidarDataBlock& block,
		//	int r0 = 0, int nPntRow = 0, int c0 = 0, int nCol = 0);

		//// 功能：点云数组转换为PointCloud对象 - 彩色点云
		//// 输入：point_cloud - 空点云指针。block - 内含点云数组。
		////       r0, nPntRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		//// 输出：point_cloud - 转换得到的PointCloud对象。
		//// 返回值：转换的点云的点数。
		//static int PointsetToPointXYZRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, LidarDataBlock& block,
		//	int r0 = 0, int nPntRow = 0, int c0 = 0, int nCol = 0);

		//// 功能：几何点云转换为PointCloud对象 - 不含法线
		//// 输入：point_cloud - 空点云指针。block - 几何点云。minSlope - 去除倾斜度小的点。
		////       r0, nRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		//// 输出：point_cloud - 转换得到的PointCloud对象。
		//// 返回值：转换的点云的点数。
		//static int PointGeoToPointXYZI(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, PatchGeometry& block, double minSlope,
		//	int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		//// 功能：几何点云转换为PointCloud对象 - 含法线
		//// 输入：point_cloud - 空点云指针。block - 几何点云。minSlope - 去除倾斜度小的点。
		////       r0, nRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		//// 输出：point_cloud - 转换得到的PointCloud对象。
		//// 返回值：转换的点云的点数。
		//static int PointGeoToPointNormal(pcl::PointCloud<pcl::PointNormal>::Ptr& point_cloud, PatchGeometry& block, double minSlope,
		//	int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		//// 功能：几何点云转换为PointCloud对象 - 曲率
		//// 输入：point_cloud - 空点云指针。block - 几何点云。minSlope - 去除倾斜度小的点。
		////       r0, nRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		//// 输出：point_cloud - 转换得到的PointCloud对象。
		//// 返回值：转换的点云的点数。
		//static int PointGeoToCurvatures(pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr& point_cloud, PatchGeometry& block, double minSlope,
		//	int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		/************************ 读写点云（静态函数） ************************/
		// 功能：从PCD文件读取PointCloud对象
		// 输入：file_name - 文件名。point_cloud - 空点云指针。
		// 输出：point_cloud - 得到数据的PointCloud对象。
		// 返回值：0表示成功，<0表示失败。
		static int loadPCDFile(const std::string& file_name, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud);
		static int loadPCDFile(const std::string& file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud);
		static int loadPCDFile(const std::string& file_name, pcl::PointCloud<pcl::PointNormal>::Ptr& point_cloud);

		// 功能：将PointCloud对象写入PCD文件
		// 输入：file_name - 文件名。point_cloud - 待存盘的PointCloud对象。binary_mode - 用二进制格式还是ASCII格式
		// 返回值：0表示成功，<0表示失败。
		static int savePCDFile(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, bool binary_mode = false);
		static int savePCDFile(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, bool binary_mode = false);
		static int savePCDFile(const std::string& file_name, const pcl::PointCloud<pcl::PointNormal>::Ptr& point_cloud, bool binary_mode = false);

		// 功能：从PLY文件读取PointCloud对象
		// 输入：file_name - 文件名。point_cloud - 空点云指针。
		// 输出：point_cloud - 得到数据的PointCloud对象。
		// 返回值：0表示成功，<0表示失败。
		static int loadPLYFile(const std::string& file_name, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud);
		static int loadPLYFile(const std::string& file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud);
		static int loadPLYFile(const std::string& file_name, pcl::PointCloud<pcl::PointNormal>::Ptr& point_cloud);

		// 功能：将PointCloud对象写入PLY文件
		// 输入：file_name - 文件名。point_cloud - 待存盘的PointCloud对象。binary_mode - 用二进制格式还是ASCII格式
		// 返回值：0表示成功，<0表示失败。
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
