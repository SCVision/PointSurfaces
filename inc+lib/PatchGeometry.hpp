// PatchGeometry.hpp
//
/********************************************************************************
* Geometry on surfaces in raw 3D data.
* 
* PatchGeometry用于对三维扫描数据进行表面建模和几何计算。
* 【注】本模块不需要PCL支持，但构造点云功能使用PCL数据类型PointCloud，因此需要引用PCL头文件。
* 以PCL 1.13为例，附加包含目录需要添加：
* %PCL_ROOT%\include\pcl-1.13;
* %PCL_ROOT%\3rdParty\Boost\include\boost-1_82;
* %PCL_ROOT%\3rdParty\Eigen\eigen3
* 
* 1. 使用步骤：
* ①表面建模。有两种方式：
* 方式1：用InitializeData()设置重采样的角度，然后用PatchRemodeling()从数据源（LidarDataBlock对象）重采样建立表面模型。
* 【注】重采样的行列角度必须是递增的。
* 方式2：用InitializeModel()直接用数据源（LidarDataBlock对象）建立表面模型，不进行重采样。
* 【注】行角度自动调整为递增的。。
* ②计算几何量。在表面建模的基础上，用CalculateGeometry()计算法线、曲率等几何量。
*
* 2. 表面几何数据结构PatchDataInfo
* ①PatchDataInfo结构用GetDataInfo()获得。
* ②结构内含三维矩阵datablock[rowTotal][colTotal][datumSize]，表示colTotal×rowTotal的采样点矩阵，每个数据点长度datumSize。有效数据点数为nPnts。
*   基本点的结构为PatchPoint_basic，完整点的结构为PatchPoint_geo，相应的datumSize为lenPatchPoint_basic和lenPatchPoint_geo。
*   每个点的内容包括采样点的位置、点密度、倾斜度、切线、法线、主曲率、主曲率切矢量等。
* ③PatchGeometry对象自动管理数据空间，不需要主动释放。
* 
* 3. 数据读写
* ①用ReadPointGeo()和WritePointGeo()读写表面几何数据文件（GEO文件）。
* ②数据文件的格式如下。
*   标题行：<datumSize>, <colTotal>, <rowTotal>, angleCol[0], ..., angleCol[N], 0, 0, ...
*   第m+1行：angleRow[m],  datablock[m][0], datablock[m][1], ..., datablock[m][N]
*
* Writen by Lin, Jingyu, linjy02@hotmail.com, 2023.09
*
********************************************************************************/

#ifndef PatchGeometry_HPP
#define PatchGeometry_HPP

//#include <memory> // for smart pointer
//#include <cstdint>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
//#include "..\LidarDataContainer\LidarDataBlock.hpp"

namespace Lidar3D
{
	// 表面点矩阵
	struct PatchDataInfo
	{
		// data info
		std::int32_t dataTotal;		// length of data buffer: >datumSize x colTotal x rowTotal. 0 - new structure
		std::int32_t datumSize;		// length of one data point. 
									// refer to struct PatchPoint_basic & PatchPoint_geo
		std::int32_t colTotal;		// maximum points in a column.
		std::int32_t rowTotal;		// maximum points in a row.
		std::int32_t modelState;	// 0 - new, 1 - initialized, 2 - patch model, 3 - normal & curvature
		std::int32_t nPnts;			// number of available points.

		// data buffer
		double* angleCol;   // column angles (deg), length: colTotal
		double* angleRow;   // row angles (deg), length: rowTotal
		double* datablock;	// data buffer, size: dataTotal, dim: 3 (datumSize x colTotal x rowTotal)
							// refer to struct LidarPointXYZ & LidarPointGeo
	};

	// 表面点的结构 - 简单
	struct PatchPoint_basic
	{
		// points
		double distance;				// distance 
		double intensity, brightness;	// distance & intensity of reflected light
		double p_x, p_y, p_z;			// xyz coordinates
		double density;                 // density
		double slope;					// sine of the angle between surface and laser beam (0~1)
	};
	const int lenPatchPoint_basic = sizeof(PatchPoint_basic) / sizeof(double);

	// 表面点的结构 - 含几何
	struct PatchPoint_geo
	{
		// points
		double distance;				// distance 
		double intensity, brightness;	// intensity & brightness (0.0~1.0) of reflected light
		double p_x, p_y, p_z;			// xyz coordinates
		double density;                 // density
		double slope;					// sine of the angle between surface and laser beam (0~1). 0.0 = invalid point

		// geometry
		double n_x, n_y, n_z;			// unit normal vector (t1 x t2 / |t1 x t2|)
		double pc1, pc2;				// principle curvature max & min. |pc1| >= |pc2|
		double pc_x, pc_y, pc_z;		// direction corresponding to pc1 (unit vector)
		double h11, h12, h22;			// the second fundamental form
		double belief;					// belief of normal and curvature (0~1). 0.0 = invalid normal

		// reversed
		double dx_col, dy_col, dz_col;		// tangent vector (Dp_x/Dcol, Dp_y/Dcol, Dp_z/Dcol)
		double dx_row, dy_row, dz_row;		// tangent vector (Dp_x/Drow, Dp_y/Drow, Dp_z/Drow)
	};
	const int lenPatchPoint_geo = sizeof(PatchPoint_geo) / sizeof(double);

	class LidarDataBlock;

	class __declspec(dllexport) PatchGeometry
	{
	public:
		PatchGeometry();
		~PatchGeometry();

		// 功能：判断对象是否初始化 。0表示未初始化。
		// 返回值：数据空间的大小（GetDataInfo().dataTotal）。
		inline int isPatchAvailable() { return data.dataTotal; }

		// 功能：得到建模状态。
		// 返回值：0 - 新对象，1 - 已初始化，2 - 已建立表面模型, 3 - 已计算法线曲率
		inline int GetModelState() { return data.modelState; }

		// 功能：得到最大的点数。
		inline int GetPointTotal() { return data.colTotal * data.rowTotal; }

		// 功能：得到数据的信息结构。
		// 返回值：数据的信息结构。
		inline const PatchDataInfo& GetDataInfo() { return data; }

		// 功能：得到指定数据点的位置，不检查溢出。数据起始地址为GetDataInfo().datablock。
		// 输入：col, row - 指定点所在列和行。
		// 返回值：指定数据点在数组的起始位置。
		inline int GetDataPos(int col, int row) { return data.datumSize * (data.colTotal * row + col); }

		/************************ 访问管理 ************************/
		// 功能：初始化数据空间，之后用PatchRemodeling()建立表面模型。
		// 输入：angle_row[row_total] - 行角度，即一行的每各点对应的角度。角度必须递增。
		//       angle_col[col_total] - 列角度，即一列的每各点对应的角度。角度必须递增。
		//		 datum_size - 每个点的长度。
		// 返回值：数据缓冲区的长度（data.dataTotal）。0表示失败。成功后modelState为1。
		// 注意：行角度和列角度必须递增。
		int InitializeData(double* angle_row, int row_total, double* angle_col, int col_total, int datum_size = lenPatchPoint_geo);

		// 功能：初始化数据空间（行角度和列角度是等角间隔的），之后用PatchRemodeling()建立表面模型。
		// 输入：angle_col0:dAngle_col:angle_col1, angle_row0:dAngle_row:angle_row1 - 行、列角度。dAngle_col、dAngle_row必须为正数，角度递增。
		//		 datum_size - 每个点的长度，缺省值为lenLidarPointGeo，最小值为lenLidarPointXYZ。
		// 返回值：数据缓冲区的长度（data.dataTotal）。0表示失败。成功后modelState为1。
		// 注意：行角度和列角度必须递增。
		int InitializeData(double dAngle_row, double angle_row0,  double angle_row1, double dAngle_col, double angle_col0, double angle_col1, int datum_size = lenPatchPoint_geo);

		// 功能：按照初始化的角度重采样建立表面模型。
		// 输入：src - 包含点云数组的数据源。
		//		 angle_gau - 参数：高斯核函数宽度（deg）。建议用1.5~2倍扫描步长。
		//		 search_method - 表面点搜索方法，取值1、2、3、4。推荐取4。
		//                       方法1检测采样点，很慢但结果准确，方法2、3是爬山法，方法4是方法2的改进版，速度较快。
		//		 obj_func - 表面点目标函数，取值0、1、2，表示补偿距离次数。推荐取2。
		//		 contra - 对比度。对包含反射光强的数据有效，对比度越大则点云亮度越暗、纹理越清晰。
		// 返回值：有效点数。0表示建模失败。成功后modelState为2。
		// 注意：1. 数据源src的行角度和列角度必须递增或递减，不能来回扫描。
		//       2. 数据源src必须已经调用Lidardata2XYZI()转换为点云数组。
		int PatchRemodeling(LidarDataBlock& src, double angle_gau, int search_method = 4, int obj_func = 2, double contra = 1.0);

		// 功能：直接用数据源建立表面模型，不需要InitializeData()+PatchRemodeling()。
		// 输入：src - 包含点云数组的数据源。
		// 返回值：有效点数。0表示建模失败。成功后modelState为2。
		// 注意：1. 数据源src的行角度必须递增或递减，建模后是递增的。列角度必须递增。
		//		 2. 数据源src必须已经调用Lidardata2XYZI()转换为点云数组。
		int InitializeModel(LidarDataBlock& src);

		// 功能：计算表面法线和曲率，成功后modelState为3。必须先建立表面模型。
		// 输入：min_nbrSpace - 邻域点最小距离（m）。该值与max_nbrAngle定义天顶纬度。。
		//		 max_nbrAngle - 邻域点最大角间隔（deg）。该值与min_nbrSpace定义天顶纬度。
		void CalculateGeometry(double min_nbrSpace = 0.001, double max_nbrAngle = 5.73);

		// 功能：清除一个几何点（将属性slope和belief置零）。
		// 输入：col, row - 指定点所在列和行。不检测有效性。
		void ClearOnePoint(int col, int row);

		// 功能：克隆表面模型。原有的数据被删除。
		// 输入：src - 源对象。
		//       r0, nRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		// 返回值：克隆的数据的点数。
		int CloneFrom(PatchGeometry& src, int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		// 功能：调整点云的亮度。
		// 输入：contra - 对比度。对包含反射光强的数据有效，对比度越大则点云亮度越暗、纹理越清晰。
		// 返回值：被调整的点数。
		void TuneContrast(double contra);

		/************************ 文件读写 ************************/
		// 功能：写入geo数据文件。根据属性modelState为2或3决定是否写入几何信息。
		// 输入：datafn - 文件名。
		//       r0, nRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		// 返回值：写入数据的行数（文本文件行数）。
		// 说明：数据文件的格式如下。
		//       标题行：<datumSize>, <colTotal>, <rowTotal>, angleCol[0], ..., angleCol[N-1], 0, 0, ...
		//		 第m+1行：angleRow[m],  datablock[m][0], datablock[m][1], ..., datablock[m][N-1]
		//       数据文件可以用MATLAB函数importdata(<datafn>)读取。
		int WritePatchGeo(const char* datafn, int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		// 功能：读取geo数据文件。原数据被删除。
		// 输入：datafn - 数据文件名。
		// 返回值：实际读取数据的行数（data.nRow）。0表示失败。
		// 说明：根据数据文件是否含几何信息，属性modelState设置为2或3。
		int ReadPatchGeo(const char* datafn);

		/************************ 构造点云（静态函数） ************************/
		// 功能：几何点云转换为PointCloud对象 - 不含法线
		// 输入：point_cloud - 空点云指针。block - 几何点云。minSlope - 去除倾斜度小的点。
		//       r0, nRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		// 输出：point_cloud - 转换得到的PointCloud对象。
		// 返回值：转换的点云的点数。
		static int PointGeoToPointXYZI(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, PatchGeometry& block, double minSlope,
			int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		// 功能：几何点云转换为PointCloud对象 - 含法线
		// 输入：point_cloud - 空点云指针。block - 几何点云。minSlope - 去除倾斜度小的点。
		//       r0, nRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		// 输出：point_cloud - 转换得到的PointCloud对象。
		// 返回值：转换的点云的点数。
		static int PointGeoToPointNormal(pcl::PointCloud<pcl::PointNormal>::Ptr& point_cloud, PatchGeometry& block, double minSlope,
			int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		// 功能：几何点云转换为PointCloud对象 - 曲率
		// 输入：point_cloud - 空点云指针。block - 几何点云。minSlope - 去除倾斜度小的点。
		//       r0, nRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		// 输出：point_cloud - 转换得到的PointCloud对象。
		// 返回值：转换的点云的点数。
		static int PointGeoToCurvatures(pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr& point_cloud, PatchGeometry& block, double minSlope,
			int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

	private:
		// 数据信息，可以用GetDataInfo()获得，禁止修改。
		PatchDataInfo data;      // surface data
	};
}
#endif // #ifndef PatchGeometry_HPP
