// LidarDataBlock.hpp
//
/********************************************************************************
* Data structure for raw 3D data.
* 
* LidarDataBlock用于管理实时三维扫描数据。
* 【注】本模块不需要PCL支持，但构造点云功能使用PCL数据类型PointCloud，因此需要引用PCL头文件。
* 以PCL 1.13为例，附加包含目录需要添加：
* %PCL_ROOT%\include\pcl-1.13;
* %PCL_ROOT%\3rdParty\Boost\include\boost-1_82;
* %PCL_ROOT%\3rdParty\Eigen\eigen3
*
* 1. 三维扫描数据结构LidarDataInfo
* ①LidarDataInfo结构用GetDataInfo()获得，包含三维扫描数据和点云数组。
* ②三维扫描数据是三维矩阵，行数、列数、每个数据点的数据量分别为rowTotal、colTotal、datumSize，实际有效数据行数为nRow。
*   数据保存在数组datablock[dataTotal]中，数组可容纳的数据量dataTotal≥datumSize×colTotal×rowTotal。
*   目前datumSize仅支持取值1或2，第一个数据是物体距离d，第二个数据是反射光强i。
*   各行和各列对应的角度分别为angleRow[rowTotal]和angleCol[colTotal]。每行还有一个数据时戳tsRow[rowTotal]。
* ③点云数组是三维矩阵，行数、列数、每个数据点的数据量分别为rowTotal、colTotal、pntDatumSize，有效点云行数为nPntRow，有效点数为nPnts。
*   点云保存在数组points[pntDataTotal]中，数组可容纳的数据量pntDataTotal≥pntDatumSize×colTotal×rowTotal。
*   点云点的结构为LidarPoint_xyzi，相应的pntDatumSize为lenLidarPoint_xyzi。
* ④LidarDataBlock对象自动管理数据空间，不需要主动释放。
* 
* 2. 输入三维扫描数据有三种方式：
* ①用InitializeBlock()初始化后用AddRow()逐行输入。注意用SetMechanic()设置机械参数，缺省参数为全0。
* ②用ReadLidardata()从数据文件输入。数据的大小和机械参数由数据文件定义。
* ③用CloneFrom()从其它LidarDataBlock对象克隆。
*
* 3. 数据读写
* ①ReadLidardata()和WriteLidardata()用于读写三维扫描数据文件（L3D文件）。
* ②WritePoints()用于将点云数组保存为文本文件。
* 
* 4. 点云转换
* ①三维扫描数据要用Lidardata2XYZI()转换为点云数组，之后才能转换为PointCloud对象。
* ②对于包含反射光强的三维扫描数据，所转换的点云数组可以设置对比度。对比度越大则点云亮度越暗、但纹理越清晰。
* ③转换为点云数组前要用SetMechanics()设置机械参数，缺省参数为全0。设置参数后已转换的点云失效，需要再调用Lidardata2XYZI()转换。
*
* 5. 构造PointCloud对象
* 用PointsetToPointXYZ()、PointsetToPointXYZI()、PointsetToPointXYZRGBA生成PointCloud对象。

* Writen by Lin, Jingyu, linjy02@hotmail.com, 2021.05
* Revised by Lin, Jingyu, linjy02@hotmail.com, 2022.03
* Revised by Lin, Jingyu, linjy02@hotmail.com, 2023.05
* Revised by Lin, Jingyu, linjy02@hotmail.com, 2023.07
*
********************************************************************************/

#ifndef LidarDataBlock_HPP
#define LidarDataBlock_HPP

//#include <memory> // for smart pointer
//#include <cstdint>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

namespace Lidar3D
{
	// 管理三维扫描原始数据和点云数组的数据结构
	struct LidarDataInfo
	{
		// LiDAR data info
		std::int32_t lidarType;	// type of LiDAR (enum LiDAR_Type).
		std::int32_t dataTotal;	// total length of data buffer: >datumSize x colTotal x rowTotal. 0 - new structure
		std::int32_t datumSize;	// length of one data point. refer to struct LidarData_xxx
		std::int32_t colTotal;		// maximum points in a column.
		std::int32_t rowTotal;		// maximum points in a row.
		std::int32_t nRow;			// number of available data rows. 

		// LiDAR data buffer
		double* angleCol;   // column angles (deg), length: colTotal
		double* angleRow;   // row angles (deg), length: rowTotal
		double* tsRow;      // time stamp of each row (ms), length: rowTotal
		double* datablock;    // LiDAR data buffer, size: dataTotal, dim: 3 (datumSize x colTotal x rowTotal). refer to struct LidarPoint_xyzi

		// lidar mechanical parameters for points transforming
		// [0] - paraLa, [1] - paraLx (m)
		// [2] - paraDPsi, [3] - paraDtheta, [4] - paraDgamma (deg)
		// [5] - maxDist (m)
		// [6] - minDTs (ms), 0 for no shift
		double lidar_params[8];

		// point cloud info and buffer
		std::int32_t pntDataTotal;	// total length of poin cloud buffer: >pntDatumSize x colTotal x rowTotal. 0 - new structure
		std::int32_t pntDatumSize;	// length of one point (>=3). refer to struct LidarPoint_xxx
		std::int32_t nPnts;			// number of available points.
		std::int32_t nPntRow;		// number of available point rows. 
		double* points;				// point cloud buffer, size: pntDataTotal, dim: 3 (pntDatumSize x colTotal x rowTotal). refer to struct LidarPoint_xyzi

		// private data for speeding up calculation, initialized by InitializeBlock()
		double* sin_theta;		// buffer of sin(column angles), length: colTotal
		double* cos_theta;		// buffer of cos(column angles), length: colTotal
		double colRes;			// reciprocal of total number of steps in a column round 
	};

	// 三维扫描系统类型
	enum LiDAR_Type  // type of LiDAR
	{
		LiDAR_Type_none = 0,    // no data 
		LiDAR_Type_3D = 1,      // 3D LiDAR
		LiDAR_Type_multiecho,   // multi-echo LiDAR
		LiDAR_Type_RGB,         // 3D LiDAR with RGB camera
		LiDAR_Type_4D,          // 3D LiDAR on rotatory stage
		LiDAR_Type_resampled,   // resampled data
		LiDAR_Type_end	        // unknown 
	};

	// 原始数据点的结构
	struct LidarData_basic
	{
		double distance;			// distance
		union
		{
			double intensity;			// intensity of reflected light
			union
			{
				std::uint32_t rgba;		// rgb color
				struct
				{
					std::uint8_t b;
					std::uint8_t g;
					std::uint8_t r;
					std::uint8_t a;
				};
			};
		};
	};
	const int lenLidarData_basic = sizeof(LidarData_basic) / sizeof(double);

	// 点云点的结构
	struct LidarPoint_xyzi
	{
		union
		{
			double intensity;			// intensity of reflected light
			union
			{
				std::uint32_t rgba;		// rgb color
				struct
				{
					std::uint8_t b;
					std::uint8_t g;
					std::uint8_t r;
					std::uint8_t a;
				};
			};
		};

		double brightness;		// transform from intensity of reflected light
		double p_x, p_y, p_z;	// xyz coordinates
		double p_dist;			// sqrt(x*x+y*y+z*z). 0 = invalid point
	};
	const int lenLidarPoint_xyzi = sizeof(LidarPoint_xyzi) / sizeof(double);

	class __declspec(dllexport) LidarDataBlock
	{
	public:
		LidarDataBlock();
		~LidarDataBlock();

		// 功能：判断对象是否初始化 。0表示未初始化。
		// 返回值：三维扫描数据空间大小（GetDataInfo().dataTotal）。
		inline int isBlockAvailable() { return data.dataTotal; }

		// 功能：判断是否收到三维扫描数据。0表示无数据。
		// 返回值：三维扫描数据的实际行数（GetDataInfo().nRow）。
		inline int isDataAvailable() { return data.nRow; }

		// 功能：得到有效的点云点数。
		inline int GetPointTotal() { return data.colTotal * data.nPntRow; }

		// 功能：得到可容纳的点云点数。
		inline int GetPointMax() { return data.colTotal * data.rowTotal; }

		// 功能：得到包含三维扫描数据信息的结构。
		// 返回值：包含三维扫描数据信息的结构（属性data）。
		inline const LidarDataInfo& GetDataInfo() { return data; }

		// 功能：得到指定数据点的位置。数据起始地址为GetDataInfo().datablock。
		// 输入：col, row - 指定点所在列和行。不检测有效性。
		// 返回值：指定数据点在数组的起始位置。
		inline int GetDataPos(int col, int row) { return data.datumSize * (data.colTotal * row + col); }

		// 功能：得到指定点云点的位置。数据起始地址为GetDataInfo().points。
		// 输入：col, row - 指定点所在列和行。
		// 返回值：指定数据点在数组的起始位置。
		inline int GetPointPos(int col, int row) { return data.pntDatumSize * (data.colTotal * row + col); }

		/************************ 数据管理 ************************/
		// 功能：初始化数据空间并填充列角度。
		// 输入：datum_size, col_total, row_total - 数据空间的大小。
		//       angle_col[col_total] - 列角度，即一行的每各点对应的角度。如果angle_col为NULL则保留原角度数组，首次初始化不能为NULL。
		//       pnt_datum_size - 与col_total、row_total定义点云空间的大小。
		//       lidar_type - 三维扫描系统类型，默认为LiDAR_Type_3D
		// 返回值：三维扫描数据缓冲区的长度（data.dataTotal）。0表示失败。
		int InitializeBlock(int datum_size, int col_total, int row_total, double* angle_col, int pnt_datum_size = lenLidarPoint_xyzi, int lidar_type = LiDAR_Type_3D);

		// 功能：清空数据。不改变参数和数据空间。
		inline void ClearData() { data.nRow = data.nPnts = data.nPntRow = 0; }

		// 功能：仅清空点云。不改变参数和数据空间。
		inline void ClearPoints() { data.nPnts = data.nPntRow = 0; }

		// 功能：清除一个点云点。
		// 输入：col, row - 指定点所在列和行。不检测有效性。
		void ClearOnePoint(int col, int row);

		// 功能：输入一行三维扫描数据。
		// 输入：src[nSrc] - 新数据行。angleRow - 数据行对应的角度。timeStamp - 数据行对应的时戳。
		// 返回值：插入后数据的行数（data.nRow）。0表示未分配空间或数据满。
		// 注意：新数据行长度nSrc应不小于data.datumSize*data.colTotal。
		int AddRow(double* src, int nSrc, double angleRow, double timeStamp);

		// 功能：克隆三维扫描数据和点云。原数据被删除。
		// 输入：src - 源LidarDataBlock对象。
		//       r0, nRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		// 返回值：克隆的数据的点数。
		int CloneFrom(LidarDataBlock& src, int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		/************************ 数据转换 ************************/
		// 功能：设置三维扫描系统的机械参数，用于点云转换。参见LidarDataInfo结构的lidar_params[8]。
		// 输入：lidar_params[8] - 三维扫描系统的机械参数。
		void SetMechanics(double params[8]);

		// 功能：读取三维扫描系统的机械参数。
		inline const double* GetMechanics();

		// 功能：将新增的三维扫描数据实时转换为XYZI点云数组。
		// 输入：contra - 对比度。对于包含反射光强的三维扫描数据，对比度越大则点云亮度越暗、纹理越清晰。
		//		 correct - 1表示根据时戳对行角度进行校准；0表示直接转换，不校准行角度。
		// 返回值：转换的点云的点数。
		// 说明：仅转换新增点云。如果需要重新转换全部数据，则先调用ClearPoints()，再进行转换。
		int Lidardata2XYZI(double contra = 1.0, int correct = 1);

		// 功能：调整已用Lidardata2XYZI()转换的点云的对比度。
		// 输入：contra - 对比度。对于包含反射光强的三维扫描数据，对比度越大则点云亮度越暗、纹理越清晰。
		// 返回值：被调整的点数。
		int TuneContrast(double contra);

		/************************ 构造点云（PointCloud对象） ************************/
		// 功能：点云数组转换为PointCloud对象 - 无色点云
		// 输入：point_cloud - 空点云指针。block - 内含点云数组。
		//       r0, nPntRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		// 输出：point_cloud - 转换得到的PointCloud对象。
		// 返回值：转换的点云的点数。
		static int PointsetToPointXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, LidarDataBlock& block,
			int r0 = 0, int nPntRow = 0, int c0 = 0, int nCol = 0);

		// 功能：点云数组转换为PointCloud对象 - 含光强的点云
		// 输入：point_cloud - 空点云指针。block - 内含点云数组。
		//       r0, nPntRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		// 输出：point_cloud - 转换得到的PointCloud对象。
		// 返回值：转换的点云的点数。
		static int PointsetToPointXYZI(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, LidarDataBlock& block,
			int r0 = 0, int nPntRow = 0, int c0 = 0, int nCol = 0);

		// 功能：点云数组转换为PointCloud对象 - 彩色点云
		// 输入：point_cloud - 空点云指针。block - 内含点云数组。
		//       r0, nPntRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		// 输出：point_cloud - 转换得到的PointCloud对象。
		// 返回值：转换的点云的点数。
		static int PointsetToPointXYZRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, LidarDataBlock& block,
			int r0 = 0, int nPntRow = 0, int c0 = 0, int nCol = 0);

		/************************ 文件读写 ************************/
		// 功能：从数据文件读取三维扫描数据。
		// 输入：datafn - 数据文件名。
		// 返回值：实际读取数据的行数（data.nRow）。0表示失败。
		int ReadLidardata(const char* datafn);

		// 功能：将三维扫描数据写入文件。数据文件可以用MATLAB函数importdata(<datafn>)读取。
		// 输入：datafn - 文件名。
		//       r0, nRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		// 返回值：写入数据的行数（文本文件行数）。
		// 说明：数据文件的格式如下。如果标题行以0，0，0开头，则datumSize=1，colTotal和rowTotal读取文件获得。
		//       标题行：<datumSize>, <colTotal>, <rowTotal>, angleCol[0], ..., angleCol[N-1], 8个参数, 0, 0, ...
		//       第m+1行：m+1, tsRow[m], angleRow[m],  datablock[m][0], datablock[m][1], ...
		int WriteLidardata(const char* datafn, int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		// 功能：将点云数据写入文本文件。数据文件可以用MATLAB函数importdata(<datafn>)读取。
		// 输入：datafn - 文件名。
		//       r0, nRow - 裁剪参数：起始行和行数。c0, nCol - 裁剪参数：起始列和列数。
		// 返回值：写入点云的点数（文本文件行数）。
		int WritePoints(const char* datafn, int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

	private:
		// 数据信息，可以用GetDataInfo()获得，禁止修改。
		LidarDataInfo data;      // raw lidar data and point cloud
	};
}
#endif // #ifndef LidarDataBlock_HPP
