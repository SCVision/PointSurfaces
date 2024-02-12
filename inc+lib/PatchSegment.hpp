// PatchSegment.hpp
//
/********************************************************************************
* Segment 3D surfaces.
* 
* PatchSegment用于分割表面几何模型。
* 【注】本模块不需要PCL支持，但构造点云功能使用PCL数据类型PointCloud，因此需要引用PCL头文件。
* 以PCL 1.13为例，附加包含目录需要添加：
* %PCL_ROOT%\include\pcl-1.13;
* %PCL_ROOT%\3rdParty\Boost\include\boost-1_82;
* %PCL_ROOT%\3rdParty\Eigen\eigen3
* 
* 1. 使用步骤：
* ①初始化。
*   用InitializeRegions()设置表面模型数据源（PatchGeometry对象）。开始分割前数据源应完成几何计算。
* ②选取种子点。
*   种子点可以指定（例如用MATLAB查看曲率图），或者选取最小曲率点。见FindCurvxxx()函数。
* ③分割。
*   用RegionGrowing()进行一次分割。全部分割可以反复调用②③。
* ④查看分割结果。
*   用WriteRegions()保存分割矩阵，在MATLAB中用importdata(<datafn>)读取后查看。
* ⑤合并碎块。
*   点数较少的分块向位置和朝向相近的大分块合并。
*
* 2. 分块数据结构RegionsInfo
* ①RegionsInfo结构用GetDataInfo()获得。
* ②结构内含二维矩阵region[rowTotal][colTotal]，表示colTotal×rowTotal的分割矩阵。
*   每个点的内容是表面模型对应点的分块编号。有效分块编号取值范围为[1, nRegion]，nRegion是分块总数。
* ③RegionsInfo对象自动管理数据空间，不需要主动释放。
* 
*
* Writen by Lin, Jingyu, linjy02@hotmail.com, 2023.12
*
********************************************************************************/

#ifndef PatchSegment_HPP
#define PatchSegment_HPP

#include <memory> // for smart pointer
#include <vector> // for vector
//#include <Eigen/Geometry>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
//#include "../PatchGeometry/PatchGeometry.hpp"

namespace Lidar3D
{
	struct RegionFeature
	{
		double pc1, pc2;			// curvature
	};
	const int lenRegionFeature = sizeof(RegionFeature) / sizeof(double);

	struct RegionsInfo
	{
		// regions
		std::int32_t pntTotal;		// length of region: >colTotal x rowTotal. 0 - new structure
		std::int32_t colTotal;		// points in a column.
		std::int32_t rowTotal;		// points in a row.
		std::int32_t nRegion;		// number of regions.
		int* region;				// region map, size: dataTotal, dim: 2 (colTotal x rowTotal)
		PatchGeometry* source;

		// features
		std::vector<RegionFeature> feas;			// features, size: lenRegionFeature x RegionTotal
	};

	class PatchGeometry;

	class __declspec(dllexport) PatchSegment
	{
	public:
	public:
		PatchSegment();
		~PatchSegment();

		// 功能：判断对象是否初始化 。
		// 返回值：1表示已初始化，0表示未初始化。
		int isRegionsAvailable() { return data.source && data.source->isPatchAvailable() && data.pntTotal; }

		// 功能：得到数据的信息结构。
		// 返回值：数据的信息结构。
		inline const RegionsInfo& GetDataInfo() { return data; }

		// 功能：得到最大的点数。
		inline int GetPointTotal() { return data.colTotal * data.rowTotal; }

		// 功能：得到指定数据点的位置，不检查溢出。
		// 输入：col, row - 指定点所在列和行。
		// 返回值：指定数据点的序号。
		inline int GetPointPos(int col, int row) { return data.colTotal * row + col; }

		// 功能：初始化数据空间。
		// 输入：patch - 待分割的表面模型。
		// 返回值：数据空间的长度（data.pntTotal）。0表示失败。
		int InitializeRegions(PatchGeometry& patch);

		/************************ 分割算法 ************************/
		// 功能：在未分割的点中搜索满足条件的种子点。
		// 输入：无。
		// 输出：(col, row) - 种子点位置（θ、φ方向序号）。
		// 返回值：-1表示找不到有效点。
		double FindCurvMinValue(int& col, int& row);	//曲率最小
		double FindCurvMinMean(int& col, int& row);		// 邻域点曲率均值最小
		double FindCurvMinVariance(int& col, int& row);	// 邻域点曲率方差最小

		// 功能：用RegionGrowing方法进行表面模型分割。
		// 输入：thr_gap - 相对点距阈值（m）。用于检测断面。推荐0.01。
		//       thr_curv - 相对曲率阈值。用于检测拐角，取值2.0~3.0。推荐2.5。
		//       thr_smooth - 曲率平滑阈值。用于统计曲率分布，取值不大于thr_curv。推荐2.0。
		//       (seed_col, seed_row) - 种子点位置（θ、φ方向序号）。
		// 返回值：分割区域的点数。0表示失败。
		int RegionGrowing(double thr_gap, double thr_curv, double thr_smooth, int seed_col, int seed_row);

		/************************ 文件读写 ************************/
		// 功能：写入分割数据文件。数据文件可以用MATLAB函数importdata(<datafn>)读取。
		// 输入：datafn - 文件名。
		// 返回值：数据点数（data.colTotal * data.rowTotal）。0表示失败。
		int WriteRegions(const char* datafn);

		// 功能：读取分割数据文件。
		// 输入：datafn - 数据文件名。
		// 返回值：数据点数（data.colTotal * data.rowTotal）。0表示失败。
		int ReadRegions(const char* datafn);

		/************************ 构造点云（静态函数） ************************/
		// 功能：分块转换为PointCloud对象 - 不含法线。
		// 输入：point_cloud - 空点云指针。regions - 分块数据。
		//       idReg - 分块编号。minSlope - 去除倾斜度小的点。
		// 输出：point_cloud - 转换得到的PointCloud对象。
		// 返回值：得到的点云的点数。
		static int RegionToPointXYZI(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, PatchSegment& regions, int idReg, double minSlope);

		// 功能：分块转换为PointCloud对象 - 含法线。
		// 输入：point_cloud - 空点云指针。regions - 分块数据。
		//       idReg - 分块编号。minSlope - 去除倾斜度小的点。
		// 输出：point_cloud - 转换得到的PointCloud对象。
		// 返回值：得到的点云的点数。
		static int RegionToPointNormal(pcl::PointCloud<pcl::PointNormal>::Ptr& point_cloud, PatchSegment& regions, int idReg, double minSlope);

		/************************ 测试算法 ************************/
		//static int GraphCut(PatchGeometry& patch, std::vector<Eigen::Vector2i>& dir_obj, std::vector<Eigen::Vector2i>& dir_bkg,
		//	double n0_obj, double r0_obj, double r0_bkg, double lambda_curv, double miu, PatchGeometry& obj, PatchGeometry& bkg);

	private:
		// 数据信息，可以用GetDataInfo()获得，禁止修改。
		RegionsInfo data;
	};
}
#endif // #ifndef PatchSegment_HPP
