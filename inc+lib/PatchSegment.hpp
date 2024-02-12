// PatchSegment.hpp
//
/********************************************************************************
* Segment 3D surfaces.
* 
* PatchSegment���ڷָ���漸��ģ�͡�
* ��ע����ģ�鲻��ҪPCL֧�֣���������ƹ���ʹ��PCL��������PointCloud�������Ҫ����PCLͷ�ļ���
* ��PCL 1.13Ϊ�������Ӱ���Ŀ¼��Ҫ��ӣ�
* %PCL_ROOT%\include\pcl-1.13;
* %PCL_ROOT%\3rdParty\Boost\include\boost-1_82;
* %PCL_ROOT%\3rdParty\Eigen\eigen3
* 
* 1. ʹ�ò��裺
* �ٳ�ʼ����
*   ��InitializeRegions()���ñ���ģ������Դ��PatchGeometry���󣩡���ʼ�ָ�ǰ����ԴӦ��ɼ��μ��㡣
* ��ѡȡ���ӵ㡣
*   ���ӵ����ָ����������MATLAB�鿴����ͼ��������ѡȡ��С���ʵ㡣��FindCurvxxx()������
* �۷ָ
*   ��RegionGrowing()����һ�ηָȫ���ָ���Է������âڢۡ�
* �ܲ鿴�ָ�����
*   ��WriteRegions()����ָ������MATLAB����importdata(<datafn>)��ȡ��鿴��
* �ݺϲ���顣
*   �������ٵķֿ���λ�úͳ�������Ĵ�ֿ�ϲ���
*
* 2. �ֿ����ݽṹRegionsInfo
* ��RegionsInfo�ṹ��GetDataInfo()��á�
* �ڽṹ�ں���ά����region[rowTotal][colTotal]����ʾcolTotal��rowTotal�ķָ����
*   ÿ����������Ǳ���ģ�Ͷ�Ӧ��ķֿ��š���Ч�ֿ���ȡֵ��ΧΪ[1, nRegion]��nRegion�Ƿֿ�������
* ��RegionsInfo�����Զ��������ݿռ䣬����Ҫ�����ͷš�
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

		// ���ܣ��ж϶����Ƿ��ʼ�� ��
		// ����ֵ��1��ʾ�ѳ�ʼ����0��ʾδ��ʼ����
		int isRegionsAvailable() { return data.source && data.source->isPatchAvailable() && data.pntTotal; }

		// ���ܣ��õ����ݵ���Ϣ�ṹ��
		// ����ֵ�����ݵ���Ϣ�ṹ��
		inline const RegionsInfo& GetDataInfo() { return data; }

		// ���ܣ��õ����ĵ�����
		inline int GetPointTotal() { return data.colTotal * data.rowTotal; }

		// ���ܣ��õ�ָ�����ݵ��λ�ã�����������
		// ���룺col, row - ָ���������к��С�
		// ����ֵ��ָ�����ݵ����š�
		inline int GetPointPos(int col, int row) { return data.colTotal * row + col; }

		// ���ܣ���ʼ�����ݿռ䡣
		// ���룺patch - ���ָ�ı���ģ�͡�
		// ����ֵ�����ݿռ�ĳ��ȣ�data.pntTotal����0��ʾʧ�ܡ�
		int InitializeRegions(PatchGeometry& patch);

		/************************ �ָ��㷨 ************************/
		// ���ܣ���δ�ָ�ĵ��������������������ӵ㡣
		// ���룺�ޡ�
		// �����(col, row) - ���ӵ�λ�ã��ȡ��շ�����ţ���
		// ����ֵ��-1��ʾ�Ҳ�����Ч�㡣
		double FindCurvMinValue(int& col, int& row);	//������С
		double FindCurvMinMean(int& col, int& row);		// ��������ʾ�ֵ��С
		double FindCurvMinVariance(int& col, int& row);	// ��������ʷ�����С

		// ���ܣ���RegionGrowing�������б���ģ�ͷָ
		// ���룺thr_gap - ��Ե����ֵ��m�������ڼ����档�Ƽ�0.01��
		//       thr_curv - ���������ֵ�����ڼ��սǣ�ȡֵ2.0~3.0���Ƽ�2.5��
		//       thr_smooth - ����ƽ����ֵ������ͳ�����ʷֲ���ȡֵ������thr_curv���Ƽ�2.0��
		//       (seed_col, seed_row) - ���ӵ�λ�ã��ȡ��շ�����ţ���
		// ����ֵ���ָ�����ĵ�����0��ʾʧ�ܡ�
		int RegionGrowing(double thr_gap, double thr_curv, double thr_smooth, int seed_col, int seed_row);

		/************************ �ļ���д ************************/
		// ���ܣ�д��ָ������ļ��������ļ�������MATLAB����importdata(<datafn>)��ȡ��
		// ���룺datafn - �ļ�����
		// ����ֵ�����ݵ�����data.colTotal * data.rowTotal����0��ʾʧ�ܡ�
		int WriteRegions(const char* datafn);

		// ���ܣ���ȡ�ָ������ļ���
		// ���룺datafn - �����ļ�����
		// ����ֵ�����ݵ�����data.colTotal * data.rowTotal����0��ʾʧ�ܡ�
		int ReadRegions(const char* datafn);

		/************************ ������ƣ���̬������ ************************/
		// ���ܣ��ֿ�ת��ΪPointCloud���� - �������ߡ�
		// ���룺point_cloud - �յ���ָ�롣regions - �ֿ����ݡ�
		//       idReg - �ֿ��š�minSlope - ȥ����б��С�ĵ㡣
		// �����point_cloud - ת���õ���PointCloud����
		// ����ֵ���õ��ĵ��Ƶĵ�����
		static int RegionToPointXYZI(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, PatchSegment& regions, int idReg, double minSlope);

		// ���ܣ��ֿ�ת��ΪPointCloud���� - �����ߡ�
		// ���룺point_cloud - �յ���ָ�롣regions - �ֿ����ݡ�
		//       idReg - �ֿ��š�minSlope - ȥ����б��С�ĵ㡣
		// �����point_cloud - ת���õ���PointCloud����
		// ����ֵ���õ��ĵ��Ƶĵ�����
		static int RegionToPointNormal(pcl::PointCloud<pcl::PointNormal>::Ptr& point_cloud, PatchSegment& regions, int idReg, double minSlope);

		/************************ �����㷨 ************************/
		//static int GraphCut(PatchGeometry& patch, std::vector<Eigen::Vector2i>& dir_obj, std::vector<Eigen::Vector2i>& dir_bkg,
		//	double n0_obj, double r0_obj, double r0_bkg, double lambda_curv, double miu, PatchGeometry& obj, PatchGeometry& bkg);

	private:
		// ������Ϣ��������GetDataInfo()��ã���ֹ�޸ġ�
		RegionsInfo data;
	};
}
#endif // #ifndef PatchSegment_HPP
