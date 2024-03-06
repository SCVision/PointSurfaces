// PatchGeometry.hpp
//
/********************************************************************************
* Geometry on surfaces in raw 3D data.
* 
* PatchGeometry���ڶ���άɨ�����ݽ��б��潨ģ�ͼ��μ��㡣
* ��ע����ģ�鲻��ҪPCL֧�֣���������ƹ���ʹ��PCL��������PointCloud�������Ҫ����PCLͷ�ļ���
* ��PCL 1.13Ϊ�������Ӱ���Ŀ¼��Ҫ��ӣ�
* %PCL_ROOT%\include\pcl-1.13;
* %PCL_ROOT%\3rdParty\Boost\include\boost-1_82;
* %PCL_ROOT%\3rdParty\Eigen\eigen3
* 
* 1. ʹ�ò��裺
* �ٱ��潨ģ�������ַ�ʽ��
* ��ʽ1����InitializeData()�����ز����ĽǶȣ�Ȼ����PatchRemodeling()������Դ��LidarDataBlock�����ز�����������ģ�͡�
* ��ע���ز��������нǶȱ����ǵ����ġ�
* ��ʽ2����InitializeModel()ֱ��������Դ��LidarDataBlock���󣩽�������ģ�ͣ��������ز�����
* ��ע���нǶ��Զ�����Ϊ�����ġ���
* �ڼ��㼸�������ڱ��潨ģ�Ļ����ϣ���CalculateGeometry()���㷨�ߡ����ʵȼ�������
*
* 2. ���漸�����ݽṹPatchDataInfo
* ��PatchDataInfo�ṹ��GetDataInfo()��á�
* �ڽṹ�ں���ά����datablock[rowTotal][colTotal][datumSize]����ʾcolTotal��rowTotal�Ĳ��������ÿ�����ݵ㳤��datumSize����Ч���ݵ���ΪnPnts��
*   ������ĽṹΪPatchPoint_basic��������ĽṹΪPatchPoint_geo����Ӧ��datumSizeΪlenPatchPoint_basic��lenPatchPoint_geo��
*   ÿ��������ݰ����������λ�á����ܶȡ���б�ȡ����ߡ����ߡ������ʡ���������ʸ���ȡ�
* ��PatchGeometry�����Զ��������ݿռ䣬����Ҫ�����ͷš�
* 
* 3. ���ݶ�д
* ����ReadPointGeo()��WritePointGeo()��д���漸�������ļ���GEO�ļ�����
* �������ļ��ĸ�ʽ���¡�
*   �����У�<datumSize>, <colTotal>, <rowTotal>, angleCol[0], ..., angleCol[N], 0, 0, ...
*   ��m+1�У�angleRow[m],  datablock[m][0], datablock[m][1], ..., datablock[m][N]
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
	// ��������
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

	// �����Ľṹ - ��
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

	// �����Ľṹ - ������
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

		// ���ܣ��ж϶����Ƿ��ʼ�� ��0��ʾδ��ʼ����
		// ����ֵ�����ݿռ�Ĵ�С��GetDataInfo().dataTotal����
		inline int isPatchAvailable() { return data.dataTotal; }

		// ���ܣ��õ���ģ״̬��
		// ����ֵ��0 - �¶���1 - �ѳ�ʼ����2 - �ѽ�������ģ��, 3 - �Ѽ��㷨������
		inline int GetModelState() { return data.modelState; }

		// ���ܣ��õ����ĵ�����
		inline int GetPointTotal() { return data.colTotal * data.rowTotal; }

		// ���ܣ��õ����ݵ���Ϣ�ṹ��
		// ����ֵ�����ݵ���Ϣ�ṹ��
		inline const PatchDataInfo& GetDataInfo() { return data; }

		// ���ܣ��õ�ָ�����ݵ��λ�ã�����������������ʼ��ַΪGetDataInfo().datablock��
		// ���룺col, row - ָ���������к��С�
		// ����ֵ��ָ�����ݵ����������ʼλ�á�
		inline int GetDataPos(int col, int row) { return data.datumSize * (data.colTotal * row + col); }

		/************************ ���ʹ��� ************************/
		// ���ܣ���ʼ�����ݿռ䣬֮����PatchRemodeling()��������ģ�͡�
		// ���룺angle_row[row_total] - �нǶȣ���һ�е�ÿ�����Ӧ�ĽǶȡ��Ƕȱ��������
		//       angle_col[col_total] - �нǶȣ���һ�е�ÿ�����Ӧ�ĽǶȡ��Ƕȱ��������
		//		 datum_size - ÿ����ĳ��ȡ�
		// ����ֵ�����ݻ������ĳ��ȣ�data.dataTotal����0��ʾʧ�ܡ��ɹ���modelStateΪ1��
		// ע�⣺�нǶȺ��нǶȱ��������
		int InitializeData(double* angle_row, int row_total, double* angle_col, int col_total, int datum_size = lenPatchPoint_geo);

		// ���ܣ���ʼ�����ݿռ䣨�нǶȺ��нǶ��ǵȽǼ���ģ���֮����PatchRemodeling()��������ģ�͡�
		// ���룺angle_col0:dAngle_col:angle_col1, angle_row0:dAngle_row:angle_row1 - �С��нǶȡ�dAngle_col��dAngle_row����Ϊ�������Ƕȵ�����
		//		 datum_size - ÿ����ĳ��ȣ�ȱʡֵΪlenLidarPointGeo����СֵΪlenLidarPointXYZ��
		// ����ֵ�����ݻ������ĳ��ȣ�data.dataTotal����0��ʾʧ�ܡ��ɹ���modelStateΪ1��
		// ע�⣺�нǶȺ��нǶȱ��������
		int InitializeData(double dAngle_row, double angle_row0,  double angle_row1, double dAngle_col, double angle_col0, double angle_col1, int datum_size = lenPatchPoint_geo);

		// ���ܣ����ճ�ʼ���ĽǶ��ز�����������ģ�͡�
		// ���룺src - �����������������Դ��
		//		 angle_gau - ��������˹�˺�����ȣ�deg����������1.5~2��ɨ�貽����
		//		 search_method - ���������������ȡֵ1��2��3��4���Ƽ�ȡ4��
		//                       ����1�������㣬���������׼ȷ������2��3����ɽ��������4�Ƿ���2�ĸĽ��棬�ٶȽϿ졣
		//		 obj_func - �����Ŀ�꺯����ȡֵ0��1��2����ʾ��������������Ƽ�ȡ2��
		//		 contra - �Աȶȡ��԰��������ǿ��������Ч���Աȶ�Խ�����������Խ��������Խ������
		// ����ֵ����Ч������0��ʾ��ģʧ�ܡ��ɹ���modelStateΪ2��
		// ע�⣺1. ����Դsrc���нǶȺ��нǶȱ��������ݼ�����������ɨ�衣
		//       2. ����Դsrc�����Ѿ�����Lidardata2XYZI()ת��Ϊ�������顣
		int PatchRemodeling(LidarDataBlock& src, double angle_gau, int search_method = 4, int obj_func = 2, double contra = 1.0);

		// ���ܣ�ֱ��������Դ��������ģ�ͣ�����ҪInitializeData()+PatchRemodeling()��
		// ���룺src - �����������������Դ��
		// ����ֵ����Ч������0��ʾ��ģʧ�ܡ��ɹ���modelStateΪ2��
		// ע�⣺1. ����Դsrc���нǶȱ��������ݼ�����ģ���ǵ����ġ��нǶȱ��������
		//		 2. ����Դsrc�����Ѿ�����Lidardata2XYZI()ת��Ϊ�������顣
		int InitializeModel(LidarDataBlock& src);

		// ���ܣ�������淨�ߺ����ʣ��ɹ���modelStateΪ3�������Ƚ�������ģ�͡�
		// ���룺min_nbrSpace - �������С���루m������ֵ��max_nbrAngle�����춥γ�ȡ���
		//		 max_nbrAngle - ��������Ǽ����deg������ֵ��min_nbrSpace�����춥γ�ȡ�
		void CalculateGeometry(double min_nbrSpace = 0.001, double max_nbrAngle = 5.73);

		// ���ܣ����һ�����ε㣨������slope��belief���㣩��
		// ���룺col, row - ָ���������к��С��������Ч�ԡ�
		void ClearOnePoint(int col, int row);

		// ���ܣ���¡����ģ�͡�ԭ�е����ݱ�ɾ����
		// ���룺src - Դ����
		//       r0, nRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		// ����ֵ����¡�����ݵĵ�����
		int CloneFrom(PatchGeometry& src, int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		// ���ܣ��������Ƶ����ȡ�
		// ���룺contra - �Աȶȡ��԰��������ǿ��������Ч���Աȶ�Խ�����������Խ��������Խ������
		// ����ֵ���������ĵ�����
		void TuneContrast(double contra);

		/************************ �ļ���д ************************/
		// ���ܣ�д��geo�����ļ�����������modelStateΪ2��3�����Ƿ�д�뼸����Ϣ��
		// ���룺datafn - �ļ�����
		//       r0, nRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		// ����ֵ��д�����ݵ��������ı��ļ���������
		// ˵���������ļ��ĸ�ʽ���¡�
		//       �����У�<datumSize>, <colTotal>, <rowTotal>, angleCol[0], ..., angleCol[N-1], 0, 0, ...
		//		 ��m+1�У�angleRow[m],  datablock[m][0], datablock[m][1], ..., datablock[m][N-1]
		//       �����ļ�������MATLAB����importdata(<datafn>)��ȡ��
		int WritePatchGeo(const char* datafn, int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		// ���ܣ���ȡgeo�����ļ���ԭ���ݱ�ɾ����
		// ���룺datafn - �����ļ�����
		// ����ֵ��ʵ�ʶ�ȡ���ݵ�������data.nRow����0��ʾʧ�ܡ�
		// ˵�������������ļ��Ƿ񺬼�����Ϣ������modelState����Ϊ2��3��
		int ReadPatchGeo(const char* datafn);

		/************************ ������ƣ���̬������ ************************/
		// ���ܣ����ε���ת��ΪPointCloud���� - ��������
		// ���룺point_cloud - �յ���ָ�롣block - ���ε��ơ�minSlope - ȥ����б��С�ĵ㡣
		//       r0, nRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		// �����point_cloud - ת���õ���PointCloud����
		// ����ֵ��ת���ĵ��Ƶĵ�����
		static int PointGeoToPointXYZI(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, PatchGeometry& block, double minSlope,
			int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		// ���ܣ����ε���ת��ΪPointCloud���� - ������
		// ���룺point_cloud - �յ���ָ�롣block - ���ε��ơ�minSlope - ȥ����б��С�ĵ㡣
		//       r0, nRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		// �����point_cloud - ת���õ���PointCloud����
		// ����ֵ��ת���ĵ��Ƶĵ�����
		static int PointGeoToPointNormal(pcl::PointCloud<pcl::PointNormal>::Ptr& point_cloud, PatchGeometry& block, double minSlope,
			int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		// ���ܣ����ε���ת��ΪPointCloud���� - ����
		// ���룺point_cloud - �յ���ָ�롣block - ���ε��ơ�minSlope - ȥ����б��С�ĵ㡣
		//       r0, nRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		// �����point_cloud - ת���õ���PointCloud����
		// ����ֵ��ת���ĵ��Ƶĵ�����
		static int PointGeoToCurvatures(pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr& point_cloud, PatchGeometry& block, double minSlope,
			int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

	private:
		// ������Ϣ��������GetDataInfo()��ã���ֹ�޸ġ�
		PatchDataInfo data;      // surface data
	};
}
#endif // #ifndef PatchGeometry_HPP
