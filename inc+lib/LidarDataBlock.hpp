// LidarDataBlock.hpp
//
/********************************************************************************
* Data structure for raw 3D data.
* 
* LidarDataBlock���ڹ���ʵʱ��άɨ�����ݡ�
* ��ע����ģ�鲻��ҪPCL֧�֣���������ƹ���ʹ��PCL��������PointCloud�������Ҫ����PCLͷ�ļ���
* ��PCL 1.13Ϊ�������Ӱ���Ŀ¼��Ҫ��ӣ�
* %PCL_ROOT%\include\pcl-1.13;
* %PCL_ROOT%\3rdParty\Boost\include\boost-1_82;
* %PCL_ROOT%\3rdParty\Eigen\eigen3
*
* 1. ��άɨ�����ݽṹLidarDataInfo
* ��LidarDataInfo�ṹ��GetDataInfo()��ã�������άɨ�����ݺ͵������顣
* ����άɨ����������ά����������������ÿ�����ݵ���������ֱ�ΪrowTotal��colTotal��datumSize��ʵ����Ч��������ΪnRow��
*   ���ݱ���������datablock[dataTotal]�У���������ɵ�������dataTotal��datumSize��colTotal��rowTotal��
*   ĿǰdatumSize��֧��ȡֵ1��2����һ���������������d���ڶ��������Ƿ����ǿi��
*   ���к͸��ж�Ӧ�ĽǶȷֱ�ΪangleRow[rowTotal]��angleCol[colTotal]��ÿ�л���һ������ʱ��tsRow[rowTotal]��
* �۵�����������ά����������������ÿ�����ݵ���������ֱ�ΪrowTotal��colTotal��pntDatumSize����Ч��������ΪnPntRow����Ч����ΪnPnts��
*   ���Ʊ���������points[pntDataTotal]�У���������ɵ�������pntDataTotal��pntDatumSize��colTotal��rowTotal��
*   ���Ƶ�ĽṹΪLidarPoint_xyzi����Ӧ��pntDatumSizeΪlenLidarPoint_xyzi��
* ��LidarDataBlock�����Զ��������ݿռ䣬����Ҫ�����ͷš�
* 
* 2. ������άɨ�����������ַ�ʽ��
* ����InitializeBlock()��ʼ������AddRow()�������롣ע����SetMechanic()���û�е������ȱʡ����Ϊȫ0��
* ����ReadLidardata()�������ļ����롣���ݵĴ�С�ͻ�е�����������ļ����塣
* ����CloneFrom()������LidarDataBlock�����¡��
*
* 3. ���ݶ�д
* ��ReadLidardata()��WriteLidardata()���ڶ�д��άɨ�������ļ���L3D�ļ�����
* ��WritePoints()���ڽ��������鱣��Ϊ�ı��ļ���
* 
* 4. ����ת��
* ����άɨ������Ҫ��Lidardata2XYZI()ת��Ϊ�������飬֮�����ת��ΪPointCloud����
* �ڶ��ڰ��������ǿ����άɨ�����ݣ���ת���ĵ�������������öԱȶȡ��Աȶ�Խ�����������Խ����������Խ������
* ��ת��Ϊ��������ǰҪ��SetMechanics()���û�е������ȱʡ����Ϊȫ0�����ò�������ת���ĵ���ʧЧ����Ҫ�ٵ���Lidardata2XYZI()ת����
*
* 5. ����PointCloud����
* ��PointsetToPointXYZ()��PointsetToPointXYZI()��PointsetToPointXYZRGBA����PointCloud����

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
	// ������άɨ��ԭʼ���ݺ͵�����������ݽṹ
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

	// ��άɨ��ϵͳ����
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

	// ԭʼ���ݵ�Ľṹ
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

	// ���Ƶ�Ľṹ
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

		// ���ܣ��ж϶����Ƿ��ʼ�� ��0��ʾδ��ʼ����
		// ����ֵ����άɨ�����ݿռ��С��GetDataInfo().dataTotal����
		inline int isBlockAvailable() { return data.dataTotal; }

		// ���ܣ��ж��Ƿ��յ���άɨ�����ݡ�0��ʾ�����ݡ�
		// ����ֵ����άɨ�����ݵ�ʵ��������GetDataInfo().nRow����
		inline int isDataAvailable() { return data.nRow; }

		// ���ܣ��õ���Ч�ĵ��Ƶ�����
		inline int GetPointTotal() { return data.colTotal * data.nPntRow; }

		// ���ܣ��õ������ɵĵ��Ƶ�����
		inline int GetPointMax() { return data.colTotal * data.rowTotal; }

		// ���ܣ��õ�������άɨ��������Ϣ�Ľṹ��
		// ����ֵ��������άɨ��������Ϣ�Ľṹ������data����
		inline const LidarDataInfo& GetDataInfo() { return data; }

		// ���ܣ��õ�ָ�����ݵ��λ�á�������ʼ��ַΪGetDataInfo().datablock��
		// ���룺col, row - ָ���������к��С��������Ч�ԡ�
		// ����ֵ��ָ�����ݵ����������ʼλ�á�
		inline int GetDataPos(int col, int row) { return data.datumSize * (data.colTotal * row + col); }

		// ���ܣ��õ�ָ�����Ƶ��λ�á�������ʼ��ַΪGetDataInfo().points��
		// ���룺col, row - ָ���������к��С�
		// ����ֵ��ָ�����ݵ����������ʼλ�á�
		inline int GetPointPos(int col, int row) { return data.pntDatumSize * (data.colTotal * row + col); }

		/************************ ���ݹ��� ************************/
		// ���ܣ���ʼ�����ݿռ䲢����нǶȡ�
		// ���룺datum_size, col_total, row_total - ���ݿռ�Ĵ�С��
		//       angle_col[col_total] - �нǶȣ���һ�е�ÿ�����Ӧ�ĽǶȡ����angle_colΪNULL����ԭ�Ƕ����飬�״γ�ʼ������ΪNULL��
		//       pnt_datum_size - ��col_total��row_total������ƿռ�Ĵ�С��
		//       lidar_type - ��άɨ��ϵͳ���ͣ�Ĭ��ΪLiDAR_Type_3D
		// ����ֵ����άɨ�����ݻ������ĳ��ȣ�data.dataTotal����0��ʾʧ�ܡ�
		int InitializeBlock(int datum_size, int col_total, int row_total, double* angle_col, int pnt_datum_size = lenLidarPoint_xyzi, int lidar_type = LiDAR_Type_3D);

		// ���ܣ�������ݡ����ı���������ݿռ䡣
		inline void ClearData() { data.nRow = data.nPnts = data.nPntRow = 0; }

		// ���ܣ�����յ��ơ����ı���������ݿռ䡣
		inline void ClearPoints() { data.nPnts = data.nPntRow = 0; }

		// ���ܣ����һ�����Ƶ㡣
		// ���룺col, row - ָ���������к��С��������Ч�ԡ�
		void ClearOnePoint(int col, int row);

		// ���ܣ�����һ����άɨ�����ݡ�
		// ���룺src[nSrc] - �������С�angleRow - �����ж�Ӧ�ĽǶȡ�timeStamp - �����ж�Ӧ��ʱ����
		// ����ֵ����������ݵ�������data.nRow����0��ʾδ����ռ����������
		// ע�⣺�������г���nSrcӦ��С��data.datumSize*data.colTotal��
		int AddRow(double* src, int nSrc, double angleRow, double timeStamp);

		// ���ܣ���¡��άɨ�����ݺ͵��ơ�ԭ���ݱ�ɾ����
		// ���룺src - ԴLidarDataBlock����
		//       r0, nRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		// ����ֵ����¡�����ݵĵ�����
		int CloneFrom(LidarDataBlock& src, int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		/************************ ����ת�� ************************/
		// ���ܣ�������άɨ��ϵͳ�Ļ�е���������ڵ���ת�����μ�LidarDataInfo�ṹ��lidar_params[8]��
		// ���룺lidar_params[8] - ��άɨ��ϵͳ�Ļ�е������
		void SetMechanics(double params[8]);

		// ���ܣ���ȡ��άɨ��ϵͳ�Ļ�е������
		inline const double* GetMechanics();

		// ���ܣ�����������άɨ������ʵʱת��ΪXYZI�������顣
		// ���룺contra - �Աȶȡ����ڰ��������ǿ����άɨ�����ݣ��Աȶ�Խ�����������Խ��������Խ������
		//		 correct - 1��ʾ����ʱ�����нǶȽ���У׼��0��ʾֱ��ת������У׼�нǶȡ�
		// ����ֵ��ת���ĵ��Ƶĵ�����
		// ˵������ת���������ơ������Ҫ����ת��ȫ�����ݣ����ȵ���ClearPoints()���ٽ���ת����
		int Lidardata2XYZI(double contra = 1.0, int correct = 1);

		// ���ܣ���������Lidardata2XYZI()ת���ĵ��ƵĶԱȶȡ�
		// ���룺contra - �Աȶȡ����ڰ��������ǿ����άɨ�����ݣ��Աȶ�Խ�����������Խ��������Խ������
		// ����ֵ���������ĵ�����
		int TuneContrast(double contra);

		/************************ ������ƣ�PointCloud���� ************************/
		// ���ܣ���������ת��ΪPointCloud���� - ��ɫ����
		// ���룺point_cloud - �յ���ָ�롣block - �ں��������顣
		//       r0, nPntRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		// �����point_cloud - ת���õ���PointCloud����
		// ����ֵ��ת���ĵ��Ƶĵ�����
		static int PointsetToPointXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, LidarDataBlock& block,
			int r0 = 0, int nPntRow = 0, int c0 = 0, int nCol = 0);

		// ���ܣ���������ת��ΪPointCloud���� - ����ǿ�ĵ���
		// ���룺point_cloud - �յ���ָ�롣block - �ں��������顣
		//       r0, nPntRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		// �����point_cloud - ת���õ���PointCloud����
		// ����ֵ��ת���ĵ��Ƶĵ�����
		static int PointsetToPointXYZI(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, LidarDataBlock& block,
			int r0 = 0, int nPntRow = 0, int c0 = 0, int nCol = 0);

		// ���ܣ���������ת��ΪPointCloud���� - ��ɫ����
		// ���룺point_cloud - �յ���ָ�롣block - �ں��������顣
		//       r0, nPntRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		// �����point_cloud - ת���õ���PointCloud����
		// ����ֵ��ת���ĵ��Ƶĵ�����
		static int PointsetToPointXYZRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, LidarDataBlock& block,
			int r0 = 0, int nPntRow = 0, int c0 = 0, int nCol = 0);

		/************************ �ļ���д ************************/
		// ���ܣ��������ļ���ȡ��άɨ�����ݡ�
		// ���룺datafn - �����ļ�����
		// ����ֵ��ʵ�ʶ�ȡ���ݵ�������data.nRow����0��ʾʧ�ܡ�
		int ReadLidardata(const char* datafn);

		// ���ܣ�����άɨ������д���ļ��������ļ�������MATLAB����importdata(<datafn>)��ȡ��
		// ���룺datafn - �ļ�����
		//       r0, nRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		// ����ֵ��д�����ݵ��������ı��ļ���������
		// ˵���������ļ��ĸ�ʽ���¡������������0��0��0��ͷ����datumSize=1��colTotal��rowTotal��ȡ�ļ���á�
		//       �����У�<datumSize>, <colTotal>, <rowTotal>, angleCol[0], ..., angleCol[N-1], 8������, 0, 0, ...
		//       ��m+1�У�m+1, tsRow[m], angleRow[m],  datablock[m][0], datablock[m][1], ...
		int WriteLidardata(const char* datafn, int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

		// ���ܣ�����������д���ı��ļ��������ļ�������MATLAB����importdata(<datafn>)��ȡ��
		// ���룺datafn - �ļ�����
		//       r0, nRow - �ü���������ʼ�к�������c0, nCol - �ü���������ʼ�к�������
		// ����ֵ��д����Ƶĵ������ı��ļ���������
		int WritePoints(const char* datafn, int r0 = 0, int nRow = 0, int c0 = 0, int nCol = 0);

	private:
		// ������Ϣ��������GetDataInfo()��ã���ֹ�޸ġ�
		LidarDataInfo data;      // raw lidar data and point cloud
	};
}
#endif // #ifndef LidarDataBlock_HPP
