// LidarDataServer.hpp
//
/********************************************************************************
* Server and displayer for real-time LiDAR data.
* 
* LidarServer�Ǽ����״�����ʵʱ����Ľ��նˣ����Ͷ���LidarDataSource��������ʵʱ���ռ����״����ݡ�
* ���а���һ��3D���ڣ�LidarDataViewer���󣩣�����ʵʱ��ʾ���յ����ݡ����ݴ�����ù������ݿ飨DataSharer���󣩡�
* ��Ҫģ��LidarDataBlock��DataSharer_x64.dll֧�֡�
* 
* ��LidarServer�����������衿
* 1. ����LidarServer������ServerStart()����ʵʱ���ݽ��գ���ServerStop()ֹͣ���ݽ��ա�
* 2. LidarServer����ͬʱ���ն������Դ���͵����ݡ�����Դ����Ż�Id��ʶ��
*    ��GetSourceTotal()���Եõ�����Դ��������
* 3. ����Դ�ֶη������ݣ�LidarServer�����յ������ݱ�����һ��LidarDataBlock�����С�
*    ��GetLidarData(idx)���Եõ�������Դ��LidarDataBlock����
* 3. ʵʱ��������ʱ����ʱ��AddPointCloud()���½��յ������ݶ�ת��Ϊ���Ʋ�����3D���ڣ�����ʵʱ��ʾ���ơ�
* 4. ����ldv��3D���ڣ�LidarDataViewer���󣩣�������������3D������
*
*
* Writen by Lin, Jingyu, linjy02@hotmail.com, 2022.03
* Revised by Lin, Jingyu, linjy02@hotmail.com, 2023.05
* Revised by Lin, Jingyu, linjy02@hotmail.com, 2023.07
*
********************************************************************************/


#ifndef LidarDataServer_HPP
#define LidarDataServer_HPP

//#include <vector>
//#include "pcl/point_types.h"
//#include "pcl/point_cloud.h"
//#include "..\LidarDataContainer\LidarDataBlock.hpp"
#include "PointCloudViewer.hpp"
#include "DataSharer.hpp"

namespace Lidar3D
{
	class LidarDataBlock;

	class __declspec(dllexport) LidarDataServer
	{
	public:
		LidarDataServer();
		~LidarDataServer();

		/************************ ���ݹ��� ************************/
		// ���ܣ���ʼ�������ݡ����ط�����״̬��1 - ���ڽ��ա�-1 - ֹͣ���ա�0 - ����ֹͣ���ա�
		int ServerStart();
		// ���ܣ�ֹͣ�������ݡ�
		inline void ServerStop() { bReceiving = 0;	}
		// ���ܣ���÷�����״̬��1 - ���ڽ��ա�-1 - ֹͣ���ա�0 - ����ֹͣ���ա�
		inline int IsServerReceiving() { return bReceiving; }

		// ���ܣ�ֹͣ�������ݣ�ɾ���������ݺ͵��ơ�
		void ClearAllData();

		// ���ܣ��������Դ��������
		inline int GetSourceTotal() { return source_id.size(); }

		// ���ܣ���������ԴID��������Դ��š�
		// ���룺srcId - ����ԴID��
		// ����ֵ������Դ��š�-1��ʾ�Ҳ�����
		int GetSourceIndex(int srcId);

		// ���ܣ���������Դ��Ų�������ԴID��
		// ���룺idx - ����Դ��š�
		// ����ֵ������ԴID��0��ʾidx������Χ��
		int GetSourceID(int idx);

		// ���ܣ����ָ������Դ�ļ����״����ݶ���
		// ���룺idx - ����Դ��š�
		// ����ֵ�������״����ݶ���NULL��ʾidx������Χ��
		LidarDataBlock* GetLidarData(int idx);

		/************************ ���Ʋ��� ************************/
		// ���ܣ���ָ������Դ����������ת��Ϊ���ƺ����3D���ڡ�
		// ���룺idx - ���Ƶ�����Դ����š�
		//       bRGB - �Ƿ�ת��Ϊ��ɫ���ơ�0��ʾת��Ϊ��ɫ���ơ�
		//       contra - �Աȶȡ����ڰ��������ǿ�ļ����״����ݣ��Աȶ�Խ�����������Խ��������Խ������
		// ����ֵ��ת���ĵ��Ƶ�������0��ʾ�޵��ơ�
		// ˵������������ʱ���Ʒֶ����ӡ��˺������ϴε��ô˺��������ӵ�����ת�������3D���ڡ�
		int AddPointCloudNew(int idx, int bRGB, double contra = 1.0);

		// ���ܣ���ָ������Դ����������ת��Ϊ���ƺ����3D���ڣ�ɾ���Ѿ�����ĵ��ơ�
		// ���룺idx - ���Ƶ�����Դ����š�
		//       bRGB - �Ƿ�ת��Ϊ��ɫ���ơ�0��ʾת��Ϊ��ɫ���ơ�
		//       contra - �Աȶȡ����ڰ��������ǿ�ļ����״����ݣ��Աȶ�Խ�����������Խ��������Խ������
		// ����ֵ��ת���ĵ��Ƶ�������0��ʾ�޵��ơ�
		int AddPointCloudAll(int idx, int bRGB, double contra = 1.0);

		// ���ܣ�ɾ��3D�����еĵ��ƣ���ɾ�����ݺ���ת���ĵ������顣
		// ���룺idx - ���Ƶ�����Դ����š�
		void RemovePointCloud(int idx);

		// ���ܣ�����������飬��ɾ�����ݺ�3D�����еĵ��ơ�
		// ���룺idx - ���Ƶ�����Դ����š�
		void ClearPointset(int idx);

		//// ���ܣ�ʹָ������Դ�����е��Ƹ�����ͨ�����õ�Ĵ�С����
		//// ���룺idx - ���Ƶ�����Դ����š�radius - ��Ĵ�С����ֵ��Ч����
		//// ����ֵ�������ĵ��Ƶ�������0��ʾ�޵��ơ�
		//// ˵����radiusΪ1��������С��Ϊ2���Ա���Ϊ������
		//int HightlightPointCloud(int idx, int radius);

		// ���ܣ����ָ������Դ�ĵ���������
		// ���룺idx - ���Ƶ�����Դ����š�
		inline int GetPointCloudTotal(int idx) { return pc_total[idx]; }
	
		// ���ܣ���õ���ID��
		// ���룺srcId - ����ԴID��pntSn - ���Ʊ�ţ�ȡֵ0 - (GetPointCloudTotal()-1)��
		// ����ֵ������ID�����srcId��pntSn��Ч�������ID��Ч��
		// ˵��1������ID�������ģ���GetPointCloudId(Id, Sn + N) - GetPointCloudId(Id, Sn) = N��
		// ˵��2��������ldv.ViewerRemovePointCloud()��ldv.ViewerSetPointRadius()��ldv.ViewerAddPointXYZ()
		inline int GetPointCloudId(int srcId, int pntSn) { return srcId * 10000 + pntSn; }

		/************************ ���� ************************/
	public:
		PointCloudViewer* ldv;						// 3D���ڶ��󣬿���ֱ�ӷ���

	private:
		// ���ڼ����״����ݹ���
		std::vector<int> source_id;					// ������ԴID
		std::vector<LidarDataBlock*> lidar_data;	// ������Դ�ļ����״�����
		std::vector<int> row_left;					// ������Դ��ʣ������0��ʾ������ɣ�
		
		// ���ڵ������ݹ���
		//std::vector<LidarPointSet*> lidar_points;	// ������Դ�ĵ���
		std::vector<int> pc_total;					// ������Դ��PointCloud�ĸ�����0��ʾû�е��ƣ�
		std::vector<int> pc_nRow;					// ������Դ����ת���ĵ��Ƶ�����������AddPointCloud()������������

		// �������ݽ���
		DataSharer* shared_srcInfo;					// ���ڴ�������Դ�Ĺ������ݿ�
		std::vector<DataSharer*> shared_data;		// ���ڴ������ݵĹ������ݿ�
		int bReceiving;		// �����̵߳�״̬��1 - ���ڽ��ա�-1 - ֹͣ���ա�0 - ����ֹͣ���ա�
		int byteDataMax;    // �����̵߳���ʱ������������ֽ�����������������
		static DWORD __stdcall ServerLoop(void* p);// �������ݵ��߳�
	};
}
#endif // #ifndef LidarDataServer_HPP
