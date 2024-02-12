// LidarDataServer.hpp
//
/********************************************************************************
* Server and displayer for real-time LiDAR data.
* 
* LidarServer是激光雷达数据实时传输的接收端（发送端是LidarDataSource），用于实时接收激光雷达数据。
* 其中包含一个3D窗口（LidarDataViewer对象），可以实时显示接收的数据。数据传输采用共享数据块（DataSharer对象）。
* 需要模块LidarDataBlock和DataSharer_x64.dll支持。
* 
* 【LidarServer基本操作步骤】
* 1. 创建LidarServer对象，用ServerStart()启动实时数据接收，用ServerStop()停止数据接收。
* 2. LidarServer可以同时接收多个数据源发送的数据。数据源用序号或Id标识。
*    用GetSourceTotal()可以得到数据源的总数。
* 3. 数据源分段发送数据，LidarServer将接收到的数据保存在一个LidarDataBlock对象中。
*    用GetLidarData(idx)可以得到各数据源的LidarDataBlock对象。
* 3. 实时接收数据时，定时用AddPointCloud()将新接收到的数据段转换为点云并加入3D窗口，可以实时显示点云。
* 4. 属性ldv是3D窗口（LidarDataViewer对象），可以用于其它3D操作。
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

		/************************ 数据管理 ************************/
		// 功能：开始接收数据。返回服务器状态：1 - 正在接收。-1 - 停止接收。0 - 正在停止接收。
		int ServerStart();
		// 功能：停止接收数据。
		inline void ServerStop() { bReceiving = 0;	}
		// 功能：获得服务器状态：1 - 正在接收。-1 - 停止接收。0 - 正在停止接收。
		inline int IsServerReceiving() { return bReceiving; }

		// 功能：停止接收数据，删除所有数据和点云。
		void ClearAllData();

		// 功能：获得数据源的总数。
		inline int GetSourceTotal() { return source_id.size(); }

		// 功能：按照数据源ID查找数据源序号。
		// 输入：srcId - 数据源ID。
		// 返回值：数据源序号。-1表示找不到。
		int GetSourceIndex(int srcId);

		// 功能：按照数据源序号查找数据源ID。
		// 输入：idx - 数据源序号。
		// 返回值：数据源ID。0表示idx超出范围。
		int GetSourceID(int idx);

		// 功能：获得指定数据源的激光雷达数据对象。
		// 输入：idx - 数据源序号。
		// 返回值：激光雷达数据对象。NULL表示idx超出范围。
		LidarDataBlock* GetLidarData(int idx);

		/************************ 点云操作 ************************/
		// 功能：将指定数据源的新增数据转换为点云后放入3D窗口。
		// 输入：idx - 点云的数据源的序号。
		//       bRGB - 是否转换为彩色点云。0表示转换为无色点云。
		//       contra - 对比度。对于包含反射光强的激光雷达数据，对比度越大则点云亮度越暗、纹理越清晰。
		// 返回值：转换的点云的行数。0表示无点云。
		// 说明：接收数据时点云分段增加。此函数将上次调用此函数后增加的数据转换后放入3D窗口。
		int AddPointCloudNew(int idx, int bRGB, double contra = 1.0);

		// 功能：将指定数据源的所有数据转换为点云后放入3D窗口，删除已经放入的点云。
		// 输入：idx - 点云的数据源的序号。
		//       bRGB - 是否转换为彩色点云。0表示转换为无色点云。
		//       contra - 对比度。对于包含反射光强的激光雷达数据，对比度越大则点云亮度越暗、纹理越清晰。
		// 返回值：转换的点云的行数。0表示无点云。
		int AddPointCloudAll(int idx, int bRGB, double contra = 1.0);

		// 功能：删除3D窗口中的点云，不删除数据和已转换的点云数组。
		// 输入：idx - 点云的数据源的序号。
		void RemovePointCloud(int idx);

		// 功能：清除点云数组，不删除数据和3D窗口中的点云。
		// 输入：idx - 点云的数据源的序号。
		void ClearPointset(int idx);

		//// 功能：使指定数据源的所有点云高亮（通过设置点的大小）。
		//// 输入：idx - 点云的数据源的序号。radius - 点的大小（正值有效）。
		//// 返回值：高亮的点云的数量。0表示无点云。
		//// 说明：radius为1是正常大小，为2可以表现为高亮。
		//int HightlightPointCloud(int idx, int radius);

		// 功能：获得指定数据源的点云数量。
		// 输入：idx - 点云的数据源的序号。
		inline int GetPointCloudTotal(int idx) { return pc_total[idx]; }
	
		// 功能：获得点云ID。
		// 输入：srcId - 数据源ID。pntSn - 点云编号，取值0 - (GetPointCloudTotal()-1)。
		// 返回值：点云ID。如果srcId或pntSn无效，则点云ID无效。
		// 说明1：点云ID是连续的，即GetPointCloudId(Id, Sn + N) - GetPointCloudId(Id, Sn) = N。
		// 说明2：可用于ldv.ViewerRemovePointCloud()、ldv.ViewerSetPointRadius()、ldv.ViewerAddPointXYZ()
		inline int GetPointCloudId(int srcId, int pntSn) { return srcId * 10000 + pntSn; }

		/************************ 属性 ************************/
	public:
		PointCloudViewer* ldv;						// 3D窗口对象，可以直接访问

	private:
		// 用于激光雷达数据管理
		std::vector<int> source_id;					// 各数据源ID
		std::vector<LidarDataBlock*> lidar_data;	// 各数据源的激光雷达数据
		std::vector<int> row_left;					// 各数据源所剩行数（0表示传输完成）
		
		// 用于点云数据管理
		//std::vector<LidarPointSet*> lidar_points;	// 各数据源的点云
		std::vector<int> pc_total;					// 各数据源的PointCloud的个数（0表示没有点云）
		std::vector<int> pc_nRow;					// 各数据源的已转换的点云的行数。用于AddPointCloud()计算新增点云

		// 用于数据接收
		DataSharer* shared_srcInfo;					// 用于传输数据源的共享数据块
		std::vector<DataSharer*> shared_data;		// 用于传输数据的共享数据块
		int bReceiving;		// 接收线程的状态：1 - 正在接收。-1 - 停止接收。0 - 正在停止接收。
		int byteDataMax;    // 接收线程的临时缓冲区的最大字节数。可随需求增大。
		static DWORD __stdcall ServerLoop(void* p);// 接收数据的线程
	};
}
#endif // #ifndef LidarDataServer_HPP
