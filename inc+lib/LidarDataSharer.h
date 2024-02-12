// LidarDataSharer.h
//
/********************************************************************************
* Structures for sharing raw LiDAR data.
* 
* 【说明】
*     本文件定义用于激光雷达数据共享的数据结构。
*     激光雷达数据共享时使用两个共享数据块，一个传递数据源信息，另一个传递数据内容。两个共享数据块
* 使用相同数据块名称，但前者序号为0，后者序号为数据源ID。数据源ID是一个唯一标识数据源的正整数（必须>0）。
*     数据源信息用LidarSrcInfo结构定义。第一个成员为数据源ID，第二个成员为每次传输的数据内容的字节数，
* 即传递数据内容的共享数据块的大小。其它成员为数据源的结构信息。激光雷达数据源信息还包含列角度，但不在
* LidarSrcInfo结构中，而是在传输数据源信息时作为数据内容传输。
*     数据内容分为两部分：
* （1）数据包信息头，用LidarDataHeader结构定义，其中rowNow是本数据包传输的数据行数。
* （2）按行排列的数据。每行包括数据行前缀和数据行。数据行前缀用LidarDataPrefix结构定义。
* 数据内容总长度 = [数据包前缀LidarDataHeader] + [数据行前缀LidarDataPrefix + 数据行] * LidarDataHeader.rowNow，
* 不应大于LidarSrcInfo.byteDataBuf。
*     数据源传输数据时，先传输数据源信息和列角度，然后分段传输各行数据。服务器接收数据时，根据数据源信息
* 登记数据源，然后将接收到的数据段整合成完整的激光雷达数据。
* 
* 
* Writen by Lin, Jingyu, linjy02@hotmail.com, 2021.07
* Revised by Lin, Jingyu, linjy02@hotmail.com, 2022.03
*
********************************************************************************/

#ifndef LidarDataShare_H
#define LidarDataShare_H

#include <cstdint>

struct LidarSrcInfo			// 数据源信息，公共的共享数据块的数据内容
{
	// control info
	std::int32_t idSource;			// data source ID (>0). 0 = the structure has been read.
	std::int32_t byteDataBuf;		// maximum BYTEs required for a packet of shared data. 
							// >= sizeof(LidarPreData) + sizeof(double)*datumSize*colTotal*M
	// specified data info
	std::int32_t lidarType;		// type of LiDAR data (enum LiDAR_Type)
	std::int32_t datumSize;		// length of one data point.
	std::int32_t colTotal;			// points in a row.
	std::int32_t rowTotal;			// estimated maximum rows of the whole data
	double lidar_params[8]; // Lidar system parameters
	//double angle_col[2048]; // the angles of columns angle_col[colTotal]. 
};

struct LidarDataHeader		// 数据包的信息头，专用的共享数据块的数据信息
{
	// packet prefix
	std::int32_t rowNow;			// number of effected rows in this packet. 0 = data has been read.
	std::int32_t rowLeft;			// estimated rows not sent. 0 = no more data
	//std::int32_t rowStart;			// starting row
	//std::int32_t rowPacket;		// maximum number of rows in a packet
};

struct LidarDataPrefix		// 数据包的数据行的前缀，专用的共享数据块的数据内容
{
	// row prefix
	double time_stamp;		// time stamp of each row (ms). 
	double angle_row;		// row angle (deg). 
	/****************************************************************
	double data[colTotal];	// one row
	****************************************************************/
};

#endif // #ifndef LidarDataShare_H
