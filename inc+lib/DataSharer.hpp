// DataSharer.hpp
//
// DataSharer：提供进程间数据共享服务。
// 
// 使用方法：
// 1. 创建DataSharer对象。
// 2. 调用OpenSharedBlock()创建共享数据块，设置名称、序号和大小。名称和序号标识数据块，数据块大小应能容纳数据信息和数据内容。
//    用GetBlockName()和GetBlockSN()可以查询数据块名称和序号。
// 3. 调用WriteData()写入数据与信息，调用ReadData()读取数据与信息。
// 4. 调用WriteImage()写入图像数据与信息，调用ReadImage()读取图像数据与信息。
// 5. 共享数据块创建后大小不能改变。如果需要增大，则调用OpenSharedBlock()，改变数据块名称或序号。
// 注意：名称前缀"Global\\"表示允许跨网络访问。
//
// by Jingyu Lin, linjy02@hotmail.com, 2019.02

#ifndef DataSharer_H
#define DataSharer_H

#include <cstdint>

// 图像数据的信息结构。图像数据的字节数为Stride*Height。
struct InfoImage {
	int32_t Width;
	int32_t Height;
	int32_t Channel;			// 每个像素的字节数。
	int32_t Stride;			// 图像一行字节数，必须是4的倍数，即Stride=(Width*Channel+3)/4*4。
	double secPerFrame;		// 传送图像序列时两幅图像之间的时间间隔（秒）
};

class __declspec(dllexport) DataSharer
{
public:
	DataSharer();
	~DataSharer();

	// 功能：创建共享数据块，并释放之前创建的共享数据块。
	// 输入：BlockName - 数据块名称，最长255字节。snBlock - 数据块序号。szBlock - 数据块字节数，应能容纳数据信息和数据内容。
	// 返回值：成功则返回共享内存大小，否则返回0。
	// 说明：数据块名称和序号用于标识共享数据块。如果存在同名数据块并且字节数小于szBlock则失败。
	int OpenSharedBlock(const char* BlockName, int32_t snBlock, int32_t szBlock);

	// 功能：将数据及其信息写入共享数据块。
	// 输入：infoSrc[szInfo] - 被写入的数据信息。dataSrc[szData] - 被写入的数据内容。type - 数据类型，0是一般数据，1是图像
	// 返回值：写入的字节数（包括信息和数据），0表示写入失败。
	// 说明：先写信息再写数据。infoSrc为NULL或szInfo为0则无数据信息，只写入数据。dataSrc为NULL或szData为0则写入失败。
	int WriteData(uint8_t* infoSrc, int32_t szInfo, uint8_t* dataSrc, int32_t szData, int32_t type);

	// 功能：从共享数据块读取数据及其信息。
	// 输入：dest[szDest] - 用于保存数据及其信息的缓冲区。
	// 输出：dest - 读出的数据及其信息，信息在前数据在后。byteInfo - 数据信息的字节数。byteData - 数据内容的字节数。数据内容紧跟数据信息。
	//       type - 数据类型。0是一般数据，1是图像。secStamp - 数据更新的时戳（秒）。updated - 1表示上次调用Read之后数据已经更新。
	// 返回值：读出的字节数（包括信息和数据），0表示读取失败。
	// 说明：dest为NULL或szDest为0则仅读取数据块信息，返回值是可读取的数据字节数。
	int ReadData(uint8_t* dest, int32_t szDest, int32_t& byteInfo, int32_t& byteData, int32_t& type, double& secStamp, int32_t& updated);

	// 功能：将图像及其信息写入共享数据块。
	// 输入：imgSrc[szSrcImg] - 被写入的图像。Width，Height，Channel - 图像的宽、高和颜色通道。
	//       secPerFrame - 传送图像序列时两幅图像之间的时间间隔（秒）。
	// 返回值：图像数据字节数，0表示写入失败。
	// 说明：图像数据字节数为Channel×Width×Height，其中Channel×Width应为4的倍数。
	int WriteImage(uint8_t* imgSrc, int32_t szImgSrc, int32_t Width, int32_t Height, int32_t Channel, double secPerFrame);

	// 功能：从共享数据块读取图像及其信息。
	// 输入：imgDest[szImgDest] - 用于保存图像的缓冲区。
	// 输出：imgDest - 读出的图像数据。infoImg - 图像信息。secStamp - 数据更新的时戳（秒）。updated - 1表示上次调用Read之后数据已经更新。
	// 返回值：读出的图像数据字节数，0表示失败。
	// 说明：imgDest为NULL或szImgDest为0则仅读取图像信息，返回值是可读取的字节数。headerData.typeData应为1，否则读取失败。
	int ReadImage(uint8_t* imgDest, int32_t szImgDest, InfoImage& infoImg, double& secStamp, int32_t& updated);

	// 功能：读取共享数据块的名称、序号和大小。NULL和-1表示未成功调用Openxxx()。
	const char* GetBlockName();
	int GetBlockSN();
	int GetBlockBytes();

private:
	uint8_t privatedata[296];
};

#endif // #ifndef DataSharer_H
