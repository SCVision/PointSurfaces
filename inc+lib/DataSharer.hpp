// DataSharer.hpp
//
// DataSharer���ṩ���̼����ݹ������
// 
// ʹ�÷�����
// 1. ����DataSharer����
// 2. ����OpenSharedBlock()�����������ݿ飬�������ơ���źʹ�С�����ƺ���ű�ʶ���ݿ飬���ݿ��СӦ������������Ϣ���������ݡ�
//    ��GetBlockName()��GetBlockSN()���Բ�ѯ���ݿ����ƺ���š�
// 3. ����WriteData()д����������Ϣ������ReadData()��ȡ��������Ϣ��
// 4. ����WriteImage()д��ͼ����������Ϣ������ReadImage()��ȡͼ����������Ϣ��
// 5. �������ݿ鴴�����С���ܸı䡣�����Ҫ���������OpenSharedBlock()���ı����ݿ����ƻ���š�
// ע�⣺����ǰ׺"Global\\"��ʾ�����������ʡ�
//
// by Jingyu Lin, linjy02@hotmail.com, 2019.02

#ifndef DataSharer_H
#define DataSharer_H

#include <cstdint>

// ͼ�����ݵ���Ϣ�ṹ��ͼ�����ݵ��ֽ���ΪStride*Height��
struct InfoImage {
	int32_t Width;
	int32_t Height;
	int32_t Channel;			// ÿ�����ص��ֽ�����
	int32_t Stride;			// ͼ��һ���ֽ�����������4�ı�������Stride=(Width*Channel+3)/4*4��
	double secPerFrame;		// ����ͼ������ʱ����ͼ��֮���ʱ�������룩
};

class __declspec(dllexport) DataSharer
{
public:
	DataSharer();
	~DataSharer();

	// ���ܣ������������ݿ飬���ͷ�֮ǰ�����Ĺ������ݿ顣
	// ���룺BlockName - ���ݿ����ƣ��255�ֽڡ�snBlock - ���ݿ���š�szBlock - ���ݿ��ֽ�����Ӧ������������Ϣ���������ݡ�
	// ����ֵ���ɹ��򷵻ع����ڴ��С�����򷵻�0��
	// ˵�������ݿ����ƺ�������ڱ�ʶ�������ݿ顣�������ͬ�����ݿ鲢���ֽ���С��szBlock��ʧ�ܡ�
	int OpenSharedBlock(const char* BlockName, int32_t snBlock, int32_t szBlock);

	// ���ܣ������ݼ�����Ϣд�빲�����ݿ顣
	// ���룺infoSrc[szInfo] - ��д���������Ϣ��dataSrc[szData] - ��д����������ݡ�type - �������ͣ�0��һ�����ݣ�1��ͼ��
	// ����ֵ��д����ֽ�����������Ϣ�����ݣ���0��ʾд��ʧ�ܡ�
	// ˵������д��Ϣ��д���ݡ�infoSrcΪNULL��szInfoΪ0����������Ϣ��ֻд�����ݡ�dataSrcΪNULL��szDataΪ0��д��ʧ�ܡ�
	int WriteData(uint8_t* infoSrc, int32_t szInfo, uint8_t* dataSrc, int32_t szData, int32_t type);

	// ���ܣ��ӹ������ݿ��ȡ���ݼ�����Ϣ��
	// ���룺dest[szDest] - ���ڱ������ݼ�����Ϣ�Ļ�������
	// �����dest - ���������ݼ�����Ϣ����Ϣ��ǰ�����ں�byteInfo - ������Ϣ���ֽ�����byteData - �������ݵ��ֽ������������ݽ���������Ϣ��
	//       type - �������͡�0��һ�����ݣ�1��ͼ��secStamp - ���ݸ��µ�ʱ�����룩��updated - 1��ʾ�ϴε���Read֮�������Ѿ����¡�
	// ����ֵ���������ֽ�����������Ϣ�����ݣ���0��ʾ��ȡʧ�ܡ�
	// ˵����destΪNULL��szDestΪ0�����ȡ���ݿ���Ϣ������ֵ�ǿɶ�ȡ�������ֽ�����
	int ReadData(uint8_t* dest, int32_t szDest, int32_t& byteInfo, int32_t& byteData, int32_t& type, double& secStamp, int32_t& updated);

	// ���ܣ���ͼ������Ϣд�빲�����ݿ顣
	// ���룺imgSrc[szSrcImg] - ��д���ͼ��Width��Height��Channel - ͼ��Ŀ��ߺ���ɫͨ����
	//       secPerFrame - ����ͼ������ʱ����ͼ��֮���ʱ�������룩��
	// ����ֵ��ͼ�������ֽ�����0��ʾд��ʧ�ܡ�
	// ˵����ͼ�������ֽ���ΪChannel��Width��Height������Channel��WidthӦΪ4�ı�����
	int WriteImage(uint8_t* imgSrc, int32_t szImgSrc, int32_t Width, int32_t Height, int32_t Channel, double secPerFrame);

	// ���ܣ��ӹ������ݿ��ȡͼ������Ϣ��
	// ���룺imgDest[szImgDest] - ���ڱ���ͼ��Ļ�������
	// �����imgDest - ������ͼ�����ݡ�infoImg - ͼ����Ϣ��secStamp - ���ݸ��µ�ʱ�����룩��updated - 1��ʾ�ϴε���Read֮�������Ѿ����¡�
	// ����ֵ��������ͼ�������ֽ�����0��ʾʧ�ܡ�
	// ˵����imgDestΪNULL��szImgDestΪ0�����ȡͼ����Ϣ������ֵ�ǿɶ�ȡ���ֽ�����headerData.typeDataӦΪ1�������ȡʧ�ܡ�
	int ReadImage(uint8_t* imgDest, int32_t szImgDest, InfoImage& infoImg, double& secStamp, int32_t& updated);

	// ���ܣ���ȡ�������ݿ�����ơ���źʹ�С��NULL��-1��ʾδ�ɹ�����Openxxx()��
	const char* GetBlockName();
	int GetBlockSN();
	int GetBlockBytes();

private:
	uint8_t privatedata[296];
};

#endif // #ifndef DataSharer_H
