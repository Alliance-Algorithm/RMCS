
#ifndef _MV_OBSOLETE_CAM_PARAMS_H_
#define _MV_OBSOLETE_CAM_PARAMS_H_

#include "PixelType.h"

/// \~chinese 输出帧的信息             \~english Output Frame Information
typedef struct _MV_FRAME_OUT_INFO_
{
  unsigned short nWidth;             ///< [OUT]  \~chinese 图像宽            \~english Image Width
  unsigned short nHeight;            ///< [OUT]  \~chinese 图像高            \~english Image Height
  enum MvGvspPixelType enPixelType;  ///< [OUT]  \~chinese 像素格式          \~english Pixel Type

  unsigned int nFrameNum;  ///< [OUT]  \~chinese 帧号              \~english Frame Number
  unsigned int
    nDevTimeStampHigh;  ///< [OUT]  \~chinese 时间戳高32位      \~english Timestamp high 32 bits
  unsigned int
    nDevTimeStampLow;  ///< [OUT]  \~chinese 时间戳低32位      \~english Timestamp low 32 bits
  unsigned int
    nReserved0;  ///< [OUT]  \~chinese 保留，8字节对齐   \~english Reserved, 8-byte aligned
  int64_t
    nHostTimeStamp;  ///< [OUT]  \~chinese 主机生成的时间戳  \~english Host-generated timestamp

  unsigned int nFrameLen;

  unsigned int nLostPacket;  // 本帧丢包数
  unsigned int nReserved[2];
} MV_FRAME_OUT_INFO;

/// \~chinese 保存图片参数            \~english Save image type
typedef struct _MV_SAVE_IMAGE_PARAM_T_
{
  unsigned char * pData;  ///< [IN]   \~chinese 输入数据缓存      \~english Input Data Buffer
  unsigned int nDataLen;  ///< [IN]   \~chinese 输入数据大小      \~english Input Data Size
  enum MvGvspPixelType
    enPixelType;           ///< [IN]   \~chinese 输入像素格式      \~english Input Data Pixel Format
  unsigned short nWidth;   ///< [IN]   \~chinese 图像宽            \~english Image Width
  unsigned short nHeight;  ///< [IN]   \~chinese 图像高            \~english Image Height

  unsigned char *
    pImageBuffer;          ///< [OUT]  \~chinese 输出图片缓存      \~english Output Image Buffer
  unsigned int nImageLen;  ///< [OUT]  \~chinese 输出图片大小      \~english Output Image Size
  unsigned int
    nBufferSize;  ///< [IN]   \~chinese 提供的输出缓冲区大小  \~english Output buffer size provided
  enum MV_SAVE_IAMGE_TYPE
    enImageType;  ///< [IN]   \~chinese 输出图片格式      \~english Output Image Format

} MV_SAVE_IMAGE_PARAM;

typedef struct _MV_IMAGE_BASIC_INFO_
{
  unsigned short nWidthValue;
  unsigned short nWidthMin;
  unsigned int nWidthMax;
  unsigned int nWidthInc;

  unsigned int nHeightValue;
  unsigned int nHeightMin;
  unsigned int nHeightMax;
  unsigned int nHeightInc;

  float fFrameRateValue;
  float fFrameRateMin;
  float fFrameRateMax;

  unsigned int enPixelType;  ///< [OUT]  \~chinese 当前的像素格式    \~english Current pixel format
  unsigned int
    nSupportedPixelFmtNum;  ///< [OUT]  \~chinese 支持的像素格式种类    \~english Support pixel format
  unsigned int enPixelList[MV_MAX_XML_SYMBOLIC_NUM];
  unsigned int nReserved[8];

} MV_IMAGE_BASIC_INFO;

/// \~chinese 噪声特性类型    \~english Noise feature type
typedef enum _MV_CC_BAYER_NOISE_FEATURE_TYPE
{
  MV_CC_BAYER_NOISE_FEATURE_TYPE_INVALID =
    0,  ///<        \~chinese 无效值                        \~english Invalid
  MV_CC_BAYER_NOISE_FEATURE_TYPE_PROFILE =
    1,  ///<        \~chinese 噪声曲线                      \~english Noise curve
  MV_CC_BAYER_NOISE_FEATURE_TYPE_LEVEL =
    2,  ///<        \~chinese 噪声水平                      \~english Noise level
  MV_CC_BAYER_NOISE_FEATURE_TYPE_DEFAULT =
    1,  ///<        \~chinese 默认值                        \~english Default

} MV_CC_BAYER_NOISE_FEATURE_TYPE;

/// \~chinese Bayer格式降噪特性信息    \~english Denoise profile info
typedef struct _MV_CC_BAYER_NOISE_PROFILE_INFO_T_
{
  unsigned int nVersion;  ///<  \~chinese 版本                           \~english version
  MV_CC_BAYER_NOISE_FEATURE_TYPE
    enNoiseFeatureType;  ///<  \~chinese 噪声特性类型       \~english noise feature type
  enum MvGvspPixelType
    enPixelType;    ///<  \~chinese 图像格式                       \~english image format
  int nNoiseLevel;  ///<  \~chinese 平均噪声水平                   \~english noise level
  unsigned int
    nCurvePointNum;   ///<  \~chinese 曲线点数                       \~english curve point number
  int * nNoiseCurve;  ///<  \~chinese 噪声曲线                       \~english noise curve
  int * nLumCurve;    ///<  \~chinese 亮度曲线                       \~english luminance curve

  unsigned int nRes[8];  ///<       \~chinese 预留                           \~english Reserved

} MV_CC_BAYER_NOISE_PROFILE_INFO;

/// \~chinese Bayer格式噪声估计参数    \~english Bayer noise estimate param
typedef struct _MV_CC_BAYER_NOISE_ESTIMATE_PARAM_T_
{
  unsigned int nWidth;   ///< [IN]  \~chinese 图像宽(大于等于8)              \~english Width
  unsigned int nHeight;  ///< [IN]  \~chinese 图像高(大于等于8)              \~english Height
  enum MvGvspPixelType
    enPixelType;  ///< [IN]  \~chinese 像素格式                       \~english Pixel format

  unsigned char *
    pSrcData;  ///< [IN]  \~chinese 输入数据缓存                   \~english Input data buffer
  unsigned int
    nSrcDataLen;  ///< [IN]  \~chinese 输入数据大小                   \~english Input data size

  unsigned int
    nNoiseThreshold;  ///< [IN]  \~chinese 噪声阈值(0-4095)               \~english Noise Threshold

  unsigned char *
    pCurveBuf;  ///< [IN]  \~chinese 用于存储噪声曲线和亮度曲线（需要外部分配，缓存大小：4096 * sizeof(int) * 2）     \~english Buffer used to store noise and brightness curves, size:4096 * sizeof(int) * 2)
  MV_CC_BAYER_NOISE_PROFILE_INFO
    stNoiseProfile;  ///< [OUT]  \~chinese 降噪特性信息               \~english Denoise profile

  unsigned int
    nThreadNum;  ///< [IN]  \~chinese 线程数量，0表示算法库根据硬件自适应；1表示单线程（默认）；大于1表示线程数目      \~english Thread number, 0 means that the library is adaptive to the hardware, 1 means single thread(Default value), Greater than 1 indicates the number of threads

  unsigned int nRes[8];  ///<       \~chinese 预留                           \~english Reserved

} MV_CC_BAYER_NOISE_ESTIMATE_PARAM;

/// \~chinese Bayer格式空域降噪参数    \~english Bayer spatial Denoise param
typedef struct _MV_CC_BAYER_SPATIAL_DENOISE_PARAM_T_
{
  unsigned int nWidth;   ///< [IN]  \~chinese 图像宽(大于等于8)              \~english Width
  unsigned int nHeight;  ///< [IN]  \~chinese 图像高(大于等于8)              \~english Height
  enum MvGvspPixelType
    enPixelType;  ///< [IN]  \~chinese 像素格式                       \~english Pixel format

  unsigned char *
    pSrcData;  ///< [IN]  \~chinese 输入数据缓存                   \~english Input data buffer
  unsigned int
    nSrcDataLen;  ///< [IN]  \~chinese 输入数据大小                   \~english Input data size

  unsigned char *
    pDstBuf;  ///< [OUT] \~chinese 输出降噪后的数据               \~english Output data buffer
  unsigned int
    nDstBufSize;  ///< [IN]  \~chinese 提供的输出缓冲区大小           \~english Provided output buffer size
  unsigned int
    nDstBufLen;  ///< [OUT] \~chinese 输出降噪后的数据长度           \~english Output data length

  MV_CC_BAYER_NOISE_PROFILE_INFO
    stNoiseProfile;  ///< [IN]  \~chinese 降噪特性信息(来源于噪声估计)   \~english Denoise profile
  unsigned int
    nDenoiseStrength;  ///< [IN]  \~chinese 降噪强度(0-100)                \~english nDenoise Strength
  unsigned int
    nSharpenStrength;  ///< [IN]  \~chinese 锐化强度(0-32)                 \~english Sharpen Strength
  unsigned int
    nNoiseCorrect;  ///< [IN]  \~chinese 噪声校正系数(0-1280)           \~english Noise Correct

  unsigned int
    nThreadNum;  ///< [IN]  \~chinese 线程数量，0表示算法库根据硬件自适应；1表示单线程（默认）；大于1表示线程数目         \~english Thread number, 0 means that the library is adaptive to the hardware, 1 means single thread(Default value), Greater than 1 indicates the number of threads

  unsigned int nRes[8];  ///<       \~chinese 预留                           \~english Reserved

} MV_CC_BAYER_SPATIAL_DENOISE_PARAM;

/// \~chinese CLUT参数                  \~english CLUT param
typedef struct _MV_CC_CLUT_PARAM_T_
{
  bool bCLUTEnable;  ///< [IN]  \~chinese 是否启用CLUT           \~english CLUT enable
  unsigned int
    nCLUTScale;  ///< [IN]  \~chinese 量化系数(2的整数幂,最大65536)  \~english Quantitative scale(Integer power of 2, <= 65536)
  unsigned int
    nCLUTSize;  ///< [IN]  \~chinese CLUT大小,[17,33]（建议值17）     \~english CLUT size[17,33](Recommended values of 17)
  unsigned char * pCLUTBuf;  ///< [IN]  \~chinese 量化CLUT表             \~english CLUT buffer
  unsigned int
    nCLUTBufLen;  ///< [IN]  \~chinese 量化CLUT缓存大小(nCLUTSize*nCLUTSize*nCLUTSize*sizeof(int)*3)  \~english CLUT buffer length(nCLUTSize*nCLUTSize*nCLUTSize*sizeof(int)*3)

  unsigned int nRes[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_CLUT_PARAM;

/// \~chinese 锐化结构体                \~english Sharpen structure
typedef struct _MV_CC_SHARPEN_PARAM_T_
{
  unsigned int nWidth;      ///< [IN]  \~chinese 图像宽度(最小8)        \~english Image Width
  unsigned int nHeight;     ///< [IN]  \~chinese 图像高度(最小8)        \~english Image Height
  unsigned char * pSrcBuf;  ///< [IN]  \~chinese 输入数据缓存           \~english Input data buffer
  unsigned int nSrcBufLen;  ///< [IN]  \~chinese 输入数据大小           \~english Input data length
  enum MvGvspPixelType
    enPixelType;  ///< [IN]  \~chinese 像素格式               \~english Pixel format

  unsigned char * pDstBuf;  ///< [OUT] \~chinese 输出数据缓存           \~english Output data buffer
  unsigned int
    nDstBufSize;  ///< [IN]  \~chinese 提供的输出缓冲区大小   \~english Provided output buffer size
  unsigned int nDstBufLen;  ///< [OUT] \~chinese 输出数据长度           \~english Output data length

  unsigned int
    nSharpenAmount;  ///< [IN]  \~chinese 锐度调节强度，[0,500]  \~english Sharpen amount,[0,500]  //   [nSharpenAmount 作废， 使用 nSharpenPosAmount & nSharpenNegAmount 替代 ]
  unsigned int
    nSharpenRadius;  ///< [IN]  \~chinese 锐度调节半径(半径越大，耗时越长)，[1,21]   \~english Sharpen radius(The larger the radius, the longer it takes),[1,21]
  unsigned int
    nSharpenThreshold;  ///< [IN]  \~chinese 锐度调节阈值，[0,255]  \~english Sharpen threshold,[0,255]

  unsigned int nSharpenPosAmount;  // [IN]     锐度调节正向强度，范围:[0, 500]
  unsigned int nSharpenNegAmount;  // [IN]     锐度调节负向强度，范围:[0, 500]

  unsigned int nRes[6];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_SHARPEN_PARAM;

/// \~chinese 色彩校正结构体            \~english Color correct structure
typedef struct _MV_CC_COLOR_CORRECT_PARAM_T_
{
  unsigned int nWidth;      ///< [IN]  \~chinese 图像宽度               \~english Image Width
  unsigned int nHeight;     ///< [IN]  \~chinese 图像高度               \~english Image Height
  unsigned char * pSrcBuf;  ///< [IN]  \~chinese 输入数据缓存           \~english Input data buffer
  unsigned int nSrcBufLen;  ///< [IN]  \~chinese 输入数据大小           \~english Input data length
  enum MvGvspPixelType
    enPixelType;  ///< [IN]  \~chinese 像素格式               \~english Pixel format

  unsigned char * pDstBuf;  ///< [OUT] \~chinese 输出数据缓存           \~english Output data buffer
  unsigned int
    nDstBufSize;  ///< [IN]  \~chinese 提供的输出缓冲区大小   \~english Provided output buffer size
  unsigned int nDstBufLen;  ///< [OUT] \~chinese 输出数据长度           \~english Output data length

  unsigned int
    nImageBit;  ///< [IN]  \~chinese 有效图像位数(8,10,12,16)   \~english Image bit(8 or 10 or 12 or 16)
  MV_CC_GAMMA_PARAM stGammaParam;  ///< [IN]  \~chinese Gamma信息              \~english Gamma info
  MV_CC_CCM_PARAM_EX stCCMParam;   ///< [IN]  \~chinese CCM信息                \~english CCM info
  MV_CC_CLUT_PARAM stCLUTParam;    ///< [IN]  \~chinese CLUT信息               \~english CLUT info

  unsigned int nRes[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_COLOR_CORRECT_PARAM;

/// \~chinese 矩形ROI结构体             \~english Rect ROI structure
typedef struct _MV_CC_RECT_I_
{
  unsigned int nX;       ///< \~chinese 矩形左上角X轴坐标            \~english X Position
  unsigned int nY;       ///< \~chinese 矩形左上角Y轴坐标            \~english Y Position
  unsigned int nWidth;   ///< \~chinese 矩形宽度                     \~english Rect Width
  unsigned int nHeight;  ///< \~chinese 矩形高度                     \~english Rect Height

} MV_CC_RECT_I;

/// \~chinese 噪声估计结构体            \~english Noise estimate structure
typedef struct _MV_CC_NOISE_ESTIMATE_PARAM_T_
{
  unsigned int nWidth;   ///< [IN]  \~chinese 图像宽度(最小8)        \~english Image Width
  unsigned int nHeight;  ///< [IN]  \~chinese 图像高度(最小8)        \~english Image Height
  enum MvGvspPixelType
    enPixelType;            ///< [IN]  \~chinese 像素格式               \~english Pixel format
  unsigned char * pSrcBuf;  ///< [IN]  \~chinese 输入数据缓存           \~english Input data buffer
  unsigned int nSrcBufLen;  ///< [IN]  \~chinese 输入数据大小           \~english Input data length

  MV_CC_RECT_I * pstROIRect;  ///< [IN]  \~chinese 图像ROI                \~english Image ROI
  unsigned int nROINum;       ///< [IN]  \~chinese ROI个数                \~english ROI number

  ///< \~chinese Bayer域噪声估计参数，Mono8/RGB域无效     \~english Bayer Noise estimate param,Mono8/RGB formats are invalid
  unsigned int
    nNoiseThreshold;  ///< [IN]  \~chinese 噪声阈值[0,4095]       \~english Noise threshold[0,4095]
  ///< \~chinese 建议值:8bit,0xE0;10bit,0x380;12bit,0xE00     \~english Suggestive value:8bit,0xE0;10bit,0x380;12bit,0xE00

  unsigned char *
    pNoiseProfile;  ///< [OUT] \~chinese 输出噪声特性           \~english Output Noise Profile
  unsigned int
    nNoiseProfileSize;  ///< [IN]  \~chinese 提供的输出缓冲区大小   \~english Provided output buffer size
  unsigned int
    nNoiseProfileLen;  ///< [OUT] \~chinese 输出噪声特性长度       \~english Output Noise Profile length

  unsigned int nRes[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_NOISE_ESTIMATE_PARAM;

/// \~chinese 空域降噪结构体            \~english Spatial denoise structure
typedef struct _MV_CC_SPATIAL_DENOISE_PARAM_T_
{
  unsigned int nWidth;   ///< [IN]  \~chinese 图像宽度(最小8)        \~english Image Width
  unsigned int nHeight;  ///< [IN]  \~chinese 图像高度(最小8)        \~english Image Height
  enum MvGvspPixelType
    enPixelType;            ///< [IN]  \~chinese 像素格式               \~english Pixel format
  unsigned char * pSrcBuf;  ///< [IN]  \~chinese 输入数据缓存           \~english Input data buffer
  unsigned int nSrcBufLen;  ///< [IN]  \~chinese 输入数据大小           \~english Input data length

  unsigned char * pDstBuf;  ///< [OUT] \~chinese 输出降噪后的数据       \~english Output data buffer
  unsigned int
    nDstBufSize;  ///< [IN]  \~chinese 提供的输出缓冲区大小   \~english Provided output buffer size
  unsigned int nDstBufLen;  ///< [OUT] \~chinese 输出降噪后的数据长度   \~english Output data length

  unsigned char *
    pNoiseProfile;  ///< [IN]  \~chinese 输入噪声特性           \~english Input Noise Profile
  unsigned int
    nNoiseProfileLen;  ///< [IN]  \~chinese 输入噪声特性长度       \~english Input Noise Profile length

  ///< \~chinese Bayer域空域降噪参数，Mono8/RGB域无效     \~english Bayer Spatial denoise param,Mono8/RGB formats are invalid
  unsigned int
    nBayerDenoiseStrength;  ///< [IN]  \~chinese 降噪强度[0,100]        \~english Denoise Strength[0,100]
  unsigned int
    nBayerSharpenStrength;  ///< [IN]  \~chinese 锐化强度[0,32]         \~english Sharpen Strength[0,32]
  unsigned int
    nBayerNoiseCorrect;  ///< [IN]  \~chinese 噪声校正系数[0,1280]   \~english Noise Correct[0,1280]

  ///< \~chinese Mono8/RGB域空域降噪参数，Bayer域无效     \~english Mono8/RGB Spatial denoise param,Bayer formats are invalid
  unsigned int
    nNoiseCorrectLum;  ///< [IN]  \~chinese 亮度校正系数[1,2000]   \~english Noise Correct Lum[1,2000]
  unsigned int
    nNoiseCorrectChrom;  ///< [IN]  \~chinese 色调校正系数[1,2000]   \~english Noise Correct Chrom[1,2000]
  unsigned int
    nStrengthLum;  ///< [IN]  \~chinese 亮度降噪强度[0,100]    \~english Strength Lum[0,100]
  unsigned int
    nStrengthChrom;  ///< [IN]  \~chinese 色调降噪强度[0,100]    \~english Strength Chrom[0,100]
  unsigned int
    nStrengthSharpen;  ///< [IN]  \~chinese 锐化强度[1,1000]       \~english Strength Sharpen[1,1000]

  unsigned int nRes[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_SPATIAL_DENOISE_PARAM;

/// \~chinese LSC标定结构体             \~english LSC calib structure
typedef struct _MV_CC_LSC_CALIB_PARAM_T_
{
  unsigned int nWidth;   ///< [IN]  \~chinese 图像宽度[16,65535]     \~english Image Width
  unsigned int nHeight;  ///< [IN]  \~chinese 图像高度[16-65535]     \~english Image Height
  enum MvGvspPixelType
    enPixelType;            ///< [IN]  \~chinese 像素格式               \~english Pixel format
  unsigned char * pSrcBuf;  ///< [IN]  \~chinese 输入数据缓存           \~english Input data buffer
  unsigned int nSrcBufLen;  ///< [IN]  \~chinese 输入数据长度           \~english Input data length

  unsigned char *
    pCalibBuf;  ///< [OUT] \~chinese 输出标定表缓存         \~english Output calib buffer
  unsigned int
    nCalibBufSize;  ///< [IN]  \~chinese 提供的标定表缓冲大小(nWidth*nHeight*sizeof(unsigned short))    \~english Provided output buffer size
  unsigned int
    nCalibBufLen;  ///< [OUT] \~chinese 输出标定表缓存长度     \~english Output calib buffer length

  unsigned int nSecNumW;  ///< [IN]  \~chinese 宽度分块数             \~english Width Sec num
  unsigned int nSecNumH;  ///< [IN]  \~chinese 高度分块数             \~english Height Sec num
  unsigned int nPadCoef;  ///< [IN]  \~chinese 边缘填充系数[1,5]      \~english Pad Coef[1,5]
  unsigned int
    nCalibMethod;  ///< [IN]  \~chinese 标定方式(0-中心为基准;1-最亮区域为基准;2-目标亮度为基准) \~english Calib method
  unsigned int
    nTargetGray;  ///< [IN]  \~chinese 目标亮度(标定方式为2时有效)    \~english Target Gray
  ///< \~chinese 8位,范围：[0,255]            \~english 8bit,range:[0,255]
  ///< \~chinese 10位,范围：[0,1023]          \~english 10bit,range:[0,1023]
  ///< \~chinese 12位,范围：[0,4095]          \~english 12bit,range:[0,4095]

  unsigned int nRes[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_LSC_CALIB_PARAM;

/// \~chinese LSC校正结构体             \~english LSC correct structure
typedef struct _MV_CC_LSC_CORRECT_PARAM_T_
{
  unsigned int nWidth;   ///< [IN]  \~chinese 图像宽度[16,65535]     \~english Image Width
  unsigned int nHeight;  ///< [IN]  \~chinese 图像高度[16,65535]     \~english Image Height
  enum MvGvspPixelType
    enPixelType;            ///< [IN]  \~chinese 像素格式               \~english Pixel format
  unsigned char * pSrcBuf;  ///< [IN]  \~chinese 输入数据缓存           \~english Input data buffer
  unsigned int nSrcBufLen;  ///< [IN]  \~chinese 输入数据长度           \~english Input data length

  unsigned char * pDstBuf;  ///< [OUT] \~chinese 输出数据缓存           \~english Output data buffer
  unsigned int
    nDstBufSize;  ///< [IN]  \~chinese 提供的输出缓冲区大小   \~english Provided output buffer size
  unsigned int nDstBufLen;  ///< [OUT] \~chinese 输出数据长度           \~english Output data length

  unsigned char *
    pCalibBuf;  ///< [IN]  \~chinese 输入标定表缓存         \~english Input calib buffer
  unsigned int
    nCalibBufLen;  ///< [IN]  \~chinese 输入标定表缓存长度     \~english Input calib buffer length

  unsigned int nRes[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_LSC_CORRECT_PARAM;

/// \~chinese 某个节点对应的子节点个数最大值    \~english The maximum number of child nodes corresponding to a node
#define MV_MAX_XML_NODE_NUM_C 128

/// \~chinese 节点名称字符串最大长度            \~english The maximum length of node name string
#define MV_MAX_XML_NODE_STRLEN_C 64

/// \~chinese 节点String值最大长度              \~english The maximum length of Node String
#define MV_MAX_XML_STRVALUE_STRLEN_C 64

/// \~chinese 节点描述字段最大长度              \~english The maximum length of the node description field
#define MV_MAX_XML_DISC_STRLEN_C 512

/// \~chinese 最多的单元数                      \~english The maximum number of units
#define MV_MAX_XML_ENTRY_NUM 10

/// \~chinese 父节点个数上限                    \~english The maximum number of parent nodes
#define MV_MAX_XML_PARENTS_NUM 8

/// \~chinese 每个已经实现单元的名称长度        \~english The length of the name of each unit that has been implemented
#define MV_MAX_XML_SYMBOLIC_STRLEN_C 64

enum MV_XML_Visibility
{
  V_Beginner = 0,   ///< Always visible
  V_Expert = 1,     ///< Visible for experts or Gurus
  V_Guru = 2,       ///< Visible for Gurus
  V_Invisible = 3,  ///< Not Visible
  V_Undefined = 99  ///< Object is not yet initialized
};

/// \~chinese 单个节点基本属性 | en:Single Node Basic Attributes
typedef struct _MV_XML_NODE_FEATURE_
{
  enum MV_XML_InterfaceType enType;     ///< \~chinese 节点类型         \~english Node Type
  enum MV_XML_Visibility enVisivility;  ///< \~chinese 是否可见         \~english Is visibility
  char strDescription
    [MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 节点描述,目前暂不支持   \~english Node Description, NOT SUPPORT NOW
  char strDisplayName
    [MV_MAX_XML_NODE_STRLEN_C];               ///< \~chinese 显示名称         \~english Display Name
  char strName[MV_MAX_XML_NODE_STRLEN_C];     ///< \~chinese 节点名           \~english Node Name
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 提示             \~english Notice

  unsigned int nReserved[4];
} MV_XML_NODE_FEATURE;

/// \~chinese 节点列表 | en:Node List
typedef struct _MV_XML_NODES_LIST_
{
  unsigned int nNodeNum;  ///< \~chinese 节点个数             \~english Node Number
  MV_XML_NODE_FEATURE stNodes[MV_MAX_XML_NODE_NUM_C];
} MV_XML_NODES_LIST;

typedef struct _MV_XML_FEATURE_Value_
{
  enum MV_XML_InterfaceType enType;  ///< \~chinese 节点类型         \~english Node Type
  char strDescription
    [MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 节点描述,目前暂不支持   \~english Node Description, NOT SUPPORT NOW
  char strDisplayName
    [MV_MAX_XML_NODE_STRLEN_C];               ///< \~chinese 显示名称         \~english Display Name
  char strName[MV_MAX_XML_NODE_STRLEN_C];     ///< \~chinese 节点名           \~english Node Name
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 提示             \~english Notice
  unsigned int nReserved[4];
} MV_XML_FEATURE_Value;

typedef struct _MV_XML_FEATURE_Base_
{
  enum MV_XML_AccessMode enAccessMode;  ///< \~chinese 访问模式         \~english Access Mode
} MV_XML_FEATURE_Base;

typedef struct _MV_XML_FEATURE_Integer_
{
  char strName[MV_MAX_XML_NODE_STRLEN_C];
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
  char strDescription
    [MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 目前暂不支持     \~english NOT SUPPORT NOW
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];

  enum MV_XML_Visibility enVisivility;  ///< \~chinese 是否可见         \~english Visible
  enum MV_XML_AccessMode enAccessMode;  ///< \~chinese 访问模式         \~english Access Mode
  int
    bIsLocked;  ///< \~chinese 是否锁定。0-否；1-是,目前暂不支持    \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW
  int64_t nValue;      ///< \~chinese 当前值           \~english Current Value
  int64_t nMinValue;   ///< \~chinese 最小值           \~english Min Value
  int64_t nMaxValue;   ///< \~chinese 最大值           \~english Max Value
  int64_t nIncrement;  ///< \~chinese 增量             \~english Increment

  unsigned int nReserved[4];

} MV_XML_FEATURE_Integer;

typedef struct _MV_XML_FEATURE_Boolean_
{
  char strName[MV_MAX_XML_NODE_STRLEN_C];
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
  char strDescription
    [MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 目前暂不支持     \~english NOT SUPPORT NOW
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];

  enum MV_XML_Visibility enVisivility;  ///< \~chinese 是否可见         \~english Visible
  enum MV_XML_AccessMode enAccessMode;  ///< \~chinese 访问模式         \~english Access Mode
  int
    bIsLocked;  ///< \~chinese 是否锁定。0-否；1-是,目前暂不支持    \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW
  bool bValue;  ///< \~chinese 当前值           \~english Current Value

  unsigned int nReserved[4];
} MV_XML_FEATURE_Boolean;

typedef struct _MV_XML_FEATURE_Command_
{
  char strName[MV_MAX_XML_NODE_STRLEN_C];
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
  char strDescription
    [MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 目前暂不支持     \~english NOT SUPPORT NOW
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];

  enum MV_XML_Visibility enVisivility;  ///< \~chinese 是否可见         \~english Visible
  enum MV_XML_AccessMode enAccessMode;  ///< \~chinese 访问模式         \~english Access Mode
  int
    bIsLocked;  ///< \~chinese 是否锁定。0-否；1-是,目前暂不支持    \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW

  unsigned int nReserved[4];
} MV_XML_FEATURE_Command;

typedef struct _MV_XML_FEATURE_Float_
{
  char strName[MV_MAX_XML_NODE_STRLEN_C];
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
  char strDescription
    [MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 目前暂不支持     \~english NOT SUPPORT NOW
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];

  enum MV_XML_Visibility enVisivility;  ///< \~chinese 是否可见         \~english Visible
  enum MV_XML_AccessMode enAccessMode;  ///< \~chinese 访问模式         \~english Access Mode
  int
    bIsLocked;  ///< \~chinese 是否锁定。0-否；1-是,目前暂不支持    \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW
  double dfValue;      ///< \~chinese 当前值           \~english Current Value
  double dfMinValue;   ///< \~chinese 最小值           \~english Min Value
  double dfMaxValue;   ///< \~chinese 最大值           \~english Max Value
  double dfIncrement;  ///< \~chinese 增量             \~english Increment

  unsigned int nReserved[4];
} MV_XML_FEATURE_Float;

typedef struct _MV_XML_FEATURE_String_
{
  char strName[MV_MAX_XML_NODE_STRLEN_C];
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
  char strDescription
    [MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 目前暂不支持     \~english NOT SUPPORT NOW
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];

  enum MV_XML_Visibility enVisivility;  ///< \~chinese 是否可见         \~english Visible
  enum MV_XML_AccessMode enAccessMode;  ///< \~chinese 访问模式         \~english Access Mode
  int
    bIsLocked;  ///< \~chinese 是否锁定。0-否；1-是,目前暂不支持    \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW
  char
    strValue[MV_MAX_XML_STRVALUE_STRLEN_C];  ///< \~chinese 当前值           \~english Current Value

  unsigned int nReserved[4];
} MV_XML_FEATURE_String;

typedef struct _MV_XML_FEATURE_Register_
{
  char strName[MV_MAX_XML_NODE_STRLEN_C];
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
  char strDescription
    [MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 目前暂不支持     \~english NOT SUPPORT NOW
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];

  enum MV_XML_Visibility enVisivility;  ///< \~chinese 是否可见         \~english Visible
  enum MV_XML_AccessMode enAccessMode;  ///< \~chinese 访问模式         \~english Access Mode
  int
    bIsLocked;  ///< \~chinese 是否锁定。0-否；1-是,目前暂不支持    \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW
  int64_t nAddrValue;  ///< \~chinese 当前值           \~english Current Value

  unsigned int nReserved[4];
} MV_XML_FEATURE_Register;

typedef struct _MV_XML_FEATURE_Category_
{
  char strDescription
    [MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 节点描述 目前暂不支持    \~english Node Description, NOT SUPPORT NOW
  char strDisplayName
    [MV_MAX_XML_NODE_STRLEN_C];               ///< \~chinese 显示名称         \~english Display Name
  char strName[MV_MAX_XML_NODE_STRLEN_C];     ///< \~chinese 节点名           \~english Node Name
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 提示             \~english Notice

  enum MV_XML_Visibility enVisivility;  ///< \~chinese 是否可见         \~english Visible

  unsigned int nReserved[4];
} MV_XML_FEATURE_Category;

typedef struct _MV_XML_FEATURE_EnumEntry_
{
  char strName[MV_MAX_XML_NODE_STRLEN_C];
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
  char strDescription
    [MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 目前暂不支持     \~english NOT SUPPORT NOW
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];
  int bIsImplemented;
  int nParentsNum;
  MV_XML_NODE_FEATURE stParentsList[MV_MAX_XML_PARENTS_NUM];

  enum MV_XML_Visibility enVisivility;  ///< \~chinese 是否可见         \~english Visible
  int64_t nValue;                       ///< \~chinese 当前值           \~english Current Value
  enum MV_XML_AccessMode enAccessMode;  ///< \~chinese 访问模式         \~english Access Mode
  int
    bIsLocked;  ///< \~chinese 是否锁定。0-否；1-是,目前暂不支持    \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW
  int nReserved[8];

} MV_XML_FEATURE_EnumEntry;

typedef struct _MV_XML_FEATURE_Enumeration_
{
  enum MV_XML_Visibility enVisivility;  ///< \~chinese 是否可见         \~english Visible
  char strDescription
    [MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 节点描述 目前暂不支持    \~english Node Description, NOT SUPPORT NOW
  char strDisplayName
    [MV_MAX_XML_NODE_STRLEN_C];               ///< \~chinese 显示名称         \~english Display Name
  char strName[MV_MAX_XML_NODE_STRLEN_C];     ///< \~chinese 节点名           \~english Node Name
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 提示             \~english Notice

  int nSymbolicNum;  ///< \~chinese ymbolic数        \~english Symbolic Number
  char strCurrentSymbolic
    [MV_MAX_XML_SYMBOLIC_STRLEN_C];  ///< \~chinese 当前Symbolic索引    \~english Current Symbolic Index
  char strSymbolic[MV_MAX_XML_SYMBOLIC_NUM][MV_MAX_XML_SYMBOLIC_STRLEN_C];
  enum MV_XML_AccessMode enAccessMode;  ////< \~chinese 访问模式        \~english Access Mode
  int
    bIsLocked;  ///< \~chinese 是否锁定。0-否；1-是,目前暂不支持    \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW
  int64_t nValue;  ///< \~chinese 当前值           \~english Current Value

  unsigned int nReserved[4];
} MV_XML_FEATURE_Enumeration;

typedef struct _MV_XML_FEATURE_Port_
{
  enum MV_XML_Visibility enVisivility;  ///< \~chinese 是否可见         \~english Visible
  char strDescription
    [MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 节点描述,目前暂不支持    \~english Node Description, NOT SUPPORT NOW
  char strDisplayName
    [MV_MAX_XML_NODE_STRLEN_C];               ///< \~chinese 显示名称         \~english Display Name
  char strName[MV_MAX_XML_NODE_STRLEN_C];     ///< \~chinese 节点名           \~english Node Name
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];  ///< \~chinese 提示             \~english Notice

  enum MV_XML_AccessMode enAccessMode;  ///< \~chinese 访问模式         \~english Access Mode
  int
    bIsLocked;  ///< \~chinese 是否锁定。0-否；1-是,目前暂不支持    \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW

  unsigned int nReserved[4];
} MV_XML_FEATURE_Port;

typedef struct _MV_XML_CAMERA_FEATURE_
{
  enum MV_XML_InterfaceType enType;
  union {
    MV_XML_FEATURE_Integer stIntegerFeature;
    MV_XML_FEATURE_Float stFloatFeature;
    MV_XML_FEATURE_Enumeration stEnumerationFeature;
    MV_XML_FEATURE_String stStringFeature;
  } SpecialFeature;

} MV_XML_CAMERA_FEATURE;

/// \~chinese 图片保存参数              \~english Save Image Parameters
typedef struct _MV_SAVE_IMAGE_PARAM_T_EX_
{
  unsigned char * pData;  ///< [IN]  \~chinese 输入数据缓存           \~english Input Data Buffer
  unsigned int nDataLen;  ///< [IN]  \~chinese 输入数据长度           \~english Input Data length
  enum MvGvspPixelType
    enPixelType;  ///< [IN]  \~chinese 输入数据的像素格式     \~english Input Data Pixel Format
  unsigned short nWidth;   ///< [IN]  \~chinese 图像宽                 \~english Image Width
  unsigned short nHeight;  ///< [IN]  \~chinese 图像高                 \~english Image Height

  unsigned char *
    pImageBuffer;          ///< [OUT] \~chinese 输出图片缓存           \~english Output Image Buffer
  unsigned int nImageLen;  ///< [OUT] \~chinese 输出图片长度           \~english Output Image length
  unsigned int
    nBufferSize;  ///< [IN]  \~chinese 提供的输出缓冲区大小   \~english Output buffer size provided
  enum MV_SAVE_IAMGE_TYPE
    enImageType;  ///< [IN]  \~chinese 输出图片格式           \~english Output Image Format
  unsigned int
    nJpgQuality;  ///< [IN]  \~chinese JPG编码质量(50-99]，其它格式无效   \~english Encoding quality(50-99]，Other formats are invalid

  unsigned int
    iMethodValue;  ///< [IN]  \~chinese 插值方法 0-快速 1-均衡 2-最优（其它值默认为最优）  \~english Bayer interpolation method  0-Fast 1-Equilibrium 2-Optimal

  unsigned int nReserved[3];  ///<       \~chinese 预留                   \~english Reserved

} MV_SAVE_IMAGE_PARAM_EX;

/// \~chinese 图片保存参数              \~english Save Image Parameters
typedef struct _MV_SAVE_IMG_TO_FILE_PARAM_
{
  enum MvGvspPixelType
    enPixelType;  ///< [IN]  \~chinese输入数据的像素格式      \~english The pixel format of the input data
  unsigned char * pData;   ///< [IN]  \~chinese 输入数据缓存           \~english Input Data Buffer
  unsigned int nDataLen;   ///< [IN]  \~chinese 输入数据长度           \~english Input Data length
  unsigned short nWidth;   ///< [IN]  \~chinese 图像宽                 \~english Image Width
  unsigned short nHeight;  ///< [IN]  \~chinese 图像高                 \~english Image Height
  enum MV_SAVE_IAMGE_TYPE
    enImageType;  ///< [IN]  \~chinese 输入图片格式           \~english Input Image Format
  unsigned int
    nQuality;  ///< [IN]  \~chinese JPG编码质量(50-99]，PNG编码质量[0-9]，其它格式无效 \~english JPG Encoding quality(50-99],PNG Encoding quality[0-9]，Other formats are invalid
  char pImagePath[256];  ///< [IN]  \~chinese 输入文件路径           \~english Input file path

  int
    iMethodValue;  ///< [IN]  \~chinese 插值方法 0-快速 1-均衡 2-最优（其它值默认为最优）  \~english Bayer interpolation method  0-Fast 1-Equilibrium 2-Optimal

  unsigned int nReserved[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_SAVE_IMG_TO_FILE_PARAM;

// \~chinese 像素转换结构体            \~english Pixel convert structure
typedef struct _MV_CC_PIXEL_CONVERT_PARAM_
{
  unsigned short nWidth;   ///< [IN]  \~chinese 图像宽                 \~english Width
  unsigned short nHeight;  ///< [IN]  \~chinese 图像高                 \~english Height

  enum MvGvspPixelType
    enSrcPixelType;  ///< [IN]  \~chinese 源像素格式             \~english Source pixel format
  unsigned char * pSrcData;  ///< [IN]  \~chinese 输入数据缓存           \~english Input data buffer
  unsigned int nSrcDataLen;  ///< [IN]  \~chinese 输入数据长度           \~english Input data length

  enum MvGvspPixelType
    enDstPixelType;  ///< [IN]  \~chinese 目标像素格式           \~english Destination pixel format
  unsigned char *
    pDstBuffer;          ///< [OUT] \~chinese 输出数据缓存           \~english Output data buffer
  unsigned int nDstLen;  ///< [OUT] \~chinese 输出数据长度           \~english Output data length
  unsigned int
    nDstBufferSize;  ///< [IN]  \~chinese 提供的输出缓冲区大小   \~english Provided output buffer size

  unsigned int nRes[4];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_PIXEL_CONVERT_PARAM;

#endif /* _MV_OBSOLETE_CAM_PARAMS_H_ */
