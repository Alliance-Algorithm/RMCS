
#ifndef _MV_CAMERA_PARAMS_H_
#define _MV_CAMERA_PARAMS_H_

#include "PixelType.h"

#ifndef __cplusplus
typedef char bool;
#define true 1
#define false 0
#endif

/// \~chinese 排序方式                  \~english The Method of Sorting
typedef enum _MV_SORT_METHOD_
{
  SortMethod_SerialNumber =
    0,  ///< \~chinese 按序列号排序                     \~english Sorting by SerialNumber
  SortMethod_UserID =
    1,  ///< \~chinese 按用户自定义名字排序             \~english Sorting by UserID
  SortMethod_CurrentIP_ASC =
    2,  ///< \~chinese 按当前IP地址排序（升序）         \~english Sorting by current IP（Ascending）
  SortMethod_CurrentIP_DESC =
    3,  ///< \~chinese 按当前IP地址排序（降序）         \~english Sorting by current IP（Descending）

} MV_SORT_METHOD;

/// \~chinese GigE设备信息              \~english GigE device info
typedef struct _MV_GIGE_DEVICE_INFO_
{
  unsigned int
    nIpCfgOption;  ///< [OUT] \~chinese IP配置选项             \~english IP Configuration Options
  unsigned int
    nIpCfgCurrent;          ///< [OUT] \~chinese 当前IP配置             \~english IP Configuration
  unsigned int nCurrentIp;  ///< [OUT] \~chinese 当前IP地址             \~english Current Ip
  unsigned int
    nCurrentSubNetMask;  ///< [OUT] \~chinese 当前子网掩码           \~english Curtent Subnet Mask
  unsigned int
    nDefultGateWay;  ///< [OUT] \~chinese 当前网关               \~english Current Gateway
  unsigned char
    chManufacturerName[32];  ///< [OUT] \~chinese 制造商名称             \~english Manufacturer Name
  unsigned char chModelName[32];  ///< [OUT] \~chinese 型号名称               \~english Model Name
  unsigned char
    chDeviceVersion[32];  ///< [OUT] \~chinese 设备版本               \~english Device Version
  unsigned char chManufacturerSpecificInfo
    [48];  ///< [OUT] \~chinese 制造商的具体信息       \~english Manufacturer Specific Information
  unsigned char
    chSerialNumber[16];  ///< [OUT] \~chinese 序列号                 \~english Serial Number
  unsigned char
    chUserDefinedName[16];  ///< [OUT] \~chinese 用户自定义名称         \~english User Defined Name
  unsigned int nNetExport;  ///< [OUT] \~chinese 网口IP地址             \~english NetWork IP Address

  unsigned int nReserved[4];  ///<       \~chinese 预留                   \~english Reserved

} MV_GIGE_DEVICE_INFO;

///< \~chinese 最大的数据信息大小       \~english Maximum data information size
#define INFO_MAX_BUFFER_SIZE 64

/// \~chinese USB设备信息               \~english USB device info
typedef struct _MV_USB3_DEVICE_INFO_
{
  unsigned char
    CrtlInEndPoint;  ///< [OUT] \~chinese 控制输入端点           \~english Control input endpoint
  unsigned char
    CrtlOutEndPoint;  ///< [OUT] \~chinese 控制输出端点           \~english Control output endpoint
  unsigned char StreamEndPoint;  ///< [OUT] \~chinese 流端点                 \~english Flow endpoint
  unsigned char EventEndPoint;  ///< [OUT] \~chinese 事件端点               \~english Event endpoint
  unsigned short idVendor;   ///< [OUT] \~chinese 供应商ID号             \~english Vendor ID Number
  unsigned short idProduct;  ///< [OUT] \~chinese 产品ID号               \~english Device ID Number
  unsigned int nDeviceNumber;  ///< [OUT] \~chinese 设备索引号             \~english Device Number
  unsigned char chDeviceGUID
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 设备GUID号             \~english Device GUID Number
  unsigned char chVendorName
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 供应商名字             \~english Vendor Name
  unsigned char chModelName
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 型号名字               \~english Model Name
  unsigned char chFamilyName
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 家族名字               \~english Family Name
  unsigned char chDeviceVersion
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 设备版本               \~english Device Version
  unsigned char chManufacturerName
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 制造商名字             \~english Manufacturer Name
  unsigned char chSerialNumber
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 序列号                 \~english Serial Number
  unsigned char chUserDefinedName
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 用户自定义名字         \~english User Defined Name
  unsigned int nbcdUSB;  ///< [OUT] \~chinese 支持的USB协议          \~english Support USB Protocol
  unsigned int nDeviceAddress;  ///< [OUT] \~chinese 设备地址               \~english Device Address

  unsigned int nReserved[2];  ///<       \~chinese 预留                   \~english Reserved

} MV_USB3_DEVICE_INFO;

/// \~chinese CameraLink设备信息        \~english CameraLink device info
typedef struct _MV_CamL_DEV_INFO_
{
  unsigned char
    chPortID[INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 端口号                 \~english Port ID
  unsigned char chModelName
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 型号名字               \~english Model Name
  unsigned char chFamilyName
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 名称                   \~english Family Name
  unsigned char chDeviceVersion
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 设备版本               \~english Device Version
  unsigned char chManufacturerName
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 制造商名字             \~english Manufacturer Name
  unsigned char chSerialNumber
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 序列号                 \~english Serial Number

  unsigned int nReserved[38];  ///<       \~chinese 预留                   \~english Reserved

} MV_CamL_DEV_INFO;

///< \~chinese 设备传输层协议类型       \~english Device Transport Layer Protocol Type
#define MV_UNKNOW_DEVICE \
  0x00000000  ///< \~chinese 未知设备类型，保留意义       \~english Unknown Device Type, Reserved
#define MV_GIGE_DEVICE 0x00000001  ///< \~chinese GigE设备                     \~english GigE Device
#define MV_1394_DEVICE \
  0x00000002  ///< \~chinese 1394-a/b 设备                \~english 1394-a/b Device
#define MV_USB_DEVICE 0x00000004  ///< \~chinese USB 设备                     \~english USB Device
#define MV_CAMERALINK_DEVICE \
  0x00000008  ///< \~chinese CameraLink设备               \~english CameraLink Device
#define MV_VIR_GIGE_DEVICE \
  0x00000010  ///< \~chinese 虚拟GigE设备                 \~english Virtual GigE Device
#define MV_VIR_USB_DEVICE \
  0x00000020  ///< \~chinese 虚拟USB设备                 \~english Virtual USB Device
#define MV_GENTL_GIGE_DEVICE \
  0x00000040  ///< \~chinese 自研网卡下GigE设备          \~english GenTL GigE Device

/// \~chinese 设备信息                  \~english Device info
typedef struct _MV_CC_DEVICE_INFO_
{
  unsigned short nMajorVer;   ///< [OUT] \~chinese 主要版本               \~english Major Version
  unsigned short nMinorVer;   ///< [OUT] \~chinese 次要版本               \~english Minor Version
  unsigned int nMacAddrHigh;  ///< [OUT] \~chinese 高MAC地址              \~english High MAC Address
  unsigned int nMacAddrLow;   ///< [OUT] \~chinese 低MAC地址              \~english Low MAC Address
  unsigned int
    nTLayerType;  ///< [OUT] \~chinese 设备传输层协议类型     \~english Device Transport Layer Protocol Type

  unsigned int nReserved[4];  ///<       \~chinese 预留                   \~english Reserved

  union {
    MV_GIGE_DEVICE_INFO
      stGigEInfo;  ///< [OUT] \~chinese GigE设备信息           \~english GigE Device Info
    MV_USB3_DEVICE_INFO
      stUsb3VInfo;  ///< [OUT] \~chinese USB设备信息            \~english USB Device Info
    MV_CamL_DEV_INFO
      stCamLInfo;  ///< [OUT] \~chinese CameraLink设备信息     \~english CameraLink Device Info
                   // more ...
  } SpecialInfo;

} MV_CC_DEVICE_INFO;

///< \~chinese 最多支持的传输层实例个数 \~english The maximum number of supported transport layer instances
#define MV_MAX_TLS_NUM 8
///< \~chinese 最大支持的设备个数       \~english The maximum number of supported devices
#define MV_MAX_DEVICE_NUM 256

/// \~chinese 设备信息列表              \~english Device Information List
typedef struct _MV_CC_DEVICE_INFO_LIST_
{
  unsigned int
    nDeviceNum;  ///< [OUT] \~chinese 在线设备数量           \~english Online Device Number
  MV_CC_DEVICE_INFO * pDeviceInfo
    [MV_MAX_DEVICE_NUM];  ///< [OUT] \~chinese 支持最多256个设备      \~english Support up to 256 devices

} MV_CC_DEVICE_INFO_LIST;

/// \~chinese 通过GenTL枚举到的接口信息 \~english Interface Information with GenTL
typedef struct _MV_GENTL_IF_INFO_
{
  unsigned char chInterfaceID
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese GenTL接口ID            \~english Interface ID
  unsigned char chTLType
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 传输层类型             \~english GenTL Type
  unsigned char chDisplayName
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese Interface显示名称      \~english Display Name
  unsigned int
    nCtiIndex;  ///< [OUT] \~chinese GenTL的cti文件索引     \~english The Index of Cti Files

  unsigned int nReserved[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_GENTL_IF_INFO;

///< \~chinese 最大支持的GenTL接口数量  \~english The maximum number of GenTL interface supported
#define MV_MAX_GENTL_IF_NUM 256

/// \~chinese 通过GenTL枚举到的接口信息列表 \~english Inferface Information List with GenTL
typedef struct _MV_GENTL_IF_INFO_LIST_
{
  unsigned int
    nInterfaceNum;  ///< [OUT] \~chinese 在线接口数量           \~english Online Inferface Number
  MV_GENTL_IF_INFO * pIFInfo
    [MV_MAX_GENTL_IF_NUM];  ///< [OUT] \~chinese 支持最多256个接口      \~english Support up to 256 inferfaces

} MV_GENTL_IF_INFO_LIST;

/// \~chinese 通过GenTL枚举到的设备信息 \~english Device Information with GenTL
typedef struct _MV_GENTL_DEV_INFO_
{
  unsigned char chInterfaceID
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese GenTL接口ID            \~english Interface ID
  unsigned char chDeviceID
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 设备ID                 \~english Device ID
  unsigned char chVendorName
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 供应商名字             \~english Vendor Name
  unsigned char chModelName
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 型号名字               \~english Model Name
  unsigned char chTLType
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 传输层类型             \~english GenTL Type
  unsigned char chDisplayName
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 设备显示名称           \~english Display Name
  unsigned char chUserDefinedName
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 用户自定义名字         \~english User Defined Name
  unsigned char chSerialNumber
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 序列号                 \~english Serial Number
  unsigned char chDeviceVersion
    [INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese 设备版本号             \~english Device Version
  unsigned int
    nCtiIndex;  ///< [OUT] \~chinese GenTL的cti文件索引     \~english The Index of Cti Files

  unsigned int nReserved[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_GENTL_DEV_INFO;

///< \~chinese 最大支持的GenTL设备数量  \~english The maximum number of GenTL devices supported
#define MV_MAX_GENTL_DEV_NUM 256

/// \~chinese 通过GenTL枚举到的设备信息列表 \~english Device Information List with GenTL
typedef struct _MV_GENTL_DEV_INFO_LIST_
{
  unsigned int
    nDeviceNum;  ///< [OUT] \~chinese 在线设备数量           \~english Online Device Number
  MV_GENTL_DEV_INFO * pDeviceInfo
    [MV_MAX_GENTL_DEV_NUM];  ///< [OUT] \~chinese 支持最多256个设备      \~english Support up to 256 devices

} MV_GENTL_DEV_INFO_LIST;

/// \~chinese 设备的访问模式            \~english Device Access Mode
#define MV_ACCESS_Exclusive \
  1  /// \~chinese 独占权限，其他APP只允许读CCP寄存器                    \~english Exclusive authority, other APP is only allowed to read the CCP register
#define MV_ACCESS_ExclusiveWithSwitch \
  2  /// \~chinese 可以从5模式下抢占权限，然后以独占权限打开             \~english You can seize the authority from the 5 mode, and then open with exclusive authority
#define MV_ACCESS_Control \
  3  /// \~chinese 控制权限，其他APP允许读所有寄存器                     \~english Control authority, allows other APP reading all registers
#define MV_ACCESS_ControlWithSwitch \
  4  /// \~chinese 可以从5的模式下抢占权限，然后以控制权限打开           \~english You can seize the authority from the 5 mode, and then open with control authority
#define MV_ACCESS_ControlSwitchEnable \
  5  /// \~chinese 以可被抢占的控制权限打开                              \~english Open with seized control authority
#define MV_ACCESS_ControlSwitchEnableWithKey \
  6  /// \~chinese 可以从5的模式下抢占权限，然后以可被抢占的控制权限打开 \~english You can seize the authority from the 5 mode, and then open with seized control authority
#define MV_ACCESS_Monitor \
  7  /// \~chinese 读模式打开设备，适用于控制权限下                      \~english Open with read mode and is available under control authority

/// \~chinese Chunk内容                 \~english The content of ChunkData
typedef struct _MV_CHUNK_DATA_CONTENT_
{
  unsigned char * pChunkData;  ///< [OUT] \~chinese Chunk数据              \~english Chunk Data
  unsigned int nChunkID;       ///< [OUT] \~chinese Chunk ID               \~english Chunk ID
  unsigned int nChunkLen;      ///< [OUT] \~chinese Chunk的长度            \~english Chunk Length

  unsigned int nReserved[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CHUNK_DATA_CONTENT;

/// \~chinese 输出帧的信息              \~english Output Frame Information
typedef struct _MV_FRAME_OUT_INFO_EX_
{
  unsigned short
    nWidth;  ///< [OUT] \~chinese 图像宽(最大65535，超出请用nExtendWidth)    \~english Image Width (over 65535, use nExtendWidth)
  unsigned short
    nHeight;  ///< [OUT] \~chinese 图像高(最大65535，超出请用nExtendHeight)   \~english Image Height(over 65535, use nExtendHeight)
  enum MvGvspPixelType
    enPixelType;  ///< [OUT] \~chinese 像素格式               \~english Pixel Type

  unsigned int nFrameNum;  ///< [OUT] \~chinese 帧号                   \~english Frame Number
  unsigned int
    nDevTimeStampHigh;  ///< [OUT] \~chinese 时间戳高32位           \~english Timestamp high 32 bits
  unsigned int
    nDevTimeStampLow;  ///< [OUT] \~chinese 时间戳低32位           \~english Timestamp low 32 bits
  unsigned int
    nReserved0;  ///< [OUT] \~chinese 保留，8字节对齐        \~english Reserved, 8-byte aligned
  int64_t
    nHostTimeStamp;  ///< [OUT] \~chinese 主机生成的时间戳       \~english Host-generated timestamp

  unsigned int nFrameLen;  ///< [OUT] \~chinese 帧的长度               \~english The Length of Frame

  /// \~chinese 设备水印时标      \~english Device frame-specific time scale
  unsigned int nSecondCount;  ///< [OUT] \~chinese 秒数                   \~english The Seconds
  unsigned int
    nCycleCount;  ///< [OUT] \~chinese 周期数                 \~english The Count of Cycle
  unsigned int
    nCycleOffset;  ///< [OUT] \~chinese 周期偏移量             \~english The Offset of Cycle

  float fGain;          ///< [OUT] \~chinese 增益                   \~english Gain
  float fExposureTime;  ///< [OUT] \~chinese 曝光时间               \~english Exposure Time
  unsigned int
    nAverageBrightness;  ///< [OUT] \~chinese 平均亮度               \~english Average brightness

  /// \~chinese 白平衡相关        \~english White balance
  unsigned int nRed;    ///< [OUT] \~chinese 红色                   \~english Red
  unsigned int nGreen;  ///< [OUT] \~chinese 绿色                   \~english Green
  unsigned int nBlue;   ///< [OUT] \~chinese 蓝色                   \~english Blue

  unsigned int nFrameCounter;  ///< [OUT] \~chinese 总帧数                 \~english Frame Counter
  unsigned int
    nTriggerIndex;  ///< [OUT] \~chinese 触发计数               \~english Trigger Counting

  unsigned int nInput;   ///< [OUT] \~chinese 输入                   \~english Input
  unsigned int nOutput;  ///< [OUT] \~chinese 输出                   \~english Output

  /// \~chinese ROI区域           \~english ROI Region
  unsigned short nOffsetX;  ///< [OUT] \~chinese 水平偏移量             \~english OffsetX
  unsigned short nOffsetY;  ///< [OUT] \~chinese 垂直偏移量             \~english OffsetY
  unsigned short
    nChunkWidth;  ///< [OUT] \~chinese Chunk宽                \~english The Width of Chunk
  unsigned short
    nChunkHeight;  ///< [OUT] \~chinese Chunk高                \~english The Height of Chunk

  unsigned int
    nLostPacket;  ///< [OUT] \~chinese 本帧丢包数             \~english Lost Packet Number In This Frame

  unsigned int
    nUnparsedChunkNum;  ///< [OUT] \~chinese 未解析的Chunkdata个数  \~english Unparsed Chunk Number
  union {
    MV_CHUNK_DATA_CONTENT *
      pUnparsedChunkContent;  ///< [OUT] \~chinese 未解析的Chunk          \~english Unparsed Chunk Content
    int64_t nAligning;        ///< [OUT] \~chinese 校准                   \~english Aligning
  } UnparsedChunkList;

  unsigned int nExtendWidth;   ///< [OUT] \~chinese 图像宽(扩展变量)       \~english Image Width
  unsigned int nExtendHeight;  ///< [OUT] \~chinese 图像高(扩展变量)       \~english Image Height

  unsigned int nReserved[34];  ///<       \~chinese 预留                   \~english Reserved

} MV_FRAME_OUT_INFO_EX;

/// \~chinese 图像结构体，输出图像地址及图像信息    \~english Image Struct, output the pointer of Image and the information of the specific image
typedef struct _MV_FRAME_OUT_
{
  unsigned char * pBufAddr;  ///< [OUT] \~chinese 图像指针地址           \~english  pointer of image
  MV_FRAME_OUT_INFO_EX
    stFrameInfo;  ///< [OUT] \~chinese 图像信息               \~english information of the specific image

  unsigned int nRes[16];  ///<       \~chinese 预留                   \~english Reserved

} MV_FRAME_OUT;

/// \~chinese 取流策略                  \~english The strategy of Grabbing
typedef enum _MV_GRAB_STRATEGY_
{
  MV_GrabStrategy_OneByOne =
    0,  ///< \~chinese 从旧到新一帧一帧的获取图像   \~english Grab One By One
  MV_GrabStrategy_LatestImagesOnly =
    1,  ///< \~chinese 获取列表中最新的一帧图像     \~english Grab The Latest Image
  MV_GrabStrategy_LatestImages =
    2,  ///< \~chinese 获取列表中最新的图像         \~english Grab The Latest Images
  MV_GrabStrategy_UpcomingImage =
    3,  ///< \~chinese 等待下一帧图像               \~english Grab The Upcoming Image

} MV_GRAB_STRATEGY;

/// \~chinese 网络传输的相关信息        \~english Network transmission information
typedef struct _MV_NETTRANS_INFO_
{
  int64_t
    nReceiveDataSize;  ///< [OUT] \~chinese 已接收数据大小[Start和Stop之间]    \~english Received Data Size
  int
    nThrowFrameCount;  ///< [OUT] \~chinese 丢帧数量                           \~english Throw frame number
  unsigned int
    nNetRecvFrameCount;  ///< [OUT] \~chinese 已接收的帧数                       \~english Received Frame Count
  int64_t
    nRequestResendPacketCount;  ///< [OUT] \~chinese 请求重发包数                       \~english Request Resend Packet Count
  int64_t
    nResendPacketCount;  ///< [OUT] \~chinese 重发包数                           \~english Resend Packet Count

} MV_NETTRANS_INFO;

/// \~chinese 信息类型                  \~english Information Type
#define MV_MATCH_TYPE_NET_DETECT \
  0x00000001  ///< \~chinese 网络流量和丢包信息               \~english Network traffic and packet loss information
#define MV_MATCH_TYPE_USB_DETECT \
  0x00000002  ///< \~chinese host接收到来自U3V设备的字节总数  \~english The total number of bytes host received from U3V device

/// \~chinese 全匹配的一种信息结构体    \~english A fully matched information structure
typedef struct _MV_ALL_MATCH_INFO_
{
  unsigned int
    nType;  ///< [IN]  \~chinese 需要输出的信息类型，e.g. MV_MATCH_TYPE_NET_DETECT  \~english Information type need to output ,e.g. MV_MATCH_TYPE_NET_DETECT
  void *
    pInfo;  ///< [OUT] \~chinese 输出的信息缓存，由调用者分配                       \~english Output information cache, which is allocated by the caller
  unsigned int
    nInfoSize;  ///< [IN]  \~chinese 信息缓存的大小                                     \~english Information cache size

} MV_ALL_MATCH_INFO;

/// \~chinese 网络流量和丢包信息反馈结构体，对应类型为 MV_MATCH_TYPE_NET_DETECT     \~english Network traffic and packet loss feedback structure, the corresponding type is MV_MATCH_TYPE_NET_DETECT
typedef struct _MV_MATCH_INFO_NET_DETECT_
{
  int64_t
    nReceiveDataSize;  ///< [OUT] \~chinese 已接收数据大小[Start和Stop之间]    \~english Received data size
  int64_t
    nLostPacketCount;  ///< [OUT] \~chinese 丢失的包数量                       \~english Number of packets lost
  unsigned int
    nLostFrameCount;  ///< [OUT] \~chinese 丢帧数量                           \~english Number of frames lost
  unsigned int
    nNetRecvFrameCount;  ///< [OUT] \~chinese 保留                               \~english Received Frame Count
  int64_t
    nRequestResendPacketCount;  ///< [OUT] \~chinese 请求重发包数                       \~english Request Resend Packet Count
  int64_t
    nResendPacketCount;  ///< [OUT] \~chinese 重发包数                           \~english Resend Packet Count

} MV_MATCH_INFO_NET_DETECT;

/// \~chinese host收到从u3v设备端的总字节数，对应类型为 MV_MATCH_TYPE_USB_DETECT    \~english The total number of bytes host received from the u3v device side, the corresponding type is MV_MATCH_TYPE_USB_DETECT
typedef struct _MV_MATCH_INFO_USB_DETECT_
{
  int64_t
    nReceiveDataSize;  ///< [OUT] \~chinese 已接收数据大小 [Open和Close之间]   \~english Received data size
  unsigned int
    nReceivedFrameCount;  ///< [OUT] \~chinese 已收到的帧数                       \~english Number of frames received
  unsigned int
    nErrorFrameCount;  ///< [OUT] \~chinese 错误帧数                           \~english Number of error frames

  unsigned int
    nReserved[2];  ///<       \~chinese 保留                               \~english Reserved

} MV_MATCH_INFO_USB_DETECT;

/// \~chinese 显示帧信息                \~english Display frame information
typedef struct _MV_DISPLAY_FRAME_INFO_
{
  void * hWnd;             ///< [IN] \~chinese 窗口句柄                \~english HWND
  unsigned char * pData;   ///< [IN] \~chinese 显示的数据              \~english Data Buffer
  unsigned int nDataLen;   ///< [IN] \~chinese 数据长度                \~english Data Size
  unsigned short nWidth;   ///< [IN] \~chinese 图像宽                  \~english Width
  unsigned short nHeight;  ///< [IN] \~chinese 图像高                  \~english Height
  enum MvGvspPixelType
    enPixelType;  ///< [IN] \~chinese 像素格式                \~english Pixel format

  unsigned int nRes[4];  ///<      \~chinese 保留                    \~english Reserved

} MV_DISPLAY_FRAME_INFO;

typedef struct _MV_DISPLAY_FRAME_INFO_EX_
{
  unsigned int nWidth;   ///< [IN]   \~chinese 图像宽                \~english Width
  unsigned int nHeight;  ///< [IN]   \~chinese 图像高                \~english Height
  enum MvGvspPixelType
    enPixelType;  ///< [IN]   \~chinese 像素格式              \~english Pixel type

  unsigned char *
    pImageBuf;  ///< [IN]   \~chinese 输入图像缓存          \~english Input image buffer
  unsigned int
    nImageBufLen;  ///< [IN]   \~chinese 输入图像长度          \~english Input image length

  unsigned int nRes[4];  ///<      \~chinese 保留                    \~english Reserved

} MV_DISPLAY_FRAME_INFO_EX;

/// \~chinese 保存的3D数据格式          \~english The saved format for 3D data
enum MV_SAVE_POINT_CLOUD_FILE_TYPE
{
  MV_PointCloudFile_Undefined =
    0,  ///< \~chinese 未定义的点云格式             \~english Undefined point cloud format
  MV_PointCloudFile_PLY =
    1,  ///< \~chinese PLY点云格式                  \~english The point cloud format named PLY
  MV_PointCloudFile_CSV =
    2,  ///< \~chinese CSV点云格式                  \~english The point cloud format named CSV
  MV_PointCloudFile_OBJ =
    3,  ///< \~chinese OBJ点云格式                  \~english The point cloud format named OBJ

};

/// \~chinese 保存3D数据到缓存          \~english Save 3D data to buffer
typedef struct _MV_SAVE_POINT_CLOUD_PARAM_
{
  unsigned int
    nLinePntNum;  ///< [IN]  \~chinese 行点数，即图像宽       \~english The number of points in each row,which is the width of the image
  unsigned int
    nLineNum;  ///< [IN]  \~chinese 行数，即图像高         \~english The number of rows,which is the height of the image

  enum MvGvspPixelType
    enSrcPixelType;  ///< [IN]  \~chinese 输入数据的像素格式     \~english The pixel format of the input data
  unsigned char * pSrcData;  ///< [IN]  \~chinese 输入数据缓存           \~english Input data buffer
  unsigned int nSrcDataLen;  ///< [IN]  \~chinese 输入数据长度           \~english Input data length

  unsigned char *
    pDstBuf;  ///< [OUT] \~chinese 输出像素数据缓存       \~english Output pixel data buffer
  unsigned int
    nDstBufSize;  ///< [IN]  \~chinese 提供的输出缓冲区大小(nLinePntNum * nLineNum * (16*3 + 4) + 2048)   \~english Output buffer size provided(nLinePntNum * nLineNum * (16*3 + 4) + 2048)
  unsigned int
    nDstBufLen;  ///< [OUT] \~chinese 输出像素数据缓存长度   \~english Output pixel data buffer size
  MV_SAVE_POINT_CLOUD_FILE_TYPE
    enPointCloudFileType;  ///< [IN]  \~chinese 提供输出的点云文件类型 \~english Output point data file type provided

  unsigned int nReserved[8];  ///<       \~chinese 保留字段               \~english Reserved

} MV_SAVE_POINT_CLOUD_PARAM;

/// \~chinese 保存图片格式              \~english Save image type
enum MV_SAVE_IAMGE_TYPE
{
  MV_Image_Undefined =
    0,                ///< \~chinese 未定义的图像格式             \~english Undefined Image Type
  MV_Image_Bmp = 1,   ///< \~chinese BMP图像格式                  \~english BMP Image Type
  MV_Image_Jpeg = 2,  ///< \~chinese JPEG图像格式                 \~english Jpeg Image Type
  MV_Image_Png = 3,   ///< \~chinese PNG图像格式                  \~english Png  Image Type
  MV_Image_Tif = 4,   ///< \~chinese TIFF图像格式                 \~english TIFF Image Type

};

/// \~chinese 图片保存参数              \~english Save Image Parameters
typedef struct _MV_SAVE_IMAGE_PARAM_EX3_
{
  unsigned char * pData;  ///< [IN]  \~chinese 输入数据缓存           \~english Input Data Buffer
  unsigned int nDataLen;  ///< [IN]  \~chinese 输入数据长度           \~english Input Data length
  enum MvGvspPixelType
    enPixelType;  ///< [IN]  \~chinese 输入数据的像素格式     \~english Input Data Pixel Format
  unsigned int nWidth;   ///< [IN]  \~chinese 图像宽                 \~english Image Width
  unsigned int nHeight;  ///< [IN]  \~chinese 图像高                 \~english Image Height

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

} MV_SAVE_IMAGE_PARAM_EX3;

// 保存图片到文件参数
typedef struct _MV_SAVE_IMAGE_TO_FILE_PARAM_EX_
{
  unsigned int nWidth;          // [IN]     图像宽
  unsigned int nHeight;         // [IN]     图像高
  MvGvspPixelType enPixelType;  // [IN]     输入数据的像素格式
  unsigned char * pData;        // [IN]     输入数据缓存
  unsigned int nDataLen;        // [IN]     输入数据大小

  MV_SAVE_IAMGE_TYPE enImageType;  // [IN]     输入图片格式
  char * pcImagePath;              // [IN]     输入文件路径

  unsigned int nQuality;  // [IN]     JPG编码质量(50-99]，PNG编码质量[0-9]
  int iMethodValue;       // [IN]     插值方法 0-快速 1-均衡 2-最优（其它值默认为最优）
  unsigned int nReserved[8];

} MV_SAVE_IMAGE_TO_FILE_PARAM_EX;

/// \~chinese 旋转角度                  \~english Rotation angle
typedef enum _MV_IMG_ROTATION_ANGLE_
{
  MV_IMAGE_ROTATE_90 = 1,
  MV_IMAGE_ROTATE_180 = 2,
  MV_IMAGE_ROTATE_270 = 3,

} MV_IMG_ROTATION_ANGLE;

/// \~chinese 图像旋转结构体            \~english Rotate image structure
typedef struct _MV_CC_ROTATE_IMAGE_PARAM_T_
{
  enum MvGvspPixelType
    enPixelType;         ///< [IN]  \~chinese 像素格式               \~english Pixel format
  unsigned int nWidth;   ///< [IN][OUT] \~chinese 图像宽             \~english Width
  unsigned int nHeight;  ///< [IN][OUT] \~chinese 图像高             \~english Height

  unsigned char * pSrcData;  ///< [IN]  \~chinese 输入数据缓存           \~english Input data buffer
  unsigned int nSrcDataLen;  ///< [IN]  \~chinese 输入数据长度           \~english Input data length

  unsigned char * pDstBuf;  ///< [OUT] \~chinese 输出数据缓存           \~english Output data buffer
  unsigned int nDstBufLen;  ///< [OUT] \~chinese 输出数据长度           \~english Output data length
  unsigned int
    nDstBufSize;  ///< [IN]  \~chinese 提供的输出缓冲区大小   \~english Provided output buffer size

  MV_IMG_ROTATION_ANGLE
    enRotationAngle;  ///< [IN]  \~chinese 旋转角度               \~english Rotation angle

  unsigned int nRes[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_ROTATE_IMAGE_PARAM;

/// \~chinese 翻转类型                  \~english Flip type
typedef enum _MV_IMG_FLIP_TYPE_
{
  MV_FLIP_VERTICAL = 1,
  MV_FLIP_HORIZONTAL = 2,

} MV_IMG_FLIP_TYPE;

/// \~chinese 图像翻转结构体            \~english Flip image structure
typedef struct _MV_CC_FLIP_IMAGE_PARAM_T_
{
  enum MvGvspPixelType
    enPixelType;         ///< [IN]  \~chinese 像素格式               \~english Pixel format
  unsigned int nWidth;   ///< [IN]  \~chinese 图像宽                 \~english Width
  unsigned int nHeight;  ///< [IN]  \~chinese 图像高                 \~english Height

  unsigned char * pSrcData;  ///< [IN]  \~chinese 输入数据缓存           \~english Input data buffer
  unsigned int nSrcDataLen;  ///< [IN]  \~chinese 输入数据长度           \~english Input data length

  unsigned char * pDstBuf;  ///< [OUT] \~chinese 输出数据缓存           \~english Output data buffer
  unsigned int nDstBufLen;  ///< [OUT] \~chinese 输出数据长度           \~english Output data length
  unsigned int
    nDstBufSize;  ///< [IN]  \~chinese 提供的输出缓冲区大小   \~english Provided output buffer size

  MV_IMG_FLIP_TYPE enFlipType;  ///< [IN]  \~chinese 翻转类型               \~english Flip type

  unsigned int nRes[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_FLIP_IMAGE_PARAM;

/// \~chinese 像素转换结构体            \~english Pixel convert structure
typedef struct _MV_CC_PIXEL_CONVERT_PARAM_EX_
{
  unsigned int nWidth;   ///< [IN]  \~chinese 图像宽                 \~english Width
  unsigned int nHeight;  ///< [IN]  \~chinese 图像高                 \~english Height

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

} MV_CC_PIXEL_CONVERT_PARAM_EX;

/// \~chinese Gamma类型                 \~english Gamma type
typedef enum _MV_CC_GAMMA_TYPE_
{
  MV_CC_GAMMA_TYPE_NONE = 0,   ///< \~chinese 不启用                       \~english Disable
  MV_CC_GAMMA_TYPE_VALUE = 1,  ///< \~chinese Gamma值                      \~english Gamma value
  MV_CC_GAMMA_TYPE_USER_CURVE =
    2,  ///< \~chinese Gamma曲线                    \~english Gamma curve
  ///< \~chinese 8位,长度：256*sizeof(unsigned char)      \~english 8bit,length:256*sizeof(unsigned char)
  ///< \~chinese 10位,长度：1024*sizeof(unsigned short)   \~english 10bit,length:1024*sizeof(unsigned short)
  ///< \~chinese 12位,长度：4096*sizeof(unsigned short)   \~english 12bit,length:4096*sizeof(unsigned short)
  ///< \~chinese 16位,长度：65536*sizeof(unsigned short)  \~english 16bit,length:65536*sizeof(unsigned short)
  MV_CC_GAMMA_TYPE_LRGB2SRGB =
    3,  ///< \~chinese linear RGB to sRGB           \~english linear RGB to sRGB
  MV_CC_GAMMA_TYPE_SRGB2LRGB =
    4,  ///< \~chinese sRGB to linear RGB(仅色彩插值时支持，色彩校正时无效) \~english sRGB to linear RGB

} MV_CC_GAMMA_TYPE;

// Gamma信息
/// \~chinese Gamma信息结构体           \~english Gamma info structure
typedef struct _MV_CC_GAMMA_PARAM_T_
{
  MV_CC_GAMMA_TYPE enGammaType;  ///< [IN]  \~chinese Gamma类型              \~english Gamma type
  float fGammaValue;  ///< [IN]  \~chinese Gamma值:0.1 ~ 4.0       \~english Gamma value:0.1 ~ 4.0
  unsigned char *
    pGammaCurveBuf;  ///< [IN]  \~chinese Gamma曲线缓存          \~english Gamma curve buffer
  unsigned int
    nGammaCurveBufLen;  ///< [IN]  \~chinese Gamma曲线长度          \~english Gamma curve buffer size

  unsigned int nRes[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_GAMMA_PARAM;

/// \~chinese CCM参数                   \~english CCM param
typedef struct _MV_CC_CCM_PARAM_T_
{
  bool bCCMEnable;  ///< [IN]  \~chinese 是否启用CCM            \~english CCM enable
  int nCCMat
    [9];  ///< [IN]  \~chinese CCM矩阵(-8192~8192)    \~english Color correction matrix(-8192~8192)

  unsigned int nRes[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_CCM_PARAM;

/// \~chinese CCM参数                   \~english CCM param
typedef struct _MV_CC_CCM_PARAM_EX_T_
{
  bool bCCMEnable;  ///< [IN]  \~chinese 是否启用CCM            \~english CCM enable
  int nCCMat
    [9];  ///< [IN]  \~chinese CCM矩阵(-65536~65536)  \~english Color correction matrix(-65536~65536)
  unsigned int
    nCCMScale;  ///< [IN]  \~chinese 量化系数（2的整数幂,最大65536）    \~english Quantitative scale(Integer power of 2, <= 65536)

  unsigned int nRes[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_CCM_PARAM_EX;

/// \~chinese 对比度调节结构体          \~english Contrast structure
typedef struct _MV_CC_CONTRAST_PARAM_T_
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
    nContrastFactor;  ///< [IN]  \~chinese 对比度值，[1,10000]     \~english Contrast factor,[1,10000]

  unsigned int nRes[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_CONTRAST_PARAM;

/// \~chinese 水印信息                  \~english  Frame-specific information
typedef struct _MV_CC_FRAME_SPEC_INFO_
{
  /// \~chinese 设备水印时标      \~english Device frame-specific time scale
  unsigned int nSecondCount;  ///< [OUT] \~chinese 秒数                   \~english The Seconds
  unsigned int
    nCycleCount;  ///< [OUT] \~chinese 周期数                 \~english The Count of Cycle
  unsigned int
    nCycleOffset;  ///< [OUT] \~chinese 周期偏移量             \~english The Offset of Cycle

  float fGain;          ///< [OUT] \~chinese 增益                   \~english Gain
  float fExposureTime;  ///< [OUT] \~chinese 曝光时间               \~english Exposure Time
  unsigned int
    nAverageBrightness;  ///< [OUT] \~chinese 平均亮度               \~english Average brightness

  /// \~chinese 白平衡相关        \~english White balance
  unsigned int nRed;    ///< [OUT] \~chinese 红色                   \~english Red
  unsigned int nGreen;  ///< [OUT] \~chinese 绿色                   \~english Green
  unsigned int nBlue;   ///< [OUT] \~chinese 蓝色                   \~english Blue

  unsigned int nFrameCounter;  ///< [OUT] \~chinese 总帧数                 \~english Frame Counter
  unsigned int
    nTriggerIndex;  ///< [OUT] \~chinese 触发计数               \~english Trigger Counting

  unsigned int nInput;   ///< [OUT] \~chinese 输入                   \~english Input
  unsigned int nOutput;  ///< [OUT] \~chinese 输出                   \~english Output

  /// \~chinese ROI区域           \~english ROI Region
  unsigned short nOffsetX;  ///< [OUT] \~chinese 水平偏移量             \~english OffsetX
  unsigned short nOffsetY;  ///< [OUT] \~chinese 垂直偏移量             \~english OffsetY
  unsigned short
    nFrameWidth;  ///< [OUT] \~chinese 水印宽                 \~english The Width of Chunk
  unsigned short
    nFrameHeight;  ///< [OUT] \~chinese 水印高                 \~english The Height of Chunk

  unsigned int nReserved[16];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_FRAME_SPEC_INFO;

/// \~chinese 无损解码参数              \~english High Bandwidth decode structure
typedef struct _MV_CC_HB_DECODE_PARAM_T_
{
  unsigned char * pSrcBuf;  ///< [IN]  \~chinese 输入数据缓存           \~english Input data buffer
  unsigned int nSrcLen;     ///< [IN]  \~chinese 输入数据大小           \~english Input data size

  unsigned int nWidth;      ///< [OUT] \~chinese 图像宽                 \~english Width
  unsigned int nHeight;     ///< [OUT] \~chinese 图像高                 \~english Height
  unsigned char * pDstBuf;  ///< [OUT] \~chinese 输出数据缓存           \~english Output data buffer
  unsigned int
    nDstBufSize;  ///< [IN]  \~chinese 提供的输出缓冲区大小   \~english Provided output buffer size
  unsigned int nDstBufLen;  ///< [OUT] \~chinese 输出数据大小           \~english Output data size
  enum MvGvspPixelType
    enDstPixelType;  ///< [OUT] \~chinese 输出的像素格式         \~english Output pixel format

  MV_CC_FRAME_SPEC_INFO
    stFrameSpecInfo;  ///< [OUT] \~chinese 水印信息               \~english Frame Spec Info

  unsigned int nRes[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_HB_DECODE_PARAM;

/// \~chinese 录像格式定义              \~english Record Format Type
typedef enum _MV_RECORD_FORMAT_TYPE_
{
  MV_FormatType_Undefined =
    0,  ///< \~chinese 未定义的录像格式             \~english Undefined Recode Format Type
  MV_FormatType_AVI =
    1,  ///< \~chinese AVI录像格式                  \~english AVI Recode Format Type

} MV_RECORD_FORMAT_TYPE;

/// \~chinese 录像参数                  \~english Record Parameters
typedef struct _MV_CC_RECORD_PARAM_T_
{
  enum MvGvspPixelType
    enPixelType;  ///< [IN]  \~chinese 输入数据的像素格式     \~english Pixel Type

  unsigned short nWidth;   ///< [IN]  \~chinese 图像宽(2的倍数)        \~english Width
  unsigned short nHeight;  ///< [IN]  \~chinese 图像高(2的倍数)        \~english Height

  float fFrameRate;       ///< [IN]  \~chinese 帧率fps(大于1/16)      \~english The Rate of Frame
  unsigned int nBitRate;  ///< [IN]  \~chinese 码率kbps(128-16*1024)  \~english The Rate of Bitrate

  MV_RECORD_FORMAT_TYPE
    enRecordFmtType;  ///< [IN]  \~chinese 录像格式               \~english Recode Format Type

  char *
    strFilePath;  ///< [IN]  \~chinese 录像文件存放路径(如果路径中存在中文，需转成utf-8)  \~english File Path

  unsigned int nRes[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_RECORD_PARAM;

/// \~chinese 传入的图像数据            \~english Input Data
typedef struct _MV_CC_INPUT_FRAME_INFO_T_
{
  unsigned char * pData;  ///< [IN]  \~chinese 图像数据指针           \~english Record Data
  unsigned int
    nDataLen;  ///< [IN]  \~chinese 图像大小               \~english The Length of Record Data

  unsigned int nRes[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_INPUT_FRAME_INFO;

/// \~chinese 采集模式                  \~english Acquisition mode
typedef enum _MV_CAM_ACQUISITION_MODE_
{
  MV_ACQ_MODE_SINGLE = 0,      ///< \~chinese 单帧模式                     \~english Single Mode
  MV_ACQ_MODE_MUTLI = 1,       ///< \~chinese 多帧模式                     \~english Multi Mode
  MV_ACQ_MODE_CONTINUOUS = 2,  ///< \~chinese 持续采集模式                 \~english Continuous Mode

} MV_CAM_ACQUISITION_MODE;

/// \~chinese 增益模式                  \~english Gain Mode
typedef enum _MV_CAM_GAIN_MODE_
{
  MV_GAIN_MODE_OFF = 0,   ///< \~chinese 关闭                         \~english Single Mode
  MV_GAIN_MODE_ONCE = 1,  ///< \~chinese 一次                         \~english Multi Mode
  MV_GAIN_MODE_CONTINUOUS =
    2,  ///< \~chinese 连续                         \~english Continuous Mode

} MV_CAM_GAIN_MODE;

/// \~chinese 曝光模式                  \~english Exposure Mode
typedef enum _MV_CAM_EXPOSURE_MODE_
{
  MV_EXPOSURE_MODE_TIMED = 0,  ///< \~chinese 时间                         \~english Timed
  MV_EXPOSURE_MODE_TRIGGER_WIDTH =
    1,  ///< \~chinese 触发脉冲宽度                 \~english TriggerWidth
} MV_CAM_EXPOSURE_MODE;

/// \~chinese 自动曝光模式              \~english Auto Exposure Mode
typedef enum _MV_CAM_EXPOSURE_AUTO_MODE_
{
  MV_EXPOSURE_AUTO_MODE_OFF = 0,   ///< \~chinese 关闭                         \~english Off
  MV_EXPOSURE_AUTO_MODE_ONCE = 1,  ///< \~chinese 一次                         \~english Once
  MV_EXPOSURE_AUTO_MODE_CONTINUOUS =
    2,  ///< \~chinese 连续                         \~english Continuous

} MV_CAM_EXPOSURE_AUTO_MODE;

/// \~chinese 触发模式                  \~english Trigger Mode
typedef enum _MV_CAM_TRIGGER_MODE_
{
  MV_TRIGGER_MODE_OFF = 0,  ///< \~chinese 关闭                         \~english Off
  MV_TRIGGER_MODE_ON = 1,   ///< \~chinese 打开                         \~english ON

} MV_CAM_TRIGGER_MODE;

/// \~chinese Gamma选择器               \~english Gamma Selector
typedef enum _MV_CAM_GAMMA_SELECTOR_
{
  MV_GAMMA_SELECTOR_USER =
    1,  ///< \~chinese 用户                         \~english Gamma Selector User
  MV_GAMMA_SELECTOR_SRGB =
    2,  ///< \~chinese sRGB                         \~english Gamma Selector sRGB

} MV_CAM_GAMMA_SELECTOR;

/// \~chinese 白平衡                    \~english White Balance
typedef enum _MV_CAM_BALANCEWHITE_AUTO_
{
  MV_BALANCEWHITE_AUTO_OFF = 0,   ///< \~chinese 关闭                         \~english Off
  MV_BALANCEWHITE_AUTO_ONCE = 2,  ///< \~chinese 一次                         \~english Once
  MV_BALANCEWHITE_AUTO_CONTINUOUS =
    1,  ///< \~chinese 连续                         \~english Continuous

} MV_CAM_BALANCEWHITE_AUTO;

/// \~chinese 触发源                    \~english Trigger Source
typedef enum _MV_CAM_TRIGGER_SOURCE_
{
  MV_TRIGGER_SOURCE_LINE0 = 0,     ///< \~chinese Line0                        \~english Line0
  MV_TRIGGER_SOURCE_LINE1 = 1,     ///< \~chinese Line1                        \~english Line1
  MV_TRIGGER_SOURCE_LINE2 = 2,     ///< \~chinese Line2                        \~english Line2
  MV_TRIGGER_SOURCE_LINE3 = 3,     ///< \~chinese Line3                        \~english Line3
  MV_TRIGGER_SOURCE_COUNTER0 = 4,  ///< \~chinese Conuter0                     \~english Conuter0

  MV_TRIGGER_SOURCE_SOFTWARE = 7,  ///< \~chinese 软触发                       \~english Software
  MV_TRIGGER_SOURCE_FrequencyConverter =
    8,  ///< \~chinese 变频器                       \~english Frequency Converter

} MV_CAM_TRIGGER_SOURCE;

/// \~chinese GigEVision IP配置         \~english GigEVision IP Configuration
#define MV_IP_CFG_STATIC 0x05000000  ///< \~chinese 静态                         \~english Static
#define MV_IP_CFG_DHCP 0x06000000    ///< \~chinese DHCP                         \~english DHCP
#define MV_IP_CFG_LLA 0x04000000     ///< \~chinese LLA                          \~english LLA

/// \~chinese GigEVision网络传输模式    \~english GigEVision Net Transfer Mode
#define MV_NET_TRANS_DRIVER 0x00000001  ///< \~chinese 驱动                         \~english Driver
#define MV_NET_TRANS_SOCKET 0x00000002  ///< \~chinese Socket                       \~english Socket

/// \~chinese CameraLink波特率          \~english CameraLink Baud Rates (CLUINT32)
#define MV_CAML_BAUDRATE_9600 0x00000001  ///< \~chinese 9600                         \~english 9600
#define MV_CAML_BAUDRATE_19200 \
  0x00000002  ///< \~chinese 19200                        \~english 19200
#define MV_CAML_BAUDRATE_38400 \
  0x00000004  ///< \~chinese 38400                        \~english 38400
#define MV_CAML_BAUDRATE_57600 \
  0x00000008  ///< \~chinese 57600                        \~english 57600
#define MV_CAML_BAUDRATE_115200 \
  0x00000010  ///< \~chinese 115200                       \~english 115200
#define MV_CAML_BAUDRATE_230400 \
  0x00000020  ///< \~chinese 230400                       \~english 230400
#define MV_CAML_BAUDRATE_460800 \
  0x00000040  ///< \~chinese 460800                       \~english 460800
#define MV_CAML_BAUDRATE_921600 \
  0x00000080  ///< \~chinese 921600                       \~english 921600
#define MV_CAML_BAUDRATE_AUTOMAX \
  0x40000000  ///< \~chinese 最大值                       \~english Auto Max

/// \~chinese 异常消息类型              \~english Exception message type
#define MV_EXCEPTION_DEV_DISCONNECT \
  0x00008001  ///< \~chinese 设备断开连接                 \~english The device is disconnected
#define MV_EXCEPTION_VERSION_CHECK \
  0x00008002  ///< \~chinese SDK与驱动版本不匹配          \~english SDK does not match the driver version

/// \~chinese U3V流异常类型
typedef enum _MV_CC_STREAM_EXCEPTION_TYPE_
{
  MV_CC_STREAM_EXCEPTION_ABNORMAL_IMAGE = 0x4001,  ///< \~chinese 异常的图像，该帧被丢弃
  MV_CC_STREAM_EXCEPTION_LIST_OVERFLOW = 0x4002,   ///< \~chinese 缓存列表溢出，清除最旧的一帧
  MV_CC_STREAM_EXCEPTION_LIST_EMPTY = 0x4003,      ///< \~chinese 缓存列表为空，该帧被丢弃
  MV_CC_STREAM_EXCEPTION_RECONNECTION = 0x4004,    ///< \~chinese 断流恢复
  MV_CC_STREAM_EXCEPTION_DISCONNECTED = 0x4005,    ///< \~chinese 断流,恢复失败,取流被中止
  MV_CC_STREAM_EXCEPTION_DEVICE = 0x4006,          ///< \~chinese 设备异常,取流被中止

} MV_CC_STREAM_EXCEPTION_TYPE;

///< \~chinese 设备Event事件名称最大长度    \~english Max length of event name
#define MAX_EVENT_NAME_SIZE 128

/// \~chinese Event事件回调信息\        \~english Event callback infomation
typedef struct _MV_EVENT_OUT_INFO_
{
  char EventName
    [MAX_EVENT_NAME_SIZE];  ///< [OUT] \~chinese Event名称              \~english Event name

  unsigned short nEventID;  ///< [OUT] \~chinese Event号                \~english Event ID
  unsigned short
    nStreamChannel;  ///< [OUT] \~chinese 流通道序号             \~english Circulation number

  unsigned int nBlockIdHigh;  ///< [OUT] \~chinese 帧号高位               \~english BlockId high
  unsigned int nBlockIdLow;   ///< [OUT] \~chinese 帧号低位               \~english BlockId low

  unsigned int
    nTimestampHigh;            ///< [OUT] \~chinese 时间戳高位             \~english Timestramp high
  unsigned int nTimestampLow;  ///< [OUT] \~chinese 时间戳低位             \~english Timestramp low

  void * pEventData;            ///< [OUT] \~chinese Event数据              \~english Event data
  unsigned int nEventDataSize;  ///< [OUT] \~chinese Event数据长度          \~english Event data len

  unsigned int nReserved[16];  ///<       \~chinese 预留                   \~english Reserved

} MV_EVENT_OUT_INFO;

/// \~chinese 文件存取                  \~english File Access
typedef struct _MV_CC_FILE_ACCESS_T
{
  const char * pUserFileName;  ///< [IN]  \~chinese 用户文件名             \~english User file name
  const char * pDevFileName;  ///< [IN]  \~chinese 设备文件名             \~english Device file name

  unsigned int nReserved[32];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_FILE_ACCESS;

/// \~chinese 文件存取                  \~english File Access
typedef struct _MV_CC_FILE_ACCESS_E
{
  char *
    pUserFileBuf;  ///< [IN]       \~chinese 用户数据缓存                                                                 \~english User data Buffer
  unsigned int
    pFileBufSize;  ///< [IN]       \~chinese 用户数据缓存大小                                                             \~english data buffer size
  unsigned int
    pFileBufLen;  ///< [OUT][IN]  \~chinese 文件数据缓存总长度(读取时输出文件数据总长度,写入时输入文件数据总长度)          \~english data buffer len
  const char *
    pDevFileName;  ///< [IN]       \~chinese 设备文件名                                                                    \~english Device file name

  unsigned int nReserved
    [32];  ///<           \~chinese 预留                                                                          \~english Reserved
} MV_CC_FILE_ACCESS_EX;

/// \~chinese 文件存取进度              \~english File Access Progress
typedef struct _MV_CC_FILE_ACCESS_PROGRESS_T
{
  int64_t nCompleted;  ///< [OUT] \~chinese 已完成的长度           \~english Completed Length
  int64_t nTotal;      ///< [OUT] \~chinese 总长度                 \~english Total Length

  unsigned int nReserved[8];  ///<       \~chinese 预留                   \~english Reserved

} MV_CC_FILE_ACCESS_PROGRESS;

/// \~chinese Gige的传输类型            \~english The transmission type of Gige
typedef enum _MV_GIGE_TRANSMISSION_TYPE_
{
  MV_GIGE_TRANSTYPE_UNICAST =
    0x0,  ///< \~chinese 表示单播(默认)                              \~english Unicast mode
  MV_GIGE_TRANSTYPE_MULTICAST =
    0x1,  ///< \~chinese 表示组播(组播IP范围[224.*.*.*-239.*.*.*])   \~english Multicast mode
  MV_GIGE_TRANSTYPE_LIMITEDBROADCAST =
    0x2,  ///< \~chinese 表示局域网内广播，暂不支持               \~english Limited broadcast mode,not support
  MV_GIGE_TRANSTYPE_SUBNETBROADCAST =
    0x3,  ///< \~chinese 表示子网内广播，暂不支持                 \~english Subnet broadcast mode,not support
  MV_GIGE_TRANSTYPE_CAMERADEFINED =
    0x4,  ///< \~chinese 表示从设备获取，暂不支持                 \~english Transtype from camera,not support
  MV_GIGE_TRANSTYPE_UNICAST_DEFINED_PORT =
    0x5,  ///< \~chinese 表示用户自定义应用端接收图像数据Port号   \~english User Defined Receive Data Port
  MV_GIGE_TRANSTYPE_UNICAST_WITHOUT_RECV =
    0x00010000,  ///< \~chinese 表示设置了单播，但本实例不接收图像数据   \~english Unicast without receive data
  MV_GIGE_TRANSTYPE_MULTICAST_WITHOUT_RECV =
    0x00010001,  ///< \~chinese 表示组播模式，但本实例不接收图像数据     \~english Multicast without receive data

} MV_GIGE_TRANSMISSION_TYPE;

/// \~chinese 网络传输模式              \~english Transmission type
typedef struct _MV_TRANSMISSION_TYPE_T
{
  MV_GIGE_TRANSMISSION_TYPE
    enTransmissionType;  ///< [IN]  \~chinese 传输模式                   \~english Transmission type
  unsigned int nDestIp;  ///< [IN]  \~chinese 目标IP，组播模式下有意义   \~english Destination IP
  unsigned short
    nDestPort;  ///< [IN]  \~chinese 目标Port，组播模式下有意义 \~english Destination port

  unsigned int nReserved[32];  ///<       \~chinese 预留                       \~english Reserved

} MV_TRANSMISSION_TYPE;

/// \~chinese 动作命令信息              \~english Action Command
typedef struct _MV_ACTION_CMD_INFO_T
{
  unsigned int
    nDeviceKey;  ///< [IN]  \~chinese 设备密钥                                   \~english Device Key;
  unsigned int
    nGroupKey;  ///< [IN]  \~chinese 组键                                       \~english Group Key
  unsigned int
    nGroupMask;  ///< [IN]  \~chinese 组掩码                                     \~english Group Mask

  unsigned int
    bActionTimeEnable;  ///< [IN]  \~chinese 只有设置成1时Action Time才有效，非1时无效  \~english Action Time Enable
  int64_t
    nActionTime;  ///< [IN]  \~chinese 预定的时间，和主频有关                     \~english Action Time

  const char *
    pBroadcastAddress;  ///< [IN]  \~chinese 广播包地址                                 \~english Broadcast Address
  unsigned int
    nTimeOut;  ///< [IN]  \~chinese 等待ACK的超时时间，如果为0表示不需要ACK    \~english TimeOut

  unsigned int
    bSpecialNetEnable;  ///< [IN]  \~chinese 只有设置成1时指定的网卡IP才有效，非1时无效 \~english Special IP Enable
  unsigned int
    nSpecialNetIP;  ///< [IN]  \~chinese 指定的网卡IP                               \~english Special Net IP address

  unsigned int nReserved
    [14];  ///<       \~chinese 预留                                       \~english Reserved

} MV_ACTION_CMD_INFO;

/// \~chinese 动作命令返回信息          \~english Action Command Result
typedef struct _MV_ACTION_CMD_RESULT_T
{
  unsigned char strDeviceAddress
    [12 + 3 + 1];  ///< [OUT] \~chinese 设备IP                 \~english IP address of the device

  int
    nStatus;  ///< [OUT] \~chinese 状态码                 \~english status code returned by the device
              //1.0x0000:success.
              //2.0x8001:Command is not supported by the device.
  //3.0x8013:The device is not synchronized to a master clock to be used as time reference.
  //4.0x8015:A device queue or packet data has overflowed.
  //5.0x8016:The requested scheduled action command was requested at a time that is already past.

  unsigned int nReserved[4];  ///<      \~chinese 预留                    \~english Reserved

} MV_ACTION_CMD_RESULT;

/// \~chinese 动作命令返回信息列表      \~english Action Command Result List
typedef struct _MV_ACTION_CMD_RESULT_LIST_T
{
  unsigned int
    nNumResults;  ///< [OUT] \~chinese 返回值个数             \~english Number of returned values
  MV_ACTION_CMD_RESULT *
    pResults;  ///< [OUT] \~chinese 动作命令结果           \~english Reslut of action command

} MV_ACTION_CMD_RESULT_LIST;

/// \~chinese 每个节点对应的接口类型    \~english Interface type corresponds to each node
enum MV_XML_InterfaceType
{
  IFT_IValue,        ///< \~chinese Value                        \~english IValue interface
  IFT_IBase,         ///< \~chinese Base                         \~english IBase interface
  IFT_IInteger,      ///< \~chinese Integer                      \~english IInteger interface
  IFT_IBoolean,      ///< \~chinese Boolean                      \~english IBoolean interface
  IFT_ICommand,      ///< \~chinese Command                      \~english ICommand interface
  IFT_IFloat,        ///< \~chinese Float                        \~english IFloat interface
  IFT_IString,       ///< \~chinese String                       \~english IString interface
  IFT_IRegister,     ///< \~chinese Register                     \~english IRegister interface
  IFT_ICategory,     ///< \~chinese Category                     \~english ICategory interface
  IFT_IEnumeration,  ///< \~chinese Enumeration                  \~english IEnumeration interface
  IFT_IEnumEntry,    ///< \~chinese EnumEntry                    \~english IEnumEntry interface
  IFT_IPort,         ///< \~chinese Port                         \~english IPort interface
};

/// \~chinese 节点的访问模式            \~english Node Access Mode
enum MV_XML_AccessMode
{
  AM_NI,         ///< \~chinese 不可实现                     \~english Not implemented
  AM_NA,         ///< \~chinese 不可用                       \~english Not available
  AM_WO,         ///< \~chinese 只写                         \~english Write Only
  AM_RO,         ///< \~chinese 只读                         \~english Read Only
  AM_RW,         ///< \~chinese 读写                         \~english Read and Write
  AM_Undefined,  ///< \~chinese 未定义                       \~english Object is not yet initialized
  AM_CycleDetect,  ///< \~chinese 内部用于AccessMode循环检测   \~english used internally for AccessMode cycle detection
};

/// \~chinese 最大XML符号数             \~english Max XML Symbolic Number
#define MV_MAX_XML_SYMBOLIC_NUM 64
/// \~chinese 枚举类型值                \~english Enumeration Value
typedef struct _MVCC_ENUMVALUE_T
{
  unsigned int nCurValue;  ///< [OUT] \~chinese 当前值                 \~english Current Value
  unsigned int
    nSupportedNum;  ///< [OUT] \~chinese 数据的有效数据个数     \~english Number of valid data
  unsigned int nSupportValue
    [MV_MAX_XML_SYMBOLIC_NUM];  ///< [OUT] \~chinese 支持的枚举值           \~english Support Value

  unsigned int nReserved[4];  ///<       \~chinese 预留                   \~english Reserved

} MVCC_ENUMVALUE;

/// \~chinese 最大枚举条目对应的符号长度            \~english Max Enum Entry Symbolic Number
#define MV_MAX_SYMBOLIC_LEN 64
/// \~chinese 枚举类型条目                          \~english Enumeration Entry
typedef struct _MVCC_ENUMENTRY_T
{
  unsigned int nValue;  ///< [IN]  \~chinese 指定值                 \~english Value
  char
    chSymbolic[MV_MAX_SYMBOLIC_LEN];  ///< [OUT] \~chinese 指定值对应的符号       \~english Symbolic

  unsigned int nReserved[4];  ///< \~chinese 预留                         \~english Reserved

} MVCC_ENUMENTRY;

/// \~chinese Int类型值                 \~english Int Value
typedef struct _MVCC_INTVALUE_T
{
  unsigned int nCurValue;  ///< [OUT] \~chinese 当前值                 \~english Current Value
  unsigned int nMax;       ///< [OUT] \~chinese 最大值                 \~english Max
  unsigned int nMin;       ///< [OUT] \~chinese 最小值                 \~english Min
  unsigned int nInc;       ///< [OUT] \~chinese                        \~english Inc

  unsigned int nReserved[4];  ///<       \~chinese 预留                   \~english Reserved

} MVCC_INTVALUE;

/// \~chinese Int类型值Ex               \~english Int Value Ex
typedef struct _MVCC_INTVALUE_EX_T
{
  int64_t nCurValue;  ///< [OUT] \~chinese 当前值                 \~english Current Value
  int64_t nMax;       ///< [OUT] \~chinese 最大值                 \~english Max
  int64_t nMin;       ///< [OUT] \~chinese 最小值                 \~english Min
  int64_t nInc;       ///< [OUT] \~chinese Inc                    \~english Inc

  unsigned int nReserved[16];  ///<       \~chinese 预留                   \~english Reserved

} MVCC_INTVALUE_EX;

/// \~chinese Float类型值               \~english Float Value
typedef struct _MVCC_FLOATVALUE_T
{
  float fCurValue;  ///< [OUT] \~chinese 当前值                 \~english Current Value
  float fMax;       ///< [OUT] \~chinese 最大值                 \~english Max
  float fMin;       ///< [OUT] \~chinese 最小值                 \~english Min

  unsigned int nReserved[4];  ///<       \~chinese 预留                   \~english Reserved

} MVCC_FLOATVALUE;

/// \~chinese String类型值              \~english String Value
typedef struct _MVCC_STRINGVALUE_T
{
  char chCurValue[256];  ///< [OUT] \~chinese 当前值                 \~english Current Value

  int64_t nMaxLength;         ///< [OUT] \~chinese 最大长度               \~english MaxLength
  unsigned int nReserved[2];  ///<       \~chinese 预留                   \~english Reserved

} MVCC_STRINGVALUE;

/// \~chinese 辅助线颜色                \~english Color of Auxiliary Line
typedef struct _MVCC_COLORF
{
  float
    fR;  ///< [IN] \~chinese 红色，根据像素颜色的相对深度，范围为[0.0 , 1.0]，代表着[0, 255]的颜色深度   \~english Red，Range[0.0, 1.0]
  float
    fG;  ///< [IN] \~chinese 绿色，根据像素颜色的相对深度，范围为[0.0 , 1.0]，代表着[0, 255]的颜色深度   \~english Green，Range[0.0, 1.0]
  float
    fB;  ///< [IN] \~chinese 蓝色，根据像素颜色的相对深度，范围为[0.0 , 1.0]，代表着[0, 255]的颜色深度   \~english Blue，Range[0.0, 1.0]
  float
    fAlpha;  ///< [IN] \~chinese 透明度，根据像素颜色的相对透明度，范围为[0.0 , 1.0] (此参数功能暂不支持)    \~english Alpha，Range[0.0, 1.0](Not Support)
  unsigned int nReserved[4];  ///<      \~chinese 预留                        \~english Reserved

} MVCC_COLORF;

/// \~chinese 自定义点                    \~english Point defined
typedef struct _MVCC_POINTF
{
  float
    fX;  ///< [IN] \~chinese 该点距离图像左边缘距离，根据图像的相对位置，范围为[0.0 , 1.0]   \~english Distance From Left，Range[0.0, 1.0]
  float
    fY;  ///< [IN] \~chinese 该点距离图像上边缘距离，根据图像的相对位置，范围为[0.0 , 1.0]   \~english Distance From Top，Range[0.0, 1.0]
  unsigned int nReserved[4];  ///<      \~chinese 预留                        \~english Reserved

} MVCC_POINTF;

/// \~chinese 矩形框区域信息            \~english Rect Area Info
typedef struct _MVCC_RECT_INFO
{
  float
    fTop;  ///< [IN] \~chinese 矩形上边缘距离图像上边缘的距离，根据图像的相对位置，范围为[0.0 , 1.0]   \~english Distance From Top，Range[0, 1.0]
  float
    fBottom;  ///< [IN] \~chinese 矩形下边缘距离图像上边缘的距离，根据图像的相对位置，范围为[0.0 , 1.0]   \~english Distance From Top，Range[0, 1.0]
  float
    fLeft;  ///< [IN] \~chinese 矩形左边缘距离图像左边缘的距离，根据图像的相对位置，范围为[0.0 , 1.0]   \~english Distance From Left，Range[0, 1.0]
  float
    fRight;  ///< [IN] \~chinese 矩形右边缘距离图像左边缘的距离，根据图像的相对位置，范围为[0.0 , 1.0]   \~english Distance From Left，Range[0, 1.0]

  MVCC_COLORF
    stColor;  ///< [IN] \~chinese 辅助线颜色                      \~english Color of Auxiliary Line
  unsigned int
    nLineWidth;  ///< [IN] \~chinese 辅助线宽度，宽度只能是1或2      \~english Width of Auxiliary Line, width is 1 or 2
  unsigned int nReserved[4];  ///<      \~chinese 预留                            \~english Reserved

} MVCC_RECT_INFO;

/// \~chinese 圆形框区域信息            \~english Circle Area Info
typedef struct _MVCC_CIRCLE_INFO
{
  MVCC_POINTF
    stCenterPoint;  ///< [IN] \~chinese 圆心信息                        \~english Circle Point Info

  float
    fR1;  ///< [IN] \~chinese 宽向半径，根据图像的相对位置[0, 1.0]，半径与圆心的位置有关，需保证画出的圆在显示框范围之内，否则报错  \~english Windth Radius, Range[0, 1.0]
  float
    fR2;  ///< [IN] \~chinese 高向半径，根据图像的相对位置[0, 1.0]，半径与圆心的位置有关，需保证画出的圆在显示框范围之内，否则报错  \~english Height Radius, Range[0, 1.0]

  MVCC_COLORF
    stColor;  ///< [IN] \~chinese 辅助线颜色信息                  \~english Color of Auxiliary Line
  unsigned int
    nLineWidth;  ///< [IN] \~chinese 辅助线宽度，宽度只能是1或2      \~english Width of Auxiliary Line, width is 1 or 2
  unsigned int nReserved[4];  ///<      \~chinese 预留                            \~english Reserved

} MVCC_CIRCLE_INFO;

/// \~chinese 线条辅助线信息    \~english Linear Auxiliary Line Info
typedef struct _MVCC_LINES_INFO
{
  MVCC_POINTF
    stStartPoint;  ///< [IN] \~chinese 线条辅助线的起始点坐标          \~english The Start Point of Auxiliary Line
  MVCC_POINTF
    stEndPoint;  ///< [IN] \~chinese 线条辅助线的终点坐标            \~english The End Point of Auxiliary Line
  MVCC_COLORF
    stColor;  ///< [IN] \~chinese 辅助线颜色信息                  \~english Color of Auxiliary Line
  unsigned int
    nLineWidth;  ///< [IN] \~chinese 辅助线宽度，宽度只能是1或2      \~english Width of Auxiliary Line, width is 1 or 2
  unsigned int nReserved[4];  ///<  \~chinese 预留                            \~english Reserved

} MVCC_LINES_INFO;

///< \~chinese 分时曝光时最多将源图像拆分的个数 \~english The maximum number of source image to be split in time-division exposure
#define MV_MAX_SPLIT_NUM 8

/// \~chinese 图像重构的方式        \~english Image reconstruction method
typedef enum _MV_IMAGE_RECONSTRUCTION_METHOD_
{
  MV_SPLIT_BY_LINE =
    1,  ///< \~chinese 源图像按行拆分成多张图像         \~english Source image split into multiple images by line

} MV_IMAGE_RECONSTRUCTION_METHOD;

/// \~chinese 图像重构后的图像列表      \~english List of images after image reconstruction
typedef struct _MV_OUTPUT_IMAGE_INFO_
{
  unsigned int
    nWidth;  ///< [OUT]       \~chinese 源图像宽              \~english Source Image Width
  unsigned int
    nHeight;  ///< [OUT]       \~chinese 源图像高              \~english Source Image Height
  enum MvGvspPixelType
    enPixelType;  ///< [OUT]       \~chinese 像素格式              \~english Pixel format

  unsigned char *
    pBuf;  ///< [IN][OUT]   \~chinese 输出数据缓存          \~english Output data buffer
  unsigned int
    nBufLen;  ///< [OUT]       \~chinese 输出数据长度          \~english Output data length
  unsigned int
    nBufSize;  ///< [IN]        \~chinese 提供的输出缓冲区大小  \~english Provided output buffer size

  unsigned int nRes[8];  ///<             \~chinese 预留                  \~english Reserved
} MV_OUTPUT_IMAGE_INFO;

/// \~chinese 重构图像参数信息      \~english Restructure image parameters
typedef struct _MV_RECONSTRUCT_IMAGE_PARAM_
{
  unsigned int nWidth;   ///< [IN]  \~chinese 源图像宽          \~english Source Image Width
  unsigned int nHeight;  ///< [IN]  \~chinese 源图像高          \~english Source Image Height
  enum MvGvspPixelType enPixelType;  ///< [IN]  \~chinese 像素格式          \~english Pixel format

  unsigned char * pSrcData;  ///< [IN]  \~chinese 输入数据缓存      \~english Input data buffer
  unsigned int nSrcDataLen;  ///< [IN]  \~chinese 输入数据长度      \~english Input data length

  unsigned int nExposureNum;  ///< [IN]  \~chinese 曝光个数(1-8]     \~english Exposure number
  MV_IMAGE_RECONSTRUCTION_METHOD
    enReconstructMethod;  ///< [IN]  \~chinese 图像重构方式      \~english Image restructuring method

  MV_OUTPUT_IMAGE_INFO stDstBufList
    [MV_MAX_SPLIT_NUM];  ///< [OUT] \~chinese 输出数据缓存信息  \~english Output data info

  unsigned int nRes[4];
} MV_RECONSTRUCT_IMAGE_PARAM;

#endif /* _MV_CAMERA_PARAMS_H_ */
