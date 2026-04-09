
#ifndef _MV_ERROR_DEFINE_H_
#define _MV_ERROR_DEFINE_H_

#include "MvISPErrorDefine.h"

/********************************************************************/
///  \~chinese
///  \name 正确码定义
///  @{
///  \~english
///  \name Definition of correct code
///  @{
#define MV_OK 0x00000000  ///< \~chinese 成功，无错误             \~english Successed, no error
/// @}

/********************************************************************/
///  \~chinese
///  \name 通用错误码定义:范围0x80000000-0x800000FF
///  @{
///  \~english
///  \name Definition of General error code
///  @{
#define MV_E_HANDLE \
  0x80000000  ///< \~chinese 错误或无效的句柄         \~english Error or invalid handle
#define MV_E_SUPPORT \
  0x80000001  ///< \~chinese 不支持的功能             \~english Not supported function
#define MV_E_BUFOVER 0x80000002  ///< \~chinese 缓存已满                 \~english Buffer overflow
#define MV_E_CALLORDER \
  0x80000003  ///< \~chinese 函数调用顺序错误         \~english Function calling order error
#define MV_E_PARAMETER \
  0x80000004  ///< \~chinese 错误的参数               \~english Incorrect parameter
#define MV_E_RESOURCE \
  0x80000006  ///< \~chinese 资源申请失败             \~english Applying resource failed
#define MV_E_NODATA 0x80000007  ///< \~chinese 无数据                   \~english No data
#define MV_E_PRECONDITION \
  0x80000008  ///< \~chinese 前置条件有误，或运行环境已发生变化   \~english Precondition error, or running environment changed
#define MV_E_VERSION \
  0x80000009  ///< \~chinese 版本不匹配               \~english Version mismatches
#define MV_E_NOENOUGH_BUF \
  0x8000000A  ///< \~chinese 传入的内存空间不足       \~english Insufficient memory
#define MV_E_ABNORMAL_IMAGE \
  0x8000000B  ///< \~chinese 异常图像，可能是丢包导致图像不完整   \~english Abnormal image, maybe incomplete image because of lost packet
#define MV_E_LOAD_LIBRARY \
  0x8000000C  ///< \~chinese 动态导入DLL失败          \~english Load library failed
#define MV_E_NOOUTBUF \
  0x8000000D  ///< \~chinese 没有可输出的缓存         \~english No Avaliable Buffer
#define MV_E_ENCRYPT 0x8000000E   ///< \~chinese 加密错误                 \~english Encryption error
#define MV_E_OPENFILE 0x8000000F  ///< \~chinese 打开文件出现错误         \~english open file error
#define MV_E_UNKNOW 0x800000FF    ///< \~chinese 未知的错误               \~english Unknown error
/// @}

/********************************************************************/
///  \~chinese
///  \name GenICam系列错误:范围0x80000100-0x800001FF
///  @{
///  \~english
///  \name GenICam Series Error Codes: Range from 0x80000100 to 0x800001FF
///  @{
#define MV_E_GC_GENERIC 0x80000100  ///< \~chinese 通用错误                 \~english General error
#define MV_E_GC_ARGUMENT \
  0x80000101  ///< \~chinese 参数非法                 \~english Illegal parameters
#define MV_E_GC_RANGE \
  0x80000102  ///< \~chinese 值超出范围               \~english The value is out of range
#define MV_E_GC_PROPERTY 0x80000103  ///< \~chinese 属性                     \~english Property
#define MV_E_GC_RUNTIME \
  0x80000104  ///< \~chinese 运行环境有问题           \~english Running environment error
#define MV_E_GC_LOGICAL 0x80000105  ///< \~chinese 逻辑错误                 \~english Logical error
#define MV_E_GC_ACCESS \
  0x80000106  ///< \~chinese 节点访问条件有误         \~english Node accessing condition error
#define MV_E_GC_TIMEOUT 0x80000107  ///< \~chinese 超时                     \~english Timeout
#define MV_E_GC_DYNAMICCAST \
  0x80000108  ///< \~chinese 转换异常                 \~english Transformation exception
#define MV_E_GC_UNKNOW \
  0x800001FF  ///< \~chinese GenICam未知错误          \~english GenICam unknown error
/// @}

/********************************************************************/
///  \~chinese
///  \name GigE_STATUS对应的错误码:范围0x80000200-0x800002FF
///  @{
///  \~english
///  \name GigE_STATUS Error Codes: Range from 0x80000200 to 0x800002FF
///  @{
#define MV_E_NOT_IMPLEMENTED \
  0x80000200  ///< \~chinese 命令不被设备支持         \~english The command is not supported by device
#define MV_E_INVALID_ADDRESS \
  0x80000201  ///< \~chinese 访问的目标地址不存在     \~english The target address being accessed does not exist
#define MV_E_WRITE_PROTECT \
  0x80000202  ///< \~chinese 目标地址不可写           \~english The target address is not writable
#define MV_E_ACCESS_DENIED \
  0x80000203  ///< \~chinese 设备无访问权限           \~english No permission
#define MV_E_BUSY \
  0x80000204  ///< \~chinese 设备忙，或网络断开       \~english Device is busy, or network disconnected
#define MV_E_PACKET \
  0x80000205  ///< \~chinese 网络包数据错误           \~english Network data packet error
#define MV_E_NETER 0x80000206  ///< \~chinese 网络相关错误             \~english Network error
#define MV_E_IP_CONFLICT \
  0x80000221  ///< \~chinese 设备IP冲突               \~english Device IP conflict
/// @}

/********************************************************************/
///  \~chinese
///  \name USB_STATUS对应的错误码:范围0x80000300-0x800003FF
///  @{
///  \~english
///  \name USB_STATUS Error Codes: Range from 0x80000300 to 0x800003FF
///  @{
#define MV_E_USB_READ \
  0x80000300  ///< \~chinese 读usb出错                \~english Reading USB error
#define MV_E_USB_WRITE \
  0x80000301  ///< \~chinese 写usb出错                \~english Writing USB error
#define MV_E_USB_DEVICE \
  0x80000302  ///< \~chinese 设备异常                 \~english Device exception
#define MV_E_USB_GENICAM 0x80000303  ///< \~chinese GenICam相关错误          \~english GenICam error
#define MV_E_USB_BANDWIDTH \
  0x80000304  ///< \~chinese 带宽不足                 \~english Insufficient bandwidth
#define MV_E_USB_DRIVER \
  0x80000305  ///< \~chinese 驱动不匹配或者未装驱动   \~english Driver mismatch or unmounted drive
#define MV_E_USB_UNKNOW \
  0x800003FF  ///< \~chinese USB未知的错误            \~english USB unknown error
/// @}

/********************************************************************/
///  \~chinese
///  \name 升级时对应的错误码:范围0x80000400-0x800004FF
///  @{
///  \~english
///  \name Upgrade Error Codes: Range from 0x80000400 to 0x800004FF
///  @{
#define MV_E_UPG_FILE_MISMATCH \
  0x80000400  ///< \~chinese 升级固件不匹配           \~english Firmware mismatches
#define MV_E_UPG_LANGUSGE_MISMATCH \
  0x80000401  ///< \~chinese 升级固件语言不匹配       \~english Firmware language mismatches
#define MV_E_UPG_CONFLICT \
  0x80000402  ///< \~chinese 升级冲突（设备已经在升级了再次请求升级即返回此错误） \~english Upgrading conflicted (repeated upgrading requests during device upgrade)
#define MV_E_UPG_INNER_ERR \
  0x80000403  ///< \~chinese 升级时设备内部出现错误   \~english Camera internal error during upgrade
#define MV_E_UPG_UNKNOW \
  0x800004FF  ///< \~chinese 升级时未知错误           \~english Unknown error during upgrade
/// @}

#endif  //_MV_ERROR_DEFINE_H_
