
#ifndef _MV_CAMERA_CTRL_H_
#define _MV_CAMERA_CTRL_H_

#include "CameraParams.h"
#include "MvErrorDefine.h"
#include "MvObsoleteInterfaces.h"

#ifndef MV_CAMCTRL_API

#if (defined(_WIN32) || defined(WIN64))
#if defined(MV_CAMCTRL_EXPORTS)
#define MV_CAMCTRL_API __declspec(dllexport)
#else
#define MV_CAMCTRL_API __declspec(dllimport)
#endif
#else
#ifndef __stdcall
#define __stdcall
#endif

#ifndef MV_CAMCTRL_API
#define MV_CAMCTRL_API
#endif
#endif

#endif

#ifdef MV_CAMCTRL_API

#if (defined(_WIN32) || defined(WIN64))
#if defined(MV_CAMCTRL_EXPORTS)
#define MV_CAMCTRL_API __declspec(dllexport)
#else
#define MV_CAMCTRL_API __declspec(dllimport)
#endif
#else
#ifndef __stdcall
#define __stdcall
#endif

#if defined(MV_CAMCTRL_EXPORTS)
#define MV_CAMCTRL_API __attribute__((visibility("default")))
#else
#define MV_CAMCTRL_API
#endif
#endif

#endif

#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

#ifdef __cplusplus
extern "C" {
#endif

/****************************** ch: 摘要 | en: Instructions**********************************************/

/** 
*   @~chinese
*     该头文件主要包含9部分：
*     1.相机的控制和取流接口
*     2.相机属性万能配置接口&读写寄存器接口
*     3.相机升级
*     4.注册异常回调和事件接口
*     5.仅GigE设备支持的接口
*     6.仅CameraLink 设备支持的接口
*     7.仅U3V设备支持的接口
*     8.GenTL相关接口
*     9.图像保存、格式转换等相关接口

*   @~english
*     The document mainly consists of nine parts:
*     1.Camera control and streaming·
*     2.Camera attribute nodes set and obtained universal interface
*     3.Camera upgrade and read-write register memory interface
*     4.Enrol abnormal callbacks and event interface
*     5.Only support GigE device interface
*     6.Only support camlink device interface
*     7.Only support U3V device interface
*     8.Only support GenTL device interface
*     9.Related image save and format convert interface
**/

/**************************Part1 ch: 相机的控制和取流接口 | en: Camera control and streaming******************************************/
/*
 *  @~chinese
 *  @brief  获取SDK版本号
 *  @return 返回4字节版本号
            |主    |次    |修正  |  测试|
             8bits  8bits  8bits  8bits
 *  @remarks 比如返回值为0x01000001，即SDK版本号为V1.0.0.1。
 
 *  @~english
 *  @brief  Get SDK Version
 *  @return Always return 4 Bytes of version number 
            |Main    |Sub    |Rev  |  Test|
             8bits  8bits  8bits  8bits
 *  @remarks For example, if the return value is 0x01000001, the SDK version is V1.0.0.1.
 ************************************************************************/
MV_CAMCTRL_API unsigned int __stdcall MV_CC_GetSDKVersion();

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取支持的传输层
 *  @return 支持的传输层编号 

 *  @~english
 *  @brief  Get supported Transport Layer
 *  @return Supported Transport Layer number
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumerateTls();

/********************************************************************/ /**
 *  @~chinese
 *  @brief  枚举设备
 *  @param  nTLayerType                 [IN]            枚举传输层, 参数定义参见CameraParams.h定义, 如: #define MV_GIGE_DEVICE 0x00000001 GigE设备
 *  @param  pstDevList                  [IN][OUT]       设备列表
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 设备列表的内存是在SDK内部分配的，多线程调用该接口时会进行设备列表内存的释放和申请,建议尽量避免多线程枚举操作。
 *  @remarks 参数枚举传输层，适配传入MV_GIGE_DEVICE、MV_1394_DEVICE、MV_USB_DEVICE、MV_CAMERALINK_DEVICE；MV_GIGE_DEVICE该参数
             传出所有GiGE相关的设备信息（包含虚拟GiGE和GenTL下的GiGE设备），MV_USB_DEVICE该参数传出所有USB设备，包含虚拟USB设备。

 *  @~english
 *  @brief  Enumerate Device
 *  @param  nTLayerType                 [IN]            Enumerate TLs, Refer to the 'CameraParams.h' for parameter definitions, for example, #define MV_GIGE_DEVICE 0x00000001
 *  @param  pstDevList                  [IN][OUT]       Device List
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks The memory of the device list is allocated within the SDK. When the interface is invoked by multiple threads, the memory of the device list will be released and applied
             It is recommended to avoid multithreaded enumeration operations as much as possible.
 *  @remarks Transmission layer of enumeration, param only include MV_GIGE_DEVICE、MV_1394_DEVICE、MV_USB_DEVICE、MV_CAMERALINK_DEVICE;
             MV_GIGE_DEVICE can output virtual and GenTL GiGE devices, MV_USB_DEVICE can output all USB devices, include virtual usb devices.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumDevices(
  IN unsigned int nTLayerType, IN OUT MV_CC_DEVICE_INFO_LIST * pstDevList);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  根据厂商名字枚举设备
 *  @param  nTLayerType                 [IN]            枚举传输层, 参数定义参见CameraParams.h定义, 如: #define MV_GIGE_DEVICE 0x00000001 GigE设备
 *  @param  pstDevList                  [IN][OUT]       设备列表
 *  @param  strManufacturerName         [IN]            厂商名字
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 参数枚举传输层，适配传入MV_GIGE_DEVICE、MV_1394_DEVICE、MV_USB_DEVICE、MV_CAMERALINK_DEVICE；MV_GIGE_DEVICE该参数
            传出所有GiGE相关的设备信息（包含虚拟GiGE和GenTL下的GiGE设备），MV_USB_DEVICE该参数传出所有USB设备，包含虚拟USB设备。
 *  @remarks 设备列表的内存是在SDK内部分配的，多线程调用该接口时会进行设备列表内存的释放和申请,建议尽量避免多线程枚举操作。

 *  @~english
 *  @brief  Enumerate device according to manufacture name
 *  @param  nTLayerType                 [IN]            Transmission layer of enumeration, , Refer to the 'CameraParams.h' for parameter definitions, for example, #define MV_GIGE_DEVICE 0x00000001
 *  @param  pstDevList                  [IN][OUT]       Device list
 *  @param  strManufacturerName         [IN]            Manufacture Name
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Transmission layer of enumeration, param only include MV_GIGE_DEVICE、MV_1394_DEVICE、MV_USB_DEVICE、MV_CAMERALINK_DEVICE;
             MV_GIGE_DEVICE can output virtual and GenTL GiGE devices, MV_USB_DEVICE can output all USB devices, include virtual usb devices.
 *  @remarks The memory of the device list is allocated within the SDK. When the interface is invoked by multiple threads, the memory of the device list will be released and applied.
             It is recommended to avoid multithreaded enumeration operations as much as possible.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumDevicesEx(
  IN unsigned int nTLayerType, IN OUT MV_CC_DEVICE_INFO_LIST * pstDevList,
  IN const char * strManufacturerName);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  枚举设备扩展（可指定排序方式枚举、根据厂商名字过滤）
 *  @param  nTLayerType                 [IN]            枚举传输层（区分每一种传输层类型，不耦合）, 参数定义参见CameraParams.h定义, 如: #define MV_GIGE_DEVICE 0x00000001 GigE设备
 *  @param  pstDevList                  [IN][OUT]       设备列表
 *  @param  strManufacturerName         [IN]            厂商名字（可传NULL，即不过滤）
 *  @param  enSortMethod                [IN]            排序方式
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 设备列表的内存是在SDK内部分配的，多线程调用该接口时会进行设备列表内存的释放和申请,建议尽量避免多线程枚举操作
             strManufacturerName可传入NULL，若传入NULL则返回排好序的所有设备列表,若不为NULL则只返回排好序的指定厂商设备列表。

 *  @~english
 *  @brief  Enumerate device according to the specified ordering
 *  @param  nTLayerType                 [IN]            Transmission layer of enumeration(All layer protocol type can input), Refer to the 'CameraParams.h' for parameter definitions, for example, #define MV_GIGE_DEVICE 0x00000001
 *  @param  pstDevList                  [IN][OUT]       Device list
 *  @param  strManufacturerName         [IN]            Manufacture Name
 *  @param  enSortMethod                [IN]            Sorting Method
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks The memory of the device list is allocated within the SDK. When the interface is invoked by multiple threads, the memory of the device list will be released and applied.
             It is recommended to avoid multithreaded enumeration operations as much as possible.
             strManufacturerName can be passed in NULL,if NULL is passed in, it will return the sorted list of all devices.
             If it is not NULL,it will only return the sorted list of the specified manufacturer's devices.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumDevicesEx2(
  IN unsigned int nTLayerType, IN OUT MV_CC_DEVICE_INFO_LIST * pstDevList,
  IN const char * strManufacturerName, IN MV_SORT_METHOD enSortMethod);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设备是否可达
 *  @param  pstDevInfo                  [IN]            设备信息结构体
 *  @param  nAccessMode                 [IN]            访问权限,参数定义参见CameraParams.h定义, 如:#define MV_ACCESS_Exclusive 1 
 *  @remarks 读取设备CCP寄存器的值，判断当前状态是否具有某种访问权限
             如果设备不支持MV_ACCESS_ExclusiveWithSwitch、MV_ACCESS_ControlWithSwitch、MV_ACCESS_ControlSwitchEnableWithKey这三种模式，接口返回false。
             目前设备不支持这3种抢占模式，国际上主流的厂商的设备也都暂不支持这3种模式。
             该接口支持 虚拟相机，GIGE相机，U3V相机，自研网卡设备。
             该接口不支持CameraLink设备,不支持导入.cti方式采集卡设备。
 
 *  @~english
 *  @brief  Is the device accessible
 *  @param  pstDevInfo                  [IN]            Device Information Structure
 *  @param  nAccessMode                 [IN]            Access Right, Refer to the 'CameraParams.h' for parameter definitions, for example, #define MV_ACCESS_Exclusive 1 
 *  @return Access, return true. Not access, return false
 *  @remarks Read device CCP register value and determine current access permission.
             Return false if the device does not support the modes MV_ACCESS_ExclusiveWithSwitch, MV_ACCESS_ControlWithSwitch, MV_ACCESS_ControlSwitchEnableWithKey.
             Currently the device does not support the 3 preemption modes, neither do the devices from other mainstream manufacturers.
             This API is not supported by CameraLink device. 
 **************************************************************************/
MV_CAMCTRL_API bool __stdcall MV_CC_IsDeviceAccessible(
  IN MV_CC_DEVICE_INFO * pstDevInfo, IN unsigned int nAccessMode);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置SDK日志路径
 *  @param  strSDKLogPath      [IN]   SDK日志路径
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 设置路径之后，可以指定路径存放日志, V2.4.1版本新增日志服务，开启服务之后该接口无效，默认日志服务为开启状态。
 
 *  @~english
 *  @brief  Set SDK log path
 *  @param  strSDKLogPath             [IN]           SDK log path
 *  @return Access, return true. Not access, return false
 *  @remarks For version V2.4.1, added log service, this API is invalid when the service is enabled.And The logging service is enabled by default
             This API is used to set the log file storing path.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetSDKLogPath(IN const char * strSDKLogPath);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  创建设备句柄
 *  @param  handle                      [IN][OUT]       设备句柄
 *  @param  pstDevInfo                  [IN]            设备信息结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 根据输入的设备信息，创建库内部必须的资源和初始化内部模块
             通过该接口创建句柄，调用SDK接口，会默认生成SDK日志文件，如果不需要生成日志文件，可以通过MV_CC_CreateHandleWithoutLog创建句柄
 
 *  @~english
 *  @brief  Create Device Handle
 *  @param  handle                      [IN][OUT]       Device handle
 *  @param  pstDevInfo                  [IN]            Device Information Structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Create required resources within library and initialize internal module according to input device information. 
             Create handle and call SDK interface through this interface, and SDK log file will be created by default.
             Creating handle through MV_CC_CreateHandleWithoutLog will not generate log files.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CreateHandle(
  OUT void ** handle, IN const MV_CC_DEVICE_INFO * pstDevInfo);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  创建设备句柄，不生成日志
 *  @param  handle                      [IN][OUT]       设备句柄
 *  @param  pstDevInfo                  [IN]            设备信息结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 根据输入的设备信息，创建库内部必须的资源和初始化内部模块
             通过该接口创建句柄，调用SDK接口，不会默认生成SDK日志文件，如果需要生成日志文件可以通过MV_CC_CreateHandle创建句柄，日志文件自动生成
 
 *  @~english
 *  @brief  Create Device Handle without log
 *  @param  handle                      [IN][OUT]       Device handle
 *  @param  pstDevInfo                  [IN]            Device Information Structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Create required resources within library and initialize internal module according to input device information.
             Create handle and call SDK interface through this interface, and SDK log file will not be created. To create logs,
             create handle through MV_CC_CreateHandle, and log files will be automatically generated.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CreateHandleWithoutLog(
  OUT void ** handle, IN const MV_CC_DEVICE_INFO * pstDevInfo);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  销毁设备句柄
 *  @param  handle                      [IN]            设备句柄
 *  @return 成功，返回MV_OK；错误，返回错误码 
 
 *  @~english
 *  @brief  Destroy Device Handle
 *  @param  handle                      [IN]            Device handle
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DestroyHandle(IN void * handle);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  打开设备
 *  @param  handle                      [IN]            设备句柄
 *  @param  nAccessMode                 [IN]            访问权限, 参数定义参见CameraParams.h定义, 如:#define MV_ACCESS_Exclusive 1
 *  @param  nSwitchoverKey              [IN]            切换访问权限时的密钥
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 根据设置的设备参数，找到对应的设备，连接设备, 调用接口时可不传入nAccessMode和nSwitchoverKey，此时默认设备访问模式为独占权限。
             目前设备暂不支持MV_ACCESS_ExclusiveWithSwitch、MV_ACCESS_ControlWithSwitch、MV_ACCESS_ControlSwitchEnable、MV_ACCESS_ControlSwitchEnableWithKey这四种抢占模式
             对于U3V设备，nAccessMode、nSwitchoverKey这两个参数无效
 
 *  @~english
 *  @brief  Open Device
 *  @param  handle                      [IN]            Device handle
 *  @param  nAccessMode                 [IN]            Access Right, Refer to the 'CameraParams.h' for parameter definitions, for example, #define MV_ACCESS_Exclusive 1
 *  @param  nSwitchoverKey              [IN]            Switch key of access right
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Find specific device and connect according to set device parameters.When calling the interface, the input of nAccessMode and nSwitchoverKey is optional, 
             and the device access mode is exclusive. Currently the device does not support the following preemption modes:
             MV_ACCESS_ExclusiveWithSwitch, MV_ACCESS_ControlWithSwitch, MV_ACCESS_ControlSwitchEnableWithKey; For USB3Vision device, nAccessMode, nSwitchoverKey are invalid. 
 ************************************************************************/
#ifndef __cplusplus
MV_CAMCTRL_API int __stdcall MV_CC_OpenDevice(
  IN void * handle, IN unsigned int nAccessMode, IN unsigned short nSwitchoverKey);
#else
MV_CAMCTRL_API int __stdcall MV_CC_OpenDevice(
  IN void * handle, IN unsigned int nAccessMode = MV_ACCESS_Exclusive,
  IN unsigned short nSwitchoverKey = 0);
#endif

/********************************************************************/ /**
 *  @~chinese
 *  @brief  关闭设备
 *  @param  handle                      [IN]            设备句柄
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 通过MV_CC_OpenDevice连接设备后，可以通过该接口断开设备连接，释放资源
 
 *  @~english
 *  @brief  Close Device
 *  @param  handle                      [IN]            Device handle
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After connecting to device through MV_CC_OpenDevice, use this interface to disconnect and release resources.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CloseDevice(IN void * handle);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  判断设备是否处于连接状态
 *  @param  handle                      [IN]            设备句柄
 *  @return 设备处于连接状态，返回true；没连接或失去连接，返回false
 
 *  @~english
 *  @brief  Is The Device Connected
 *  @param  handle                      [IN]            Device handle
 *  @return Connected, return true. Not Connected or DIsconnected, return false
 ***********************************************************************/
MV_CAMCTRL_API bool __stdcall MV_CC_IsDeviceConnected(IN void * handle);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  注册图像数据回调
 *  @param  handle                      [IN]            设备句柄
 *  @param  cbOutput                    [IN]            回调函数指针
 *  @param  pUser                       [IN]            用户自定义变量
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 通过该接口可以设置图像数据回调函数，在MV_CC_CreateHandle之后即可调用, 图像数据采集有两种方式，两种方式不能复用：
             方式一：调用MV_CC_RegisterImageCallBackEx设置图像数据回调函数，然后调用MV_CC_StartGrabbing开始采集，采集的图像数据在设置的回调函数中返回
             方式二：调用MV_CC_StartGrabbing开始采集，然后在应用层循环调用MV_CC_GetOneFrameTimeout获取指定像素格式的帧数据，
             获取帧数据时上层应用程序需要根据帧率控制好调用该接口的频率。
             该接口不支持CameraLink设备。
 
 *  @~english
 *  @brief  Register the image callback function
 *  @param  handle                      [IN]            Device handle
 *  @param  cbOutput                    [IN]            Callback function pointer
 *  @param  pUser                       [IN]            User defined variable
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After MV_CC_CreateHandle, call this interface to set image data callback function.There are two available image data acquisition modes, and cannot be used together:
             Mode 1: Call MV_CC_RegisterImageCallBack to set image data callback function, and then callMV_CC_StartGrabbing to start acquiring. The acquired image data will return in the set callback function.
             Mode 2: Call MV_CC_StartGrabbing to start acquiring, and then call MV_CC_GetOneFrameTimeout repeatedly in application layer to get frame data of specified pixel format. When getting frame data,
             the frequency of calling this interface should be controlled by upper layer application according to frame rate.
             This API is not supported by CameraLink device. 
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterImageCallBackEx(
  IN void * handle,
  IN void(__stdcall * cbOutput)(
    unsigned char * pData, MV_FRAME_OUT_INFO_EX * pFrameInfo, void * pUser),
  IN void * pUser);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  注册图像数据回调，RGB
 *  @param  handle                      [IN]            设备句柄
 *  @param  cbOutput                    [IN]            回调函数指针
 *  @param  pUser                       [IN]            用户自定义变量
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 通过该接口可以设置图像数据回调函数，在MV_CC_CreateHandle之后即可调用。图像数据采集有两种方式，两种方式不能复用：
             方式一：调用MV_CC_RegisterImageCallBackForRGB设置RGB24格式图像数据回调函数，然后调用MV_CC_StartGrabbing开始采集，采集的图像数据在设置的回调函数中返回。
             方式二：调用MV_CC_StartGrabbing开始采集，然后在应用层循环调用MV_CC_GetImageForRGB获取RGB24格式的帧数据,
                      获取帧数据时上层应用程序需要根据帧率控制好调用该接口的频率。
             该接口不支持CameraLink设备。 
 
 *  @~english
 *  @brief  register image data callback, RGB
 *  @param  handle                      [IN]            Device handle
 *  @param  cbOutput                    [IN]            Callback function pointer
 *  @param  pUser                       [IN]            User defined variable
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Before calling this API to set image data callback function, you should call this API MV_CC_CreateHandle.There are two image acquisition modes, the two modes cannot be reused:
             Mode 1: Call MV_CC_RegisterImageCallBackForRGB to set RGB24 format image data callback function, and then call MV_CC_StartGrabbing to start acquisition, the collected image data will be returned in the configured callback function.
             Mode 2: Call MV_CC_StartGrabbing to start acquisition, and the call MV_CC_GetImageForRGB repeatedly in application layer to get frame data with RGB24 format.
                     When getting frame data, the upper application program should control the frequency of calling this API according to frame rate.
             This API is not supported by CameraLink device. 
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterImageCallBackForRGB(
  IN void * handle,
  IN void(__stdcall * cbOutput)(
    unsigned char * pData, MV_FRAME_OUT_INFO_EX * pFrameInfo, void * pUser),
  IN void * pUser);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  注册图像数据回调，BGR
 *  @param  handle                      [IN]            设备句柄
 *  @param  cbOutput                    [IN]            回调函数指针
 *  @param  pUser                       [IN]            用户自定义变量
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 通过该接口可以设置图像数据回调函数，在MV_CC_CreateHandle之后即可调用。图像数据采集有两种方式，两种方式不能复用:
             方式一：调用MV_CC_RegisterImageCallBackForBGR设置BGR24图像数据回调函数，然后调用MV_CC_StartGrabbing开始采集，采集的图像数据在设置的回调函数中返回。
             方式二：调用MV_CC_StartGrabbing开始采集，然后在应用层循环调用MV_CC_GetImageForBGR获取BGR24格式的帧数据,
                      获取帧数据时上层应用程序需要根据帧率控制好调用该接口的频率。
             该接口不支持CameraLink设备。 
 
 *  @~english
 *  @brief  register image data callback, BGR
 *  @param  handle                      [IN]            Device handle
 *  @param  cbOutput                    [IN]            Callback function pointer
 *  @param  pUser                       [IN]            User defined variable
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Before calling this API to set image data callback function, you should call this API MV_CC_CreateHandle.There are two image acquisition modes, the two modes cannot be reused:
             Mode 1: Call MV_CC_RegisterImageCallBackForBGR to set RGB24 format image data callback function, and then call MV_CC_StartGrabbing to start acquisition, the collected image data will be returned in the configured callback function.
             Mode 2: Call MV_CC_StartGrabbing to start acquisition, and the call MV_CC_GetImageForBGR repeatedly in application layer to get frame data with BGR24 format.
                     When getting frame data,the upper application program should control the frequency of calling this API according to frame rate.
             This API is not supported by CameraLink device. 
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterImageCallBackForBGR(
  IN void * handle,
  IN void(__stdcall * cbOutput)(
    unsigned char * pData, MV_FRAME_OUT_INFO_EX * pFrameInfo, void * pUser),
  IN void * pUser);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  开始取流
 *  @param  handle                      [IN]            设备句柄
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 该接口不支持CameraLink设备。
 
 *  @~english
 *  @brief  Start Grabbing
 *  @param  handle                      [IN]            Device handle
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API is not supported by CameraLink device. 
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_StartGrabbing(IN void * handle);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  停止取流
 *  @param  handle                      [IN]            设备句柄
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 该接口不支持CameraLink设备。 
 
 *  @~english
 *  @brief  Stop Grabbing
 *  @param  handle                      [IN]            Device handle
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API is not supported by CameraLink device. 
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_StopGrabbing(IN void * handle);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取一帧RGB数据，此函数为查询式获取，每次调用查询内部
            缓存有无数据，有数据则获取数据，无数据返回错误码
 *  @param  handle                      [IN]            设备句柄
 *  @param  pData                       [IN][OUT]       图像数据接收指针
 *  @param  nDataSize                   [IN]            接收缓存大小
 *  @param  pstFrameInfo                [IN][OUT]       图像信息结构体
 *  @param  nMsec                       [IN]            等待超时时间
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 每次调用该接口，将查询内部缓存是否有数据，如果有数据则转换成RGB24格式返回，如果没有数据则返回错误码。
             因为图像转换成RGB24格式有耗时，所以当数据帧率过高时该接口可能会导致丢帧。调用该接口获取图像数据帧之前需要先调用MV_CC_StartGrabbing启动图像采集。
             该接口为主动式获取帧数据，上层应用程序需要根据帧率，控制好调用该接口的频率。
             该接口不支持CameraLink设备。
 
 *  @~english
 *  @brief  Get one frame of RGB data, this function is using query to get data
            query whether the internal cache has data, get data if there has, return error code if no data
 *  @param  handle                      [IN]            Device handle
 *  @param  pData                       [IN][OUT]       Image data receiving buffer
 *  @param  nDataSize                   [IN]            Buffer size
 *  @param  pstFrameInfo                [IN][OUT]       Image information structure
 *  @param  nMsec                       [IN]            Waiting timeout
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Each time the API is called, the internal cache is checked for data. If there is data, it will be transformed as RGB24 format for return, if there is no data, return error code. 
             As time-consuming exists when transform the image to RGB24 format,this API may cause frame loss when the data frame rate is too high.
             Before calling this API to get image data frame, call MV_CC_StartGrabbing to start image acquisition.
             This API can get frame data actively, the upper layer program should control the frequency of calling this API according to the frame rate.
             This API is not supported by CameraLink device.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetImageForRGB(
  IN void * handle, IN OUT unsigned char * pData, IN unsigned int nDataSize,
  IN OUT MV_FRAME_OUT_INFO_EX * pstFrameInfo, IN int nMsec);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取一帧BGR数据，此函数为查询式获取，每次调用查询内部
            缓存有无数据，有数据则获取数据，无数据返回错误码
 *  @param  handle                      [IN]            设备句柄
 *  @param  pData                       [IN][OUT]       图像数据接收指针
 *  @param  nDataSize                   [IN]            接收缓存大小
 *  @param  pstFrameInfo                [IN][OUT]       图像信息结构体
 *  @param  nMsec                       [IN]            等待超时时间
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 每次调用该接口，将查询内部缓存是否有数据，如果有数据则转换成BGR24格式返回，如果没有数据则返回错误码。
             因为图像转换成BGR24格式有耗时，所以当数据帧率过高时该接口可能会导致丢帧.调用该接口获取图像数据帧之前需要先调用MV_CC_StartGrabbing启动图像采集。
             该接口为主动式获取帧数据，上层应用程序需要根据帧率，控制好调用该接口的频率。
             该接口不支持CameraLink设备。
 
 *  @~english
 *  @brief  Get one frame of BGR data, this function is using query to get data
            query whether the internal cache has data, get data if there has, return error code if no data
 *  @param  handle                      [IN]            Device handle
 *  @param  pData                       [IN][OUT]       Image data receiving buffer
 *  @param  nDataSize                   [IN]            Buffer size
 *  @param  pstFrameInfo                [IN][OUT]       Image information structure
 *  @param  nMsec                       [IN]            Waiting timeout
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Before calling this API to set image data callback function, you should call this API MV_CC_CreateHandle.
             There are two image acquisition modes, the two modes cannot be reused: 
             Mode 1: Call MV_CC_RegisterImageCallBackForBGR to set RGB24 format image data callback function, and then call MV_CC_StartGrabbing to start acquisition, the collected image data will be returned in the configured callback function.
             Mode 2: Call MV_CC_StartGrabbing to start acquisition, and the call MV_CC_GetImageForBGR repeatedly in application layer to get frame data with BGR24 format.
             When getting frame data, the upper application program should control the frequency of calling this API according to frame rate.
             This API is not supported by CameraLink device.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetImageForBGR(
  IN void * handle, IN OUT unsigned char * pData, IN unsigned int nDataSize,
  IN OUT MV_FRAME_OUT_INFO_EX * pstFrameInfo, IN int nMsec);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  使用内部缓存获取一帧图片（与MV_CC_Display不能同时使用）
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstFrame                    [IN][OUT]       图像数据和图像信息
 *  @param  nMsec                       [IN]            等待超时时间，输入INFINITE时表示无限等待，直到收到一帧数据或者停止取流
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 调用该接口获取图像数据帧之前需要先调用MV_CC_StartGrabbing启动图像采集。该接口为主动式获取帧数据，上层应用程序需要根据帧率，控制好调用该接口的频率,
             该接口支持设置超时时间，SDK内部等待直到有数据时返回，可以增加取流平稳性，适合用于对平稳性要求较高的场合。
             该接口与MV_CC_FreeImageBuffer配套使用，当处理完取到的数据后，需要用MV_CC_FreeImageBuffer接口将pFrame内的数据指针权限进行释放。
             该接口与MV_CC_GetOneFrameTimeout相比，有着更高的效率。且其取流缓存的分配是由sdk内部自动分配的，而MV_CC_GetOneFrameTimeout接口是需要客户自行分配。
             该接口在调用MV_CC_Display后无法取流。
             该接口对于U3V、GIGE设备均可支持。
             该接口不支持CameraLink设备。
 
 *  @~english
 *  @brief  Get a frame of an image using an internal cache
 *  @param  handle                      [IN]            Device handle
 *  @param  pstFrame                    [IN][OUT]       Image data and image information
 *  @param  nMsec                       [IN]            Waiting timeout
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Before calling this API to get image data frame, you should call MV_CC_StartGrabbing to start image acquisition.
             This API can get frame data actively, the upper layer program should control the frequency of calling this API according to the frame rate. This API support setting timeout, and SDK will wait to return until data appears. This function will increase the streaming stability, which can be used in the situation with high stability requirement. 
             This API and MV_CC_FreeImageBuffer should be called in pairs, after processing the acquired data, you should call MV_CC_FreeImageBuffer to release the data pointer permission of pFrame. 
             This interface is more efficient than MV_CC_GetOneFrameTimeout. The allocation of the stream cache is automatically allocated within the SDK.The MV_CC_GetOneFrameTimeout interface needs to be allocated by customers themselves. 
             This API cannot be called to stream after calling MV_CC_Display.
             This API is not supported by CameraLink device. 
             This API is supported by both USB3 vision camera and GigE camera. 
 *****************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetImageBuffer(
  IN void * handle, IN OUT MV_FRAME_OUT * pstFrame, IN unsigned int nMsec);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  释放图像缓存(此接口用于释放不再使用的图像缓存，与MV_CC_GetImageBuffer配套使用)
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstFrame                    [IN]            图像数据和图像数据
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 该接口与MV_CC_GetImageBuffer配套使用，使用MV_CC_GetImageBuffer接口取到的图像数据pFrame，需要用MV_CC_FreeImageBuffer接口进行权限释放
             该接口对于取流效率高于GetOneFrameTimeout接口，且GetImageBuffer在不进行Free的情况下，最大支持输出的节点数与SetImageNode接口所设置的节点数相等，默认节点数是1
             该接口对于U3V、GIGE设备均可支持
             该接口不支持CameraLink设备。
 
 *  @~english
 *  @brief  Free image buffer(this interface can free image buffer, used with MV_CC_GetImageBuffer)
 *  @param  handle                      [IN]            Device handle
 *  @param  pstFrame                    [IN]            Image data and image information
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API and MV_CC_GetImageBuffer should be called in pairs, before calling MV_CC_GetImageBuffer to get image data pFrame, you should call MV_CC_FreeImageBuffer to release the permission.
             Compared with API MV_CC_GetOneFrameTimeout
             this API has higher efficiency of image acquisition. The max. number of nodes can be outputted is same as the "nNum" of API MV_CC_SetImageNodeNum, default value is 1.
             This API is not supported by CameraLink device.
             This API is supported by both USB3 vision camera and GigE camera. 
 **********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FreeImageBuffer(IN void * handle, IN MV_FRAME_OUT * pstFrame);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  采用超时机制获取一帧图片，SDK内部等待直到有数据时返回
 *  @param  handle                      [IN]            设备句柄
 *  @param  pData                       [IN][OUT]       图像数据接收指针
 *  @param  nDataSize                   [IN]            接收缓存大小
 *  @param  pstFrameInfo                [IN][OUT]       图像信息结构体
 *  @param  nMsec                       [IN]            等待超时时间
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 调用该接口获取图像数据帧之前需要先调用MV_CC_StartGrabbing启动图像采集
             该接口为主动式获取帧数据，上层应用程序需要根据帧率，控制好调用该接口的频率
             该接口支持设置超时时间，SDK内部等待直到有数据时返回，可以增加取流平稳性，适合用于对平稳性要求较高的场合
             该接口对于U3V、GIGE设备均可支持
             该接口不支持CameraLink设备。
 
 *  @~english
 *  @brief  Timeout mechanism is used to get image, and the SDK waits inside until the data is returned
 *  @param  handle                      [IN]            Device handle
 *  @param  pData                       [IN][OUT]       Image data receiving buffer
 *  @param  nDataSize                   [IN]            Buffer size
 *  @param  pstFrameInfo                [IN][OUT]       Image information structure
 *  @param  nMsec                       [IN]            Waiting timeout
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Before calling this API to get image data frame, call MV_CC_StartGrabbing to start image acquisition.
             This API can get frame data actively, the upper layer program should control the frequency of calling this API according to the frame rate.
             This API supports setting timeout, SDK will wait to return until data appears. This function will increase the streaming stability, which can be used in the situation with high stability requirement.
             Both the USB3Vision and GIGE camera can support this API.
             This API is not supported by CameraLink device.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetOneFrameTimeout(
  IN void * handle, IN OUT unsigned char * pData, IN unsigned int nDataSize,
  IN OUT MV_FRAME_OUT_INFO_EX * pstFrameInfo, IN unsigned int nMsec);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  清除取流数据缓存
 *  @param  handle                      [IN]            设备句柄
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 该接口允许用户在不停止取流的时候，就能清除缓存中不需要的图像
             该接口在连续模式切触发模式后，可以清除历史数据。
 
 *  @~english
 *  @brief  if Image buffers has retrieved the data，Clear them
 *  @param  handle                      [IN]            Device handle
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface allows user to clear the unnecessary images from the buffer memory without stopping acquisition.
             This interface allows user to clear previous data after switching from continuous mode to trigger mode. 
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ClearImageBuffer(IN void * handle);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取当前图像缓存区的有效图像个数
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnValidImageNum             [IN][OUT]       当前图像缓存区中有效图像个数的指针
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 
 
 *  @~english
 *  @brief  Get the number of valid images in the current image buffer
 *  @param  handle                      [IN]            Device handle
 *  @param  pnValidImageNum             [IN][OUT]       The number of valid images in the current image buffer
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks 
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetValidImageNum(
  IN void * handle, IN OUT unsigned int * pnValidImageNum);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  显示一帧图像
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstDisplayInfo              [IN]            图像信息
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 与设备类型无关

 *  @~english
 *  @brief  Display one frame image
 *  @param  handle                      [IN]            Device handle
 *  @param  pstDisplayInfo              [IN]            Frame Info
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Not related to device type
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DisplayOneFrame(
  IN void * handle, IN MV_DISPLAY_FRAME_INFO * pstDisplayInfo);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  显示一帧图像
 *  @param  handle                      [IN]            设备句柄
 *  @param  hWnd                        [IN]            窗口句柄
 *  @param  pstDisplayInfo              [IN]            图像信息
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 该接口支持PixelType_Gvsp_RGB8_Packed，PixelType_Gvsp_BGR8_Packed，PixelType_Gvsp_Mono8三种像素格式的渲染宽高大小至int类型
 *           其余像素格式渲染仅支持宽高至short
 
 *  @~english
 *  @brief  Display one frame image
 *  @param  handle                      [IN]            Device handle
 *  @param  hWnd                        [IN]            HWND
 *  @param  pstDisplayInfo              [IN]            Frame Info
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API support rendering in three pixel formats:PixelType_Gvsp_RGB8_Packed,PixelType_Gvsp_BGR8_Packed and PixelType_Gvsp_Mono8,width and height to int.
 *           The rest of the pixel format rendering only supports width and height to short.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DisplayOneFrameEx(
  IN void * handle, IN void * hWnd, IN MV_DISPLAY_FRAME_INFO_EX * pstDisplayInfo);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置SDK内部图像缓存节点个数，大于等于1，在抓图前调用
 *  @param  handle                      [IN]            设备句柄
 *  @param  nNum                        [IN]            缓存节点个数
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 调用该接口可以设置SDK内部图像缓存节点个数，在调用MV_CC_StartGrabbing开始抓图前调用。
             在SDK中默认是1个节点。
             对于CameraLink设备：该接口不支持串口方式连接,但支持GenTL方式连接。
 
 *  @~english
 *  @brief  Set the number of the internal image cache nodes in SDK, Greater than or equal to 1, to be called before the capture
 *  @param  handle                      [IN]            Device handle
 *  @param  nNum                        [IN]            Image Node Number
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Call this interface to set the number of SDK internal image buffer nodes. The interface should be called before calling MV_CC_StartGrabbing for capturing. 
             For CameraLink devices: This interface does not support serial connection, but supports GenTL connection.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetImageNodeNum(IN void * handle, IN unsigned int nNum);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置取流策略（该接口只支持windows平台）
 *  @param  handle                      [IN]            设备句柄
 *  @param  enGrabStrategy              [IN]            策略枚举值
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 该接口定义了四种取流策略，用户可以根据实际需求进行选择。具体描述如下：
             OneByOne:从旧到新一帧一帧的从输出缓存列表中获取图像，打开设备后默认为该策略
             LatestImagesOnly:仅从输出缓存列表中获取最新的一帧图像，同时清空输出缓存列表
             LatestImages:从输出缓存列表中获取最新的OutputQueueSize帧图像，其中OutputQueueSize范围为1-ImageNodeNum，可用MV_CC_SetOutputQueueSize接口设置，ImageNodeNum默认为1，
                          可用MV_CC_SetImageNodeNum接口设置 OutputQueueSize设置成1等同于LatestImagesOnly策略，OutputQueueSize设置成ImageNodeNum等同于OneByOne策略
             UpcomingImage:在调用取流接口时忽略输出缓存列表中所有图像，并等待设备即将生成的一帧图像。该策略只支持GigE设备，不支持U3V设备
 
 *  @~english
 *  @brief  Set Grab Strategy（This interface only supports windows platform）
 *  @param  handle                      [IN]            Device handle
 *  @param  enGrabStrategy              [IN]            The value of Grab Strategy
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface is set by four image acquisition approaches, the user may choose one as needed. Specific details are as followed: 
             OneByOne:Obtain image from output cache list frame by frame in order, this function is default strategy when device is on.
             LatestImagesOnly:Obtain the latest image from output cache list only, meanwhile clear output cache list.
             LatestImages:Obtain the latest OutputQueueSize image from output cache list, the range of OutputQueueSize is 1-ImageNodeNum, 
                          the user may set the value of MV_CC_SetOutputQueueSizeinterface,the default value of ImageNodeNum is 1,
                          If the user usesMV_CC_SetImageNodeNuminterface to set up OutputQueueSize,when the value of OutputQueueSize is set to be 1, 
                          the function will be same as LatestImagesOnly; if the value of OutputQueueSize is set to be ImageNodeNum, the function will be same as OneByOne.
             UpcomingImage:Ignore all images in output cache list when calling image acuiqisiotn interface, wait the next upcoming image generated. Support for GigE camera only, not support for U3V camera.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGrabStrategy(
  IN void * handle, IN MV_GRAB_STRATEGY enGrabStrategy);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置输出缓存个数（该接口只支持windows平台，只有在MV_GrabStrategy_LatestImages策略下才有效，范围：1-ImageNodeNum）
 *  @param  handle                      [IN]            设备句柄
 *  @param  nOutputQueueSize            [IN]            输出缓存个数
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 该接口需与LatestImages取流策略配套调用，用于设置LatestImages策略下最多允许缓存图像的个数。可以在取流过程中动态调节输出缓存个数
 
 *  @~english
 *  @brief  Set The Size of Output Queue(This interface only supports windows platform, Only work under the strategy of MV_GrabStrategy_LatestImages，rang：1-ImageNodeNum)
 *  @param  handle                      [IN]            Device handle
 *  @param  nOutputQueueSize            [IN]            The Size of Output Queue
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface must be used with LatestImages Grab strategy, it is used for setting the maximum allowance queue size of the image under the LatestImages strategy.
             The user may change the output queue size while grabbing images.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetOutputQueueSize(
  IN void * handle, IN unsigned int nOutputQueueSize);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取设备信息，取流之前调用
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstDevInfo                  [IN][OUT]       返回给调用者有关设备信息结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 支持用户在打开设备后获取设备信息，不支持GenTL设备
             若该设备是GigE设备，则调用该接口存在阻塞风险，因此不建议在取流过程中调用该接口。
 
 *  @~english
 *  @brief  Get device information
 *  @param  handle                      [IN]            Device handle
 *  @param  pstDevInfo                  [IN][OUT]       Structure pointer of device information
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks The API support users to access device information after opening the device，don't support GenTL Devices
             If the device is a GigE camera, there is a blocking risk in calling the interface, so it is not recommended to call the interface during the fetching process. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetDeviceInfo(
  IN void * handle, IN OUT MV_CC_DEVICE_INFO * pstDevInfo);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取各种类型的信息
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstInfo                     [IN][OUT]       返回给调用者有关设备各种类型的信息结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 接口里面输入需要获取的信息类型（指定MV_ALL_MATCH_INFO结构体中的nType类型），获取对应的信息（在MV_ALL_MATCH_INFO结构体中pInfo里返回）
             该接口的调用前置条件取决于所获取的信息类型，获取GigE设备的MV_MATCH_TYPE_NET_DETECT信息需在开启抓图之后调用，获取U3V设备的MV_MATCH_TYPE_USB_DETECT信息需在打开设备之后调用
             该接口不支持CameraLink设备。 
 
 *  @~english
 *  @brief  Get various type of information
 *  @param  handle                      [IN]            Device handle
 *  @param  pstInfo                     [IN][OUT]       Structure pointer of various type of information
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Input required information type (specify nType in structure MV_ALL_MATCH_INFO) in the interface and get corresponding information (return in pInfo of structure MV_ALL_MATCH_INFO)
             The calling precondition of this interface is determined by obtained information type. Call after enabling capture to get MV_MATCH_TYPE_NET_DETECT information of GigE device,
             and call after starting device to get MV_MATCH_TYPE_USB_DETECT information of USB3Vision device.
             This API is not supported by CameraLink device. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAllMatchInfo(
  IN void * handle, IN OUT MV_ALL_MATCH_INFO * pstInfo);

/*******************Part2 ch: 相机属性万能配置接口 | en: Camera attribute nodes universal interface*******************/

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取Integer属性值
 *  @param  handle                      [IN]            设备句柄
 *  @param  strKey                      [IN]            属性键值，如获取宽度信息则为"Width"
 *  @param  pstIntValue                 [IN][OUT]       返回给调用者有关设备属性结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 连接设备之后调用该接口可以获取int类型的指定节点的值。
             strKey取值可以参考XML节点参数类型列表，表格里面数据类型为“IInteger”的节点值都可以通过该接口获取，strKey参数取值对应列表里面的“名称”一列。
 
 *  @~english
 *  @brief  Get Integer value
 *  @param  handle                      [IN]            Device handle
 *  @param  strKey                      [IN]            Key value, for example, using "Width" to get width
 *  @param  pstIntValue                 [IN][OUT]       Structure pointer of camera features
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks You can call this API to get the value of camera node with integer type after connecting the device. For strKey value, refer to MvCameraNode. 
             All the node values of "IInteger" in the list can be obtained via this API. strKey corresponds to the Name column.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetIntValueEx(
  IN void * handle, IN const char * strKey, IN OUT MVCC_INTVALUE_EX * pstIntValue);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置Integer型属性值
 *  @param  handle                      [IN]            设备句柄
 *  @param  strKey                      [IN]            属性键值，如获取宽度信息则为"Width"
 *  @param  nValue                      [IN]            想要设置的设备的属性值
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 连接设备之后调用该接口可以设置int类型的指定节点的值。
             strKey取值可以参考XML节点参数类型列表，表格里面数据类型为“IInteger”的节点值都可以通过该接口设置，strKey参数取值对应列表里面的“名称”一列。
 
 *  @~english
 *  @brief  Set Integer value
 *  @param  handle                      [IN]            Device handle
 *  @param  strKey                      [IN]            Key value, for example, using "Width" to set width
 *  @param  nValue                      [IN]            Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks You can call this API to get the value of camera node with integer type after connecting the device. For strKey value, refer to MvCameraNode.
             All the node values of "IInteger" in the list can be obtained via this API. strKey corresponds to the Name column.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetIntValueEx(
  IN void * handle, IN const char * strKey, IN int64_t nValue);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取Enum属性值
 *  @param  handle                      [IN]            设备句柄
 *  @param  strKey                      [IN]            属性键值，如获取像素格式信息则为"PixelFormat"
 *  @param  pstEnumValue                [IN][OUT]       返回给调用者有关设备属性结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 连接设备之后调用该接口可以获取Enum类型的指定节点的值。
             strKey取值可以参考XML节点参数类型列表，表格里面数据类型为“IEnumeration”的节点值都可以通过该接口获取，strKey参数取值对应列表里面的“名称”一列。
 
 *  @~english
 *  @brief  Get Enum value
 *  @param  handle                      [IN]            Device handle
 *  @param  strKey                      [IN]            Key value, for example, using "PixelFormat" to get pixel format
 *  @param  pstEnumValue                [IN][OUT]       Structure pointer of camera features
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to get specified Enum nodes. For value of strKey, see MvCameraNode, 
             The node values of IEnumeration can be obtained through this interface, strKey value corresponds to the Name column.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetEnumValue(
  IN void * handle, IN const char * strKey, IN OUT MVCC_ENUMVALUE * pstEnumValue);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置Enum型属性值
 *  @param  handle                      [IN]            设备句柄
 *  @param  strKey                      [IN]            属性键值，如获取像素格式信息则为"PixelFormat"
 *  @param  nValue                      [IN]            想要设置的设备的属性值
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 连接设备之后调用该接口可以设置Enum类型的指定节点的值。
             strKey取值可以参考XML节点参数类型列表，表格里面数据类型为“IEnumeration”的节点值都可以通过该接口设置，strKey参数取值对应列表里面的“名称”一列。
 
 *  @~english
 *  @brief  Set Enum value
 *  @param  handle                      [IN]            Device handle
 *  @param  strKey                      [IN]            Key value, for example, using "PixelFormat" to set pixel format
 *  @param  nValue                      [IN]            Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to get specified Enum nodes. For value of strKey, see MvCameraNode,
             The node values of IEnumeration can be obtained through this interface, strKey value corresponds to the Name column.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetEnumValue(
  IN void * handle, IN const char * strKey, IN unsigned int nValue);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取Enum型节点指定值的符号
 *  @param  handle                      [IN]            设备句柄
 *  @param  strKey                      [IN]            属性键值，如获取像素格式信息则为"PixelFormat"
 *  @param  pstEnumEntry                [IN][OUT]           想要获取的设备的属性符号
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 连接设备之后调用该接口可以获取Enum类型的指定节点的值所对应的符号。
             strKey取值可以参考XML节点参数类型列表，表格里面数据类型为“IEnumeration”的节点值都可以通过该接口设置，strKey参数取值对应列表里面的“名称”一列。
 
 *  @~english
 *  @brief  Get the symbolic of the specified value of the Enum type node
 *  @param  handle                      [IN]            Device handle
 *  @param  strKey                      [IN]            Key value, for example, using "PixelFormat" to set pixel format
 *  @param  pstEnumEntry                [IN][OUT]           Symbolic to get
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Call this interface after connecting the device to obtain the symbol corresponding to the value of the specified node of Enum type.
			 The strKey value can refer to the XML node parameter type list. The node value with the data type "IEnumeration" in the table can be set through this interface. 
			 The strKey parameter value corresponds to the "Name" column in the list.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetEnumEntrySymbolic(
  IN void * handle, IN const char * strKey, IN OUT MVCC_ENUMENTRY * pstEnumEntry);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置Enum型属性值
 *  @param  handle                      [IN]            设备句柄
 *  @param  strKey                      [IN]            属性键值，如获取像素格式信息则为"PixelFormat"
 *  @param  strValue                    [IN]            想要设置的设备的属性字符串
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 连接设备之后调用该接口可以设置Enum类型的指定节点的值。
             strKey取值可以参考XML节点参数类型列表，表格里面数据类型为“IEnumeration”的节点值都可以通过该接口设置，strKey参数取值对应列表里面的“名称”一列。
 
 *  @~english
 *  @brief  Set Enum value
 *  @param  handle                      [IN]            Device handle
 *  @param  strKey                      [IN]            Key value, for example, using "PixelFormat" to set pixel format
 *  @param  strValue                    [IN]            Feature String to set
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Call this API after connecting the device. All the values of nodes with IEnumeration type can be set via this API.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetEnumValueByString(
  IN void * handle, IN const char * strKey, IN const char * strValue);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取Float属性值
 *  @param  handle                      [IN]            设备句柄
 *  @param  strKey                      [IN]            属性键值
 *  @param  pstFloatValue               [IN][OUT]       返回给调用者有关设备属性结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 连接设备之后调用该接口可以获取float类型的指定节点的值。
             strKey取值可以参考XML节点参数类型列表，表格里面数据类型为“IFloat”的节点值都可以通过该接口获取，strKey参数取值对应列表里面的“名称”一列。
 
 *  @~english
 *  @brief  Get Float value
 *  @param  handle                      [IN]            Device handle
 *  @param  strKey                      [IN]            Key value
 *  @param  pstFloatValue               [IN][OUT]       Structure pointer of camera features
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to get specified float node. For detailed strKey value see: MvCameraNode.
             The node values of IFloat can be obtained through this interface, strKey value corresponds to the Name column.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetFloatValue(
  IN void * handle, IN const char * strKey, IN OUT MVCC_FLOATVALUE * pstFloatValue);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置float型属性值
 *  @param  handle                      [IN]            设备句柄
 *  @param  strKey                      [IN]            属性键值
 *  @param  fValue                      [IN]            想要设置的设备的属性值
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 连接设备之后调用该接口可以设置float类型的指定节点的值。
             strKey取值可以参考XML节点参数类型列表，表格里面数据类型为“IFloat”的节点值都可以通过该接口设置，strKey参数取值对应列表里面的“名称”一列。
 
 *  @~english
 *  @brief  Set float value
 *  @param  handle                      [IN]            Device handle
 *  @param  strKey                      [IN]            Key value
 *  @param  fValue                      [IN]            Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to set specified float node. For detailed strKey value see: MvCameraNode. 
            The node values of IFloat can be set through this interface, strKey value corresponds to the Name column.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetFloatValue(
  IN void * handle, IN const char * strKey, IN float fValue);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取Boolean属性值
 *  @param  handle                      [IN]            设备句柄
 *  @param  strKey                      [IN]            属性键值
 *  @param  pbValue                     [IN][OUT]       返回给调用者有关设备属性值
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 连接设备之后调用该接口可以获取bool类型的指定节点的值。
             strKey取值可以参考XML节点参数类型列表，表格里面数据类型为“IBoolean”的节点值都可以通过该接口获取，strKey参数取值对应列表里面的“名称”一列。
 
 *  @~english
 *  @brief  Get Boolean value
 *  @param  handle                      [IN]            Device handle
 *  @param  strKey                      [IN]            Key value
 *  @param  pbValue                     [IN][OUT]       Structure pointer of camera features
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to get specified bool nodes. For value of strKey, see MvCameraNode.
             The node values of IBoolean can be obtained through this interface, strKey value corresponds to the Name column.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBoolValue(
  IN void * handle, IN const char * strKey, IN OUT bool * pbValue);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置Boolean型属性值
 *  @param  handle                      [IN]            设备句柄
 *  @param  strKey                      [IN]            属性键值
 *  @param  bValue                      [IN]            想要设置的设备的属性值
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 连接设备之后调用该接口可以设置bool类型的指定节点的值。
             strKey取值可以参考XML节点参数类型列表，表格里面数据类型为“IBoolean”的节点值都可以通过该接口设置，strKey参数取值对应列表里面的“名称”一列。
 
 *  @~english
 *  @brief  Set Boolean value
 *  @param  handle                      [IN]            Device handle
 *  @param  strKey                      [IN]            Key value
 *  @param  bValue                      [IN]            Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to set specified bool nodes. For value of strKey, see MvCameraNode. 
             The node values of IBoolean can be set through this interface, strKey value corresponds to the Name column.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBoolValue(
  IN void * handle, IN const char * strKey, IN bool bValue);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取String属性值
 *  @param  handle                      [IN]            设备句柄
 *  @param  strKey                      [IN]            属性键值
 *  @param  pstStringValue              [IN][OUT]       返回给调用者有关设备属性结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 连接设备之后调用该接口可以获取string类型的指定节点的值。
             strKey取值可以参考XML节点参数类型列表，表格里面数据类型为“IString”的节点值都可以通过该接口获取，strKey参数取值对应列表里面的“名称”一列。
 
 *  @~english
 *  @brief  Get String value
 *  @param  handle                      [IN]            Device handle
 *  @param  strKey                      [IN]            Key value
 *  @param  pstStringValue              [IN][OUT]       Structure pointer of camera features
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to get specified string nodes. For value of strKey, see MvCameraNode.
             The node values of IString can be obtained through this interface, strKey value corresponds to the Name column.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetStringValue(
  IN void * handle, IN const char * strKey, IN OUT MVCC_STRINGVALUE * pstStringValue);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置String型属性值
 *  @param  handle                      [IN]            设备句柄
 *  @param  strKey                      [IN]            属性键值
 *  @param  strValue                    [IN]            想要设置的设备的属性值
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 连接设备之后调用该接口可以设置string类型的指定节点的值。
             strKey取值可以参考XML节点参数类型列表，表格里面数据类型为“IString”的节点值都可以通过该接口设置，strKey参数取值对应列表里面的“名称”一列。
 
 *  @~english
 *  @brief  Set String value
 *  @param  handle                      [IN]            Device handle
 *  @param  strKey                      [IN]            Key value
 *  @param  strValue                    [IN]            Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to set specified string nodes. For value of strKey, see MvCameraNode.
             The node values of IString can be set through this interface, strKey value corresponds to the Name column.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetStringValue(
  IN void * handle, IN const char * strKey, IN const char * strValue);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置Command型属性值
 *  @param  handle                      [IN]            设备句柄
 *  @param  strKey                      [IN]            属性键值
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 连接设备之后调用该接口可以设置指定的Command类型节点。
             strKey取值可以参考XML节点参数类型列表，表格里面数据类型为“ICommand”的节点都可以通过该接口设置，strKey参数取值对应列表里面的“名称”一列。
 
 *  @~english
 *  @brief  Send Command
 *  @param  handle                      [IN]            Device handle
 *  @param  strKey                      [IN]            Key value
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to set specified Command nodes. For value of strKey, see MvCameraNode. 
             The node values of ICommand can be set through this interface, strKey value corresponds to the Name column.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetCommandValue(IN void * handle, IN const char * strKey);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  读内存
 *  @param  handle                      [IN]            设备句柄
 *  @param  pBuffer                     [IN][OUT]       作为返回值使用，保存读到的内存值（GEV设备内存值是按照大端模式存储的，其它协议设备按照小端存储）
 *  @param  nAddress                    [IN]            待读取的内存地址，该地址可以从设备的Camera.xml文件中获取，形如xxx_RegAddr的xml节点值
 *  @param  nLength                     [IN]            待读取的内存长度
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 访问设备，读取某段寄存器的数据。
 
 *  @~english
 *  @brief  Read Memory
 *  @param  handle                      [IN]            Device Handle
 *  @param  pBuffer                     [IN][OUT]       Used as a return value, save the read-in memory value ( Memory value is stored in accordance with the big end model)
 *  @param  nAddress                    [IN]            Memory address to be read, which can be obtained from the Camera.xml file of the device, the form xml node value of xxx_RegAddr
 *  @param  nLength                     [IN]            Length of the memory to be read
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Access device, read the data from certain register.
*************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ReadMemory(
  IN void * handle, IN OUT void * pBuffer, IN int64_t nAddress, IN int64_t nLength);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  写内存
 *  @param  handle                      [IN]            设备句柄
 *  @param  pBuffer                     [IN]            待写入的内存值（注意GEV设备内存值要按照大端模式存储，其它协议设备按照小端存储）
 *  @param  nAddress                    [IN]            待写入的内存地址，该地址可以从设备的Camera.xml文件中获取，形如xxx_RegAddr的xml节点值
 *  @param  nLength                     [IN]            待写入的内存长度
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 访问设备，把一段数据写入某段寄存器。
 
 *  @~english
 *  @brief  Write Memory
 *  @param  handle                      [IN]            Device Handle
 *  @param  pBuffer                     [IN]            Memory value to be written ( Note the memory value to be stored in accordance with the big end model)
 *  @param  nAddress                    [IN]            Memory address to be written, which can be obtained from the Camera.xml file of the device, the form xml node value of xxx_RegAddr
 *  @param  nLength                     [IN]            Length of the memory to be written
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Access device, write a piece of data into a certain segment of register.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_WriteMemory(
  IN void * handle, IN const void * pBuffer, IN int64_t nAddress, IN int64_t nLength);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  清除GenICam节点缓存
 *  @param  handle                      [IN]            设备句柄
 *  @return 成功，返回MV_OK；错误，返回错误码
 
 *  @~english
 *  @brief  Invalidate GenICam Nodes
 *  @param  handle                      [IN]            Device handle
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_InvalidateNodes(IN void * handle);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取设备属性树XML
 *  @param  handle                      [IN]            设备句柄
 *  @param  pData                       [IN][OUT]       XML数据接收缓存
 *  @param  nDataSize                   [IN]            接收缓存大小
 *  @param  pnDataLen                   [IN][OUT]       实际数据大小
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 当pData为NULL或nDataSize比实际的xml文件小时，不拷贝数据，由pnDataLen返回xml文件大小
             当pData为有效缓存地址，且缓存足够大时，拷贝完整数据保存在该缓存里面，并由pnDataLen返回xml文件实际大小。
 
 *  @~english
 *  @brief  Get camera feature tree XML
 *  @param  handle                      [IN]            Device handle
 *  @param  pData                       [IN][OUT]       XML data receiving buffer
 *  @param  nDataSize                   [IN]            Buffer size
 *  @param  pnDataLen                   [IN][OUT]       Actual data length
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks When pData is NULL or nDataSize than the actual XML file hours, do not copy the data, returned by pnDataLen XML file size.
             When pData is a valid cache address and the cache is large enough, copy the full data into the cache, and pnDataLen returns the actual size of the XML file.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetGenICamXML(
  IN void * handle, IN OUT unsigned char * pData, IN unsigned int nDataSize,
  IN OUT unsigned int * pnDataLen);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获得当前节点的访问模式
 *  @param  handle                      [IN]            设备句柄
 *  @param  strName                     [IN]            节点名称
 *  @param  penAccessMode               [IN][OUT]       节点的访问模式
 *  @return 成功，返回MV_OK；错误，返回错误码

 *  @~english
 *  @brief  Get Access mode of cur node
 *  @param  handle                      [IN]            Device handle
 *  @param  strName                     [IN]            Name of node
 *  @param  penAccessMode               [IN][OUT]       Access mode of the node
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetNodeAccessMode(
  IN void * handle, IN const char * strName, IN OUT MV_XML_AccessMode * penAccessMode);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获得当前节点的类型
 *  @param  handle                      [IN]            设备句柄
 *  @param  strName                     [IN]            节点名称
 *  @param  penInterfaceType            [IN][OUT]       节点的类型
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 该接口可以在调用万能接口之前，提前知道节点类型，方便用户选择合适的万能接口进行节点值的设置和获取。

 *  @~english
 *  @brief  Get Interface Type of cur node
 *  @param  handle                      [IN]            Device handle
 *  @param  strName                     [IN]            Name of node
 *  @param  penInterfaceType            [IN][OUT]       Interface Type of the node
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks The interface can know the node type in advance before calling the universal interface, so as to facilitate users to select the appropriate universal interface for setting and obtaining the node value.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetNodeInterfaceType(
  IN void * handle, IN const char * strName, IN OUT MV_XML_InterfaceType * penInterfaceType);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  保存设备属性
 *  @param  handle                      [IN]            设备句柄
 *  @param  strFileName                 [IN]            属性文件名
 *  @return 成功，返回MV_OK；错误，返回错误码 
 
 *  @~english
 *  @brief  Save camera feature
 *  @param  handle                      [IN]            Device handle
 *  @param  strFileName                 [IN]            File name
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FeatureSave(IN void * handle, IN const char * strFileName);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  导入设备属性
 *  @param  handle                      [IN]            设备句柄
 *  @param  strFileName                 [IN]            属性文件名
 *  @return 成功，返回MV_OK；错误，返回错误码 
 
 *  @~english
 *  @brief  Load camera feature
 *  @param  handle                      [IN]            Device handle
 *  @param  strFileName                 [IN]            File name
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FeatureLoad(IN void * handle, IN const char * strFileName);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  从设备读取文件
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstFileAccess               [IN]            文件存取结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 
 *  @~english
 *  @brief  Read the file from the camera
 *  @param  handle                      [IN]            Device handle
 *  @param  pstFileAccess               [IN]            File access structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FileAccessRead(
  IN void * handle, IN MV_CC_FILE_ACCESS * pstFileAccess);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  从设备读取文件,文件是Data数据
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstFileAccess               [IN]            文件存取结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 避免文件操作权限问题读失败 
 
 *  @~english
 *  @brief  Read the file data from the camera
 *  @param  handle                      [IN]            Device handle
 *  @param  pstFileAccess               [IN]            File access structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FileAccessReadEx(
  IN void * handle, IN OUT MV_CC_FILE_ACCESS_EX * pstFileAccessEx);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  将文件写入设备
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstFileAccess               [IN]            文件存取结构体
 *  @return 成功，返回MV_OK；错误，返回错误码
 
 *  @~english
 *  @brief  Write the file to camera
 *  @param  handle                      [IN]            Device handle
 *  @param  pstFileAccess               [IN]            File access structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FileAccessWrite(
  IN void * handle, IN MV_CC_FILE_ACCESS * pstFileAccess);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  将缓存(buffer)写入设备
 *  @param  handle                        [IN]            设备句柄
 *  @param  pstFileAccessEx               [IN][OUT]       文件存取结构体
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks 该接口直接使用缓存数据，进行读写操作，避免直接操作文件出现无权限的问题,是MV_CC_FileAccessWrite的扩展接口
 
 *  @~english
 *  @brief  Write the data(buffer) to camera
 *  @param  handle                        [IN]            Device handle
 *  @param  pstFileAccessEx               [IN][OUT]       File access structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface uses cached data for read and write,solve the problem of no permissions in direct operation files, it's an extended interface of MV_CC_FileAccessWrite.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FileAccessWriteEx(
  IN void * handle, IN OUT MV_CC_FILE_ACCESS_EX * pstFileAccessEx);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取文件存取的进度
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstFileAccessProgress       [IN][OUT]       进度内容
 *  @return 成功，返回MV_OK；错误，返回错误码 （当前文件存取的状态）
 
 *  @~english
 *  @brief  Get File Access Progress 
 *  @param  handle                      [IN]            Device handle
 *  @param  pstFileAccessProgress       [IN][OUT]       File access Progress
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetFileAccessProgress(
  IN void * handle, IN OUT MV_CC_FILE_ACCESS_PROGRESS * pstFileAccessProgress);

/*******************Part3 ch: 相机升级 | en: Camera upgrade *******************/

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设备本地升级
 *  @param  handle                      [IN]            设备句柄
 *  @param  strFilePathName             [IN]            文件名
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 通过该接口可以将升级固件文件发送给设备进行升级。
             该接口需要等待升级固件文件成功传给设备端之后再返回，响应时间可能较长。
 
 *  @~english
 *  @brief  Device Local Upgrade
 *  @param  handle                      [IN]            Device handle
 *  @param  strFilePathName             [IN]            File name
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Call this API to send the upgrade firmware to the device for upgrade.
             This API will wait for return until the upgrade firmware is sent to the device, this response may take a long time.
             For CameraLink device, it keeps sending upgrade firmware continuously. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LocalUpgrade(IN void * handle, IN const void * strFilePathName);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取升级进度
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnProcess                   [IN][OUT]       进度接收地址
 *  @return 成功,返回MV_OK,失败,返回错误码
 
 *  @~english
 *  @brief  Get Upgrade Progress
 *  @param  handle                      [IN]            Device handle
 *  @param  pnProcess                   [IN][OUT]       Progress receiving address
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetUpgradeProcess(
  IN void * handle, IN OUT unsigned int * pnProcess);

/*******************Part4  ch: 注册异常回调和事件接口 | en: Enrol abnormal callbacks and event interface*******************/

/********************************************************************/ /**
 *  @~chinese
 *  @brief  注册异常消息回调，在打开设备之后调用
 *  @param  handle                      [IN]            设备句柄
 *  @param  cbException                 [IN]            异常回调函数指针
 *  @param  pUser                       [IN]            用户自定义变量
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 该接口需要在MV_CC_OpenDevice打开设备之后调用。
             设备异常断开连接后可以在回调里面获取到异常消息，GigE设备掉线之后需要先调用MV_CC_CloseDevice接口关闭设备，再调用MV_CC_OpenDevice接口重新打开设备。
 
 *  @~english
 *  @brief  Register Exception Message CallBack, call after open device
 *  @param  handle                      [IN]            Device handle
 *  @param  cbException                 [IN]            Exception Message CallBack Function Pointer
 *  @param  pUser                       [IN]            User defined variable
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Call this interface after the device is opened by MV_CC_OpenDevice. 
             When device is exceptionally disconnected, the exception message can be obtained from callback function. For Disconnected GigE device,
             first call MV_CC_CloseDevice to shut device, and then call MV_CC_OpenDevice to reopen the device. 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterExceptionCallBack(
  IN void * handle, IN void(__stdcall * cbException)(unsigned int nMsgType, void * pUser),
  IN void * pUser);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  注册全部事件回调，在打开设备之后调用
 *  @param  handle                      [IN]            设备句柄
 *  @param  cbEvent                     [IN]            事件回调函数指针
 *  @param  pUser                       [IN]            用户自定义变量
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 通过该接口设置事件回调，可以在回调函数里面获取采集、曝光等事件信息
             该接口不支持CameraLink设备。
 
 *  @~english
 *  @brief  Register event callback, which is called after the device is opened
 *  @param  handle                      [IN]            Device handle
 *  @param  cbEvent                     [IN]            Event CallBack Function Pointer
 *  @param  pUser                       [IN]            User defined variable
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Call this API to set the event callback function to get the event information, e.g., acquisition, exposure, and so on
             This API is not supported by CameraLink device.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterAllEventCallBack(
  IN void * handle, IN void(__stdcall * cbEvent)(MV_EVENT_OUT_INFO * pEventInfo, void * pUser),
  IN void * pUser);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  注册单个事件回调，在打开设备之后调用
 *  @param  handle                      [IN]            设备句柄
 *  @param  strEventName                [IN]            事件名称
 *  @param  cbEvent                     [IN]            事件回调函数指针
 *  @param  pUser                       [IN]            用户自定义变量
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 通过该接口设置事件回调，可以在回调函数里面获取采集、曝光等事件信息。
             该接口不支持CameraLink设备。
 
 *  @~english
 *  @brief  Register single event callback, which is called after the device is opened
 *  @param  handle                      [IN]            Device handle
 *  @param  strEventName                [IN]            Event name
 *  @param  cbEvent                     [IN]            Event CallBack Function Pointer
 *  @param  pUser                       [IN]            User defined variable
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Call this API to set the event callback function to get the event information, e.g., acquisition, exposure, and so on.
             This API is not supported by CameraLink device .
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterEventCallBackEx(
  IN void * handle, IN const char * strEventName,
  IN void(__stdcall * cbEvent)(MV_EVENT_OUT_INFO * pEventInfo, void * pUser), IN void * pUser);

/*******************Part5 ch: 仅GigE设备支持的接口 | en: Only support GigE interface*******************/

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置枚举超时时间，仅支持GigE协议，范围 1 - UINT_MAX，（包括1，不包括UINT_MAX）
 *  @param  nMilTimeout                 [IN]            超时时间，默认100ms
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 在调用MV_CC_EnumDevices等枚举接口前使用该接口，可设置枚举GIGE设备的网卡最大超时时间（默认100ms）,可以减少最大超时时间，来加快枚举GIGE设备的速度
 *  @remarks 该接口仅支持输入无符号整数

 *  @~english
 *  @brief  Set enumerate device timeout,only support GigE,range 1 - UINT_MAX,(the minimum value is greater than or equal to 1,maximum value less than UINT_MAX)
 *  @param  nMilTimeout                 [IN]            time out,default 100ms
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Before calling enum device interfaces,call MV_GIGE_SetEnumDevTimeout to set max timeout,can reduce the maximum timeout to speed up the enumeration of GigE devices.
 *  @remarks This API only support input of unsigned int.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetEnumDevTimeout(IN unsigned int nMilTimeout);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  强制IP
 *  @param  handle                      [IN]            设备句柄
 *  @param  nIP                         [IN]            设置的IP
 *  @param  nSubNetMask                 [IN]            子网掩码
 *  @param  nDefaultGateWay             [IN]            默认网关
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 强制设置设备网络参数（包括IP、子网掩码、默认网关），强制设置之后将需要重新创建设备句柄，仅GigEVision设备支持
             如果设备为DHCP的状态，调用该接口强制设置设备网络参数之后设备将会重启。
 
 *  @~english
 *  @brief  Force IP
 *  @param  handle                      [IN]            Device handle
 *  @param  nIP                         [IN]            IP to set
 *  @param  nSubNetMask                 [IN]            Subnet mask
 *  @param  nDefaultGateWay             [IN]            Default gateway
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Force setting camera network parameter (including IP address, subnet mask, default gateway). After forced setting, device handle should be created again. 
             This function is only supported by GigEVision camera.
             If device is in DHCP status, after calling this API to force setting camera network parameter, the device will restart.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_ForceIpEx(
  IN void * handle, IN unsigned int nIP, IN unsigned int nSubNetMask,
  IN unsigned int nDefaultGateWay);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  配置IP方式
 *  @param  handle                      [IN]            设备句柄
 *  @param  nType                       [IN]            IP类型，见MV_IP_CFG_x
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 发送命令设置设备的IP方式，如DHCP、LLA等，仅GigEVision设备支持。
 
 *  @~english
 *  @brief  IP configuration method
 *  @param  handle                      [IN]            Device handle
 *  @param  nType                       [IN]            IP type, refer to MV_IP_CFG_x
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Send command to set camera IP mode, such as DHCP and LLA, only supported by GigEVision.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetIpConfig(IN void * handle, IN unsigned int nType);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置仅使用某种模式,type: MV_NET_TRANS_x，不设置时，默认优先使用driver
 *  @param  handle                      [IN]            设备句柄
 *  @param  nType                       [IN]            网络传输模式，见MV_NET_TRANS_x
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 通过该接口可以设置SDK内部优先使用的网络模式，默认优先使用驱动模式，仅GigEVision设备支持。
 
 *  @~english
 *  @brief  Set to use only one mode,type: MV_NET_TRANS_x. When do not set, priority is to use driver by default
 *  @param  handle                      [IN]            Device handle
 *  @param  nType                       [IN]            Net transmission mode, refer to MV_NET_TRANS_x
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarksSet SDK internal priority network mode through this interface, drive mode by default, only supported by GigEVision camera.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetNetTransMode(IN void * handle, IN unsigned int nType);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取网络传输信息
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstInfo                     [IN][OUT]       信息结构体
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 通过该接口可以获取网络传输相关信息，包括已接收数据大小、丢帧数量等，在MV_CC_StartGrabbing开启采集之后调用。仅GigEVision设备支持。
 
 *  @~english
 *  @brief  Get net transmission information
 *  @param  handle                      [IN]            Device handle
 *  @param  pstInfo                     [IN][OUT]       Information Structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Get network transmission information through this API, including received data size, number of lost frames.
             Call this API after starting image acquiring through MV_CC_StartGrabbing. This API is supported only by GigEVision Camera.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetNetTransInfo(
  IN void * handle, IN OUT MV_NETTRANS_INFO * pstInfo);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置枚举命令的回复包类型
 *  @param  nMode                       [IN]            回复包类型（默认广播），0-单播，1-广播
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口只对GigE相机有效。

 *  @~english
 *  @brief  Setting the ACK mode of devices Discovery.
 *  @param  nMode                       [IN]            ACK mode（Default-Broadcast）,0-Unicast,1-Broadcast.
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This interface is ONLY effective on GigE cameras.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetDiscoveryMode(IN unsigned int nMode);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置GVSP取流超时时间
 *  @param  handle                      [IN]            设备句柄
 *  @param  nMillisec                   [IN]            超时时间，默认300ms，范围：>10ms
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 连接设备之后，取流动作发生前，调用该接口可以设置GVSP取流超时时间。GVSP取流超时设置过短可能造成图像异常，设置过长可能造成取流时间变长。
 
 *  @~english
 *  @brief  Set GVSP streaming timeout
 *  @param  handle                      [IN]            Device handle
 *  @param  nMillisec                   [IN]            Timeout, default 300ms, range: >10ms
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, and just before start streaming, 
 *           call this interface to set GVSP streaming timeout value.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGvspTimeout(IN void * handle, IN unsigned int nMillisec);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取GVSP取流超时时间
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnMillisec                  [IN][OUT]       超时时间指针，以毫秒位单位
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口用于获取当前的GVSP取流超时时间
 
 *  @~english
 *  @brief  Get GVSP streaming timeout
 *  @param  handle                      [IN]            Device handle
 *  @param  pnMillisec                  [IN][OUT]       Timeout, ms as unit
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface is used to get the current GVSP streaming timeout.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGvspTimeout(
  IN void * handle, IN OUT unsigned int * pnMillisec);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置GVCP命令超时时间
 *  @param  handle                      [IN]            设备句柄
 *  @param  nMillisec                   [IN]            超时时间，默认500ms，范围：0-10000ms
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 连接设备之后调用该接口可以设置GVCP命令超时时间。
 
 *  @~english
 *  @brief  Set GVCP cammand timeout
 *  @param  handle                      [IN]            Device handle
 *  @param  nMillisec                   [IN]            Timeout, default 500ms, range: 0-10000ms
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to set GVCP command timeout time.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGvcpTimeout(IN void * handle, IN unsigned int nMillisec);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取GVCP命令超时时间
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnMillisec                  [IN][OUT]       超时时间指针，以毫秒位单位
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口用于获取当前的GVCP超时时间。
 
 *  @~english
 *  @brief  Get GVCP cammand timeout
 *  @param  handle                      [IN]            Device handle
 *  @param  pnMillisec                  [IN][OUT]       Timeout, ms as unit
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface is used to get the current GVCP timeout.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGvcpTimeout(
  IN void * handle, IN OUT unsigned int * pnMillisec);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置重传GVCP命令次数
 *  @param  handle                      [IN]            设备句柄
 *  @param  nRetryGvcpTimes             [IN]            重传次数，范围：0-100
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口用于在GVCP包传输异常时，增加重传的次数，在一定程度上可以避免设备掉线，范围为0-100。
 
 *  @~english
 *  @brief  Set the number of retry GVCP cammand
 *  @param  handle                      [IN]            Device handle
 *  @param  nRetryGvcpTimes             [IN]            The number of retries，rang：0-100
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface is used to increase The Times of retransmission when GVCP packet transmission is abnormal,and to some extent,
             it can avoid dropping the camera, with a range of 0-100.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetRetryGvcpTimes(
  IN void * handle, IN unsigned int nRetryGvcpTimes);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取重传GVCP命令次数
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnRetryGvcpTimes            [IN][OUT]       重传次数指针
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口用于获取当前的GVCP重传次数，默认3次。
 
 *  @~english
 *  @brief  Get the number of retry GVCP cammand
 *  @param  handle                      [IN]            Device handle
 *  @param  pnRetryGvcpTimes            [IN][OUT]       The number of retries
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface is used to get the current number of GVCP retransmissions, which defaults to 3.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetRetryGvcpTimes(
  IN void * handle, IN OUT unsigned int * pnRetryGvcpTimes);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取最佳的packet size，该接口目前只支持GigE设备
 *  @param  handle                      [IN]            设备句柄
 *  @return 最佳packetsize
 *  @remarks 获取最佳的packet size，对应GigEVision设备是SCPS。
             该接口需要在MV_CC_OpenDevice之后、MV_CC_StartGrabbing之前调用。
             该接口不支持CameraLink设备、U3V设备。
			 该接口不支持GenTL设备（协议不支持）,如果是GenTL方式添加的网口相机，建议根据网络实际情况配置GevSCPSPacketSize，或者配置1500。
 
 *  @~english
 *  @brief  Get the optimal Packet Size, Only support GigE Camera
 *  @param  handle                      [IN]            Device handle
 *  @return Optimal packetsize
 *  @remarks To get optimized packet size, for GigEVision device is SCPS, for USB3Vision device is the size of packet read from drive each time, 
             and it is the size of a packet transported on the network. The interface should be called after MV_CC_OpenDevice and before MV_CC_StartGrabbing.
             This API is not supported by CameraLink device and U3V device. 
			 This interface does not support GenTL devices (protocol not supported). If a network camera is added in GenTL mode, it is recommended to configure GevSCPSPacketSize according to the actual network situation,or 1500.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetOptimalPacketSize(IN void * handle);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置是否打开重发包支持，及重发包设置
 *  @param  handle                      [IN]            设备句柄
 *  @param  bEnable                     [IN]            是否支持重发包
 *  @param  nMaxResendPercent           [IN]            最大重发比
 *  @param  nResendTimeout              [IN]            重发超时时间
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 连接设备之后调用该接口可以设置重发包属性，仅GigEVision设备支持。
 
 *  @~english
 *  @brief  Set whethe to enable resend, and set resend
 *  @param  handle                      [IN]            Device handle
 *  @param  bEnable                     [IN]            enable resend
 *  @param  nMaxResendPercent           [IN]            Max resend persent
 *  @param  nResendTimeout              [IN]            Resend timeout
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to set resend packet properties, only supported by GigEVision camera.
 ************************************************************************/
#ifndef __cplusplus
MV_CAMCTRL_API int __stdcall MV_GIGE_SetResend(
  IN void * handle, IN unsigned int bEnable, IN unsigned int nMaxResendPercent,
  IN unsigned int nResendTimeout);
#else
MV_CAMCTRL_API int __stdcall MV_GIGE_SetResend(
  IN void * handle, IN unsigned int bEnable, IN unsigned int nMaxResendPercent = 10,
  IN unsigned int nResendTimeout = 50);
#endif

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置重传命令最大尝试次数
 *  @param  handle                      [IN]            设备句柄
 *  @param  nRetryTimes                 [IN]            重传命令最大尝试次数，默认20
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口必须在调用MV_GIGE_SetResend开启重传包功能之后调用，否则失败且返回MV_E_CALLORDER
 
 *  @~english
 *  @brief  set the max resend retry times
 *  @param  handle                      [IN]            Device handle
 *  @param  nRetryTimes                 [IN]            The max times to retry resending lost packets，default 20
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface MUST be called after enabling resending lost packets by calling MV_GIGE_SetResend,
 *           otherwise would fail and return MV_E_CALLORDER.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetResendMaxRetryTimes(
  IN void * handle, IN unsigned int nRetryTimes);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取重传命令最大尝试次数
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnRetryTimes                [IN][OUT]       重传命令最大尝试次数
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口必须在调用MV_GIGE_SetResend开启重传包功能之后调用，否则失败且返回MV_E_CALLORDER
 
 *  @~english
 *  @brief  get the max resend retry times
 *  @param  handle                      [IN]            Device handle
 *  @param  pnRetryTimes                [IN][OUT]       The max times to retry resending lost packets
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface MUST be called after enabling resending lost packets by calling MV_GIGE_SetResend,
 *           otherwise would fail and return MV_E_CALLORDER. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetResendMaxRetryTimes(
  IN void * handle, IN OUT unsigned int * pnRetryTimes);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置同一重传包多次请求之间的时间间隔
 *  @param  handle                      [IN]            设备句柄
 *  @param  nMillisec                   [IN]            同一重传包多次请求之间的时间间隔，默认10ms
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口必须在调用MV_GIGE_SetResend开启重传包功能之后调用，否则失败且返回MV_E_CALLORDER
 
 *  @~english
 *  @brief  set time interval between same resend requests
 *  @param  handle                      [IN]            Device handle
 *  @param  nMillisec                   [IN]            The time interval between same resend requests,default 10ms
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface MUST be called after enabling resending lost packets by calling MV_GIGE_SetResend,
 *           otherwise would fail and return MV_E_CALLORDER. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetResendTimeInterval(
  IN void * handle, IN unsigned int nMillisec);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取同一重传包多次请求之间的时间间隔
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnMillisec                  [IN][OUT]       同一重传包多次请求之间的时间间隔
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口必须在调用MV_GIGE_SetResend开启重传包功能之后调用，否则失败且返回MV_E_CALLORDER
 
 *  @~english
 *  @brief  get time interval between same resend requests
 *  @param  handle                      [IN]            Device handle
 *  @param  pnMillisec                  [IN][OUT]       The time interval between same resend requests
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface MUST be called after enabling resending lost packets by calling MV_GIGE_SetResend,
 *           otherwise would fail and return MV_E_CALLORDER. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetResendTimeInterval(
  IN void * handle, IN OUT unsigned int * pnMillisec);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置传输模式，可以为单播模式、组播模式等
 *  @param  handle                      [IN]            设备句柄
 *  @param  stTransmissionType          [IN]            传输模式结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 通过该接口可以设置传输模式为单播、组播等模式，仅GigEVision设备支持。

 *  @~english
 *  @brief  Set transmission type,Unicast or Multicast
 *  @param  handle                      [IN]            Device handle
 *  @param  stTransmissionType          [IN]            Struct of transmission type
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Call this API to set the transmission mode as single cast mode and multicast mode. And this API is only valid for GigEVision camera. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetTransmissionType(
  IN void * handle, IN MV_TRANSMISSION_TYPE * pstTransmissionType);

/********************************************************************/ /**
 *  @~chinese
 *  @brief   发出动作命令
 *  @param   pstActionCmdInfo           [IN]            动作命令信息
 *  @param   pstActionCmdResults        [IN][OUT]       动作命令返回信息列表
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 仅GigEVision设备支持。

 *  @~english
 *  @brief  Issue Action Command
 *  @param   pstActionCmdInfo           [IN]            Action Command
 *  @param   pstActionCmdResults        [IN][OUT]       Action Command Result List
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API is supported only by GigEVision camera.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_IssueActionCommand(
  IN MV_ACTION_CMD_INFO * pstActionCmdInfo, IN OUT MV_ACTION_CMD_RESULT_LIST * pstActionCmdResults);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取组播状态
 *  @param  pstDevInfo                  [IN]            设备信息结构体
 *  @param  pbStatus                    [IN][OUT]       组播状态,true:组播状态，false:非组播
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口用于判断设备当前是否处于组播状态，解决客户端枚举时需要打开设备判断组播的问题。

 *  @~english
 *  @brief  Get Multicast Status
 *  @param  pstDevInfo                  [IN]            Device Information Structure
 *  @param  pbStatus                    [IN][OUT]       Status of Multicast
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface is used to determine whether the camera is currently in multicast state, 
             and to solve the problem that the client needs to turn on the camera to determine multicast when enumerating.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetMulticastStatus(
  IN MV_CC_DEVICE_INFO * pstDevInfo, IN OUT bool * pbStatus);

/*******************Part6 ch: 仅CameraLink 设备支持的接口 | en: Only support camlink device interface*******************/

/***********************************************************************************************************/ /**
 *  @~chinese
 *  @brief  设置设备波特率
 *  @param  handle                      [IN]            设备句柄
 *  @param  nBaudrate                   [IN]            设置的波特率值，数值参考CameraParams.h中宏定义，如#define MV_CAML_BAUDRATE_9600  0x00000001
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 该接口支持在设备未连接时调用。
             因硬件/系统/外部干扰等因素,配置高波特率可能导致通信异常，建议配置波特率最大小于115200
 
 *  @~english
 *  @brief  Set device baudrate using one of the CL_BAUDRATE_XXXX value   
 *  @param  handle                      [IN]            Device handle
 *  @param  nBaudrate                   [IN]            baud rate to set. Refer to the 'CameraParams.h' for parameter definitions, for example, #define MV_CAML_BAUDRATE_9600  0x00000001
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This API is supported only by CameraLink device.
             This API support calls when devices are not connected.
             Due to hardware/system/external interference and other factors, configuring a high baud rate may cause abnormal communication. 
             It is recommended to configure a baud rate of less than 115200
************************************************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_SetDeviceBaudrate(IN void * handle, IN unsigned int nBaudrate);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取设备波特率
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnCurrentBaudrate           [IN][OUT]       波特率信息指针，数值参考CameraParams.h中宏定义，如#define MV_CAML_BAUDRATE_9600  0x00000001
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 该接口支持在设备未连接时调用。
 
 *  @~english
 *  @brief  Returns the current device baudrate, using one of the CL_BAUDRATE_XXXX value
 *  @param  handle                      [IN]            Device handle
 *  @param  pnCurrentBaudrate           [IN][OUT]       Return pointer of baud rate to user. Refer to the 'CameraParams.h' for parameter definitions, for example, #define MV_CAML_BAUDRATE_9600  0x00000001
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This API is supported only by CameraLink device.
             This API support calls when devices are not connected.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_GetDeviceBaudrate(
  IN void * handle, IN OUT unsigned int * pnCurrentBaudrate);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取设备与主机间连接支持的波特率
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnBaudrateAblity            [IN][OUT]       支持的波特率信息的指针。所支持波特率的或运算结果，单个数值参考CameraParams.h中宏定义，如MV_CAML_BAUDRATE_9600  0x00000001
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 该接口支持在设备未连接时调用。
 
 *  @~english
 *  @brief  Returns supported baudrates of the combined device and host interface
 *  @param  handle                      [IN]            Device handle
 *  @param  pnBaudrateAblity            [IN][OUT]       Return pointer of the supported baudrates to user. 'OR' operation results of the supported baudrates. Refer to the 'CameraParams.h' for single value definitions, for example, MV_CAML_BAUDRATE_9600  0x00000001
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This API is supported only by CameraLink device.
             This API support calls when devices are not connected.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_GetSupportBaudrates(
  IN void * handle, IN OUT unsigned int * pnBaudrateAblity);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置串口操作等待时长
 *  @param  handle                      [IN]            设备句柄
 *  @param  nMillisec                   [IN]            串口操作的等待时长, 单位为ms
 *  @return 成功,返回MV_OK,失败,返回错误码

 *  @~english
 *  @brief  Sets the timeout for operations on the serial port
 *  @param  handle                      [IN]            Device handle
 *  @param  nMillisec                   [IN]            Timeout in [ms] for operations on the serial port.
 *  @return Success, return MV_OK. Failure, return error code 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_SetGenCPTimeOut(IN void * handle, IN unsigned int nMillisec);

/*******************Part7 ch: 仅U3V设备支持的接口 | en: Only support U3V device interface*******************/

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置U3V的传输包大小
 *  @param  handle                      [IN]            设备句柄
 *  @param  nTransferSize               [IN]            传输的包大小, Byte，默认为1M，rang：>=0x400，建议最大值：[windows] rang <= 0x400000；[Linux] rang <= 0x200000
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 增加传输包大小可以适当降低取流时的CPU占用率。但不同的PC和不同USB扩展卡存在不同的兼容性，如果该参数设置过大可能会出现取不到图像的风险。
 
 *  @~english
 *  @brief  Set transfer size of U3V device
 *  @param  handle                      [IN]            Device handle
 *  @param  nTransferSize               [IN]            Transfer size，Byte，default：1M，rang：>=0x400，Recommended maximum: [windows] rang <= 0x400000; [Linux] rang <= 0x200000
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Increasing the transmission packet size can reduce the CPU utilization at the time of fetching. However, different PCS and different USB extension CARDS have different compatibility, and if this parameter is set too large, there may be the risk of not getting the image.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_SetTransferSize(
  IN void * handle, IN unsigned int nTransferSize);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取U3V的传输包大小
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnTransferSize              [IN][OUT]           传输的包大小指针, Byte
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 该接口用于获取当前的U3V传输包大小，默认1M。
 
 *  @~english
 *  @brief  Get transfer size of U3V device
 *  @param  handle                      [IN]            Device handle
 *  @param  pnTransferSize              [IN][OUT]           Transfer size，Byte
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This interface is used to get the current U3V transfer packet size, default 1M.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_GetTransferSize(
  IN void * handle, IN OUT unsigned int * pnTransferSize);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置U3V的传输通道个数
 *  @param  handle                      [IN]            设备句柄
 *  @param  nTransferWays               [IN]            传输通道个数，范围：1-10
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 用户可以根据PC的性能、设备出图帧率、图像大小和内存使用率等因素对该参数进行调节。但不同的PC和不同的USB扩展卡存在不同的兼容性。
 
 *  @~english
 *  @brief  Set transfer ways of U3V device
 *  @param  handle                      [IN]            Device handle
 *  @param  nTransferWays               [IN]            Transfer ways，rang：1-10
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Users can adjust this parameter according to PC performance, camera image frame rate, image size, memory utilization and other factors. But different PCS and different USB expansion CARDS have different compatibility.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_SetTransferWays(
  IN void * handle, IN unsigned int nTransferWays);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取U3V的传输通道个数
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnTransferWays              [IN][OUT]       传输通道个数指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 该接口用于获取当前的U3V异步取流节点个数，2000W设备的MONO8默认为3个，YUV为默认2个，RGB为默认1个，其它情况默认8个节点。
 
 *  @~english
 *  @brief  Get transfer ways of U3V device
 *  @param  handle                      [IN]            Device handle
 *  @param  pnTransferWays              [IN][OUT]       Transfer ways
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This interface is used to get the current number of U3V asynchronous feed nodes. For 2000W camera, MONO8 defaults to 3, YUV defaults to 2, RGB defaults to 1, and other cases default to 8 nodes.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_GetTransferWays(
  IN void * handle, IN OUT unsigned int * pnTransferWays);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  注册流异常消息回调，在打开设备之后调用（只支持U3V相机，不支持GenTL设备）
 *  @param  handle                      [IN]            设备句柄
 *  @param  cbException                 [IN]            异常回调函数指针
 *  @param  pUser                       [IN]            用户自定义变量
 *  @return 成功,返回MV_OK,失败,返回错误码
 
 *  @~english
 *  @brief  Register exception stream callBack, call after open device (only support U3V Camera, don't support GenTL Device)
 *  @param  handle                      [IN]            Device handle
 *  @param  cbException                 [IN]            Exception callback function pointer
 *  @param  pUser                       [IN]            User defined variable
 *  @return Success, return MV_OK. Failure, return error code 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_RegisterStreamExceptionCallBack(
  IN void * handle,
  IN void(__stdcall * cbException)(MV_CC_STREAM_EXCEPTION_TYPE enExceptionType, void * pUser),
  IN void * pUser);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置U3V的事件缓存节点个数
 *  @param  handle                      [IN]            设备句柄
 *  @param  nEventNodeNum               [IN]            事件缓存节点个数，范围：1-64
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 该接口用于设置当前的U3V事件缓存节点个数，默认情况下为5个。
 
 *  @~english
 *  @brief  Set the number of U3V device event cache nodes
 *  @param  handle                      [IN]            Device handle
 *  @param  nEventNodeNum               [IN]            Event Node Number
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This interface is used to set the current number of U3V event nodes. default to 5 nodes.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_SetEventNodeNum(
  IN void * handle, IN unsigned int nEventNodeNum);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置U3V的同步读写超时时间，范围为1000 ~ UINT_MAX(包含1000，不包含UINT_MAX)
 *  @param  handle                      [IN]            设备句柄
 *  @param  nMills                      [IN]            设置同步读写超时时间,默认时间为1000ms
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 增加设置同步读取时间接口，兼容部分相机配置参数很慢，超过1000ms的情况

 *  @~english
 *  @brief  Set U3V Synchronisation timeout,range is 1000 ~ UINT_MAX(minimum value greater than 1000,maximum value less than UINT_MAX)
 *  @param  handle               [IN]            Device handle
 *  @param  nMills               [IN]            set synchronisation timeout(ms),default 1000ms
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Increasing the SetSyncTimeOut can compatible with some camera configuretion parameters very slow,more than 1000ms 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_SetSyncTimeOut(IN void * handle, IN unsigned int nMills);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取U3V相机同步读写超时时间
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnMills                     [IN][OUT]       获取的超时时间(ms)
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks 该接口用于获取当前的U3V同步读写超时时间大小，默认1000ms。
 
 *  @~english
 *  @brief  Get U3V Camera Synchronisation timeout
 *  @param  handle                      [IN]            Device handle
 *  @param  pnMills                     [IN][OUT]       Get Synchronisation time(ms)
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This interface is used to get the current U3V timeout, default 1000ms.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_GetSyncTimeOut(IN void * handle, IN OUT unsigned int * pnMills);

/*******************Part8 ch: GenTL相关接口 | en: GenTL related interface*******************/

/******************************************************************************/ /**
 *  @~chinese
 *  @brief  通过GenTL枚举Interfaces
 *  @param  pstIFList                   [IN][OUT]       Interfaces列表
 *  @param  strGenTLPath                [IN]            GenTL的cti文件路径
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks Interfaces列表的内存是在SDK内部分配的，多线程调用该接口时会进行设备列表内存的释放和申请
             建议尽量避免多线程枚举操作。
 
 *  @~english
 *  @brief  Enumerate Interfaces with GenTL
 *  @param  pstIFList                   [IN][OUT]       Interfaces List
 *  @param  strGenTLPath                [IN]            GenTL cti file path
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks The memory of the Interfaces list is allocated within the SDK. When the interface is invoked by multiple threads, the memory of the device list will be released and applied.\n
             It is recommended to avoid multithreaded enumeration operations as much as possible.
 *******************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumInterfacesByGenTL(
  IN OUT MV_GENTL_IF_INFO_LIST * pstIFList, IN const char * strGenTLPath);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  卸载cti库
 *  @param  pGenTLPath                [IN]            枚举卡时加载的cti文件路径
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 卸载前需要保证通过该cti枚举出的相机已全部关闭，否则报错前置条件错误。
 
 *  @~english
 *  @brief  Unload cti library
 *  @param  pGenTLPath                [IN]            GenTL cti file path
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Make sure that all devices enumerated by this cti are already closed.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_UnloadGenTLLibrary(IN const char * pGenTLPath);

/*****************************************************************************************************/ /**
 *  @~chinese
 *  @brief  通过GenTL Interface枚举设备
 *  @param  pstIFInfo                   [IN]            Interface信息
 *  @param  pstDevList                  [IN][OUT]           设备列表
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 设备列表的内存是在SDK内部分配的，多线程调用该接口时会进行设备列表内存的释放和申请
             建议尽量避免多线程枚举操作。
 
 *  @~english
 *  @brief  Enumerate Devices with GenTL interface
 *  @param  pstIFInfo                   [IN]            Interface information
 *  @param  pstDevList                  [IN][OUT]           Device List
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks The memory of the list is allocated within the SDK. When the interface is invoked by multiple threads, the memory of the device list will be released and applied.\n
             It is recommended to avoid multithreaded enumeration operations as much as possible.
 *****************************************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumDevicesByGenTL(
  IN MV_GENTL_IF_INFO * pstIFInfo, IN OUT MV_GENTL_DEV_INFO_LIST * pstDevList);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  通过GenTL设备信息创建设备句柄
 *  @param  handle                      [IN][OUT]       设备句柄
 *  @param  pstDevInfo                  [IN]            设备信息结构体指针
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 根据输入的设备信息，创建库内部必须的资源和初始化内部模块。
 
 *  @~english
 *  @brief  Create Device Handle with GenTL Device Info
 *  @param  handle                      [IN][OUT]       Device handle
 *  @param  pstDevInfo                  [IN]            Device Information
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Create required resources within library and initialize internal module according to input device information.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CreateHandleByGenTL(
  IN OUT void ** handle, IN const MV_GENTL_DEV_INFO * pstDevInfo);

/*******************Part9 ch: 图像保存、格式转换等相关接口 | en: Related image save and format convert interface*******************/

/********************************************************************/ /**
 *  @~chinese
 *  @brief  保存图片，支持Bmp和Jpeg.
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstSaveParam                [IN][OUT]       保存图片参数结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 通过该接口可以将从设备采集到的原始图像数据转换成JPEG或者BMP等格式并存放在指定内存中，然后用户可以将转换之后的数据直接保存成图片文件。
             该接口调用无接口顺序要求，有图像源数据就可以进行转换，可以先调用MV_CC_GetOneFrameTimeout或者MV_CC_RegisterImageCallBackEx设置回调函数，获取一帧图像数据，然后再通过该接口转换格式。
             该接口支持图像 宽、高、总长最大至 UINT_MAX, 其中MV_CC_SaveImageEx2支持 宽、高、总长最大至 USHRT_MAX
			 JPEG格式最大支持宽高为65500
 
 *  @~english
 *  @brief  Save image, support Bmp and Jpeg.
 *  @param  handle                      [IN]            Device handle
 *  @param  pstSaveParam                [IN][OUT]       Save image parameters structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Once there is image data, you can call this API to convert the data.
             You can also call MV_CC_GetOneFrameTimeout or MV_CC_RegisterImageCallBackEx or MV_CC_GetImageBuffer to get one image frame and set the callback function, and then call this API to convert the format.
             Comparing with the API MV_CC_SaveImageEx2, this API support the parameter nWidth/nHeight/nDataLen to UINT_MAX. 
			 JPEG format supports a maximum width and height of 65500
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImageEx3(
  IN void * handle, IN OUT MV_SAVE_IMAGE_PARAM_EX3 * pstSaveParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  保存图像到文件
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstSaveFileParam            [IN][OUT]       保存图片文件参数结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口支持BMP/JPEG/PNG/TIFF。
             该接口支持图像 宽、高、总长最大至 UINT_MAX, 其中MV_CC_SaveImageToFile支持 宽、高、总长最大至 USHRT_MAX
             该接口是MV_CC_SaveImageToFile接口的扩展接口
			 JPEG格式最大支持宽高为65500
 *  @~english
 *  @brief  Save the image file.
 *  @param  handle                      [IN]            Device handle
 *  @param  pstSaveFileParam            [IN][OUT]       Save the image file parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API support BMP/JPEG/PNG/TIFF.
             Comparing with the API MV_CC_SaveImageToFile, this API support the parameter nWidth/nHeight/nDataLen to UINT_MAX. 
			 JPEG format supports a maximum width and height of 65500
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImageToFileEx(
  IN void * handle, IN OUT MV_SAVE_IMAGE_TO_FILE_PARAM_EX * pstSaveFileParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  保存3D点云数据，支持PLY、CSV和OBJ三种格式
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstPointDataParam           [IN][OUT]       保存点云数据参数结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 3D数据格式保存成3D文件格式，支持PLY/CSV/OBJ，
             目前支持PixelType_Gvsp_Coord3D_ABC32、PixelType_Gvsp_Coord3D_ABC32f、PixelType_Gvsp_Coord3D_AB32、PixelType_Gvsp_Coord3D_AB32f、PixelType_Gvsp_Coord3D_AC32、PixelType_Gvsp_Coord3D_AC32f,
             暂不支持其他3D格式。
 
 *  @~english
 *  @brief  Save 3D point data, support PLY、CSV and OBJ
 *  @param  handle                      [IN]            Device handle
 *  @param  pstPointDataParam           [IN][OUT]       Save 3D point data parameters structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Save the 3D data format to 3D file format，support PLY、CSV and OBJ,
             only support PixelType_Gvsp_Coord3D_ABC32、PixelType_Gvsp_Coord3D_ABC32f、PixelType_Gvsp_Coord3D_AB32、PixelType_Gvsp_Coord3D_AB32f、PixelType_Gvsp_Coord3D_AC32、PixelType_Gvsp_Coord3D_AC32f
             Other 3D format is not supported now.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SavePointCloudData(
  IN void * handle, IN OUT MV_SAVE_POINT_CLOUD_PARAM * pstPointDataParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  图像旋转
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstRotateParam              [IN][OUT]       图像旋转参数结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口只支持MONO8/RGB24/BGR24格式数据的90/180/270度旋转。
 
 *  @~english
 *  @brief  Rotate Image
 *  @param  handle                      [IN]            Device handle
 *  @param  pstRotateParam              [IN][OUT]       Rotate image parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API only support 90/180/270 rotation of data in the MONO8/RGB24/BGR24 format.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RotateImage(
  IN void * handle, IN OUT MV_CC_ROTATE_IMAGE_PARAM * pstRotateParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  图像翻转
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstFlipParam                [IN][OUT]       图像翻转参数结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口只支持MONO8/RGB24/BGR24格式数据的垂直和水平翻转。
 
 *  @~english
 *  @brief  Flip Image
 *  @param  handle                      [IN]            Device handle
 *  @param  pstFlipParam                [IN][OUT]       Flip image parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API only support vertical and horizontal reverse of data in the MONO8/RGB24/BGR24 format.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FlipImage(
  IN void * handle, IN OUT MV_CC_FLIP_IMAGE_PARAM * pstFlipParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  像素格式转换
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstCvtParam                 [IN][OUT]       像素格式转换参数结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 通过将接口可以将从设备采集到的原始图像数据转换成用户所需的像素格式并存放在指定内存中。
             该接口调用无接口顺序要求，有图像源数据就可以进行转换，可以先调用MV_CC_GetOneFrameTimeout或者MV_CC_RegisterImageCallBackEx设置回调函数，
             获取一帧图像数据，然后再通过该接口转换格式。如果设备当前采集图像是JPEG压缩的格式，则不支持调用该接口进行转换。
             该接口支持图像 宽、高、总长最大至 UINT_MAX, 其中MV_CC_ConvertPixelType支持 宽、高、总长最大至 USHRT_MAX
 
 *  @~english
 *  @brief  Pixel format conversion
 *  @param  handle                      [IN]            Device handle
 *  @param  pstCvtParam                 [IN][OUT]       Convert Pixel Type parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API is used to transform the collected original data to pixel format and save to specified memory. 
             There is no order requirement to call this API, the transformation will execute when there is image data. 
             First call MV_CC_GetOneFrameTimeout or MV_CC_RegisterImageCallBackEx to set callback function, and get a frame of image data,
             then call this API to transform the format.
             Comparing with the API MV_CC_ConvertPixelType, this API support the parameter nWidth/nHeight/nSrcDataLen to UINT_MAX. 

 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ConvertPixelTypeEx(
  IN void * handle, IN OUT MV_CC_PIXEL_CONVERT_PARAM_EX * pstCvtParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置插值算法类型
 *  @param  handle                      [IN]            设备句柄
 *  @param  nBayerCvtQuality            [IN]            Bayer的插值方法  0-快速 1-均衡 2-最优（默认为最优）
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 设置内部图像转换接口的Bayer插值算法类型参数，MV_CC_ConvertPixelType、MV_CC_GetImageForRGB/BGR接口内部使用的插值算法是该接口所设定的。

 *  @~english
 *  @brief  Interpolation algorithm type setting
 *  @param  handle                      [IN]            Device handle
 *  @param  nBayerCvtQuality            [IN]            Bayer interpolation method  0-Fast 1-Equilibrium 2-Optimal
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Set the bell interpolation quality parameters of the internal image conversion interface, 
             and the interpolation algorithm used in the MV_CC_ConvertPixelType and MV_CC_GetImageForRGB/BGR interfaces is set by this interface.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerCvtQuality(
  IN void * handle, IN unsigned int nBayerCvtQuality);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  插值算法平滑使能设置
 *  @param  handle                      [IN]            设备句柄
 *  @param  bFilterEnable               [IN]            平滑使能(默认关闭)
 *  @return 成功，返回#MV_OK；错误，返回错误码 
 *  @remarks 设置内部图像转换接口的贝尔插值平滑使能参数，MV_CC_ConvertPixelType、MV_CC_SaveImageEx3接口内部使用的插值算法是该接口所设定的。

 *  @~english
 *  @brief  Filter type of the bell interpolation quality algorithm setting
 *  @param  handle                      [IN]            Device handle
 *  @param  bFilterEnable               [IN]            Filter type enable
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Set the bell interpolation filter type parameters of the internal image conversion interface, 
             and the interpolation algorithm used in the MV CC ConvertPixelType and MV_CC_SaveImageEx3 interfaces is set by this interface.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerFilterEnable(IN void * handle, IN bool bFilterEnable);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置Bayer格式的Gamma值
 *  @param  handle                      [IN]            设备句柄
 *  @param  fBayerGammaValue            [IN]            Gamma值:0.1 ~ 4.0
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 设置该值后，在调用MV_CC_ConvertPixelType、MV_CC_SaveImageEx3、MV_CC_SaveImageToFile接口将Bayer8/10/12/16格式转成RGB24/48， RGBA32/64，BGR24/48，BGRA32/64时起效。

 *  @~english
 *  @brief  Set Gamma value
 *  @param  handle                      [IN]            Device handle
 *  @param  fBayerGammaValue            [IN]            Gamma value[0.1,4.0]
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After setting the value, it works when calling MV_CC_ConvertPixelType\MV_CC_SaveImageEx3\MV_CC_SaveImageToFile API convert Bayer8/10/12/16 to RGB24/48， RGBA32/64，BGR24/48，BGRA32/64.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerGammaValue(IN void * handle, IN float fBayerGammaValue);

/********************************************************************/ /**
 *  @~chinese
 *  @brief   设置Mono8/Bayer8/10/12/16格式的Gamma值
 *  @param   handle                           [IN] 设备句柄
 *  @param   MvGvspPixelType enSrcPixelType   [IN] 像素格式,支持PixelType_Gvsp_Mono8,Bayer8/10/12/16
 *  @param   fGammaValue                      [IN] Gamma值:0.1 ~ 4.0
 *  @return  成功，返回MV_OK；错误，返回错误码 
 *  @remarks 设置Mono8的gamma值后，在调用MV_CC_ConvertPixelType接口将Mono8转成Mono8时gamma值起效。
 *  @remarks 设置Bayer8/10/12/16的gamma值后，在调用MV_CC_ConvertPixelType、MV_CC_SaveImageToFile、MV_CC_SaveImageEx3接口将Bayer8/10/12/16转RGB24/48， RGBA32/64，BGR24/48，BGRA32/64时gamma值起效。
 *  @remarks 该接口兼容MV_CC_SetBayerGammaValue接口，新增支持Mono8像素格式
 
 *  @~english
 *  @brief  Set Gamma value
 *  @param  handle                           [IN]            Device handle
 *  @param  MvGvspPixelType enSrcPixelType   [IN]            PixelType,support PixelType_Gvsp_Mono8,Bayer8/10/12/16
 *  @param  fGammaValue                      [IN]            Gamma value:0.1~ 4.0
 *  @remarks After setting the gamma of Mono8 ，the gamma value takes effect when calling MV_CC_ConvertPixelType converts Mono8 to Mono8.
 *  @remarks After setting the gamma of Bayer8/10/12/16, the gamma value takes effect when calling MV_CC_ConvertPixelType\MV_CC_SaveImageToFile\MV_CC_SaveImageEx3 converts Bayer8/10/12/16 to RGB24/48,RGBA32/64,BGR24/48,BGRA32/64.
 *  @remarks This API compatible with MV_CC_SetBayerGammaValue, adds Mono8 PixelType.
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGammaValue(
  IN void * handle, IN enum MvGvspPixelType enSrcPixelType, IN float fGammaValue);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置Bayer格式的Gamma信息
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstGammaParam               [IN]            Gamma信息   
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 设置该信息后，在调用MV_CC_ConvertPixelType、MV_CC_SaveImageEx3、MV_CC_SaveImageToFile接口将Bayer8/10/12/16格式转成RGB24/48， RGBA32/64，BGR24/48，BGRA32/64时Gamma值起效。

 *  @~english
 *  @brief  Set Gamma param
 *  @param  handle                      [IN]            Device handle
 *  @param  pstGammaParam               [IN]            Gamma param
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After setting the param, it work in the calling MV_CC_ConvertPixelType\MV_CC_SaveImageEx3\MV_CC_SaveImageToFile API convert Bayer8/10/12/16 to RGB24/48， RGBA32/64，BGR24/48，BGRA32/64.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerGammaParam(
  IN void * handle, IN MV_CC_GAMMA_PARAM * pstGammaParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置Bayer格式的CCM使能和矩阵，量化系数默认1024
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstCCMParam                 [IN]            CCM参数
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 开启CCM并设置CCM矩阵后，在调用MV_CC_ConvertPixelType、MV_CC_SaveImageEx3接口将Bayer8/10/12/16格式转成RGB24/48， RGBA32/64，BGR24/48，BGRA32/64时起效。 

 *  @~english
 *  @brief  Set CCM param,Scale default 1024
 *  @param  handle                      [IN]            Device handle
 *  @param  pstCCMParam                 [IN]            CCM parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After enable the color correction and set the color correction matrix, It work in the calling MV_CC_ConvertPixelType\MV_CC_SaveImageEx3 API convert Bayer8/10/12/16 to RGB24/48， RGBA32/64，BGR24/48，BGRA32/64.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerCCMParam(
  IN void * handle, IN MV_CC_CCM_PARAM * pstCCMParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置Bayer格式的CCM使能和矩阵
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstCCMParam                 [IN]            CCM参数
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 开启CCM并设置CCM矩阵后，在调用MV_CC_ConvertPixelType、MV_CC_SaveImageEx3接口将Bayer8/10/12/16格式转成RGB24/48， RGBA32/64，BGR24/48，BGRA32/64时起效。

 *  @~english
 *  @brief  Set CCM param
 *  @param  handle                      [IN]            Device handle
 *  @param  pstCCMParam                 [IN]            CCM parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After enable the color correction and set the color correction matrix, It work in the calling MV_CC_ConvertPixelType\MV_CC_SaveImageEx3 API convert Bayer8/10/12/16 to RGB24/48， RGBA32/64，BGR24/48，BGRA32/64.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerCCMParamEx(
  IN void * handle, IN MV_CC_CCM_PARAM_EX * pstCCMParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  图像对比度调节
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstContrastParam            [IN][OUT]       对比度调节参数
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 

 *  @~english
 *  @brief  Adjust image contrast
 *  @param  handle                      [IN]            Device handle
 *  @param  pstContrastParam            [IN][OUT]       Contrast parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ImageContrast(
  IN void * handle, IN OUT MV_CC_CONTRAST_PARAM * pstContrastParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  无损解码
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstDecodeParam              [IN][OUT]       无损解码参数结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 将从相机中取到的无损压缩码流解码成裸数据，同时支持解析当前相机实时图像的水印信息（如果输入的无损码流不是当前相机或者不是实时取流的，则水印解析可能异常）;
			 若解码失败，请检查以下情况：（1）需要CPU支持 SSE AVX指令集（2）若当前帧异常（丢包等）,可能导致解码异常（3）相机出图异常， 即使不丢包也会异常

 *  @~english
 *  @brief  High Bandwidth Decode
 *  @param  handle                      [IN]            Device handle
 *  @param  pstDecodeParam              [IN][OUT]       High Bandwidth Decode parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Decode the lossless compressed data from the camera into raw data，At the same time, it supports parsing the watermark information of the real-time image of the current camera (if the input lossless code stream is not the current camera or is not real-time streaming, the watermark parsing may be abnormal);
			 If decoding fails, please check the following: (1) The CPU is required to support the SSE AVX instruction set. (2) If the current frame is abnormal (packet loss, etc.), it may cause decoding exceptions. (3) The camera plot is abnormal, even if there is no packet loss, it may cause exceptions
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_HB_Decode(
  IN void * handle, IN OUT MV_CC_HB_DECODE_PARAM * pstDecodeParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  在图像上绘制矩形框辅助线
 *  @param  handle                      [IN]            设备句柄
 *  @param  pRectInfo                   [IN]            矩形辅助线的信息
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks  该接口仅支持windows平台
 
 *  @~english
 *  @brief  Draw Rect Auxiliary Line
 *  @param  handle                      [IN]            Device handle
 *  @param  pRectInfo                   [IN]            Rect Auxiliary Line Info
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface only supports windows platform.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DrawRect(IN void * handle, IN MVCC_RECT_INFO * pRectInfo);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  在图像上绘制圆形辅助线
 *  @param  handle                      [IN]            设备句柄
 *  @param  pCircleInfo                 [IN]            圆形辅助线的信息
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks  该接口仅支持windows平台
 
 *  @~english
 *  @brief  Draw Circle Auxiliary Line
 *  @param  handle                      [IN]            Device Handle
 *  @param  pCircleInfo                 [IN]            Circle Auxiliary Line Info
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface only supports windows platform.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DrawCircle(IN void * handle, IN MVCC_CIRCLE_INFO * pCircleInfo);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  在图像上绘制线条
 *  @param  handle                      [IN]            设备句柄
 *  @param  pLinesInfo                  [IN]            线条辅助线信息
 *  @return 成功，返回MV_OK；错误，返回错误码
 *  @remarks  该接口仅支持windows平台
 
 *  @~english
 *  @brief  Draw Line Auxiliary Line
 *  @param  handle                      [IN]            Device Handle
 *  @param  pLinesInfo                  [IN]            Linear Auxiliary Line Info
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks  This interface only supports windows platform.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DrawLines(IN void * handle, IN MVCC_LINES_INFO * pLinesInfo);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  开始录像
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstRecordParam              [IN]            录像参数结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
			该接口最大支持Width*Height为8000*8000大小，否则会导致调用MV_CC_InputOneFrame接口错误。
 
 *  @~english
 *  @brief  Start Record
 *  @param  handle                      [IN]            Device handle
 *  @param  pstRecordParam              [IN]            Record param structure
 *  @return Success, return MV_OK. Failure, return error code
	The maximum supported width * height of this interface is 8000 * 8000, otherwise it will result in calling MV_ CC_ InputOneFrame interface error.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_StartRecord(
  IN void * handle, IN MV_CC_RECORD_PARAM * pstRecordParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  输入录像数据
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstInputFrameInfo           [IN]            录像数据结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 
 *  @~english
 *  @brief  Input RAW data to Record
 *  @param  handle                      [IN]            Device handle
 *  @param  pstInputFrameInfo           [IN]            Record data structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_InputOneFrame(
  IN void * handle, IN MV_CC_INPUT_FRAME_INFO * pstInputFrameInfo);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  停止录像
 *  @param  handle                      [IN]            设备句柄
 *  @return 成功，返回MV_OK；错误，返回错误码 
 
 *  @~english
 *  @brief  Stop Record
 *  @param  handle                      [IN]            Device handle
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_StopRecord(IN void * handle);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  打开获取或设置相机参数的GUI界面
 *  @param  handle                      [IN]            设备句柄
 *  @return 成功，返回MV_OK，失败，返回错误码。
 *  @remarks 通过MV_CC_OpenDevice连接设备后，可以通过该接口获取或设置设备参数。
 *  @remarks 限制：在同一线程中多相机同时调用该接口，只能打开当前一个GUI界面，需要关闭当前相机GUI界面后，才可打开另一个相机的GUI界面（后续版本优化）
			 该接口仅支持windows平台

 *  @~english
 *  @brief  Open the GUI interface for getting or setting camera parameters
 *  @param  handle                      [IN]            Device handle
 *  @return Success, return MV_OK, Failure, return error code.
 *  @remarks After connecting to device through MV_CC_OpenDevice, use this interface to get or set device params.
 *  @remarks Limit: calling this interface multiple times in the same thread can only open one GUI interface. 
             You need to wait until the previous GUI interface is closed before opening the next GUI interface.(Subsequent version optimization)
			 This interface only supports windows platform.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_OpenParamsGUI(IN void * handle);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  重构图像(用于分时曝光功能)
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstReconstructParam         [IN][OUT]       重构图像参数
 *  @return 成功，返回MV_OK，失败，返回错误码。
 *  @remarks 图像分割支持任意像素格式，图像分割应与线阵相机的“MultiLightControl”节点搭配使用，该节点可设置多个不同的曝光值，如MultiLightControl=2,
             相机会将两个不同曝光值所对应的两张图像交叠合并为一张图像(实际高度为两张图像的高度)发送给上层应用程序，
             调用该接口并传入分时曝光值nExposureNum为2，可将相机发送的一张图像分割为2张图像，这两张图像分别对应一个曝光值。
             若使用普通相机或未打开线阵相机的“MultiLightControl”节点，则图像分割无意义，只是将图像按行分割为2，3，4张图像，
             每张图像的高度变为原图像的1/2，1/3，1/4(由nExposureNum决定)。

 *  @~english
 *  @brief  Reconstruct Image(For time-division exposure function)
 *  @param  handle                      [IN]            Device handle
 *  @param  pstReconstructParam         [IN][OUT]       Reconstruct image parameters
 *  @return Success, return MV_OK, Failure, return error code.
 *  @remarks Image segmentation supports any pixel format. Image segmentation should be used with the "MultiLightControl" node of the linear array camera. This node can set multiple different exposure values, such as MultiLightControl=2, 
             The camera will overlap and merge two images corresponding to two different exposure values into one image (the actual height is the height of the two images) and send it to the upper application. 
             Call the interface and pass in nExposureNum is two. One image sent by the camera can be divided into two images, each of which corresponds to an exposure value. 
             If an ordinary camera is used or the "MultiLightControl" node of the linear array camera is not turned on, the image segmentation is meaningless, but the image is divided into 2, 3, and 4 images by line. 
             The height of each image becomes 1/2, 1/3, 1/4 of the original image (determined by nExposureNum).
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ReconstructImage(
  IN void * handle, IN OUT MV_RECONSTRUCT_IMAGE_PARAM * pstReconstructParam);

#ifdef __cplusplus
}
#endif

#endif  //_MV_CAMERA_CTRL_H_
