
#ifndef _MV_OBSOLETE_INTERFACES_H_
#define _MV_OBSOLETE_INTERFACES_H_

#include "CameraParams.h"
#include "MvErrorDefine.h"
#include "ObsoleteCamParams.h"

/**
*  @brief  动态库导入导出定义
*  @brief  Import and export definition of the dynamic library 
*/
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

#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************************/
/* 不建议使用的接口                     	                    		*/
/* Interfaces not recommended                                           */
/************************************************************************/
/************************************************************************
 *  @fn     MV_CC_GetImageInfo
 *  @brief  获取图像基本信息
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstInfo                     [IN][OUT]       返回给调用者有关相机图像基本信息结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 
 *  @fn     MV_CC_GetImageInfo
 *  @brief  Get basic information of image
 *  @param  handle                      [IN]            Device handle
 *  @param  pstInfo                     [IN][OUT]       Structure pointer of image basic information
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetImageInfo(
  IN void * handle, IN OUT MV_IMAGE_BASIC_INFO * pstInfo);

/************************************************************************
 *  @fn     MV_CC_GetTlProxy
 *  @brief  获取GenICam代理
 *  @param  handle                 [IN]           句柄地址
 *  @return GenICam代理类指针 ，正常返回值非NULL；异常返回NULL
 
 *  @fn     MV_CC_GetTlProxy
 *  @brief  Get GenICam proxy
 *  @param  handle                 [IN]           Handle address
 *  @return GenICam proxy pointer, normal, return non-NULL; exception, return NULL
 ************************************************************************/
MV_CAMCTRL_API void * __stdcall MV_CC_GetTlProxy(IN void * handle);

/***********************************************************************
 *  @fn         MV_XML_GetRootNode
 *  @brief      获取根节点
 *  @param       handle                 [IN]          句柄
 *  @param       pstNode                [OUT]         根节点信息结构体
 *  @return 成功，返回MV_OK；错误，返回错误码
 
 *  @fn         MV_XML_GetRootNode
 *  @brief      Get root node
 *  @param       handle                 [IN]          Handle
 *  @param       pstNode                [OUT]         Root node information structure
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetRootNode(
  IN void * handle, IN OUT MV_XML_NODE_FEATURE * pstNode);

/***********************************************************************
 *  @fn         MV_XML_GetChildren
 *  @brief      从xml中获取指定节点的所有子节点，根节点为Root
 *  @param       handle                 [IN]          句柄
 *  @param       pstNode                [IN]          根节点信息结构体
 *  @param       pstNodesList           [OUT]         节点列表结构体
 *  @return 成功，返回MV_OK；错误，返回错误码
 
 *  @fn         MV_XML_GetChildren
 *  @brief      Get all children node of specific node from xml, root node is Root
 *  @param       handle                 [IN]          Handle
 *  @param       pstNode                [IN]          Root node information structure
 *  @param       pstNodesList           [OUT]         Node information structure
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetChildren(
  IN void * handle, IN MV_XML_NODE_FEATURE * pstNode, IN OUT MV_XML_NODES_LIST * pstNodesList);

/***********************************************************************
 *  @fn         MV_XML_GetNodeFeature
 *  @brief      获得当前节点的属性
 *  @param       handle                 [IN]          句柄
 *  @param       pstNode                [IN]          根节点信息结构体
 *  @param       pstFeature             [OUT]         当前节点属性结构体，
                           pstFeature 具体结构体内容参考 MV_XML_FEATURE_x
 *  @return 成功，返回MV_OK；错误，返回错误码
 
 *  @fn         MV_XML_GetNodeFeature
 *  @brief      Get current node feature
 *  @param       handle                 [IN]          Handle
 *  @param       pstNode                [IN]          Root node information structure
 *  @param       pstFeature             [OUT]         Current node feature structure
                           Details of pstFeature refer to MV_XML_FEATURE_x
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetNodeFeature(
  IN void * handle, IN MV_XML_NODE_FEATURE * pstNode, IN OUT void * pstFeature);

/***********************************************************************
 *  @fn         MV_XML_UpdateNodeFeature
 *  @brief      更新节点
 *  @param       handle                 [IN]          句柄
 *  @param       enType                 [IN]          节点类型
 *  @param       pstFeature             [OUT]         当前节点属性结构体
 *  @return 成功，返回MV_OK；错误，返回错误码
 
 *  @fn         MV_XML_UpdateNodeFeature
 *  @brief      Update node
 *  @param       handle                 [IN]          Handle
 *  @param       enType                 [IN]          Node type
 *  @param       pstFeature             [OUT]         Current node feature structure
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_UpdateNodeFeature(
  IN void * handle, IN enum MV_XML_InterfaceType enType, IN void * pstFeature);

// 有节点需要更新时的回调函数
// 当调用MV_XML_UpdateNodeFeature接口更新节点属性时，注册的回调函数cbUpdate会在pstNodesList中返回与之相关联的节点
/***********************************************************************
 *  @fn         MV_XML_RegisterUpdateCallBack
 *  @brief      注册更新回调
 *  @param       handle                 [IN]          句柄
 *  @param       cbUpdate               [IN]          回调函数指针
 *  @param       pUser                  [IN]          用户自定义变量
 *  @return 成功，返回MV_OK；错误，返回错误码
 
 *  @fn         MV_XML_RegisterUpdateCallBack
 *  @brief      Register update callback
 *  @param       handle                 [IN]          Handle
 *  @param       cbUpdate               [IN]          Callback function pointer
 *  @param       pUser                  [IN]          User defined variable
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_RegisterUpdateCallBack(
  IN void * handle,
  IN void(__stdcall * cbUpdate)(
    enum MV_XML_InterfaceType enType, void * pstFeature, MV_XML_NODES_LIST * pstNodesList,
    void * pUser),
  IN void * pUser);

/************************************************************************/
/* 弃用的接口（存在更优化的接口可替换）                     			*/
/* Abandoned interface                                                  */
/************************************************************************/
/***********************************************************************
 *  @fn         MV_CC_GetOneFrame
 *  @brief      获取一帧图像，此函数为查询式获取，每次调用查询内部缓存有
                无数据，有数据则范围数据，无数据返回错误码
                （该接口已弃用，建议改用 MV_CC_GetOneFrameTimeOut接口）
 *  @param       handle                 [IN]          句柄
 *  @param       pData                  [OUT]         图像数据接收指针
 *  @param       nDataSize              [IN]          接收缓存大小
 *  @param       pFrameInfo             [OUT]         图像信息结构体
 *  @return 成功，返回MV_OK；错误，返回错误码
 
 *  @fn         MV_CC_GetOneFrame
 *  @brief      Get one frame data, this function is using query to get data, 
                query whether the internal cache has data, return data if there has, return error code if no data
                (This interface is abandoned, it is recommended to use the MV_CC_GetOneFrameTimeOut)
 *  @param       handle                 [IN]          Handle
 *  @param       pData                  [OUT]         Recevied image data pointer
 *  @param       nDataSize              [IN]          Recevied buffer size
 *  @param       pFrameInfo             [OUT]         Image information structure
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetOneFrame(
  IN void * handle, IN OUT unsigned char * pData, IN unsigned int nDataSize,
  IN OUT MV_FRAME_OUT_INFO * pFrameInfo);

/***********************************************************************
 *  @fn         MV_CC_GetOneFrameEx
 *  @brief      获取一帧trunck数据，此函数为查询式获取，每次调用查询内部
                缓存有无数据，有数据则范围数据，无数据返回错误码
                （该接口已弃用，建议改用 MV_CC_GetOneFrameTimeOut接口）
 *  @param       handle                 [IN]          句柄
 *  @param       pData                  [OUT]         图像数据接收指针
 *  @param       nDataSize              [IN]          接收缓存大小
 *  @param       pFrameInfo             [OUT]         图像信息结构体
 *  @return 成功，返回MV_OK；错误，返回错误码
 
 *  @fn         MV_CC_GetOneFrameEx
 *  @brief      Get one frame of trunck data, this function is using query to get data, 
                query whether the internal cache has data, return data if there has, return error code if no data
                (This interface is abandoned, it is recommended to use the MV_CC_GetOneFrameTimeOut)
 *  @param       handle                 [IN]          Handle
 *  @param       pData                  [OUT]         Recevied image data pointer
 *  @param       nDataSize              [IN]          Recevied buffer size
 *  @param       pFrameInfo             [OUT]         Image information structure
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetOneFrameEx(
  IN void * handle, IN OUT unsigned char * pData, IN unsigned int nDataSize,
  IN OUT MV_FRAME_OUT_INFO_EX * pFrameInfo);

/***********************************************************************
 *  @fn         MV_CC_RegisterImageCallBack
 *  @brief      注册图像数据回调（该接口已弃用，建议改用 MV_CC_RegisterImageCallBackEx接口）
 *  @param       handle                 [IN]          句柄
 *  @param       cbOutput               [IN]          回调函数指针
 *  @param       pUser                  [IN]          用户自定义变量
 *  @return 成功，返回MV_OK；错误，返回错误码
 
 *  @fn         MV_CC_RegisterImageCallBack
 *  @brief      Register image data callback (This interface is abandoned, it is recommended to use the MV_CC_RegisterImageCallBackEx)
 *  @param       handle                 [IN]          Handle
 *  @param       cbOutput               [IN]          Callback function pointer
 *  @param       pUser                  [IN]          User defined variable
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterImageCallBack(
  void * handle,
  void(__stdcall * cbOutput)(unsigned char * pData, MV_FRAME_OUT_INFO * pFrameInfo, void * pUser),
  void * pUser);

/************************************************************************
 *  @fn     MV_CC_SaveImage
 *  @brief  保存图片（该接口已弃用，建议改用 MV_CC_SaveImageEx2接口）
 *  @param  pSaveParam             [IN][OUT]          保存图片参数结构体
                       pData;              // [IN]     输入数据缓存
                       nDataLen;           // [IN]     输入数据大小
                       enPixelType;        // [IN]     输入数据的像素格式
                       nWidth;             // [IN]     图像宽
                       nHeight;            // [IN]     图像高
                       pImageBuffer;       // [OUT]    输出图片缓存
                       nImageLen;          // [OUT]    输出图片大小
                       nBufferSize;        // [IN]     提供的输出缓冲区大小
                       enImageType;        // [IN]     输出图片格式
 *  @return 成功，返回MV_OK；错误，返回错误码 
 
 *  @fn     MV_CC_SaveImage
 *  @brief  Save image (This interface is abandoned, it is recommended to use the MV_CC_SaveImageEx)
 *  @param  pSaveParam             [IN][OUT]          Save image parameters structure
                       pData;              // [IN]     Input data buffer
                       nDataLen;           // [IN]     Input data size
                       enPixelType;        // [IN]     Input data pixel format
                       nWidth;             // [IN]     Width
                       nHeight;            // [IN]     Height
                       pImageBuffer;       // [OUT]    Output image buffer
                       nImageLen;          // [OUT]    Output image size
                       nBufferSize;        // [IN]     Provided output buffer size
                       enImageType;        // [IN]     Output image type
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImage(IN OUT MV_SAVE_IMAGE_PARAM * pSaveParam);

/************************************************************************
 *  @fn     MV_CC_SaveImageEx
 *  @brief  保存图片，支持Bmp和Jpeg.编码质量在50-99之前 （该接口已弃用，建议改用 MV_CC_SaveImageEx2接口）
 *  @param  pSaveParam             [IN][OUT]          保存图片参数结构体
                       pData;              // [IN]     输入数据缓存
                       nDataLen;           // [IN]     输入数据大小
                       enPixelType;        // [IN]     输入数据的像素格式
                       nWidth;             // [IN]     图像宽
                       nHeight;            // [IN]     图像高
                       pImageBuffer;       // [OUT]    输出图片缓存
                       nImageLen;          // [OUT]    输出图片大小
                       nBufferSize;        // [IN]     提供的输出缓冲区大小
                       enImageType;        // [IN]     输出图片格式
                       nJpgQuality;        // [IN]     编码质量, (50-99]
                       nReserved[4];
 *  @return 成功，返回MV_OK；错误，返回错误码 
 
 *  @fn     MV_CC_SaveImageEx
 *  @brief  Save image, support Bmp and Jpeg. Encoding quality, (50-99]
 *  @param  pSaveParam             [IN][OUT]           Save image parameters structure
                       pData;              // [IN]     Input data buffer
                       nDataLen;           // [IN]     Input data size
                       enPixelType;        // [IN]     Pixel format of input data
                       nWidth;             // [IN]     Image width
                       nHeight;            // [IN]     Image height
                       pImageBuffer;       // [OUT]    Output image buffer
                       nImageLen;          // [OUT]    Output image size
                       nBufferSize;        // [IN]     Output buffer size provided
                       enImageType;        // [IN]     Output image format
                       nJpgQuality;        // [IN]     Encoding quality, (50-99]
                       nReserved[4];
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImageEx(IN OUT MV_SAVE_IMAGE_PARAM_EX * pSaveParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  Bayer噪声估计（该接口已弃用，建议改用ISP Tool方式进行标定）
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstNoiseEstimateParam       [IN][OUT]       Bayer噪声估计参数结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口只支持Bayer8/Bayer10/Bayer12格式,其它Bayer格式需先转成Bayer8/Bayer10/Bayer12格式。\n
             该接口只有在打开我司特定彩色相机后才可以正常使用，当相机被断开或者掉线后，继续使用该接口会报错。
 
 *  @~english
 *  @brief  Noise estimate of Bayer format
 *  @param  handle                      [IN]            Device handle
 *  @param  pstNoiseEstimateParam       [IN][OUT]       Noise estimate parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API only support Bayer8/Bayer10/Bayer12 format, other Bayer format must Convert to Bayer8/Bayer10/Bayer12 format.\n
             This API is only available when the camera is turned on, and when the camera is disconnected or disconnected, continuing to use This API will return an error.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_BayerNoiseEstimate(
  IN void * handle, IN OUT MV_CC_BAYER_NOISE_ESTIMATE_PARAM * pstNoiseEstimateParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  Bayer空域降噪（该接口已弃用，建议改用ISP Tool方式进行降噪）
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstSpatialDenoiseParam      [IN][OUT]       Bayer空域降噪参数结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口只支持Bayer8/Bayer10/Bayer12格式,其它Bayer格式需先转成Bayer8/Bayer10/Bayer12格式。\n
             该接口只有在打开我司特定彩色相机后才可以正常使用，当相机被断开或者掉线后，继续使用该接口会报错。
 
 *  @~english
 *  @brief  Spatial Denoise of Bayer format
 *  @param  handle                      [IN]            Device handle
 *  @param  pstSpatialDenoiseParam      [IN][OUT]       Spatial Denoise parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API only support Bayer8/Bayer10/Bayer12 format, other Bayer format must Convert to Bayer8/Bayer10/Bayer12 format.\n
             This API is only available when the camera is turned on, and when the camera is disconnected or disconnected, continuing to use This API will return an error.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_BayerSpatialDenoise(
  IN void * handle, IN OUT MV_CC_BAYER_SPATIAL_DENOISE_PARAM * pstSpatialDenoiseParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置Bayer格式的CLUT使能和信息（该接口已弃用，建议改用ISP Tool方式进行设置）
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstCLUTParam                [IN]            CLUT参数
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 开启CLUT并设置CLUT信息后，在调用MV_CC_ConvertPixelType、MV_CC_SaveImageEx2接口将Bayer8/10/12/16格式转成RGB24/48， RGBA32/64，BGR24/48，BGRA32/64时起效。

 *  @~english
 *  @brief  Set CLUT param
 *  @param  handle                      [IN]            Device handle
 *  @param  pstCLUTParam                [IN]            CLUT parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After enable the CLUT and set CLUT, It work in the calling MV_CC_ConvertPixelType\MV_CC_SaveImageEx2 API convert Bayer8/10/12/16 to RGB24/48， RGBA32/64，BGR24/48，BGRA32/64.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerCLUTParam(
  IN void * handle, IN MV_CC_CLUT_PARAM * pstCLUTParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  图像锐化（该接口已弃用，建议改用ISP Tool方式进行锐化）
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstSharpenParam             [IN]            锐化参数
 *  @return 成功，返回MV_OK；错误，返回错误码 

 *  @~english
 *  @brief  Image sharpen
 *  @param  handle                      [IN]            Device handle
 *  @param  pstSharpenParam             [IN]            Sharpen parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ImageSharpen(
  IN void * handle, IN OUT MV_CC_SHARPEN_PARAM * pstSharpenParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  色彩校正（包括CCM和CLUT）（该接口已弃用，建议改用ISP Tool方式进行校正）
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstColorCorrectParam        [IN]            色彩校正参数
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口支持单独CCM或者CLUT，也支持同时进行CCM和CLUT，用户可以通过CCM和CLUT信息中的使能开关进行选择。

 *  @~english
 *  @brief  Color Correct(include CCM and CLUT)
 *  @param  handle                      [IN]            Device handle
 *  @param  pstColorCorrectParam        [IN]            Color Correct parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API supports CCM or CLUT alone, as well as CCM and CLUT at the same time. The user can select by means of the enable switch in CCM and CLUT information.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ColorCorrect(
  IN void * handle, IN OUT MV_CC_COLOR_CORRECT_PARAM * pstColorCorrectParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  噪声估计（该接口已弃用，建议改用ISP Tool方式进行标定）
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstNoiseEstimateParam       [IN]            噪声估计参数
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 如果用户选择全图做噪声估计，nROINum可输入0，pstROIRect可置空。

 *  @~english
 *  @brief  Noise Estimate
 *  @param  handle                      [IN]            Device handle
 *  @param  pstNoiseEstimateParam       [IN]            Noise Estimate parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks If the user selects the full image, nROINum can be typed with 0 and pstROIRect empty.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_NoiseEstimate(
  IN void * handle, IN OUT MV_CC_NOISE_ESTIMATE_PARAM * pstNoiseEstimateParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  空域降噪（该接口已弃用，建议改用ISP Tool方式进行降噪）
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstSpatialDenoiseParam      [IN]            空域降噪参数
 *  @return 成功，返回MV_OK；错误，返回错误码 

 *  @~english
 *  @brief  Spatial Denoise
 *  @param  handle                      [IN]            Device handle
 *  @param  pstSpatialDenoiseParam      [IN]            Spatial Denoise parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SpatialDenoise(
  IN void * handle, IN OUT MV_CC_SPATIAL_DENOISE_PARAM * pstSpatialDenoiseParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  LSC标定
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstLSCCalibParam            [IN]            标定参数
 *  @return 成功，返回MV_OK；错误，返回错误码 

 *  @~english
 *  @brief  LSC Calib
 *  @param  handle                      [IN]            Device handle
 *  @param  pstLSCCalibParam            [IN]            LSC Calib parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LSCCalib(
  IN void * handle, IN OUT MV_CC_LSC_CALIB_PARAM * pstLSCCalibParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  LSC校正
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstLSCCorrectParam          [IN]            校正参数
 *  @return 成功，返回MV_OK；错误，返回错误码 

 *  @~english
 *  @brief  LSC Correct
 *  @param  handle                      [IN]            Device handle
 *  @param  pstLSCCorrectParam          [IN]            LSC Correct parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LSCCorrect(
  IN void * handle, IN OUT MV_CC_LSC_CORRECT_PARAM * pstLSCCorrectParam);

/************************************************************************
 *  @fn     MV_GIGE_ForceIp
 *  @brief  强制IP（该接口已弃用，建议改用 MV_GIGE_ForceIpEx接口）
 *  @param  handle：设备句柄
 *  @param  nIP               [IN]      设置的IP
 *  @return 见返回错误码
 
 *  @fn     MV_GIGE_ForceIp
 *  @brief  Force IP (This interface is abandoned, it is recommended to use the MV_GIGE_ForceIpEx)
 *  @param  handle Handle
 *  @param  nIP               [IN]      IP to set
 *  @return Refer to error code
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_ForceIp(IN void * handle, unsigned int nIP);

/************************************************************************
 *  @fn     MV_CC_RegisterEventCallBack
 *  @brief  注册事件回调（该接口已弃用，建议改用 MV_CC_RegisterEventCallBackEx接口）
 *  @param  handle：设备句柄
 *  @param  cbEvent           [IN]      事件回调函数指针
 *  @param  pUser             [IN]      用户自定义变量
 *  @return 见返回错误码
 
 *  @fn     MV_CC_RegisterEventCallBack
 *  @brief  Register event callback (this interface has been deprecated and is recommended to be converted to the MV_CC_RegisterEventCallBackEx interface)
 *  @param  handle：设备句柄
 *  @param  cbEvent           [IN]      event callback pointer
 *  @param  pUser             [IN]      User defined value
 *  @return 见返回错误码
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterEventCallBack(
  void * handle, void(__stdcall * cbEvent)(unsigned int nExternalEventId, void * pUser),
  void * pUser);

/***********************************************************************
 *  @fn         MV_CC_Display
 *  @brief      显示图像，注册显示窗口，内部自动显示（与MV_CC_GetImageBuffer不能同时使用，建议改用MV_CC_DisplayOneFrame接口）
 *  @param       handle                 [IN]          句柄
 *  @param       hWnd                   [IN]          显示窗口句柄
 *  @return 成功，返回MV_OK；错误，返回错误码
 
 *  @fn         MV_CC_Display
 *  @brief      Display one frame image, register display window, automatic display internally
 *  @param      handle                 [IN]          Handle
 *  @param      hWnd                   [IN]          Display Window Handle
 *  @return     Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_Display(IN void * handle, void * hWnd);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetIntValue(IN void* handle,
                                                           IN const char* strKey,
                                                           OUT MVCC_INTVALUE *pIntValue);
 *  @brief  获取Integer属性值（建议改用MV_CC_GetIntValueEx接口）
 *  @param  void* handle                [IN]        相机句柄
 *  @param  char* strKey                [IN]        属性键值，如获取宽度信息则为"Width"
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机属性结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetIntValue(IN void* handle,
                                                           IN const char* strKey,
                                                           OUT MVCC_INTVALUE *pIntValue);
 *  @brief  Get Integer value
 *  @param  void* handle                [IN]        Handle
 *  @param  char* strKey                [IN]        Key value, for example, using "Width" to get width
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of camera features
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetIntValue(
  IN void * handle, IN const char * strKey, OUT MVCC_INTVALUE * pIntValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetIntValue(IN void* handle,
                                                           IN const char* strKey,
                                                           IN unsigned int nValue);
 *  @brief  设置Integer型属性值（建议改用MV_CC_SetIntValueEx接口）
 *  @param  void* handle                [IN]        相机句柄
 *  @param  char* strKey                [IN]        属性键值，如获取宽度信息则为"Width"
 *          const unsigned int nValue   [IN]        想要设置的相机的属性值
 *  @return 成功,返回MV_OK,失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetIntValue(IN void* handle,
                                                           IN const char* strKey,
                                                           IN unsigned int nValue);
 *  @brief  Set Integer value
 *  @param  void* handle                [IN]        Handle
 *  @param  char* strKey                [IN]        Key value, for example, using "Width" to set width
 *          const unsigned int nValue   [IN]        Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetIntValue(
  IN void * handle, IN const char * strKey, IN unsigned int nValue);

/************************************************************************/
/* 相机参数获取和设置，此模块的所有接口已废弃，建议使用万能接口代替   */
/* Get and set camara parameters, all interfaces of this module will be replaced by general interface*/
/************************************************************************/
/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetWidth(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取图像宽度
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机宽度的信息结构体指针
 *          返回的pstValue结构体的意义
 *                  unsigned int    nCurValue;      // 代表相机当前的宽度值
 *                  unsigned int    nMax;           // 表示相机允许的最大可设置的宽度值
 *                  unsigned int    nMin;           // 表示相机允许的最小可设置的宽度值
 *                  unsigned int    nInc;           // 表示相机设置的宽度增量必须是nInc的倍数，否则无效
 *  @return 成功,返回MV_OK,并获得相应参数信息的结构体, 失败, 返回错误码
 *  
 *          其他整型结构体参数的接口可参照此接口
 
 * @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetWidth(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get image width
 *  @param  void* handle                [IN]        Camera Handle
 *          MVCC_INTVALUE* pstValue     [IN][OUT]   Returns the information structure pointer about the camera's width for the caller
 *          The meaning of returns pstValue structure
 *                  unsigned int    nCurValue;      // Represents the current width value of the camera
 *                  unsigned int    nMax;           // Indicates the maximum settable width value allowed by the camera
 *                  unsigned int    nMin;           // Indicates the minimum settable width value allowed by the camera
 *                  unsigned int    nInc;           // Indicates that the width increment set by the camera must be a multiple of nInc, otherwise it is invalid
 *  @return Success, return MV_OK, and get the structure of the corresponding parameters. Failure, return error code
 *  
 *          Other Integer structure parameters interface can refer to this interface
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetWidth(IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
*  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetWidth(IN void* handle, IN const unsigned int nValue);
*  @brief  设置图像宽度
*  @param  void* handle                [IN]        相机句柄
*          const unsigned int nValue   [IN]        想要设置的相机宽度的值,注意此宽度值必须是MV_CC_GetWidth接口返回的pstValue中的nInc的倍数才能设置成功
*  @return 成功,返回MV_OK,并且相机宽度将会更改为相应值，失败,返回错误码

* @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetWidth(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set image width
 *  @param  void* handle                [IN]        Camera Handle
 *          const unsigned int nValue   [IN]        To set the value of the camera width, note that the width value must be a multiple of nInc in the pstValue returned by the MV_CC_GetWidth interface
 *  @return Success, return MV_OK, and the camera width will change to the corresponding value. Failure, return error code
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetWidth(IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetHeight(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取图像高度
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机高度的信息结构体指针
 *  @return 成功,返回MV_OK,并将高度信息返回到结构体中，失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetHeight(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get image height
 *  @param  void* handle                [IN]        Camera handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Return pointer of information structure related to camera height to user
 *  @return Success, return MV_OK, and return height information to the structure. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetHeight(IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetHeight(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置图像高度
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的相机宽度的值,注意此宽度值必须是MV_CC_GetWidth接口返回的pstValue中的nInc的倍数才能设置成功
 *  @return 成功,返回MV_OK,并且相机高度将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetHeight(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set image height
 *  @param  void* handle                [IN]        Camera Handle
 *          const unsigned int nValue   [IN]        Camera height value to set, note that this value must be times of nInc of pstValue returned by MV_CC_GetWidth
 *  @return Success, return MV_OK, and the camera height will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetHeight(IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAOIoffsetX(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取图像X偏移
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机X偏移的信息结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAOIoffsetX(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get image X offset
 *  @param  void* handle                [IN]        Camera Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Return pointer of information structure related to camera X offset to user
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAOIoffsetX(IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAOIoffsetX(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置图像AOI偏移
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的相机AOI的值
 *  @return 成功,返回MV_OK,并且相机AOI偏移将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAOIoffsetX(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set image X offset
 *  @param  void* handle                [IN]        Camera Handle
 *          const unsigned int nValue   [IN]        Camera X offset value to set
 *  @return Success, return MV_OK, and the camera X offset will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAOIoffsetX(IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAOIoffsetY(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取图像Y偏移
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机Y偏移的信息结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAOIoffsetY(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get image Y offset
 *  @param  void* handle                [IN]        Camera Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Return pointer of information structure related to camera Y offset to user
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAOIoffsetY(IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAOIoffsetX(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置图像AOI偏移
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的相机AOI的值
 *  @return 成功,返回MV_OK,并且相机AOI偏移将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAOIoffsetY(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set image Y offset
 *  @param  void* handle                [IN]        Camera Handle
 *          const unsigned int nValue   [IN]        Camera Y offset value to set
 *  @return Success, return MV_OK, and the camera Y offset will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAOIoffsetY(IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAutoExposureTimeLower(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取曝光下限
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机曝光值下限结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAutoExposureTimeLower(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get exposure lower limit
 *  @param  void* handle                [IN]        Camera Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Return pointer of information structure related to camera exposure lower to user
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAutoExposureTimeLower(
  IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAutoExposureTimeLower(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置曝光值下限
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的曝光值下限
 *  @return 成功,返回MV_OK,并且相机曝光下限将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAutoExposureTimeLower(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set exposure lower limit
 *  @param  void* handle                [IN]        Camera Handle
 *          const unsigned int nValue   [IN]        Exposure lower to set
 *  @return Success, return MV_OK, and the camera exposure time lower limit value will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAutoExposureTimeLower(
  IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAutoExposureTimeUpper(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取曝光上限
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机曝光值上限结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAutoExposureTimeUpper(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get exposure upper limit
 *  @param  void* handle                [IN]        Camera Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Return pointer of information structure related to camera exposure upper to user
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAutoExposureTimeUpper(
  IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAutoExposureTimeUpper(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置曝光值上限
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的曝光值上限
 *  @return 成功,返回MV_OK,并且相机曝光上限将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAutoExposureTimeUpper(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set exposure upper limit
 *  @param  void* handle                [IN]        Camera Handle
 *          const unsigned int nValue   [IN]        Exposure upper to set
 *  @return Success, return MV_OK, and the camera exposure time upper limit value will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAutoExposureTimeUpper(
  IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBrightness(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取亮度值
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机亮度结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBrightness(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get brightness
 *  @param  void* handle                [IN]        Camera Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Return pointer of information structure related to camera brightness to user
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBrightness(IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBrightness(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置亮度值
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的亮度值
 *  @return 成功,返回MV_OK,并且相机亮度将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBrightness(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set brightness
 *  @param  void* handle                [IN]        Camera Handle
 *          const unsigned int nValue   [IN]        Brightness upper to set
 *  @return Success, return MV_OK, and the camera brightness value will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBrightness(IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetFrameRate(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  获取帧率
 *  @param  void* handle                [IN]        相机句柄
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   返回给调用者有关相机帧率的信息结构体指针
 *          返回的pstValue结构体的意义
 *                                      float           fCurValue;      // 表示相机当前的帧率
 *                                      float           fMax;           // 表示相机允许设置的最大帧率
 *                                      float           fMin;           // 表示相机允许设置的最小帧率
 *  @return 成功,返回MV_OK,并获得相应参数信息的结构体, 失败, 返回错误码
 *  
 *          其他浮点型结构体参数的接口可参照此接口
 
 * @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetFrameRate(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  Get Frame Rate
 *  @param  void* handle                [IN]        Camera Handle
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   Return pointer of information structure related to camera frame rate to user
 *          The meaning of returns pstValue structure
 *                                      float           fCurValue;      // Indicates the current frame rate of the camera
 *                                      float           fMax;           // Indicates the maximum frame rate allowed by the camera
 *                                      float           fMin;           // Indicates the minimum frame rate allowed by the camera
 *  @return Success, return MV_OK, and get the structure of the corresponding parameters. Failure, return error code
 *  
 *          Other interface of Float structure parameters can refer to this interface
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetFrameRate(
  IN void * handle, IN OUT MVCC_FLOATVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetFrameRate(IN void* handle, IN const float fValue);
 *  @brief  设置帧率
 *  @param  void* handle                [IN]        相机句柄
 *          const float fValue          [IN]        想要设置的相机帧率
 *  @return 成功,返回MV_OK,并且相机帧率将会更改为相应值，失败,返回错误码
 
 * @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetFrameRate(IN void* handle, IN const float fValue);
 *  @brief  Set frame rate
 *  @param  void* handle                [IN]        Camera Handle
 *          const float fValue          [IN]        Camera frame rate to set 
 *  @return Success, return MV_OK, and camera frame rate will be changed to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetFrameRate(IN void * handle, IN const float fValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGain(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  获取增益
 *  @param  void* handle                [IN]        相机句柄
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   返回给调用者有关相机增益的信息结构体指针
 *          返回的pstValue结构体的意义
 *                                      float           fCurValue;      // 表示相机当前的帧率
 *                                      float           fMax;           // 表示相机允许设置的最大帧率
 *                                      float           fMin;           // 表示相机允许设置的最小帧率
 *  @return 成功,返回MV_OK,并获得相应参数信息的结构体, 失败, 返回错误码
 *  
 *          其他浮点型结构体参数的接口可参照此接口
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGain(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  Get Gain
 *  @param  void* handle                [IN]        Camera Handle
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   Return pointer of information structure related to gain to user
 *  @return Success, return MV_OK, and get the structure of the corresponding parameters. Failure, return error code
 *                                      float           fCurValue;      // Camera current gain
 *                                      float           fMax;           // The maximum gain camera allowed
 *                                      float           fMin;           // The minimum gain camera allowed
 *  @return Success, return MV_OK, and get the structure of the corresponding parameters. Failure, return error code
 *  
 *          Other interface of Float structure parameters can refer to this interface
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetGain(IN void * handle, IN OUT MVCC_FLOATVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGain(IN void* handle, IN const float fValue);
 *  @brief  设置帧率
 *  @param  void* handle                [IN]        相机句柄
 *          const float fValue          [IN]        想要设置的相机帧率
 *  @return 成功,返回MV_OK,并且相机帧率将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGain(IN void* handle, IN const float fValue);
 *  @brief  Set Gain
 *  @param  void* handle                [IN]        Camera Handle
 *          const float fValue          [IN]        Gain value to set
 *  @return Success, return MV_OK, and the camera gain value will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGain(IN void * handle, IN const float fValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetExposureTime(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  获取曝光时间
 *  @param  void* handle                [IN]        相机句柄
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   返回给调用者有关相机曝光时间的信息结构体指针
 *          返回的pstValue结构体的意义
 *                                      float           fCurValue;      // 表示相机当前的帧率
 *                                      float           fMax;           // 表示相机允许设置的最大帧率
 *                                      float           fMin;           // 表示相机允许设置的最小帧率
 *  @return 成功,返回MV_OK,并获得相应参数信息的结构体, 失败, 返回错误码
 *  
 *          其他浮点型结构体参数的接口可参照此接口
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetExposureTime(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  Get exposure time
 *  @param  void* handle                [IN]        Camera Handle
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   Return pointer of information structure related to exposure time to user
 *  @return Success, return MV_OK, and get the structure of the corresponding parameters. Failure, return error code
 *                                      float           fCurValue;      // Camera current exposure time
 *                                      float           fMax;           // The maximum exposure time camera allowed
 *                                      float           fMin;           // The minimum exposure time camera allowed
 *  @return Success, return MV_OK, and get the structure of the corresponding parameters. Failure, return error code
 *  
 *          Other interface of Float structure parameters can refer to this interface
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetExposureTime(
  IN void * handle, IN OUT MVCC_FLOATVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetExposureTime(IN void* handle, IN const float fValue);
 *  @brief  设置曝光时间
 *  @param  void* handle                [IN]        相机句柄
 *          const float fValue          [IN]        想要设置的相机帧率
 *  @return 成功,返回MV_OK,并且相机帧率将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetExposureTime(IN void* handle, IN const float fValue);
 *  @brief  Set exposure time
 *  @param  void* handle                [IN]        Camera Handle
 *          const float fValue          [IN]        Exposure time to set
 *  @return Success, return MV_OK, and the camera exposure time value will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetExposureTime(IN void * handle, IN const float fValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetPixelFormat(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  获取像素格式
 *  @param  void* handle                [IN]        相机句柄
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   返回给调用者的有关像素格式的信息结构体指针
 *          返回的pstValue结构体的意义
 *          unsigned int    nCurValue;                              //  相机当前的像素格式，是枚举类型,比如说PixelType_Gvsp_Mono8, 这里获得的是其整型值,具体数值参照PixelType.h的MvGvspPixelType枚举类型
 *          unsigned int    nSupportedNum;                          //  相机支持的像素格式的个数
 *          unsigned int    nSupportValue[MV_MAX_XML_SYMBOLIC_NUM]; //  相机所有支持的像素格式对应的整型值列表，后面要设置像素格式时，参数必须是这个数组中的一种，否则无效
 *  @return 成功,返回MV_OK,并获得相应参数信息的结构体, 失败, 返回错误码
 *  
            其他枚举类型参数接口可参照此接口，有关相应参数的枚举类型对应的整型值请查找PixelType.h 和 CameraParams.h中相应的定义
 
 * @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetPixelFormat(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  Get Pixel Format
 *  @param  void* handle                [IN]        Camera Handle
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   Returns the information structure pointer about pixel format for the caller
 *          The meaning of returns pstValue structure
 *          unsigned int    nCurValue;                              //  The current pixel format of the camera, is the enumeration type, such as PixelType_Gvsp_Mono8, here is the integer value, the specific value please refer to MvGvspPixelType enumeration type in PixelType.h
 *          unsigned int    nSupportedNum;                          //  Number of pixel formats supported by the camera
 *          unsigned int    nSupportValue[MV_MAX_XML_SYMBOLIC_NUM]; //  The integer values list correspond to all supported pixel formats of the camera, followed by when set the pixel format, the parameter must be one of this list, otherwise invalid
 *  @return Success, return MV_OK, and get the structure of the corresponding parameters. Failure, return error code
 *  
            Other interface of Enumeration structure parameters can refer to this interface, look for the corresponding definition in PixelType.h and CameraParams.h for the integer values of the enum type parameter
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetPixelFormat(
  IN void * handle, IN OUT MVCC_ENUMVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetPixelFormat(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置像素格式
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        要设置的像素格式对应的整型值，调用此接口时可以直接填写枚举值，如MV_CC_SetPixelFormat(m_handle, PixelType_Gvsp_RGB8_Packed);
 *  @return 成功,返回MV_OK,并且相机像素格式将会更改为相应值，失败,返回错误码
 *  
 *          要设置的枚举类型必须是Get接口返回的nSupportValue[MV_MAX_XML_SYMBOLIC_NUM]中的一种，否则会失败
 
 * @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetPixelFormat(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set Pixel Format
 *  @param  void* handle                [IN]        Camera Handle
 *          const unsigned int nValue   [IN]        The corresponding integer value for pixel format to be set, when calling this interface can be directly filled in enumeration values, such as MV_CC_SetPixelFormat(m_handle, PixelType_Gvsp_RGB8_Packed);
 *  @return Success, return MV_OK, and the camera pixel format will change to the corresponding value. Failure, return error code
 *  
 *          Other interface of Enumeration structure parameters can refer to this interface, the enumeration type to be set must be one of the nSupportValue [MV_MAX_XML_SYMBOLIC_NUM] returned by the Get interface, otherwise it will fail
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetPixelFormat(IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAcquisitionMode(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  获取采集模式
 *  @param  void* handle                [IN]        相机句柄
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   返回给调用者的有关采集模式的信息结构体指针
 *  @return 成功,返回MV_OK,并获得相应参数信息的结构体, 失败, 返回错误码
 *  
            可参照接口MV_CC_GetPixelFormat，参考 CameraParam.h 中的 MV_CAM_ACQUISITION_MODE 定义
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAcquisitionMode(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  Get acquisition mode
 *  @param  void* handle                [IN]        Handle
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   Structure pointer of acquisition mode
 *  @return Success, return MV_OK, and get the structure of the corresponding parameters. Failure, return error code
 *  
            Refer to MV_CC_GetPixelFormat and definition of MV_CAM_ACQUISITION_MODE in CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAcquisitionMode(
  IN void * handle, IN OUT MVCC_ENUMVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAcquisitionMode(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置像素格式
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        要设置的采集模式对应的整型值
 *  @return 成功,返回MV_OK,并且相机采集模式将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAcquisitionMode(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set acquisition mode
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Integer value to set corresponding to acquisition mode
 *  @return Success, return MV_OK, and the camera acquisition mode will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAcquisitionMode(
  IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGainMode(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  获取增益模式
 *  @param  void* handle                [IN]        相机句柄
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   返回给调用者的有关增益模式的信息结构体指针
 *  @return 成功,返回MV_OK,并获得相应参数信息的结构体, 失败, 返回错误码
 *  
            可参照接口MV_CC_GetPixelFormat，参考 CameraParam.h 中的 MV_CAM_GAIN_MODE 定义
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGainMode(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  Get gain mode
 *  @param  void* handle                [IN]        Handle
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]    Structure pointer of gain mode
 *  @return Success, return MV_OK, and get the structure of the corresponding parameters. Failure, return error code
 *  
            Refer to MV_CC_GetPixelFormat and definition of MV_CAM_GAIN_MODE in CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetGainMode(IN void * handle, IN OUT MVCC_ENUMVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGainMode(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置增益模式
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        要设置的增益模式对应的整型值
 *  @return 成功,返回MV_OK,并且相机增益模式将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGainMode(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set gain mode
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Integer value to set corresponding to gain mode
 *  @return Success, return MV_OK, and the camera gain mode will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGainMode(IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetExposureAutoMode(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  获取自动曝光模式
 *  @param  void* handle                [IN]        相机句柄
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   返回给调用者的有关自动曝光模式的信息结构体指针
 *  @return 成功,返回MV_OK,并获得相应参数信息的结构体, 失败, 返回错误码
 *  
            可参照接口MV_CC_GetPixelFormat，参考 CameraParam.h 中的 MV_CAM_EXPOSURE_AUTO_MODE 定义
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetExposureAutoMode(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  Get auto exposure mode
 *  @param  void* handle                [IN]        Handle
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   Structure pointer of auto exposure mode
 *  @return Success, return MV_OK, and get the structure of the corresponding parameters. Failure, return error code
 *  
            Refer to MV_CC_GetPixelFormat and definition of MV_CAM_EXPOSURE_AUTO_MODE in CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetExposureAutoMode(
  IN void * handle, IN OUT MVCC_ENUMVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetExposureAutoMode(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置自动曝光模式
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        要设置的自动曝光模式对应的整型值
 *  @return 成功,返回MV_OK,并且相机自动曝光模式将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetExposureAutoMode(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set auto exposure mode
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Integer value to set corresponding to auto exposure mode
 *  @return Success, return MV_OK, and the camera auto exposure mode will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetExposureAutoMode(
  IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerMode(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  获取触发模式
 *  @param  void* handle                [IN]        相机句柄
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   返回给调用者的有关触发模式的信息结构体指针
 *  @return 成功,返回MV_OK,并获得相应参数信息的结构体, 失败, 返回错误码
 *  
            可参照接口MV_CC_GetPixelFormat，参考 CameraParam.h 中的 MV_CAM_TRIGGER_MODE 定义
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerMode(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  Get trigger mode
 *  @param  void* handle                [IN]        Handle
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   Structure pointer of trigger mode
 *  @return Success, return MV_OK, and get the structure of the corresponding parameters. Failure, return error code
 *  
            Refer to MV_CC_GetPixelFormat and definition of MV_CAM_TRIGGER_MODE in CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerMode(
  IN void * handle, IN OUT MVCC_ENUMVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerMode(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置触发模式
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        要设置的触发模式对应的整型值
 *  @return 成功,返回MV_OK,并且相机触发模式将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerMode(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set trigger mode
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Integer value to set corresponding to trigger mode
 *  @return Success, return MV_OK, and the camera trigger mode will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerMode(IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerDelay(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  获取触发延时
 *  @param  void* handle                [IN]        相机句柄
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   返回给调用者有关相机触发延时的信息结构体指针
 *  @return 成功,返回MV_OK,并获得相应参数信息的结构体, 失败, 返回错误码
 *  
 *          可参照接口MV_CC_GetFrameRate
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerDelay(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  Get tigger delay
 *  @param  void* handle                [IN]        Handle
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   Structure pointer of trigger delay
 *  @return Success, return MV_OK, and get the structure of the corresponding parameters. Failure, return error code
 *  
 *          Refer to MV_CC_GetFrameRate
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerDelay(
  IN void * handle, IN OUT MVCC_FLOATVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerDelay(IN void* handle, IN const float fValue);
 *  @brief  设置触发延时
 *  @param  void* handle                [IN]        相机句柄
 *          const float fValue          [IN]        想要设置的相机触发延时
 *  @return 成功,返回MV_OK,并且相机触发延时将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerDelay(IN void* handle, IN const float fValue);
 *  @brief  Set tigger delay
 *  @param  void* handle                [IN]        Handle
 *          const float fValue          [IN]        Trigger delay to set
 *  @return Success, return MV_OK, and the camera trigger delay will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerDelay(IN void * handle, IN const float fValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerSource(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  获取触发源
 *  @param  void* handle                [IN]        相机句柄
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   返回给调用者的有关触发源的信息结构体指针
 *  @return 成功,返回MV_OK,并获得相应参数信息的结构体, 失败, 返回错误码
 *  
            可参照接口MV_CC_GetPixelFormat，参考 CameraParam.h 中的 MV_CAM_TRIGGER_SOURCE 定义
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerSource(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  Get trigger source
 *  @param  void* handle                [IN]        Handle
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   Structure pointer of trigger source
 *  @return Success, return MV_OK, and get the structure of the corresponding parameters. Failure, return error code
 *  
            Refer to MV_CC_GetPixelFormat and definition of MV_CAM_TRIGGER_SOURCE in CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerSource(
  IN void * handle, IN OUT MVCC_ENUMVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerSource(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置触发源
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        要设置的触发源对应的整型值
 *  @return 成功,返回MV_OK,并且相机触发源将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerSource(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set trigger source
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Integer value to set corresponding to trigger source
 *  @return Success, return MV_OK, and the camera trigger source will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerSource(IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_TriggerSoftwareExecute(IN void* handle);
 *  @brief  软触发一次（接口仅在已选择的触发源为软件触发时有效）
 *  @param  void* handle                [IN]        相机句柄
 *  @return 成功,返回MV_OK, 失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_TriggerSoftwareExecute(IN void* handle);
 *  @brief  Execute software trigger once (this interface only valid when the trigger source is set to software)
 *  @param  void* handle                [IN]        Handle
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_TriggerSoftwareExecute(IN void * handle);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGammaSelector(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  获取Gamma类型
 *  @param  void* handle                [IN]        相机句柄
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   返回给调用者的有关Gamma类型的信息结构体指针
 *  @return 成功,返回MV_OK,并获得相应参数信息的结构体, 失败, 返回错误码
 *  
            可参照接口MV_CC_GetPixelFormat，参考 CameraParam.h 中的 MV_CAM_GAMMA_SELECTOR 定义
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGammaSelector(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  Get Gamma mode
 *  @param  void* handle                [IN]        Handle
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   Structure pointer of gamma mode
 *  @return Success, return MV_OK, and get the structure of the corresponding parameters. Failure, return error code
 *  
            Refer to MV_CC_GetPixelFormat and definition of MV_CAM_GAMMA_SELECTOR in CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetGammaSelector(
  IN void * handle, IN OUT MVCC_ENUMVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGammaSelector(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置Gamma类型
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        要设置的Gamma类型对应的整型值
 *  @return 成功,返回MV_OK,并且相机Gamma类型将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGammaSelector(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set Gamma mode
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Integer value to set corresponding to gamma mode
 *  @return Success, return MV_OK, and the camera gamma mode will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGammaSelector(IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGamma(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  获取Gamma值
 *  @param  void* handle                [IN]        相机句柄
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   返回给调用者有关相机Gamma值的信息结构体指针
 *  @return 成功,返回MV_OK,并获得相应参数信息的结构体, 失败, 返回错误码
 *  
 *          可参照接口MV_CC_GetFrameRate
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGamma(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  Get Gamma value
 *  @param  void* handle                [IN]        Handle
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   Structure pointer of gamma value
 *  @return Success, return MV_OK, and get the structure of the corresponding parameters. Failure, return error code
 *  
 *          Refer to MV_CC_GetFrameRate
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetGamma(IN void * handle, IN OUT MVCC_FLOATVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGamma(IN void* handle, IN const float fValue);
 *  @brief  设置Gamma值
 *  @param  void* handle                [IN]        相机句柄
 *          const float fValue          [IN]        想要设置的相机Gamma值
 *  @return 成功,返回MV_OK,并且相机Gamma值将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGamma(IN void* handle, IN const float fValue);
 *  @brief  Set Gamma value
 *  @param  void* handle                [IN]        Handle
 *          const float fValue          [IN]        Gamma value to set
 *  @return Success, return MV_OK, and the camera gamma value will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGamma(IN void * handle, IN const float fValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetSharpness(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取锐度
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机锐度结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetSharpness(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get sharpness
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of sharpness
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetSharpness(IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetSharpness(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置锐度
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的锐度
 *  @return 成功,返回MV_OK,并且相机锐度将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetSharpness(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set sharpness
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Sharpness to set
 *  @return Success, return MV_OK, and the camera sharpness will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetSharpness(IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetHue(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取灰度
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机灰度结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetHue(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get Hue
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of Hue
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetHue(IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetHue(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置灰度
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的灰度
 *  @return 成功,返回MV_OK,并且相机灰度将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetHue(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set Hue
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Hue to set
 *  @return Success, return MV_OK, and the camera Hue will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetHue(IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetSaturation(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取饱和度
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机饱和度结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
  *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetSaturation(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get Saturation
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of Saturation
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetSaturation(IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetSaturation(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置饱和度
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的饱和度
 *  @return 成功,返回MV_OK,并且相机饱和度将会更改为相应值，失败,返回错误码
 
*  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetSaturation(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set Saturation
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Saturation to set
 *  @return Success, return MV_OK, and the camera Saturation will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetSaturation(IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceWhiteAuto(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  获取自动白平衡
 *  @param  void* handle                [IN]        相机句柄
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   返回给调用者的有关自动白平衡的信息结构体指针
 *  @return 成功,返回MV_OK,并获得相应参数信息的结构体, 失败, 返回错误码
 *  
            可参照接口MV_CC_GetPixelFormat，参考 CameraParam.h 中的 MV_CAM_BALANCEWHITE_AUTO 定义
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceWhiteAuto(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  Get Auto white balance
 *  @param  void* handle                [IN]        Handle
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   Structure pointer of auto white balance
 *  @return Success, return MV_OK, and get the structure of the corresponding parameters. Failure, return error code
 *  
            Refer to MV_CC_GetPixelFormat and definition of MV_CAM_BALANCEWHITE_AUTO in CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceWhiteAuto(
  IN void * handle, IN OUT MVCC_ENUMVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceWhiteAuto(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置自动白平衡
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        要设置的自动白平衡对应的整型值
 *  @return 成功,返回MV_OK,并且相机自动白平衡将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceWhiteAuto(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set Auto white balance
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Integer value to set corresponding to auto white balance
 *  @return Success, return MV_OK, and the camera auto white balance will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceWhiteAuto(
  IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioRed(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取白平衡 红
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机白平衡 红结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioRed(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get white balance red
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of white balance red
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioRed(
  IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioRed(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置白平衡 红
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的白平衡 红
 *  @return 成功,返回MV_OK,并且相机白平衡 红将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioRed(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set white balance red
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        White balance red to set
 *  @return Success, return MV_OK, and the camera white balance red will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioRed(
  IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioGreen(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取白平衡 绿
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机白平衡 绿结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioGreen(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get white balance green
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of white balance green
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioGreen(
  IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioGreen(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置白平衡 绿
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的白平衡 绿
 *  @return 成功,返回MV_OK,并且相机白平衡 绿将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioGreen(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set white balance green
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        White balance green to set
 *  @return Success, return MV_OK, and the camera white balance green will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioGreen(
  IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioBlue(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取白平衡 蓝
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机白平衡 蓝结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioBlue(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get white balance blue
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of white balance blue
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioBlue(
  IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioBlue(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置白平衡 蓝
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的白平衡 蓝
 *  @return 成功,返回MV_OK,并且相机白平衡 蓝将会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioBlue(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set white balance blue
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        White balance blue to set
 *  @return Success, return MV_OK, and the camera white balance blue will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioBlue(
  IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetFrameSpecInfoAbility(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取水印信息内包含的信息类型
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机水印信息内包含的信息类型结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetFrameSpecInfoAbility(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get information type included by frame stamp
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of information type included by frame stamp
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetFrameSpecInfoAbility(
  IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetFrameSpecInfoAbility(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置水印信息内包含的信息类型
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的水印信息内包含的信息类型
 *  @return 成功,返回MV_OK,并且相机水印信息内包含的信息类型会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetFrameSpecInfoAbility(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set information type included by frame stamp
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Information type included by frame stamp to set
 *  @return Success, return MV_OK, and the camera information type included by frame stamp will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetFrameSpecInfoAbility(
  IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetDeviceUserID(IN void* handle, IN OUT MVCC_STRINGVALUE* pstValue);
 *  @brief  获取设备自定义名字
 *  @param  void* handle                [IN]        相机句柄
 *          MVCC_STRINGVALUE* pstValue  [IN OUT]    返回给调用者有关相机名字结构体指针
 *  @return 成功,返回MV_OK,并且获取到相机的自定义名字，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetDeviceUserID(IN void* handle, IN OUT MVCC_STRINGVALUE* pstValue);
 *  @brief  Get device user defined name
 *  @param  void* handle                [IN]        Handle
 *          MVCC_STRINGVALUE* pstValue  [IN OUT]    Structure pointer of device name
 *  @return Success, return MV_OK, and get the camera user defined name. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetDeviceUserID(
  IN void * handle, IN OUT MVCC_STRINGVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetDeviceUserID(IN void* handle, IN const char* chValue);
 *  @brief  设置设备自定义名字
 *  @param  void* handle                [IN]        相机句柄
 *          IN const char* chValue      [IN]        设备名字
 *  @return 成功,返回MV_OK,并且设置设备自定义名字，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetDeviceUserID(IN void* handle, IN const char* chValue);
 *  @brief  Set device user defined name
 *  @param  void* handle                [IN]        Handle
 *          IN const char* chValue      [IN]        Device name
 *  @return Success, return MV_OK, and set the camera user defined name. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetDeviceUserID(IN void * handle, IN const char * chValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBurstFrameCount(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取一次触发的帧数
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机一次触发的帧数结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBurstFrameCount(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get frame number trigger by once
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of frame number trigger by once
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBurstFrameCount(
  IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBurstFrameCount(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置一次触发的帧数
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的一次触发的帧数
 *  @return 成功,返回MV_OK,并且相机一次触发的帧数会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBurstFrameCount(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set frame number trigger by once
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Frame number trigger by once to set
 *  @return Success, return MV_OK, and the camera frame number trigger by once will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBurstFrameCount(
  IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAcquisitionLineRate(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取行频
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机行频结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAcquisitionLineRate(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get line rate
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of line rate
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAcquisitionLineRate(
  IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAcquisitionLineRate(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置行频
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的行频
 *  @return 成功,返回MV_OK,并且相机行频会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAcquisitionLineRate(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set line rate
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Line rate to set
 *  @return Success, return MV_OK, and the camera line rate will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAcquisitionLineRate(
  IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetHeartBeatTimeout(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取心跳信息
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机心跳信息结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetHeartBeatTimeout(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get heartbeat information
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of heartbeat information
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetHeartBeatTimeout(
  IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetHeartBeatTimeout(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置心跳信息
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的心跳信息
 *  @return 成功,返回MV_OK,并且相机心跳信息会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetHeartBeatTimeout(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set heartbeat information
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Heartbeat information to set
 *  @return Success, return MV_OK, and the camera heartbeat information will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetHeartBeatTimeout(
  IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCPSPacketSize(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取网络包大小
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机网络包大小结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCPSPacketSize(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get network packet size
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of network packet size
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCPSPacketSize(
  IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCPSPacketSize(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置网络包大小
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的网络包大小
 *  @return 成功,返回MV_OK,并且相机网络包大小会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCPSPacketSize(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set network packet size
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Packet size to set
 *  @return Success, return MV_OK, and change packet size to setting value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCPSPacketSize(
  IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCPD(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  获取网络包发送间隔
 *  @param  void* handle                [IN]        相机句柄
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   返回给调用者有关相机网络包发送间隔结构体指针
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  
 *          可参照接口MV_CC_GetWidth
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCPD(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get network packet sending delay
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of network packet sending delay
 *  @return Success, return MV_OK. Failure, return error code
 *  
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCPD(IN void * handle, IN OUT MVCC_INTVALUE * pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCPD(IN void* handle, IN const unsigned int nValue);
 *  @brief  设置网络包发送间隔
 *  @param  void* handle                [IN]        相机句柄
 *          const unsigned int nValue   [IN]        想要设置的网络包发送间隔
 *  @return 成功,返回MV_OK,并且相机网络包发送间隔会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCPD(IN void* handle, IN const unsigned int nValue);
 *  @brief  Set network packet sending delay
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Packet delay to set
 *  @return Success, return MV_OK, and change packet delay to setting value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCPD(IN void * handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCDA(IN void* handle, unsigned int* pnIP);
 *  @brief  获取接收端IP地址，0xa9fe0102 表示 169.254.1.2
 *  @param  void* handle                [IN]        相机句柄
 *  @param  unsigned int* pnIP          [IN][OUT]   返回给调用者接收端IP地址
 *  @return 成功,返回MV_OK,失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCDA(IN void* handle, unsigned int* pnIP);
 *  @brief  Get receiver IP address, 0xa9fe0102 indicates 169.254.1.2
 *  @param  void* handle                [IN]        Handle
 *  @param  unsigned int* pnIP          [IN][OUT]   Receiver IP address
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCDA(IN void * handle, unsigned int * pnIP);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCDA(IN void* handle, unsigned int nIP);
 *  @brief  设置接收端IP地址
 *  @param  void* handle                [IN]        相机句柄
 *          unsigned int nIP            [IN]        想要设置的接收端IP地址
 *  @return 成功,返回MV_OK,并且相机接收端IP地址会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCDA(IN void* handle, unsigned int nIP);
 *  @brief  Set receiver IP address
 *  @param  void* handle                [IN]        Handel
 *          unsigned int nIP            [IN]        Receiver IP address to set
 *  @return Success, return MV_OK, and change receiver IP address to setting value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCDA(IN void * handle, unsigned int nIP);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCSP(IN void* handle, unsigned int* pnPort);
 *  @brief  获取发送端的端口号
 *  @param  void* handle                [IN]        相机句柄
 *  @param  unsigned int* pnPort        [IN][OUT]   返回给调用者发送端的端口号
 *  @return 成功,返回MV_OK,失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCSP(IN void* handle, unsigned int* pnPort);
 *  @brief  Get transmitter port number
 *  @param  void* handle                [IN]        Handle
 *  @param  unsigned int* pnPort        [IN][OUT]   Transmitter port number
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCSP(IN void * handle, unsigned int * pnPort);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCSP(IN void* handle, unsigned int nPort);
 *  @brief  设置发送端的端口号
 *  @param  void* handle                [IN]        相机句柄
 *          unsigned int nPort          [IN]        想要设置的发送端的端口号
 *  @return 成功,返回MV_OK,并且相机发送端的端口号会更改为相应值，失败,返回错误码
 
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCSP(IN void* handle, unsigned int nPort);
 *  @brief  Set transmitter port number
 *  @param  void* handle                [IN]        Handle
 *          unsigned int nPort          [IN]        Transmitter port number to set
 *  @return Success, return MV_OK, and change transmitter port number to setting value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCSP(IN void * handle, unsigned int nPort);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  设置设备波特率
 *  @param  handle                      [IN]            设备句柄
 *  @param  nBaudrate                   [IN]            设置的波特率值，数值参考CameraParams.h中宏定义，如#define MV_CAML_BAUDRATE_9600  0x00000001
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks （该接口已弃用，建议改用 MV_CAML_SetDeviceBaudrate接口）
 
 *  @~english
 *  @brief  Set device baudrate using one of the CL_BAUDRATE_XXXX value
 *  @param  handle                      [IN]            Device handle
 *  @param  nBaudrate                   [IN]            baud rate to set. Refer to the 'CameraParams.h' for parameter definitions, for example, #define MV_CAML_BAUDRATE_9600  0x00000001
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks (This interface is abandoned, it is recommended to use the MV_CAML_SetDeviceBaudrate)
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_SetDeviceBauderate(IN void * handle, unsigned int nBaudrate);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取设备波特率
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnCurrentBaudrate           [OUT]           波特率信息指针，数值参考CameraParams.h中宏定义，如#define MV_CAML_BAUDRATE_9600  0x00000001
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks （该接口已弃用，建议改用 MV_CAML_GetDeviceBaudrate接口）
 
 *  @~english
 *  @brief  Returns the current device baudrate, using one of the CL_BAUDRATE_XXXX value
 *  @param  handle                      [IN]            Device handle
 *  @param  pnCurrentBaudrate           [OUT]           Return pointer of baud rate to user. Refer to the 'CameraParams.h' for parameter definitions, for example, #define MV_CAML_BAUDRATE_9600  0x00000001
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks (This interface is abandoned, it is recommended to use the MV_CAML_GetDeviceBaudrate)
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_GetDeviceBauderate(
  IN void * handle, unsigned int * pnCurrentBaudrate);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  获取设备与主机间连接支持的波特率
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnBaudrateAblity            [OUT]           支持的波特率信息的指针。所支持波特率的或运算结果，单个数值参考CameraParams.h中宏定义，如MV_CAML_BAUDRATE_9600  0x00000001
 *  @return 成功,返回MV_OK,失败,返回错误码
 *  @remarks （该接口已弃用，建议改用 MV_CAML_GetSupportBaudrates接口）
 
 *  @~english
 *  @brief  Returns supported baudrates of the combined device and host interface
 *  @param  handle                      [IN]            Device handle
 *  @param  pnBaudrateAblity            [OUT]           Return pointer of the supported baudrates to user. 'OR' operation results of the supported baudrates. Refer to the 'CameraParams.h' for single value definitions, for example, MV_CAML_BAUDRATE_9600  0x00000001
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks (This interface is abandoned, it is recommended to use the MV_CAML_GetSupportBaudrates)
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_GetSupportBauderates(
  IN void * handle, unsigned int * pnBaudrateAblity);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  保存图片，支持Bmp和Jpeg.
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstSaveParam                [IN][OUT]       保存图片参数结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 通过将接口可以将从设备采集到的原始图像数据转换成JPEG或者BMP等格式并存放在指定内存中，然后用户可以将转换之后的数据直接保存成图片文件。
             该接口调用无接口顺序要求，有图像源数据就可以进行转换，可以先调用MV_CC_GetOneFrameTimeout或者MV_CC_RegisterImageCallBackEx设置回调函数，获取一帧图像数据，然后再通过该接口转换格式。
             MV_CC_SaveImageEx2比MV_CC_SaveImageEx增加参数handle，为了保证与其他接口的统一。
 
 *  @~english
 *  @brief  Save image, support Bmp and Jpeg.
 *  @param  handle                      [IN]            Device handle
 *  @param  pstSaveParam                [IN][OUT]       Save image parameters structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Once there is image data, you can call this API to convert the data.
             You can also call MV_CC_GetOneFrameTimeout or MV_CC_RegisterImageCallBackEx or MV_CC_GetImageBuffer to get one image frame and set the callback function, and then call this API to convert the format.
             Comparing with the API MV_CC_SaveImageEx, this API added the parameter handle to ensure the unity with other API. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImageEx2(
  IN void * handle, MV_SAVE_IMAGE_PARAM_EX * pstSaveParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  保存图像到文件
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstSaveFileParam            [IN][OUT]       保存图片文件参数结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 该接口支持BMP/JPEG/PNG/TIFF。
 
 *  @~english
 *  @brief  Save the image file.
 *  @param  handle                      [IN]            Device handle
 *  @param  pstSaveFileParam            [IN][OUT]       Save the image file parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API support BMP/JPEG/PNG/TIFF.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImageToFile(
  IN void * handle, MV_SAVE_IMG_TO_FILE_PARAM * pstSaveFileParam);

/********************************************************************/ /**
 *  @~chinese
 *  @brief  像素格式转换
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstCvtParam                 [IN][OUT]       像素格式转换参数结构体
 *  @return 成功，返回MV_OK；错误，返回错误码 
 *  @remarks 通过将接口可以将从设备采集到的原始图像数据转换成用户所需的像素格式并存放在指定内存中。
             该接口调用无接口顺序要求，有图像源数据就可以进行转换，可以先调用MV_CC_GetOneFrameTimeout或者MV_CC_RegisterImageCallBackEx设置回调函数，
             获取一帧图像数据，然后再通过该接口转换格式。如果设备当前采集图像是JPEG压缩的格式，则不支持调用该接口进行转换。
 
 *  @~english
 *  @brief  Pixel format conversion
 *  @param  handle                      [IN]            Device handle
 *  @param  pstCvtParam                 [IN][OUT]       Convert Pixel Type parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API is used to transform the collected original data to pixel format and save to specified memory. 
             There is no order requirement to call this API, the transformation will execute when there is image data. 
             First call MV_CC_GetOneFrameTimeout or MV_CC_RegisterImageCallBackEx to set callback function, and get a frame of image data,
             then call this API to transform the format.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ConvertPixelType(
  IN void * handle, IN OUT MV_CC_PIXEL_CONVERT_PARAM * pstCvtParam);

#ifdef __cplusplus
}
#endif

#endif  //_MV_OBSOLETE_INTERFACES_H_
