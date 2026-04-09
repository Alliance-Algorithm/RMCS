
#ifndef _MV_ISP_ERROR_DEFINE_H_
#define _MV_ISP_ERROR_DEFINE_H_

/************************************************************************
* 来自ISP算法库的错误码
************************************************************************/
// 通用类型
#define MV_ALG_OK 0x00000000   //处理正确
#define MV_ALG_ERR 0x10000000  //不确定类型错误

// 能力检查
#define MV_ALG_E_ABILITY_ARG 0x10000001  //能力集中存在无效参数

// 内存检查
#define MV_ALG_E_MEM_NULL 0x10000002        //内存地址为空
#define MV_ALG_E_MEM_ALIGN 0x10000003       //内存对齐不满足要求
#define MV_ALG_E_MEM_LACK 0x10000004        //内存空间大小不够
#define MV_ALG_E_MEM_SIZE_ALIGN 0x10000005  //内存空间大小不满足对齐要求
#define MV_ALG_E_MEM_ADDR_ALIGN 0x10000006  //内存地址不满足对齐要求

// 图像检查
#define MV_ALG_E_IMG_FORMAT 0x10000007     //图像格式不正确或者不支持
#define MV_ALG_E_IMG_SIZE 0x10000008       //图像宽高不正确或者超出范围
#define MV_ALG_E_IMG_STEP 0x10000009       //图像宽高与step参数不匹配
#define MV_ALG_E_IMG_DATA_NULL 0x1000000A  //图像数据存储地址为空

// 输入输出参数检查
#define MV_ALG_E_CFG_TYPE 0x1000000B   //设置或者获取参数类型不正确
#define MV_ALG_E_CFG_SIZE 0x1000000C   //设置或者获取参数的输入、输出结构体大小不正确
#define MV_ALG_E_PRC_TYPE 0x1000000D   //处理类型不正确
#define MV_ALG_E_PRC_SIZE 0x1000000E   //处理时输入、输出参数大小不正确
#define MV_ALG_E_FUNC_TYPE 0x1000000F  //子处理类型不正确
#define MV_ALG_E_FUNC_SIZE 0x10000010  //子处理时输入、输出参数大小不正确

// 运行参数检查
#define MV_ALG_E_PARAM_INDEX 0x10000011  //index参数不正确
#define MV_ALG_E_PARAM_VALUE 0x10000012  //value参数不正确或者超出范围
#define MV_ALG_E_PARAM_NUM 0x10000013    //param_num参数不正确

// 接口调用检查
#define MV_ALG_E_NULL_PTR 0x10000014      //函数参数指针为空
#define MV_ALG_E_OVER_MAX_MEM 0x10000015  //超过限定的最大内存
#define MV_ALG_E_CALL_BACK 0x10000016     //回调函数出错

// 算法库加密相关检查
#define MV_ALG_E_ENCRYPT 0x10000017  //加密错误
#define MV_ALG_E_EXPIRE 0x10000018   //算法库使用期限错误

// 内部模块返回的基本错误类型
#define MV_ALG_E_BAD_ARG 0x10000019    //参数范围不正确
#define MV_ALG_E_DATA_SIZE 0x1000001A  //数据大小不正确
#define MV_ALG_E_STEP 0x1000001B       //数据step不正确

// cpu指令集支持错误码
#define MV_ALG_E_CPUID 0x1000001C  //cpu不支持优化代码中的指令集

#define MV_ALG_WARNING 0x1000001D  //警告

#define MV_ALG_E_TIME_OUT 0x1000001E          //算法库超时
#define MV_ALG_E_LIB_VERSION 0x1000001F       //算法版本号出错
#define MV_ALG_E_MODEL_VERSION 0x10000020     //模型版本号出错
#define MV_ALG_E_GPU_MEM_ALLOC 0x10000021     //GPU内存分配错误
#define MV_ALG_E_FILE_NON_EXIST 0x10000022    //文件不存在
#define MV_ALG_E_NONE_STRING 0x10000023       //字符串为空
#define MV_ALG_E_IMAGE_CODEC 0x10000024       //图像解码器错误
#define MV_ALG_E_FILE_OPEN 0x10000025         //打开文件错误
#define MV_ALG_E_FILE_READ 0x10000026         //文件读取错误
#define MV_ALG_E_FILE_WRITE 0x10000027        //文件写错误
#define MV_ALG_E_FILE_READ_SIZE 0x10000028    //文件读取大小错误
#define MV_ALG_E_FILE_TYPE 0x10000029         //文件类型错误
#define MV_ALG_E_MODEL_TYPE 0x1000002A        //模型类型错误
#define MV_ALG_E_MALLOC_MEM 0x1000002B        //分配内存错误
#define MV_ALG_E_BIND_CORE_FAILED 0x1000002C  //线程绑核失败

// 降噪特有错误码
#define MV_ALG_E_DENOISE_NE_IMG_FORMAT 0x10402001      //噪声特性图像格式错误
#define MV_ALG_E_DENOISE_NE_FEATURE_TYPE 0x10402002    //噪声特性类型错误
#define MV_ALG_E_DENOISE_NE_PROFILE_NUM 0x10402003     //噪声特性个数错误
#define MV_ALG_E_DENOISE_NE_GAIN_NUM 0x10402004        //噪声特性增益个数错误
#define MV_ALG_E_DENOISE_NE_GAIN_VAL 0x10402005        //噪声曲线增益值输入错误
#define MV_ALG_E_DENOISE_NE_BIN_NUM 0x10402006         //噪声曲线柱数错误
#define MV_ALG_E_DENOISE_NE_INIT_GAIN 0x10402007       //噪声估计初始化增益设置错误
#define MV_ALG_E_DENOISE_NE_NOT_INIT 0x10402008        //噪声估计未初始化
#define MV_ALG_E_DENOISE_COLOR_MODE 0x10402009         //颜色空间模式错误
#define MV_ALG_E_DENOISE_ROI_NUM 0x1040200a            //图像ROI个数错误
#define MV_ALG_E_DENOISE_ROI_ORI_PT 0x1040200b         //图像ROI原点错误
#define MV_ALG_E_DENOISE_ROI_SIZE 0x1040200c           //图像ROI大小错误
#define MV_ALG_E_DENOISE_GAIN_NOT_EXIST 0x1040200d     //输入的相机增益不存在(增益个数已达上限)
#define MV_ALG_E_DENOISE_GAIN_BEYOND_RANGE 0x1040200e  //输入的相机增益不在范围内
#define MV_ALG_E_DENOISE_NP_BUF_SIZE 0x1040200f        //输入的噪声特性内存大小错误

#endif  //_MV_ISP_ERROR_DEFINE_H_
