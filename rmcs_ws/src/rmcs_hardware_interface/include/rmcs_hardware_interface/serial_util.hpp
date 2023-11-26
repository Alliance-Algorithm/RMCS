#pragma once
/*
Creation Date: 2023/04/21
Latest Update: 2023/05/05
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 串口工具
*/

#include <cstdint>

#include "serial/serial.h"
#include "verify.hpp"

namespace serial {

    // 标记该项不存在
    struct None { };

#pragma pack(push, 1)
    template <typename HeadDataType, HeadDataType HeadValue>
    struct Head {
        HeadDataType data;

        // 令其值正确
        void MakeCorrect() {
            data = HeadValue;
        }

        // 令其值不正确
        void MakeIncorrect() {
            data = ~HeadValue;
        }

        // 判断值是否正确
        bool IsCorrect() {
            return data == HeadValue;
        }
    };

    template <typename DataType, typename HeadType, typename ChecksumResultType>
    struct Package {
        HeadType head;
        DataType data;
        ChecksumResultType crc;
    };

    template <typename DataType>
    struct Package<DataType, None, None> {
        DataType data;
    };

    template <typename DataType, typename HeadType>
    struct Package<DataType, HeadType, None> {
        HeadType head;
        DataType data;
    };

    template <typename DataType, typename ChecksumResultType>
    struct Package<DataType, None, ChecksumResultType> {
        DataType data;
        ChecksumResultType crc;
    };
#pragma pack(pop)


    // 有头有校验的串口数据包发送器
    template <typename DataType, typename HeadType, typename ChecksumCalculatorType>
    class SerialSender {
    public:
        explicit SerialSender(serial::Serial& serial) : _serial(serial) {
            _pkg.head.MakeCorrect();
        }
        SerialSender(const SerialSender&) = delete;
        SerialSender(SerialSender&&) = delete;

        void Send() {
            ChecksumCalculatorType::Append(_pkg);
            _serial.write(reinterpret_cast<uint8_t*>(&_pkg), sizeof(_pkg));
        }

        DataType& Data = _pkg.data;
    private:
        serial::Serial& _serial;
        Package<DataType, HeadType, typename ChecksumCalculatorType::ResultType> _pkg;
    };

    // 无头无校验的串口数据包发送器
    template <typename DataType>
    class SerialSender<DataType, None, None> {
    public:
        explicit SerialSender(serial::Serial& serial) : _serial(serial) { }
        SerialSender(const SerialSender&) = delete;
        SerialSender(SerialSender&&) = delete;

        void Send() {
            _serial.write(reinterpret_cast<uint8_t*>(&_pkg), sizeof(_pkg));
        }

        DataType& Data = _pkg.data;
    private:
        serial::Serial& _serial;
        Package<DataType, None, None> _pkg;
    };

    // 有头无校验的串口数据包发送器
    template <typename DataType, typename HeadType>
    class SerialSender<DataType, HeadType, None> {
    public:
        explicit SerialSender(serial::Serial& serial) : _serial(serial) {
            _pkg.head.MakeCorrect();
        }
        SerialSender(const SerialSender&) = delete;
        SerialSender(SerialSender&&) = delete;

        void Send() {
            _serial.write(reinterpret_cast<uint8_t*>(&_pkg), sizeof(_pkg));
        }

        DataType& Data = _pkg.data;
    private:
        serial::Serial& _serial;
        Package<DataType, HeadType, None> _pkg;
    };

    // 无头有校验的串口数据包发送器
    template <typename DataType, typename ChecksumCalculatorType>
    class SerialSender<DataType, None, ChecksumCalculatorType> {
    public:
        explicit SerialSender(serial::Serial& serial) : _serial(serial) { }
        SerialSender(const SerialSender&) = delete;
        SerialSender(SerialSender&&) = delete;

        void Send() {
            ChecksumCalculatorType::Append(_pkg);
            _serial.write(reinterpret_cast<uint8_t*>(&_pkg), sizeof(_pkg));
        }

        DataType& Data = _pkg.data;
    private:
        serial::Serial& _serial;
        Package<DataType, None, typename ChecksumCalculatorType::ResultType> _pkg;
    };


    enum class ReceiveResult : uint8_t {
        Success = 0, Timeout = 1, InvaildHeader = 2, InvaildVerifyDegit = 4
    };

    // 有头有校验的串口数据包接收器
    template <typename DataType, typename HeadType, typename ChecksumCalculatorType>
    class SerialReceiver {
    public:
        explicit SerialReceiver(serial::Serial& serial) : _serial(serial) { }
        SerialReceiver(const SerialReceiver&) = delete;
        SerialReceiver(SerialReceiver&&) = delete;

        /*! 接收最多一个数据，不丢弃任何数据，除校验未通过之外。适用于接收发送频率较低的数据。
        * 该方法可以造成阻塞，由于每次调用该方法只会调用一次serial.read，因此超时时间取决于Serial对象本身的超时时间设定，若设为0则不阻塞。
        * 使用GetCacheSize方法可以获取已缓存数据的长度，使用ClearCache方法可以强制清空缓存的数据。
        *
        * \return 成功时，返回Success
        * 当未接收到足够数量数据，导致串口接收超时时，该方法返回Timeout，在下次调用该方法时会尝试接收剩余部分。
        * 当接收到的数据包头未对齐，或校验未通过，该方法返回InvaildHeader或InvaildVerifyDegit，并向后寻找匹配的头，在下次调用该方法时尝试接收剩余部分。
        */
        ReceiveResult Receive() {
            ReceiveResult result;
            // 接收尽可能多的数据
            _cacheSize += _serial.read(reinterpret_cast<uint8_t*>(&_receivePkg->head) + _cacheSize, sizeof(PackageType) - _cacheSize);

            if (_cacheSize >= sizeof(HeadType)) {                 // 成功接收到包头部分
                if (_receivePkg->head.IsCorrect()) {               // 数据包头正确
                    if (_cacheSize == sizeof(PackageType)) {    // 若包头正确且数据接收完整，则尝试进一步校验
                        if (ChecksumCalculatorType::Verify(*_receivePkg)) {
                            _cacheSize = 0;
                            std::swap(_receivePkg, _resultPkg);
                            return ReceiveResult::Success;
                        }
                        result = ReceiveResult::InvaildVerifyDegit;
                    }
                    else return ReceiveResult::Timeout;                         // 若包头正确但数据未接收完整，则返回Timeout，等待下一次接收
                }
                else result = ReceiveResult::InvaildHeader;

                // 若数据包头错误，则向后寻找匹配的头，若找到，则把数据整体向前对齐，以便下一次接收剩余部分
                // 即使没有找到匹配的头，当遍历剩余字节数低于头的字节数时，也会把剩余字节向前对齐
                --_cacheSize;
                uint8_t* bufferHead = reinterpret_cast<uint8_t*>(&_receivePkg->head) + 1;

                while (true) {
                    if (_cacheSize < sizeof(HeadType) || reinterpret_cast<HeadType*>(bufferHead)->IsCorrect()) {
                        for (size_t i = 0; i < _cacheSize; ++i)
                            reinterpret_cast<uint8_t*>(&_receivePkg->head)[i] = bufferHead[i];
                        break;
                    }
                    --_cacheSize;
                    ++bufferHead;
                }
                return result;
            }
            else return ReceiveResult::Timeout;                   // 若连包头都没接到，也返回Timeout
        }

        /* 获取接收到的数据 */
        const DataType& GetReceivedData() { return _resultPkg->data; }

        /* 获取当前缓存的数据长度 */
        size_t GetCacheSize() { return _cacheSize; }

        /* 重置当前接收缓存 */
        void ClearCache() { _cacheSize = 0; }

    private:
        serial::Serial& _serial;
        size_t _cacheSize = 0;  // 标记缓存的数据长度

        using PackageType = Package<DataType, HeadType, typename ChecksumCalculatorType::ResultType>;
        // _receivePkg指向接收缓冲区, _resultPkg指向上一个已接收并验证通过的缓冲区
        PackageType _pkg[2], * _receivePkg = &_pkg[0], * _resultPkg = &_pkg[1];
    };

    // 无头无校验的串口数据包接收器
    template <typename DataType>
    class SerialReceiver<DataType, None, None> {
    public:
        explicit SerialReceiver(serial::Serial& serial) : _serial(serial) { }
        SerialReceiver(const SerialReceiver&) = delete;
        SerialReceiver(SerialReceiver&&) = delete;

        /*! 接收最多一个数据，不丢弃任何数据，除校验未通过之外。适用于接收发送频率较低的数据。
        * 该方法可以造成阻塞，由于每次调用该方法只会调用一次serial.read，因此超时时间取决于Serial对象本身的超时时间设定，若设为0则不阻塞。
        * 使用GetCacheSize方法可以获取已缓存数据的长度，使用ClearCache方法可以强制清空缓存的数据。
        *
        * \return 成功时，返回Success
        * 当未接收到足够数量数据，导致串口接收超时时，该方法返回Timeout，在下次调用该方法时会尝试接收剩余部分。
        * 当接收到的数据包头未对齐，或校验未通过，该方法返回InvaildHeader或InvaildVerifyDegit，并向后寻找匹配的头，在下次调用该方法时尝试接收剩余部分。
        */
        ReceiveResult Receive() {
            _cacheSize += _serial.read(reinterpret_cast<uint8_t*>(_receivePkg) + _cacheSize, sizeof(DataType) - _cacheSize);

            if (_cacheSize == sizeof(DataType)) {
                _cacheSize = 0;
                std::swap(_receivePkg, _resultPkg);
                return ReceiveResult::Success;
            }
            else return ReceiveResult::Timeout;
        }

        /* 获取接收到的数据 */
        const DataType& GetReceivedData() { return _resultPkg->data; }

        /* 获取当前缓存的数据长度 */
        size_t GetCacheSize() { return _cacheSize; }

        /* 重置当前接收缓存 */
        void ClearCache() { _cacheSize = 0; }

    private:
        serial::Serial& _serial;
        size_t _cacheSize = 0;  // 标记缓存的数据长度
        Package<DataType, None, None> _pkg[2], * _receivePkg = &_pkg[0], * _resultPkg = &_pkg[1];
    };

    // 有头无校验的串口数据包接收器
    template <typename DataType, typename HeadType>
    class SerialReceiver<DataType, HeadType, None> {
    public:
        explicit SerialReceiver(serial::Serial& serial) : _serial(serial) { }
        SerialReceiver(const SerialReceiver&) = delete;
        SerialReceiver(SerialReceiver&&) = delete;

        /*! 接收最多一个数据，不丢弃任何数据，除校验未通过之外。适用于接收发送频率较低的数据。
        * 该方法可以造成阻塞，由于每次调用该方法只会调用一次serial.read，因此超时时间取决于Serial对象本身的超时时间设定，若设为0则不阻塞。
        * 使用GetCacheSize方法可以获取已缓存数据的长度，使用ClearCache方法可以强制清空缓存的数据。
        *
        * \return 成功时，返回Success
        * 当未接收到足够数量数据，导致串口接收超时时，该方法返回Timeout，在下次调用该方法时会尝试接收剩余部分。
        * 当接收到的数据包头未对齐，或校验未通过，该方法返回InvaildHeader或InvaildVerifyDegit，并向后寻找匹配的头，在下次调用该方法时尝试接收剩余部分。
        */
        ReceiveResult Receive() {
            ReceiveResult result;
            // 接收尽可能多的数据
            _cacheSize += _serial.read(reinterpret_cast<uint8_t*>(&_receivePkg->head) + _cacheSize, sizeof(HeadType) + sizeof(DataType) - _cacheSize);

            if (_cacheSize >= sizeof(HeadType)) {                 // 成功接收到包头部分
                if (_receivePkg->head.IsCorrect()) {               // 数据包头正确
                    if (_cacheSize == sizeof(HeadType) + sizeof(DataType)) {    // 若包头正确且数据接收完整，则返回Success
                        _cacheSize = 0;
                        std::swap(_receivePkg, _resultPkg);
                        return ReceiveResult::Success;
                    }
                    else return ReceiveResult::Timeout;                         // 若包头正确但数据未接收完整，则返回Timeout，等待下一次接收
                }
                else result = ReceiveResult::InvaildHeader;

                // 若数据包头错误，则向后寻找匹配的头，若找到，则把数据整体向前对齐，以便下一次接收剩余部分
                // 即使没有找到匹配的头，当遍历剩余字节数低于头的字节数时，也会把剩余字节向前对齐
                --_cacheSize;
                uint8_t* bufferHead = reinterpret_cast<uint8_t*>(&_receivePkg->head) + 1;

                while (true) {
                    if (_cacheSize < sizeof(HeadType) || reinterpret_cast<HeadType*>(bufferHead)->IsCorrect()) {
                        for (size_t i = 0; i < _cacheSize; ++i)
                            reinterpret_cast<uint8_t*>(&_receivePkg->head)[i] = bufferHead[i];
                        break;
                    }
                    --_cacheSize;
                    ++bufferHead;
                }
                return result;
            }
            else return ReceiveResult::Timeout;                   // 若连包头都没接到，也返回Timeout
        }

        /* 获取接收到的数据 */
        const DataType& GetReceivedData() { return _resultPkg->data; }

        /* 获取当前缓存的数据长度 */
        size_t GetCacheSize() { return _cacheSize; }

        /* 重置当前接收缓存 */
        void ClearCache() { _cacheSize = 0; }

    private:
        serial::Serial& _serial;
        size_t _cacheSize = 0;  // 标记缓存的数据长度

        // _receivePkg指向接收缓冲区, _resultPkg指向上一个已接收并验证通过的缓冲区
        Package<DataType, HeadType, None> _pkg[2], * _receivePkg = &_pkg[0], * _resultPkg = &_pkg[1];

    };

    // 无头有校验的串口数据包接收器
    template <typename DataType, typename ChecksumCalculatorType>
    class SerialReceiver<DataType, None, ChecksumCalculatorType> {
    public:
        explicit SerialReceiver(serial::Serial& serial) : _serial(serial) { }
        SerialReceiver(const SerialReceiver&) = delete;
        SerialReceiver(SerialReceiver&&) = delete;

        /*! 接收最多一个数据，不丢弃任何数据，除校验未通过之外。适用于接收发送频率较低的数据。
        * 该方法可以造成阻塞，由于每次调用该方法只会调用一次serial.read，因此超时时间取决于Serial对象本身的超时时间设定，若设为0则不阻塞。
        * 使用GetCacheSize方法可以获取已缓存数据的长度，使用ClearCache方法可以强制清空缓存的数据。
        *
        * \return 成功时，返回Success
        * 当未接收到足够数量数据，导致串口接收超时时，该方法返回Timeout，在下次调用该方法时会尝试接收剩余部分。
        * 当接收到的数据包头未对齐，或校验未通过，该方法返回InvaildHeader或InvaildVerifyDegit，并向后寻找匹配的头，在下次调用该方法时尝试接收剩余部分。
        */
        ReceiveResult Receive() {
            // 接收尽可能多的数据
            _cacheSize += _serial.read(reinterpret_cast<uint8_t*>(&_receivePkg->data) + _cacheSize,
                sizeof(DataType) + sizeof(typename ChecksumCalculatorType::ResultType) - _cacheSize);

            if (_cacheSize == sizeof(DataType) + sizeof(typename ChecksumCalculatorType::ResultType)) {    // 若数据接收完整，则尝试进一步校验
                if (ChecksumCalculatorType::Verify(*_receivePkg)) {
                    // 校验通过则返回Success
                    _cacheSize = 0;
                    std::swap(_receivePkg, _resultPkg);
                    return ReceiveResult::Success;
                }
                else {
                    // 若校验不通过，则把数据整体向前对齐一位，下一次接收剩余部分
                    --_cacheSize;
                    uint8_t* bufferHead = reinterpret_cast<uint8_t*>(&_receivePkg->data) + 1;
                    for (size_t i = 0; i < _cacheSize; ++i)
                        reinterpret_cast<uint8_t*>(&_receivePkg->data)[i] = bufferHead[i];
                    return ReceiveResult::InvaildVerifyDegit;
                }
            }

            // 若数据未接收完整，则返回Timeout，等待下一次接收
            return ReceiveResult::Timeout;
        }

        /* 获取接收到的数据 */
        const DataType& GetReceivedData() { return _resultPkg->data; }

        /* 获取当前缓存的数据长度 */
        size_t GetCacheSize() { return _cacheSize; }

        /* 重置当前接收缓存 */
        void ClearCache() { _cacheSize = 0; }

    private:
        serial::Serial& _serial;
        size_t _cacheSize = 0;  // 标记缓存的数据长度

        // _receivePkg指向接收缓冲区, _resultPkg指向上一个已接收并验证通过的缓冲区
        Package<DataType, None, typename ChecksumCalculatorType::ResultType> _pkg[2], * _receivePkg = &_pkg[0], * _resultPkg = &_pkg[1];
    };

}
