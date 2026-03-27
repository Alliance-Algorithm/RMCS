#pragma once

#include <libusb.h>

namespace librmcs::host::transport::usb::helper {

constexpr const char* libusb_errname(int number) {
    switch (number) {
    case LIBUSB_ERROR_IO: return "ERROR_IO";
    case LIBUSB_ERROR_INVALID_PARAM: return "ERROR_INVALID_PARAM";
    case LIBUSB_ERROR_ACCESS: return "ERROR_ACCESS";
    case LIBUSB_ERROR_NO_DEVICE: return "ERROR_NO_DEVICE";
    case LIBUSB_ERROR_NOT_FOUND: return "ERROR_NOT_FOUND";
    case LIBUSB_ERROR_BUSY: return "ERROR_BUSY";
    case LIBUSB_ERROR_TIMEOUT: return "ERROR_TIMEOUT";
    case LIBUSB_ERROR_OVERFLOW: return "ERROR_OVERFLOW";
    case LIBUSB_ERROR_PIPE: return "ERROR_PIPE";
    case LIBUSB_ERROR_INTERRUPTED: return "ERROR_INTERRUPTED";
    case LIBUSB_ERROR_NO_MEM: return "ERROR_NO_MEM";
    case LIBUSB_ERROR_NOT_SUPPORTED: return "ERROR_NOT_SUPPORTED";
    case LIBUSB_ERROR_OTHER: return "ERROR_OTHER";
    default: return "UNKNOWN";
    }
}

} // namespace librmcs::host::transport::usb::helper
