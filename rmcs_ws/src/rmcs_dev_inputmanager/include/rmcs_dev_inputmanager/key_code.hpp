
#pragma once

namespace test_utils::input_manager {
class KeyCode {
public:
  enum class KeyCodeEnum : long long {
    None = 0,
    A = 1,
    B = A << 1,
    C = B << 1,
    D = C << 1,
    E = D << 1,
    F = E << 1,
    G = F << 1,
    H = G << 1,
    I = H << 1,
    J = I << 1,
    K = J << 1,
    L = K << 1,
    M = L << 1,
    N = M << 1,
    O = N << 1,
    P = O << 1,
    Q = P << 1,
    R = Q << 1,
    S = R << 1,
    T = S << 1,
    U = T << 1,
    V = U << 1,
    W = V << 1,
    X = W << 1,
    Y = X << 1,
    Z = Y << 1,
    Shift = Z << 1,
    Ctrl = Shift << 1,
    Alt = Ctrl << 1,
  };

public:
  explicit KeyCode(long long) = delete;
  explicit KeyCode(int) = delete;
  explicit KeyCode(char) = delete;
  explicit KeyCode(unsigned char data) {
    if (data < control_offset)
      data_ = KeyCodeEnum::None;
    else if (data < control_offset + 3)
      data_ = KeyCodeEnum(1 << (data - control_offset + 26));
    else if (data < alpha_beta_offset)
      data_ = KeyCodeEnum::None;
    else if (data < alpha_beta_offset + 26)
      data_ = KeyCodeEnum(1 << (data - alpha_beta_offset + 26));
    else
      data_ = KeyCodeEnum::None;
  };
  explicit KeyCode(KeyCodeEnum &&data) { data_ = data; };

  KeyCode operator+(const KeyCode &data) {
    return KeyCode(static_cast<KeyCodeEnum>(
        static_cast<long long>(data_) | static_cast<long long>(data.data_)));
  }
  bool operator==(const KeyCodeEnum &data) {
    return (static_cast<long long>(data_) & static_cast<long long>(data)) == 0;
  }

  KeyCode() : data_{KeyCodeEnum::None} {};

private:
  KeyCodeEnum data_;
  const static unsigned char alpha_beta_offset = 64;
  const static unsigned char control_offset = 15;
};
} // namespace test_utils::input_manager