// PCA9685 (I2C 16ch PWM) の簡易ドライバ。Linuxの /dev/i2c-N を直接使う (追加ライブラリ不要)。
// 動作確認はしていません。実機で、I2Cのバス番号とアドレスを確認してください
// (確認: `sudo i2cdetect -y -r <バス番号>`、i2c-tools が必要)。
#pragma once

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <thread>

namespace nodding_lidar
{

class Pca9685
{
public:
  // bus: /dev/i2c-<bus>, address: 7bitアドレス (通常 0x40)
  // osc_hz: 内部発振器の周波数 (25MHzが公称。実測でずれる場合は調整する)
  Pca9685(int bus, int address, double pwm_freq_hz, double osc_hz = 25.0e6)
  : freq_hz_(pwm_freq_hz)
  {
    const std::string dev = "/dev/i2c-" + std::to_string(bus);
    fd_ = ::open(dev.c_str(), O_RDWR);
    if (fd_ < 0) {
      throw std::runtime_error("cannot open " + dev + " (権限とバス番号を確認してください)");
    }
    if (::ioctl(fd_, I2C_SLAVE, address) < 0) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("cannot set I2C slave address");
    }

    // PWM周波数の設定は、スリープ中にしか書き込めない
    write_reg(kMode1, 0x10);  // SLEEP
    double prescale = std::round(osc_hz / (4096.0 * pwm_freq_hz)) - 1.0;
    prescale = std::clamp(prescale, 3.0, 255.0);
    write_reg(kPrescale, static_cast<uint8_t>(prescale));
    write_reg(kMode2, 0x04);  // OUTDRV (トーテムポール出力)
    write_reg(kMode1, 0x00);  // スリープ解除
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    write_reg(kMode1, 0xA0);  // RESTART + AI (レジスタ自動インクリメント)
  }

  ~Pca9685()
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  Pca9685(const Pca9685 &) = delete;
  Pca9685 & operator=(const Pca9685 &) = delete;

  // channel(0-15) のパルス幅を us で設定する
  void set_pulse_us(int channel, double pulse_us)
  {
    if (channel < 0 || channel > 15) {
      throw std::runtime_error("channel must be 0-15");
    }
    const double period_us = 1.0e6 / freq_hz_;
    double ticks = std::round(pulse_us / period_us * 4096.0);
    ticks = std::clamp(ticks, 0.0, 4095.0);
    const auto off = static_cast<uint16_t>(ticks);

    const uint8_t reg = static_cast<uint8_t>(kLed0OnL + 4 * channel);
    const uint8_t buf[5] = {
      reg, 0x00, 0x00, static_cast<uint8_t>(off & 0xFF), static_cast<uint8_t>(off >> 8)};
    if (::write(fd_, buf, sizeof(buf)) != static_cast<ssize_t>(sizeof(buf))) {
      throw std::runtime_error("I2C write failed");
    }
  }

private:
  static constexpr uint8_t kMode1 = 0x00;
  static constexpr uint8_t kMode2 = 0x01;
  static constexpr uint8_t kLed0OnL = 0x06;
  static constexpr uint8_t kPrescale = 0xFE;

  void write_reg(uint8_t reg, uint8_t value)
  {
    const uint8_t buf[2] = {reg, value};
    if (::write(fd_, buf, sizeof(buf)) != static_cast<ssize_t>(sizeof(buf))) {
      throw std::runtime_error("I2C write failed");
    }
  }

  int fd_{-1};
  double freq_hz_;
};

}  // namespace nodding_lidar
