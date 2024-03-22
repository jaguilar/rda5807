#include <fcntl.h>
#include <libexplain/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>  // Include the header file for ioctl
#include <unistd.h>

#include <array>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string_view>

constexpr uint32_t addr_5807_random_access = 0x11;

namespace rda5807 {

enum class Register : uint8_t {
  kChipId = 0x00,
  kControl = 0x02,
  kChannel = 0x03,
  kThreshold = 0x05,
};

enum class Band {
  kUs = 0,
  kJp = 1,
  kWorld = 2,
  kEastEurope = 3,
};

class I2CConnection {
 public:
  static I2CConnection Create(std::string_view device_path) {
    I2CConnection connection;
    connection.fd_ = open(device_path.data(), O_RDWR);
    if (connection.fd_ < 0) {
      throw std::runtime_error("Failed to open I2C device");
    }
    return connection;
  }

  ~I2CConnection() { close(fd_); }

  uint16_t ReadRegister(Register r) {
    uint8_t buf[1] = {static_cast<uint8_t>(r)};
    uint8_t res[2] = {0, 0};
    std::array<i2c_msg, 2> msgs{i2c_msg{
                                    .addr = addr_5807_random_access,
                                    .flags = 0,
                                    .len = 1,
                                    .buf = buf,
                                },
                                i2c_msg{
                                    .addr = addr_5807_random_access,
                                    .flags = I2C_M_RD,
                                    .len = 2,
                                    .buf = res,
                                }};
    i2c_rdwr_ioctl_data data = {
        .msgs = msgs.data(),
        .nmsgs = msgs.size(),
    };
    if (int err = ioctl(fd_, I2C_RDWR, &data); err < 0) {
      printf("failed to read i2c %d with detail %s\n", err,
             explain_ioctl(fd_, I2C_RDWR, &data));
      exit(1);
    }
    // Note: results from the device are big-endian.
    return (static_cast<uint16_t>(res[0]) << 8) | res[1];
  }

  void SetRegister(Register r, uint16_t val) {
    uint8_t buf[3] = {static_cast<uint8_t>(r), static_cast<uint8_t>(val >> 8),
                      static_cast<uint8_t>(val)};
    std::array<i2c_msg, 1> msgs{i2c_msg{
        .addr = addr_5807_random_access,
        .flags = 0,
        .len = 3,
        .buf = buf,
    }};
    i2c_rdwr_ioctl_data data = {
        .msgs = msgs.data(),
        .nmsgs = msgs.size(),
    };
    if (int err = ioctl(fd_, I2C_RDWR, &data); err < 0) {
      printf("failed to write i2c %d with detail %s\n", err,
             explain_ioctl(fd_, I2C_RDWR, &data));
      exit(1);
    }
  }

  void Reset() {
    SetRegister(Register::kControl, 0b10);  // Reset.
  }

  void Init() {
    uint16_t val = 0;
    val |= 1 << 13;  // mono
    val |= 1 << 11;  // rclk is not wired on this board.
    val |= 1 << 2;   // new demodulate method
    val |= 1 << 0;   // enable.
    SetRegister(Register::kControl, val);
  }

  void SetMute(bool mute) {
    constexpr int mutebit = 14;
    const uint16_t muteval = (mute ? 0 : 1) << mutebit;
    constexpr uint16_t mutemask = ~(1 << mutebit);

    uint16_t val = ReadRegister(Register::kControl);
    val = (val & mutemask) | muteval;
    SetRegister(Register::kControl, val);
  }

  // Note: maximum volume is 15 (1111), volumes above 15 will be truncated.
  // Volume is logarithmic.
  void SetVolume(uint8_t vol) {
    constexpr uint16_t volume_mask = 0xfff0;
    uint16_t val = ReadRegister(Register::kThreshold);
    val = (val & volume_mask) | (vol & 0b1111);
    SetRegister(Register::kThreshold, val);
  }

  void Seek() {
    constexpr int seekbit = 8;
    constexpr int seekupbit = 9;
    constexpr uint16_t seekmask = ~(1 << seekbit | 1 << seekupbit);
    const uint16_t seekval = 1 << 8 | 1 << 9;
    uint16_t val = ReadRegister(Register::kControl);
    val |= seekval;
    SetRegister(Register::kControl, val);
  }

  // Reports the current channel in kilohertz.
  uint64_t GetChannel() {
    const uint16_t val = ReadRegister(Register::kChannel);

    const uint16_t chan = val >> 6;
    const uint64_t base = BandBaseKhz(Band((val & 0b1100) >> 2));
    const uint64_t space = SpaceKhz(val & 0b0011);

    return static_cast<uint64_t>(base) + space * chan;
  }

  void SetChannel(Band band, uint64_t khz) {
    khz -= BandBaseKhz(band);
    printf("khzoffset: %lu\n", khz);

    // Use the widest spacing that exactly matches the target frequency,
    // else 25.
    uint8_t space_idx = 5;
    uint64_t space_khz = 0;
    for (int i : {1, 0, 2, 3}) {
      uint64_t i_khz = SpaceKhz(i);
      space_idx = i;
      space_khz = i_khz;
      if (!(khz % i_khz)) break;
    }
    printf("space_khz: %lu [%d]\n", space_khz, space_idx);

    khz += space_khz / 2;  // Round rather than truncate.
    khz /= space_khz;
    printf("chan: %lu\n", khz);

    // Construct the new tune value.
    uint16_t val =
        (khz << 6) | (1 << 4) | (static_cast<uint8_t>(band) << 2) | space_idx;
    printf("setchannel %04x\n", val);
    SetRegister(Register::kChannel, val);
  }

  static uint64_t SpaceKhz(uint8_t space_idx) {
    switch (space_idx) {
      case 0:
        return 100;
      case 1:
        return 200;
      case 2:
        return 50;
      case 3:
        return 25;
    }
    printf("Illegal space index: %d\n", space_idx);
    exit(1);
  }

  static uint64_t BandBaseKhz(Band band) {
    switch (static_cast<int>(band)) {
      case 0:
        return 87000;
      case 1:
      case 2:
        return 76000;
      case 3:
        return 65000;
    }
    printf("Illegal band index: %d\n", static_cast<int>(band));
    exit(1);
  }

  void Tune() {}

 private:
  int fd_;
};

void main() {
  auto conn = I2CConnection::Create("/dev/i2c-1");
  // conn.Reset();
  // conn.Init();

  printf("chipid: %04x\n", conn.ReadRegister(Register::kChipId));
  printf("control: %04x\n", conn.ReadRegister(Register::kControl));
  printf("channel: %04x\n", conn.ReadRegister(Register::kChannel));
  printf("threshold: %04x\n", conn.ReadRegister(Register::kThreshold));

  conn.SetVolume(10);
  printf("threshold: %04x\n", conn.ReadRegister(Register::kThreshold));
  conn.Seek();
  printf("control: %04x\n", conn.ReadRegister(Register::kControl));

  printf("channel: %ld\n", conn.GetChannel());
  conn.SetChannel(Band::kUs, 102700);
  printf("channel: %ld\n", conn.GetChannel());
}

}  // namespace rda5807

int main(int, char **) {
  rda5807::main();
  return 0;
}
