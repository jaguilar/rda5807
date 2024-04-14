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

#include "absl/base/attributes.h"
#include "absl/base/optimization.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_format.h"

constexpr uint32_t addr_5807_random_access = 0x11;

namespace rda5807 {

enum class Register : uint8_t {
  kChipId = 0x00,
  kControl = 0x02,
  kChannel = 0x03,
  kVolumeAndSeek = 0x05,
};

enum class Band {
  kUs = 0,
  kJp = 1,
  kWorld = 2,
  kEastEurope = 3,
};

struct Reg0 {
  uint8_t chip_id;

  template <typename Sink>
  friend void AbslStringify(Sink& sink, const Reg0& reg0) {
    absl::Format(&sink, "chip_id: %02x", reg0.chip_id);
  }

  static Reg0 Parse(uint16_t x) {
    return Reg0{.chip_id = static_cast<uint8_t>((x >> 8) & 0xff)};
  }
};

struct Reg2 {
  bool enable = false;
  bool soft_reset = false;
  bool new_method = false;
  bool rds_enable = false;
  uint8_t clock_mode =
      0;  // See datasheet. My impl doesn't use a reference clock.
  bool seek_wrap = false;
  bool seek = false;
  bool seek_up = false;
  bool reference_clock_direct_input = false;
  bool reference_clock_noncalibrate = false;
  bool bass_boost = false;
  bool mono = false;
  bool disable_mute = false;
  bool dhiz = false;

  static Reg2 Parse(uint16_t x) {
    return {
        .enable = (x & (1 << 0)) != 0,
        .soft_reset = (x & (1 << 1)) != 0,
        .new_method = (x & (1 << 2)) != 0,
        .rds_enable = (x & (1 << 3)) != 0,
        .clock_mode = static_cast<uint8_t>((x >> 4) & 0b111),
        .seek_wrap = (x & (1 << 7)) == 0,
        .seek = (x & (1 << 8)) != 0,
        .seek_up = (x & (1 << 9)) != 0,
        .reference_clock_direct_input = (x & (1 << 10)) != 0,
        .reference_clock_noncalibrate = (x & (1 << 11)) != 0,
        .bass_boost = (x & (1 << 12)) != 0,
        .mono = (x & (1 << 13)) != 0,
        .disable_mute = (x & (1 << 14)) != 0,
        .dhiz = (x & (1 << 15)) != 0,
    };
  }

  uint16_t Serialize() const {
    uint16_t x = 0;
    x |= enable << 0;
    x |= soft_reset << 1;
    x |= new_method << 2;
    x |= rds_enable << 3;
    x |= clock_mode << 4;
    x |= !seek_wrap << 7;
    x |= seek << 8;
    x |= seek_up << 9;
    x |= reference_clock_direct_input << 10;
    x |= reference_clock_noncalibrate << 11;
    x |= bass_boost << 12;
    x |= mono << 13;
    x |= disable_mute << 14;
    x |= dhiz << 15;
    return x;
  }

  template <typename Sink>
  friend void AbslStringify(Sink& sink, const Reg2& reg2) {
    absl::Format(&sink,
                 "enable: %d, soft_reset: %d, new_method: %d, "
                 "rds_enable: %d, clock_mode: %d, seek_wrap: %d, seek: %d, "
                 "seek_up: %d, reference_clock_direct_input: %d, "
                 "reference_clock_noncalibrate: %d, bass_boost: %d, mono: "
                 "%d, disable_mute: %d, dhiz: %d",
                 reg2.enable, reg2.soft_reset, reg2.new_method, reg2.rds_enable,
                 reg2.clock_mode, reg2.seek_wrap, reg2.seek, reg2.seek_up,
                 reg2.reference_clock_direct_input,
                 reg2.reference_clock_noncalibrate, reg2.bass_boost, reg2.mono,
                 reg2.disable_mute, reg2.dhiz);
  }
};

struct Reg3 {
  enum ChannelSpacing {
    k100Khz = 0,
    k200Khz = 1,
    k50Khz = 2,
    k25Khz = 3,
  };
  ChannelSpacing spacing = k100Khz;

  static int64_t SpaceKhz(ChannelSpacing spacing) {
    switch (spacing) {
      case k100Khz:
        return 100;
      case k200Khz:
        return 200;
      case k50Khz:
        return 50;
      case k25Khz:
        return 25;
    }
    ABSL_UNREACHABLE();
  }
  int64_t SpaceKhz() const { return Reg3::SpaceKhz(spacing); }

  enum Band {
    kUs = 0,
    kJp = 1,
    kWorld = 2,
    kEastEurope = 3,
  };
  Band band = kUs;

  int64_t BandBaseKhz() const {
    switch (band) {
      case kUs:
        return 87000;
      case kJp:
        ABSL_FALLTHROUGH_INTENDED;
      case kWorld:
        return 76000;
      case 3:
        return 65000;
    }
    printf("Illegal band index: %d\n", static_cast<int>(band));
    exit(1);
  }

  bool tune = false;
  bool direct_mode = false;

  // band base + channel * spacing. Up to 9 bits.
  uint16_t channel = 0;

  static Reg3 Parse(uint16_t x) {
    Reg3 r{
        .spacing = static_cast<ChannelSpacing>(x & 0b11),
        .band = static_cast<Band>((x >> 2) & 0b11),
        .tune = (x & (1 << 4)) != 0,
        .direct_mode = (x & (1 << 5)) != 0,
        .channel = static_cast<uint16_t>(x >> 6),
    };
    CHECK_LE(r.channel, 0b111111111);
    return r;
  }

  uint16_t Serialize() const {
    uint16_t x = 0;
    x |= spacing;
    x |= band << 2;
    x |= tune << 4;
    x |= direct_mode << 5;
    x |= channel << 6;
    return x;
  }

  int64_t KHz() const { return BandBaseKhz() + channel * SpaceKhz(); }

  template <typename T>
  friend void AbslStringify(T& sink, const Reg3& reg3) {
    absl::Format(&sink,
                 "spacing: %d (%dkHz), band: %d, tune: %d, direct_mode: %d, "
                 "channel: %d (%d kHz)",
                 reg3.spacing, reg3.SpaceKhz(), reg3.band, reg3.tune,
                 reg3.direct_mode, reg3.channel, reg3.KHz());
  }

  void SetChannel(Band band, uint64_t khz) {
    khz -= BandBaseKhz();
    printf("khzoffset: %lu\n", khz);

    // Use the widest spacing that exactly matches the target frequency,
    // else 25.
    ChannelSpacing chosen_spacing;
    for (ChannelSpacing i : {k200Khz, k100Khz, k50Khz, k25Khz}) {
      chosen_spacing = i;
      if (!(khz % SpaceKhz(i))) break;
    }
    const int64_t chosen_spacing_khz = SpaceKhz(chosen_spacing);
    absl::PrintF("space_khz: %d [%d kHz]\n", chosen_spacing,
                 chosen_spacing_khz);

    khz += chosen_spacing_khz / 2;  // Round rather than truncate.
    uint16_t channel = khz / chosen_spacing_khz;
    absl::PrintF("chan: %d\n", channel);

    // Construct the new tune value.
    this->spacing = chosen_spacing;
    this->channel = channel;
  }
};

struct Reg5 {
  uint8_t volume = 0b1111;

  static Reg5 Parse(uint16_t x) {
    return {.volume = static_cast<uint8_t>(x & 0b1111)};
  }

  uint16_t Serialize() const {
    CHECK_LE(volume, 0b1111);
    return volume & 0b1111;
  }

  template <typename T>
  friend void AbslStringify(T& sink, const Reg5& reg5) {
    absl::Format(&sink, "volume: %d", reg5.volume);
  }
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

  uint16_t ReadRegister(Register r) const {
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

 private:
  int fd_;
};

void DumpRegisters(const I2CConnection& conn) {
  Reg0 reg0 = Reg0::Parse(conn.ReadRegister(Register::kChipId));
  LOG(INFO) << reg0;
  Reg2 reg2 = Reg2::Parse(conn.ReadRegister(Register::kControl));
  LOG(INFO) << reg2;
  Reg3 reg3 = Reg3::Parse(conn.ReadRegister(Register::kChannel));
  LOG(INFO) << reg3;
  Reg5 reg5 = Reg5::Parse(conn.ReadRegister(Register::kVolumeAndSeek));
  LOG(INFO) << reg5;
}

void main() {
  auto conn = I2CConnection::Create("/dev/i2c-1");
  DumpRegisters(conn);

  Reg2 reg2 = Reg2::Parse(conn.ReadRegister(Register::kControl));
  reg2.seek = false;
  reg2.seek_wrap = false;
  reg2.reference_clock_direct_input = false;
  reg2.disable_mute = true;
  conn.SetRegister(Register::kControl, reg2.Serialize());
  LOG(INFO) << "After adjusting control: "
            << Reg2::Parse(conn.ReadRegister(Register::kControl));

  Reg5 reg5 = Reg5::Parse(conn.ReadRegister(Register::kVolumeAndSeek));
  reg5.volume = 15;
  conn.SetRegister(Register::kVolumeAndSeek, reg5.Serialize());
  LOG(INFO) << Reg5::Parse(conn.ReadRegister(Register::kVolumeAndSeek));

  Reg3 reg3 = Reg3::Parse(conn.ReadRegister(Register::kChannel));
  reg3.tune = true;
  reg3.direct_mode = true;
  conn.SetRegister(Register::kChannel, reg3.Serialize());
  LOG(INFO) << "After adjusting channel: "
            << Reg3::Parse(conn.ReadRegister(Register::kChannel));
  reg3.SetChannel(Reg3::Band::kUs, 91100);
  conn.SetRegister(Register::kChannel, reg3.Serialize());
  LOG(INFO) << "Sending: " << reg3;
  LOG(INFO) << "After adjusting channel: "
            << Reg3::Parse(conn.ReadRegister(Register::kChannel));
}

}  // namespace rda5807

int main(int, char **) {
  rda5807::main();
  return 0;
}
