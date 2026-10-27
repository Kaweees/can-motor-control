#include <array>
#include <cstdint>
#include <cstring>
#include <dlfcn.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

// ABI declarations verified against this image's DWARF types. Only pure
// conversion routines and the data-only Motor constructor are invoked.
struct Limit { double p, v, t; };
struct Mit { double kp, kd, q, dq, tau; };
struct State { int id, err; uint16_t raw_q, raw_dq, raw_tau; double q, dq, tau; int mos, rotor; bool valid; };
static_assert(sizeof(State) == 56);

template<class T> T symbol(void* library, const char* name) {
  auto p = dlsym(library, name);
  if (!p) throw std::runtime_error(dlerror());
  return reinterpret_cast<T>(p);
}

int main() {
  void* lib = dlopen("/workspace/install/control/lib/libcontrol.so", RTLD_NOW);
  if (!lib) throw std::runtime_error(dlerror());
  auto limits = symbol<const Limit*>(lib, "_ZN7openarm12damiao_motor18MOTOR_LIMIT_PARAMSE");
  auto pack = symbol<std::vector<uint8_t>(*)(uint8_t, const Mit&)>(lib,
      "_ZN7openarm12damiao_motor16CanPacketEncoder21pack_mit_control_dataENS0_9MotorTypeERKNS0_8MITParamE");
  auto construct = symbol<void(*)(void*, uint8_t, uint32_t, uint32_t)>(lib,
      "_ZN7openarm12damiao_motor5MotorC1ENS0_9MotorTypeEjj");
  auto decode = symbol<State(*)(const void*, const std::vector<uint8_t>&)>(lib,
      "_ZN7openarm12damiao_motor16CanPacketDecoder22parse_motor_state_dataERKNS0_5MotorERKSt6vectorIhSaIhEE");
  std::cout << std::unitbuf << std::setprecision(17);
  for (uint8_t type = 0; type < 13; ++type) {
    const auto lim = limits[type];
    std::cout << "limits," << unsigned(type) << ',' << lim.p << ',' << lim.v << ',' << lim.t << '\n';
    for (const Mit& command : std::array<Mit, 8>{{
        {0, 0, 0, 0, 0},
        {40, 3, 0.55, 0.2, 0.3},
        {65, 4.5, -1.2, -0.3, -0.7},
        {500, 5, lim.p, lim.v, lim.t},
        {600, 6, -2 * lim.p, 2 * lim.v, -2 * lim.t},
        {0.1, 0.04, -1.429, 0, 0.8},
        {0.1, 0.04, -2.858, 0, 0},
        {0.1, 0.04, 0.17148, 0, -0.8}}}) {
      const auto bytes = pack(type, command);
      std::cout << "mit," << unsigned(type) << ',' << command.kp << ',' << command.kd
                << ',' << command.q << ',' << command.dq << ',' << command.tau << ',';
      for (uint8_t b : bytes) std::cout << std::hex << std::setw(2) << std::setfill('0') << unsigned(b);
      std::cout << std::dec << '\n';
    }
    alignas(16) std::array<unsigned char, 512> motor{};
    construct(motor.data(), type, 1, 0x11);
    for (const std::vector<uint8_t>& payload : {
        std::vector<uint8_t>{0x11, 0x91, 0x33, 0x90, 0x09, 0x00, 30, 35},
        std::vector<uint8_t>{0x11, 0x91, 0x55, 0x90, 0x09, 0x00, 30, 35},
        std::vector<uint8_t>{0x11, 0xff, 0xff, 0xff, 0xff, 0xff, 30, 35}}) {
      const auto state = decode(motor.data(), payload);
      std::cout << "state," << unsigned(type) << ',';
      for (uint8_t b : payload) std::cout << std::hex << std::setw(2) << std::setfill('0') << unsigned(b);
      std::cout << std::dec << ',' << state.q << ',' << state.dq << ',' << state.tau
                << ',' << state.mos << ',' << state.rotor << ',' << state.valid << '\n';
    }
  }
}
