// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

void HandleSignal(int) { g_stop_requested = 1; }

}  // namespace

int main(int argc, char** argv) {
  constexpr int kExpectedArgumentCount = 7;
  const char* marker_path = std::getenv("MENTOR_PI_FAKE_AGENT_MARKER");
  const char* expected_device =
      std::getenv("MENTOR_PI_FAKE_AGENT_EXPECTED_DEVICE");
  if (marker_path == nullptr || expected_device == nullptr) {
    std::cerr << "fake Agent requires marker and expected-device variables\n";
    return 2;
  }

  const bool arguments_match =
      argc == kExpectedArgumentCount && std::string{argv[1]} == "serial" &&
      std::string{argv[2]} == "--dev" &&
      std::string{argv[3]} == expected_device &&
      std::string{argv[4]} == "--baudrate" &&
      std::string{argv[5]} == "1000000" && std::string{argv[6]} == "-v4";
  if (!arguments_match) {
    std::cerr << "fake Agent received an unexpected command line\n";
    return 3;
  }

  std::ofstream marker{marker_path, std::ios::out | std::ios::trunc};
  if (!marker.is_open()) {
    std::cerr << "fake Agent could not create its readiness marker\n";
    return 4;
  }
  marker << "READY\n";
  marker.close();

  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);
  while (g_stop_requested == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }
  return 0;
}
