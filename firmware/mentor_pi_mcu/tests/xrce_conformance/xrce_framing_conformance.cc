// Copyright 2026 Mentor Pi contributors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <uxr/client/profile/transport/custom/custom_transport.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

namespace {

constexpr std::size_t kTransportCapacity = 4096U;
constexpr std::size_t kMtu = UXR_CONFIG_CUSTOM_TRANSPORT_MTU;
constexpr std::size_t kMaxRecoveryAttempts = 4U;

static_assert(kMtu == 512U,
              "The native conformance client must match the firmware MTU");

[[noreturn]] void Fail() { std::abort(); }

void Require(bool condition) {
  if (!condition) {
    Fail();
  }
}

struct FixedPipe {
  std::array<std::uint8_t, kTransportCapacity> bytes{};
  std::size_t read_position = 0U;
  std::size_t write_position = 0U;

  std::size_t Write(const std::uint8_t* source, std::size_t length,
                    std::size_t chunk_limit) {
    const std::size_t available = bytes.size() - write_position;
    const std::size_t count =
        std::min(std::min(length, available), chunk_limit);
    for (std::size_t index = 0U; index < count; ++index) {
      bytes[write_position + index] = source[index];
    }
    write_position += count;
    return count;
  }

  std::size_t Read(std::uint8_t* destination, std::size_t length,
                   std::size_t chunk_limit) {
    const std::size_t available = write_position - read_position;
    const std::size_t count =
        std::min(std::min(length, available), chunk_limit);
    for (std::size_t index = 0U; index < count; ++index) {
      destination[index] = bytes[read_position + index];
    }
    read_position += count;
    return count;
  }
};

struct TransportFixture {
  uxrCustomTransport transport{};
  FixedPipe pipe{};
  std::size_t read_chunk = kTransportCapacity;
  std::size_t write_chunk = kTransportCapacity;
  std::size_t open_count = 0U;
  std::size_t close_count = 0U;
  std::size_t read_count = 0U;
};

bool Open(uxrCustomTransport* transport) {
  auto* fixture = static_cast<TransportFixture*>(transport->args);
  ++fixture->open_count;
  return true;
}

bool Close(uxrCustomTransport* transport) {
  auto* fixture = static_cast<TransportFixture*>(transport->args);
  ++fixture->close_count;
  return true;
}

std::size_t Write(uxrCustomTransport* transport, const std::uint8_t* buffer,
                  std::size_t length, std::uint8_t* error_code) {
  auto* fixture = static_cast<TransportFixture*>(transport->args);
  *error_code = 0U;
  return fixture->pipe.Write(buffer, length, fixture->write_chunk);
}

std::size_t Read(uxrCustomTransport* transport, std::uint8_t* buffer,
                 std::size_t length, int /*timeout*/,
                 std::uint8_t* error_code) {
  auto* fixture = static_cast<TransportFixture*>(transport->args);
  ++fixture->read_count;
  *error_code = 0U;
  return fixture->pipe.Read(buffer, length, fixture->read_chunk);
}

void Initialize(TransportFixture* fixture) {
  uxr_set_custom_transport_callbacks(&fixture->transport, true, Open, Close,
                                     Write, Read);
  Require(uxr_init_custom_transport(&fixture->transport, fixture));
  Require(fixture->open_count == 1U);
}

void CloseAndCheck(TransportFixture* fixture) {
  Require(uxr_close_custom_transport(&fixture->transport));
  Require(fixture->close_count == 1U);
}

template <std::size_t Size>
void RoundTrip(const std::array<std::uint8_t, Size>& payload,
               std::size_t read_chunk, std::size_t write_chunk) {
  TransportFixture fixture{};
  fixture.read_chunk = read_chunk;
  fixture.write_chunk = write_chunk;
  Initialize(&fixture);

  Require(fixture.transport.comm.send_msg(&fixture.transport, payload.data(),
                                          payload.size()));
  std::uint8_t* received = nullptr;
  std::size_t received_size = 0U;
  Require(fixture.transport.comm.recv_msg(&fixture.transport, &received,
                                          &received_size, 100));
  Require(received != nullptr);
  Require(received_size == payload.size());
  for (std::size_t index = 0U; index < payload.size(); ++index) {
    Require(received[index] == payload[index]);
  }

  CloseAndCheck(&fixture);
}

void TestRepresentativeXrcePayloadAtEveryReadSplit() {
  // A deterministic XRCE-shaped payload containing both stream-framing
  // reserved octets. Framing is generated and consumed solely by the pinned
  // client's documented custom-transport API.
  constexpr std::array<std::uint8_t, 24U> kPayload = {
      0x81U, 0x80U, 0x01U, 0x01U, 0x0FU, 0x01U, 0x00U, 0x10U,
      0x00U, 0x01U, 0x02U, 0x03U, 0x7EU, 0x7DU, 0x20U, 0x00U,
      0xFFU, 0x55U, 0xAAU, 0x11U, 0x22U, 0x33U, 0x44U, 0x00U,
  };

  for (std::size_t split = 1U; split <= kPayload.size() + 8U; ++split) {
    RoundTrip(kPayload, split, kTransportCapacity);
  }
}

void TestReservedOctetStuffingAtEveryWriteSplit() {
  std::array<std::uint8_t, kMtu> payload{};
  for (std::size_t index = 0U; index < payload.size(); ++index) {
    payload[index] =
        (index % 2U == 0U) ? UXR_FRAMING_BEGIN_FLAG : UXR_FRAMING_ESC_FLAG;
  }

  for (std::size_t split = 1U; split <= 42U; ++split) {
    RoundTrip(payload, 1U + (split % 17U), split);
  }
}

struct WireFrame {
  std::array<std::uint8_t, kTransportCapacity> bytes{};
  std::size_t size = 0U;
};

template <std::size_t Size>
WireFrame EncodeFrame(const std::array<std::uint8_t, Size>& payload) {
  TransportFixture fixture{};
  Initialize(&fixture);
  Require(fixture.transport.comm.send_msg(&fixture.transport, payload.data(),
                                          payload.size()));

  WireFrame frame{};
  frame.size = fixture.pipe.write_position;
  Require(frame.size <= frame.bytes.size());
  for (std::size_t index = 0U; index < frame.size; ++index) {
    frame.bytes[index] = fixture.pipe.bytes[index];
  }
  CloseAndCheck(&fixture);
  return frame;
}

void AppendBytes(FixedPipe* pipe, const std::uint8_t* bytes, std::size_t size) {
  Require(pipe->Write(bytes, size, kTransportCapacity) == size);
}

enum class RecoveryInput {
  kTruncated,
  kChecksumCorrupted,
  kDuplicatedTruncated,
};

struct RecoveryCase {
  RecoveryInput input;
  std::size_t read_chunk;
  std::size_t probe_frame_count;
  std::size_t expected_probe_count;
  bool requires_idle_delimiter_boundary;
};

void AppendMalformedFrame(FixedPipe* pipe, const WireFrame& frame,
                          RecoveryInput input) {
  Require(frame.size > 2U);
  const std::size_t truncated_size = frame.size - 1U;

  switch (input) {
    case RecoveryInput::kTruncated:
      AppendBytes(pipe, frame.bytes.data(), truncated_size);
      break;
    case RecoveryInput::kChecksumCorrupted: {
      std::array<std::uint8_t, kTransportCapacity> corrupted = frame.bytes;
      corrupted[frame.size - 1U] ^= 0x01U;
      AppendBytes(pipe, corrupted.data(), frame.size);
      break;
    }
    case RecoveryInput::kDuplicatedTruncated:
      AppendBytes(pipe, frame.bytes.data(), truncated_size);
      AppendBytes(pipe, frame.bytes.data(), truncated_size);
      break;
  }
}

template <std::size_t Size>
bool PayloadMatches(const std::uint8_t* received, std::size_t received_size,
                    const std::array<std::uint8_t, Size>& expected) {
  if (received == nullptr || received_size != expected.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < expected.size(); ++index) {
    if (received[index] != expected[index]) {
      return false;
    }
  }
  return true;
}

void TestMalformedFrameRejectionAndConditionalRecovery() {
  constexpr std::array<std::uint8_t, 16U> kRejectedPayload = {
      0x81U, 0x80U, 0x01U, 0x00U, 0x09U, 0x01U, 0x08U, 0x00U,
      0x7EU, 0x7DU, 0x10U, 0x20U, 0x30U, 0x40U, 0x50U, 0x60U,
  };
  constexpr std::array<std::uint8_t, 12U> kProbePayload = {
      0x81U, 0x80U, 0x02U, 0x00U, 0x09U, 0x01U,
      0x04U, 0x00U, 0xA5U, 0x11U, 0x7EU, 0x7DU,
  };
  constexpr std::array<std::uint8_t, 12U> kRecoveryPayload = {
      0x81U, 0x80U, 0x03U, 0x00U, 0x09U, 0x01U,
      0x04U, 0x00U, 0xC3U, 0x5AU, 0x33U, 0xCCU,
  };
  // The first two cases keep three valid frames continuously available after
  // the truncated CRC: the immediate frame is lost and the next two are
  // returned for this exact bulk/one-byte schedule. The next two isolate the
  // conditional idle-delimiter recovery point after that immediate loss.
  constexpr std::array<RecoveryCase, 7U> kCases = {{
      {RecoveryInput::kTruncated, kTransportCapacity, 3U, 2U, false},
      {RecoveryInput::kTruncated, 1U, 3U, 2U, false},
      {RecoveryInput::kTruncated, kTransportCapacity, 1U, 0U, true},
      {RecoveryInput::kTruncated, 1U, 1U, 0U, true},
      {RecoveryInput::kChecksumCorrupted, kTransportCapacity, 1U, 1U, false},
      {RecoveryInput::kChecksumCorrupted, 1U, 1U, 1U, false},
      {RecoveryInput::kDuplicatedTruncated, 7U, 1U, 1U, false},
  }};

  const WireFrame malformed_frame = EncodeFrame(kRejectedPayload);
  const WireFrame probe_frame = EncodeFrame(kProbePayload);
  const WireFrame recovery_frame = EncodeFrame(kRecoveryPayload);

  for (const RecoveryCase& test_case : kCases) {
    TransportFixture fixture{};
    fixture.read_chunk = test_case.read_chunk;
    Initialize(&fixture);

    AppendMalformedFrame(&fixture.pipe, malformed_frame, test_case.input);
    for (std::size_t index = 0U; index < test_case.probe_frame_count; ++index) {
      AppendBytes(&fixture.pipe, probe_frame.bytes.data(), probe_frame.size);
    }

    std::size_t observed_probe_count = 0U;
    std::size_t receive_calls = 0U;
    const std::size_t phase_one_receive_budget =
        fixture.pipe.write_position + kMaxRecoveryAttempts;
    for (std::size_t attempt = 0U; attempt < phase_one_receive_budget;
         ++attempt) {
      std::uint8_t* received = nullptr;
      std::size_t received_size = 0U;
      ++receive_calls;
      if (fixture.transport.comm.recv_msg(&fixture.transport, &received,
                                          &received_size, 0)) {
        Require(PayloadMatches(received, received_size, kProbePayload));
        ++observed_probe_count;
        Require(observed_probe_count <= test_case.expected_probe_count);
      }
      if (fixture.pipe.read_position == fixture.pipe.write_position &&
          fixture.transport.framing_io.rb_head ==
              fixture.transport.framing_io.rb_tail) {
        break;
      }
    }
    Require(fixture.pipe.read_position == fixture.pipe.write_position);
    Require(fixture.transport.framing_io.rb_head ==
            fixture.transport.framing_io.rb_tail);
    Require(observed_probe_count == test_case.expected_probe_count);

    if (test_case.requires_idle_delimiter_boundary) {
      // Pinned client 2.4.2 consumes a following frame delimiter while waiting
      // for the missing CRC octet. A zero-byte transport read immediately after
      // a later delimiter is the exact conditional recovery point tested here.
      Require(recovery_frame.bytes[0] == UXR_FRAMING_BEGIN_FLAG);
      AppendBytes(&fixture.pipe, recovery_frame.bytes.data(), 1U);
      std::uint8_t* received = nullptr;
      std::size_t received_size = 0U;
      ++receive_calls;
      Require(!fixture.transport.comm.recv_msg(&fixture.transport, &received,
                                               &received_size, 0));
      AppendBytes(&fixture.pipe, recovery_frame.bytes.data() + 1U,
                  recovery_frame.size - 1U);
    } else {
      AppendBytes(&fixture.pipe, recovery_frame.bytes.data(),
                  recovery_frame.size);
    }

    bool recovered = false;
    for (std::size_t attempt = 0U; attempt < kMaxRecoveryAttempts; ++attempt) {
      std::uint8_t* received = nullptr;
      std::size_t received_size = 0U;
      ++receive_calls;
      if (fixture.transport.comm.recv_msg(&fixture.transport, &received,
                                          &received_size, 0)) {
        Require(PayloadMatches(received, received_size, kRecoveryPayload));
        recovered = true;
        break;
      }
    }

    Require(recovered);
    // With timeout zero, each successful raw read consumes at least one wire
    // byte. Permit one final empty read per public receive call, but no polling
    // beyond that deterministic budget.
    Require(fixture.read_count <= fixture.pipe.write_position + receive_calls);
    CloseAndCheck(&fixture);
  }
}

}  // namespace

int main() {
  TestRepresentativeXrcePayloadAtEveryReadSplit();
  TestReservedOctetStuffingAtEveryWriteSplit();
  TestMalformedFrameRejectionAndConditionalRecovery();
  return 0;
}
