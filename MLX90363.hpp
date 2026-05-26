#pragma once

// Header-only C++17 driver for the Melexis MLX90363 magnetic-rotary sensor
// over hardware SPI on an AVR. Templated on the SS / SCLK / MOSI / MISO pin
// types so the slave-select line can be any IOpin while the SPI bus pins
// default to the chip's hardware ones (atmega32u4 PB0/1/2/3).
//
// Usage outline:
//
//   #include <MLX90363.hpp>
//   using namespace AVR;
//   using MyMLX = MLX90363<IOpin<Ports::B, 7>>;  // SS = PB7
//   ISR(SPI_STC_vect) { MyMLX::isr(); }
//
//   int main() {
//     MyMLX::init();
//     sei();
//     while (true) {
//       MyMLX::prepareGET1Message(MyMLX::MessageType::Alpha);
//       MyMLX::startTransmitting();
//       _delay_us(MyMLX::tMeasurementAlpha_us);  // wait for measurement
//       auto alpha = MyMLX::getAlpha();
//       (void)alpha;
//     }
//   }
//
// The library does NOT install the SPI interrupt vector — the user code must
// define `ISR(SPI_STC_vect)` and dispatch to `::isr()`. This avoids name
// collisions across multiple instantiations and makes the dispatch explicit.
//
// No `Clock` dependency: callers are responsible for waiting between
// transmissions (the chip needs ~1 ms after a measurement command before the
// result is valid). Datasheet timings are exposed as `constexpr u4 tXxx_us`
// constants for convenience.

#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <AVR++/Atomic.hpp>
#include <AVR++/IOpin.hpp>
#include <AVR++/SPI.hpp>
#include <AVR++/basicTypes.hpp>
#include <AVR++/bitTypes.hpp>

namespace MLX90363Lib {

using namespace Basic;
using AVR::Atomic;

namespace detail {
// CRC lookup table from the MLX90363 datasheet (cba_256_TAB). Stored in
// program memory so it doesn't sit in 256 bytes of SRAM. The table is
// `inline` (C++17) so each TU that includes this header references the
// same flash copy.
inline const u1 cba_256_TAB[] PROGMEM = {
    0x00, 0x2f, 0x5e, 0x71, 0xbc, 0x93, 0xe2, 0xcd, 0x57, 0x78, 0x09, 0x26, 0xeb, 0xc4, 0xb5, 0x9a, 0xae, 0x81, 0xf0,
    0xdf, 0x12, 0x3d, 0x4c, 0x63, 0xf9, 0xd6, 0xa7, 0x88, 0x45, 0x6a, 0x1b, 0x34, 0x73, 0x5c, 0x2d, 0x02, 0xcf, 0xe0,
    0x91, 0xbe, 0x24, 0x0b, 0x7a, 0x55, 0x98, 0xb7, 0xc6, 0xe9, 0xdd, 0xf2, 0x83, 0xac, 0x61, 0x4e, 0x3f, 0x10, 0x8a,
    0xa5, 0xd4, 0xfb, 0x36, 0x19, 0x68, 0x47, 0xe6, 0xc9, 0xb8, 0x97, 0x5a, 0x75, 0x04, 0x2b, 0xb1, 0x9e, 0xef, 0xc0,
    0x0d, 0x22, 0x53, 0x7c, 0x48, 0x67, 0x16, 0x39, 0xf4, 0xdb, 0xaa, 0x85, 0x1f, 0x30, 0x41, 0x6e, 0xa3, 0x8c, 0xfd,
    0xd2, 0x95, 0xba, 0xcb, 0xe4, 0x29, 0x06, 0x77, 0x58, 0xc2, 0xed, 0x9c, 0xb3, 0x7e, 0x51, 0x20, 0x0f, 0x3b, 0x14,
    0x65, 0x4a, 0x87, 0xa8, 0xd9, 0xf6, 0x6c, 0x43, 0x32, 0x1d, 0xd0, 0xff, 0x8e, 0xa1, 0xe3, 0xcc, 0xbd, 0x92, 0x5f,
    0x70, 0x01, 0x2e, 0xb4, 0x9b, 0xea, 0xc5, 0x08, 0x27, 0x56, 0x79, 0x4d, 0x62, 0x13, 0x3c, 0xf1, 0xde, 0xaf, 0x80,
    0x1a, 0x35, 0x44, 0x6b, 0xa6, 0x89, 0xf8, 0xd7, 0x90, 0xbf, 0xce, 0xe1, 0x2c, 0x03, 0x72, 0x5d, 0xc7, 0xe8, 0x99,
    0xb6, 0x7b, 0x54, 0x25, 0x0a, 0x3e, 0x11, 0x60, 0x4f, 0x82, 0xad, 0xdc, 0xf3, 0x69, 0x46, 0x37, 0x18, 0xd5, 0xfa,
    0x8b, 0xa4, 0x05, 0x2a, 0x5b, 0x74, 0xb9, 0x96, 0xe7, 0xc8, 0x52, 0x7d, 0x0c, 0x23, 0xee, 0xc1, 0xb0, 0x9f, 0xab,
    0x84, 0xf5, 0xda, 0x17, 0x38, 0x49, 0x66, 0xfc, 0xd3, 0xa2, 0x8d, 0x40, 0x6f, 0x1e, 0x31, 0x76, 0x59, 0x28, 0x07,
    0xca, 0xe5, 0x94, 0xbb, 0x21, 0x0e, 0x7f, 0x50, 0x9d, 0xb2, 0xc3, 0xec, 0xd8, 0xf7, 0x86, 0xa9, 0x64, 0x4b, 0x3a,
    0x15, 0x8f, 0xa0, 0xd1, 0xfe, 0x33, 0x1c, 0x6d, 0x42,
};

inline u1 lookupCRC(u1 b) { return pgm_read_byte(&cba_256_TAB[b]); }
} // namespace detail

/**
 * MLX90363 SPI driver.
 *
 * @tparam SS    Slave-select pin type (e.g. `AVR::IOpin<AVR::Ports::B, 7>`).
 *               Active-low; library drives it HIGH when idle.
 * @tparam SCLK  SPI clock pin. Defaults to the hardware SCLK pin.
 * @tparam MOSI  SPI MOSI pin. Defaults to the hardware MOSI pin.
 * @tparam MISO  SPI MISO pin. Defaults to the hardware MISO pin.
 *
 * The user is responsible for installing the SPI interrupt vector and
 * dispatching it to `MLX90363<...>::isr()`.
 */
template <typename SS, typename SCLK = AVR::SPI::SCLK, typename MOSI = AVR::SPI::MOSI, typename MISO = AVR::SPI::MISO>
class MLX90363 {
public:
  /// The fixed message length that the MLX90363 sends.
  static constexpr u1 messageLength = 8;

  /// Resolution of the angle reading (alpha/beta) in bits.
  static constexpr u1 resolutionBits = 14;

  /// Selected datasheet timings (5V supply, microseconds). Caller waits
  /// before reading the result of a measurement command.
  static constexpr u4 tMeasurementAlpha_us = 920;
  static constexpr u4 tMeasurementAlphaBeta_us = 1050;
  static constexpr u4 tMeasurementXYZ_us = 920;
  static constexpr u4 tStartUp_us = 20000;
  static constexpr u4 tShort_us = 120;

  /// Top-level state of the receive state machine. Reset to `Init` by
  /// `init()` and updated as transmissions progress.
  enum class ResponseState : u1 {
    Init,
    Ready,
    Receiving,
    Received,
    failedCRC,
    TypeA,
    TypeAB,
    TypeXYZ,
    Other,
  };

  /// The 2-bit "marker" field attached to incoming MLX90363 messages.
  enum class MessageType : b2 { Alpha = 0, AlphaBeta = 1, XYZ = 2, Other = 3 };

  /// MLX90363 opcodes (see datasheet). The "incoming" and "outgoing" groups
  /// are interleaved per the datasheet's two-column layout.
  enum class Opcode : b6 {
    GET1 = 0x13,
    GET2 = 0x14,
    GET3 = 0x15,
    Get3Ready = 0x2D,
    MemoryRead = 0x01,
    MemoryRead_Answer = 0x02,
    EEPROMWrite = 0x03,
    EEPROMWrite_Challenge = 0x04,
    EEChallengeAns = 0x05,
    EEReadAnswer = 0x28,
    EEReadChallenge = 0x0F,
    EEPROMWrite_Status = 0x0E,
    NOP__Challenge = 0x10,
    Challenge__NOP_MISO_Packet = 0x11,
    DiagnosticDetails = 0x16,
    Diagnostics_Answer = 0x17,
    OscCounterStart = 0x18,
    OscCounterStart_Acknowledge = 0x19,
    OscCounterStop = 0x1A,
    OscCounterStopAck_CounterValue = 0x1B,
    Reboot = 0x2F,
    Standby = 0x31,
    StandbyAck = 0x32,
    Error_frame = 0x3D,
    NothingToTransmit = 0x3E,
    Ready_Message = 0x2C,
  };

private:
  inline static u1 TxBuffer[messageLength]{};
  inline static u1 RxBuffer[messageLength]{};
  inline static u1 bufferPosition = messageLength;

  inline static ResponseState responseState = ResponseState::Init;
  inline static void (*volatile alphaHandler)(u2 a) = nullptr;

  inline static volatile Atomic<u2> alpha{};
  inline static volatile Atomic<u2> beta{};
  inline static volatile Atomic<u2> X{};
  inline static volatile Atomic<u2> Y{};
  inline static volatile Atomic<u2> Z{};
  inline static volatile Atomic<u1> failedCRCs{0};

  // u1 (single-byte) volatiles don't need Atomic — AVR loads/stores are
  // already atomic at byte granularity.
  inline static volatile u1 err = 0;
  inline static volatile u1 VG = 0;
  // Sentinel value the chip can never set, so the first real ROLL is detected
  // by `hasNewData()`.
  inline static volatile u1 ROLL = 0xff;

  static inline void sendSPI(u1 b) { *AVR::SPI::DR = b; }
  static inline u1 receiveSPI() { return *AVR::SPI::DR; }

  static void startTransmittingUnsafe() {
    bufferPosition = 0;
    SS::clr(); // assert SS (active-low)
    sendSPI(TxBuffer[bufferPosition]);
    responseState = ResponseState::Receiving;
  }

  static void fillTxBufferCRC() {
    u1 crc = 0xff;
    crc = detail::lookupCRC(TxBuffer[0] ^ crc);
    crc = detail::lookupCRC(TxBuffer[1] ^ crc);
    crc = detail::lookupCRC(TxBuffer[2] ^ crc);
    crc = detail::lookupCRC(TxBuffer[3] ^ crc);
    crc = detail::lookupCRC(TxBuffer[4] ^ crc);
    crc = detail::lookupCRC(TxBuffer[5] ^ crc);
    crc = detail::lookupCRC(TxBuffer[6] ^ crc);
    TxBuffer[7] = ~crc;
  }

  static bool checkRxBufferCRC() {
    u1 crc = 0xff;
    crc = detail::lookupCRC(RxBuffer[0] ^ crc);
    crc = detail::lookupCRC(RxBuffer[1] ^ crc);
    crc = detail::lookupCRC(RxBuffer[2] ^ crc);
    crc = detail::lookupCRC(RxBuffer[3] ^ crc);
    crc = detail::lookupCRC(RxBuffer[4] ^ crc);
    crc = detail::lookupCRC(RxBuffer[5] ^ crc);
    crc = detail::lookupCRC(RxBuffer[6] ^ crc);
    return RxBuffer[7] == (u1)~crc;
  }

  static void handleAlpha() {
    alpha.setUnsafe(RxBuffer[0] | ((RxBuffer[1] & 0x3f) << 8));
    err = RxBuffer[1] >> 6;
    VG = RxBuffer[4];
    ROLL = RxBuffer[6] & 0x3f;
  }

  static void handleAlphaBeta() {
    alpha.setUnsafe(RxBuffer[0] | ((RxBuffer[1] & 0x3f) << 8));
    beta.setUnsafe(RxBuffer[2] | ((RxBuffer[3] & 0x3f) << 8));
    err = RxBuffer[1] >> 6;
    VG = RxBuffer[4];
    ROLL = RxBuffer[6] & 0x3f;
  }

  static void handleXYZ() {
    X.setUnsafe(RxBuffer[0] | ((RxBuffer[1] & 0x3f) << 8));
    Y.setUnsafe(RxBuffer[2] | ((RxBuffer[3] & 0x3f) << 8));
    Z.setUnsafe(RxBuffer[4] | ((RxBuffer[5] & 0x3f) << 8));
    err = RxBuffer[1] >> 6;
    ROLL = RxBuffer[6] & 0x3f;
  }

  static void handleResponse() {
    if (!checkRxBufferCRC()) {
      responseState = ResponseState::failedCRC;
      auto &ref = failedCRCs.getUnsafe();
      if (ref != 0xff)
        ref++;
      return;
    }

    const u1 marker = RxBuffer[6] >> 6;

    if (marker == 0) {
      handleAlpha();
      responseState = ResponseState::TypeA;
      if (auto handler = alphaHandler)
        handler(alpha.getUnsafe());
      return;
    }

    if (marker == 1) {
      handleAlphaBeta();
      responseState = ResponseState::TypeAB;
      return;
    }

    if (marker == 2) {
      handleXYZ();
      responseState = ResponseState::TypeXYZ;
      return;
    }

    responseState = ResponseState::Other;
  }

  static inline void setCommandUnsafe(MessageType t, Opcode c) { TxBuffer[6] = (u1(t) << 6) | u1(c); }
  static inline void setCommandUnsafe(Opcode c) { setCommandUnsafe(MessageType::Other, c); }

public:
  /**
   * Initialise the SPI hardware and slave-select line.
   *
   * Sets SS as an output and de-asserts it (HIGH). Also forces the AVR's
   * hardware-SS pin (PB0 on atmega32u4) to be an output — if it were an
   * input pulled LOW, the SPI hardware would automatically switch to slave
   * mode and break everything.
   */
  static void init() {
    // User slave-select: de-assert then make output.
    SS::set();
    SS::output();

    MOSI::output();
    SCLK::output();

    // Make sure the hardware-SS line (PB0 on atmega32u4) is an output so the
    // SPI hardware can't kick us out of master mode.
    AVR::SPI::SS::output();
    AVR::SPI::SS::set();

    // Stop any running transfer, then configure: SPI enable, interrupt
    // enable, master, MSB-first, mode 1, F_CPU/64.
    AVR::SPI::CR->byte = 0;
    AVR::SPI::SR->byte = 0;
    AVR::SPI::CR->byte = 0b11010111;

    bufferPosition = messageLength;
    responseState = ResponseState::Ready;
  }

  /**
   * SPI complete interrupt handler. The user must dispatch their
   * `ISR(SPI_STC_vect)` to this function.
   */
  static inline void isr() {
    RxBuffer[bufferPosition++] = receiveSPI();

    if (bufferPosition == messageLength) {
      SS::set(); // de-assert SS
      responseState = ResponseState::Received;
      handleResponse();
    } else {
      sendSPI(TxBuffer[bufferPosition]);
    }
  }

  /**
   * Set the SPI bus speed by writing the divider bits of SPCR. See the
   * atmega32u4 datasheet for valid values.
   */
  static void setSPISpeed(u1 divider) { AVR::SPI::CR->Divider = divider; }

  /**
   * @return true while a transmission is in flight (SS is asserted).
   */
  static inline bool isTransmitting() { return bufferPosition != messageLength; }

  /**
   * Begin sending whatever is currently in the TxBuffer. No-op if a
   * transmission is already running.
   */
  static void startTransmitting() {
    if (isTransmitting())
      return;
    startTransmittingUnsafe();
  }

  /**
   * Abort any running transmission. Resets the SPI hardware.
   */
  static void stopTransmitting() {
    if (!isTransmitting())
      return;

    AVR::SPI::CR->byte = 0;
    SS::set(); // de-assert SS

    // Clear status flag and re-enable.
    AVR::SPI::SR->byte = 0;
    (void)*AVR::SPI::DR;
    AVR::SPI::CR->byte = 0b11010111;

    bufferPosition = messageLength;
    responseState = ResponseState::Ready;
  }

  /**
   * Build a standard GET1 message for the given response type. Caller must
   * not call this while a transmission is running (no-op in that case).
   */
  static void prepareGET1Message(MessageType type, u2 timeout = 0xffff, bool resetRoll = false) {
    if (isTransmitting())
      return;
    TxBuffer[0] = 0;
    TxBuffer[1] = resetRoll;
    TxBuffer[2] = timeout & 0xff;
    TxBuffer[3] = timeout >> 8;
    TxBuffer[4] = 0;
    TxBuffer[5] = 0;
    setCommandUnsafe(type, Opcode::GET1);
    fillTxBufferCRC();
  }

  /**
   * Build a memory-read message for two consecutive 16-bit addresses.
   */
  static void prepareReadMessage(u2 addr0, u2 addr1) {
    if (isTransmitting())
      return;
    TxBuffer[0] = (u1)addr0;
    TxBuffer[1] = (u1)(addr0 >> 8);
    TxBuffer[2] = (u1)addr1;
    TxBuffer[3] = (u1)(addr1 >> 8);
    TxBuffer[4] = 0;
    TxBuffer[5] = 0;
    setCommandUnsafe(Opcode::MemoryRead);
    fillTxBufferCRC();
  }

  // --- Read decoded values -----------------------------------------------

  static inline u2 getAlpha() { return alpha; }
  static inline u2 getBeta() { return beta; }
  static inline u2 getX() { return X; }
  static inline u2 getY() { return Y; }
  static inline u2 getZ() { return Z; }

  /// Read variants safe to call from inside an ISR (no atomic block).
  static inline u2 getAlphaUnsafe() { return alpha.getUnsafe(); }
  static inline u2 getBetaUnsafe() { return beta.getUnsafe(); }
  static inline u2 getXUnsafe() { return X.getUnsafe(); }
  static inline u2 getYUnsafe() { return Y.getUnsafe(); }
  static inline u2 getZUnsafe() { return Z.getUnsafe(); }

  /// Read variants that assume interrupts are currently enabled and use the
  /// cheaper sei/cli sequence rather than save-restore.
  static inline u2 getAlphaForceInterruptsOn() { return alpha.getForceInterruptsOn(); }
  static inline u2 getBetaForceInterruptsOn() { return beta.getForceInterruptsOn(); }
  static inline u2 getXForceInterruptsOn() { return X.getForceInterruptsOn(); }
  static inline u2 getYForceInterruptsOn() { return Y.getForceInterruptsOn(); }
  static inline u2 getZForceInterruptsOn() { return Z.getForceInterruptsOn(); }

  static inline u1 getRoll() { return ROLL; }
  static inline u1 getErr() { return err; }
  static inline u1 getVG() { return VG; }

  /// Read and clear the CRC-failure counter (saturates at 0xff).
  static inline u1 getCRCFailures() { return failedCRCs.getAndSet(0); }

  /**
   * @return true if the chip's ROLL counter has advanced since the last
   *         call. Updates `lastRoll` to the current value.
   */
  static inline bool hasNewData(u1 &lastRoll) {
    const u1 r = ROLL;
    if (r == lastRoll)
      return false;
    lastRoll = r;
    return true;
  }

  /**
   * Register a callback invoked from the ISR whenever an Alpha-type
   * response is decoded. Pass `nullptr` to disable.
   */
  static inline void setAlphaHandler(void (*handler)(u2 a)) { alphaHandler = handler; }

  static inline ResponseState getResponseState() { return responseState; }

  /// Last received opcode (low 6 bits of byte 6 of the RxBuffer).
  static inline b6 getReceivedOpCode() { return RxBuffer[6] & 0x3f; }

  /// Direct buffer access for advanced users (e.g. memory-write flows).
  static inline u1 *getTxBuffer() { return TxBuffer; }
  static inline const u1 *getRxBuffer() { return RxBuffer; }
};

} // namespace MLX90363Lib
