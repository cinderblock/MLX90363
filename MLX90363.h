/* 
 * File:   MLX90363.h
 * Author: Cameron
 *
 * Created on December 10, 2014, 4:11 PM
 */

#ifndef MLX90363_H
#define	MLX90363_H

#include <SPI.h>

class MLX90363 {
 /**
  * The fixed message length that the MLX90363 sends
  */
 static constexpr unsigned char messageLength = 8;
 
 /**
  * Staged transmit buffer. Will be sent automatically by the interrupt routine
  * if properly started and left alone so that repeat messages are trivial.
  */
 static unsigned char TxBuffer[messageLength];
 
 /**
  * The buffer for the incoming/received message
  */
 static unsigned char RxBuffer[messageLength];
 
 /**
  * Where we are while sending/reading the data in the SPI interrupt
  */
 static unsigned char bufferPosition;
 
 enum class ResponseState : unsigned char;
 
 static SPISettings spiSettings;
 
 static ResponseState responseState;
 
 unsigned short alpha;
 unsigned short beta;
 unsigned short X;
 unsigned short Y;
 unsigned short Z;
 unsigned char err;
 unsigned char VG;
 unsigned char ROLL;
 
 /**
  * INTERNAL: Reset the buffer position and start the transmission sequence.
  * 
  * Unsafe because it does not check if there is already a transmission running.
  */
 static void startTransmittingUnsafe();
 
 /**
  * Calculate and write the correct CRC for the message in and to the TxBuffer
  */
 static void fillTxBufferCRC();
 
 /**
  * Check the checksum of the data in the RxBuffer
  * @return 
  */
 static bool checkRxBufferCRC();

 /**
  * OpCodes from the MLX90363 datasheet.
  * 
  */
 enum class Opcode: unsigned int {
  // Following the format from the datasheet, they organized all the opcodes as
  // Outgoing      or       Incoming
  GET1 = 0x13,
  GET2 = 0x14,
  GET3 = 0x15,              Get3Ready = 0x2D,
  MemoryRead = 0x01,        MemoryRead_Answer = 0x02,
  EEPROMWrite = 0x03,       EEPROMWrite_Challenge = 0x04,
  EEChallengeAns = 0x05,    EEReadAnswer = 0x28,
  EEReadChallenge = 0x0F,   EEPROMWrite_Status = 0x0E,
  NOP__Challenge = 0x10,    Challenge__NOP_MISO_Packet = 0x11,
  DiagnosticDetails = 0x16, Diagnostics_Answer = 0x17,
  OscCounterStart = 0x18,   OscCounterStart_Acknowledge = 0x19,
  OscCounterStop = 0x1A,    OscCounterStopAck_CounterValue = 0x1B,
  Reboot = 0x2F,
  Standby = 0x31,           StandbyAck = 0x32,
                            Error_frame = 0x3D,
                            NothingToTransmit = 0x3E,
                            Ready_Message = 0x2C,
 };
 
 /**
  * Handle a standard Alpha response from the MLX90363
  */
 void handleAlpha();
 
 /**
  * Handle a standard AlphaBeta response from the MLX90363
  */
 void handleAlphaBeta();
 
 /**
  * Handle a standard XYZ response from the MLX90363
  */
 void handleXYZ();
 
 /**
  * Handle a new available byte from the SPI receive buffer
  */
 static void handleIncomingByte();
 
 static MLX90363 * currentMLX;
 
 const unsigned int pin;
 unsigned long dataReadyTime;
 
public:
  
  MLX90363(const unsigned int p);
  
  bool update();
 
 /**
  * MLX requires a time between data checks.
  * This function returns true when the required time has passed
  */
 bool isMeasurementReady();
 
 enum class ResponseState: unsigned char {
  Init, Ready, Receiving, Received, failedCRC, TypeA, TypeAB, TypeXYZ, Other
 };
 
 /**
  * The 2-bit marker attached to all incoming messages for easy processing
  */
 enum class MessageType: unsigned int {
  Alpha = 0, AlphaBeta = 1, XYZ = 2, Other = 3
 };
 
 /**
  * Initialize the hardware.
  */
 static void init();
 
 inline bool hasNewData(unsigned char& lastRoll) {
  unsigned char const r = ROLL;
  if (r == lastRoll) return false;
  lastRoll = r;
  return true;
 }
 
 /**
  * Set the SPI hardware's divider
  * @param 
  */
 static void setSPISpeed(unsigned char const);
 
 /**
  * Check if we're still talking on the SPI bus
  * @return 
  */
 inline static bool isTransmitting() {
  // Any of these would work. Not sure which is most effective
  return bufferPosition != messageLength;
  //return !SS.isHigh();
  // Let's use the low level test from Board::
  // return Board::SPI::isSlaveSelected();
 }
 
 /**
  * Start sending whatever is in the buffer, unless it's already being used
  * 
  */
 static void startTransmitting();
 
 static unsigned int getReceivedOpCode();

 /**
  * Handle a received message.
  * 
  * Checks the CRC
  * Checks for standard marker
  *  and passes the message off to another internal handler
  */
 static void handleResponse();
 
 // inline static unsigned short getAlpha() {return (unsigned short const) alpha;}
 inline unsigned short getAlpha() {return alpha;}
 
 inline unsigned short getBeta() {return beta;}
 
 inline unsigned short getX() {return X;}
 
 inline unsigned short getY() {return Y;}
 
 inline unsigned short getZ() {return Z;}
 
 inline unsigned char getRoll() {return ROLL;}
 
 inline unsigned char getErr() {return err;}
 
 /**
  * Construct a standard GET1 message
  * @param type
  * @param timeout
  * @param resetRoll
  */
 static void prepareGET1Message(MessageType const type, const unsigned short timeout = 0xffff, bool const resetRoll = false);

 static constexpr unsigned int alphaBits = 14;
 static constexpr unsigned int alphaModulo = 1 << alphaBits;
 static constexpr unsigned int alphaMask = alphaModulo - 1;
};

#endif	/* MLX90363_H */
