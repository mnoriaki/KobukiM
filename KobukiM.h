#ifndef __KOBUKIM_H__
#define __KOBUKIM_H__

#include <stdint.h>

const uint8_t Kobuki_Header0 = 0xaa;
const uint8_t Kobuki_Header1 = 0x55;

const uint8_t Kobuki_CmdBaseControl = 1;
const uint8_t Kobuki_CmdSound = 3;
const uint8_t Kobuki_CmdSoundSequence = 4;
const uint8_t Kobuki_CmdRequestExtra = 9;
const uint8_t Kobuki_CmdGeneralPurposeOutput = 12;
const uint8_t Kobuki_CmdSetControllerGain = 13;
const uint8_t Kobuki_CmdGetControllerGain = 14;

const uint8_t Kobuki_FbBasicSensorData = 1;
const uint8_t Kobuki_FbDockingIR = 3;
const uint8_t Kobuki_FbInertialSensor = 4;
const uint8_t Kobuki_FbCliff = 5;
const uint8_t Kobuki_FbCurrent = 6;
const uint8_t Kobuki_FbHardwareVersion = 10;
const uint8_t Kobuki_FbFirmwareVersion = 11;
const uint8_t Kobuki_FbRawGyro = 13;
const uint8_t Kobuki_FbGeneralPurposeInput = 16;
const uint8_t Kobuki_FbUDID = 19;
const uint8_t Kobuki_FbControllerInfo = 21;

uint8_t Kobuki_CalChecksum(uint8_t *packet);
bool Kobuki_ChkChecksum(uint8_t *packet);

struct KobukiBasicSensor {
  uint16_t t;
  uint8_t bumper;
  uint8_t wheelDrop;
  uint8_t cliff;
  uint16_t leftEncoder;
  uint16_t rightEncoder;
  int8_t leftPWM;
  int8_t rightPWM;
  uint8_t button;
  uint8_t charger;
  uint8_t battery = 167; // 16.7[V]
  uint8_t overCurrent;
};

struct KobukiPIDs {
  uint32_t FKp = 100*1000;
  uint32_t FKi = 100;
  uint32_t FKd = 2000;

  uint32_t Kp = 100*1000;
  uint32_t Ki = 100;
  uint32_t Kd = 2000;
};

class KobukiByteStream {
 private:
  uint8_t *p;
  uint8_t *length;

 public:
  uint8_t packet[256+5];

  KobukiByteStream() {
    reset();
  }

  uint8_t len() {
    return *length + 5;
  }

  void reset() {
    packet[0] = Kobuki_Header0;
    packet[1] = Kobuki_Header1;
    length = packet+2;
    p = packet+3;
    *length = 0;
  }

  void addBasicSensor(const KobukiBasicSensor &bs);
  void addCliff(uint16_t cliffR, uint16_t cliffC, uint16_t cliffL);
  void addControllerInfo(uint8_t type, uint32_t kp, uint32_t ki, uint32_t kd);
  void addCurrent(uint16_t curL, uint16_t curR);
  void addDockingIR(uint8_t right, uint8_t central, uint8_t left);
  void addFirmwareVersion(uint8_t major, uint8_t minor, uint8_t patch);
  void addGeneralPurposeInput(uint16_t din, uint16_t a0, uint16_t a1, uint16_t a2, uint16_t a3);
  void addHardwareVersion(uint8_t major, uint8_t minor, uint8_t patch);
  void addInertialSensor(int16_t ang, int16_t angRate);
  void addRawGyro1(uint8_t id, int16_t x, int16_t y, int16_t z);
  void addRawGyro2(uint8_t id, int16_t x, int16_t y, int16_t z,
		   int16_t x2, int16_t y2, int16_t z2);
  void addRawGyro3(uint8_t id, int16_t x, int16_t y, int16_t z,
		   int16_t x2, int16_t y2, int16_t z2,
		   int16_t x3, int16_t y3, int16_t z3);
  void addUDID(uint32_t UDID0, uint32_t UDID1, uint32_t UDID2);
  void setCheckSum();

 private:
  void addByte(uint8_t c) {
    *p = c;
    p ++;
    *length = *length + 1;
  }

  void addShort(uint16_t s) {
    *p = s & 0xff; p ++;
    *p = (s >> 8) & 0xff; p ++;
    *length = *length + 2;
  }

  void addLong(uint32_t s) {
    *p = s & 0xff; p ++;
    *p = (s >> 8) & 0xff; p ++;
    *p = (s >> 16) & 0xff; p ++;
    *p = (s >> 24) & 0xff; p ++;
    *length = *length + 4;
  }
};


class KobukiByteStreamReciever {
 private:
  uint8_t flags;
  uint8_t *p;
  uint16_t stat;

 public:
  uint8_t *length;
  uint8_t packet[256+5];
  int16_t spd, radius;
  KobukiPIDs kPIDs;

  const uint8_t flagHardwareVersion = 0x1;
  const uint8_t flagFirmwareVersion = 0x2;
  const uint8_t flagUDID = 0x4;
  const uint8_t flagPID = 0x8;
  const uint8_t flagSetPID = 0x10;
  const uint8_t flagUpdateVel = 0x20;

  KobukiByteStreamReciever() {
    reset();
  }

  void clearFlag(uint8_t f) {
    flags &= ~f;
  }

  bool isSet(uint8_t f) {
    return (flags & f) ? true : false;
  }

  void reset() {
    packet[0] = Kobuki_Header0;
    packet[1] = Kobuki_Header1;
    length = packet + 2;
    p = length + 1;
    *p = 0;
  }

  bool recv(uint8_t c);
  void handle();

 private:
  uint16_t toUint16(uint8_t *p) {
    uint16_t r = *p;
    r |= (*(p+1) << 8);
    return r;
  }
  int16_t toInt16(uint8_t *p) {
    uint16_t r = *p;
    r |= (*(p+1) << 8);
    return (int16_t)r;
  }
  uint32_t toUint32(uint8_t *p) {
    uint32_t r = *p;
    r |= (*(p+1) << 8);
    r |= (*(p+2) << 16);
    r |= (*(p+3) << 24);
    return r;
  }
  int32_t toInt32(uint8_t *p) {
    uint32_t r = *p;
    r |= (*(p+1) << 8);
    r |= (*(p+2) << 16);
    r |= (*(p+3) << 24);
    return (int32_t)r;
  }
};

#endif // !defined(__KOBUKIM_H__)
