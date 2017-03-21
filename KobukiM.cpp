/*
BSD 2-Clause License

Copyright (c) 2017, Noriaki Mitsunaga
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <KobukiM.h>

uint8_t Kobuki_CalChecksum(uint8_t *packet) {
  uint8_t *p = packet + 2;
  int sz = *p + 1;
  uint8_t cs = 0;
  
  for (int i=0; i<sz; i++, p++) {
    cs ^= *p;
  }
  return cs;
}

bool Kobuki_ChkChecksum(uint8_t *packet) {
  uint8_t* p = packet + 2;
  int sz = *p + 2;
  uint8_t cs = 0;
  
  for (int i=0; i<sz; i++, p++) {
    cs ^= *p;
  }
  return cs ? false:true;
}

bool KobukiByteStreamReciever::recv(uint8_t c) {
  bool ret = false;
#if 0
  if (stat >= 2) {
    if (c < 0x10)
      lcd.print(0, HEX);
    lcd.print(c, HEX);
  }
#endif
  if (stat == 0) {
    if (c == Kobuki_Header0)
      stat ++;
  } else if (stat == 1) {
    if (c == Kobuki_Header1) {
      stat ++;
      // lcd.setCursor(0, 0);
    } else 
      stat = 0;
  } else if (stat == 2) {
    *length = c;
    stat ++;
  } else {
    *p = c;
    p ++;
    stat ++;
    if (stat == (*length + 4)) { // 4 = 2(header) + 1(len) + 1(sum)
      //ret = true;
      ret = Kobuki_ChkChecksum(packet);
      stat = 0;
      //lcd.print(' ');
    }
  }
  return ret;
}

void KobukiByteStreamReciever::handle() {
  uint8_t *q = packet + 3;
  uint8_t len = *length;
  uint8_t pos = 0;
    
  while (pos<len) {
    uint8_t l = *(q+1);

    switch(*q) {
    case Kobuki_CmdBaseControl:
      if (l != 4)
	return;
#if 0
      lcd.setCursor(0, 1);
      for (int i=0; i<4; i++) {
	lcd.print(*(q+i+2), HEX);
	lcd.print(' ');
      }
#endif
      spd = toInt16(q+2);
      radius = toInt16(q+4);
      flags |= flagUpdateVel;
      break;
    case Kobuki_CmdSound:
      if (l != 3)
	return;
      // ignore
      break;
    case Kobuki_CmdSoundSequence:
      if (l != 1)
	return;
      // ignore
      break;
    case Kobuki_CmdRequestExtra:
      if (l != 2)
	return;
      {
	short f = toUint16(q+2);
	if (f & 0x1)
	  flags |= flagHardwareVersion;
	else if (f & 0x2)
	  flags |= flagFirmwareVersion;
	else if (f & 0x8)
	  flags |= flagUDID;
      }
      break;
    case Kobuki_CmdGeneralPurposeOutput:
      if (l != 2)
	return;
      // ignore
      break;
    case Kobuki_CmdSetControllerGain:
      if (l != 13)
	return;
      if (*(q+2) == 0) {
	kPIDs.FKp = toUint32(q+3);
	kPIDs.FKi = toUint32(q+7);
	kPIDs.FKd = toUint32(q+11);
      } else {
	kPIDs.Kp = toUint32(q+3);
	kPIDs.Ki = toUint32(q+7);
	kPIDs.Kd = toUint32(q+11);
      }
      flags |= flagSetPID;
      break;
    case Kobuki_CmdGetControllerGain:
      if (l != 1)
	return;
      flags |= flagPID;
      break;
    default:
      break;
    }
    pos += (l+2); q += (l+2);
  }
}

void KobukiByteStream::addBasicSensor(const KobukiBasicSensor &bs) {
  addByte(Kobuki_FbBasicSensorData);
  addByte(15);
  addShort(bs.t);
  addByte(bs.bumper);
  addByte(bs.wheelDrop);
  addByte(bs.cliff);
  addShort((uint16_t)bs.leftEncoder);
  addShort((uint16_t)bs.rightEncoder);
  addByte((uint8_t)bs.leftPWM);
  addByte((uint8_t)bs.rightPWM);
  addByte(bs.button);
  addByte(bs.charger);
  addByte(bs.battery);
  addByte(bs.overCurrent);
}


void KobukiByteStream::addDockingIR(uint8_t right, uint8_t central, uint8_t left) {
  addByte(Kobuki_FbDockingIR);
  addByte(3);
  addByte(right);
  addByte(central);
  addByte(left);
}

void KobukiByteStream::addInertialSensor(int16_t ang, int16_t angRate) {
  addByte(Kobuki_FbInertialSensor);
  addByte(7);
  addShort(ang);
  addShort(angRate);
  addByte(0);
  addByte(0);
  addByte(0);
}

void KobukiByteStream::addCliff(uint16_t cliffR, uint16_t cliffC, uint16_t cliffL) {
  addByte(Kobuki_FbCliff);
  addByte(6);
  addShort(cliffR);
  addShort(cliffC);
  addShort(cliffL);
}

void KobukiByteStream::addCurrent(uint16_t curL, uint16_t curR) {
  addByte(Kobuki_FbCurrent);
  addByte(2);
  addByte(curL);
  addByte(curR);
}

void KobukiByteStream::addHardwareVersion(uint8_t major, uint8_t minor, uint8_t patch) {
  addByte(Kobuki_FbHardwareVersion);
  addByte(4);
  addByte(patch);
  addByte(minor);
  addByte(major);
  addByte(0);
}

void KobukiByteStream::addFirmwareVersion(uint8_t major, uint8_t minor, uint8_t patch) {
  addByte(Kobuki_FbFirmwareVersion);
  addByte(4);
  addByte(patch);
  addByte(minor);
  addByte(major);
  addByte(0);
};

void KobukiByteStream::addRawGyro1(uint8_t id, int16_t x, int16_t y, int16_t z) {
  addByte(Kobuki_FbRawGyro);
  addByte(2+6*1);
  addByte(id);
  addByte(3*1);
  addShort(x);
  addShort(y);
  addShort(z);
}

void KobukiByteStream::addRawGyro2(uint8_t id, int16_t x, int16_t y, int16_t z,
				   int16_t x2, int16_t y2, int16_t z2) {
  addByte(Kobuki_FbRawGyro);
  addByte(2+6*2);
  addByte(id);
  addByte(3*2);
  addShort(x);
  addShort(y);
  addShort(z);
  addShort(x2);
  addShort(y2);
  addShort(z2);
}

void KobukiByteStream::addRawGyro3(uint8_t id, int16_t x, int16_t y, int16_t z,
				   int16_t x2, int16_t y2, int16_t z2,
				   int16_t x3, int16_t y3, int16_t z3) {
  addByte(Kobuki_FbRawGyro);
  addByte(2+6*3);
  addByte(id);
  addByte(3*3);
  addShort(x);
  addShort(y);
  addShort(z);
  addShort(x2);
  addShort(y2);
  addShort(z2);
  addShort(x3);
  addShort(y3);
  addShort(z3);
}

void KobukiByteStream::addGeneralPurposeInput(uint16_t din, uint16_t a0, uint16_t a1, uint16_t a2, uint16_t a3) {
  addByte(Kobuki_FbGeneralPurposeInput);
  addByte(16);
  addShort(din);
  addShort(a0);
  addShort(a1);
  addShort(a2);
  addShort(a3);
  addShort(0);
  addShort(0);
  addShort(0);
}

void KobukiByteStream::addUDID(uint32_t UDID0, uint32_t UDID1, uint32_t UDID2) {
  addByte(Kobuki_FbUDID);
  addByte(12);
  addLong(UDID0);
  addLong(UDID1);
  addLong(UDID2);
}

void KobukiByteStream::addControllerInfo(uint8_t type, uint32_t kp, uint32_t ki, uint32_t kd) {
  addByte(Kobuki_FbControllerInfo);
  addByte(13);
  addByte(type);
  addLong(kp);
  addLong(ki);
  addLong(kd);
}

void KobukiByteStream::setCheckSum() {
  *p = Kobuki_CalChecksum(packet);
  p ++;
  *p = 0;
}
