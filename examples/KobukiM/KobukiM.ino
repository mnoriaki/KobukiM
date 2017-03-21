// Kobuki にみせかけるためのプログラム
#include <I2CLiquidCrystal.h> // I2C 液晶ライブラリ
#include <Wire.h>             // I2C 通信ライブラリ(I2C 液晶に必要)
#include <KobukiM.h>          // Kobuki の通信を模擬するライブラリ

I2CLiquidCrystal lcd(0x3c, (uint8_t)127); // I2C 液晶 (OLED)

const float wheelW = 640.0;       // 左右の車輪間の距離 (Melcy の値)
const float KobukiWheelW = 230.0; // 左右の車輪間の距離 (Kobukiの値)

const int feedbackPacketInterval = 20; // 20[ms] (50[Hz])

KobukiBasicSensor bs;                // 現在のセンサの値
KobukiByteStream fbPacket;           // PCへ送るパケット
KobukiByteStreamReciever fbRecv;     // PCから受け取るパケット
KobukiPIDs kPIDs;                    // 現在の PID の値
uint8_t gyroID = 0;                  // Gyro のカウンタ
uint32_t prev = 0;                   // 前回パケットをPCへ送った時刻[ms]
int16_t wheelL = 0, wheelR = 0;      // 車輪速度の目標値
int32_t leftEnc, rightEnc;           // エンコーダの値

void sendPacket();
void setWheel(int16_t spd, int16_t radius);

void setup() {
  Serial.begin(115200);  // USB
  Serial1.begin(115200); // via PC10/PC11 (FTDI)

  lcd.begin(16, 2);
  lcd.print("Hello,");
  lcd.setCursor(0, 1);
  lcd.print(" this is Melcy");
}

void loop() {
  uint32_t cur = millis();

  while (Serial1.available() > 0) {
    char c = Serial1.read();
    // Serial.write(c);   // for debug
    if (fbRecv.recv(c)) { // パケットを受け取ったとき
      // パケットを処理する
      fbRecv.handle();

      if (fbRecv.isSet(fbRecv.flagSetPID)) { // PIDゲインの更新
	kPIDs = fbRecv.kPIDs;
	fbRecv.clearFlag(fbRecv.flagSetPID);
      }
      if (fbRecv.isSet(fbRecv.flagUpdateVel)) { // 目標速度の更新
	setWheel(fbRecv.spd, fbRecv.radius);
	fbRecv.clearFlag(fbRecv.flagUpdateVel);
	
#if 1
	// 液晶に現在の目標速度を表示
	//lcd.setCursor(0, 1);
	lcd.clear();
	lcd.print(fbRecv.spd);
	lcd.print(' ');
	lcd.print(fbRecv.radius);
	lcd.print(' ');
	lcd.print(wheelL);
	lcd.print(' ');
	lcd.print(wheelR);
#endif
      }
      fbRecv.reset();
      break;
    }
  }

  if (cur - prev > feedbackPacketInterval) { // Send feedback every 20ms (50Hz);
    // PC へパケットを送る
    sendPacket();

    // 次に更新する時刻を決める
    prev += feedbackPacketInterval;

    leftEnc ++;     // テスト用
    rightEnc += 2;  // テスト用
  }
}

void sendPacket() {
    // データの準備
    bs.t = millis();
    bs.leftEncoder = leftEnc;    // エンコーダの値
    bs.rightEncoder = rightEnc;  // エンコーダの値

    // パケットの組み立て
    fbPacket.reset();
    fbPacket.addBasicSensor(bs);
    fbPacket.addDockingIR(0, 0, 0);
    fbPacket.addInertialSensor(0, 0);
    fbPacket.addCliff(0, 0, 0);
    fbPacket.addCurrent(0, 0);
    fbPacket.addRawGyro2(gyroID, 0, 0, 0, 0, 0, 0);
    gyroID += 2; // Gyro のタイムスタンプをダミーで更新
    fbPacket.addGeneralPurposeInput(0, 0, 0, 0, 0);

    // PCから要求されているパケットがあれば追加
    if (fbRecv.isSet(fbRecv.flagHardwareVersion)) {
      fbPacket.addHardwareVersion(1, 1, 0);
      fbRecv.clearFlag(fbRecv.flagHardwareVersion);
    }
    if (fbRecv.isSet(fbRecv.flagFirmwareVersion)) {
      fbPacket.addFirmwareVersion(1, 2, 9);
      fbRecv.clearFlag(fbRecv.flagFirmwareVersion);
    }
    if (fbRecv.isSet(fbRecv.flagUDID)) {
      fbPacket.addUDID(1, 2, 3);
      fbRecv.clearFlag(fbRecv.flagUDID);
    }
    if (fbRecv.isSet(fbRecv.flagPID)) {
      fbPacket.addControllerInfo(1, kPIDs.Kp, kPIDs.Ki, kPIDs.Kd);
      fbRecv.clearFlag(fbRecv.flagUDID);
    }
    // チェックサムの追加
    fbPacket.setCheckSum();

    // パケットの送信
    Serial1.write(fbPacket.packet, fbPacket.len());
}

void setWheel(int16_t spd, int16_t radius) {
  float wL = .0, wR = .0;

  if (radius == 0) {
    wL = wR = spd; // [mm/s]
  } else if (radius == 1) {
    wL = -spd * (wheelW/KobukiWheelW);
    wR = spd * (wheelW/KobukiWheelW);
  } else {
    float r = radius;
    float v = spd;
    if (radius > 1) 
      v = v / (r + KobukiWheelW/2);
    else
      v = v / (r - KobukiWheelW/2);
    float w = v / r;
    wL = (r - wheelW/2.0)*v;
    wR = (r + wheelW/2.0)*v;
  }
  
  // ここから mm/s を カウント/制御周期 に変換する
  
  // 車輪の半径で割る（車輪の速度 mm/s → 車輪の回転速度rad/s）
  // 車輪直径 150mm (半径 75mm) XXX
  wL /= 75.0;
  wR /= 75.0;

  // ギア比をかける (車輪の回転速度rad/s → モータの回転速度rad/s)
  // モータ(DME60B8HPB)のギアヘッド(8DG30)がギア比が 30
  // モータと車輪の間のギア比が XXX
  // (小原歯車工業(KHK)のSS2-42 と SS2-xx ??)
  wL *= (30.0*2.0);
  wR *= (30.0*2.0);

  // 2πで割って、モータ1回転当たりのエンコーダのカウント×4 をかける
  // (モータの回転速度 rad/s → モータの回転数 rot/s → カウント/s)
  // ×4 は４逓倍でマイコンがカウントしているから
  // HEDS-5500 A12 の場合は 500 count/rot
  wL = wL / (2.0*M_PI) * 500 * 4;
  wR = wR / (2.0*M_PI) * 500 * 4;
  
  // 制御周期(0.05s XXX) をかける
  wL *= 0.05;
  wR *= 0.05;

  // ここまで (mm/s を カウント/制御周期 に変換する)

  // 浮動小数点→整数の変換
  wheelL = (int16_t)(wL+.5);
  wheelR = (int16_t)(wR+.5);
}
