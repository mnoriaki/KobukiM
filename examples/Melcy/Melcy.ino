#include <board/board.h>

#include <PS2X_lib.h>  //for v1.6
#include <I2CLiquidCrystal.h>
#include <Wire.h>
#include <KobukiM.h>          // Kobuki の通信を模擬するライブラリ

#if 0 // STmicro でなく Atmega を使っててストする場合は 1 にする
void pwmWrite(uint8_t pin, uint16_t out) {
  analogWrite(pin, (out >> 8) & 0xff);
}
#endif

#define USE_LCD // USE I2C LCD (OLED)

//////////////////////////////////////////////////
// パラメータ
//////////////////////////////////////////////////
const float wheelW = 640.0;       // 左右の車輪間の距離 (Melcy の値)
const float KobukiWheelW = 640.0;//230.0; // 左右の車輪間の距離 (Kobukiの値)

const int feedbackPacketInterval = 20; // 20[ms] (50[Hz])

// ピン周り
// VS-C1
const int pinVSC1clk = 13; // PA5
const int pinVSC1cmd = 11; // PA7
const int pinVSC1atn = 2;  // PA10
const int pinVSC1dat = 12; // PA6
// Encoder R (5V, TIMER3)
// TIM3_CH1: PC6=CN10-4, TIM3_CH2: 9 (PC7)
// Encoder L (5V, TIMER4)
// TIM4_CH1: 10 (PB6), TIM4_CH2: PB7=CN7-21
// Motors
// Motor motorL(3, 4, 5); //PWM=3, A=4, B=5
// Motor motorR(6, 7, 8); //PWM=6, A=7, B=8
// Serial PC10 (CN7-1), PC11 (CN7-2)
// 緊急停止：PC12 (5V)

// 2: VS-C1
// 3: motorL PWM
// 4: motorL A
// 5: motorL B
// 6: motorR PWM
// 7: motorR A
// 8: motorR B
// 9: Encoder R
// 10: Encoder L
// 11: VS-C1
// 12: VS-C1
// 13: VS-C1

//////////////////////////////////////////////////
// Motor class の定義
//////////////////////////////////////////////////
class Motor {
  public:
    Motor() {}
    Motor(uint8_t PWM, uint8_t A, uint8_t B) {
      pinPWM = PWM; pinA = A; pinB = B;
    }
    void init() {
      pinMode(pinA, OUTPUT);
      pinMode(pinB, OUTPUT);
      pinMode(pinPWM, PWM);

      digitalWrite(pinA, HIGH);
      digitalWrite(pinB, LOW);
      pwmWrite(pinPWM, 0);
    }
    void write(int32_t pwm) {
      if (pwm > 65535) {
        pwm = 65535;
      } else if (pwm < -65535) {
        pwm = -65535;
      }
      if (pwm == 0) {
        pwmWrite(pinPWM, 0);
      } else if (pwm > 0) {
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, LOW);
        pwmWrite(pinPWM, pwm);
      } else {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, HIGH);
        pwmWrite(pinPWM, -pwm);
      }
    }
    void breake() {
      pwmWrite(pinPWM, 0);
      digitalWrite(pinA, LOW);
      digitalWrite(pinB, LOW);
    }
  private:
    int pinA, pinB, pinPWM;
};

//////////////////////////////////////////////////
// Encoder class の定義
//////////////////////////////////////////////////
class Encoder {
  public:
    Encoder() {}
    Encoder(const timer_dev* TIMER) {
      TMR = TIMER;
    }
    void init() {
      if (TMR == NULL)
        return;

      // count bouth TI1, TI2's edges
      TMR->regs.adv->SMCR = TIMER_SMCR_SMS_ENCODER3;

      // set inputs (CC1S = 01, CC2S = 01)
      //TMR->regs.adv->CCMR1 = TIMER_CCMR1_CC2S_INPUT_TI1 | TIMER_CCMR1_CC1S_INPUT_TI1; // no filter
      TMR->regs.adv->CCMR1 = 0xf1f1;  // with filter

      // CC2P = 0, CC1P = 0
      TMR->regs.adv->CCER = 0;

      // no prescaler (1:1)
      TMR->regs.adv->PSC = 0;

      // set CR2 to its default value
      TMR->regs.adv->CR2 = 0;

      // set auto reload register as 0xffff
      TMR->regs.adv->ARR = 0xffff;

      // reset counts
      TMR->regs.bas->CNT = 0;

      // enable count
      TMR->regs.adv->CR1 |= TIMER_CR1_CEN;
    }
    int64_t read() {
      if (TMR == NULL)
        return 0;

      uint16_t cur = TMR->regs.bas->CNT;
      int32_t delta = cur - prev;
      if (delta > 32767)
        count += (delta - 65536);
      else if (delta > -32767)
        count += delta;
      else
        count += (65536 + delta);
      prev = cur;
      return count;
    }
    void reset() {
      if (TMR == NULL)
        return;

      count = 0;
      prev = TMR->regs.bas->CNT;
    }
  private:
    const timer_dev* TMR;
    uint16_t prev;
    int32_t count;
};

//////////////////////////////////////////////////
// 関数のプロトタイプ宣言
//////////////////////////////////////////////////
void sendPacket();
void setWheel(int16_t spd, int16_t radius);

//////////////////////////////////////////////////
// グローバル変数の宣言
//////////////////////////////////////////////////

// I2C 液晶
I2CLiquidCrystal lcd(0x3c, (uint8_t)127);

// PS2 コントローラ周り
PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning
//you must always either restart your Arduino after you conect the controller,
//or call config_gamepad(pins) again after connecting the controller.
int error = 0;
byte type = 0;
byte vibrate = 0;

// Kobuki もどき
KobukiBasicSensor bs;                // 現在のセンサの値
KobukiByteStream fbPacket;           // PCへ送るパケット
KobukiByteStreamReciever fbRecv;     // PCから受け取るパケット
KobukiPIDs kPIDs;                    // 現在の PID の値
uint8_t gyroID = 0;                  // Gyro のカウンタ
uint32_t prevKobukiPacketMillis = 0;                   // 前回パケットをPCへ送った時刻[ms]
//int16_t wheelL = 0, wheelR = 0;      // 車輪速度の目標値
//int32_t leftEnc, rightEnc;           // エンコーダの値

// エンコーダとモータ
// TIMER3 is fully remapped to use 5V torelant pins.
// TIM3_CH1: PC6=CN10-4, TIM3_CH2: PC7 = D9
Encoder encR(TIMER3);
// TIMER4 is not remapped (5V torelant pins are asigned)
// TIM4_CH1: PB6=D10, TIM4_CH2: PB7=CN7-21
Encoder encL(TIMER4);

Motor motorL(3, 4, 5); //PWM=3, A=4, B=5
Motor motorR(6, 7, 8); //PWM=6, A=7, B=8

unsigned long prevControlledMillis = 0;

// モータの PID 制御関連
int32_t peL = 0, peR = 0;
int32_t posL = 0, posR = 0;
int32_t intL = 0, intR = 0;
int32_t refL = 0, refR = 0;
int32_t Kp = 0, Kd = 0, Ki = 0;
unsigned long previousMillis50 = 0;

//////////////////////////////////////////////////
// モータの PID 制御関連の関数
//////////////////////////////////////////////////
void setGain() {
  if (Kp == 0 && Kd == 0 && Ki == 0) {
    intL = 0;
    intR = 0;
  }

  Kp = 50; Kd = 10; Ki = 5;
}

void resetGain() {
  Kp = Kd = Ki = 0;
  intL = 0;
  intR = 0;
}

//////////////////////////////////////////////////
// setup 関数
//////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // via PC10/PC11 (FTDI)
  motorL.init();
  motorR.init();

  // Remap TIMER3 to use 5V torelant pins.
  // TIM3_CH1: PC6=CN10-4, TIM3_CH2: PC7 = D9
  afio_remap(AFIO_REMAP_TIM3_FULL);

  // init encoders
  encL.init();
  encR.init();

#ifdef USE_LCD
    lcd.begin(16, 2);
    lcd.print("Hello,");
    lcd.setCursor(0, 1);
    lcd.print(" this is Melcy");
#endif
  
#if 1
  error = ps2x.config_gamepad(pinVSC1clk, pinVSC1cmd, pinVSC1atn, pinVSC1dat, true, true); //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error

  if (error == 0) {
    Serial.println(F("Found Controller, configured successful"));
    Serial.println(F("Try out all the buttons, X will vibrate the controller, faster as you press harder;"));
    Serial.println(F("holding L1 or R1 will print out the analog stick values."));
    Serial.println(F("Go to www.billporter.info for updates and to report bugs."));
  }

  else if (error == 1)
    Serial.println(F("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips"));

  else if (error == 2)
    Serial.println(F("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips"));

  else if (error == 3)
    Serial.println(F("Controller refusing to enter Pressures mode, may not support it. "));

  //Serial.print(Fps2x.Analog(1), HEX);

  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.println(F("Unknown Controller type"));
      break;
    case 1:
      Serial.println(F("DualShock Controller Found"));
      break;
    case 2:
      Serial.println(F("GuitarHero Controller Found"));
      break;
  }
#endif

  setGain();
}

//////////////////////////////////////////////////
// loop 関数
//////////////////////////////////////////////////
void loop() {
  unsigned long currentMillis = millis();
  bool doPS2 = false;

  if (currentMillis - previousMillis50 > 50) {
    previousMillis50 += 50;
    doPS2 = true;
    int32_t pL = encL.read();
    int32_t pR = - encR.read();
    int32_t velL = pL - posL;
    int32_t velR = pR - posR;

    posL = pL;
    posR = pR;

    Serial.print(posL);
    Serial.print("\t");
    Serial.print(posR);
    Serial.print("\t");

    Serial.print(refL);
    Serial.print("\t");
    Serial.print(refR);
    Serial.print("\t");
    Serial.print(velL);
    Serial.print("\t");
    Serial.print(velR);
    Serial.println();

    int32_t eL = refL - velL;
    int32_t eR = refR - velR;

    // P
    int32_t pwmL = Kp * eL;
    int32_t pwmR = Kp * eR;

    // I
    intL += (refL - velL);
    intR += (refR - velR);
    pwmL += Ki * intL;
    pwmR += Ki * intR;

    // D
    pwmL += Kd * (eL - peL);
    pwmR += Kd * (eR - peR);
    peL = eL;
    peR = eR;

    motorL.write(pwmL);
    motorR.write(-pwmR);
  }

  uint32_t cur = millis();

  // PC → マイコン への Kobuki パケットを受け取って処理する
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
        prevControlledMillis = currentMillis;
        setGain();
#ifdef USE_LCD
        // 液晶に現在の目標速度を表示
        //lcd.setCursor(0, 1);
        lcd.clear();
        lcd.print(fbRecv.spd);
        lcd.print(' ');
        lcd.print(fbRecv.radius);
        lcd.print(' ');
        lcd.print(refL);
        lcd.print(' ');
        lcd.print(refR);
#endif
      }
      fbRecv.reset();
      break;
    }
  }

  // マイコン → PC へ Kobuki パケットを送る
  if (cur - prevKobukiPacketMillis > feedbackPacketInterval) { // Send feedback every 20ms (50Hz);
    // PC へパケットを送る
    sendPacket();

    // 次に更新する時刻を決める
    prevKobukiPacketMillis += feedbackPacketInterval;
  }

#if 1
  // PS2 ゲームコントローラの処理
  /* You must Read Gamepad to get new values
    Read GamePad and set vibration values
    ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
    if you don't enable the rumble, use ps2x.read_gamepad(); with no values

    you should call this at least once a second
  */
  if (doPS2) {
    if (error == 1) //skip loop if no controller found
      return;

    if (type == 2) { //Guitar Hero Controller

      ps2x.read_gamepad();          //read controller

      if (ps2x.ButtonPressed(GREEN_FRET))
        Serial.println(F("Green Fret Pressed"));
      if (ps2x.ButtonPressed(RED_FRET))
        Serial.println(F("Red Fret Pressed"));
      if (ps2x.ButtonPressed(YELLOW_FRET))
        Serial.println(F("Yellow Fret Pressed"));
      if (ps2x.ButtonPressed(BLUE_FRET))
        Serial.println(F("Blue Fret Pressed"));
      if (ps2x.ButtonPressed(ORANGE_FRET))
        Serial.println(F("Orange Fret Pressed"));


      if (ps2x.ButtonPressed(STAR_POWER))
        Serial.println(F("Star Power Command"));

      if (ps2x.Button(UP_STRUM))         //will be TRUE as long as button is pressed
        Serial.println(F("Up Strum"));
      if (ps2x.Button(DOWN_STRUM))
        Serial.println(F("DOWN Strum"));


      if (ps2x.Button(PSB_START))                  //will be TRUE as long as button is pressed
        Serial.println(F("Start is being held"));
      if (ps2x.Button(PSB_SELECT))
        Serial.println(F("Select is being held"));


      if (ps2x.Button(ORANGE_FRET)) // print stick value IF TRUE
      {
        Serial.print(F("Wammy Bar Position:"));
        Serial.println(ps2x.Analog(WHAMMY_BAR), DEC);
      }
    }

    else { //DualShock Controller

      ps2x.read_gamepad(false, vibrate);          //read controller and set large motor to spin at 'vibrate' speed

      if (ps2x.Button(PSB_PAD_UP)) {        //will be TRUE as long as button is pressed
        prevControlledMillis = currentMillis;
        refL += 60;
        refR += 60;

        if (refL > 600)
          refL = 600;
        if (refR > 600)
          refR = 600;
        setGain();
      }
      if (ps2x.Button(PSB_PAD_RIGHT)) {
        prevControlledMillis = currentMillis;
        refL += 60;
        if (refL > 600)
          refL = 600;

        if (refR != 0) {
          if (refR < 0)
            refR += 60;
          else if (refR > 0)
            refR -= 60;
        }
        if (abs(refR) < 60)
          refR = 0;
        setGain();
      }
      if (ps2x.Button(PSB_PAD_LEFT)) {
        prevControlledMillis = currentMillis;
        refR += 60;
        if (refR > 600)
          refR = 600;

        if (refL != 0) {
          if (refL < 0)
            refL += 60;
          else if (refL > 0)
            refL -= 60;
        }
        if (abs(refL) < 60)
          refL = 0;
        setGain();
      }
      if (ps2x.Button(PSB_PAD_DOWN)) {
        prevControlledMillis = currentMillis;
        refL -= 60;
        refR -= 60;

        if (refL < -600)
          refL = -600;
        if (refR < -600)
          refR = -600;
        setGain();
      }


      vibrate = ps2x.Analog(PSAB_BLUE);        //this will set the large motor vibrate speed based on
      //how hard you press the blue (X) button

      if (ps2x.NewButtonState())               //will be TRUE if any button changes state (on to off, or off to on)
      {
        if (ps2x.Button(PSB_L2)) {
          Serial.println(F("L2 pressed"));
          refR = 0;
          refL = 0;
          resetGain();
        }
      }
    }
  }
#endif
  if ((currentMillis - prevControlledMillis >= 1000) || digitalRead(PC12) == LOW) {
    refR = 0;
    refL = 0;
    resetGain();
  }
}

//////////////////////////////////////////////////
// Kobuki のパケット処理
//////////////////////////////////////////////////

// マイコン → PC
void sendPacket() {
  // データの準備
  bs.t = millis();
  bs.leftEncoder  = encL.read(); // エンコーダの値
  bs.rightEncoder = -encR.read(); // エンコーダの値

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

// Kobuki 用の指令 → このロボット用の指令に変換
void setWheel(int16_t spd, int16_t radius) {
  float wL = .0, wR = .0;

  if (radius == 0) {
    wL = wR = spd; // [mm/s]
  } else if (radius == 1) {
    wL = -spd * (wheelW / KobukiWheelW);
    wR = spd * (wheelW / KobukiWheelW);
  } else {
    float r = radius;
    float v = spd;
    if (radius > 1)
      v = v / (r + KobukiWheelW / 2);
    else
      v = v / (r - KobukiWheelW / 2);
    float w = v / r;
    wL = (r - wheelW / 2.0) * v;
    wR = (r + wheelW / 2.0) * v;
  }

  // ここから mm/s を カウント/制御周期 に変換する

  // 車輪の半径で割る（車輪の速度 mm/s → 車輪の回転速度rad/s）
  // 車輪直径 210mm (半径 105mm)
  wL /= 105.0;
  wR /= 105.0;

  // ギア比をかける (車輪の回転速度rad/s → モータの回転速度rad/s)
  // モータ(SS40E2-E0-H3-12.5-DC24V-500P/R)のギアヘッド(8DG12.5)がギア比が 12.5
  // モータと車輪の間のギア比が 42:22
  // (小原歯車工業(KHK)のSS2-42 と SS2-22)
  wL *= (12.5 * (42.0 / 22.0));
  wR *= (12.5 * (42.0 / 22.0));

  // 2πで割って、モータ1回転当たりのエンコーダのカウント×4 をかける
  // (モータの回転速度 rad/s → モータの回転数 rot/s → カウント/s)
  // ×4 は４逓倍でマイコンがカウントしているから
  // HEDS-5500 A12 の場合は 500 count/rot の4倍
  wL = wL / (2.0 * M_PI) * (4*500);
  wR = wR / (2.0 * M_PI) * (4*500);

  // 制御周期(0.05s = 50ms) をかける
  wL *= 0.05;
  wR *= 0.05;

  // ここまで (mm/s を カウント/制御周期 に変換する)

  // 浮動小数点→整数の変換
  refL = (int16_t)(wL + .5);
  refR = (int16_t)(wR + .5);
}
