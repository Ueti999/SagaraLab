//
//  @file MotorWRITE_Single_Sample20231219.ino
//  @brief MotorREAD MotorWRITE(Single) sample code 
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2023/12/19
//  @copyright Kondo Kagaku Co.,Ltd. 2023
//

//   
//   MotorWRITE_Single_Sample20231219.inoはPMXのMotorREADコマンド、MotorWRITEコマンドに関する関数を実行するサンプルコードです。
//   MotorWRITE_Single_Sample20231219.ino is a sample code that executes functions related to the MotorREAD and MotorWRITE functions of PMX.
//

//
//  Sample Board : ArduinoNanoEvery
//  NanoEvery <=> RS-485Board
//  RX(PC5)   <=  R  (Serial1)
//  TX(PC4)    => D  (Serial1)
//  D2(PA0)    => EN_IN
//  VIN       <=  VOUT
//  GND       <=> GND
//  +5V        => IOREF
//


#include <Arduino.h>

#include <PmxHardSerialClass.h>
#include <DataConvert.h>

// サーボとArduino間の通信設定
const byte EN_PIN = 2;        // EN(enable)ピンのピン番号
const long BAUDRATE = 115200; // 通信速度[bps]
const int TIMEOUT = 1000;      // タイムアウトまでの時間[ms]

// インスタンス＋ENピン(2番ピン)およびUARTの指定
PmxHardSerial pmx(&Serial1,EN_PIN,BAUDRATE,TIMEOUT);

const byte ServoID = 0;   //サーボのID番号

// 制御モードを指定する
// 以下の各ビットを1にすると制御モードとして指定できます
// Bit7:0 Bit6:0 Bit5:移動時間 Bit4:PWM Bit3:トルク Bit2:電流 Bit1:速度 Bit0:位置
byte controlMode = PMX::ControlMode::Position;    // 位置制御モード値
//byte controlMode = 0x01  // 数値でも指定できます。

// 応答データを指定する
// 以下の各ビットを1にすると応答データとして指定できます
// Bit7:入力電圧 Bit6:CPU温度 Bit5:モータ温度 Bit4:PWM出力 Bit3:推定トルク Bit2:電流 Bit1:速度 Bit0:位置
// receiveMode = PMX.ReceiveDataOption.Position # 位置情報を返す
byte receiveMode = PMX::ReceiveDataOption::Full;  // すべて返す
//byte receiveMode = 0xFF  // 数値でも指定できます。



void setup() {

  Serial.begin(115200);   // PCと通信を開始する

  delay(500);   // サーボが起動するまで少し待つ
  pmx.begin();  // サーボモータの通信初期設定

  uint16_t flag;

  // 制御モードの設定します
  // MemWRITEコマンドで第三引数のwriteOptが0x01のときはトルクONでも強制的に書き込めます
  byte writeOpt = 1;
  flag = pmx.setControlMode(ServoID, controlMode, writeOpt);
  Serial.print("setControlMode=");
  Serial.println(flag, HEX);

  // 応答データ指定します
  flag = pmx.setMotorReceive(ServoID, receiveMode, writeOpt);
  Serial.print("setMotorReceive=");
  Serial.println(flag, HEX);

  // サーボのトルクをオンにします
  long receiveData[8];
  flag = pmx.setMotorTorqueOn(ServoID, receiveMode, receiveData, controlMode);
  Serial.print("setMotorTorqueOn=");
  Serial.println(flag, HEX);
  
  /*
  // サーボのトルクをオフにする場合は下記の関数を実行します    
  flag = pmx.setMotorFree(ServoID, receiveMode, receiveData, controlMode);
  Serial.print("setMotorFree=");
  Serial.println(flag, HEX);
  */

}


void loop() {

  uint16_t flag;
  long receiveData[8];

   // MotorWRITE関数で動作指令
  long pos = 0;  // 目標位置
  int writeDataCount = 1;
  long writeDatas[1] = {pos};
  flag = pmx.MotorWRITE(ServoID, writeDatas, writeDataCount, receiveMode, receiveData, controlMode);
  Serial.print("MotorWRITE=");
  Serial.println(flag, HEX);

  // 目標地点に到達するまで待ちます
  delay(2000);


  // MotorWRITESingle（組合せ1個）で動作指令
  long targetVal = 18000;  // 目標位置
  flag = pmx.MotorWRITESingle(ServoID, targetVal, receiveMode, receiveData, controlMode);
  Serial.print("MotorWRITE-Single=");
  Serial.println(flag, HEX);

  // 目標地点に到達するまで待ちます
  delay(1000);


  // MotorREADで現在の状態を確認
  byte torqueSw;
  flag = pmx.MotorREAD(ServoID, receiveMode, receiveData, controlMode, &torqueSw);
  Serial.print("MotorREAD");
  Serial.println(flag, HEX);

  Serial.print("TorqueSwitch=");
  Serial.println(torqueSw);
  Serial.print("NowPosition=");
  Serial.println(receiveData[0]);
  Serial.print("NowSpeed=");
  Serial.println(receiveData[1]);
  Serial.print("NowCurrent=");
  Serial.println(receiveData[2]);
  Serial.print("NowTorque=");
  Serial.println(receiveData[3]);
  Serial.print("NowPWM=");
  Serial.println(receiveData[4]);
  Serial.print("NowMotorTemp=");
  Serial.println(receiveData[5]);
  Serial.print("NowCpuTemp=");
  Serial.println(receiveData[6]);
  Serial.print("NowVoltage=");
  Serial.println(receiveData[7]);

}
