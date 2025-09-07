//
//  @file MotorWRITE_PosTime_Sample20240220.ino
//  @brief MotorREAD MotorWRITE(Double) sample code 
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2024/2/20
//  @copyright Kondo Kagaku Co.,Ltd. 2024
//

//   
//   MotorWRITE_Double_Sample20240220.inoはPMXのMotorREADコマンド、MotorWRITEコマンドに関する関数を実行するサンプルコードです。
//   MotorWRITE_PosTime_Sample20240220.ino is a sample code that executes functions related to the MotorREAD and MotorWRITE functions of PMX.
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
byte controlMode = PMX::ControlMode::Position + PMX::ControlMode::Time;    // 位置制御モード値+移動時間
//byte controlMode = 0x01 + 0x20  // 数値でも指定できます。

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

  // 均等補間モードを指定
  byte writeOpt = 1;
  flag = pmx.setTrajectory(ServoID, PMX::TrajectoryType::Even, writeOpt);
  Serial.print("setTrajectory=");
  Serial.println(flag, HEX);

  // MotorWRITE関数で動作指令
  long pos = 0;      // 目標位置
  long time = 1000;       // 移動時間
  int writeDataCount = 2;
  long writeDatas[writeDataCount] = {pos, time};
  flag = pmx.MotorWRITE(ServoID, writeDatas, writeDataCount, receiveMode, receiveData, controlMode);
  Serial.print("MotorWRITE=");
  Serial.println(flag, HEX);

  // 目標地点に到達するまで待ちます
  delay(2000);


  // 5次多項式補間モードを指定
  flag = pmx.setTrajectory(ServoID, PMX::TrajectoryType::FifthPoly, writeOpt);
  Serial.print("setTrajectory=");
  Serial.println(flag, HEX);

  // MotorWRITEDouble（組合せ2個）で動作指令
  long targetVal1 = 18000; // 目標位置
  long targetVal2 = 2000;   // 移動時間
  flag = pmx.MotorWRITEDouble(ServoID, targetVal1, targetVal2, receiveMode, receiveData, controlMode);
  Serial.print("MotorWRITE-Double=");
  Serial.println(flag, HEX);

  // 目標地点に到達するまで待ちます
  delay(3000);


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
