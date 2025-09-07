//
//  @file MemWRITE_MotorControl_Sample20240220.ino
//  @brief MemWRITE sample code 
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2024/02/220
//  @copyright Kondo Kagaku Co.,Ltd. 2024
//

//   
//   MemWRITE_MotorControl_Sample20240220.inoはPMXのMemWRITE関数を使用してモータを制御するサンプルコードです。
//   MemWRITE_MotorControl_Sample20240220.ino is a sample code that controls a motor using the MemWRITE function of PMX.//

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
const byte EN_PIN = 32;        // EN(enable)ピンのピン番号
const long BAUDRATE = 115200; // 通信速度[bps]
const int TIMEOUT = 1000;      // タイムアウトまでの時間[ms]

// インスタンス＋ENピン(2番ピン)およびUARTの指定
PmxHardSerial pmx(&Serial1,EN_PIN,BAUDRATE,TIMEOUT);

const byte ServoID = 0;   //サーボのID番号

const byte ForcedWriteOpt = 0x01; //MemWriteのWriteオプションの強制書き込みモード値

void setup() {

  Serial.begin(115200);   // PCと通信を開始する

  delay(500);   // サーボが起動するまで少し待つ
  pmx.begin();  // サーボモータの通信初期設定

  uint16_t flag;


  // 制御モードを指定する
  // 以下の各ビットを1にすると制御モードとして指定できます
  // Bit7:0 Bit6:0 Bit5:移動時間 Bit4:PWM Bit3:トルク Bit2:電流 Bit1:速度 Bit0:位置
  Serial.println("位置制御モードを指定");
  byte controlMode = PMX::ControlMode::Position;    // 位置制御モード値
  //byte controlMode = 0x01  // 数値でも指定できます。

  // 制御モードの設定します
  // MemWRITEコマンドで第三引数のwriteOptが0x01(ForcedWriteOpt)のときはトルクONでも強制的に書き込めます
  flag = pmx.setControlMode(ServoID, controlMode, ForcedWriteOpt);
  Serial.print("setControlMode=");
  Serial.println(flag, HEX);


  // MemWRITEコマンドでサーボのトルクをONにします
  Serial.println("トルクオンします");
  flag = pmx.setTorqueSwitch(ServoID, PMX::TorqueSwitchType::TorqueOn, ForcedWriteOpt);
  Serial.print("flag=");
  Serial.println(flag, HEX);


  Serial.println("位置制御の指令値として90°を指定");
  flag = pmx.MemWRITEToInt16(ServoID, PMX::RamAddrList::GoalCommandValue1, 9000, ForcedWriteOpt);
  Serial.print("flag=");
  Serial.println(flag, HEX);

  delay(1000);  //目標位置に到着するまで待つ

  // MemREADでサーボのデータを読み出します
  long posData;
  flag = pmx.getPosition(ServoID, &posData, controlMode);
  Serial.print("現在位置:");
  Serial.println(posData);

  short spdData;
  flag = pmx.getSpeed(ServoID, &spdData);
  Serial.print("現在速度:");
  Serial.println(spdData);

  short crtData;
  flag = pmx.getCurrent(ServoID, &crtData);
  Serial.print("現在電流値:");
  Serial.println(crtData);

  Serial.println("位置制御の指令値として0°を指定");
  pmx.MemWRITEToInt16(ServoID, PMX::RamAddrList::GoalCommandValue1, 0, ForcedWriteOpt);
  Serial.print("flag=");
  Serial.println(flag, HEX);

  delay(2000);



  ////　制御モードが2つの場合 ////
  
  // 誤動作を防ぐため、制御モードを切り替えるまえにBrakeモードに切り替える
  Serial.println("モードを切り替えるためBrake状態にしておきます");
  flag = pmx.setTorqueSwitch(ServoID, PMX::TorqueSwitchType::Brake, ForcedWriteOpt);
  Serial.print("flag=");
  Serial.println(flag, HEX);

  // 制御モードの設定します
  Serial.println("位置制御+電流制御モードを指定");
  controlMode = PMX::ControlMode::PositionCurrent;    // 位置制御モード値 + 電流制御モード値
  flag = pmx.setControlMode(ServoID, controlMode, ForcedWriteOpt);
  Serial.print("setControlMode=");
  Serial.println(flag, HEX);

  // MemWRITEコマンドでサーボのトルクをONにします
  Serial.println("トルクオンします");
  flag = pmx.setTorqueSwitch(ServoID, PMX::TorqueSwitchType::TorqueOn, ForcedWriteOpt);
  Serial.print("flag=");
  Serial.println(flag, HEX);

  Serial.println("電流制御の指令値として200mAを指定");
  flag = pmx.MemWRITEToInt16(ServoID, PMX::RamAddrList::GoalCommandValue2, 200, ForcedWriteOpt);
  Serial.print("flag=");
  Serial.println(flag, HEX);

  Serial.println("位置制御の指令値として90°を指定");
  flag = pmx.MemWRITEToInt16(ServoID, PMX::RamAddrList::GoalCommandValue1, 9000, ForcedWriteOpt);
  Serial.print("flag=");
  Serial.println(flag, HEX);

  delay(1500);

  Serial.println("位置制御の指令値として0°を指定");
  pmx.MemWRITEToInt16(ServoID, PMX::RamAddrList::GoalCommandValue1, 0, ForcedWriteOpt);
  Serial.print("flag=");
  Serial.println(flag, HEX);

  delay(3000);



  ////　移動時間と組み合わせた位置制御の場合 ////
  
  // 誤動作を防ぐため、制御モードを切り替えるまえにBrakeモードに切り替える
  Serial.println("モードを切り替えるためBrakeにしておきます");
  flag = pmx.setTorqueSwitch(ServoID, PMX::TorqueSwitchType::Brake, ForcedWriteOpt);
  Serial.print("flag=");
  Serial.println(flag, HEX);

  // 制御モードの設定します
  Serial.println("位置制御+移動時間モードを指定");
  controlMode = PMX::ControlMode::PositionTime;    // 位置制御モード値 + 移動時間
  flag = pmx.setControlMode(ServoID, controlMode, ForcedWriteOpt);
  Serial.print("setControlMode=");
  Serial.println(flag, HEX);
  
  // 補間モードを指定します。初期値で「均等補間」に設定されているため「均等補間」で動かす場合は省くことができます。
  Serial.println("補間モードを均等補間に指定");
  flag = pmx.setTrajectory(ServoID, PMX::TrajectoryType::Even, ForcedWriteOpt);  //均等補間を指定する場合
  //flag = pmx.setTrajectory(ServoID, PMX::TrajectoryType::FifthPoly, ForcedWriteOpt);  //5次多項式補間を指定する場合
  Serial.print("setTrajectory=");
  Serial.println(flag, HEX);

  // MemWRITEコマンドでサーボのトルクをONにします
  flag = pmx.setTorqueSwitch(ServoID, PMX::TorqueSwitchType::TorqueOn, ForcedWriteOpt);
  Serial.print("flag=");
  Serial.println(flag, HEX);

  Serial.println("移動時間の指令値として1000msを指定");
  flag = pmx.MemWRITEToUint16(ServoID, PMX::RamAddrList::GoalCommandValue2, 1000, ForcedWriteOpt);
  Serial.print("flag=");
  Serial.println(flag, HEX);

  Serial.println("位置制御の指令値として90°を指定");
  flag = pmx.MemWRITEToInt16(ServoID, PMX::RamAddrList::GoalCommandValue1, 9000, ForcedWriteOpt);
  Serial.print("flag=");
  Serial.println(flag, HEX);

  delay(2000);

  Serial.println("位置制御の指令値として0°を指定");
  pmx.MemWRITEToInt16(ServoID, PMX::RamAddrList::GoalCommandValue1, 0, ForcedWriteOpt);
  Serial.print("flag=");
  Serial.println(flag, HEX);

  delay(2000);


  // MemWRITEコマンドでサーボのトルクをFreeにします
  Serial.println("Freeにします");
  flag = pmx.setTorqueSwitch(ServoID, PMX::TorqueSwitchType::Free, ForcedWriteOpt);
  Serial.print("flag=");
  Serial.println(flag, HEX);

}


void loop() {

}
