//
//  @file LOAD_Sample20231219.ino
//  @brief PMX Load sample code 
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2023/12/19
//  @copyright Kondo Kagaku Co.,Ltd. 2023
//

//   
//   LOAD_Sample20231219.inoはPMXのLOADコマンドを実行するサンプルコードです。
//   LOAD_Sample20231219.ino is a sample code that executes PMX's LOAD functions.
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


void setup() {

  Serial.begin(115200);   // PCと通信を開始する

  delay(500);   // サーボが起動するまで少し待つ
  pmx.begin();  // サーボモータの通信初期設定

  uint16_t flag;

  // LOAD関数を使ってROMに保存されたデータをRAMに展開する
  Serial.println("ROMに保存されたデータをRAMに展開します");

  flag = pmx.LOAD(ServoID);  // LOADコマンドを実行
  
  if(flag == 0){
    Serial.println("LOADコマンドが正常に動作しました");
  }
  else{
    Serial.println("LOADコマンドを送信後にエラーが返ってきました");
  }
  Serial.print("flag=");
  Serial.println(flag, HEX);

  // サーボのLOADコマンドの処理が完了するまで待つ
  delay(500);

}


void loop() {

}
