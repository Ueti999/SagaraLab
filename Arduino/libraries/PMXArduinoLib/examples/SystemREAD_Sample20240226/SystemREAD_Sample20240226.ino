//
//  @file SystemREAD_Sample20240226.ino
//  @brief SystemREAD sample code 
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2024/02/26
//  @copyright Kondo Kagaku Co.,Ltd. 2024
//

//   
//   SystemREAD_Sample20240226.inoはPMXのSystemREADコマンドを実行するサンプルコードです。
//   SystemREAD_Sample20230226.ino is a sample code that executes functions related to the SystemREAD functions of PMX.
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

  // SystemREADコマンドを実行し、サーボのデータを取得します。
  Serial.println("SystemReadでデータを取得します");
  
  byte systemReadByte[13];
  flag = pmx.SystemREAD(ServoID, systemReadByte);

  if(flag == 0){
    byte serialNumByte[4] = {systemReadByte[0], systemReadByte[1], systemReadByte[2], systemReadByte[3]};
    unsigned long serialNum;
    serialNum = DataConv::bytesToUint32(serialNumByte);  // 4Byteのデータを連結
    Serial.print("シリアル番号:");
    Serial.println(serialNum);
  }
  else{
    Serial.print("Error=");
    Serial.println(flag, HEX);
    Serial.println("エラーだったため以降の処理は中止しました");
    return;
  }

  // 専用の関数でシリアル番号のみを取得します。
  // 以下の関数で取得したシリアル番号をFactoryResetなどのコマンドで使用することができます。
  byte serialByteNum[4];    // サーボのシリアル番号を格納する配列
  flag = pmx.getSerialNumber(ServoID, serialByteNum);

  // シリアル番号の他にも以下の専用の関数で情報を取得することができます。
  unsigned short modelNum;
  unsigned short seriesNum;
  flag = pmx.getModelNum(ServoID, &modelNum, &seriesNum);
  Serial.print("モデル番号:");
  Serial.println(modelNum);
  Serial.print("シリーズ:");
  Serial.println(seriesNum);

  byte verData[4];
  flag = pmx.getVersion(ServoID, verData);
  Serial.print("ファームウェアバージョン:");
  Serial.print(verData[3]);
  Serial.print(verData[2]);
  Serial.print(verData[1]);
  Serial.println(verData[0]);

  byte respTime;
  flag = pmx.getResponseTime(ServoID, &respTime);
  Serial.print("応答時間:");
  Serial.println(respTime);

}


void loop() {

}
