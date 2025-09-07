//
//  @file SystemWRITE_IDChange_Sample20240226.ino
//  @brief SystemWRITE sample code 
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2024/02/26
//  @copyright Kondo Kagaku Co.,Ltd. 2024
//

//   
//   SystemWRITE_IDChange_Sample20240226.inoはPMXのSystemWRITEコマンドを実行するサンプルコードです。
//   SystemWRITE_IDChange_Sample20240226.ino is a sample code that executes functions related to the SystemREAD and SystemWRITE functions of PMX.
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
const int TIMEOUT = 2000;     // タイムアウトまでの時間[ms]
                              // ※SystemWRITEはサーボからの返事を受け取るまでに時間がかかるため、timeoutを長めに設定しておく

// インスタンス＋ENピン(2番ピン)およびUARTの指定
PmxHardSerial pmx(&Serial1,EN_PIN,BAUDRATE,TIMEOUT);

void setup() {

  Serial.begin(115200);   // PCと通信を開始する

  delay(2000);   // サーボが起動するまで少し待つ

  pmx.begin();  // サーボモータの通信初期設定

  uint16_t flag;

  // SystemWRITEコマンドを実行し、サーボのID番号を変更します。
  Serial.println("SystemWRITEでID番号を変更します");

  // ※手元にUSBアダプターがないなど特別な理由がない限り以下の処理はPCからサーボマネージャを使用して書き換えることをお勧めします。

  byte ServoID = 0;   // 現在のサーボのID番号
  byte newID = 2;    // 新しいID番号

  // サーボのID番号を変更するための専用の関数です。
  // この関数内でSystemWRITEコマンドが実行されます。
  flag = pmx.setId(ServoID, newID);
  Serial.println("SystemWRITEコマンドを実行しました。");


  if(flag == 0x00){ //データが返ってきた場合は正常

    delay(1000);   // サーボの処理が完了するまで待つ

    // ID番号が正常に変更されたかを確認するために、新しいID番号でトルクスイッチの状態を取得してみます。

    byte trqSwitchData;   // 読み出したデータを格納するための変数
    flag = pmx.getTorqueSwitch(newID, &trqSwitchData); // トルクスイッチの状態を読み出す

    if(flag == 0){
      Serial.println("ID番号が正常に書き換わりました。");
      Serial.print("変更前=");
      Serial.println(ServoID);
      Serial.print("新しいID=");
      Serial.println(newID);
    }
    else{
      Serial.println("変更後のIDを読み取りましたが異常でした");
      Serial.println("ID番号が正常に書き換わっていない可能性があります。エラー状態を確認してください。");
      Serial.print("flag=");
      Serial.println(flag, HEX);   // flagの状態を表示　0であれば異常なし
    }
  }
  else{ //setID関数を呼んだが異常だった
    Serial.println("SystemWRITEを実行しましたがエラーでした");
    Serial.println("ID番号が正常に書き換わっていない可能性があります。エラー状態を確認してください。");
    Serial.print("flag=");
    Serial.println(flag, HEX);   // flagの状態を表示　0であれば異常なし
  }

}


void loop() {

}
