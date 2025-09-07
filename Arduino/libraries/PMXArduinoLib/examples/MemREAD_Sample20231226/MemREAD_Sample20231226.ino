//
//  @file MemREAD_Sample20231219.py
//  @brief MemREAD sample code 
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2023/12/19
//  @copyright Kondo Kagaku Co.,Ltd. 2023
//

//  
//   MemREADSample_20231219.inoはPMXのMemREADコマンドに関する関数を実行するサンプルコードです。
//   MemREADSample_20231219.ino is a sample code that executes functions related to the MemREAD function of PMX.
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
const byte EN_PIN = 32;        // EN(enable)ピンのピン番号
const long BAUDRATE = 115200; // 通信速度[bps]
//const long BAUDRATE = 3000000; // 通信速度[bps]

const int TIMEOUT = 1000;      // タイムアウトまでの時間[ms]

// インスタンス＋ENピン(2番ピン)およびUARTの指定
PmxHardSerial pmx(&Serial1,EN_PIN,BAUDRATE,TIMEOUT);

const byte ServoID = 0;   //サーボのID番号

void setup() {

  Serial.begin(115200);   // PCと通信を開始する

  delay(500);   // サーボが起動するまで少し待つ
  pmx.begin();  // サーボモータの通信初期設定

  uint16_t flag;

  unsigned short address = 0;

  // MemRead関数を直接使ってステータス情報を呼び出す(返信データはbyte配列)
  // 300番地はサーボの状態を読み出すことができます。
  // 6bytes読み出すことで現在位置、現在速度、現在電流値を一気に読み出せます。ただし、動作していませんので現在速度は0、現在電流値は微量な数値になります。
  Serial.println("300番地(現在位置)から6byte読み出します(返り値はbytes型)");
  byte rxData[6];   // サーボから読み出したデータを受け取るための配列
  flag = pmx.MemREAD(ServoID, PMX::RamAddrList::NowPosition, 6, rxData);  // MemREADコマンドを実行
  if(flag == 0)   // flagが0だった時正常にデータが返ってきている
  {
    for(int i = 0; i < 5; i++){
      Serial.print(rxData[i]);  // 読み出したデータをそのまま表示
      Serial.print(", ");
    }
    Serial.println(rxData[5]);
  }
  else
  {
    Serial.print("Error=");
    Serial.println(flag, HEX);
  }

  // pmx.MemREAD()で読み出したデータはリトルエンディアンで分割されています。以下の関数で一つのデータに連結することができます。
  // 例として現在位置のみ連結します。
  if(flag == 0)   // flagが0だった時正常にデータが返ってきている
  {
    int posData;
    byte rxPosByte[2] = {rxData[0], rxData[1]};   // 現在位置のデータのみ抜粋
    posData = DataConv::bytesToInt16(rxPosByte);  // 2Byteのデータを連結
    Serial.print("Position Data=");
    Serial.println(posData);
  }


  // pmx.MemREAD()の場合は分割されたデータを受け取るため連結し直す必要がありますが、下記の関数は戻り値で連結したデータを受け取ることができます。
  // MemREADで様々な型でデータを取得することができます。詳しくは下記アドレスを参照してください
  // https://www.kondokagaku.jp/wp_mn/archives/1144#6

  // 各データの型は「メモリマップ一覧」を参照してください。
  // https://www.kondokagaku.jp/wp_mn/archives/1144#i-36

  // MemREAD関数を使ってを現在位置（単位[1/100度]）(2byte符号あり)を取得する
  Serial.println("2byte(signed)の位置データを取得します");
  short int16Data;   // サーボから読み出したデータを受け取るための変数
  flag = pmx.MemREADToInt16(ServoID, PMX::RamAddrList::NowPosition, &int16Data);
  if(flag == 0)
  {
    Serial.print("int16Data=");
    Serial.println(int16Data);
  }
  else
  {
    Serial.print("Error=");
    Serial.println(flag, HEX);
  }


  // MemRead関数を使って電圧(単位[mV])(2byte符号なし)を取得する
  Serial.println("2byte(unsigned)の電圧データを取得します");
  unsigned short uint16Data;   // サーボから読み出したデータを受け取るための変数
  flag = pmx.MemREADToUint16(ServoID, PMX::RamAddrList::InputVoltage, &uint16Data);
  if(flag == 0)
  {
    Serial.print("uint16Data=");
    Serial.println(uint16Data);
  }
  else
  {
    Serial.print("Error=");
    Serial.println(flag, HEX);
  }


  // MemRead関数を使って位置制御のPゲインを取得する
  Serial.println("4byte(unsigned)の位置制御のPゲインを取得します");
  unsigned long uint32Data;   // サーボから読み出したデータを受け取るための変数
  flag = pmx.MemREADToUint32(ServoID, PMX::RamAddrList::PositionKp, &uint32Data);
  if(flag == 0)
  {
    Serial.print("uint32Data=");
    Serial.println(uint32Data);
  }
  else
  {
    Serial.print("Error=");
    Serial.println(flag, HEX);
  }


  // MemRead関数を使ってトルクスイッチの状態を取得する
  Serial.println("1byte(unsigned)のトルクスイッチの状態を取得します");
  // TorqueSwitch Data: 0x01=TorqueON, 0x02=Free, 0x04=Brake, 0x08=Hold
  byte byteData;   // サーボから読み出したデータを受け取るための変数
  flag = pmx.MemREADToByte(ServoID, PMX::RamAddrList::TorqueSwitch, &byteData);
  if(flag == 0)
  {
    Serial.print("byteData=");
    Serial.println(byteData);
  }
  else
  {
    Serial.print("Error=");
    Serial.println(flag, HEX);
  }


  // ライブラリには専用の関数を用意している場合があります。
  // 下記はトルクスイッチの状態を読み出すための関数です。これにより、アドレスを指定する必要がなくなり、プログラムもすっきりします。

  // トルクスイッチの状態を取得する
  Serial.println("トルクスイッチの状態を専用の関数で読み出します");
  flag = pmx.getTorqueSwitch(ServoID, &byteData);
  if(flag == 0)
  {
    Serial.print("byteData=");
    Serial.println(byteData);
  }
  else
  {
    Serial.print("Error=");
    Serial.println(flag, HEX);
  }


  // エラー情報を読み出し、エラーを解除します
  // エラー情報を読み出すと解除されます
  // getFullStatus関数を使用することでエラー情報を項目に分けて読み出すことができます。
  // 2ByteデータのRAMアクセスエラーはまとまった状態で受け取ることができます。
  Serial.println("エラーステータス(400番地から6byte)からエラー情報を読み出します");
  byte sysSt;           // システムエラーを受け取る変数
  byte motorSt;         // モータエラーを受け取る変数
  unsigned short ramSt; // RAMアクセスエラーを受け取る変数
  flag = pmx.getFullStatus(ServoID, &sysSt, &motorSt, &ramSt);  // エラー状態を読み出す
  Serial.print("ステータスエラー"); // 各データを表示
  Serial.println(flag);
  Serial.print("システムエラー=");
  Serial.println(sysSt);
  Serial.print("モータエラー=");
  Serial.println(motorSt);
  Serial.print("RAMアクセスエラー=");
  Serial.println(ramSt);

}


void loop() {

}
