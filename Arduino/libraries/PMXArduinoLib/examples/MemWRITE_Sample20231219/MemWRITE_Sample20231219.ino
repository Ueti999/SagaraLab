//
//  @file MemWRITE_Sample20231209.ino
//  @brief MemWRITE sample code 
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2023/12/19
//  @copyright Kondo Kagaku Co.,Ltd. 2023
//

//   
//   MemWRITE_Sample20231219.inoはPMXのMemWRITEコマンドに関する関数を実行するサンプルコードです。
//   MemWRITE_Sample20231219.ino is a sample code that executes functions related to the MemWRITE function of PMX.
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

  // MemWRITE関数を直接使って入力最小電圧値に9500を書き込む
  Serial.println("76番地(入力電圧最小値設定)に[9500](9.5V)を書き込みます");
  short newMinVoltageLimit = 9500;    // MemWRITEで新しく書き込むデータ
  byte txDataArray[2];
  DataConv::uint16ToBytes(newMinVoltageLimit, txDataArray); // データを2Byteに分割する
  flag = pmx.MemWRITE(ServoID, PMX::RamAddrList::MinVoltageLimit, txDataArray, 2, 0);  // MemWRITEコマンドを実行
  Serial.print("flag=");
  Serial.println(flag, HEX);


  byte rxDataArray[2];   // サーボから読み出したデータを受け取るための配列
  flag = pmx.MemREAD(ServoID, PMX::RamAddrList::MinVoltageLimit, 2, rxDataArray);  // MemREADコマンドを実行
  if(flag == 0)   // flagが0だった時正常にデータが返ってきている
  {
    int readData;
    readData = DataConv::bytesToInt16(rxDataArray); //2Byteの分割したデータを1つにまとめる
    Serial.print("readData=");
    Serial.println(readData);
  }
  else
  {
    Serial.print("Error=");
    Serial.println(flag, HEX);
  }


  // 下記の関数を使用することで、データを分割することなくMemWRITEを実行できる    
  // MemWRITE関数を使ってCW方向のリミット角度値(int16(符号あり2byte))に25000を書き込む
  // MemWRITE自体TorqueOn状態の場合は書き込めない仕様だが、オプションを0x01にすることによりTorqueOn状態でも書き込めるようになる

  // 各データの型は「メモリマップ一覧」を参照してください。
  // https://www.kondokagaku.jp/wp_mn/archives/1144#i-36

  Serial.println("CW方向最大角値に25000を書き込みます");
  pmx.MemWRITEToInt16(ServoID, PMX::RamAddrList::CwPositionLimit, 25000, 1);
  Serial.print("flag=");
  Serial.println(flag, HEX);

  // 同じアドレスのデータをMemREAD関数で読み出し、書き込んだデータが書き込めているか確認をする
  Serial.println("CW方向最大角値を読み出します");
  short int16Data;   // サーボから読み出したデータを受け取るための変数
  flag = pmx.MemREADToInt16(ServoID, PMX::RamAddrList::CwPositionLimit, &int16Data);
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


  // 符号なし2byte型の場合
  Serial.println("入力電圧最大値設定に13000を書き込みます");
  flag = pmx.MemWRITEToUint16(ServoID, PMX::RamAddrList::MaxVoltageLimit, 13000, 1);
  Serial.print("flag=");
  Serial.println(flag, HEX);

  // 同じアドレスのデータをMemREAD関数で読み出し、書き込んだデータが書き込めているか確認をする
  Serial.println("入力電圧最小値設定を読み出します");
  unsigned short uint16Data;   // サーボから読み出したデータを受け取るための変数
  flag = pmx.MemREADToUint16(ServoID, PMX::RamAddrList::MaxVoltageLimit, &uint16Data);
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


  // 符号なし4byte型の場合
  Serial.println("位置制御のPゲインに2000を書き込みます");
  flag = pmx.MemWRITEToUint32(ServoID, PMX::RamAddrList::PositionKp, 2000, 1);
  Serial.print("flag=");
  Serial.println(flag, HEX);

  // 同じアドレスのデータをMemREAD関数で読み出し、書き込んだデータが書き込めているか確認をする
  Serial.println("位置制御のPゲインを読み出します");
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


  // 符号なし1byte型の場合
  Serial.println("トルクスイッチに2(Free)を書き込みます");
  flag = pmx.MemWRITEToByte(ServoID, PMX::RamAddrList::TorqueSwitch, 2, 1);
  Serial.print("flag=");
  Serial.println(flag, HEX);

  // 同じアドレスのデータをMemREAD関数で読み出し、書き込んだデータが書き込めているか確認をする
  Serial.println("トルクスイッチを読み出します");
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
  // 下記は入力電圧最小値設定に書き込むための関数です。これにより、アドレスを指定する必要がなくなり、プログラムもすっきりします。
  // さらに関連する設定値と％の二つの項目を一行で書き込むことが可能です。

  // この関数内でMemWRITEコマンドを実行しています
  Serial.println("入力電圧最小値（入力電圧最小値と入力電圧最小時の出力％値）の設定をします");
  flag = pmx.setMinVoltageLimit(ServoID, 8000, 50);  // id , minVol , limPower(%)
  Serial.print("flag=");
  Serial.println(flag, HEX);

  // 入力電圧最小値の設定されているか確認
  Serial.println("入力電圧最小値（入力電圧最小値と入力電圧最小時の出力％値）を読み出します");
  unsigned short minVol;
  flag = pmx.getMinVoltageLimit(ServoID, &minVol);
  if(flag == 0)
  {
    Serial.print("minVol=");
    Serial.println(minVol);
  }
  else
  {
    Serial.print("Error=");
    Serial.println(flag, HEX);
  }

  unsigned short minVolPower;
  flag = pmx.getMinVoltageLimitPower(ServoID, &minVolPower);
  if(flag == 0)
  {
    Serial.print("minVolPower=");
    Serial.println(minVolPower);
  }
  else
  {
    Serial.print("Error=");
    Serial.println(flag, HEX);
  }


  // MemWRITEで書き込んだデータを保存する場合は以下のSAVEコマンドを実行してからサーボの電源を切ってください
  /*
  // SAVE関数を使ってRAMのデータをROMに保存する
  Serial.println("RAMのデータをROMに保存します");
  flag = pmx.SAVE(ServoID);  // SAVEコマンドを実行
  Serial.print("SAVE flag=");
  Serial.println(flag, HEX);
  
  // サーボのSAVEコマンドの処理が完了するまで待つ
  delay(500);
  */
}


void loop() {

}
