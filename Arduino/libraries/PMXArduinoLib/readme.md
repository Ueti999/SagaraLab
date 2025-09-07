
# PMX Arduino Library  

## 概要(Overview)
近藤科学製PMXサーボモータをArduinoのSerial(HardwareSerial)から動かすためのライブラリです。  
This library is for connecting a Kondo Kagaku PMX servo motor from an Arudino serial(HardwareSerial) port.

## 説明(Description)
近藤科学製PMXサーボモータをArduinoで動かすためのライブラリです。  
This library is for connecting a Kondo Kagaku PMX servo motor using Arudino.

Arduinoから
- PMXサーボモータにトルクON/OFFの指令を出すことができるようになります。  
  You will be able to send torque ON/OFF commands to the PMX servo motor.

- PMXサーボモータから目標位置や目標電流を指令できるようになります。  
  You will be able to command goal position and goal current from the PMX servo motor.

- PMXサーボモータの現在位置や電流情報を取得できるようになります。  
  You will be able to retrieve current position and current information from the PMX servo motor.

...etc


ArduinoからはSerial(HardwareSerial)を用いて通信をします。  
Communications can be performed from Arudino using a serial port (HardwareSerial).

- PmxBaseClass  
  PMXサーボモータのコマンド生成や定義  
  PMX servo motor command generation and definition

- PmxHardSerialClass  
  HardSerialを使用してPMXサーボモータの通信をする  
  Communicate with PMX servo motor using HardSerial

- PmxCRC  
  PMXで使用するCRC16を計算する  
  Calculate CRC16 for use with PMX

- DataConvert  
  byte配列をint16やuint32等に変換する  
  Converting a byte array to int16 or uint32, etc

## 更新履歴(Revision History)
### V1.0.0 (2023/12)
 - First Release
### V1.0.1 (2023/12)
- MemREADのサンプルプログラムを一部修正しました  
  Modifications were made to the MemREAD sample program. 
### V1.0.2 (2024/02)
- RamAddrListにショートブレーキ設定(530~)および目標指令値(700番台)のアドレスを追加しました  
  RamAddrList add [short brake],[GoalCommandValue]
- MotorWriteコマンドを使用せずMotorのON/OFFが設定する「setTorqueSwitch」関数を追加しました  
  Add function [setTorqueSwitch]
- 一部スペルミスがあったので修正しました。  
  Fixed some spelling mistakes.
- 700番台を使用する「MemWRITE_MotorControl_Sample」のサンプルプログラムを追加しました  
  Add sample program [MemWRITE_MotorControl_Sample]

### V1.0.3(2024/07)
- PMXファームウェアバージョンV.1.1.0.0に対応する下記の機能を追加しました。   
  Added functionality compatible with PMX V.1.1.0.0.
  - 制御モードの組み合わせが追加されました  
    Added control mode combinations
  
  - RamAddrListに制御ゲイン、プリセットゲイン、LED点灯モードのアドレスを追加しました  
    Added address to RamAddrList
  - getPresetAllPidGainsほか制御ゲイン、プリセットゲイン関係の関数を追加しました  
    Added getPresetAllPidGains and other functions related to control gain and preset gain.
  - LED点灯モードに対応するgetLedMode、setLedMode関数を追加しました  
    Added getLedMode and setLedMode functions corresponding to LED lighting mode
  - エラーリストに実行エラーを追加しました  
    Added [Run error] to error list
  
- その他軽微なバグを修正しました   
  Fixed minor bugs 


## Requirement
- Arduino Nano Every (Serial1)  
- Arduino Uno R3 (Serial)  
  ※Not use Arduino Uno R4(ArduinoUnoR4BoardsV1.0.5)   
- M5StackBasic(V2.6) (Serial1)  
- Teensy4.0 / 4.1 (Serial1 - Serial5)   
   (unconfirmed Serial6,Serial7)




## 使い方(Usage)
PMXの仕様については下記ページをご覧ください。  
Please refer to the following page for specifications of the PMX servo motor.  
[PMX Online Manural] (https://kondo-robot.com/faq/pmx-servo-series-online-manual)



ArduinoからアクセスできるPmxHardSerialClassを使うにはPmxBaceClassをリンクできるようにしてください。
To use the PmxHardSerialClass accessible from Arduino, ensure that you can link the PmxBaseClass.

他のマイコンを使いたい場合は、PmxBaceClassを派生させると便利です。  
To use another computer, deriving PmxBaceClass is a convenient approach.

サンプルプログラムではArduino Nano Everyを使用しています。　　
The sample program uses Arduino Nano Every.  

マイコンや接続方法によってSerialの番号が変わってきます。  
The serial number varies depending on the microcontroller and connection method.  

他のマイコンを使う場合はSerialの設定を変更してください。  
If you use another microcontroller, please change the Serial settings.

## Licence
Copyright 2024 Kondo Kagaku co.,ltd.  
[MIT](http://opensource.org/licenses/mit-license.php)
see to MIT_Licence.txt


## Author
近藤科学株式会社  
Kondo Kagaku co.,ltd.  
近藤科学ホームページ:(<http://kondo-robot.com/>)
