
/** 
* @file PmxBaseClass.h
* @brief  PMX library header file
* @author Kondo Kagaku Co.,Ltd.
* @author T.Nobuhara,Gotanda,hn.Kondo
* @date 2024/07/08
* @version 1.0.3
* @copyright Kondo Kagaku Co.,Ltd. 2024
* @warning コメントに不備がありますのでご注意ください
*
* @mainpage PmxArdinoLibの概要
* このライブラリは近藤科学製PMXサーボモータと通信するための関数になります<br>
* 使い方および詳細は、下記弊社HPをご覧ください。<br>
* <A HREF="http://kondo-robot.com/">http://kondo-robot.com/</A><br>
* 不具合等ありましたら、弊社HPを参照にご連絡ください。<br>
* PmxBaseは、PMXの基本となる部分を記載しています。
* 通信の違いを吸収するためHardwareSerialやSoftwareSerialに派生しています。


* @section changelog 更新履歴
* - 2023/12 : V1.0.0 
*     - First Release
* - 2023/12 : V1.0.1
*     - MemREADのサンプルプログラムを一部修正しました
* - 2024/02 : V1.0.2
*    - RamAddrListにショートブレーキ設定(530~)および目標指令値(700番台)のアドレスを追加しました
*    - MotorWriteコマンドを使用せずMotorのON/OFFが設定する「setTorqueSwitch」関数を追加しました
*    - 一部スペルミスがあったので修正しました
*    - 700番台を使用する「MemWRITE_MotorControl_Sample」のサンプルプログラムを追加しました
* - 2024/07 : V1.0.3
*    - PMXファームウェアバージョンV.1.1.0.0に対応する下記の機能を追加しました。
*    - RamAddrListに制御ゲイン、プリセットゲイン、LED点灯モードのアドレスを追加しました
*    - setAllPresetNumほか制御ゲイン、プリセットゲイン関係の関数を追加しました
*    - LED点灯モードに対応するgetLedMode、setLedMode関数を追加しました


**/	

#ifndef __Pmx_Base_Class_h__
#define __Pmx_Base_Class_h__

#include "Arduino.h"

/// @brief PMXの固有値の定義
/// @details PMXの固有値を定義しています。
namespace PMX
{ 

    static constexpr byte ErrorByteData = 0xFF;     //!< byte型のエラー値の定義
    static constexpr unsigned short ErrorUint16Data = 0x7FFF;   //!< byte型のエラー値の定義
    static constexpr unsigned long ErrorUint32Data = 0x7FFFFFFF;    //!< byte型のエラー値の定義

    ///
    /// @brief RAMのアドレス一覧
    ///
    namespace RamAddrList
    {
        constexpr unsigned short PositionKp = 0;    //!< 位置制御のPゲイン
        
        constexpr unsigned short PositionKi = 4;    //!< 位置制御のIゲイン
        
        constexpr unsigned short PositionKd = 8;    //!< 位置制御のDゲイン
        
        constexpr unsigned short PositionSt = 12;   //!< 位置制御のストレッチ


        constexpr unsigned short SpeedKp = 16;      //!< 速度制御のPゲイン

        constexpr unsigned short SpeedKi = 20;      //!< 速度制御のIゲイン

        constexpr unsigned short SpeedKd = 24;      //!< 速度制御のDゲイン


        constexpr unsigned short CurrentKp = 32;    //!< 電流制御のPゲイン

        constexpr unsigned short CurrentKi = 36;    //!< 電流制御のIゲイン

        constexpr unsigned short CurrentKd = 40;    //!< 電流制御のDゲイン


        constexpr unsigned short TorqueKp = 48;     //!< トルク制御のPゲイン

        constexpr unsigned short TorqueKi = 52;     //!< トルク制御のIゲイン

        constexpr unsigned short TorqueKd = 56;     //!< トルク制御のDゲイン


        constexpr unsigned short PositionDeadBand = 64;     //!< 位置制御　不感帯

        constexpr unsigned short SpeedDeadBand = 66;        //!< 速度制御　不感帯

        constexpr unsigned short CurrentDeadBand = 68;      //!< 電流制御　不感帯

        constexpr unsigned short TorqueDeadBand = 70;       //!< トルク制御　不感帯
        

        constexpr unsigned short CenterOffset = 72;     //!< 中央値オフセット

        constexpr unsigned short CloneReverse = 74;     //!< クローン/リバース


        constexpr unsigned short MinVoltageLimit = 76;      //!< 入力電圧最小値

        constexpr unsigned short MinVoltageLimitPower = 78; //!< 入力電圧最小時の出力％値

        constexpr unsigned short MaxVoltageLimit = 80;      //!< 入力電圧最大値

        constexpr unsigned short MaxVoltageLimitPower = 82; //!< 入力電圧最大時の出力％値

        constexpr unsigned short CurrentLimit = 84;         //!< モータ消費電流最大値

        constexpr unsigned short CurrentLimitPower = 86;    //!< モータ消費電流最大時の出力％値

        constexpr unsigned short MotorTempLimit = 88;       //!< モータ温度最大値

        constexpr unsigned short MotorTempLimitPower = 90;  //!< モータ温度最大時の出力％値

        constexpr unsigned short CpuTempLimit = 92;         //!< CPU温度最大値

        constexpr unsigned short CpuTempLimitPower = 94;    //!< CPU温度最大時の出力％値


        constexpr unsigned short CwPositionLimit = 96;      //!< CW方向最大角値

        constexpr unsigned short CwPositionLimitPower = 98; //!< CW方向最大角閾値外時の出力％値

        constexpr unsigned short CcwPositionLimit = 100;    //!< CCW方向最大角値

        constexpr unsigned short CcwPositionLimitPower = 102;   //!< CCW方向最大角閾値外時の出力％値

        constexpr unsigned short MaxGoalSpeed = 104;        //!< 最大速度指令値

        constexpr unsigned short MaxGoalCurrent = 106;      //!< 最大電流指令値

        constexpr unsigned short MaxGoalTorque = 108;       //!< 最大動推定トルク指令値

        constexpr unsigned short TotalPowerRate = 110;      //!< モータ出力制限％値

        constexpr unsigned short LockDetectTime = 112;      //!< ロック時間

        constexpr unsigned short LockThresholdPower = 114;  //!< ロックと認識される出力割合

        constexpr unsigned short LockDetectOutputPower = 116;   //!< ロック時間の出力％値
        

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short PresetPosAddr = 118;   //!< 位置制御ゲインプリセット(PMXのV1.1.0.0～)

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short PresetSpdAddr = 119;   //!< 速度制御ゲインプリセット(PMXのV1.1.0.0～)

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short PresetCurAddr = 120;   //!< 電流制御ゲインプリセット(PMXのV1.1.0.0～)

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short PresetTrqAddr = 121;   //!< トルク制御ゲインプリセット(PMXのV1.1.0.0～)


        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short PositionKp2 = 124;     //!< 位置制御のPゲイン2(PMXのV1.1.0.0～)
        
        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short PositionKi2 = 128;     //!< 位置制御のIゲイン2(PMXのV1.1.0.0～)
        
        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short PositionKd2 = 132;     //!< 位置制御のDゲイン2(PMXのV1.1.0.0～)
        
        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short PositionSt2 = 136;     //!< 位置制御のストレッチ2(PMXのV1.1.0.0～)


        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short SpeedKp2 = 140;        //!< 速度制御のPゲイン2(PMXのV1.1.0.0～)

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short SpeedKi2 = 144;        //!< 速度制御のIゲイン2(PMXのV1.1.0.0～)

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short SpeedKd2 = 148;        //!< 速度制御のDゲイン2(PMXのV1.1.0.0～)


        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short CurrentKp2 = 156;      //!< 電流制御のPゲイン2(PMXのV1.1.0.0～)

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short CurrentKi2 = 160;      //!< 電流制御のIゲイン2(PMXのV1.1.0.0～)

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short CurrentKd2 = 164;      //!< 電流制御のDゲイン2(PMXのV1.1.0.0～)


        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short TorqueKp2 = 172;       //!< トルク制御のPゲイン2(PMXのV1.1.0.0～)

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short TorqueKi2 = 176;       //!< トルク制御のIゲイン2(PMXのV1.1.0.0～)

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short TorqueKd2 = 180;       //!< トルク制御のDゲイン2(PMXのV1.1.0.0～)


        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short PositionKp3 = 188;     //!< 位置制御のPゲイン3(PMXのV1.1.0.0～)
        
        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short PositionKi3 = 192;     //!< 位置制御のIゲイン3(PMXのV1.1.0.0～)
        
        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short PositionKd3 = 196;     //!< 位置制御のDゲイン3(PMXのV1.1.0.0～)
        
        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short PositionSt3 = 200;     //!< 位置制御のストレッチ3(PMXのV1.1.0.0～)


        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short SpeedKp3 = 204;        //!< 速度制御のPゲイン3(PMXのV1.1.0.0～)

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short SpeedKi3 = 208;        //!< 速度制御のIゲイン3(PMXのV1.1.0.0～)

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short SpeedKd3 = 212;        //!< 速度制御のDゲイン3(PMXのV1.1.0.0～)


        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short CurrentKp3 = 220;      //!< 電流制御のPゲイン3(PMXのV1.1.0.0～)

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short CurrentKi3 = 224;      //!< 電流制御のIゲイン3(PMXのV1.1.0.0～)

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short CurrentKd3 = 228;      //!< 電流制御のDゲイン3(PMXのV1.1.0.0～)


        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short TorqueKp3 = 236;       //!< トルク制御のPゲイン3(PMXのV1.1.0.0～)

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short TorqueKi3 = 240;       //!< トルク制御のIゲイン3(PMXのV1.1.0.0～)

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short TorqueKd3 = 244;       //!< トルク制御のDゲイン3(PMXのV1.1.0.0～)


        constexpr unsigned short NowPosition = 300;     //!< 現在位置

        constexpr unsigned short NowSpeed = 302;        //!< 現在速度

        constexpr unsigned short NowCurrent = 304;      //!< 現在電流

        constexpr unsigned short NowTorque = 306;       //!< 出力推定トルク

        constexpr unsigned short NowPwm = 308;          //!< 現在のPWM割合

        constexpr unsigned short MotorTemp = 310;       //!< モータ温度

        constexpr unsigned short CPUTemp = 312;         //!< CPU温度

        constexpr unsigned short InputVoltage = 314;    //!< 入力電圧

        constexpr unsigned short TrajectoryTime = 316;  //!< 現在の補間時間
  
        constexpr unsigned short EncoderValue = 318;    //!< エンコーダの現在値


        constexpr unsigned short ErrorStatus = 400;     //!< エラーステータス

        constexpr unsigned short ErrorSystem = 401;     //!< システムエラー

        constexpr unsigned short ErrorMotor = 402;      //!< モータエラー

        constexpr unsigned short ErrorRamAccess = 404;  //!< RAMアクセスエラー時のアドレス


        constexpr unsigned short TorqueSwitch = 500;    //!< トルクスイッチ

        constexpr unsigned short ControlMode = 501;     //!< 制御モード
        
        constexpr unsigned short MotorReceiveData = 502;   //!< 応答データフラグ 
        
        constexpr unsigned short Trajectory = 503;      //!< 補間制御起動生成タイプ指定
        

        /// @remark PMXのV1.0.1.xから有効になります
        constexpr unsigned short ShortBrakeCurrent = 530;   //!< 電流制御時のショートブレーキ指定(PMXのV1.0.1.x～)

        /// @remark PMXのV1.0.1.xから有効になります
        constexpr unsigned short ShortBrakeTorque = 531;    //!< トルク制御時のショートブレーキ指定(PMXのV1.0.1.x～)
        
        /// @remark PMXのV1.0.1.xから有効になります
        constexpr unsigned short ShortBrakePWM = 532;       //!< PWM制御時のショートブレーキ指定(PMXのV1.0.1.x～)

        /// @remark PMXのV1.1.0.0から有効になります
        constexpr unsigned short LedMode = 533;             //!< LED点灯モード(PMXのV1.1.0.0～)


        constexpr unsigned short CenterOffsetMinRange = 600;    //!< 中央値オフセット　最小値
        
        constexpr unsigned short CenterOffsetMaxRange = 602;    //!< 中央値オフセット　最大値
        
        constexpr unsigned short MinVoltageMinRange = 604;      //!< 入力電圧最小制限値　最小値
        
        constexpr unsigned short MinVoltageMaxRange = 606;      //!< 入力電圧最小制限値　最大値
        
        constexpr unsigned short MaxVoltageMinRange = 608;      //!< 入力電圧最大制限値　最小値
        
        constexpr unsigned short MaxVoltageMaxRange = 610;      //!< 入力電圧最大制限値　最大値
        
        constexpr unsigned short FailSafeVoltageMinRange = 612; //!< フェールセーフ電圧　最小値
        
        constexpr unsigned short FailSafeVoltageMaxRange = 614; //!< フェールセーフ電圧　最大値
        
        constexpr unsigned short CurrentMinRange = 616;         //!< モータ消費電流設定値　最小値
        
        constexpr unsigned short CurrentMaxRange = 618;         //!< モータ消費電流設定値　最大値
        
        constexpr unsigned short MotorTempMinRange = 620;       //!< モータ温度設定値　最小値
        
        constexpr unsigned short MotorTempMaxRange = 622;       //!< モータ温度設定値　最大値
        
        constexpr unsigned short CpuTempMinRange = 624;         //!< CPU温度設定値　最小値
        
        constexpr unsigned short CpuTempMaxRange = 626;         //!< CPU温度設定値　最大値
        
        constexpr unsigned short CwPositionMinRange = 628;      //!< CW方向最大値　最小値
        
        constexpr unsigned short CwPositionMaxRange = 630;      //!< CW方向最大値　最大値
        
        constexpr unsigned short CcwPositionMinRange = 632;     //!< CCW方向最大値　最小値
        
        constexpr unsigned short CcwPositionMaxRange = 634;     //!< CCW方向最大値　最大値
        
        constexpr unsigned short MaxGoalSpeedMinRange = 636;    //!< 最大速度設定値　最小値
        
        constexpr unsigned short MaxGoalSpeedMaxRange = 638;    //!< 最大速度設定値　最大値
        
        constexpr unsigned short MaxGoalCurrentMinRange = 640;  //!< 最大電流設定値　最小値
        
        constexpr unsigned short MaxGoalCurrentMaxRange = 642;  //!< 最大電流設定値　最大値
        
        constexpr unsigned short MaxGoalTorqueMinRange = 644;   //!< 最大トルク設定値　最小値
        
        constexpr unsigned short MaxGoalTorqueMaxRange = 646;   //!< 最大トルク設定値　最大値

        /// @remark PMXのV1.0.1.xから有効になります
        constexpr unsigned short GoalCommandValue1 = 700;       //!< 目標指令値1(PMXのV1.0.1.x～)

        /// @remark PMXのV1.0.1.xから有効になります
        constexpr unsigned short GoalCommandValue2 = 702;       //!< 目標指令値2(PMXのV1.0.1.x～)

        /// @remark PMXのV1.0.1.xから有効になります
        constexpr unsigned short GoalCommandValue3 = 704;       //!< 目標指令値3(PMXのV1.0.1.x～)
    }

    /// 
    /// @brief コントロールモード一覧
    /// @details 位置や速度制御等の切り替えの値を定義します 
    ///
    namespace ControlMode
    {

        constexpr byte Position = 0x01;         //!< 位置制御

        constexpr byte Speed = 0x02;            //!< 速度制御
        
        constexpr byte PositionSpeed = 0x03;    //!< 位置/速度制御

        constexpr byte Current = 0x04;          //!< 電流制御
        
        constexpr byte PositionCurrent = 0x05;  //!< 位置/電流制御

        constexpr byte SpeedCurrent = 0x06;     //!< 速度/電流制御

        constexpr byte PositionSpeedCurrent = 0x07; //!< 位置/速度/電流制御

        constexpr byte Torque = 0x08;           //!< トルク制御

        constexpr byte PositionTorque = 0x09;   //!< 位置/トルク制御
        
        constexpr byte SpeedTorque = 0x0A;      //!< 速度/トルク制御

        constexpr byte PositionSpeedTorque = 0x0B;   //!< 位置/速度/トルク制御

        constexpr byte PWM = 0x10;              //!< PWM制御
        
        constexpr byte Time = 0x20;             //!< 時間(単体制御はないので定義のみ)

        constexpr byte PositionTime = 0x21;     //!< 位置/時間制御
        
        constexpr byte PositionCurrentTime = 0x25;  //!< 位置/電流/時間制御
        
        constexpr byte PositionTorqueTime = 0x29;   //!< 位置/トルク/時間制御
    }

    /// @brief PMXの時間制御で使用する加減速具合を設定します
    namespace TrajectoryType
    {
        constexpr byte Even = 0x01;         //!< 均等補間
        
        constexpr byte FifthPoly = 0x05;    //!< 5次多項式補間

        constexpr byte Error = 0xFF;        //!< エラー
    }

    ///
    /// @brief 通信速度の定義 
    /// @details PMXで設定する通信速度値を定義します。
    /// 
    namespace EditBaudrate
    {
        constexpr byte _57600 = 0x00;           //!< 57600bps
        
        constexpr byte _115200 = 0x01;          //!< 115200bps
        
        constexpr byte _625000 = 0x02;          //!< 625000bps
        
        constexpr byte _1000000 = 0x03;         //!< 1Mbps
        
        constexpr byte _1250000 = 0x04;         //!< 1.25Mbps
        
        constexpr byte _1500000 = 0x05;         //!< 1.5Mbps
        
        constexpr byte _2000000 = 0x06;         //!< 2Mbps
        
        constexpr byte _3000000 = 0x07;         //!< 3Mbps
    }

    /// @brief PMXで設定するパリティの値を定義します
    namespace EditParity
    {
        constexpr byte ParityNone = 0x00;   //!< パリティなし

        constexpr byte Odd = 0x01;          //!< パリティ奇数

        constexpr byte Even = 0x02;         //!< パリティ偶数

        constexpr byte Error = 0xFF;        //!< パリティエラー
    }

    /// @brief 送信コマンド一覧
    namespace SendCmd
    {
        constexpr byte MemREAD = 0xa0;          //!< RAM上のデータを読み取る
        
        constexpr byte MemWRITE = 0xa1;         //!< RAM上へデータを書き込む
        
        constexpr byte LOAD = 0xa2;             //!< フラッシュメモリーに保存されているデータをRAMへ展開する
        
        constexpr byte SAVE = 0xa3;             //!< RAMのデータをフラッシュメモリーに保存する
        
        constexpr byte MotorREAD = 0xa4;        //!< ポジション指定コマンド
        
        constexpr byte MotorWRITE = 0xa5;       //!< RAM上へデータを書き込む

        constexpr byte SystemREAD = 0xbb;       //!< IDシリアル番号等の読み込み
        
        constexpr byte SystemWRITE = 0xbc;      //!< ID・通信速度・パリティ書き込み
        
        constexpr byte ReBoot = 0xbd;           //!< リセットコマンド
        
        constexpr byte FactoryReset = 0xbe;     //!< 工場出荷時設定へ戻す
    }

    /// @brief  送受信データ位置 
    namespace BuffPter
    {
        //''' 送受信データ位置 '''
        constexpr int Header = 0;
        constexpr int Header1 = 1;
        constexpr int ID = 2;
        constexpr int Length = 3;
        constexpr int CMD = 4;
        constexpr int Option = 5;
        constexpr int Status = 5;
        constexpr int Data = 6;
        constexpr int ACK = 6;
    }

    ///
    /// @brief MotorREAD等で使用する応答モードのリスト
    /// @details 複数のデータの返信をもらうには、全て足したものを使用します。
    /// @code
    /// // 位置と電流値を返す値
    /// returnData = PmxLib.ReceiveDataOption.Position + PmxLib.ReceiveDataOption.Current
    /// @endcode
    namespace ReceiveDataOption
    {
        constexpr byte NoReturn = 0x00;
        constexpr byte Position = 0x01;
        constexpr byte Speed = 0x02;
        constexpr byte Current = 0x04;
        constexpr byte Torque = 0x08;
        constexpr byte Pwm = 0x10;
        constexpr byte MotorTemp = 0x20;
        constexpr byte CpuTemp = 0x40;
        constexpr byte Voltage = 0x80;
        constexpr byte Full = 0xFF;
    }

    ///
    /// @brief データが返ってこないなどサーボモータ外に起因するエラー一覧の定義
    /// @details サーボモータ外に起因するエラー(通信タイムアウト等)のデータを定義しています
    /// @details PMXと通信を行い、結果を返すときに使用します。
    /// @note PMXと通信を行った際、PMXのstatasは1byteなので、頭にもう1byte付与しエラー値とします
    /// @note PMXの通信結果を返す場合関数は、このComErrorとモータの返信statas値を付与(OR)した値を返します
    ///
    namespace ComError
    {
        constexpr unsigned short OK = 0;                //!< 正常
 
        constexpr unsigned short TimeOut = 0xFF00;      //!< 無応答/Timeoutエラー
        
        constexpr unsigned short CrcError = 0xFE00;     //!< CRCエラー
        
        constexpr unsigned short FormatError = 0xFD00;  //!< 作られたデータのフォーマットに異常があった
        
        constexpr unsigned short SendError = 0xFC00;    //!< 送信する際に異常があった
        
        constexpr unsigned short ReceiveError = 0xFB00; //!< 受信したデータに異常があった
        
        constexpr unsigned short MotorREADConvertError = 0xFA00;    //!< MotorREADのデータ変換に異常があった
                
        constexpr unsigned short NG = 0xFA00;           //!< 上記以外のエラー
        
        constexpr unsigned short ErrorMask = 0xFF00;            //!< マスク用のデータ
    }

    /// @brief コマンドの最低サイズ値の定義
    namespace MinimumLength
    {
        constexpr byte Send = 8;        //!< 送信コマンドの最低サイズ
        // Header(2),id(1),length(1),cmd(1),st(1),crc(2)
        
        constexpr byte Receive = 8;     //!< 受信コマンドの最低サイズ
        // Header(2),id(1),length(1),cmd(1),st(1),crc(2)
    }

    /// @brief PMXのステータスの返信/エラー状態の値
    namespace PmxStatusErrorList
    {
        constexpr byte SystemError = 0x01;          //!< System Error
        constexpr byte MotorError = 0x02;           //!< Motor Error
        constexpr byte CommunicationError = 0x04;   //!< Communication Error
        constexpr byte CommandError = 0x08;         //!< Command Error
        constexpr byte RamAccessError = 0x10;       //!< RAM Access Error
        constexpr byte ModeError = 0x20;            //!< Mode Error
        constexpr byte DataError = 0x40;            //!< Data Error
        constexpr byte RunError = 0x80;             //!< Run Error
    }

    /// @brief TorqueOnの値の定義 
    namespace TorqueSwitchType
    {
        constexpr byte Control = 0x00;      //!< MotorWriteのオプションで使用
        
        constexpr byte TorqueOn = 0x01;     //!< トルクON
        
        constexpr byte Free = 0x02;         //!< フリーモード 
        
        constexpr byte Brake = 0x04;        //!< ショートブレーキモード
        
        constexpr byte Hold = 0x08;         //!< Holdモード

        constexpr byte Mask = 0x0F;         //!< トルクON/OFFを抜き出すためのマスクする値

        constexpr byte Error = 0xFF;        //!< エラー
    }

    /// @brief CloneReverseの値の定義
    namespace CloneReverseType
    {
        constexpr byte Clone = 0x01;        //!< cloneOnの設定値

        constexpr byte Reverse = 0x02;      //!< ReverseOnの設定値
    }

    /// @brief LED点灯モードの値の定義
    namespace LedModeType
    {
        constexpr byte Normal = 0x00;   //!< 標準モード

        constexpr byte Off = 0x01;      //!< 消灯モード
    }

};

///
/// @brief PMXで使用する関数や定義をひとまとまりにしたものです
/// @details
/// * PMXの固有値の定義
/// * PMXのコマンドの生成、チェックおよび送受信(送信部分は外部依存)
/// * PMXで必要だと思われる機能の関数化
/// @details をしています
///
/// @warning 通信部分は外部に定義しますのでこのclassを派生して送信に必要な関数を追加してください。
///
class PmxBase
{
    static  constexpr int Version = 1000;	//!< バージョン番号

    //データ送受信 (ここだけソースに無いので記載)///////////////////////////////////////////////////////////////////
    /**
    * @brief ICS通信の送受信
    * @param[in,out] *txBuf
    * @param[in] txLen
    * @param[out] *rxBuf 受信格納バッファ
    * @param[in] rxLen  受信データ数
    * @retval true 通信成功
    * @retval false 通信失敗
    * @attention インターフェイス、子クラスで定義すること
    * @attention 送信データ数、受信データ数はコマンドによって違うので注意する
    * 
    **/
    virtual bool synchronize(byte txBuf[],byte txLen,byte rxBuf[],byte rxLen);	//インターフェイス、子クラスで定義すること
    virtual bool synchronizeVariableRead(byte *txBuf, byte txLen, byte *rxBuf, byte *rxLen);
    //virtual bool setSerialParameters(long baudrate = PMX::ErrorUint32Data,byte parity=PMX::ErrorByteData,unsigned int timeout=PMX::ErrorUint16Data);

    //送受信ログの処理
    public:
        /// @brief Logを出力するシリアルポートの設定をします。
        /// @param [in] logSerial 
        virtual void setLogSerial(HardwareSerial *logSerial); 


    protected:
        virtual HardwareSerial *getLogSerial();
        virtual void logOutputPrint(byte outputBytes[],int outputLength);


    public:

        unsigned short MemREAD(byte id, unsigned short addr, int readDataSize, byte rxData[]);            
        unsigned short MemREADToByte(byte id, unsigned short addr , byte *byteData);
        unsigned short MemREADToInt16(byte id, unsigned short addr , short *int16Data);
        unsigned short MemREADToUint16(byte id, unsigned short addr , unsigned short *uint16Data);
        unsigned short MemREADToInt32(byte id, unsigned short addr , long *int32Data);
        unsigned short MemREADToUint32(byte id, unsigned short addr , unsigned long *uint32Data);
        
        unsigned short MemWRITE(byte id, unsigned short addr, byte txDataArray[], int txDataSize, byte writeOpt=0);
        unsigned short MemWRITEToByte(byte id, unsigned short addr,byte byteData,byte writeOpt=0);
        unsigned short MemWRITEToInt16(byte id, unsigned short addr, short int16Data, byte writeOpt=0);
        unsigned short MemWRITEToUint16(byte id, unsigned short addr, unsigned short uint16Data, byte writeOpt=0);
        unsigned short MemWRITEToInt32(byte id, unsigned short addr, long int32Data, byte writeOpt=0);
        unsigned short MemWRITEToUint32(byte id, unsigned short addr, unsigned long uint32Data, byte writeOpt=0);

        unsigned short MotorREAD(byte id, byte receiveMode, long receiveData[8], byte controlMode=0x01, byte *torqueSw = nullptr);

        unsigned short MotorWRITE(byte id, byte toruqeOnSw);
        unsigned short MotorWRITE(byte id, byte toruqeOnSw, byte receiveMode, long receiveData[8], byte controlMode=0x01);
        unsigned short MotorWRITE(byte id, long writeDatas[], int writeDataCount);
        unsigned short MotorWRITE(byte id, long writeDatas[], int writeDataCount,byte receiveMode, long receiveData[8], byte controlMode=0x01);

        unsigned short MotorWRITESingle(byte id, long targetVal);
        unsigned short MotorWRITESingle(byte id, long targetVal,byte receiveMode, long receiveData[8], byte controlMode=0x01);
        unsigned short MotorWRITEDouble(byte id, long targetVal1, long targetVal2);
        unsigned short MotorWRITEDouble(byte id, long targetVal1, long targetVal2,byte receiveMode, long receiveData[8], byte controlMode=0x01);
        unsigned short MotorWRITETriple(byte id, long targetVal1, long targetVal2, long targetVal3);
        unsigned short MotorWRITETriple(byte id, long targetVal1, long targetVal2, long targetVal3,byte receiveMode, long receiveData[8], byte controlMode=0x01);

        unsigned short LOAD(byte id);
        unsigned short SAVE(byte id);
        unsigned short SystemREAD(byte id, byte rxData[]);

        //SystemWriteを利用したデータのget関数一覧
        unsigned short getSerialNumber(byte id,unsigned long *serialLongNum);
        unsigned short getSerialNumber(byte id,byte serialByteNum[4]);
        unsigned short getModelNum(byte id,unsigned long *modelFullNum);
        unsigned short getModelNum(byte id, unsigned short *modelNum, unsigned short *seriesNum);
        unsigned short getVersion(byte id,byte verData[4]);
        unsigned short getResponseTime(byte id,byte *respTime);

        unsigned short SystemWRITE(byte id, byte serialNum[4], byte option, byte newId, byte newBaudrateVal, byte newParityVal, byte newResponseTime);
        unsigned short SystemWRITE(byte id, unsigned long serialNum, byte option, byte newId, byte newBaudrateVal, byte newParityVal, byte newResponseTime);
        unsigned short SystemWRITE(byte id, byte option, byte newId, byte newBaudrateVal, byte newParityVal, byte newResponseTime);

        //SystemWriteを利用したデータのset関数一覧
        unsigned short setId(byte id, byte newId);
        unsigned short setBaudrate(byte id, byte newBaudRate);
        unsigned short setParity(byte id, byte newParityNum);
        unsigned short setResponseTime(byte id, byte respTime);

        unsigned short ReBoot(byte id, int resetTime = 0);
        unsigned short FactoryReset(byte id, byte serialNum[4] = NULL);

        //MemREADを利用したデータのget関数一覧
        //アドレス0～
        unsigned short getPositionKpGain(byte id, unsigned long *data);
        unsigned short getPositionKiGain(byte id, unsigned long *data);
        unsigned short getPositionKdGain(byte id, unsigned long *data);
        unsigned short getPositionGain(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata);
        unsigned short getPositionStretchGain(byte id, unsigned long *data);
        unsigned short getSpeedKpGain(byte id, unsigned long *data);
        unsigned short getSpeedKiGain(byte id, unsigned long *data);
        unsigned short getSpeedKdGain(byte id, unsigned long *data);
        unsigned short getSpeedGain(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata);
        unsigned short getCurrentKpGain(byte id, unsigned long *data);
        unsigned short getCurrentKiGain(byte id, unsigned long *data);
        unsigned short getCurrentKdGain(byte id, unsigned long *data);
        unsigned short getCurrentGain(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata);
        unsigned short getTorqueKpGain(byte id, unsigned long *data);
        unsigned short getTorqueKiGain(byte id, unsigned long *data);
        unsigned short getTorqueKdGain(byte id, unsigned long *data);
        unsigned short getTorqueGain(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata);
        unsigned short getCenterOffset(byte id, short *offsetData);
        // unsigned short getReverse(byte id, byte *data);
        unsigned short getCloneReverse(byte id, byte *data);
        unsigned short getMinVoltageLimit(byte id, unsigned short *data);
        unsigned short getMinVoltageLimitPower(byte id, unsigned short *data);
        unsigned short getMaxVoltageLimit(byte id, unsigned short *data);
        unsigned short getMaxVoltageLimitPower(byte id, unsigned short *data);
        unsigned short getCurrentLimit(byte id, unsigned short *data);
        unsigned short getCurrentLimitPower(byte id, unsigned short *data);
        unsigned short getMotorTempLimit(byte id, unsigned short *data);
        unsigned short getMotorTempLimitPower(byte id, unsigned short *data);
        unsigned short getCpuTempLimit(byte id, unsigned short *data);
        unsigned short getCpuTempLimitPower(byte id, unsigned short *data);
        unsigned short getCwPositionLimit(byte id, short *data);
        unsigned short getCwPositionLimitPower(byte id, unsigned short *data);
        unsigned short getCcwPositionLimit(byte id, short *data);
        unsigned short getCcwPositionLimitPower(byte id, unsigned short *data);
        unsigned short getMaxGoalSpeed(byte id, short *data);
        unsigned short getMaxGoalCurrent(byte id, short *data);
        unsigned short getMaxGoalTorque(byte id, short *data);
        unsigned short getTotalPowerRate(byte id, unsigned short *data);
        unsigned short getLockDetectTime(byte id, unsigned short *data);
        unsigned short getLockThresholdPower(byte id, unsigned short *data);
        unsigned short getLockDetectOutputPower(byte id, unsigned short *data);

        unsigned short getPositionPresetNum(byte id, byte *data);
        unsigned short getSpeedPresetNum(byte id, byte *data);
        unsigned short getCurrentPresetNum(byte id, byte *data);
        unsigned short getTorquePresetNum(byte id, byte *data);
        unsigned short getAllPresetNum(byte id, byte *pos, byte *spd, byte *cur, byte *trq);

        unsigned short getPositionKpGain2(byte id, unsigned long *data);
        unsigned short getPositionKiGain2(byte id, unsigned long *data);
        unsigned short getPositionKdGain2(byte id, unsigned long *data);
        unsigned short getPositionGain2(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata);
        unsigned short getPositionStretchGain2(byte id, unsigned long *data);
        unsigned short getSpeedKpGain2(byte id, unsigned long *data);
        unsigned short getSpeedKiGain2(byte id, unsigned long *data);
        unsigned short getSpeedKdGain2(byte id, unsigned long *data);
        unsigned short getSpeedGain2(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata);
        unsigned short getCurrentKpGain2(byte id, unsigned long *data);
        unsigned short getCurrentKiGain2(byte id, unsigned long *data);
        unsigned short getCurrentKdGain2(byte id, unsigned long *data);
        unsigned short getCurrentGain2(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata);
        unsigned short getTorqueKpGain2(byte id, unsigned long *data);
        unsigned short getTorqueKiGain2(byte id, unsigned long *data);
        unsigned short getTorqueKdGain2(byte id, unsigned long *data);
        unsigned short getTorqueGain2(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata);

        unsigned short getPositionKpGain3(byte id, unsigned long *data);
        unsigned short getPositionKiGain3(byte id, unsigned long *data);
        unsigned short getPositionKdGain3(byte id, unsigned long *data);
        unsigned short getPositionGain3(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata);
        unsigned short getPositionStretchGain3(byte id, unsigned long *data);
        unsigned short getSpeedKpGain3(byte id, unsigned long *data);
        unsigned short getSpeedKiGain3(byte id, unsigned long *data);
        unsigned short getSpeedKdGain3(byte id, unsigned long *data);
        unsigned short getSpeedGain3(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata);
        unsigned short getCurrentKpGain3(byte id, unsigned long *data);
        unsigned short getCurrentKiGain3(byte id, unsigned long *data);
        unsigned short getCurrentKdGain3(byte id, unsigned long *data);
        unsigned short getCurrentGain3(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata);
        unsigned short getTorqueKpGain3(byte id, unsigned long *data);
        unsigned short getTorqueKiGain3(byte id, unsigned long *data);
        unsigned short getTorqueKdGain3(byte id, unsigned long *data);
        unsigned short getTorqueGain3(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata);

        //アドレス300番台
        unsigned short getPosition(byte id, short *posData);
        unsigned short getPosition(byte id, unsigned short *posData);
        unsigned short getPosition(byte id, long *posData, byte controlMode = 0x01);
        unsigned short getSpeed(byte id, short *spdData);
        unsigned short getCurrent(byte id, short *curData);
        unsigned short getTorque(byte id, short *trqData);
        unsigned short getPwm(byte id, short *pwmData);
        unsigned short getMotorTemp(byte id, short *motTempData);
        unsigned short getCPUTemp(byte id, short *cpuTempData);
        unsigned short getInputVoltage(byte id, unsigned short *volData);
        unsigned short getTrajectoryTime(byte id, unsigned short *traTimeData);
        unsigned short getEncoder(byte id, unsigned short *encData);

        //アドレス400番台
        unsigned short getStatus(byte id, byte *stData);
        unsigned short getSystemStatus(byte id, byte *sysStaData);
        unsigned short getMotorStatus(byte id, byte *motStaData);
        unsigned short getRamAccessStatus(byte id, unsigned short *ramStaData);
        unsigned short getFullStatus(byte id, byte *sysSt, byte *motorSt, unsigned short *ramSt);
        unsigned short resetFullStatus(byte id);

        //アドレス500番台
        unsigned short getTorqueSwitch(byte id, byte *trqSwitchData);
        unsigned short getControlMode(byte id, byte *controlMode);
        unsigned short getMotorReceive(byte id, byte *receiveMode);
        unsigned short getTrajectory(byte id, byte *traData);
        unsigned short getLedMode(byte id, byte *ledData);

        //アドレス600番台
        unsigned short getCenterOffsetRange(byte id, short *minData, short *maxData);
        unsigned short getMinVoltageLimitRange(byte id, unsigned short *minData, unsigned short *maxData);
        unsigned short getMaxVoltageLimitRange(byte id, unsigned short *minData, unsigned short *maxData);
        unsigned short getCurrentLimitRange(byte id, unsigned short *minData, unsigned short *maxData);
        unsigned short getMotorTempLimitRange(byte id, short *minData, short *maxData);
        unsigned short getCpuTempLimitRange(byte id, short *minData, short *maxData);
        unsigned short getMaxGoalSpeedRange(byte id, short *minData, short *maxData);
        unsigned short getMaxGoalCurrentRange(byte id, short *minData, short *maxData);
        unsigned short getMaxGoalTorqueRange(byte id, short *minData, short *maxData);
        unsigned short getCwPositionLimitRange(byte id, short *minData, short *maxData);
        unsigned short getCcwPositionLimitRange(byte id, short *minData, short *maxData);


        //MemWriteを利用したデータのset関数一覧
        //アドレス0～
        unsigned short setPositionKpGain(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setPositionKiGain(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setPositionKdGain(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setPositionGain(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt=0);
        unsigned short setPositionStretchGain(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setSpeedKpGain(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setSpeedKiGain(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setSpeedKdGain(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setSpeedGain(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt=0);
        unsigned short setCurrentKpGain(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setCurrentKiGain(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setCurrentKdGain(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setCurrentGain(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt=0);
        unsigned short setTorqueKpGain(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setTorqueKiGain(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setTorqueKdGain(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setTorqueGain(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt=0);
        unsigned short setCenterOffset(byte id,short offsetData);
        unsigned short setCloneReverse(byte id,byte data);
        unsigned short setMinVoltageLimit(byte id,unsigned short minVol, unsigned short limPower ,byte writeOpt=0);
        unsigned short setMaxVoltageLimit(byte id,unsigned short maxVol, unsigned short limPower ,byte writeOpt=0);
        unsigned short setCurrentLimit(byte id, short maxCur, unsigned short limPower ,byte writeOpt=0);
        unsigned short setMotorTempLimit(byte id, short motorTemp, unsigned short limPower ,byte writeOpt=0);
        unsigned short setCpuTempLimit(byte id, short cpuTemp, unsigned short limPower ,byte writeOpt=0);
        unsigned short setPositionLimit(byte id, short cwPos, short ccwPos, unsigned short limPower ,byte writeOpt=0);
        unsigned short setMaxGoalSpeed(byte id, short maxGoalSpd, byte writeOpt=0);
        unsigned short setMaxGoalCurrent(byte id, short maxGoalCur, byte writeOpt=0);
        unsigned short setMaxGoalTorque(byte id, short maxGoalTrq, byte writeOpt=0);
        unsigned short setTotalPowerRate(byte id,unsigned short rate, byte writeOpt=0);
        unsigned short setLockDetect(byte id, unsigned short time, unsigned short power, unsigned short outputPower, byte writeOpt=0);

        unsigned short setPositionPresetNum(byte id, byte presetNum, byte writeOpt=0);
        unsigned short setSpeedPresetNum(byte id, byte presetNum, byte writeOpt=0);
        unsigned short setCurrentPresetNum(byte id, byte presetNum, byte writeOpt=0);
        unsigned short setTorquePresetNum(byte id, byte presetNum, byte writeOpt=0);
        unsigned short setAllPresetNum(byte id, byte presetNum, byte writeOpt=0);

        unsigned short setPositionKpGain2(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setPositionKiGain2(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setPositionKdGain2(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setPositionGain2(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt=0);
        unsigned short setPositionStretchGain2(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setSpeedKpGain2(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setSpeedKiGain2(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setSpeedKdGain2(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setSpeedGain2(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt=0);
        unsigned short setCurrentKpGain2(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setCurrentKiGain2(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setCurrentKdGain2(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setCurrentGain2(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt=0);
        unsigned short setTorqueKpGain2(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setTorqueKiGain2(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setTorqueKdGain2(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setTorqueGain2(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt=0);

        unsigned short setPositionKpGain3(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setPositionKiGain3(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setPositionKdGain3(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setPositionGain3(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt=0);
        unsigned short setPositionStretchGain3(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setSpeedKpGain3(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setSpeedKiGain3(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setSpeedKdGain3(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setSpeedGain3(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt=0);
        unsigned short setCurrentKpGain3(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setCurrentKiGain3(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setCurrentKdGain3(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setCurrentGain3(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt=0);
        unsigned short setTorqueKpGain3(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setTorqueKiGain3(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setTorqueKdGain3(byte id, unsigned long data, byte writeOpt=0);
        unsigned short setTorqueGain3(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt=0);


        //アドレス500番台
        unsigned short setTorqueSwitch(byte id, byte data, byte writeOpt=1);
        unsigned short setControlMode(byte id, byte controlMode,byte writeOpt=0);
        unsigned short setMotorReceive(byte id, byte receiveMode,byte writeOpt=0);
        unsigned short setTrajectory(byte id, byte trajecrotyData, byte writeOpt=0);
        unsigned short setLedMode(byte id, byte ledModeData, byte writeOpt=0);

        //MotorWRITEを利用したデータのset関数一覧
        unsigned short setMotorTorqueOn(byte id);
        unsigned short setMotorTorqueOn(byte id,byte receiveMode, long receiveData[8], byte controlMode=0x01);
        unsigned short setMotorFree(byte id);
        unsigned short setMotorFree(byte id,byte receiveMode, long receiveData[8], byte controlMode=0x01);
        unsigned short setMotorBrake(byte id);
        unsigned short setMotorBrake(byte id,byte receiveMode, long receiveData[8], byte controlMode=0x01);
        unsigned short setMotorHold(byte id);
        unsigned short setMotorHold(byte id,byte receiveMode, long receiveData[8], byte controlMode=0x01);
        unsigned short setPosition(byte id, short pos);
        unsigned short setPosition(byte id, short pos, byte receiveMode, long receiveData[8]);


    protected:
    
        bool defaultMakeCmd(byte id, byte cmd,byte txData[],byte *txDataSize ,byte header=0xfe);

        unsigned short checkRecv(byte rxBuff[],byte cmd,byte header=0xfe);

        static unsigned int __byteCounter(byte val);

        static bool __convReceiveMotorData(byte receiveMode, byte returnDataBytes[], byte receiveBytesSize, long reData[], byte controlMode=0x01);

};





#endif


