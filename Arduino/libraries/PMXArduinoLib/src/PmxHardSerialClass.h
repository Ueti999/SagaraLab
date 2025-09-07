
/** 
* @file PmxHardSerialClass.h
* @brief  PMX arduino (HardwareSerial) library header file
* @author Kondo Kagaku Co.,Ltd.
* @author kedtn(T.Nobuhara) Gotanda
* @date 2024/02/22
* @version 1.0.1
* @copyright Kondo Kagaku Co.,Ltd. 2024
**/	

#ifndef __Pmx_HardSerial_Class_h__
#define __Pmx_HardSerial_Class_h__



#include "PmxBaseClass.h"

#include "Arduino.h"
#include "HardwareSerial.h"

/// @brief PMXで使用する関数や定義をひとまとまりにしたもので、ArduinoのHardwareSerialを使用して動作します。
/// @details    ・PMXの固有値の定義
///             ・PMXのコマンドの生成、チェックおよび送受信(送信部分は外部依存)
///             ・PMXで必要だと思われる機能の関数化
///             ・通信部分はArduinoのHardwareSerialを使用しています
class PmxHardSerial : public PmxBase
{
    public:


    protected:
        HardwareSerial *pmxSerial;  //    通信を使用するためのポインタ変数

	    int	g_timeout = PMX::ErrorUint16Data;				//	タイムアウトの設定
	    long g_baudrate = PMX::ErrorUint32Data;			//	通信速度の設定
        unsigned short g_SerialConfig = 0xFF;          //    パリティの設定
        byte g_enPin = 0xFF;                //    イネーブルピンの定義
        /// byte g_rxPin = 0xFF;                //    RXピンの定義(M5などで必要な場合)
        /// byte g_txPin = 0xFF;                //    TXピンの定義(M5などで必要な場合)
        

        byte sendBuff[256];   //!< 送信バッファ
        byte receiveBuff[256];   //!< 受信バッファ




    private:
        bool _isSynchronize = false;
        bool _logOutput = false;
        HardwareSerial *_logOutputSerial = nullptr;

    

    // 関数一覧
    public:
        PmxHardSerial(HardwareSerial *hardSerial,byte enPin,long baudrate=115200, int timeout=100);
        PmxHardSerial(HardwareSerial *hardSerial,byte enPin,long baudrate, unsigned short serialConfig, int timeout);
        // PmxHardSerial(HardwareSerial *hardSerial,byte rxPin,byte txPin, byte enPin, long baudrate=115200, int timeout=1000,byte partyVal=0x00);

        //デストラクタ
        ~PmxHardSerial();

        //  通信の初期化
        bool begin(long baudrate=PMX::ErrorUint32Data, int timeout=PMX::ErrorUint16Data);

        //他の物が通信中かどうか
        bool isSynchronize();

    //イネーブルピンの処理
    protected : 
        /**
	    *	@brief enPinに割り当てられているピンをHにする
	    **/
        inline void enHigh(){digitalWrite(g_enPin, HIGH);}
        /**
        *	@brief enPinに割り当てられているピンをLにする
        **/
        inline void enLow(){digitalWrite(g_enPin, LOW);}

    //データ送受信
    public :
        virtual bool synchronize(byte *txBuf, byte txLen, byte *rxBuf, byte rxLen);
        virtual bool synchronizeVariableRead(byte *txBuf, byte txLen, byte *rxBuf, byte *rxLen);
        virtual bool synchronizeNoRead(byte *txBuf, byte txLen);
        //virtual bool setSerialParameters(long baudrate = PMX::ErrorUint32Data,byte parity=PMX::ErrorByteData,unsigned int timeout=PMX::ErrorUint16Data);
    
    //送受信ログの処理
    public:
        virtual void setLogSerial(HardwareSerial *logSerial){_logOutputSerial=logSerial;} 
        


    protected:
        virtual void logOutputPrint(byte outputBytes[],int outputSize);
        virtual HardwareSerial *getLogSerial(){return _logOutputSerial;}

    private:
        void __synchronizeWrite(byte *txBuf, byte txLen);

    //  ログの出力

};


#endif


