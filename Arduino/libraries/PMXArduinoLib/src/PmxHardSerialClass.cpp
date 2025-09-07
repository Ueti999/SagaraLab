
/** 
* @file PmxHardSerialClass.cpp
* @brief PMX arduino (HardwareSerial) library source file
* @author Kondo Kagaku Co.,Ltd.
* @author T.Nobuhara Gotanda
* @date 2024/02/22
* @version 1.0.2
* @copyright Kondo Kagaku Co.,Ltd. 2024
*
*/

#include "PmxBaseClass.h"
#include "PmxHardSerialClass.h"
#include <Arduino.h>





/**
 * @brief Construct a new Pmx Hard Serial:: Pmx Hard Serial object
 * 
 * @param [in] hardSerial ArduinoのHardwareSerialのポインタ
 * @param [in] enPin 送受信を切り替えるピン
 * @param [in] baudrate 通信速度 
 * @param [in] timeout 受信タイムアウト 
 */
PmxHardSerial::PmxHardSerial(HardwareSerial *hardSerial,byte enPin,long baudrate, int timeout)
{
    pmxSerial = hardSerial;
    g_timeout = timeout;				//	タイムアウトの設定
    g_baudrate = baudrate;			//	通信速度の設定
    g_SerialConfig = SERIAL_8N1;
    g_enPin = enPin;                //   イネーブルピンの定義

}

/**
 * @brief Construct a new Pmx Hard Serial:: Pmx Hard Serial object
 * 
 * @param [in] hardSerial ArduinoのHardwareSerialのポインタ 
 * @param [in] enPin 送受信を切り替えるピン
 * @param [in] baudrate 通信速度 
 * @param [in] serialConfig ArduinoのHardwareSerialの通信設定
 * @param [in] timeout 受信タイムアウト
 * 
 * @warning serialConfigは、PMXの設定ではなく、Arduinoの通信設定(SERIAL_8N1等)を記述する事 
 */
PmxHardSerial::PmxHardSerial(HardwareSerial *hardSerial,byte enPin,long baudrate,unsigned short serialConfig, int timeout)
{
    pmxSerial = hardSerial;
    g_timeout = timeout;				//	タイムアウトの設定
    g_baudrate = baudrate;			//	通信速度の設定
    g_SerialConfig = serialConfig;      // ArduinoのHardwareSerialの通信設定
    g_enPin = enPin;                //    イネーブルピンの定義
}



/**
 * @brief Destroy the Pmx Hard Serial:: Pmx Hard Serial object
 * 
 */
PmxHardSerial::~PmxHardSerial()
{
    if(pmxSerial)
    {
        pmxSerial->end();
    }
}

/**
 * @brief HardwareSerialポートの設定をする
 * 
 * @param baudrate 通信速度 
 * @param timeout 受信タイムアウト 
 * @return true begin成功
 * @return false begin失敗
 * 
 */
bool PmxHardSerial::begin(long baudrate, int timeout)
{
    if(pmxSerial == nullptr)
    {
        return false;
    }

    

    if(baudrate != PMX::ErrorUint32Data)
    {
        g_baudrate = baudrate;
    }
    if(timeout != PMX::ErrorUint16Data)
    {
        g_timeout = timeout;
    }
    

    pinMode(g_enPin, OUTPUT);

    pmxSerial->begin(g_baudrate,g_SerialConfig);
    //pmxSerial->begin(g_baudrate,SERIAL_8N1,g_rxPin,g_txPin);

    pmxSerial->setTimeout(g_timeout);

    return true;
}

/**
 * @brief Synchronize関数が送受信してるか確認する
 * 
 * @return true 送受信中
 * @return false 送受信待機中
 */
bool PmxHardSerial::isSynchronize()
{
    return _isSynchronize;
}


/**
 * @brief HardwareSerialを使用して送受信します
 * 
 * @param [in] txBuf 送信データ
 * @param [in] txLen 送信データ数
 * @param [out] rxBuf 受信データ
 * @param [out] rxLen 受信データ数
 * @return true 送受信成功
 * @return false 送受信失敗
 */
bool PmxHardSerial::synchronize(byte *txBuf, byte txLen, byte *rxBuf, byte rxLen)
{
    int rxSize; //受信数

	//シリアル初期化確認
	if(pmxSerial == nullptr )
	{
		return false;
	}

    //他の物が通信中だった場合抜ける
    if(_isSynchronize == true)
    {
        return false;
    }

    //通信中にする
    _isSynchronize = true;

    //送信のみをする
    this->__synchronizeWrite(txBuf,txLen);


    //受信データを初期化しておく
    for(int i = 0; i < rxLen; i++)
    {
        receiveBuff[i] = 0xFF;
    }

    //上位のバッファに影響しないように内部の受信バッファに入れておく
	rxSize = pmxSerial->readBytes(receiveBuff, rxLen);
    memcpy(rxBuf, receiveBuff, rxLen);

    //通信中を解除する
    _isSynchronize = false;

	if (rxSize != rxLen) //受信数確認
	{
		return false;
    }

	return true;

	
}


/**
 * @brief HardwareSerialを使用してtxBufのコマンドを送受信します。
 *        返信データ数がわからない場合に使用します。
 * 
 * @param [in] txBuf 送信データ
 * @param [in] txLen 送信データ数
 * @param [out] rxBuf 受信データ
 * @param [out] rxLen 受信データ数
 * 
 * @return true 送受信成功
 * @return false 送受信失敗
 * 
 * @note データを読む際、6byte目にLengthがあるので、まずは6byte取得する
 * @note Lengthを取得できたらそのデータをもとに全てのデータを取得する
 * @note データが来ない場合もあるのでTimeoutでエラー処理も行う
 * 
 */
bool PmxHardSerial::synchronizeVariableRead(byte *txBuf, byte txLen, byte *rxBuf, byte *rxLen)
{
    //シリアル初期化確認
	if(pmxSerial == nullptr )
	{
		return false;
	}

    //他の物が通信中だった場合抜ける
    if(_isSynchronize == true)
    {
        return false;
    }

    //通信中にする
    _isSynchronize = true;

    //送信のみをする
    this->__synchronizeWrite(txBuf,txLen);

    //受信データを初期化しておく
    for(int i = 0; i < 256; i++)
    {
        receiveBuff[i] = 0xFF;
    }

    //データ数まで受信する
    byte minRxSize = PMX::MinimumLength::Receive - 2 ;//CRCを除く
    byte firstRxBuffSize = pmxSerial->readBytes(receiveBuff, minRxSize);

    //そもそもデータが返ってこなかった
    if(firstRxBuffSize != minRxSize)
    {
        *rxLen = 0;

        //通信中を解除する
        _isSynchronize = false;
        
        return false;
    }

    //　長さを取得する
    byte allRxSize = receiveBuff[PMX::BuffPter::Length];
    byte secondRxBuffSize = allRxSize - minRxSize;

    //上位のバッファに影響しないように内部の受信バッファに入れておく
	byte sBuffSize = pmxSerial->readBytes(&(receiveBuff[minRxSize]), secondRxBuffSize);

    //そもそもデータが返ってこなかった
    if(secondRxBuffSize != sBuffSize)
    {
        *rxLen = minRxSize;

        //通信中を解除する
        _isSynchronize = false;

        return false;
    }

    //受信データを反映させる
    *rxLen = allRxSize;
    //バッファ上のデータをコピーする
    memcpy(rxBuf, receiveBuff, allRxSize);

    //通信中を解除する
    _isSynchronize = false;

	return true;
}

/**
 * @brief ブロードキャストIDなどでデータの返事がないとわかっている時に送信をします。
 * 
 * @param [in] txBuf 送信データ
 * @param [in] txLen 送信データ数
 * 
 * @return true 送信成功
 * @return false 送信失敗
 */
bool PmxHardSerial::synchronizeNoRead(byte *txBuf, byte txLen)
{
        //シリアル初期化確認
	if(pmxSerial == nullptr )
	{
		return false;
	}

    //他の物が通信中だった場合抜ける
    if(_isSynchronize == true)
    {
        return false;
    }

    //通信中にする
    _isSynchronize = true;

    //送信のみをする
    this->__synchronizeWrite(txBuf,txLen);

    _isSynchronize = false;

    return true;

}

/**
 * @brief PmxHardSerialで使用する通信速度、パリティ、タイムアウトなどのシリアル パラメータを設定します。
 * 
 * @param [in] baudrate 通信速度 
 * @param [in] parity パリティ
 * @param [in] timeout 受信タイムアウト 
 * @return true 
 * @return false 
 */
// bool PmxHardSerial::setSerialParameters(long baudrate,byte parity,unsigned int timeout)
// {

//     return true;
// }

/**
 * @brief Pmxのコマンドを送信のみの関数
 * 
 * @param [in] txBuf 送信データ
 * @param [in] txLen 送信データ数
 */
void PmxHardSerial::__synchronizeWrite(byte *txBuf, byte txLen)
{
    //送信バッファをコピーする
    memcpy(sendBuff, txBuf, txLen);

	pmxSerial->flush(); //待つ

	enHigh(); //送信切替
	pmxSerial->write(sendBuff, txLen);
	pmxSerial->flush();   //待つ
	
	while (pmxSerial->available() > 0) //受信バッファを消す
	{
		// buff = icsSerial->read();	//空読み
		pmxSerial->read();		//空読み
    }

    enLow();  //受信切替
}


/**
 * @brief Pmxで送信したデータ配列を表示する関数
 * 
 * @param [in] outputBytes 表示する送信データ
 * @param [in] outputLength 表示する送信データ数
 */
void PmxHardSerial::logOutputPrint(byte outputBytes[],int outputLength)
{
    if(this->getLogSerial())
    {
        this->getLogSerial()->print("(");
        for(int i = 0; i < outputLength; i++)
        {
            this->getLogSerial()->print("[0x");
            this->getLogSerial()->print(outputBytes[i],HEX);
            this->getLogSerial()->print("]");
        }
        this->getLogSerial()->println(")");
    }
}

