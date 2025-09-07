
/** 
* @file PmxBaseClass.cpp
* @brief  PMX library source file
* @author Kondo Kagaku Co.,Ltd.
* @author kedtn(T.Nobuhara) Gotanda hn.Kondo
* @date 2024/07/09
* @version 1.0.3
* @copyright Kondo Kagaku Co.,Ltd. 2024
*
*/

#include "PmxBaseClass.h"
#include "PmxCRC.h"
#include "DataConvert.h"



/**
 * @brief checkRecv関数は、受信パケットのヘッダー、パケット長、およびコマンドをチェックします。
 * 
 * @details 受信データを解析し、下記エラーかどうかチェックを行います。
 * @details * ヘッダのチェック
 * @details * timeout(コマンドを何か受信したか)
 * @details * 受信データの長さ
 * @details * 受信したコマンドが一致するかどうか
 * @details * CRC
 * 
 * @param [in] rxBuff 受信したCRCを含む全てのデータ(バイト配列)
 * @param [in] cmd 受信時にチェックするコマンド
 * @param [in] header ヘッダーがデフォルトと違う場合に指定する
 * 
 * @return unsigned short 通信の状態(PMX::ComError参照) 
 */
unsigned short PmxBase::checkRecv(byte rxBuff[],byte cmd,byte header)
{
    // sync関数で正常なデータ数が来なかったらFalseなので、それ以外をチェック
    
    // ヘッダのチェック
    if(rxBuff[0] != header || rxBuff[1] != header)
    {
        Serial.println("HeaderError");
        return PMX::ComError::ReceiveError;
    }

    //コマンドのチェック
    if(rxBuff[PMX::BuffPter::CMD] != (cmd & 0x7f ))
    {
        Serial.println("cmdError");
        return PMX::ComError::ReceiveError;
    }

    // CRCチェック
    if(false == PmxCrc16::checkCrc16(rxBuff))
    {
        return PMX::ComError::CrcError;
    }

    return PMX::ComError::OK;

}



/**
 * @brief MemREADコマンドでPMXサーボモータからデータを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] addr データを読み取る先頭のアドレス
 * @param [in] readDataSize 取得するデータサイズ
 * @param [out] rxData 指定されたアドレスから読み取ったデータ配列(bytes型)。通信失敗時にはNoneを返します。
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::MemREAD(byte id, unsigned short addr, int readDataSize, byte rxData[])
{
    int PUT_MAX_DATA_LENGTH = 244;

    if(readDataSize == 0 || readDataSize >= PUT_MAX_DATA_LENGTH)
    {
        return PMX::ComError::FormatError;
    }

    int txSize = 11;    //memREADはLength固定
    int rxSize = 8 + readDataSize;
    
    byte txbuf[txSize];
    byte rxbuf[rxSize];


    txbuf[0] = (byte)(0xFE);           // HEADER
    txbuf[1] = (byte)(0xFE);           // HEADER
    txbuf[2] = (byte)(id);             // ID
    txbuf[3] = (byte)(txSize);        // LENGTH
    txbuf[4] = PMX::SendCmd::MemREAD; //Comannd
    txbuf[5] = (byte)(0x00);           // OPTION
    txbuf[6] = (byte)(addr & 0x00ff);           //先頭アドレス（下位）
    txbuf[7] = (byte)((addr & 0xff00) >> 8);    //先頭アドレス（上位）
    txbuf[8] = (byte)(readDataSize);      // データサイズ

    PmxCrc16::setCrc16(txbuf);

    bool rxFlag= this->synchronize(txbuf, txSize , rxbuf, rxSize);

    this->logOutputPrint(txbuf,txSize);

    if(rxFlag == false)
    {
        //Serial.println("timeout");
        
        //配列データはエラー値を入れておく
        for(int i = 0; i < readDataSize; i++)
        {
            rxData[i] = 0xFF;
        }

        return PMX::ComError::TimeOut;
    }

    this->logOutputPrint(rxbuf,rxSize);

    unsigned short errorFlag = this->checkRecv(rxbuf, PMX::SendCmd::MemREAD);


    if(errorFlag != PMX::ComError::OK)  //ここの部分はステータスを含まないのでOK
    {
        
        return errorFlag;
    }

    unsigned short status = rxbuf[PMX::BuffPter::Status];

    for(int i = 0; i < readDataSize ; i++)
    {
        rxData[i] = rxbuf[PMX::BuffPter::Data + i];
    }

    return status;

}

/**
 * @brief MemREADで取得したデータをbyte(符号なし1byte)に変換して取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] addr データを読み取る先頭のアドレス
 * @param [out] byteData 指定されたアドレスから読み取ったbyte(符号なし1byte)のデータ。通信失敗時にはNoneを返します。
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::MemREADToByte(byte id, unsigned short addr , byte *byteData)
{
    unsigned short status = this->MemREAD(id, addr, 1, byteData);

    //通信エラーの時は0xFFを返す
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *byteData = PMX::ErrorByteData;
    }

    return status;
}

/**
 * @brief MemREADで取得したデータをint16(符号あり2byte)に変換して取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] addr データを読み取る先頭のアドレス
 * @param [out] int16Data 指定されたアドレスから読み取ったint16(符号あり2byte)のデータ。通信失敗時にはNoneを返します。
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::MemREADToInt16(byte id, unsigned short addr , short *int16Data)
{
    byte reByte[2];
    unsigned short status = this->MemREAD(id, addr, 2, reByte);

    *int16Data = DataConv::bytesToInt16(reByte);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *int16Data = (short)PMX::ErrorUint16Data;
    }

    return status;
}

/**
 * @brief MemREADで取得したデータをUint16(符号なし2byte)に変換して取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] addr データを読み取る先頭のアドレス
 * @param [out] uint16Data 指定されたアドレスから読み取ったUint16(符号なし2byte)のデータ。通信失敗時にはNoneを返します。
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::MemREADToUint16(byte id, unsigned short addr , unsigned short *uint16Data)
{
    byte reByte[2];
    unsigned short status = this->MemREAD(id, addr, 2, reByte);

    *uint16Data = DataConv::bytesToInt16(reByte);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *uint16Data = (short)PMX::ErrorUint16Data;
    }

    return status;
}


/**
 * @brief MemREADで取得したデータをint32(符号あり4byte)に変換して取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] addr データを読み取る先頭のアドレス
 * @param [out] int32Data 指定されたアドレスから読み取ったint32(符号あり4byte)のデータ。通信失敗時にはNoneを返します。
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::MemREADToInt32(byte id, unsigned short addr , long *int32Data)
{
    byte reByte[4];
    unsigned short status = this->MemREAD(id, addr, 4, reByte);

    *int32Data = DataConv::bytesToInt32(reByte);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *int32Data = (short)PMX::ErrorUint32Data;
    }

    return status;
}

/**
 * @brief MemREADで取得したデータをUint32(符号なし4byte)に変換して取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] addr データを読み取る先頭のアドレス
 * @param [out] uint32Data 指定されたアドレスから読み取ったUint32(符号なし4byte)のデータ。通信失敗時にはNoneを返します。
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::MemREADToUint32(byte id, unsigned short addr , unsigned long *uint32Data)
{
    byte reByte[4];
    unsigned short status = this->MemREAD(id, addr, 4, reByte);

    *uint32Data = DataConv::bytesToInt32(reByte);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *uint32Data = (short)PMX::ErrorUint32Data;
    }

    return status;
}



/**
 * @brief PMXサーボモータにMemWRITEコマンドを発行し、結果を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] addr データを書き込む先頭アドレス
 * @param [in] txDataArray 書き込むデータのbyteリスト
 * @param [in] txDataSize 書き込むデータサイズ(txDataArrayのサイズ) 
 * @param [in] writeOpt MemWRITEで使用するオプション 0:通常書き込み、1:TorqueOn中の強制書き込み
 * 
 * @return unsigned short MemWRITEの送受信結果(送信結果+PMXのstatus)
 */
unsigned short PmxBase::MemWRITE(byte id, unsigned short addr, byte txDataArray[], int txDataSize, byte writeOpt)
{
    int PUT_MAX_DATA_LENGTH = 245;

    // byte writeDataBuff[2];
    // DataConv::bytesToShort(1000, writeDataBuff);

    // 最大値の判定
    if(txDataSize == 0 || txDataSize >= PUT_MAX_DATA_LENGTH)
    {
        return PMX::ComError::FormatError;
    }

    int txSize = PMX::MinimumLength::Send + 2 + txDataSize;    //memREADはLength固定
    int rxSize = PMX::MinimumLength::Receive;
    
    
    byte txbuf[txSize];
    byte rxbuf[rxSize];

    txbuf[0] = (byte)(0xFE);            // HEADER
    txbuf[1] = (byte)(0xFE);            // HEADER
    txbuf[2] = (byte)(id);              // ID
    txbuf[3] = (byte)(txSize);          // LENGTH
    txbuf[4] = PMX::SendCmd::MemWRITE;  //Comannd
    txbuf[5] = (byte)(writeOpt);        // OPTION
    txbuf[6] = (byte)(addr & 0x00ff);           //先頭アドレス（下位）
    txbuf[7] = (byte)((addr & 0xff00) >> 8);    //先頭アドレス（上位）

    for(int i = 0; i < txDataSize; i++){
        txbuf[8+i] = (byte)(txDataArray[i]); //書込データ
    }

    PmxCrc16::setCrc16(txbuf);

    bool rxFlag= this->synchronize(txbuf, txSize , rxbuf, rxSize);

    this->logOutputPrint(txbuf,txSize);

    if(rxFlag == false)
    {
        //Serial.println("timeout");
        return PMX::ComError::TimeOut;
    }

    this->logOutputPrint(rxbuf,rxSize);

    unsigned short errorFlag = this->checkRecv(rxbuf, PMX::SendCmd::MemWRITE);


    if(errorFlag != PMX::ComError::OK)  //ここの部分はステータスを含まないのでOK
    {
        return errorFlag;
    }

    unsigned short status = rxbuf[PMX::BuffPter::Status];

    return status;
}


/**
 * @brief MemWRITEでbyte(符号なし1byte)のデータを書き込みます。その結果のステータス情報を返信します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] addr データを書き込む先頭アドレス
 * @param [in] byteData byte(符号なし1byte)の書きこみデータ
 * @param [in] writeOpt MemWRITEで使用するオプション 0:通常書き込み、1:TorqueOn中の強制書き込み
 * 
 * @return unsigned short MemWRITEの送受信結果(送信結果+PMXのstatus)
 */
unsigned short PmxBase::MemWRITEToByte(byte id, unsigned short addr, byte byteData, byte writeOpt)
{
    unsigned short status = this->MemWRITE(id, addr, &byteData, 1, writeOpt);

    return status;
}


/**
 * @brief MemWRITEでint16(符号あり2byte)のデータを書き込みます。その結果のステータス情報を返信します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] addr データを書き込む先頭アドレス
 * @param [in] int16Data int16(符号あり2byte)の書きこみデータ
 * @param [in] writeOpt MemWRITEで使用するオプション 0:通常書き込み、1:TorqueOn中の強制書き込み
 * 
 * @return unsigned short MemWRITEの送受信結果(送信結果+PMXのstatus)
 */
unsigned short PmxBase::MemWRITEToInt16(byte id, unsigned short addr, short int16Data, byte writeOpt)
{
    byte txDataArray[2];
    DataConv::int16ToBytes(int16Data, txDataArray);
    unsigned short status = this->MemWRITE(id, addr, txDataArray, 2, writeOpt);

    return status;
}


/**
 *  @brief MemWRITEでUint16(符号なし2byte)のデータを書き込みます。その結果のステータス情報を返信します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] addr データを書き込む先頭アドレス
 * @param [in] uint16Data Uint16(符号なし2byte)の書きこみデータ
 * @param [in] writeOpt MemWRITEで使用するオプション 0:通常書き込み、1:TorqueOn中の強制書き込み
 * 
 * @return unsigned short MemWRITEの送受信結果(送信結果+PMXのstatus)
 */
unsigned short PmxBase::MemWRITEToUint16(byte id, unsigned short addr, unsigned short uint16Data, byte writeOpt)
{
    byte txDataArray[2];
    DataConv::uint16ToBytes(uint16Data, txDataArray);
    unsigned short status = this->MemWRITE(id, addr, txDataArray, 2, writeOpt);

    return status;
}


/**
 * @brief MemWRITEでint32(符号あり4byte)のデータを書き込みます。その結果のステータス情報を返信します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] addr データを書き込む先頭アドレス
 * @param [in] int32Data int32(符号あり4byte)の書きこみデータ
 * @param [in] writeOpt MemWRITEで使用するオプション 0:通常書き込み、1:TorqueOn中の強制書き込み
 * 
 * @return unsigned short MemWRITEの送受信結果(送信結果+PMXのstatus)
 */
unsigned short PmxBase::MemWRITEToInt32(byte id, unsigned short addr, long int32Data, byte writeOpt)
{
    byte txDataArray[4];
    DataConv::int32ToBytes(int32Data, txDataArray);
    unsigned short status = this->MemWRITE(id, addr, txDataArray, 4, writeOpt);

    return status;
}


/**
 * @brief MemWRITEでUint32(符号なし4byte)のデータを書き込みます。その結果のステータス情報を返信します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] addr データを書き込む先頭アドレス
 * @param [in] uint32Data Uint32(符号なし4byte)の書きこみデータ
 * @param [in] writeOpt MemWRITEで使用するオプション 0:通常書き込み、1:TorqueOn中の強制書き込み
 * 
 * @return unsigned short MemWRITEの送受信結果(送信結果+PMXのstatus)
 */
unsigned short PmxBase::MemWRITEToUint32(byte id, unsigned short addr, unsigned long uint32Data, byte writeOpt)
{
    byte txDataArray[4];
    DataConv::uint32ToBytes(uint32Data, txDataArray);
    unsigned short status = this->MemWRITE(id, addr, txDataArray, 4, writeOpt);

    return status;
}


/**
 * @brief PMXサーボモータのLOADコマンドを発行し、ステータス結果を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * 
 * @return unsigned short LOADの送受信結果(送信結果+PMXのstatus)
 */
unsigned short PmxBase::LOAD(byte id)
{

    int txSize = PMX::MinimumLength::Send;    
    int rxSize = PMX::MinimumLength::Receive;
    
    byte txbuf[txSize];
    byte rxbuf[rxSize];

    txbuf[0] = (byte)(0xFE);            // HEADER
    txbuf[1] = (byte)(0xFE);            // HEADER
    txbuf[2] = (byte)(id);              // ID
    txbuf[3] = (byte)(txSize);          // LENGTH
    txbuf[4] = PMX::SendCmd::LOAD;      //Comannd
    txbuf[5] = (byte)(0x00);            // OPTION

    PmxCrc16::setCrc16(txbuf);

    bool rxFlag= this->synchronize(txbuf, txSize , rxbuf, rxSize);

    this->logOutputPrint(txbuf,txSize);

    if(rxFlag == false)
    {
        Serial.println("timeout");
        return PMX::ComError::TimeOut;
    }

    this->logOutputPrint(rxbuf,rxSize);

    unsigned short errorFlag = this->checkRecv(rxbuf, PMX::SendCmd::LOAD);


    if(errorFlag != PMX::ComError::OK)  //ここの部分はステータスを含まないのでOK
    {
        return errorFlag;
    }

    unsigned short status = rxbuf[PMX::BuffPter::Status];

    return status;
}


/**
 * @brief PMXサーボモータのSAVEコマンドを発行し、ステータス結果を取得します。
 * 
 * @param [in] id サーボモータのID番号
 * 
 * @return unsigned short SAVEコマンドの送信結果(送信結果+PMXのstatus)
 */
unsigned short PmxBase::SAVE(byte id)
{

    int txSize = PMX::MinimumLength::Send;    
    int rxSize = PMX::MinimumLength::Receive;
    
    byte txbuf[txSize];
    byte rxbuf[rxSize];

    txbuf[0] = (byte)(0xFE);            // HEADER
    txbuf[1] = (byte)(0xFE);            // HEADER
    txbuf[2] = (byte)(id);              // ID
    txbuf[3] = (byte)(txSize);          // LENGTH
    txbuf[4] = PMX::SendCmd::SAVE;      //Comannd
    txbuf[5] = (byte)(0x00);            // OPTION

    PmxCrc16::setCrc16(txbuf);

    bool rxFlag= this->synchronize(txbuf, txSize , rxbuf, rxSize);

    this->logOutputPrint(txbuf,txSize);

    if(rxFlag == false)
    {
        Serial.println("timeout");
        return PMX::ComError::TimeOut;
    }

    this->logOutputPrint(rxbuf,rxSize);

    unsigned short errorFlag = this->checkRecv(rxbuf, PMX::SendCmd::SAVE);


    if(errorFlag != PMX::ComError::OK)  //ここの部分はステータスを含まないのでOK
    {
        return errorFlag;
    }

    unsigned short status = rxbuf[PMX::BuffPter::Status];

    return status;
}


/**
 * @brief MotorREAD関数を発行し、応答モードに応じてデータを取得します。
 * 
 * @param [in] id サーボモータのID番号
 * @param [in] receiveMode 応答モード。事前に応答モードをMemWrite関数やsetMotorReceive関数で設定する必要があります。
 * @param [out] readMotorData MotorReadで読み取ったデータ配列[位置,速度,電流,トルク,PWM,モータ温度,CPU温度,電圧](合計8項目)
 * @param [in] controlMode 制御モード。デフォルト値は `0x01` です。位置制御モードの時の現在位置を判定するときに使用します。
 * @param [in] torqueSw FreeやTorqueOnの値(PMX::TorqueSwitchTypeを参照)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention rxDataListでデータがない場合はリストの中身の各データがNoneになります。
 */
unsigned short PmxBase::MotorREAD(byte id, byte receiveMode, long readMotorData[8], byte controlMode, byte *torqueSw)
{
    int readDataSize = this->__byteCounter(receiveMode);

    int txSize = PMX::MinimumLength::Send;    //MotorREADはLength固定
    int rxSize = PMX::MinimumLength::Receive + 1 + readDataSize;
    
    byte txbuf[txSize];
    byte rxbuf[rxSize];
    
    //返すデータは初期化しておく
    for(int i = 0;i<8;i++)
    {
        readMotorData[i] = PMX::ErrorUint32Data;
    }

    if(torqueSw != NULL)
    {
        *torqueSw = PMX::TorqueSwitchType::Error;
    }

    txbuf[0] = (byte)(0xFE);           // HEADER
    txbuf[1] = (byte)(0xFE);           // HEADER
    txbuf[2] = (byte)(id);             // ID
    txbuf[3] = (byte)(txSize);        // LENGTH
    txbuf[4] = PMX::SendCmd::MotorREAD; //Comannd
    txbuf[5] = (byte)(0x00);           // OPTION

    PmxCrc16::setCrc16(txbuf);

    //bool rxFlag= this->synchronize(txbuf, txSize , rxbuf, rxSize);

    byte rxNowSize;
    bool rxFlag = this->synchronizeVariableRead(txbuf, txSize , rxbuf, &rxNowSize);

    //Serial.println(rxNowSize);

    this->logOutputPrint(txbuf,txSize);

    if(rxFlag == false)
    {
        return PMX::ComError::TimeOut;
    }

    this->logOutputPrint(rxbuf,rxNowSize);

    unsigned short errorFlag = this->checkRecv(rxbuf, PMX::SendCmd::MotorREAD);
    if(errorFlag != PMX::ComError::OK)  //ここの部分はステータスを含まないのでOK
    {
        return errorFlag;
    }


    //トルクスイッチ情報を代入
    if((torqueSw != NULL) && (rxNowSize >= PMX::MinimumLength::Receive))
    {
        *torqueSw = rxbuf[PMX::BuffPter::Data];
    }



    //ステータス情報を代入
    unsigned short status = rxbuf[PMX::BuffPter::Status];



    if(rxNowSize != rxSize) //実際受信した情報と予想していた情報が違う
    {
        status += PMX::ComError::MotorREADConvertError;  //受信エラーを含んだ情報を返す
        return  status;
    }

    byte rxMotorData[readDataSize];  //受信したデータをためておくバッファ

    //受信データを抜き出す
    for(int i = 0; i < readDataSize ; i++)
    {
        rxMotorData[i] = rxbuf[PMX::BuffPter::Data + 1 + i];
    }

    //　データの代入
    bool flag = this->__convReceiveMotorData(receiveMode, rxMotorData, readDataSize, readMotorData, controlMode);
    if(flag == false)   //何かしら失敗したらエラーを付加して抜ける
    {
        status += PMX::ComError::MotorREADConvertError;  //受信エラーを含んだ情報を返す
        return  status;
    }

    return status;
}


//制御モード、応答モードからByte数をカウントする
/**
 * @brief 制御モード、応答モードを基に必要とするデータが合計何byteになるのか計算をします。
 * 
 * @param [in] val 応答モードもしくは制御モード
 * 
 * @return 制御モードや応答モードに必要とするデータ(byte)数
 */
unsigned int PmxBase::__byteCounter(byte val)
{

    
    int cnt = 0;

    for(int i = 0; i < 8; i++){
        int check = val & 0x01;
        if(check == 1){cnt++;}
        val = val >> 1;
    }
    
    //1data2byteなので、2倍の数値を返す
    return cnt * 2;
}


/**
 * @brief 受信モードに基づいてMotorWRITE/Readで送られて来たデータを変換します。
 * 
 * @param [in] receiveMode 受信モード設定
 * @param [in] receiveBytes 受信されたデータのbyte配列（コマンド等は含まない）
 * @param [in] receiveBytesSize 受信データ (コマンドを除く) を含む、receiveBytes 配列のサイズ。
 * @param [out] motorData 受信データを変換したモータデータを格納する配列。[位置,速度,電流,トルク,PWM,モータ温度,CPU温度,電圧]
 * @param [in] controlMode 制御モード。デフォルト値は `0x01` です。位置制御モードの時の現在位置を判定するときに使用します。
 * @return 受信データを変換したモータデータを返す[位置,速度,電流,トルク,PWM,モータ温度,CPU温度,電圧]
 * @attention データがない場合はリストの中身の各データがNoneになります
 */
bool PmxBase::__convReceiveMotorData(byte receiveMode, byte receiveBytes[], byte receiveBytesSize, long motorData[], byte controlMode)
{

    for(int i = 0;i<8;i++)
    {
        motorData[i] = PMX::ErrorUint32Data;
    }

    //データbyte数と変換するデータ数が違っていたら空リストを返す
    if (receiveBytesSize != (__byteCounter(receiveMode)))
    {
        return false;
    }

    int arrayCount = 0;  // データを数える(bytes配列の場所を決めるのに使用)
    
    //
    //  位置データ
    //
    if((receiveMode & PMX::ReceiveDataOption::Position) == PMX::ReceiveDataOption::Position)
    {

        //位置制御モードの時
        if((controlMode & PMX::ControlMode::Position) == PMX::ControlMode::Position)
        {
            motorData[0] = (long)DataConv::bytesToInt16(&(receiveBytes[arrayCount]));
        } 
        else
        {
            motorData[0] = (long)DataConv::bytesToUint16(&(receiveBytes[arrayCount]));
        } 

        arrayCount += 2;
        //listCount++;
    }

    //
    //  速度データ
    //
    if((receiveMode & PMX::ReceiveDataOption::Speed) == PMX::ReceiveDataOption::Speed)
    {
        motorData[1] = (long)DataConv::bytesToInt16(&(receiveBytes[arrayCount]));
        arrayCount += 2;
    }

    //
    //  電流データ
    //
    if((receiveMode & PMX::ReceiveDataOption::Current) == PMX::ReceiveDataOption::Current)
    {
        motorData[2] = (long)DataConv::bytesToInt16(&(receiveBytes[arrayCount]));
        arrayCount += 2;
    }

    //
    //  トルクデータ
    //
    if((receiveMode & PMX::ReceiveDataOption::Torque) == PMX::ReceiveDataOption::Torque)
    {
        motorData[3] = (long)DataConv::bytesToInt16(&(receiveBytes[arrayCount]));
        arrayCount += 2;
    }

    //
    //  PWMデータ
    //
    if((receiveMode & PMX::ReceiveDataOption::Pwm) == PMX::ReceiveDataOption::Pwm)
    {
        motorData[4] = (long)DataConv::bytesToInt16(&(receiveBytes[arrayCount]));
        arrayCount += 2;
    }

    //
    //  モータ温度データ
    //
    if((receiveMode & PMX::ReceiveDataOption::MotorTemp) == PMX::ReceiveDataOption::MotorTemp)
    {
        motorData[5] = (long)DataConv::bytesToInt16(&(receiveBytes[arrayCount]));
        arrayCount += 2;
    }

    //
    //  CPU温度データ
    //
    if((receiveMode & PMX::ReceiveDataOption::CpuTemp) == PMX::ReceiveDataOption::CpuTemp)
    {
        motorData[6] = (long)DataConv::bytesToInt16(&(receiveBytes[arrayCount]));
        arrayCount += 2;
    }

    //
    //  電圧データ
    //
    if((receiveMode & PMX::ReceiveDataOption::Voltage) == PMX::ReceiveDataOption::Voltage)
    {
        motorData[7] = (long)DataConv::bytesToUint16(&(receiveBytes[arrayCount]));
        arrayCount += 2;
    }


    return true;
}


/**
 * @brief MotorWRITE 機能は、モータの制御やトルクのオン/オフを切り替えるコマンドを送信し、モーターからステータスとデータを受信します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] toruqeOnSw FreeやTorqueOnの値(PMX::TorqueSwitchTypeを参照)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::MotorWRITE(byte id, byte toruqeOnSw)
{
    byte reDummyReMode = PMX::ReceiveDataOption::NoReturn;
    long dummyDatas[8];

    return this->MotorWRITE(id, toruqeOnSw, reDummyReMode, dummyDatas);
}


/**
 * @brief MotorWRITE 機能は、モータの制御やトルクのオン/オフを切り替えるコマンドを送信し、モーターからステータスとデータを受信します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] toruqeOnSw FreeやTorqueOnの値(PMX::TorqueSwitchTypeを参照)
 * @param [in] receiveMode 応答モード。返ってきたデータを変換するときに使用します。
 * @param [out] receiveData MotorReadで読み取ったデータ[位置,速度,電流,トルク,PWM,モータ温度,CPU温度,電圧](合計8項目)
 * @param [in] controlMode モータの制御モード。応答モードで現在位置を取得する時のみに使用します
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::MotorWRITE(byte id, byte toruqeOnSw, byte receiveMode, long receiveData[8], byte controlMode)
{
    //
    //  トルクON等トルクスイッチをいじる関数
    //



    //返すデータは初期化しておく
    if(receiveData != NULL)
    {    
        for(int i = 0;i<8;i++)
        {
            receiveData[i] = PMX::ErrorUint32Data;
        }
    }


    // トルクスイッチが規定値にない
    if( ! (toruqeOnSw == PMX::TorqueSwitchType::TorqueOn ||
        toruqeOnSw == PMX::TorqueSwitchType::Free ||
        toruqeOnSw == PMX::TorqueSwitchType::Brake ||
        toruqeOnSw == PMX::TorqueSwitchType::Hold ) )
    {
        return PMX::ComError::FormatError;
    }

    int readDataSize = this->__byteCounter(receiveMode);;
    

    int txSize = PMX::MinimumLength::Send;    //MotorREADはLength固定
    int rxSize = PMX::MinimumLength::Receive + 1 + readDataSize;
    
    byte txbuf[txSize]; //トルクスイッチ時は送信データ固定
    byte rxbuf[26]; //motorwriteの最大値

    
    txbuf[0] = (byte)(0xFE);           // HEADER
    txbuf[1] = (byte)(0xFE);           // HEADER
    txbuf[2] = (byte)(id);             // ID
    txbuf[3] = (byte)(txSize);        // LENGTH
    txbuf[4] = PMX::SendCmd::MotorWRITE; //Comannd
    txbuf[5] = (byte)(toruqeOnSw);           // OPTION

    PmxCrc16::setCrc16(txbuf);

    
    byte rxNowSize;
    bool rxFlag = this->synchronizeVariableRead(txbuf, txSize , rxbuf, &rxNowSize);

    //Serial.println(rxNowSize);

    this->logOutputPrint(txbuf,txSize);

    //データが来てたかどうか
    if(rxFlag == false)
    {
        return PMX::ComError::TimeOut;
    }

    //データの判定
    unsigned short errorFlag = this->checkRecv(rxbuf, PMX::SendCmd::MotorWRITE);
    if(errorFlag != PMX::ComError::OK)  //ここの部分はステータスを含まないのでOK
    {
        return errorFlag;
    }

    this->logOutputPrint(rxbuf,rxNowSize);

    //トルクスイッチ情報を代入
    // if((torqueSw != NULL) && (rxNowSize >= PMX::MinimumLength::Receive))
    // {
    //     *torqueSw = rxbuf[PMX::BuffPter::Data];
    // }

    //ステータス情報を代入
    unsigned short status = rxbuf[PMX::BuffPter::Status];

    

    //データを返さなくていい時はtrueで返す
    if(receiveMode == PMX::ReceiveDataOption::NoReturn)
    {
        return status;
    }

    // ↓↓↓↓↓↓ここからはReceiveDataの処理↓↓↓↓↓

    if(rxNowSize != rxSize) //実際受信した情報と予想していた情報が違う
    {
        status += PMX::ComError::ReceiveError;  //受信エラーを含んだ情報を返す
        return  status;
    }


    byte rxMotorData[readDataSize];  //受信したデータをためておくバッファ

    //受信データを抜き出す
    for(int i = 0; i < readDataSize ; i++)
    {
        rxMotorData[i] = rxbuf[PMX::BuffPter::Data + 1 + i];
    }

    //　データの代入
    bool flag = this->__convReceiveMotorData(receiveMode, rxMotorData, readDataSize, receiveData, controlMode);
    if(flag == false)   //何かしら失敗したらエラーを付加して抜ける
    {
        status += PMX::ComError::MotorREADConvertError;  //受信エラーを含んだ情報を返す
        return  status;
    }





    return status;

}

/**
 * @brief MotorWRITE 機能は、モータの制御やトルクのオン/オフを切り替えるコマンドを送信し、モーターからステータスとデータを受信します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] writeDatas モータへの指示値を格納
 * @param [in] writeDataCount writeDatas 配列内の要素数
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::MotorWRITE(byte id, long writeDatas[], int writeDataCount)
{
    byte reDummyMode = PMX::ReceiveDataOption::NoReturn;
    long dummyDatas[8];

    return this->MotorWRITE(id,writeDatas,writeDataCount,reDummyMode,dummyDatas);
}


/**
 * @brief MotorWRITE 機能は、モータの制御やトルクのオン/オフを切り替えるコマンドを送信し、モーターからステータスとデータを受信します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] writeDatas モータへの指示値を格納
 * @param [in] writeDataCount writeDatas 配列内の要素数
 * @param [in] receiveMode 応答モード。返ってきたデータを変換するときに使用します。
 * @param [out] receiveData MotorReadで読み取ったデータ[位置,速度,電流,トルク,PWM,モータ温度,CPU温度,電圧](合計8項目)
 * @param [in] controlMode モータの制御モード。応答モードで現在位置を取得する時のみに使用します
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention writeDatasは、位置 > 速度 > 電流 > トルク > PWM > 時間 の順番に格納される
 * @attention 位置/電流制御モードの場合は[position,current]をwriteDatasに渡します
 * @attention writeDatasは使わないデータ(位置/電流の時のPWM等)は書かない
 * 
 * @warning writeDatasの順番を間違えない事
 * @warning 複数組み合わせの場合で１つだけ変えるときでも全ての目標値を送ってください。
 * @warning controlModeやreceiveModeは事前に設定をする事
 * 
 * @attention rxDataListでデータがない場合はリストの中身の各データがNoneになります
 * 
 */
unsigned short PmxBase::MotorWRITE(byte id, long writeDatas[], int writeDataCount, byte receiveMode, long receiveData[8], byte controlMode)
{
    //
    //  モータに指令値を与える関数
    //

    
    
    //返すデータは初期化しておく
    if(receiveData != NULL)
    {    
        for(int i = 0;i<8;i++)
        {
            receiveData[i] = PMX::ErrorUint32Data;
        }
    }



    // int readDataSize = 0;
    // if( receiveMode != 0x00)
    // {
    //     readDataSize = this->__byteCounter(receiveMode);
    // }     

    int readDataSize = this->__byteCounter(receiveMode);

    int txSize = PMX::MinimumLength::Send + (writeDataCount * 2);   
    int rxSize = PMX::MinimumLength::Receive + 1 + readDataSize;
    
    byte txbuf[txSize]; //トルクスイッチ時は送信データ固定
    byte rxbuf[26]; //motorwriteの最大値

    
    txbuf[0] = (byte)(0xFE);           // HEADER
    txbuf[1] = (byte)(0xFE);           // HEADER
    txbuf[2] = (byte)(id);             // ID
    txbuf[3] = (byte)(txSize);        // LENGTH
    txbuf[4] = PMX::SendCmd::MotorWRITE; //Comannd
    txbuf[5] = (byte)(0x00);           // OPTION

    //送るデータをbyteに変換して代入する
    for(int i = 0 ; i < writeDataCount ; i++)
    {
        //short型、unsigned short型があるのでそれぞれ分ける
        byte wordBuff[2];
        if(writeDatas[i] < 0)
        {
            DataConv::int16ToBytes((short)writeDatas[i], wordBuff);
        }
        else
        {
            DataConv::uint16ToBytes((short)writeDatas[i], wordBuff);
        }

        txbuf[PMX::BuffPter::Data + (i*2)] = wordBuff[0];
        txbuf[PMX::BuffPter::Data + (i*2) + 1] = wordBuff[1];
    }

    //　CRCの代入
    PmxCrc16::setCrc16(txbuf);

    //　送受信をする（受信数不明版
    byte rxNowSize;
    bool rxFlag = this->synchronizeVariableRead(txbuf, txSize , rxbuf, &rxNowSize);

    //Serial.println(rxNowSize);

    this->logOutputPrint(txbuf,txSize);

    //データが来てたかどうか
    if(rxFlag == false)
    {
        return PMX::ComError::TimeOut;
    }

    //データの判定
    unsigned short errorFlag = this->checkRecv(rxbuf, PMX::SendCmd::MotorWRITE);
    if(errorFlag != PMX::ComError::OK)  //ここの部分はステータスを含まないのでOK
    {
        return errorFlag;
    }

    this->logOutputPrint(rxbuf,rxNowSize);

    //トルクスイッチ情報を代入
    // if((torqueSw != NULL) && (rxNowSize >= PMX::MinimumLength::Receive))
    // {
    //     *torqueSw = rxbuf[PMX::BuffPter::Data];
    // }

    //ステータス情報を代入
    unsigned short status = rxbuf[PMX::BuffPter::Status];

    

    //データを返さなくていい時はtrueで返す
    if(receiveMode == PMX::ReceiveDataOption::NoReturn)
    {
        return status;
    }

    // ↓↓↓↓↓↓ここからはReceiveDataの処理↓↓↓↓↓

    if(rxNowSize != rxSize) //実際受信した情報と予想していた情報が違う
    {
        status += PMX::ComError::MotorREADConvertError;  //変換エラーを含んだ情報を返す
        return  status;
    }


    byte rxMotorData[readDataSize];  //受信したデータをためておくバッファ

    //受信データを抜き出す
    for(int i = 0; i < readDataSize ; i++)
    {
        rxMotorData[i] = rxbuf[PMX::BuffPter::Data + 1 + i];
    }

    //　データの代入
    bool flag = this->__convReceiveMotorData(receiveMode, rxMotorData, readDataSize, receiveData, controlMode);
    if(flag == false)   //何かしら失敗したらエラーを付加して抜ける
    {
        status += PMX::ComError::MotorREADConvertError;  //受信エラーを含んだ情報を返す
        return  status;
    }

    //ここまでこれば合格
    return status;
}

/**
 * @brief MotorWRITEでデータ指示一つの場合に使用します
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] targetVal 目標値(位置、速度、電流、トルク、PWM)が入ります
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::MotorWRITESingle(byte id, long targetVal)
{
    long sendData[] = {targetVal};
    return this->MotorWRITE(id, sendData, 1);
}

/**
 * @brief MotorWRITEでデータ指示一つの場合に使用します
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] targetVal 目標値(位置、速度、電流、トルク、PWM)が入ります
 * @param [in] receiveMode 応答モード。返ってきたデータを変換するときに使用します。
 * @param [out] receiveData MotorReadで読み取ったデータ[位置,速度,電流,トルク,PWM,モータ温度,CPU温度,電圧](合計8項目)
 * @param [in] controlMode モータの制御モード。応答モードで現在位置を取得する時のみに使用します。
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::MotorWRITESingle(byte id, long targetVal,byte receiveMode, long receiveData[8], byte controlMode)
{
    long sendData[] = {targetVal};
    return this->MotorWRITE(id, sendData, 1, receiveMode, receiveData, controlMode);
}



/**
 * @brief MotorWRITEを使用して2変数を指示するときに使用します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] targetVal1 目標値1(位置、速度、電流、トルク、PWM)が入ります
 * @param [in] targetVal2 目標値2(速度、電流、トルク、PWM、時間)が入ります
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::MotorWRITEDouble(byte id, long targetVal1, long targetVal2)
{
    long sendData[] = {targetVal1,targetVal2};
    return this->MotorWRITE(id, sendData, 2);
}

/**
 * @brief MotorWRITEを使用して2変数を指示するときに使用します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] targetVal1 目標値1(位置、速度、電流、トルク、PWM)が入ります
 * @param [in] targetVal2 目標値2(速度、電流、トルク、PWM、時間)が入ります
 * @param [in] receiveMode 応答モード。返ってきたデータを変換するときに使用します。
 * @param [out] receiveData MotorReadで読み取ったデータ[位置,速度,電流,トルク,PWM,モータ温度,CPU温度,電圧](合計8項目)
 * @param [in] controlMode モータの制御モード。応答モードで現在位置を取得する時のみに使用します。
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::MotorWRITEDouble(byte id, long targetVal1, long targetVal2,byte receiveMode, long receiveData[8], byte controlMode)
{
    long sendData[] = {targetVal1,targetVal2};
    return this->MotorWRITE(id, sendData, 2, receiveMode, receiveData, controlMode);
}

/**
 * @brief MotorWRITEを使用して3変数を指示するときに使用します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] targetVal1 目標値1(位置、速度、電流、トルク、PWM)のどれかが入ります
 * @param [in] targetVal2 目標値2(速度、電流、トルク、PWM、時間)のどれかが入ります
 * @param [in] targetVal3 目標値3(電流、トルク、PWM、時間)のどれかが入ります
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::MotorWRITETriple(byte id, long targetVal1, long targetVal2, long targetVal3)
{
    long sendData[] = {targetVal1,targetVal2,targetVal3};
    return this->MotorWRITE(id, sendData, 3);
}

/**
 * @brief MotorWRITEを使用して3変数を指示するときに使用します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] targetVal1 目標値1(位置、速度、電流、トルク、PWM)のどれかが入ります
 * @param [in] targetVal2 目標値2(速度、電流、トルク、PWM、時間)のどれかが入ります
 * @param [in] targetVal3 目標値3(電流、トルク、PWM、時間)のどれかが入ります
 * @param [in] receiveMode 応答モード。返ってきたデータを変換するときに使用します。
 * @param [out] receiveData MotorReadで読み取ったデータ[位置,速度,電流,トルク,PWM,モータ温度,CPU温度,電圧](合計8項目)
 * @param [in] controlMode モータの制御モード。応答モードで現在位置を取得する時のみに使用します。
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::MotorWRITETriple(byte id, long targetVal1, long targetVal2,long targetVal3,byte receiveMode, long receiveData[8], byte controlMode)
{
    long sendData[] = {targetVal1,targetVal2,targetVal3};
    return this->MotorWRITE(id, sendData, 3, receiveMode, receiveData, controlMode);
}




/**
 * @brief SystemREAD 関数は、サーボからシステム情報を読み取ります。
 * 
 * @details 読み取ることができるデータは下記になります。
 * @details * PMX個体固有のシリアル番号
 * @details * PMXの機種番号
 * @details * バージョンの情報
 * @details * コマンドを受け取ってから返信するまでの時間
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] rxData SystemREADで取得したすべてのデータを返します。[SerialNum, ModelNum, Version, responsTime]
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::SystemREAD(byte id, byte rxData[13])
{
    int readDataSize = 13;
    int txSize = PMX::MinimumLength::Send;    //memREADはLength固定
    int rxSize = PMX::MinimumLength::Receive + readDataSize;
    
    byte txbuf[txSize];
    byte rxbuf[rxSize];

    txbuf[0] = (byte)(0xFE);            // HEADER
    txbuf[1] = (byte)(0xFE);            // HEADER
    txbuf[2] = (byte)(id);              // ID
    txbuf[3] = (byte)(txSize);          // LENGTH
    txbuf[4] = PMX::SendCmd::SystemREAD;      //Comannd
    txbuf[5] = (byte)(0x00);            // OPTION

    PmxCrc16::setCrc16(txbuf);

    bool rxFlag= this->synchronize(txbuf, txSize , rxbuf, rxSize);

    this->logOutputPrint(txbuf,txSize);

    if(rxFlag == false)
    {
        //Serial.println("timeout");
        return PMX::ComError::TimeOut;
    }

    this->logOutputPrint(rxbuf,rxSize);

    unsigned short errorFlag = this->checkRecv(rxbuf, PMX::SendCmd::SystemREAD);


    if(errorFlag != PMX::ComError::OK)  //ここの部分はステータスを含まないのでOK
    {
        return errorFlag;
    }

    unsigned short status = rxbuf[PMX::BuffPter::Status];

    
    for(int i = 0; i < readDataSize ; i++)
    {
        rxData[i] = rxbuf[PMX::BuffPter::Data + i];
    }

    return status;
}


//
//SystemREADを利用したデータのget関数一覧
//


/**
 * @brief 指定したIDのサーボからシリアル番号を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] serialLongNum シリアル番号。通信が失敗した場合はNoneが返ります。
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getSerialNumber(byte id, unsigned long *serialLongNum)
{
    byte serialNumByte[4];
    //DataConv::uint32ToBytes(PMX::ErrorUint32Data,serialNumByte);//取得する配列は初期化しておく
    unsigned short status = this->getSerialNumber(id, serialNumByte);
    *serialLongNum = DataConv::bytesToUint32(serialNumByte);

    return status;
}


/**
 * @brief 指定したIDのサーボからシリアル番号を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] serialByteNum 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getSerialNumber(byte id, byte serialByteNum[4])
{
    byte sysData[13];

    unsigned short status = this->SystemREAD(id, sysData);

    //通信エラー等々はエラーとして扱う（ステータスエラー時はそのまま
    if((status & PMX::ComError::ErrorMask) != PMX::ComError::OK)
    {
        //エラーデータをbytesに変換し、代入
        DataConv::uint32ToBytes(PMX::ErrorUint32Data, serialByteNum);
        // byte errorBytesData[4]; 
        // DataConv::uint32ToBytes(PMX::ErrorUint32Data, errorBytesData);
        // for(int i = 0;i < 4;i++)    
        // {
        //     serialByteNum[i] = errorBytesData[i];
        // }
        // *serialNum = PMX::ErrorByteData;
    }
    else
    {
        for(int i = 0;i<4;i++)
        {
            serialByteNum[i] = sysData[i];
        }
    }

    return status;
}

/**
 * @brief 指定したIDのサーボからモデル番号を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] modelFullNum 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getModelNum(byte id, unsigned long *modelFullNum)
{
    byte sysData[13];

    unsigned short status = this->SystemREAD(id, sysData);

    //通信エラー等々はエラーとして扱う（ステータスエラー時はそのまま
    if((status & PMX::ComError::ErrorMask) != PMX::ComError::OK)
    {
        *modelFullNum = PMX::ErrorUint32Data;
    }
    else
    {
        for(int i = 0;i<4;i++)
        {
            modelFullNum[i] = sysData[i+4];
        }
    }

    return status;
}


/**
 * @brief 指定したIDのサーボからモデル番号を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] modelNum 
 * @param [out] seriesNum 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getModelNum(byte id, unsigned short *modelNum, unsigned short *seriesNum)
{
    byte sysData[13];

    unsigned short status = this->SystemREAD(id, sysData);

    //通信エラー等々はエラーとして扱う（ステータスエラー時はそのまま
    if((status & PMX::ComError::ErrorMask) != PMX::ComError::OK)
    {
        
        *modelNum = PMX::ErrorUint16Data;
        *seriesNum = PMX::ErrorUint16Data;
    }
    else
    {
        byte modelData[2];
        modelData[0] = sysData[4]; 
        modelData[1] = sysData[5]; 

        *modelNum = DataConv::bytesToUint16(modelData);

        byte seriesData[2];
        seriesData[0] = sysData[6]; 
        seriesData[1] = sysData[7]; 

        *seriesNum = DataConv::bytesToUint16(seriesData);

    }


    return status;
}

/**
 * @brief 指定したIDからファームウェアのバージョン番号を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] verData バージョン番号のタプル[MAJOR,MINOR,PATCH,BUILD]。通信が失敗した場合はNoneが返ります
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getVersion(byte id,byte verData[4])
{
    byte sysData[13];

    unsigned short status = this->SystemREAD(id, sysData);

    //通信エラー等々はエラーとして扱う（ステータスエラー時はそのまま
    if((status & PMX::ComError::ErrorMask) != PMX::ComError::OK)
    {
        //*verData = PMX::ErrorByteData;
        for(int i = 0;i<4;i++)
        {
            verData[i] = PMX::ErrorByteData;
        }
    }
    else
    {
        for(int i = 0;i<4;i++)
        {
            verData[i] = sysData[i+8];
        }
    }

    return status;
}

/**
 * @brief システム読み取り操作のステータスと応答時間を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] respTime 応答時間[us]。通信が失敗した場合はNoneが返ります。
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getResponseTime(byte id,byte *respTime)
{
    byte sysData[13];
    unsigned short status = this->SystemREAD(id, sysData);

//通信エラー等々はエラーとして扱う（ステータスエラー時はそのまま
    if((status & PMX::ComError::ErrorMask) != PMX::ComError::OK)
    {
        *respTime = PMX::ErrorByteData;
    }
    else
    {
        *respTime = sysData[12];
    }

    return status;
}




/**
 * @brief SystemWRITEコマンドを使用してID、ボーレート、パリティ、および応答時間を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] serialNum サーボの固有番号。SystemReadでデータを返します
 * @param [in] option 変更するデータを指定(Bit0: ID番号の変更,Bit1: 通信速度の変更,Bit2: パリティの変更,Bit3: 応答時間の変更)
 * @param [in] newId サーボに割り当てる新しい ID。 0 ～ 239 の整数値である必要があります。
 * @param [in] newBaudrateVal 新しい通信速度設定値。直接書き込む値なので、EditBaudrateの値を使用します
 * @param [in] newParityVal パリティを変更します。直接値を書き込むので、EditParityの値を使用します
 * @param [in] newResponseTime コマンドを受け取ってから返信するまでの時間を設定します(us単位で設定します)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @note パラメータは個別に書き変えることができます。
 * @note パラメータを渡さないもしくはNoneにすると変更されません
 * 
 * @attention SystemWrite使用時にはシリアル番号がNoneの場合は関数内で読み込みます。
 */
unsigned short PmxBase::SystemWRITE(byte id, byte serialNum[4], byte option, byte newId, byte newBaudrateVal, byte newParityVal, byte newResponseTime)
{
    
    int txSize = PMX::MinimumLength::Send + 4 + 4 ;  //SystemWRITEは書き込むデータは4byte,SerialNumが4byte
    int rxSize = PMX::MinimumLength::Receive;     //SystemWRITEは固定
    
    byte txbuf[txSize];
    byte rxbuf[rxSize];

    txbuf[0] = (byte)(0xFE);            // HEADER
    txbuf[1] = (byte)(0xFE);            // HEADER
    txbuf[2] = (byte)(id);              // ID
    txbuf[3] = (byte)(txSize);          // LENGTH
    txbuf[4] = PMX::SendCmd::SystemWRITE;      //Comannd
    txbuf[5] = (byte)(option);            // OPTION

    txbuf[6] = (byte)(serialNum[0]);
    txbuf[7] = (byte)(serialNum[1]);
    txbuf[8] = (byte)(serialNum[2]);
    txbuf[9] = (byte)(serialNum[3]);
    txbuf[10] = (byte)(newId);
    txbuf[11] = (byte)(newBaudrateVal);
    txbuf[12] = (byte)(newParityVal);
    txbuf[13] = (byte)(newResponseTime);

    PmxCrc16::setCrc16(txbuf);

    bool rxFlag= this->synchronize(txbuf, txSize , rxbuf, rxSize);

    this->logOutputPrint(txbuf,txSize);

    if(rxFlag == false)
    {
        //Serial.println("timeout");
        return PMX::ComError::TimeOut;
    }

    this->logOutputPrint(rxbuf,rxSize);

    unsigned short errorFlag = this->checkRecv(rxbuf, PMX::SendCmd::SystemWRITE);


    if(errorFlag != PMX::ComError::OK)  //ここの部分はステータスを含まないのでOK
    {
        return errorFlag;
    }

    unsigned short status = rxbuf[PMX::BuffPter::Status];

    return status;
}


/**
 * @brief SystemWRITEコマンドを使用してID、ボーレート、パリティ、および応答時間を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] serialNum サーボの固有番号。SystemReadでデータを返します
 * @param [in] option 変更するデータを指定(Bit0: ID番号の変更,Bit1: 通信速度の変更,Bit2: パリティの変更,Bit3: 応答時間の変更)
 * @param [in] newId サーボに割り当てる新しい ID。 0 ～ 239 の整数値である必要があります。
 * @param [in] newBaudrateVal 新しい通信速度設定値。直接書き込む値なので、EditBaudrateの値を使用します
 * @param [in] newParityVal パリティを変更します。直接値を書き込むので、EditParityの値を使用します
 * @param [in] newResponseTime コマンドを受け取ってから返信するまでの時間を設定します(us単位で設定します)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @note パラメータは個別に書き変えることができます。
 * @note パラメータを渡さないもしくはNoneにすると変更されません
 * 
 * @attention SystemWrite使用時にはシリアル番号がNoneの場合は関数内で読み込みます。
 */
unsigned short PmxBase::SystemWRITE(byte id, unsigned long serialNum, byte option, byte newId, byte newBaudrateVal, byte newParityVal, byte newResponseTime)
{
    //systemNumでLongで取得した多場合はbyte配列にする
    byte serialByteNum[4];
    DataConv::uint32ToBytes(serialNum,serialByteNum);
    //SystemWriteでデータを書き込む
    return this->SystemWRITE(id,serialByteNum,option,newId,newBaudrateVal,newParityVal,newResponseTime);
}

/**
 * @brief SystemWRITEコマンドを使用してID、ボーレート、パリティ、および応答時間を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] option 変更するデータを指定(Bit0: ID番号の変更,Bit1: 通信速度の変更,Bit2: パリティの変更,Bit3: 応答時間の変更)
 * @param [in] newId サーボに割り当てる新しい ID。 0 ～ 239 の整数値である必要があります。
 * @param [in] newBaudrateVal 新しい通信速度設定値。直接書き込む値なので、EditBaudrateの値を使用します
 * @param [in] newParityVal パリティを変更します。直接値を書き込むので、EditParityの値を使用します
 * @param [in] newResponseTime コマンドを受け取ってから返信するまでの時間を設定します(us単位で設定します)
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @note パラメータは個別に書き変えることができます。
 * @note パラメータを渡さないもしくはNoneにすると変更されません
 * 
 * @attention SystemWrite使用時にはシリアル番号がNoneの場合は関数内で読み込みます。
 */
unsigned short PmxBase::SystemWRITE(byte id, byte option, byte newId, byte newBaudrateVal, byte newParityVal, byte newResponseTime)
{
    byte serialByteNum[4];
    unsigned short serialNumGetSt = this->getSerialNumber(id,serialByteNum);
        
    //通信エラー等々はエラーとして扱う（ステータスエラー時はそのまま
    if((serialNumGetSt & PMX::ComError::ErrorMask) != PMX::ComError::OK)
    {
        return serialNumGetSt;
    }

    //SystemWriteでデータを書き込む
    return this->SystemWRITE(id,serialByteNum,option,newId,newBaudrateVal,newParityVal,newResponseTime);

}        

//
//SystemWriteを利用したデータのset関数一覧
//


/**
 * @brief 指定したIDを新しい ID に設定します。
 * 
 * @param [in] id PMXサーボモータの現在のID番号
 * @param [in] newId PMXサーボモータのWrite新しいID番号
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention シリアル番号がNoneの場合はSystemWRITE関数内で読み込みます。
 */
unsigned short PmxBase::setId(byte id, byte newId)
{
    if((newId < 0) || (newId > 239))
    {
        return PMX::ComError::FormatError;
    }
    return this->SystemWRITE(id, 0x01, newId, 0, 0, 0);
}


/**
 * @brief 指定したIDの通信速度を設定します
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] newBaudRate 通信速度
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention シリアル番号がNoneの場合はSystemWRITE関数内で読み込みます。
 * @attention 通信速度は関数内でPMXの通信速度値に変換するので通信速度は直接渡してください。
 */
unsigned short PmxBase::setBaudrate(byte id, byte newBaudRate)
{
    if((newBaudRate < 0) || (newBaudRate > 0x07))
    {
        return PMX::ComError::FormatError;
    }
    return this->SystemWRITE(id, 0x02, 0, newBaudRate, 0, 0);
}

/**
 * @brief 指定したIDのパリティを設定します
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] newParityNum 通信で使用するパリティの設定(PMX::EditParityを参照)   
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention シリアル番号がNoneの場合はSystemWRITE関数内で読み込みます。
 */
unsigned short PmxBase::setParity(byte id, byte newParityNum)
{
    if((newParityNum < 0) || (newParityNum > 0x02))
    {
        return PMX::ComError::FormatError;
    }
    return this->SystemWRITE(id, 0x04, 0, 0, newParityNum, 0);
}

/**
 * @brief 指定したIDの応答時間を設定します
 * @details  サーボ自身がデータを受信してから送信するまでのタイミング(ResponseTime)を変更します
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] respTime 応答時間[us]
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention シリアル番号がNoneの場合はSystemWRITE関数内で読み込みます。
 */
unsigned short PmxBase::setResponseTime(byte id, byte respTime)
{
    if(respTime < 1)
    {
        return PMX::ComError::FormatError;
    }
    return this->SystemWRITE(id, 0x08, 0, 0, 0, respTime);
}



/**
 * @brief サーボモータを再起動させます。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] resetTime 再起動まで待つ時間(ms)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention ReBoot中はWrite等を行ってもエラーが返ってきます。
 * @attention ReBoot直後は内部パラメータを読み込むまでデータは返ってきません
 */
unsigned short PmxBase::ReBoot(byte id, int resetTime)
{

    int txSize = 10;    //memREADはLength固定
    int rxSize = 8;
    
    byte txbuf[txSize];
    byte rxbuf[rxSize];

    txbuf[0] = (byte)(0xFE);            // HEADER
    txbuf[1] = (byte)(0xFE);            // HEADER
    txbuf[2] = (byte)(id);              // ID
    txbuf[3] = (byte)(txSize);          // LENGTH
    txbuf[4] = PMX::SendCmd::ReBoot;  //Comannd
    txbuf[5] = (byte)(0x00);        // OPTION
    txbuf[6] = (byte)(resetTime & 0x00ff);           //先頭アドレス（下位）
    txbuf[7] = (byte)((resetTime & 0xff00) >> 8);    //先頭アドレス（上位）

    PmxCrc16::setCrc16(txbuf);

    bool rxFlag= this->synchronize(txbuf, txSize , rxbuf, rxSize);

    this->logOutputPrint(txbuf,txSize);

    if(rxFlag == false)
    {
        //Serial.println("timeout");
        return PMX::ComError::TimeOut;
    }

    this->logOutputPrint(rxbuf,rxSize);

    unsigned short errorFlag = this->checkRecv(rxbuf, PMX::SendCmd::ReBoot);


    if(errorFlag != PMX::ComError::OK)  //ここの部分はステータスを含まないのでOK
    {
        return errorFlag;
    }

    unsigned short status = rxbuf[PMX::BuffPter::Status];

    return status;
}


/**
 * @brief サーボを工場出荷時の設定にリセットします
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] serialNum シリアル番号
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @note serialNumがNone場合はこの関数内で取得するので省略可能
 */
unsigned short PmxBase::FactoryReset(byte id, byte serialNum[])
{

    int txSize = 12;    //memREADはLength固定
    int rxSize = 8;
    
    byte txbuf[txSize];
    byte rxbuf[rxSize];

    txbuf[0] = (byte)(0xFE);            // HEADER
    txbuf[1] = (byte)(0xFE);            // HEADER
    txbuf[2] = (byte)(id);              // ID
    txbuf[3] = (byte)(txSize);          // LENGTH
    txbuf[4] = PMX::SendCmd::FactoryReset;      //Comannd
    txbuf[5] = (byte)(0x00);            // OPTION

    txbuf[6] = (byte)(serialNum[0]);
    txbuf[7] = (byte)(serialNum[1]);
    txbuf[8] = (byte)(serialNum[2]);
    txbuf[9] = (byte)(serialNum[3]);

    PmxCrc16::setCrc16(txbuf);

    bool rxFlag= this->synchronize(txbuf, txSize , rxbuf, rxSize);

    this->logOutputPrint(txbuf,txSize);

    if(rxFlag == false)
    {
        //Serial.println("timeout");
        return PMX::ComError::TimeOut;
    }

    this->logOutputPrint(rxbuf,rxSize);

    unsigned short errorFlag = this->checkRecv(rxbuf, PMX::SendCmd::FactoryReset);


    if(errorFlag != PMX::ComError::OK)  //ここの部分はステータスを含まないのでOK
    {
        return errorFlag;
    }

    unsigned short status = rxbuf[PMX::BuffPter::Status];

    return status;
}


//
//MemREADを利用したデータのget関数一覧
//

/**
 * @brief 指定した ID の位置制御のPゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 位置制御のPゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPositionKpGain(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::PositionKp, data);
}

/**
 * @brief 指定した ID の位置制御のIゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 位置制御のIゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPositionKiGain(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::PositionKi, data);
}

/**
 * @brief 指定した ID の位置制御のDゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 位置制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPositionKdGain(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::PositionKd, data);
}


/**
 * @brief 指定した ID の位置制御のPIDゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] kpdata 位置制御のPゲイン
 * @param [out] kidata 位置制御のIゲイン
 * @param [out] kddata 位置制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPositionGain(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata)
{
    byte rxData[12];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::PositionKp, 12, rxData);
    byte pidData[4];
    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i];
    }
    *kpdata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+4];
    }
    *kidata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+8];
    }
    *kddata = DataConv::bytesToInt32(pidData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *kpdata = (short)PMX::ErrorUint32Data;
    }

    return status;
}


/**
 * @brief 指定した ID の位置制御のストレッチゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 位置制御のストレッチゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPositionStretchGain(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::PositionSt, data);
}


/**
 * @brief 指定した ID の速度制御のPゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 速度制御のPゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getSpeedKpGain(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::SpeedKp, data);
}


/**
 * @brief 指定した ID の速度制御のIゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 速度制御のIゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getSpeedKiGain(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::SpeedKi, data);
}

/**
 * @brief 指定した ID の速度制御のDゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param　[out] data 速度制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getSpeedKdGain(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::SpeedKd, data);
}

/**
 * @brief 指定した ID の速度制御のPIDゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param kpdata 速度制御のPゲイン
 * @param kidata 速度制御のIゲイン
 * @param kddata 速度制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getSpeedGain(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata)
{
    byte rxData[12];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::SpeedKp, 12, rxData);
    byte pidData[4];
    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i];
    }
    *kpdata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+4];
    }
    *kidata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+8];
    }
    *kddata = DataConv::bytesToInt32(pidData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *kpdata = (short)PMX::ErrorUint32Data;
    }

    return status;
}

/**
 * @brief 指定した ID の電流制御のPゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 電流制御のPゲイン
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */

unsigned short PmxBase::getCurrentKpGain(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::CurrentKp, data);
}

/**
 * @brief 指定した ID の電流制御のIゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 電流制御のIゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCurrentKiGain(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::CurrentKi, data);
}

/**
 * @brief 指定した ID の電流制御のDゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 電流制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCurrentKdGain(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::CurrentKd, data);
}

/**
 * @brief 指定した ID の電流制御のPIDゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] kpdata 電流制御のPゲイン
 * @param [out] kidata 電流制御のIゲイン
 * @param [out] kddata 電流制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCurrentGain(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata)
{
    byte rxData[12];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::CurrentKp, 12, rxData);
    byte pidData[4];
    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i];
    }
    *kpdata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+4];
    }
    *kidata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+8];
    }
    *kddata = DataConv::bytesToInt32(pidData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *kpdata = (short)PMX::ErrorUint32Data;
    }

    return status;
}

/**
 * @brief 指定した ID のトルク制御のPゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data トルク制御のPゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTorqueKpGain(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::TorqueKp, data);
}

/**
 * @brief 指定した ID のトルク制御のIゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data トルク制御のIゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTorqueKiGain(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::TorqueKi, data);
}

/**
 * @brief 指定した ID のトルク制御のDゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data トルク制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTorqueKdGain(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::TorqueKd, data);
}

/**
 * @brief 指定した ID のトルク制御のDゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] kpdata トルク制御のPゲイン
 * @param [out] kidata トルク制御のIゲイン
 * @param [out] kddata トルク制御のDゲイン
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTorqueGain(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata)
{
    byte rxData[12];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::TorqueKp, 12, rxData);
    byte pidData[4];
    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i];
    }
    *kpdata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+4];
    }
    *kidata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+8];
    }
    *kddata = DataConv::bytesToInt32(pidData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *kpdata = (short)PMX::ErrorUint32Data;
    }

    return status;
}


/**
 * @brief 指定した ID の位置制御プリセットゲイン番号を取得します。
 * @details MemREADを使用して指定したIDから位置制御プリセットゲイン番号を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data プリセットゲイン番号
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPositionPresetNum(byte id, byte *data)
{
    return this->MemREADToByte(id, PMX::RamAddrList::PresetPosAddr, data);
}

/**
 * @brief 指定した ID の速度制御プリセットゲイン番号を取得します。
 * @details MemREADを使用して指定したIDから速度制御プリセットゲイン番号を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data プリセットゲイン番号
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getSpeedPresetNum(byte id, byte *data)
{
    return this->MemREADToByte(id, PMX::RamAddrList::PresetSpdAddr, data);
}

/**
 * @brief 指定した ID の電流制御プリセットゲイン番号を取得します。
 * @details MemREADを使用して指定したIDから電流制御プリセットゲイン番号を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data プリセットゲイン番号
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCurrentPresetNum(byte id, byte *data)
{
    return this->MemREADToByte(id, PMX::RamAddrList::PresetCurAddr, data);
}

/**
 * @brief 指定した ID のトルク制御プリセットゲイン番号を取得します。
 * @details MemREADを使用して指定したIDからトルク制御プリセットゲイン番号を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data プリセットゲイン番号
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTorquePresetNum(byte id, byte *data)
{
    return this->MemREADToByte(id, PMX::RamAddrList::PresetTrqAddr, data);
}

/**
 * @brief 指定した ID の位置制御、他すべてのPIDプリセットゲイン番号を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] kpdata 位置制御のPゲイン
 * @param [out] kidata 位置制御のIゲイン
 * @param [out] kddata 位置制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getAllPresetNum(byte id, byte *pos, byte *spd, byte *cur, byte *trq)
{
    byte rxData[4];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::PresetPosAddr, 4, rxData);
    *pos = rxData[0];   //位置制御プリセット番号
    *spd = rxData[1];   //速度制御プリセット番号
    *cur = rxData[2];   //電流制御プリセット番号
    *trq = rxData[3];   //トルク制御プリセット番号

    return status;
}

/**
 * @brief 指定した ID の位置制御のPゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 位置制御のPゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPositionKpGain2(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::PositionKp2, data);
}

/**
 * @brief 指定した ID の位置制御のIゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 位置制御のIゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPositionKiGain2(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::PositionKi2, data);
}

/**
 * @brief 指定した ID の位置制御のDゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 位置制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPositionKdGain2(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::PositionKd2, data);
}


/**
 * @brief 指定した ID の位置制御のPIDゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] kpdata 位置制御のPゲイン
 * @param [out] kidata 位置制御のIゲイン
 * @param [out] kddata 位置制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPositionGain2(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata)
{
    byte rxData[12];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::PositionKp2, 12, rxData);
    byte pidData[4];
    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i];
    }
    *kpdata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+4];
    }
    *kidata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+8];
    }
    *kddata = DataConv::bytesToInt32(pidData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *kpdata = (short)PMX::ErrorUint32Data;
    }

    return status;
}

/**
 * @brief 指定した ID の位置制御のストレッチゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 位置制御のストレッチゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPositionStretchGain2(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::PositionSt2, data);
}


/**
 * @brief 指定した ID の速度制御のPゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 速度制御のPゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getSpeedKpGain2(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::SpeedKp2, data);
}


/**
 * @brief 指定した ID の速度制御のIゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 速度制御のIゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getSpeedKiGain2(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::SpeedKi2, data);
}

/**
 * @brief 指定した ID の速度制御のDゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param　[out] data 速度制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getSpeedKdGain2(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::SpeedKd2, data);
}

/**
 * @brief 指定した ID の速度制御のPIDゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param kpdata 速度制御のPゲイン
 * @param kidata 速度制御のIゲイン
 * @param kddata 速度制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getSpeedGain2(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata)
{
    byte rxData[12];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::SpeedKp2, 12, rxData);
    byte pidData[4];
    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i];
    }
    *kpdata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+4];
    }
    *kidata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+8];
    }
    *kddata = DataConv::bytesToInt32(pidData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *kpdata = (short)PMX::ErrorUint32Data;
    }

    return status;
}

/**
 * @brief 指定した ID の電流制御のPゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 電流制御のPゲイン
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */

unsigned short PmxBase::getCurrentKpGain2(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::CurrentKp2, data);
}

/**
 * @brief 指定した ID の電流制御のIゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 電流制御のIゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCurrentKiGain2(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::CurrentKi2, data);
}

/**
 * @brief 指定した ID の電流制御のDゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 電流制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCurrentKdGain2(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::CurrentKd2, data);
}

/**
 * @brief 指定した ID の電流制御のPIDゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] kpdata 電流制御のPゲイン
 * @param [out] kidata 電流制御のIゲイン
 * @param [out] kddata 電流制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCurrentGain2(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata)
{
    byte rxData[12];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::CurrentKp2, 12, rxData);
    byte pidData[4];
    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i];
    }
    *kpdata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+4];
    }
    *kidata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+8];
    }
    *kddata = DataConv::bytesToInt32(pidData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *kpdata = (short)PMX::ErrorUint32Data;
    }

    return status;
}

/**
 * @brief 指定した ID のトルク制御のPゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data トルク制御のPゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTorqueKpGain2(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::TorqueKp2, data);
}

/**
 * @brief 指定した ID のトルク制御のIゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data トルク制御のIゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTorqueKiGain2(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::TorqueKi2, data);
}

/**
 * @brief 指定した ID のトルク制御のDゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data トルク制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTorqueKdGain2(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::TorqueKd2, data);
}

/**
 * @brief 指定した ID のトルク制御のDゲイン2を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] kpdata トルク制御のPゲイン
 * @param [out] kidata トルク制御のIゲイン
 * @param [out] kddata トルク制御のDゲイン
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTorqueGain2(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata)
{
    byte rxData[12];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::TorqueKp2, 12, rxData);
    byte pidData[4];
    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i];
    }
    *kpdata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+4];
    }
    *kidata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+8];
    }
    *kddata = DataConv::bytesToInt32(pidData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *kpdata = (short)PMX::ErrorUint32Data;
    }

    return status;
}


/**
 * @brief 指定した ID の位置制御のPゲイン3を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 位置制御のPゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPositionKpGain3(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::PositionKp3, data);
}

/**
 * @brief 指定した ID の位置制御のIゲイン3を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 位置制御のIゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPositionKiGain3(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::PositionKi3, data);
}

/**
 * @brief 指定した ID の位置制御のDゲイン3を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 位置制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPositionKdGain3(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::PositionKd3, data);
}


/**
 * @brief 指定した ID の位置制御のPIDゲイン3を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] kpdata 位置制御のPゲイン
 * @param [out] kidata 位置制御のIゲイン
 * @param [out] kddata 位置制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPositionGain3(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata)
{
    byte rxData[12];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::PositionKp3, 12, rxData);
    byte pidData[4];
    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i];
    }
    *kpdata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+4];
    }
    *kidata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+8];
    }
    *kddata = DataConv::bytesToInt32(pidData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *kpdata = (short)PMX::ErrorUint32Data;
    }

    return status;
}


/**
 * @brief 指定した ID の位置制御のストレッチゲイン3を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 位置制御のストレッチゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPositionStretchGain3(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::PositionSt3, data);
}


/**
 * @brief 指定した ID の速度制御のPゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 速度制御のPゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getSpeedKpGain3(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::SpeedKp3, data);
}


/**
 * @brief 指定した ID の速度制御のIゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 速度制御のIゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getSpeedKiGain3(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::SpeedKi3, data);
}

/**
 * @brief 指定した ID の速度制御のDゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param　[out] data 速度制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getSpeedKdGain3(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::SpeedKd3, data);
}

/**
 * @brief 指定した ID の速度制御のPIDゲインを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param kpdata 速度制御のPゲイン
 * @param kidata 速度制御のIゲイン
 * @param kddata 速度制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getSpeedGain3(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata)
{
    byte rxData[12];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::SpeedKp3, 12, rxData);
    byte pidData[4];
    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i];
    }
    *kpdata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+4];
    }
    *kidata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+8];
    }
    *kddata = DataConv::bytesToInt32(pidData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *kpdata = (short)PMX::ErrorUint32Data;
    }

    return status;
}

/**
 * @brief 指定した ID の電流制御のPゲイン3を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 電流制御のPゲイン
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */

unsigned short PmxBase::getCurrentKpGain3(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::CurrentKp3, data);
}

/**
 * @brief 指定した ID の電流制御のIゲイン3を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 電流制御のIゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCurrentKiGain3(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::CurrentKi3, data);
}

/**
 * @brief 指定した ID の電流制御のDゲイン3を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data 電流制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCurrentKdGain3(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::CurrentKd3, data);
}

/**
 * @brief 指定した ID の電流制御のPIDゲイン3を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] kpdata 電流制御のPゲイン
 * @param [out] kidata 電流制御のIゲイン
 * @param [out] kddata 電流制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCurrentGain3(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata)
{
    byte rxData[12];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::CurrentKp3, 12, rxData);
    byte pidData[4];
    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i];
    }
    *kpdata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+4];
    }
    *kidata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+8];
    }
    *kddata = DataConv::bytesToInt32(pidData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *kpdata = (short)PMX::ErrorUint32Data;
    }

    return status;
}

/**
 * @brief 指定した ID のトルク制御のPゲイン3を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data トルク制御のPゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTorqueKpGain3(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::TorqueKp3, data);
}

/**
 * @brief 指定した ID のトルク制御のIゲイン3を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data トルク制御のIゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTorqueKiGain3(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::TorqueKi3, data);
}

/**
 * @brief 指定した ID のトルク制御のDゲイン3を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] data トルク制御のDゲイン
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTorqueKdGain3(byte id, unsigned long *data)
{
    return this->MemREADToUint32(id, PMX::RamAddrList::TorqueKd3, data);
}

/**
 * @brief 指定した ID のトルク制御のDゲイン3を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] kpdata トルク制御のPゲイン
 * @param [out] kidata トルク制御のIゲイン
 * @param [out] kddata トルク制御のDゲイン
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTorqueGain3(byte id, unsigned long *kpdata, unsigned long *kidata, unsigned long *kddata)
{
    byte rxData[12];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::TorqueKp3, 12, rxData);
    byte pidData[4];
    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i];
    }
    *kpdata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+4];
    }
    *kidata = DataConv::bytesToInt32(pidData);

    for(int i = 0; i < 4; i++)
    {
        pidData[i] = rxData[i+8];
    }
    *kddata = DataConv::bytesToInt32(pidData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *kpdata = (short)PMX::ErrorUint32Data;
    }

    return status;
}


/**
 * @brief 指定した ID の中央値オフセット値を取得します。
 * 
 * @details MemREADを使用して指定したIDから中央値オフセット値を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] offsetData 中央値オフセット値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCenterOffset(byte id, short *offsetData)
{
    return this->MemREADToInt16(id, PMX::RamAddrList::CenterOffset, offsetData);
}

/**
 * @brief 指定した ID のクローン/リバース値を取得します。
 * @details MemREADを使用して指定したIDからクローン/リバースを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data クローン/リバース値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCloneReverse(byte id, byte *data)
{
    return this->MemREADToByte(id, PMX::RamAddrList::CloneReverse, data);
}

/**
 * @brief 指定したIDの下限電圧リミット値を取得します。
 * @details MemREADを使用して指定したIDから下限電圧リミット値を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data 下限電圧設定値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getMinVoltageLimit(byte id, unsigned short *data)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::MinVoltageLimit, data);
}

/**
 * @brief 指定したIDの下限リミット電圧を下回った場合の出力割合を取得します。
 * @details MemREADを使用して指定したIDから下限リミット電圧を下回った場合の出力割合を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data 下限リミット電圧を下回った時の出力割合(%)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getMinVoltageLimitPower(byte id, unsigned short *data)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::MinVoltageLimitPower, data);
}

/**
 * @brief 指定したIDの上限リミット電圧を取得します。
 * @details MemREADを使用して指定したIDから上限リミット電圧を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data 入力電圧最大値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getMaxVoltageLimit(byte id, unsigned short *data)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::MaxVoltageLimit, data);
}

/**
 * @brief 指定したIDの上限リミット電圧を超えた場合の出力割合を取得します。
 * @details MemREADを使用して指定したIDから上限リミット電圧を超えた場合の出力割合を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data 上限リミット電圧を上回った時の出力割合(%)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getMaxVoltageLimitPower(byte id, unsigned short *data)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::MaxVoltageLimitPower, data);
}

/**
 * @brief 指定したIDの電流リミット値を取得します。
 * @details MemREADを使用して指定したIDから電流リミット値を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data 電流リミット値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCurrentLimit(byte id, unsigned short *data)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::CurrentLimit, data);
}

/**
 * @brief 指定したIDの電流リミットを超えた時の出力割合を取得します。
 * @details MemREADを使用して指定したIDから電流リミットを超えた時の出力割合を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data 電流リミット時の出力割合(%)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCurrentLimitPower(byte id, unsigned short *data)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::CurrentLimitPower, data);
}

/**
 * @brief 指定したIDのモータ温度リミット値を取得します。
 * @details MemREADを使用して指定したIDからモータ温度リミット値を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data モータ温度リミット値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getMotorTempLimit(byte id, unsigned short *data)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::MotorTempLimit, data);
}

/**
 * @brief 指定したIDのモータ温度リミットを超えた時の出力割合を取得します。
 * @details MemREADを使用して指定したIDからモータ温度リミットを超えた時の出力割合を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data モータ温度リミット時の出力割合(%)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)。
 */
unsigned short PmxBase::getMotorTempLimitPower(byte id, unsigned short *data)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::MotorTempLimitPower, data);
}

/**
 * @brief 指定したIDのCPU温度リミット値を取得します。
 * @details MemREADを使用して指定したIDからCPU温度リミット値を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data CPU温度リミット値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCpuTempLimit(byte id, unsigned short *data)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::CpuTempLimit, data);
}

/**
 * @brief 指定したIDのCPU温度リミットを超えた時の出力割合を取得します。
 * @details MemREADを使用して指定したIDからCPU温度リミットを超えた時の出力割合を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data CPU温度リミット時の出力割合(%)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCpuTempLimitPower(byte id, unsigned short *data)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::CpuTempLimitPower, data);
}

/**
 * @brief 指定したIDのCW方向の動作角度リミット値を取得します。
 * @details MemREADを使用して指定したIDからCW方向の動作角度リミット値を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data CW方向の動作角度リミット値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCwPositionLimit(byte id, short *data)
{
    return this->MemREADToInt16(id, PMX::RamAddrList::CwPositionLimit, data);
}

/**
 * @brief 指定したIDのCW側の角度を超えた時の出力の割合を取得します。
 * @details MemREADを使用して指定したIDからCW側の角度を超えた時の出力の割合を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data CW方向の閾値外時の出力割合(%)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCwPositionLimitPower(byte id, unsigned short *data)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::CwPositionLimitPower, data);
}

/**
 * @brief 指定したIDのCCW方向の動作角度リミット値を取得します。
 * @details MemREADを使用して指定したIDからCCW方向の動作角度リミット値を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data CW方向の動作角度リミット値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCcwPositionLimit(byte id, short *data)
{
    return this->MemREADToInt16(id, PMX::RamAddrList::CcwPositionLimit, data);
}

/**
 * @brief 指定したIDのCCW側の角度を超えた時の出力の割合を取得します。
 * @details MemREADを使用して指定したIDからCCW側の角度を超えた時の出力の割合を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data CCW方向の閾値外時の出力割合(%)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCcwPositionLimitPower(byte id, unsigned short *data)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::CcwPositionLimitPower, data);
}

/**
 * @brief 指定したIDの指令可能な最大速度値を取得します。
 * @details MemREADを使用して指定したIDから指令可能な最大速度値を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data 最大速度指令値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention 最大速度指令値を超えて速度指示した場合はエラーが返り、実行はされません。
 */
unsigned short PmxBase::getMaxGoalSpeed(byte id, short *data)
{
    return this->MemREADToInt16(id, PMX::RamAddrList::MaxGoalSpeed, data);
}

/**
 * @brief 指定したIDの指令可能な最大電流値を取得します。
 * @details MemREADを使用して指定したIDから指令可能な最大電流値を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data 最大電流指令値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention 最大電流指令値を超えて電流指示した場合はエラーが返り、実行はされません。
 */
unsigned short PmxBase::getMaxGoalCurrent(byte id, short *data)
{
    return this->MemREADToInt16(id, PMX::RamAddrList::MaxGoalCurrent, data);
}

/**
 * @brief 指定したIDの指令可能な最大推定トルク値を取得します。
 * @details MemREADを使用して指定したIDから指令可能な最大推定トルク値を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data 最大動推定トルク指令値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention 最大推定トルク指令値を超えてトルク指示した場合はエラーが返り、実行はされません。
 */
unsigned short PmxBase::getMaxGoalTorque(byte id, short *data)
{
    return this->MemREADToInt16(id, PMX::RamAddrList::MaxGoalTorque, data);
}

/**
 * @brief 指定したIDのモータに出力する割合を取得します。
 * @details MemREADを使用して指定したIDからモータに出力する割合を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data モータ出力割合(%)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @note 出力のリミットではなく全体的にモータ出力を可変させます。
 */
unsigned short PmxBase::getTotalPowerRate(byte id, unsigned short *data)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::TotalPowerRate, data);
}

/**
 * @brief 指定したIDのロック検知を認識するまでの時間を取得します。
 * @details MemREADを使用して指定したIDからロック検知を認識するまでの時間を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data ロック検知を認識するまでの時間
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getLockDetectTime(byte id, unsigned short *data)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::LockDetectTime, data);
}

/**
 * @brief 指定したIDのロック検知を開始する出力指示の閾値を取得します。
 * @details MemREADを使用して指定したIDからロック検知を開始する出力指示の閾値を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data ロック検知開始出力(%)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @note 位置制御モードのみで位置制御を含まないモードの場合は使用しません。
 */
unsigned short PmxBase::getLockThresholdPower(byte id, unsigned short *data)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::LockThresholdPower, data);
}

/**
 * @brief 指定したIDがロック検知が起きた時の出力の割合を取得します。
 * @details MemREADを使用して指定したIDからロック検知が起きた時の出力の割合を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] data ロック検知時の出力割合(%)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getLockDetectOutputPower(byte id, unsigned short *data)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::LockDetectOutputPower, data);
}

/**
 * @brief 指定したIDの現在指示位置を取得します。
 * @details 指定したIDの位置制御を含む時の現在位置を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] posData 位置制御を含む時の現在位置
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPosition(byte id, short *posData)
{
    return this->MemREADToInt16(id, PMX::RamAddrList::NowPosition, posData);
}

/**
 * @brief 指定したIDの現在指示位置を取得します。
 * @details 指定したIDの位置制御を含まない時の現在位置を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] posData 位置制御を含まない現在位置
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPosition(byte id, unsigned short *posData)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::NowPosition, posData);
}


/**
 * @brief 指定したIDの現在の位置を取得します。
 * @details MemREADを使用して指定したIDから現在位置を取得します。
 * @details PMXでは、位置制御モードが含まれるとデータが符号あり(±320deg)、含まれない場合は符号なし(0-360deg)のデータが返ってきます     
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] posData 現在の位置データ。位置制御モードによって符号有り無しが変わります 。
 * @param [in] controlMode 制御モードを入れます。デフォルトは0x01です。位置制御モードが含まれるかどうかに使用します。
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPosition(byte id, long *posData, byte controlMode)
{
    unsigned short statas;
    if((controlMode & PMX::ControlMode::Position) != 0)
    {
        short bufData;
        statas = this->MemREADToInt16(id, PMX::RamAddrList::NowPosition, &bufData);
        *posData = (long)bufData;

    }
    else
    {
        unsigned short bufData;
        statas = this->MemREADToUint16(id, PMX::RamAddrList::NowPosition, &bufData);
        *posData = (long)bufData;
    }

    return statas;
}



/**
 * @brief　指定した ID の現在の速度を取得します。
 * @details　MemREADを使用して指定したIDから現在速度を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] spdData 現在の速度データ
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getSpeed(byte id, short *spdData)
{
    return this->MemREADToInt16(id, PMX::RamAddrList::NowSpeed, spdData);
}

/**
 * @brief 指定した ID の現在の電流を取得します。
 * @details　MemREADを使用して指定したIDから現在電流を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] curData 現在の電流データ
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCurrent(byte id, short *curData)
{
    return this->MemREADToInt16(id, PMX::RamAddrList::NowCurrent, curData);
}

/**
 * @brief 指定した ID の現在のトルクを取得します。
 * @details MemREADを使用して指定したIDから現在の推定トルクを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] trqData 現在の推定トルクデータ
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTorque(byte id, short *trqData)
{
    return this->MemREADToInt16(id, PMX::RamAddrList::NowTorque, trqData);
}

/**
 * @brief 指定したIDの現在のPWMを取得します。
 * @details MemREADを使用して指定したIDから現在PWMを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] pwmData 現在のPWMデータ
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getPwm(byte id, short *pwmData)
{
    return this->MemREADToInt16(id, PMX::RamAddrList::NowPwm, pwmData);
}

/**
 * @brief 指定したIDの現在のモータ温度を取得します。
 * @details MemREADを使用して指定したIDから現在モータ温度を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] motTempData 現在のモータ温度データ
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getMotorTemp(byte id, short *motTempData)
{
    return this->MemREADToInt16(id, PMX::RamAddrList::MotorTemp, motTempData);
}

/**
 * @brief 指定された ID の現在のCPU温度を取得します。
 * @details MemREADを使用して指定したIDから現在CPU温度を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] cpuTempData 現在のCPU温度データ
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCPUTemp(byte id, short *cpuTempData)
{
    return this->MemREADToInt16(id, PMX::RamAddrList::CPUTemp, cpuTempData);
}

/**
 * @brief 指定された ID の現在の電圧を取得します。
 * @details MemREADを使用して指定したIDから現在電圧を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] volData 現在の電圧データ
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getInputVoltage(byte id, unsigned short *volData)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::InputVoltage, volData);
}

/**
 * @brief 指定された ID の現在の補間時間を取得します。
 * @details MemREADを使用して指定したIDから現在補間時間を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] traTimeData 現在の補間時間データ
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTrajectoryTime(byte id, unsigned short *traTimeData)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::TrajectoryTime, traTimeData);
}

/**
 * @brief 指定された ID のエンコーダを取得します。
 * @details MemREADを使用して指定したIDからエンコーダを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] encData エンコーダデータ
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getEncoder(byte id, unsigned short *encData)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::EncoderValue, encData);
}

/**
 * @brief 指定したIDのステータスを取得します。
 * @details MemREADを使用して指定したIDからステータスを取得します。
 * @details 詳しくは取扱説明書をご覧ください。<A HREF= https://kondo-robot.com/faq/pmx-servo-series-online-manual>PMXのオンラインマニュアル</A>
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] stData ステータスデータ
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention この関数を呼ぶとサーボ内のエラー値はリセットされます。
 */
unsigned short PmxBase::getStatus(byte id, byte *stData)
{
    return this->MemREADToByte(id, PMX::RamAddrList::ErrorStatus, stData);
}

/**
 * @brief 指定された ID のシステムステータスを取得します。
 * @details MemREADを使用して指定したIDからシステムステータスを取得します。
 * @details 詳しくは取扱説明書をご覧ください。<A HREF= https://kondo-robot.com/faq/pmx-servo-series-online-manual>PMXのオンラインマニュアル</A>
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] sysStaData システムステータスデータ
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention この関数を呼ぶとサーボ内のエラー値はリセットされます。
 */
unsigned short PmxBase::getSystemStatus(byte id, byte *sysStaData)
{
    return this->MemREADToByte(id, PMX::RamAddrList::ErrorSystem, sysStaData);
}

/**
 * @brief 指定したIDのモータステータスを取得します。
 * @details MemREADを使用して指定したIDからモータステータスを取得します。
 * @details 詳しくは取扱説明書をご覧ください。<A HREF= https://kondo-robot.com/faq/pmx-servo-series-online-manual>PMXのオンラインマニュアル</A>
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] motStaData モータステータスデータ
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention この関数を呼ぶとサーボ内のエラー値はリセットされます。
 */
unsigned short PmxBase::getMotorStatus(byte id, byte *motStaData)
{
    return this->MemREADToByte(id, PMX::RamAddrList::ErrorMotor, motStaData);
}

/**
 * @brief 指定された ID のRAMアクセスステータスを取得します。
 * @details MemREADを使用して指定したIDからRAMアクセスステータスを取得します。
 * @details 詳しくは取扱説明書をご覧ください。<A HREF= https://kondo-robot.com/faq/pmx-servo-series-online-manual>PMXのオンラインマニュアル</A>
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] ramStaData アクセスエラーになったRAMの先頭アドレス
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention この関数を呼ぶとサーボ内のエラー値はリセットされます。
 */
unsigned short PmxBase::getRamAccessStatus(byte id, unsigned short *ramStaData)
{
    return this->MemREADToUint16(id, PMX::RamAddrList::ErrorRamAccess, ramStaData);
}


/**
 * @brief 指定した ID の全てのステータス情報を取得します。
 * @details MemREADを使用して指定したIDから全てのステータスを取得します。
 * @details 詳しくは取扱説明書をご覧ください。<A HREF= https://kondo-robot.com/faq/pmx-servo-series-online-manual>PMXのオンラインマニュアル</A>
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] sysSt システムステータスデータ
 * @param [out] motorSt モータステータスデータ
 * @param [out] ramSt アクセスエラーになったRAMの先頭アドレス
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention この関数を呼ぶとサーボ内のエラー値はリセットされます。
 */
unsigned short PmxBase::getFullStatus(byte id, byte *sysSt, byte *motorSt, unsigned short *ramSt)
{
    byte dummyRead[6];
    
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::ErrorStatus, 6, dummyRead); 

    *sysSt = dummyRead[1];
    *motorSt = dummyRead[2];
    *ramSt = DataConv::bytesToUint16(&(dummyRead[4]));

    return status;

}


/**
 * @brief 指定された ID の全てのステータス情報をリセットします。
 * @details MemREADを使用して指定したIDから全てのステータス情報を取得します。
 * @details 詳しくは取扱説明書をご覧ください。<A HREF= https://kondo-robot.com/faq/pmx-servo-series-online-manual>PMXのオンラインマニュアル</A>
 * 
 * @param [in] id PMXサーボモータのID番号
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::resetFullStatus(byte id)
{
    byte dammyRead[6];
    return this->MemREAD(id, PMX::RamAddrList::ErrorStatus, 6, dammyRead);
}


/**
 * @brief 指定した ID のトルク ON 値を取得します。
 * @details MemREADを使用して指定したIDからトルクON値を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] trqSwitchData FreeやTorqueOnの値(PMX::TorqueSwitchTypeを参照)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTorqueSwitch(byte id, byte *trqSwitchData)
{
    return this->MemREADToByte(id, PMX::RamAddrList::TorqueSwitch, trqSwitchData);
}

/**
 * @brief 指定した ID の制御モードを取得します。
 * @details MemREADを使用して指定したIDから制御モードを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] controlMode 制御モード値。PMX::ControlModeを参照してください。
        
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getControlMode(byte id, byte *controlMode)
{
    return this->MemREADToByte(id, PMX::RamAddrList::ControlMode, controlMode); 
}

/**
 * @brief 指定した ID の応答データ指定値を取得します。
 * @details MemREADを使用して指定したIDから応答データ指定値を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] receiveMode 応答データ指定値(PMX::ReceiveDataOptionを参照)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getMotorReceive(byte id, byte *receiveMode)
{
    return this->MemREADToByte(id, PMX::RamAddrList::MotorReceiveData, receiveMode);
}

/**
 * @brief 指定した ID の補間制御軌道生成タイプ指定を取得します。
 * @details MemREADを使用して指定したIDから補間制御軌道生成タイプ指定を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] traData 補間制御軌道生成タイプ指定。PMX::TrajectoryTypeを参照してください。
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getTrajectory(byte id, byte *traData)
{
    return this->MemREADToByte(id, PMX::RamAddrList::Trajectory, traData);
}

/**
 * @brief 指定した ID のLED点灯モードを取得します。
 * @details MemREADを使用して指定したIDからLED点灯モードを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [out] ledData LED点灯モード。PMX::LedModeTypeを参照してください。
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getLedMode(byte id, byte *ledData)
{
    return this->MemREADToByte(id, PMX::RamAddrList::LedMode, ledData);
}


/**
 * @brief 指定したIDの中央値オフセット(CenterOffset)の設定可能な最小/最大の範囲を取得します。
 * @details MemREADを使用して指定したIDから中央値オフセット(CenterOffset)の設定可能な最小/最大の範囲を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] minData CenterOffsetの最小値
 * @param [out] maxData CenterOffsetの最大値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCenterOffsetRange(byte id, short *minData, short *maxData)
{
    byte rxData[4];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::CenterOffsetMinRange, 4, rxData);

    byte rangeData[2];

    rangeData[0] = rxData[0];
    rangeData[1] = rxData[1];
    *minData = DataConv::bytesToInt16(rangeData);

    rangeData[0] = rxData[2];
    rangeData[1] = rxData[3];
    *maxData = DataConv::bytesToInt16(rangeData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *minData = (short)PMX::ErrorUint16Data;
    }

    return status;
}



/**
 * @brief 指定したIDの入力最小電圧リミット値(MinVoltageLimit)の設定可能な最小/最大の範囲を取得します。
 * @details MemREADを使用して指定したIDから入力最小電圧リミット値(MinVoltageLimit)の設定可能な最小/最大の範囲を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] minData MinVoltageLimitの最小値
 * @param [out] maxData MinVoltageLimitの最大値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getMinVoltageLimitRange(byte id, unsigned short *minData, unsigned short *maxData)
{
    byte rxData[4];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::MinVoltageMinRange, 4, rxData);

    byte rangeData[2];

    rangeData[0] = rxData[0];
    rangeData[1] = rxData[1];
    *minData = DataConv::bytesToUint16(rangeData);

    rangeData[0] = rxData[2];
    rangeData[1] = rxData[3];
    *maxData = DataConv::bytesToUint16(rangeData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *minData = (short)PMX::ErrorUint16Data;
    }

    return status;
}


/**
 * @brief 指定した ID の最大電圧(MaxVoltage)の設定可能な範囲を取得します。
 * @details MemREADを使用して指定したIDから最大電圧(MaxVoltage)の範囲を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] minData MaxVoltageの最小値
 * @param [out] maxData MaxVoltageの最大値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getMaxVoltageLimitRange(byte id, unsigned short *minData, unsigned short *maxData)
{
    byte rxData[4];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::MaxVoltageMinRange, 4, rxData);

    byte rangeData[2];

    rangeData[0] = rxData[0];
    rangeData[1] = rxData[1];
    *minData = DataConv::bytesToUint16(rangeData);

    rangeData[0] = rxData[2];
    rangeData[1] = rxData[3];
    *maxData = DataConv::bytesToUint16(rangeData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *minData = (short)PMX::ErrorUint16Data;
    }

    return status;
}

/**
 * @brief 指定した ID の電流(Current)の設定可能な範囲を取得します。
 * @details MemREADを使用して指定したIDから電流(Current)の範囲を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] minData Currentの最小値
 * @param [out] maxData Currentの最大値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCurrentLimitRange(byte id, unsigned short *minData, unsigned short *maxData)
{
    byte rxData[4];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::CurrentMinRange, 4, rxData);

    byte rangeData[2];

    rangeData[0] = rxData[0];
    rangeData[1] = rxData[1];
    *minData = DataConv::bytesToUint16(rangeData);

    rangeData[0] = rxData[2];
    rangeData[1] = rxData[3];
    *maxData = DataConv::bytesToUint16(rangeData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *minData = (short)PMX::ErrorUint16Data;
    }

    return status;
}


/**
 * @brief 指定した ID のモータ温度(MotorTemp)の範囲を取得します。
 * @details MemREADを使用して指定したIDからモータ温度(MotorTemp)の設定可能な範囲を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] minData MotorTempの最小値
 * @param [out] maxData MotorTempの最大値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getMotorTempLimitRange(byte id, short *minData, short *maxData)
{
    byte rxData[4];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::MotorTempMinRange, 4, rxData);

    byte rangeData[2];

    rangeData[0] = rxData[0];
    rangeData[1] = rxData[1];
    *minData = DataConv::bytesToInt16(rangeData);

    rangeData[0] = rxData[2];
    rangeData[1] = rxData[3];
    *maxData = DataConv::bytesToInt16(rangeData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *minData = (short)PMX::ErrorUint16Data;
    }

    return status;
}



/**
 * @brief 指定した ID のCPU温度の範囲(CpuTemp)を取得します。
 * @details MemREADを使用して指定したIDからCPU温度(CpuTemp)の設定可能な範囲を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] minData CpuTempの最小値
 * @param [out] maxData CpuTempの最大値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCpuTempLimitRange(byte id, short *minData, short *maxData)
{
    byte rxData[4];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::CpuTempMinRange, 4, rxData);

    byte rangeData[2];

    rangeData[0] = rxData[0];
    rangeData[1] = rxData[1];
    *minData = DataConv::bytesToInt16(rangeData);

    rangeData[0] = rxData[2];
    rangeData[1] = rxData[3];
    *maxData = DataConv::bytesToInt16(rangeData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *minData = (short)PMX::ErrorUint16Data;
    }

    return status;
}




/**
 * @brief 指定したIDの指定したCW方向の動作角度リミット値(CwPosition)の設定可能な範囲を取得します。
 * @details MemREADを使用して指定したIDから指定したCW方向の動作角度リミット値(CwPosition)の範囲を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] minData CwPositionの最小値
 * @param [out] maxData CwPositionの最大値
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getCwPositionLimitRange(byte id, short *minData, short *maxData)
{
    byte rxData[4];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::CwPositionMinRange, 4, rxData);

    byte rangeData[2];

    rangeData[0] = rxData[0];
    rangeData[1] = rxData[1];
    *minData = DataConv::bytesToInt16(rangeData);

    rangeData[0] = rxData[2];
    rangeData[1] = rxData[3];
    *maxData = DataConv::bytesToInt16(rangeData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *minData = (short)PMX::ErrorUint16Data;
    }

    return status;
}

/**
 * @brief 指定したIDの指定したCCW方向の動作角度リミット値(CcwPosition)の設定可能な範囲を取得します。
 * @details MemREADを使用して指定したIDから指定したCCW方向の動作角度リミット値(CcwPosition)の範囲を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] minData CcwPositionの最小値
 * @param [out] maxData CcwPositionの最大値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */

unsigned short PmxBase::getCcwPositionLimitRange(byte id, short *minData, short *maxData)
{
    byte rxData[4];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::CcwPositionMinRange, 4, rxData);

    byte rangeData[2];

    rangeData[0] = rxData[0];
    rangeData[1] = rxData[1];
    *minData = DataConv::bytesToInt16(rangeData);

    rangeData[0] = rxData[2];
    rangeData[1] = rxData[3];
    *maxData = DataConv::bytesToInt16(rangeData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *minData = (short)PMX::ErrorUint16Data;
    }

    return status;
}

/**
 * @brief 指定した ID の最大速度指令値(MaxGoalSpeed)の設定可能な範囲を取得します。
 * @details MemREADを使用して指定したIDから最大速度指令値(MaxGoalSpeed)の範囲を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] minData MaxGoalSpeedの最小値
 * @param [out] maxData MaxGoalSpeedの最大値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getMaxGoalSpeedRange(byte id, short *minData, short *maxData)
{
    byte rxData[4];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::MaxGoalSpeedMinRange, 4, rxData);

    byte rangeData[2];

    rangeData[0] = rxData[0];
    rangeData[1] = rxData[1];
    *minData = DataConv::bytesToInt16(rangeData);

    rangeData[0] = rxData[2];
    rangeData[1] = rxData[3];
    *maxData = DataConv::bytesToInt16(rangeData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *minData = (short)PMX::ErrorUint16Data;
    }

    return status;
}


/**
 * @brief 指定した ID の最大電流指令値(MaxGoalCurrent)の設定可能な範囲を取得します。
 * @details MemREADを使用して指定したIDから最大電流指令値(MaxGoalCurrent)の範囲を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] minData MaxGoalCurrentの最小値
 * @param [out] maxData MaxGoalCurrentの最大値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getMaxGoalCurrentRange(byte id, short *minData, short *maxData)
{
    byte rxData[4];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::MaxGoalCurrentMinRange, 4, rxData);

    byte rangeData[2];

    rangeData[0] = rxData[0];
    rangeData[1] = rxData[1];
    *minData = DataConv::bytesToInt16(rangeData);

    rangeData[0] = rxData[2];
    rangeData[1] = rxData[3];
    *maxData = DataConv::bytesToInt16(rangeData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *minData = (short)PMX::ErrorUint16Data;
    }

    return status;
}



/**
 * @brief 指定した ID の最大トルク指令値(MaxGoalTorque)の設定可能な範囲を取得します。
 * @details MemREADを使用して指定したIDから最大トルク指令値(MaxGoalTorque)の範囲を取得します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [out] minData MaxGoalTorqueの最小値
 * @param [out] maxData MaxGoalTorqueの最大値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::getMaxGoalTorqueRange(byte id, short *minData, short *maxData)
{
    byte rxData[4];
    unsigned short status = this->MemREAD(id, PMX::RamAddrList::MaxGoalTorqueMinRange, 4, rxData);

    byte rangeData[2];

    rangeData[0] = rxData[0];
    rangeData[1] = rxData[1];
    *minData = DataConv::bytesToInt16(rangeData);

    rangeData[0] = rxData[2];
    rangeData[1] = rxData[3];
    *maxData = DataConv::bytesToInt16(rangeData);

    //通信エラーの時は0x7FFFを入れておく
    if( (status & PMX::ComError::ErrorMask) != 0x0000 )
    {
        *minData = (short)PMX::ErrorUint16Data;
    }

    return status;
}



//
//MemWriteを利用したデータのset関数一覧
//

/**
 * @brief 指定した ID の位置制御のPゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 位置制御のPゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPositionKpGain(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::PositionKp, data, writeOpt);
}


/**
 * @brief 指定した ID の位置制御のIゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 位置制御のIゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPositionKiGain(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::PositionKi, data, writeOpt);
}


/**
 * @brief 指定した ID の位置制御のDゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 位置制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPositionKdGain(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::PositionKd, data, writeOpt);
}

/**
 * @brief 指定した ID の位置制御のPIDゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] kpdata 位置制御のPゲイン
 * @param [in] kidata 位置制御のIゲイン
 * @param [in] kddata 位置制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPositionGain(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt)
{
    byte txDataArray[12];
    byte pidData[4];
    DataConv::uint32ToBytes(kpdata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i] = pidData[i];
    }

    DataConv::uint32ToBytes(kidata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+4] = pidData[i];
    }

    DataConv::uint32ToBytes(kddata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+8] = pidData[i];
    }

    unsigned short status = this->MemWRITE(id, PMX::RamAddrList::PositionKp, txDataArray, 12, writeOpt);

    return status;

}

/**
 * @brief 指定した ID の位置制御のストレッチゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 位置制御のストレッチゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPositionStretchGain(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::PositionSt, data, writeOpt);
}

/**
 * @brief 指定した ID の速度制御のPゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 速度制御のPゲイン
 * @param [in] writeOpt　TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setSpeedKpGain(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::SpeedKp, data, writeOpt);
}


/**
 * @brief 指定した ID の速度制御のIゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 速度制御のIゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setSpeedKiGain(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::SpeedKi, data, writeOpt);
}


/**
 * @brief 指定した ID の速度制御のDゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 速度制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setSpeedKdGain(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::SpeedKd, data, writeOpt);
}


/**
 * @brief 指定した ID の速度制御のPIDゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] kpdata 速度制御のPゲイン
 * @param [in] kidata 速度制御のIゲイン
 * @param [in] kddata 速度制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setSpeedGain(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt)
{
    byte txDataArray[12];
    byte pidData[4];
    DataConv::uint32ToBytes(kpdata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i] = pidData[i];
    }

    DataConv::uint32ToBytes(kidata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+4] = pidData[i];
    }

    DataConv::uint32ToBytes(kddata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+8] = pidData[i];
    }

    unsigned short status = this->MemWRITE(id, PMX::RamAddrList::SpeedKp, txDataArray, 12, writeOpt);

    return status;
}

/**
 * @brief 指定した ID の電流制御のPゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data 電流制御のPゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setCurrentKpGain(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::CurrentKp, data, writeOpt);
}


/**
 * @brief 指定した ID の電流制御のIゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data 電流制御のIゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setCurrentKiGain(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::CurrentKi, data, writeOpt);
}


/**
 * @brief 指定した ID の電流制御のDゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data 電流制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setCurrentKdGain(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::CurrentKd, data, writeOpt);
}


/**
 * @brief 指定した ID の電流制御のPIDゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] kpdata 電流制御のPゲイン
 * @param [in] kidata 電流制御のIゲイン
 * @param [in] kddata 電流制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setCurrentGain(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt)
{
    byte txDataArray[12];
    byte pidData[4];
    DataConv::uint32ToBytes(kpdata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i] = pidData[i];
    }

    DataConv::uint32ToBytes(kidata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+4] = pidData[i];
    }

    DataConv::uint32ToBytes(kddata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+8] = pidData[i];
    }

    unsigned short status = this->MemWRITE(id, PMX::RamAddrList::CurrentKp, txDataArray, 12, writeOpt);

    return status;

}



/**
 * @brief 指定した ID のトルク制御のPゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data トルク制御のPゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setTorqueKpGain(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::TorqueKp, data, writeOpt);
}


/** 
 * @brief 指定した ID のトルク制御のIゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data トルク制御のIゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setTorqueKiGain(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::TorqueKi, data, writeOpt);
}


/** 
 * @brief 指定した ID のトルク制御のDゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data トルク制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setTorqueKdGain(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::TorqueKd, data, writeOpt);
}


/**
 * @brief 指定した ID のトルク制御のPIDゲインを変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] kpdata トルク制御のPゲイン
 * @param [in] kidata トルク制御のIゲイン
 * @param [in] kddata トルク制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setTorqueGain(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt)
{
    byte txDataArray[12];
    byte pidData[4];
    DataConv::uint32ToBytes(kpdata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i] = pidData[i];
    }

    DataConv::uint32ToBytes(kidata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+4] = pidData[i];
    }

    DataConv::uint32ToBytes(kddata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+8] = pidData[i];
    }

    unsigned short status = this->MemWRITE(id, PMX::RamAddrList::TorqueKp, txDataArray, 12, writeOpt);

    return status;

}


/**
 * @brief 指定された ID の位置制御プリセットゲイン番号を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] presetNum モータに設定する位置制御プリセットゲイン番号
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPositionPresetNum(byte id, byte presetNum,byte writeOpt)
{
    return this->MemWRITEToByte(id, PMX::RamAddrList::PresetPosAddr, presetNum, writeOpt);
}

/**
 * @brief 指定された ID の速度制御プリセットゲイン番号を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] presetNum モータに設定する速度制御プリセットゲイン番号
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setSpeedPresetNum(byte id, byte presetNum,byte writeOpt)
{
    return this->MemWRITEToByte(id, PMX::RamAddrList::PresetSpdAddr, presetNum, writeOpt);
}

/**
 * @brief 指定された ID の電流制御プリセットゲイン番号を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] presetNum モータに設定する電流制御プリセットゲイン番号
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setCurrentPresetNum(byte id, byte presetNum,byte writeOpt)
{
    return this->MemWRITEToByte(id, PMX::RamAddrList::PresetCurAddr, presetNum, writeOpt);
}

/**
 * @brief 指定された ID のトルク制御プリセットゲイン番号を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] presetNum モータに設定するトルク制御プリセットゲイン番号
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setTorquePresetNum(byte id, byte presetNum,byte writeOpt)
{
    return this->MemWRITEToByte(id, PMX::RamAddrList::PresetTrqAddr, presetNum, writeOpt);
}


/**
 * @brief 指定した ID の位置制御、他すべてのPIDプリセット番号を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] presetNum プリセットゲイン番号
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setAllPresetNum(byte id, byte presetNum, byte writeOpt)
{
    byte txDataArray[4];

    for(int i = 0; i < 4; i++){
        txDataArray[i] = presetNum;
    }

    unsigned short status = this->MemWRITE(id, PMX::RamAddrList::PresetPosAddr, txDataArray, 4, writeOpt);

    return status;
}

/**
 * @brief 指定した ID の位置制御のPゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 位置制御のPゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPositionKpGain2(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::PositionKp2, data, writeOpt);
}


/**
 * @brief 指定した ID の位置制御のIゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 位置制御のIゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPositionKiGain2(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::PositionKi2, data, writeOpt);
}


/**
 * @brief 指定した ID の位置制御のDゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 位置制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPositionKdGain2(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::PositionKd2, data, writeOpt);
}

/**
 * @brief 指定した ID の位置制御のPIDゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] kpdata 位置制御のPゲイン
 * @param [in] kidata 位置制御のIゲイン
 * @param [in] kddata 位置制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPositionGain2(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt)
{
    byte txDataArray[12];
    byte pidData[4];
    DataConv::uint32ToBytes(kpdata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i] = pidData[i];
    }

    DataConv::uint32ToBytes(kidata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+4] = pidData[i];
    }

    DataConv::uint32ToBytes(kddata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+8] = pidData[i];
    }

    unsigned short status = this->MemWRITE(id, PMX::RamAddrList::PositionKp2, txDataArray, 12, writeOpt);

    return status;

}

/**
 * @brief 指定した ID の位置制御のストレッチゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 位置制御のストレッチゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPositionStretchGain2(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::PositionSt2, data, writeOpt);
}

/**
 * @brief 指定した ID の速度制御のPゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 速度制御のPゲイン
 * @param [in] writeOpt　TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setSpeedKpGain2(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::SpeedKp2, data, writeOpt);
}


/**
 * @brief 指定した ID の速度制御のIゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 速度制御のIゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setSpeedKiGain2(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::SpeedKi2, data, writeOpt);
}


/**
 * @brief 指定した ID の速度制御のDゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 速度制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setSpeedKdGain2(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::SpeedKd2, data, writeOpt);
}


/**
 * @brief 指定した ID の速度制御のPIDゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] kpdata 速度制御のPゲイン
 * @param [in] kidata 速度制御のIゲイン
 * @param [in] kddata 速度制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setSpeedGain2(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt)
{
    byte txDataArray[12];
    byte pidData[4];
    DataConv::uint32ToBytes(kpdata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i] = pidData[i];
    }

    DataConv::uint32ToBytes(kidata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+4] = pidData[i];
    }

    DataConv::uint32ToBytes(kddata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+8] = pidData[i];
    }

    unsigned short status = this->MemWRITE(id, PMX::RamAddrList::SpeedKp2, txDataArray, 12, writeOpt);

    return status;
}

/**
 * @brief 指定した ID の電流制御のPゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data 電流制御のPゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setCurrentKpGain2(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::CurrentKp2, data, writeOpt);
}


/**
 * @brief 指定した ID の電流制御のIゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data 電流制御のIゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setCurrentKiGain2(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::CurrentKi2, data, writeOpt);
}


/**
 * @brief 指定した ID の電流制御のDゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data 電流制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setCurrentKdGain2(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::CurrentKd2, data, writeOpt);
}


/**
 * @brief 指定した ID の電流制御のPIDゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] kpdata 電流制御のPゲイン
 * @param [in] kidata 電流制御のIゲイン
 * @param [in] kddata 電流制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setCurrentGain2(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt)
{
    byte txDataArray[12];
    byte pidData[4];
    DataConv::uint32ToBytes(kpdata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i] = pidData[i];
    }

    DataConv::uint32ToBytes(kidata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+4] = pidData[i];
    }

    DataConv::uint32ToBytes(kddata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+8] = pidData[i];
    }

    unsigned short status = this->MemWRITE(id, PMX::RamAddrList::CurrentKp2, txDataArray, 12, writeOpt);

    return status;

}


/**
 * @brief 指定した ID のトルク制御のPゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data トルク制御のPゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setTorqueKpGain2(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::TorqueKp2, data, writeOpt);
}


/** 
 * @brief 指定した ID のトルク制御のIゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data トルク制御のIゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setTorqueKiGain2(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::TorqueKi2, data, writeOpt);
}


/** 
 * @brief 指定した ID のトルク制御のDゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data トルク制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setTorqueKdGain2(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::TorqueKd2, data, writeOpt);
}


/**
 * @brief 指定した ID のトルク制御のPIDゲイン2を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] kpdata トルク制御のPゲイン
 * @param [in] kidata トルク制御のIゲイン
 * @param [in] kddata トルク制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setTorqueGain2(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt)
{
    byte txDataArray[12];
    byte pidData[4];
    DataConv::uint32ToBytes(kpdata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i] = pidData[i];
    }

    DataConv::uint32ToBytes(kidata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+4] = pidData[i];
    }

    DataConv::uint32ToBytes(kddata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+8] = pidData[i];
    }

    unsigned short status = this->MemWRITE(id, PMX::RamAddrList::TorqueKp2, txDataArray, 12, writeOpt);

    return status;

}



/**
 * @brief 指定した ID の位置制御のPゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 位置制御のPゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPositionKpGain3(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::PositionKp3, data, writeOpt);
}


/**
 * @brief 指定した ID の位置制御のIゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 位置制御のIゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPositionKiGain3(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::PositionKi3, data, writeOpt);
}


/**
 * @brief 指定した ID の位置制御のDゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 位置制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPositionKdGain3(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::PositionKd3, data, writeOpt);
}

/**
 * @brief 指定した ID の位置制御のPIDゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] kpdata 位置制御のPゲイン
 * @param [in] kidata 位置制御のIゲイン
 * @param [in] kddata 位置制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPositionGain3(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt)
{
    byte txDataArray[12];
    byte pidData[4];
    DataConv::uint32ToBytes(kpdata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i] = pidData[i];
    }

    DataConv::uint32ToBytes(kidata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+4] = pidData[i];
    }

    DataConv::uint32ToBytes(kddata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+8] = pidData[i];
    }

    unsigned short status = this->MemWRITE(id, PMX::RamAddrList::PositionKp3, txDataArray, 12, writeOpt);

    return status;

}

/**
 * @brief 指定した ID の位置制御のストレッチゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 位置制御のストレッチゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPositionStretchGain3(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::PositionSt3, data, writeOpt);
}

/**
 * @brief 指定した ID の速度制御のPゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 速度制御のPゲイン
 * @param [in] writeOpt　TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setSpeedKpGain3(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::SpeedKp3, data, writeOpt);
}


/**
 * @brief 指定した ID の速度制御のIゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 速度制御のIゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setSpeedKiGain3(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::SpeedKi3, data, writeOpt);
}


/**
 * @brief 指定した ID の速度制御のDゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号 
 * @param [in] data 速度制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setSpeedKdGain3(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::SpeedKd3, data, writeOpt);
}


/**
 * @brief 指定した ID の速度制御のPIDゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] kpdata 速度制御のPゲイン
 * @param [in] kidata 速度制御のIゲイン
 * @param [in] kddata 速度制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setSpeedGain3(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt)
{
    byte txDataArray[12];
    byte pidData[4];
    DataConv::uint32ToBytes(kpdata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i] = pidData[i];
    }

    DataConv::uint32ToBytes(kidata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+4] = pidData[i];
    }

    DataConv::uint32ToBytes(kddata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+8] = pidData[i];
    }

    unsigned short status = this->MemWRITE(id, PMX::RamAddrList::SpeedKp3, txDataArray, 12, writeOpt);

    return status;
}

/**
 * @brief 指定した ID の電流制御のPゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data 電流制御のPゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setCurrentKpGain3(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::CurrentKp3, data, writeOpt);
}


/**
 * @brief 指定した ID の電流制御のIゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data 電流制御のIゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setCurrentKiGain3(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::CurrentKi3, data, writeOpt);
}


/**
 * @brief 指定した ID の電流制御のDゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data 電流制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setCurrentKdGain3(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::CurrentKd3, data, writeOpt);
}


/**
 * @brief 指定した ID の電流制御のPIDゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] kpdata 電流制御のPゲイン
 * @param [in] kidata 電流制御のIゲイン
 * @param [in] kddata 電流制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setCurrentGain3(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt)
{
    byte txDataArray[12];
    byte pidData[4];
    DataConv::uint32ToBytes(kpdata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i] = pidData[i];
    }

    DataConv::uint32ToBytes(kidata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+4] = pidData[i];
    }

    DataConv::uint32ToBytes(kddata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+8] = pidData[i];
    }

    unsigned short status = this->MemWRITE(id, PMX::RamAddrList::CurrentKp3, txDataArray, 12, writeOpt);

    return status;

}



/**
 * @brief 指定した ID のトルク制御のPゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data トルク制御のPゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setTorqueKpGain3(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::TorqueKp3, data, writeOpt);
}


/** 
 * @brief 指定した ID のトルク制御のIゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data トルク制御のIゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setTorqueKiGain3(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::TorqueKi3, data, writeOpt);
}


/** 
 * @brief 指定した ID のトルク制御のDゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data トルク制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setTorqueKdGain3(byte id, unsigned long data, byte writeOpt)
{
    return this->MemWRITEToUint32(id, PMX::RamAddrList::TorqueKd3, data, writeOpt);
}


/**
 * @brief 指定した ID のトルク制御のPIDゲイン3を変更します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] kpdata トルク制御のPゲイン
 * @param [in] kidata トルク制御のIゲイン
 * @param [in] kddata トルク制御のDゲイン
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setTorqueGain3(byte id, unsigned long kpdata, unsigned long kidata, unsigned long kddata, byte writeOpt)
{
    byte txDataArray[12];
    byte pidData[4];
    DataConv::uint32ToBytes(kpdata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i] = pidData[i];
    }

    DataConv::uint32ToBytes(kidata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+4] = pidData[i];
    }

    DataConv::uint32ToBytes(kddata, pidData);
    for(int i = 0; i < 4; i++){
        txDataArray[i+8] = pidData[i];
    }

    unsigned short status = this->MemWRITE(id, PMX::RamAddrList::TorqueKp3, txDataArray, 12, writeOpt);

    return status;

}


/** 
 * @brief 指定した ID の中央値オフセット値を設定します。
 *  
 * @param [in] id PMXサーボモータのID番号
 * @param [in] offsetData 中央値オフセット値
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention 範囲外のデータを受け取った場合statasにエラーが返ってきます。
 * @attention エラー時は実行されません。
 */
unsigned short PmxBase::setCenterOffset(byte id,short offsetData)
{
    return this->MemWRITEToInt16(id, PMX::RamAddrList::CenterOffset, offsetData);
}

/**
 * @brief 指定した ID のクローン/リバースを設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data クローン/リバースを設定する値。クローンなら1、リバースなら2を入力します。
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setCloneReverse(byte id, byte data)
{
    return this->MemWRITEToByte(id, PMX::RamAddrList::CloneReverse, data);
}

/**
 * @brief 指定した ID の下限電圧を下回った時の挙動を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] minVol 入力電圧最小値
 * @param [in] limPower 入力電圧最小時の出力％値
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention 範囲外のデータを受け取った場合statasにエラーが返ってきます。
 * @attention エラー時は実行されません。
 */
unsigned short PmxBase::setMinVoltageLimit(byte id,unsigned short minVol, unsigned short limPower ,byte writeOpt)
{
    byte txDataArray[4];
    DataConv::uint16ToBytes(minVol, txDataArray);
    DataConv::uint16ToBytes(limPower, &(txDataArray[2]));

    return this->MemWRITE(id, PMX::RamAddrList::MinVoltageLimit, txDataArray, 4, writeOpt);
}

/**
 * @brief 指定した ID の上限電圧を上回った時の挙動を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] maxVol 入力電圧最大値
 * @param [in] limPower 入力電圧最大時の出力％値
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention 範囲外のデータを受け取った場合statasにエラーが返ってきます。
 * @attention エラー時は実行されません。
 */
unsigned short PmxBase::setMaxVoltageLimit(byte id,unsigned short maxVol, unsigned short limPower ,byte writeOpt)
{
    byte txDataArray[4];
    DataConv::uint16ToBytes(maxVol, txDataArray);
    DataConv::uint16ToBytes(limPower, &(txDataArray[2]));

    return this->MemWRITE(id, PMX::RamAddrList::MaxVoltageLimit, txDataArray, 4, writeOpt);
}

/**
 * @brief 指定した ID のモータ消費電流最大値を上回った時の挙動を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] maxCur モータ消費電流最大値
 * @param [in] limPower モータ消費電流最大時の出力％値
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * @attention 範囲外のデータを受け取った場合statasにエラーが返ってきます。
 * @attention エラー時は実行されません。
 */
unsigned short PmxBase::setCurrentLimit(byte id, short maxCur, unsigned short limPower ,byte writeOpt)
{
    byte txDataArray[4];
    DataConv::int16ToBytes(maxCur, txDataArray);
    DataConv::uint16ToBytes(limPower, &(txDataArray[2]));

    return this->MemWRITE(id, PMX::RamAddrList::CurrentLimit, txDataArray, 4, writeOpt);
}

/**
 * @brief 指定した ID のモータ温度最大値を上回った時の挙動を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] motorTemp モータ温度最大値
 * @param [in] limPower モータ温度最大時の出力％値
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 *
 * @attention 範囲外のデータを受け取った場合statasにエラーが返ってきます。
 * @attention エラー時は実行されません。
 */
unsigned short PmxBase::setMotorTempLimit(byte id, short motorTemp, unsigned short limPower ,byte writeOpt)
{
    byte txDataArray[4];
    DataConv::int16ToBytes(motorTemp, txDataArray);
    DataConv::uint16ToBytes(limPower, &(txDataArray[2]));

    return this->MemWRITE(id, PMX::RamAddrList::MotorTempLimit, txDataArray, 4, writeOpt);
}

/**
 * @brief 指定した ID のCPU温度最大値を上回った時の挙動を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] cpuTemp CPU温度最大値
 * @param [in] limPower CPU温度最大時の出力％値
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention 範囲外のデータを受け取った場合statasにエラーが返ってきます。
 * @attention エラー時は実行されません。
 */
unsigned short PmxBase::setCpuTempLimit(byte id, short cpuTemp, unsigned short limPower ,byte writeOpt)
{
    byte txDataArray[4];
    DataConv::int16ToBytes(cpuTemp, txDataArray);
    DataConv::uint16ToBytes(limPower, &(txDataArray[2]));

    return this->MemWRITE(id, PMX::RamAddrList::CpuTempLimit, txDataArray, 4, writeOpt);
}

/**
 * @brief 指定した ID の最大角を上回った時の挙動を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] cwPos CW方向最大角
 * @param [in] ccwPos CCW方向最大角
 * @param [in] limPower 最大角閾値外時の出力％値
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention 範囲外のデータを受け取った場合statasにエラーが返ってきます。
 * @attention エラー時は実行されません。
 */
unsigned short PmxBase::setPositionLimit(byte id, short cwPos, short ccwPos, unsigned short limPower ,byte writeOpt)
{
    byte txDataArray[6];
    DataConv::int16ToBytes(cwPos, txDataArray);
    DataConv::int16ToBytes(ccwPos, &(txDataArray[2]));
    DataConv::uint16ToBytes(limPower, &(txDataArray[4]));

    return this->MemWRITE(id, PMX::RamAddrList::CwPositionLimit, txDataArray, 6, writeOpt);
}

/**
 * @brief 指定した ID の最大速度指令値を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] maxGoalSpd 最大速度指令値
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention 範囲外のデータを受け取った場合statasにエラーが返ってきます。
 * @attention エラー時は実行されません。
 */
unsigned short PmxBase::setMaxGoalSpeed(byte id, short maxGoalSpd, byte writeOpt)
{
    return this->MemWRITEToInt16(id, PMX::RamAddrList::MaxGoalSpeed, maxGoalSpd, writeOpt);
}

/**
 * @brief 指定した ID の最大電流指令値を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] maxGoalCur 最大電流指令値
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention 範囲外のデータを受け取った場合statasにエラーが返ってきます。
 * @attention エラー時は実行されません。
 */
unsigned short PmxBase::setMaxGoalCurrent(byte id, short maxGoalCur, byte writeOpt)
{
    return this->MemWRITEToInt16(id, PMX::RamAddrList::MaxGoalCurrent, maxGoalCur, writeOpt);
}

/**
 * @brief 指定した ID の最大動推定トルク指令値を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] maxGoalTrq 最大動推定トルク指令値
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @attention 範囲外のデータを受け取った場合statasにエラーが返ってきます。
 * @attention エラー時は実行されません。
 */
unsigned short PmxBase::setMaxGoalTorque(byte id, short maxGoalTrq, byte writeOpt)
{
    return this->MemWRITEToInt16(id, PMX::RamAddrList::MaxGoalTorque, maxGoalTrq, writeOpt);
}

/** 
 * @brief 指定した ID のモータ出力制限％値を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] rate モータ出力制限％値
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setTotalPowerRate(byte id,unsigned short rate, byte writeOpt)
{
    return this->MemWRITEToUint16(id, PMX::RamAddrList::TotalPowerRate, rate, writeOpt);
}

/**
 * @brief 指定した ID のロック時間、ロックと認識される出力割合、およびロック時間の出力％値を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] time ロック時間
 * @param [in] power ロックと認識される出力割合
 * @param [in] outputPower ロック時間の出力％値
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setLockDetect(byte id, unsigned short time, unsigned short power, unsigned short outputPower, byte writeOpt)
{
    byte txDataArray[6];
    DataConv::uint16ToBytes(time, txDataArray);
    DataConv::uint16ToBytes(power, &(txDataArray[2]));
    DataConv::uint16ToBytes(outputPower, &(txDataArray[4]));

    return this->MemWRITE(id, PMX::RamAddrList::LockDetectTime, txDataArray, 6, writeOpt);
}


/**
 * @brief 指定された ID のトルクスイッチを設定します。（TorqueON / Free / Brake / Hold）
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] data PMXサーボモータのID番号
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは1)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * @version 1.0.2～
 * 
 * 
 * @attention この関数は基本TolqueOn時も操作するためwriteOptは1になります
 * @sa PMX::TorqueSwitchType
 * @note
 *  - TorqueON : PMX::TorqueSwitchType::TorqueOn(0x01)
 *  - Free : PMX::TorqueSwitchType::Free(0x02)
 *  - Brake : PMX::TorqueSwitchType::Brake(0x04)
 *  - Hold : PMX::TorqueSwitchType::TorqueOn(0x08)
 * 
 */
unsigned short PmxBase::setTorqueSwitch(byte id, byte data, byte writeOpt)
{
    return this->MemWRITEToByte(id, PMX::RamAddrList::TorqueSwitch, data, writeOpt);
}

/**
 * @brief 指定された ID の位置制御などの制御モードを設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] controlMode モータに設定する制御モード(ControlModeを参照)
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setControlMode(byte id, byte controlMode,byte writeOpt)
{
    return this->MemWRITEToByte(id, PMX::RamAddrList::ControlMode, controlMode, writeOpt);
}

/**
 * @brief 指定したIDのmotorREADやmotorWRITEから返信される応答モードを設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] receiveOpt 応答モード ReceiveDataOptionを参照
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setMotorReceive(byte id, byte receiveMode,byte writeOpt)
{
    return this->MemWRITEToByte(id, PMX::RamAddrList::MotorReceiveData, receiveMode, writeOpt);
}

/**
 * @brief 指定した ID の補間制御軌道生成タイプ指定を設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] trajecrotyData 補間制御軌道生成タイプ指定
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setTrajectory(byte id, byte trajecrotyData, byte writeOpt)
{
    return this->MemWRITEToByte(id, PMX::RamAddrList::Trajectory, trajecrotyData, writeOpt);
}

/**
 * @brief 指定した ID のLED点灯モードを設定します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] ledModeData LED点灯モード
 * @param [in] writeOpt TolqueOn状態でも強制的に書き込むか設定します。0:強制書き込みしない、1:強制書き込みする(デフォルトは0)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setLedMode(byte id, byte ledModeData, byte writeOpt)
{
    return this->MemWRITEToByte(id, PMX::RamAddrList::LedMode, ledModeData, writeOpt);
}


//
//MotorWRITEを利用したデータのset関数一覧
//

/**
 * @brief 指定した ID のモータをトルクオンにし、ステータス、モータのON/OFF、受信データを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setMotorTorqueOn(byte id)
{
    return this->MotorWRITE(id, PMX::TorqueSwitchType::TorqueOn);
}

/**
 * @brief 指定した ID のモータをトルクオンにし、ステータス、モータのON/OFF、受信データを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] receiveMode PMXで設定した応答モード
 * @param [out] receiveData MotorReadで読み取ったデータ[位置,速度,電流,トルク,PWM,モータ温度,CPU温度,電圧](合計8項目)
 * @param [in] controlMode モータに設定する制御モード(PMX::ControlModeを参照)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setMotorTorqueOn(byte id,byte receiveMode, long receiveData[8], byte controlMode)
{
    return this->MotorWRITE(id, PMX::TorqueSwitchType::TorqueOn, receiveMode, receiveData, controlMode);
}


/**
 * @brief 指定した ID のモータをフリーにし、ステータス、モータのON/OFF、受信データを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setMotorFree(byte id)
{
    return this->MotorWRITE(id, PMX::TorqueSwitchType::Free);
}


/**
 * @brief 指定した ID のモータをフリーにし、ステータス、モータのON/OFF、受信データを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] receiveMode PMXで設定した応答モード
 * @param [out] receiveData MotorReadで読み取ったデータ[位置,速度,電流,トルク,PWM,モータ温度,CPU温度,電圧](合計8項目)
 * @param [in] controlMode モータに設定する制御モード(PMX::ControlModeを参照)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setMotorFree(byte id,byte receiveMode, long receiveData[8], byte controlMode)
{
    return this->MotorWRITE(id, PMX::TorqueSwitchType::Free, receiveMode, receiveData, controlMode);
}


/**
 * @brief 指定した ID のモータをブレーキにし、ステータス、モータのON/OFF、受信データを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setMotorBrake(byte id)
{
    return this->MotorWRITE(id, PMX::TorqueSwitchType::Brake);
}


/**
 * @brief 指定した ID のモータをブレーキにし、ステータス、モータのON/OFF、受信データを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] receiveMode PMXで設定した応答モード
 * @param [out] receiveData MotorReadで読み取ったデータ[位置,速度,電流,トルク,PWM,モータ温度,CPU温度,電圧](合計8項目)
 * @param [in] controlMode モータに設定する制御モード(PMX::ControlModeを参照)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setMotorBrake(byte id,byte receiveMode, long receiveData[8], byte controlMode)
{
    return this->MotorWRITE(id, PMX::TorqueSwitchType::Brake, receiveMode, receiveData, controlMode);
}   


/**
 * @brief 指定した ID のモータをホールドにし、ステータス、モータのON/OFF、受信データを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setMotorHold(byte id)
{
    return this->MotorWRITE(id, PMX::TorqueSwitchType::Hold);
}

/**
 * @brief 指定した ID のモータをホールドにし、ステータス、モータのON/OFF、受信データを取得します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] receiveMode aPMXで設定した応答モード
 * @param [out] receiveData MotorReadで読み取ったデータ[位置,速度,電流,トルク,PWM,モータ温度,CPU温度,電圧](合計8項目)
 * @param [in] controlMode モータに設定する制御モード(PMX::ControlModeを参照)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setMotorHold(byte id,byte receiveMode, long receiveData[8], byte controlMode)
{
    return this->MotorWRITE(id, PMX::TorqueSwitchType::Hold, receiveMode, receiveData, controlMode);
}

/**
 * @brief 指定した ID のモータの位置パラメーターを設定し、データをモーター コントローラーに送信します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] pos 位置パラメータは、モーターの望ましい位置を表す整数です。
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 */
unsigned short PmxBase::setPosition(byte id, short pos)
{
    return this->MotorWRITESingle(id, (long)pos);
}

/**
 * @brief 指定した ID のモータの位置パラメーターを設定し、データをモーター コントローラーに送信します。
 * 
 * @param [in] id PMXサーボモータのID番号
 * @param [in] pos 位置パラメータは、モーターの望ましい位置を表す整数です。
 * @param [in] receiveMode 応答モード。返ってきたデータを変換するときに使用します。
 * @param [out] receiveData MotorReadで読み取ったデータ[位置,速度,電流,トルク,PWM,モータ温度,CPU温度,電圧](合計8項目)
 * 
 * @return unsigned short 通信の状態とステータス(送信結果(PMX::ComError参照) + PMXのstatus)
 * 
 * @note MotorWRITE関数の返値を直接返します。
 * 
 * @warning controlModeやreceiveModeは事前に設定してください。
 * @warning controlModeで設定した目標値のみを引数に渡してください。
 * @warning 複数組み合わせの場合で１つだけ変えるときでも全ての目標値を送ってください。
 * 
 * @attention receiveDataでデータがない場合はリストの中身の各データがNoneになります    
 */
unsigned short PmxBase::setPosition(byte id, short pos, byte receiveMode, long receiveData[8])
{
    return this->MotorWRITESingle(id, (long)pos, receiveMode, receiveData);
}

