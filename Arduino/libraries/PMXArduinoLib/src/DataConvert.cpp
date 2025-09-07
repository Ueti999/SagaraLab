
/** 
* @file DataConvert.cpp
* @brief  Data convert library source file
* @author Kondo Kagaku Co.,Ltd.
* @author T.Nobuhara Gotanda
* @date 2024/02/22
* @version 1.0.2
* @copyright Kondo Kagaku Co.,Ltd. 2024
*
*/

#include "DataConvert.h"

/**
 * @brief Byte配列データをint16(符号あり2byte)に変換します
 * 
 * @param [in] byteDatas Byte配列データ
 * @return short 変換後のint16(符号あり2byte)
 */
short DataConv::bytesToInt16(unsigned char byteDatas[])
{
    Int16Byte sbyte;
    sbyte.byte[0] = byteDatas[0];
    sbyte.byte[1] = byteDatas[1];
    return sbyte.int16;
}

/**
 * @brief Byte配列データをUint16(符号なし2byte)に変換します
 * 
 * @param [in] byteDatas Byte配列データ
 * @return unsigned short 変換後のUint16(符号なし2byte)
 */
unsigned short DataConv::bytesToUint16(unsigned char byteDatas[])
{   
    Uint16Byte wbyte;
    wbyte.byte[0] = byteDatas[0];
    wbyte.byte[1] = byteDatas[1];
    return wbyte.uint16;
}

/**
 * @brief Byte配列データをint32(符号あり4byte)に変換します
 * 
 * @param [in] byteDatas Byte配列データ
 * @return long 変換後のint32(符号あり4byte)
 */
long DataConv::bytesToInt32(unsigned char byteDatas[])
{
    Int32Byte lbyte;
    lbyte.byte[0] = byteDatas[0];
    lbyte.byte[1] = byteDatas[1];
    lbyte.byte[2] = byteDatas[2];
    lbyte.byte[3] = byteDatas[3];
    return lbyte.int32;
}

/**
 * @brief Byte配列データをUint32(符号なし4byte)に変換します
 * 
 * @param [in] byteDatas Byte配列データ
 * @return unsigned long 変換後のUint32(符号なし4byte)
 */
unsigned long DataConv::bytesToUint32(unsigned char byteDatas[])
{
    Uint32Byte dbyte;
    dbyte.byte[0] = byteDatas[0];
    dbyte.byte[1] = byteDatas[1];
    dbyte.byte[2] = byteDatas[2];
    dbyte.byte[3] = byteDatas[3];
    return dbyte.uint32;
}

/**
 * @brief int16(符号あり2byte)のデータをByte配列に変換します
 * 
 * @param [in] shortData int16(符号あり2byte)のデータ
 * @param [out] byteDatas 変換後のByte配列データ
 */
void DataConv::int16ToBytes(short shortData, unsigned char byteDatas[])
{
    Int16Byte sbyte;
    sbyte.int16 = shortData;
    byteDatas[0] = sbyte.byte[0];
    byteDatas[1] = sbyte.byte[1];
}


/**
 * @brief Uint16(符号なし2byte)のデータをByte配列に変換します
 * 
 * @param [in] wordData Uint16(符号なし2byte)のデータ
 * @param [out] byteDatas Byte配列データ
 */
void DataConv::uint16ToBytes(unsigned short wordData, unsigned char byteDatas[])
{
    Uint16Byte wbyte;
    wbyte.uint16 = wordData;
    byteDatas[0] = wbyte.byte[0];
    byteDatas[1] = wbyte.byte[1];
}

/**
 * @brief int32(符号あり4byte)のデータをByte配列に変換します
 * 
 * @param [in] longData int32(符号あり4byte)のデータ
 * @param [out] byteDatas Byte配列データ
 */
void DataConv::int32ToBytes(long longData, unsigned char byteDatas[])
{
    Int32Byte lbyte;
    lbyte.int32 = longData;
    byteDatas[0] = lbyte.byte[0];
    byteDatas[1] = lbyte.byte[1];
    byteDatas[2] = lbyte.byte[2];
    byteDatas[3] = lbyte.byte[3];
}

/**
 * @brief Uint32(符号なし4byte)のデータをByte配列に変換します
 * 
 * @param [in] dwordData Uint32(符号なし4byte)のデータ
 * @param [out] byteDatas Byte配列データ
 */
void DataConv::uint32ToBytes(unsigned long dwordData, unsigned char byteDatas[])
{
    Uint32Byte dbyte;
    dbyte.uint32 = dwordData;
    byteDatas[0] = dbyte.byte[0];
    byteDatas[1] = dbyte.byte[1];
    byteDatas[2] = dbyte.byte[2];
    byteDatas[3] = dbyte.byte[3];
}
