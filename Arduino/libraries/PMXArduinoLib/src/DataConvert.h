
/** 
* @file DataConvert.h
* @brief  Data convert library header file
* @author Kondo Kagaku Co.,Ltd.
* @author T.Nobuhara Gotanda
* @date 2024/02/22
* @version 1.0.2
* @copyright Kondo Kagaku Co.,Ltd. 2024
*/

#ifndef __Data_Convert_h__
#define __Data_Convert_h__

class DataConv
{
        
    ///****************************
    ///  構造体定義
    ///****************************

    public:
    /**
     * @union Uint16Byte
     * @brief unsigned 2byte(short型)のデータをアクセスできるよう共用体にします
     */
    typedef union
    {
        unsigned char   byte[2];  //!< byte x2
        unsigned short  uint16;   //!< WordByte(short)
    } Uint16Byte;

    /**
     * @union Int16Byte
     * @brief signed 2byte(short型)のデータをアクセスできるよう共用体にします
     */
    typedef union 
    {
        unsigned char   byte[2];  //!< byte x2
        short   int16;   //!< WordByte(short)
    } Int16Byte;


    /**
     * @union Uint32Byte
     * @brief unsigned 4byte(long型)のデータをアクセスできるよう共用体にします
     */
    typedef union
    {
        unsigned char   byte[4];  //!< byte x4
        unsigned short  uint16[2];  //!< WordByte(short) x2
        unsigned long  uint32;    //!< DoubleWord(long)
    } Uint32Byte;

    /**
     * @union Int32Byte
     * @brief signed 4byte(long型)のデータをアクセスできるよう共用体にします
     */
    typedef union
    {
        unsigned char   byte[4];  //!< byte x4
        unsigned short  uint16[2];  //!< WordByte(short) x2
        unsigned long  int32;    //!< DoubleWord(long)
    } Int32Byte;

    public:
        static short bytesToInt16(unsigned char byteDatas[]);
        static unsigned short bytesToUint16(unsigned char byteDatas[]);
        static long bytesToInt32(unsigned char byteDatas[]);
        static unsigned long bytesToUint32(unsigned char byteDatas[]);

        static void int16ToBytes(short shortData, unsigned char byteDatas[]);
        static void uint16ToBytes(unsigned short wordData, unsigned char byteDatas[]);
        static void int32ToBytes(long longData, unsigned char byteDatas[]);
        static void uint32ToBytes(unsigned long dwordData, unsigned char byteDatas[]); 

    
}; 


#endif