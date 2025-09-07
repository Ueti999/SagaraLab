/*
 * 3DM-CV7-AHRS USB Host Reader for Teensy 4.1
 * 
 * このプログラムは以下の機能を提供します：
 * 1. Teensy 4.1のUSBホスト機能を使用
 * 2. 3DM-CV7-AHRSとUSB通信を確立
 * 3. リアルタイムでIMUデータを受信
 * 4. シリアルモニタにデータを表示
 * 5. MIPプロトコルの基本的な解析
 * 
 * 必要なライブラリ: USBHost_t36
 * 作成者: Arduino IDE + Claude
 * 対象ハードウェア: Teensy 4.1 + 3DM-CV7-AHRS
 */

#include "USBHost_t36.h"

// =============================================================================
// USBホスト関連オブジェクト
// =============================================================================
USBHost myusb;                              // メインUSBホストコントローラ
USBHub hub1(myusb);                         // USBハブサポート（必要に応じて）
USBSerial_BigBuffer userial(myusb);         // USB Serial device（3DM-CV7-AHRS用）

// =============================================================================
// グローバル変数定義
// =============================================================================
bool deviceConnected = false;               // デバイス接続状態フラグ
uint8_t buffer[1024];                       // データ受信バッファ（1KB）
int bufferIndex = 0;                        // バッファの現在位置
unsigned long lastDataTime = 0;             // 最後にデータを受信した時刻
unsigned long lastStatusTime = 0;           // 最後に状態表示した時刻

// =============================================================================
// 初期化関数
// =============================================================================
void setup() {
  // USB Serial初期化（PC通信用）
  Serial.begin(115200);
  while (!Serial) {
    delay(100);  // USB接続完了まで待機
  }
  
  // 起動メッセージ表示
  Serial.println("=== 3DM-CV7-AHRS USB Host Reader ===");
  Serial.println("Teensy 4.1 - USB Host Mode");
  Serial.println("Initializing USB Host...");
  
  // USBホストコントローラ初期化
  myusb.begin();
  
  Serial.println("Waiting for AHRS USB device...");
  Serial.println("Connect 3DM-CV7-AHRS to Teensy USB Host port");
  Serial.println();
  
  delay(1000);  // 初期化完了待機
}

// =============================================================================
// メインループ
// =============================================================================
void loop() {
  // USBホストタスク実行（重要：毎回呼び出し必須）
  myusb.Task();
  
  // USB Serial device接続状態チェック
  if (userial && !deviceConnected) {
    // 新規接続検出
    Serial.println("3DM-CV7-AHRS USB device connected!");
    
    // デバイス情報を表示
    displayDeviceInfo();
    
    // AHRSデータストリーミング開始
    startAHRSStreaming();
    
    // 接続フラグを設定
    deviceConnected = true;
    
  } else if (!userial && deviceConnected) {
    // 切断検出
    Serial.println("3DM-CV7-AHRS USB device disconnected!");
    deviceConnected = false;
    bufferIndex = 0;  // バッファリセット
  }
  
  // データ受信処理（デバイス接続時のみ）
  if (userial.available()) {
    readAHRSData();
  }
  
  // 定期的な状態表示（5秒間隔）
  if (millis() - lastStatusTime > 5000) {
    displaySystemStatus();
    
    // デバッグ情報追加
    Serial.print("USBHost Task Status: ");
    Serial.println("Running");
    Serial.print("userial object status: ");
    Serial.println(userial ? "Valid" : "Invalid");
    
    lastStatusTime = millis();
  }
  
  // データ受信タイムアウトチェック
  checkDataTimeout();
}

// =============================================================================
// デバイス情報表示関数
// =============================================================================
void displayDeviceInfo() {
  Serial.println("--- Device Information ---");
  
  // メーカー名表示
  if (userial.manufacturer()) {
    Serial.print("Manufacturer: ");
    Serial.println((const char*)userial.manufacturer());
  }
  
  // 製品名表示
  if (userial.product()) {
    Serial.print("Product: ");
    Serial.println((const char*)userial.product());
  }
  
  // シリアル番号表示
  if (userial.serialNumber()) {
    Serial.print("Serial Number: ");
    Serial.println((const char*)userial.serialNumber());
  }
  
  // USB VID/PID表示
  Serial.print("VID: 0x");
  Serial.print(userial.idVendor(), HEX);
  Serial.print(", PID: 0x");
  Serial.println(userial.idProduct(), HEX);
  
  Serial.println("-------------------------");
}

// =============================================================================
// AHRSデータストリーミング開始関数
// =============================================================================
void startAHRSStreaming() {
  Serial.println("Starting AHRS data streaming...");
  
  /*
   * MIP（Microstrain Interface Protocol）コマンド構造
   * 
   * バイト構成:
   * [0-1]: 同期バイト (0x75, 0x65)
   * [2]:   記述子セット (0x0C = IMU Data)
   * [3]:   ペイロード長
   * [4]:   コマンドID (0x08 = Message Format)
   * [5]:   機能選択 (0x01 = 新設定適用)
   * [6]:   フィールド数
   * [7-8]: データフィールド1 (0x04 = Scaled Accel, 0x0A = 10Hz)
   * [9-10]: データフィールド2 (0x05 = Scaled Gyro, 0x0A = 10Hz)
   * [11-12]: データフィールド3 (0x06 = Scaled Mag, 0x0A = 10Hz)
   * 最後: チェックサム
   */
  uint8_t cmd[] = {
    0x75, 0x65,  // MIP同期バイト
    0x0C,        // 記述子セット（IMU）
    0x08,        // ペイロード長
    0x08,        // メッセージフォーマットコマンド
    0x01,        // 機能選択（新設定適用）
    0x03,        // フィールド記述子数
    0x04, 0x0A,  // スケール済み加速度計, 10Hz
    0x05, 0x0A,  // スケール済みジャイロ, 10Hz  
    0x06, 0x0A   // スケール済み磁力計, 10Hz
  };
  
  // MIPプロトコルのチェックサム計算
  uint16_t checksum = 0;
  for (size_t i = 0; i < sizeof(cmd); i++) {
    checksum += cmd[i];
  }
  
  // コマンドとチェックサムをデバイスに送信
  userial.write(cmd, sizeof(cmd));           // メインコマンド送信
  userial.write(checksum & 0xFF);            // チェックサム下位バイト
  userial.write((checksum >> 8) & 0xFF);     // チェックサム上位バイト
  
  Serial.println("MIP streaming command sent to AHRS device.");
  Serial.println("Expecting IMU data at 10Hz...");
}

// =============================================================================
// AHRSデータ読み取り関数
// =============================================================================
void readAHRSData() {
  // 受信可能なデータがある限りループ
  while (userial.available()) {
    uint8_t byte = userial.read();  // 1バイト読み取り
    
    // 生データをPC（シリアルモニタ）に転送
    Serial.write(byte);
    
    // 内部バッファにも格納（パケット解析用）
    buffer[bufferIndex++] = byte;
    
    // バッファオーバーフロー防止
    if (bufferIndex >= (int)sizeof(buffer)) {
      bufferIndex = 0;
      Serial.println("\\n[WARNING] Buffer overflow! Resetting buffer.");
    }
    
    // データ受信時刻を更新
    lastDataTime = millis();
  }
  
  // 十分なデータが溜まったらMIPパケット解析実行
  if (bufferIndex > 4) {
    parseMIPPackets();
  }
}

// =============================================================================
// MIPパケット解析関数
// =============================================================================
void parseMIPPackets() {
  /*
   * MIPパケットの同期バイト（0x75, 0x65）を検索し、
   * 完全なパケットが受信されていれば解析する
   */
  
  for (int i = 0; i < bufferIndex - 3; i++) {
    // 同期バイト検出チェック
    if (buffer[i] == 0x75 && buffer[i+1] == 0x65) {
      uint8_t length = buffer[i+3];  // ペイロード長取得
      
      // 完全なパケットが受信済みかチェック
      // パケット構造: [同期2][記述子1][長さ1][ペイロードN][チェックサム2]
      if (i + 4 + length + 2 <= bufferIndex) {
        // 完全なパケットを発見 -> 解析実行
        parseDataPacket(&buffer[i], length + 4 + 2);
        
        // 処理済みデータをバッファから削除
        int remainingBytes = bufferIndex - (i + 4 + length + 2);
        memmove(buffer, &buffer[i + 4 + length + 2], remainingBytes);
        bufferIndex = remainingBytes;
        break;  // 1回のループで1パケットのみ処理
      }
    }
  }
}

// =============================================================================
// データパケット解析関数
// =============================================================================
void parseDataPacket(uint8_t* packet, int packetLength) {
  uint8_t descriptor = packet[2];  // 記述子セット取得
  uint8_t length = packet[3];      // ペイロード長取得
  
  // データタイプに応じた処理分岐
  switch (descriptor) {
    case 0x80: // IMUデータセット
      Serial.print("\\n[IMU] ");
      parseIMUData(&packet[4], length);
      break;
      
    case 0x81: // GNSSデータセット
      Serial.print("\\n[GNSS] ");
      parseGNSSData(&packet[4], length);
      break;
      
    case 0x82: // 推定フィルタデータセット
      Serial.print("\\n[EST] ");
      parseEstimationData(&packet[4], length);
      break;
      
    default:
      // 未知の記述子セット
      Serial.print("\\n[Unknown Descriptor: 0x");
      Serial.print(descriptor, HEX);
      Serial.print("] ");
      break;
  }
}

// =============================================================================
// IMUデータ解析関数
// =============================================================================
void parseIMUData(uint8_t* data, int length) {
  // 簡易的なIMUデータ情報表示
  Serial.print("IMU Data Length: ");
  Serial.print(length);
  Serial.print(" bytes - ");
  
  // 実際のプロジェクトでは、ここで具体的なIMUデータ
  // （加速度、角速度、磁場）を解析・表示する
  Serial.print("Accel/Gyro/Mag data received");
}

// =============================================================================
// GNSSデータ解析関数
// =============================================================================
void parseGNSSData(uint8_t* data, int length) {
  Serial.print("GNSS Data Length: ");
  Serial.print(length);  
  Serial.print(" bytes - GPS/Position data");
}

// =============================================================================
// 推定データ解析関数
// =============================================================================
void parseEstimationData(uint8_t* data, int length) {
  Serial.print("Estimation Data Length: ");
  Serial.print(length);
  Serial.print(" bytes - Attitude/Navigation data");
}

// =============================================================================
// システム状態表示関数
// =============================================================================
void displaySystemStatus() {
  Serial.println("\\n--- System Status ---");
  
  // システム稼働時間表示
  Serial.print("Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");
  
  // USBデバイス接続状態
  Serial.print("USB Device: ");
  if (userial) {
    Serial.println("Connected");
    Serial.print("Available bytes in buffer: ");
    Serial.println(userial.available());
  } else {
    Serial.println("Not connected");
  }
  
  // 内部バッファ使用状況
  Serial.print("Internal buffer usage: ");
  Serial.print(bufferIndex);
  Serial.print("/");
  Serial.println(sizeof(buffer));
  
  // 最後のデータ受信時刻
  Serial.print("Last data received: ");
  if (lastDataTime > 0) {
    Serial.print(millis() - lastDataTime);
    Serial.println(" ms ago");
  } else {
    Serial.println("Never");
  }
  
  Serial.println("---------------------\\n");
}

// =============================================================================
// データタイムアウトチェック関数
// =============================================================================
void checkDataTimeout() {
  // 10秒間データが受信されない場合の警告表示
  if (lastDataTime > 0 && (millis() - lastDataTime) > 10000) {
    Serial.println("[WARNING] Data timeout detected!");
    Serial.println("No data received for 10 seconds.");
    Serial.println("Check AHRS device connection and power.");
    lastDataTime = 0; // 重複警告を避けるためリセット
  }
}