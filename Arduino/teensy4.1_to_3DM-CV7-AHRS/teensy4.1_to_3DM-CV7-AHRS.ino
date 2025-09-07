// Teensy 4.1 USBホスト + 3DM-CV7-AHRS 通信例

#include "USBHost_t36.h"

// USBホストオブジェクト
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBSerial userial(myusb, 1); // 標準バッファサイズで十分

// デバッグモード制御
const bool DEBUG_MODE = true;  // デバッグ出力の有効/無効
const bool SKIP_CHECKSUM = true;  // チェックサム検証をスキップ（デバッグ用）

// データバッファ
uint8_t buffer[1024];  // バッファサイズを増加
int bufferIndex = 0;

// MIPプロトコル用定数
const uint8_t SYNC1 = 0x75;
const uint8_t SYNC2 = 0x65;

// 接続状態管理
bool sensorConnected = false;
bool sensorConfigured = false;
unsigned long lastActivityTime = 0;

void setup() {
  // PCとの通信用
  Serial.begin(115200);
  while (!Serial && millis() < 3000) ; // PCのシリアルモニタを待つ
  
  Serial.println("3DM-CV7-AHRS USBホスト接続 開始");
  Serial.println("センサーをUSBホストポートに接続してください...");
  
  // USBホスト初期化
  myusb.begin();
  
  delay(2000);
}

void loop() {
  // USBホストのタスク処理
  myusb.Task();
  
  // センサー接続状態の確認と管理
  if (userial) {
    if (!sensorConnected) {
      // 新規接続検出
      Serial.println("センサーが検出されました！");
      userial.begin(115200);
      sensorConnected = true;
      sensorConfigured = false;
      lastActivityTime = millis();
    }
    
    // 初期設定
    if (!sensorConfigured && sensorConnected) {
      delay(500); // センサーの初期化待ち
      configureSensor();
      sensorConfigured = true;
    }
    
    // データ読み取り
    if (sensorConfigured) {
      readSensorData();
      
      // 必要に応じてセンサーステータス確認（頻度を下げる）
      static unsigned long lastPing = 0;
      if (millis() - lastPing > 5000) { // 5秒ごと
        sendPingCommand();
        lastPing = millis();
      }
    }
    
    // タイムアウトチェック
    if (millis() - lastActivityTime > 10000) {
      Serial.println("警告: センサーからの応答がありません");
      lastActivityTime = millis();
    }
  } else {
    // センサー切断検出
    if (sensorConnected) {
      Serial.println("センサーが切断されました");
      sensorConnected = false;
      sensorConfigured = false;
      bufferIndex = 0; // バッファクリア
    }
  }
}

void configureSensor() {
  Serial.println("センサー設定中...");
  
  // IMUデータストリームを有効化するコマンド
  uint8_t enable_imu[] = {
    0x75, 0x65,  // Sync bytes
    0x0C,        // Command set (3DM Command)
    0x0A,        // Payload length
    0x08,        // Command field (Write Message Format)
    0x01,        // Apply new settings
    0x01,        // IMU data set descriptor
    0x02,        // Number of fields
    0x04, 0x00, 0x0A,  // 加速度 (0x04), Rate divisor (10Hz)
    0x05, 0x00, 0x0A,  // ジャイロ (0x05), Rate divisor (10Hz)
    0x00, 0x00   // チェックサム用プレースホルダー（2バイト）
  };
  
  // チェックサム計算
  uint8_t checksum1, checksum2;
  calculateChecksum(enable_imu, sizeof(enable_imu) - 2, &checksum1, &checksum2);
  enable_imu[sizeof(enable_imu) - 2] = checksum1;
  enable_imu[sizeof(enable_imu) - 1] = checksum2;
  
  // IMUストリーム有効化
  userial.write(enable_imu, sizeof(enable_imu));
  delay(100);
  
  // AHRSデータストリーム（フィルター済みデータ）を有効化
  uint8_t enable_ahrs[] = {
    0x75, 0x65,  // Sync bytes
    0x0C,        // Command set
    0x10,        // Payload length (増加)
    0x08,        // Command field
    0x01,        // Apply new settings
    0x03,        // AHRS/Filter data set descriptor
    0x03,        // Number of fields
    0x04, 0x00, 0x0A,  // フィルター済み加速度 (0x04), Rate divisor
    0x05, 0x00, 0x0A,  // フィルター済みジャイロ (0x05), Rate divisor
    0x0C, 0x00, 0x0A,  // オイラー角 (0x0C), Rate divisor
    0x00, 0x00   // チェックサム
  };
  
  calculateChecksum(enable_ahrs, sizeof(enable_ahrs) - 2, &checksum1, &checksum2);
  enable_ahrs[sizeof(enable_ahrs) - 2] = checksum1;
  enable_ahrs[sizeof(enable_ahrs) - 1] = checksum2;
  
  userial.write(enable_ahrs, sizeof(enable_ahrs));
  
  Serial.println("IMUおよびAHRSデータストリーム設定完了");
}

void sendPingCommand() {
  // Ping コマンド（接続確認用）
  uint8_t ping[] = {
    0x75, 0x65,  // Sync
    0x01,        // Base command set
    0x02,        // Payload length
    0x01,        // Ping command
    0x00,        // Reserved
    0xE0, 0xC6   // Checksum (固定値)
  };
  
  userial.write(ping, sizeof(ping));
  
  if (DEBUG_MODE) {
    Serial.println("Pingコマンド送信");
  }
}

void readSensorData() {
  // USBシリアルからデータ読み取り
  while (userial.available()) {
    // バッファに余裕があるか確認
    if (bufferIndex >= (int)(sizeof(buffer) - 1)) {
      // バッファが満杯に近い場合、古いデータをシフト
      shiftBuffer();
    }
    
    uint8_t byte = userial.read();
    buffer[bufferIndex++] = byte;
    lastActivityTime = millis(); // アクティビティ記録
    
    // パケット検出と処理
    processBuffer();
  }
}

void processBuffer() {
  int processedBytes = 0;
  
  // バッファ内のすべてのパケットを処理
  while (processedBytes < bufferIndex - 3) {
    // Sync bytesを探す
    bool foundPacket = false;
    
    for (int i = processedBytes; i <= bufferIndex - 4; i++) {
      if (buffer[i] == SYNC1 && buffer[i+1] == SYNC2) {
        // MIPパケット発見
        uint8_t length = buffer[i+3];
        
        // 完全なパケットがあるか確認
        int packetSize = 4 + length + 2; // header(4) + payload + checksum(2)
        if (i + packetSize <= bufferIndex) {
          // デバッグ: パケット情報表示
          if (DEBUG_MODE) {
            Serial.print("パケット検出: Desc=0x");
            Serial.print(buffer[i+2], HEX);
            Serial.print(" Len=");
            Serial.print(length);
            Serial.print(" Size=");
            Serial.println(packetSize);
          }
          
          // チェックサム検証
          bool checksumOK = verifyChecksum(&buffer[i], packetSize);
          if (SKIP_CHECKSUM || checksumOK) {
            processMIPPacket(&buffer[i], packetSize);
          } else {
            // チェックサムエラーの詳細表示
            Serial.print("チェックサムエラー: ");
            uint8_t calc1, calc2;
            calculateChecksum(&buffer[i], packetSize - 2, &calc1, &calc2);
            Serial.print("計算値=0x");
            if (calc1 < 0x10) Serial.print("0");
            Serial.print(calc1, HEX);
            if (calc2 < 0x10) Serial.print("0");
            Serial.print(calc2, HEX);
            Serial.print(" 受信値=0x");
            if (buffer[i + packetSize - 2] < 0x10) Serial.print("0");
            Serial.print(buffer[i + packetSize - 2], HEX);
            if (buffer[i + packetSize - 1] < 0x10) Serial.print("0");
            Serial.println(buffer[i + packetSize - 1], HEX);
          }
          
          processedBytes = i + packetSize;
          foundPacket = true;
          break;
        } else {
          // 不完全なパケット - 後で処理
          processedBytes = i;
          foundPacket = false;
          break;
        }
      }
    }
    
    if (!foundPacket) {
      break;
    }
  }
  
  // 処理済みデータを削除
  if (processedBytes > 0) {
    memmove(buffer, &buffer[processedBytes], bufferIndex - processedBytes);
    bufferIndex -= processedBytes;
  }
}

void shiftBuffer() {
  // バッファの前半を削除してスペースを作る
  int shiftAmount = bufferIndex / 2;
  memmove(buffer, &buffer[shiftAmount], bufferIndex - shiftAmount);
  bufferIndex -= shiftAmount;
  Serial.println("警告: バッファシフト実行");
}

bool verifyChecksum(uint8_t* packet, int length) {
  uint8_t calc_checksum1, calc_checksum2;
  calculateChecksum(packet, length - 2, &calc_checksum1, &calc_checksum2);
  
  return (packet[length - 2] == calc_checksum1 && 
          packet[length - 1] == calc_checksum2);
}

void processMIPPacket(uint8_t* packet, int length) {
  uint8_t descriptor = packet[2];
  uint8_t payload_length = packet[3];
  
  if (DEBUG_MODE) {
    Serial.print("MIPパケット受信: ");
    Serial.print("Descriptor=0x");
    Serial.print(descriptor, HEX);
    Serial.print(" Length=");
    Serial.println(payload_length);
    
    // デバッグ用：生データを表示
    Serial.print("生データ: ");
    for (int i = 0; i < length; i++) {
      if (packet[i] < 0x10) Serial.print("0");
      Serial.print(packet[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  // IMUデータセット (0x80)の場合
  if (descriptor == 0x80) {
    parseIMUData(&packet[4], payload_length);
  }
  // AHRSデータセット (0x82)の場合  
  else if (descriptor == 0x82) {
    parseAHRSData(&packet[4], payload_length);
  }
  // コマンド応答 (0x01)の場合
  else if (descriptor == 0x01) {
    if (DEBUG_MODE) {
      Serial.println("コマンド応答受信");
    }
  }
}

void parseIMUData(uint8_t* data, int length) {
  int index = 0;
  
  if (DEBUG_MODE) {
    Serial.println("=== IMUデータ解析 ===");
    Serial.print("総バイト数: ");
    Serial.println(length);
  }
  
  while (index < length - 2) {  // 最低2バイト必要
    uint8_t field_length = data[index];
    uint8_t field_descriptor = data[index + 1];
    
    // 無限ループ防止
    if (field_length == 0 || field_length > length - index) {
      if (DEBUG_MODE) {
        Serial.println("警告: 不正なfield_length");
      }
      break;
    }
    
    if (DEBUG_MODE) {
      Serial.print("  [");
      Serial.print(index);
      Serial.print("] Field: 0x");
      if (field_descriptor < 0x10) Serial.print("0");
      Serial.print(field_descriptor, HEX);
      Serial.print(" (");
      
      // フィールドの名前を表示
      switch(field_descriptor) {
        case 0x04: Serial.print("加速度"); break;
        case 0x05: Serial.print("ジャイロ"); break;
        case 0x06: Serial.print("磁力計"); break;
        case 0xD3: Serial.print("GPSタイムスタンプ"); break;
        default: Serial.print("未知"); break;
      }
      
      Serial.print(") Len=");
      Serial.println(field_length);
    }
    
    // 加速度データ (0x04)
    if (field_descriptor == 0x04 && field_length >= 14) {
      float accel_x = parseFloat(&data[index + 2]);
      float accel_y = parseFloat(&data[index + 6]);
      float accel_z = parseFloat(&data[index + 10]);
      
      Serial.print("加速度[m/s²]: X=");
      Serial.print(accel_x, 4);
      Serial.print(" Y=");
      Serial.print(accel_y, 4);
      Serial.print(" Z=");
      Serial.println(accel_z, 4);
    }
    // ジャイロデータ (0x05)
    else if (field_descriptor == 0x05 && field_length >= 14) {
      float gyro_x = parseFloat(&data[index + 2]);
      float gyro_y = parseFloat(&data[index + 6]);
      float gyro_z = parseFloat(&data[index + 10]);
      
      Serial.print("ジャイロ[rad/s]: X=");
      Serial.print(gyro_x, 4);
      Serial.print(" Y=");
      Serial.print(gyro_y, 4);
      Serial.print(" Z=");
      Serial.println(gyro_z, 4);
    }
    // 磁力計データ (0x06)
    else if (field_descriptor == 0x06 && field_length >= 14) {
      float mag_x = parseFloat(&data[index + 2]);
      float mag_y = parseFloat(&data[index + 6]);
      float mag_z = parseFloat(&data[index + 10]);
      
      Serial.print("磁力計[Gauss]: X=");
      Serial.print(mag_x, 4);
      Serial.print(" Y=");
      Serial.print(mag_y, 4);
      Serial.print(" Z=");
      Serial.println(mag_z, 4);
    }
    // GPSタイムスタンプ (0xD3) - タイムスタンプ付きIMUデータ
    else if (field_descriptor == 0xD3 && field_length >= 14) {
      // MIPプロトコルのタイムスタンプフォーマット
      // 最初の4バイト: タイムスタンプ（秒、float）
      // 次の4バイト: タイムスタンプ（マイクロ秒、uint32）
      // 最後の4バイト: ステータスフラグ
      
      float time_seconds = parseFloat(&data[index + 2]);
      
      // uint32のマイクロ秒部分（ビッグエンディアン）
      uint32_t time_microseconds = ((uint32_t)data[index + 6] << 24) | 
                                   ((uint32_t)data[index + 7] << 16) | 
                                   ((uint32_t)data[index + 8] << 8) | 
                                   data[index + 9];
      
      // ステータスフラグ（最後の4バイト）
      uint32_t status = ((uint32_t)data[index + 10] << 24) | 
                       ((uint32_t)data[index + 11] << 16) | 
                       ((uint32_t)data[index + 12] << 8) | 
                       data[index + 13];
      
      Serial.print("GPSタイムスタンプ: ");
      if (!isnan(time_seconds) && !isinf(time_seconds)) {
        Serial.print(time_seconds, 3);
        Serial.print(" 秒 + ");
        Serial.print(time_microseconds);
        Serial.print(" μs (ステータス: 0x");
        Serial.print(status, HEX);
        Serial.println(")");
      } else {
        // タイムスタンプが無効な場合、生データを表示
        Serial.print("無効なタイムスタンプ - 生データ: ");
        for (int i = 2; i < 14; i++) {
          if (data[index + i] < 0x10) Serial.print("0");
          Serial.print(data[index + i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }
    }
    
    // 次のフィールドへ
    index += field_length;
  }
}

void parseAHRSData(uint8_t* data, int length) {
  int index = 0;
  
  if (DEBUG_MODE) {
    Serial.println("=== AHRSデータ解析 ===");
    Serial.print("総バイト数: ");
    Serial.println(length);
  }
  
  while (index < length - 2) {
    uint8_t field_length = data[index];
    uint8_t field_descriptor = data[index + 1];
    
    // 無限ループ防止
    if (field_length == 0 || field_length > length - index) {
      if (DEBUG_MODE) {
        Serial.println("警告: 不正なfield_length");
      }
      break;
    }
    
    // フィルター済み加速度 (0x04) - AHRSデータセット内
    if (field_descriptor == 0x04 && field_length >= 14) {
      float accel_x = parseFloat(&data[index + 2]);
      float accel_y = parseFloat(&data[index + 6]);
      float accel_z = parseFloat(&data[index + 10]);
      
      Serial.print("フィルター済み加速度[m/s²]: X=");
      Serial.print(accel_x, 4);
      Serial.print(" Y=");
      Serial.print(accel_y, 4);
      Serial.print(" Z=");
      Serial.println(accel_z, 4);
    }
    // フィルター済みジャイロ (0x05) - AHRSデータセット内
    else if (field_descriptor == 0x05 && field_length >= 14) {
      float gyro_x = parseFloat(&data[index + 2]);
      float gyro_y = parseFloat(&data[index + 6]);
      float gyro_z = parseFloat(&data[index + 10]);
      
      Serial.print("フィルター済みジャイロ[rad/s]: X=");
      Serial.print(gyro_x, 4);
      Serial.print(" Y=");
      Serial.print(gyro_y, 4);
      Serial.print(" Z=");
      Serial.println(gyro_z, 4);
    }
    // オイラー角 (0x0C)
    else if (field_descriptor == 0x0C && field_length >= 14) {
      float roll = parseFloat(&data[index + 2]);
      float pitch = parseFloat(&data[index + 6]);
      float yaw = parseFloat(&data[index + 10]);
      
      Serial.print("姿勢角[度]: Roll=");
      Serial.print(roll * 180.0 / PI);
      Serial.print(" Pitch=");
      Serial.print(pitch * 180.0 / PI);
      Serial.print(" Yaw=");
      Serial.println(yaw * 180.0 / PI);
    }
    // クォータニオン (0x0A)
    else if (field_descriptor == 0x0A && field_length >= 18) {
      float q0 = parseFloat(&data[index + 2]);
      float q1 = parseFloat(&data[index + 6]);
      float q2 = parseFloat(&data[index + 10]);
      float q3 = parseFloat(&data[index + 14]);
      
      Serial.print("クォータニオン: q0=");
      Serial.print(q0, 4);
      Serial.print(" q1=");
      Serial.print(q1, 4);
      Serial.print(" q2=");
      Serial.print(q2, 4);
      Serial.print(" q3=");
      Serial.println(q3, 4);
    }
    else {
      // 未知のフィールドディスクリプタ
      if (DEBUG_MODE) {
        Serial.print("未知のフィールド: 0x");
        Serial.print(field_descriptor, HEX);
        Serial.print(" Length=");
        Serial.print(field_length);
        Serial.print(" Data=");
        for (int i = 2; i < field_length && i < 18; i++) {
          if (data[index + i] < 0x10) Serial.print("0");
          Serial.print(data[index + i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }
    }
    
    index += field_length;
  }
}

// ビッグエンディアンのfloatをパース（簡略化）
float parseFloat(uint8_t* bytes) {
  union {
    float f;
    uint8_t b[4];
  } u;
  
  // ビッグエンディアンからリトルエンディアンへ変換
  // Teensyは常にリトルエンディアン
  u.b[0] = bytes[3];
  u.b[1] = bytes[2];
  u.b[2] = bytes[1];
  u.b[3] = bytes[0];
  
  return u.f;
}

// MIPプロトコル用のFletcher-16チェックサム計算
void calculateChecksum(uint8_t* data, int length, uint8_t* checksum1, uint8_t* checksum2) {
  uint8_t sum1 = 0;
  uint8_t sum2 = 0;
  
  // Fletcher-16チェックサムアルゴリズム
  for (int i = 0; i < length; i++) {
    sum1 = (sum1 + data[i]) & 0xFF;
    sum2 = (sum2 + sum1) & 0xFF;
  }
  
  *checksum1 = sum1;
  *checksum2 = sum2;
}