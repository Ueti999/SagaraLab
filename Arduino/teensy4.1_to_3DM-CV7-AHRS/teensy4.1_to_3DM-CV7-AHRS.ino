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
  
  // 起動確認用LED点滅（内蔵LED使用）
  pinMode(LED_BUILTIN, OUTPUT);
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  
  while (!Serial && millis() < 3000) ; // PCのシリアルモニタを待つ
  
  Serial.println("\n===== 起動開始 =====");
  Serial.println("3DM-CV7-AHRS USBホスト接続 開始");
  Serial.print("起動時刻: ");
  Serial.print(millis());
  Serial.println("ms");
  Serial.println("センサーをUSBホストポートに接続してください...");
  
  // USBホスト初期化
  Serial.println("USBホスト初期化中...");
  myusb.begin();
  Serial.println("USBホスト初期化完了");
  
  delay(2000);
  Serial.println("セットアップ完了 - メインループ開始");
}

void loop() {
  static unsigned long lastDebugTime = 0;
  
  // USBホストのタスク処理
  myusb.Task();
  
  // 定期的な状態表示（5秒ごと）
  if (millis() - lastDebugTime > 5000) {
    Serial.print("[状態] ");
    Serial.print(millis());
    Serial.print("ms - センサー接続: ");
    Serial.print(userial ? "検出" : "未検出");
    Serial.print(" | 設定済み: ");
    Serial.println(sensorConfigured ? "Yes" : "No");
    lastDebugTime = millis();
    
    // LED点滅で動作確認
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  
  // センサー接続状態の確認と管理
  if (userial) {
    if (!sensorConnected) {
      // 新規接続検出
      Serial.println("\n🔌 センサーが検出されました！");
      Serial.print("  ボーレート設定: ");
      userial.begin(115200);
      Serial.println("115200");
      sensorConnected = true;
      sensorConfigured = false;
      lastActivityTime = millis();
    }
    
    // 初期設定（自動設定をスキップして手動制御）
    if (!sensorConfigured && sensorConnected) {
      delay(500); // センサーの初期化待ち
      Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━");
      Serial.println("📋 センサー接続確認");
      Serial.println("現在のデータストリームを確認中...");
      Serial.println("");
      Serial.println("コマンド:");
      Serial.println("  'c' - センサーを設定");
      Serial.println("  's' - 状態確認");
      Serial.println("━━━━━━━━━━━━━━━━━━━━━━━");
      sensorConfigured = true;  // 自動設定はスキップ
    }
    
    // データ読み取り
    if (sensorConfigured) {
      readSensorData();
      
      // キーボード入力処理
      if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'c' || cmd == 'C') {
          Serial.println("\n📡 センサー設定コマンド送信...");
          configureSensor();
        } else if (cmd == 's' || cmd == 'S') {
          Serial.println("\n📊 現在の状態:");
          Serial.println("- 0x80パケット = IMU生データ");
          Serial.println("- 0x82パケット = AHRSフィルターデータ");
          Serial.println("- 0x05 = ジャイロ, 0x0C = オイラー角");
        }
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
  Serial.println("\n===== センサー設定開始 =====");
  Serial.println("⚠️ 注意: センサーが既に設定されている可能性があります");
  Serial.println("SensorConnectツールでの設定を確認してください");
  
  uint8_t checksum1, checksum2;
  
  // まず現在のデータストリームを確認するために少し待つ
  Serial.println("\n現在のデータストリームを確認中...");
  delay(1000);
  
  // Step 1: Idleコマンドを送信（センサーをアイドル状態にする）
  uint8_t idle_cmd[] = {
    0x75, 0x65,  // Sync bytes
    0x01,        // Base command set
    0x02,        // Payload length
    0x02,        // Set to Idle command
    0x00, 0x00   // チェックサム
  };
  
  calculateChecksum(idle_cmd, sizeof(idle_cmd) - 2, &checksum1, &checksum2);
  idle_cmd[sizeof(idle_cmd) - 2] = checksum1;
  idle_cmd[sizeof(idle_cmd) - 1] = checksum2;
  
  Serial.println("\nStep 1: センサーをアイドル状態にする");
  userial.write(idle_cmd, sizeof(idle_cmd));
  delay(500);
  
  // Step 2: AHRSメッセージフォーマットを設定（オイラー角のみ）
  uint8_t set_ahrs_format[] = {
    0x75, 0x65,  // Sync bytes
    0x0C,        // Command set
    0x0A,        // Payload length
    0x08,        // Message Format command
    0x02,        // Descriptor set (0x02 = AHRS/Filter)
    0x01,        // Number of fields (1つ = オイラー角のみ)
    0x0C,        // Field: Euler Angles
    0x00, 0x01,  // Rate divider (every packet = 100Hz)
    0x00, 0x00   // チェックサム
  };
  
  calculateChecksum(set_ahrs_format, sizeof(set_ahrs_format) - 2, &checksum1, &checksum2);
  set_ahrs_format[sizeof(set_ahrs_format) - 2] = checksum1;
  set_ahrs_format[sizeof(set_ahrs_format) - 1] = checksum2;
  
  Serial.println("Step 2: AHRSフォーマット設定（オイラー角のみ @ 100Hz）");
  userial.write(set_ahrs_format, sizeof(set_ahrs_format));
  delay(200);
  
  // Step 3: AHRSストリームのみを有効化
  uint8_t enable_ahrs_only[] = {
    0x75, 0x65,  // Sync bytes
    0x0C,        // Command set
    0x05,        // Payload length
    0x11,        // Enable Data Stream
    0x01,        // Function selector
    0x02,        // AHRS/Filter stream (0x82)
    0x01,        // Enable
    0x00, 0x00   // チェックサム
  };
  
  calculateChecksum(enable_ahrs_only, sizeof(enable_ahrs_only) - 2, &checksum1, &checksum2);
  enable_ahrs_only[sizeof(enable_ahrs_only) - 2] = checksum1;
  enable_ahrs_only[sizeof(enable_ahrs_only) - 1] = checksum2;
  
  Serial.println("Step 3: AHRSストリーム(0x82)のみを有効化");
  userial.write(enable_ahrs_only, sizeof(enable_ahrs_only));
  delay(200);
  
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("✅ 設定送信完了！");
  Serial.println("📊 期待される出力:");
  Serial.println("  - 0x82パケットのみ（0x80は停止）");
  Serial.println("  - Filter内0x05 = オイラー角(Euler RPY) @ 100Hz");
  Serial.println("  ※ 0x82内の0x05はオイラー角です（ジャイロではありません）");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println();
}

// Pingコマンドは削除（不要）

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
  
  // AHRSデータセット (0x82) - オイラー角データ
  if (descriptor == 0x82) {
    // デバッグモードでのみ詳細表示
    if (DEBUG_MODE) {
      Serial.print("📦 0x82 (");
      Serial.print(payload_length);
      Serial.print("bytes) ");
    }
    parseAHRSData(&packet[4], payload_length);
  }
  // IMUデータ (0x80) - 無効化されているはずだが、来た場合は警告
  else if (descriptor == 0x80) {
    static unsigned long lastWarning = 0;
    if (millis() - lastWarning > 5000) {  // 5秒ごとに警告
      Serial.println("⚠️ IMUデータ(0x80)が受信されています - 無効化を確認してください");
      lastWarning = millis();
    }
  }
}

// IMUデータ解析は不要なので削除
void parseIMUData(uint8_t* data, int length) {
  // 0x80パケットは完全に無視
  (void)data;    // 未使用パラメータの警告を抑制
  (void)length;
  return;
}

void parseAHRSData(uint8_t* data, int length) {
  int idx = 0;  // indexではなくidxを使用（名前競合回避）
  static unsigned long lastPrintTime = 0;
  static int packetCount = 0;
  
  packetCount++;
  
  while (idx < length - 2) {
    uint8_t field_length = data[idx];
    uint8_t field_descriptor = data[idx + 1];
    
    if (field_length == 0 || field_length > length - idx) break;
    
    // Estimation Filter (0x82)内での0x05 = オイラー角（Attitude Euler RPY）
    // 注意: IMU(0x80)の0x05はジャイロだが、Filter(0x82)の0x05はオイラー角
    if (field_descriptor == 0x05 && field_length >= 16) {
      // データ構造: Roll(4) + Pitch(4) + Yaw(4) + Flags(4) = 16バイト
      float roll = parseFloat(&data[idx + 2]);
      float pitch = parseFloat(&data[idx + 6]);
      float yaw = parseFloat(&data[idx + 10]);
      // 最後の4バイトはステータスフラグ
      
      // 度に変換
      float roll_deg = roll * 180.0 / PI;
      float pitch_deg = pitch * 180.0 / PI;
      float yaw_deg = yaw * 180.0 / PI;
      
      // 通常の出力
      Serial.print("🎯 [");
      Serial.print(millis());
      Serial.print("ms] Roll=");
      Serial.print(roll_deg, 2);
      Serial.print("° Pitch=");
      Serial.print(pitch_deg, 2);
      Serial.print("° Yaw=");
      Serial.print(yaw_deg, 2);
      Serial.print("°");
      
      // 100Hz確認用のレート表示（1秒ごと）
      unsigned long currentTime = millis();
      if (currentTime - lastPrintTime >= 1000) {
        Serial.print(" | Rate: ");
        Serial.print(packetCount);
        Serial.print("Hz");
        packetCount = 0;
        lastPrintTime = currentTime;
      }
      
      Serial.println();
    }
    // 0x0C - IMUデータセットでのオイラー角（今回は使用しない）
    else if (field_descriptor == 0x0C && field_length >= 14) {
      Serial.println("⚠️ IMU形式のオイラー角(0x0C)を受信");
    }
    // GPSタイムスタンプ (0xD3)
    else if (field_descriptor == 0xD3 && field_length >= 14) {
      Serial.println("⏰ タイムスタンプ受信");
    }
    // 加速度データ(0x04)
    else if (field_descriptor == 0x04 && field_length >= 14) {
      Serial.println("⚠️ 加速度データ(0x04)を受信");
    }
    // その他の未知フィールド
    else {
      Serial.print("⚠️ 未知のフィールド: 0x");
      Serial.print(field_descriptor, HEX);
      Serial.print(" (Length=");
      Serial.print(field_length);
      Serial.println(")");
    }
    
    idx += field_length;
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