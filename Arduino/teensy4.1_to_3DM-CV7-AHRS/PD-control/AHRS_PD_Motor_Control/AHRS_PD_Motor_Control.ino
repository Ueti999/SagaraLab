// 3DM-CV7-AHRS + PMX-SCR-5204HV PD制御プログラム
// Teensy 4.1 USBホスト + Serial1でモータ制御

#include "USBHost_t36.h"
#include <PmxHardSerialClass.h>
#include <DataConvert.h>
#include <math.h>

// ========== USBホスト設定 ==========
USBHost myusb;
USBHub hub1(myusb);                                                                     
USBHub hub2(myusb);
USBSerial userial(myusb, 1);

// ========== モータ制御設定 ==========
const byte EN_PIN = 31;            // Serial1のENピン31→255にしてEN信号を無効化
const long MOTOR_BAUDRATE = 115200;
const int MOTOR_TIMEOUT = 1000;
PmxHardSerial pmx(&Serial1, EN_PIN, MOTOR_BAUDRATE, MOTOR_TIMEOUT);
const byte SERVO_ID = 0;           // サーボのID

// ========== センサー設定 ==========
const uint8_t SYNC1 = 0x75;
const uint8_t SYNC2 = 0x65;
uint8_t buffer[1024];
int bufferIndex = 0;
bool sensorConnected = false;
bool sensorConfigured = false;

// ========== PD制御パラメータ ==========
const double Kp = 8.0;              // 比例ゲイン
const double Kd = 0.3;              // 微分ゲイン
double theta_target = 0.0;          // 目標角度[rad] (初期値0度)
const double CONTROL_PERIOD = 10;   // 制御周期[ms]
const int TORQUE_LIMIT = 6800;     // トルクリミット

// ========== センサーデータ ==========
float current_roll = 0.0;
float current_pitch = 0.0;
float current_yaw = 0.0;
float gyro_x = 0.0;
float gyro_y = 0.0;
float gyro_z = 0.0;
bool new_sensor_data = false;

// ========== 制御タイミング ==========
unsigned long last_control_time = 0;
unsigned long last_sensor_time = 0;

// ========== デバッグ設定 ==========
const bool DEBUG_MODE = true;
const bool SHOW_CONTROL_DATA = true;

void setup() {
  Serial.begin(115200);
  
  // 起動確認LED
  pinMode(LED_BUILTIN, OUTPUT);
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  
  while (!Serial && millis() < 3000);
  
  Serial.println("\n===== AHRS + PMX PD制御システム起動 =====");
  Serial.println("システム初期化中...");
  
  // USBホスト初期化
  Serial.print("USBホスト初期化...");
  myusb.begin();
  Serial.println("完了");
  
  // USBホスト接続確認
  Serial.println("\n[重要] 接続確認:");
  Serial.println("1. センサーはTeensy 4.1のUSBホストポート(5ピンヘッダー)に接続");
  Serial.println("2. 通常のUSBポート(Type-B)ではありません");
  Serial.println("3. USBホストの電源供給を確認");
  Serial.println("   - VUSBとVINをジャンパーで接続が必要な場合があります");
  Serial.println("\nコマンド:");
  Serial.println("  'c' - センサーを設定");
  Serial.println("  's' - 状態確認\n");
  
  // モータ初期化
  Serial.print("モータ通信初期化...");
  delay(3000);//モータ起動待ち時間を0.5sから3sに変更
  pmx.begin();
  
  // モータをトルク制御モードに設定
  setupMotorTorqueMode();
  Serial.println("完了");
  
  Serial.println("\n目標角度設定:");
  Serial.println("  '+' : 目標角度 +10度");
  Serial.println("  '-' : 目標角度 -10度");
  Serial.println("  '0' : 目標角度 0度にリセット");
  Serial.println("  'c' : センサー設定送信");
  Serial.println("\nセンサーを接続してください...");
  
  delay(2000);
}

void loop() {
  // USBホストタスク
  myusb.Task();
  
  // センサー接続管理
  manageSensorConnection();
  
  // センサーデータ読み取り
  if (sensorConfigured) {
    readSensorData();
  }
  
  // PD制御実行（10ms周期）
  unsigned long current_time = millis();
  if (current_time - last_control_time >= CONTROL_PERIOD) {
    if (new_sensor_data) {
      executePDControl();
      new_sensor_data = false;
    }
    last_control_time = current_time;
  }
  
  // キーボード入力処理
  handleKeyboardInput();
  
  // 状態表示（1秒ごと）
  static unsigned long last_display_time = 0;
  if (current_time - last_display_time >= 1000) {
    displayStatus();
    last_display_time = current_time;
  }
}

void manageSensorConnection() {
  // USBデバイス検出デバッグ
  static unsigned long last_check = 0;
  if (millis() - last_check > 2000) {
    Serial.print("USB Host Status: ");
    Serial.print("userial=");
    Serial.print(userial ? "true" : "false");
    Serial.print(", connected=");
    Serial.println(sensorConnected ? "true" : "false");
    last_check = millis();
  }
  
  if (userial) {
    if (!sensorConnected) {
      Serial.println("\n🔌 センサー検出！");
      Serial.print("VID: 0x");
      Serial.print(userial.idVendor(), HEX);
      Serial.print(", PID: 0x");
      Serial.println(userial.idProduct(), HEX);
      
      userial.begin(115200);
      sensorConnected = true;
      sensorConfigured = true;  // SensorConnectで設定済みなので即true
      delay(500);
      
      // 自動設定をスキップ（SensorConnectの設定を使用）
      Serial.println("SensorConnectの設定を使用します");
      Serial.println("期待されるデータ:");
      Serial.println("  - 0x05: オイラー角");
      Serial.println("  - 0x10: Angular Rate（フィルター済み角速度）");
    }
  } else {
    if (sensorConnected) {
      Serial.println("センサー切断");
      sensorConnected = false;
      sensorConfigured = false;
      bufferIndex = 0;
    }
  }
}

void configureSensor() {
  Serial.println("\n===== センサー設定開始 =====");
  Serial.println("⚠️ 注意: センサーが既に設定されている可能性があります");
  
  uint8_t checksum1, checksum2;
  
  // Step 1: Idle状態にする
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
  
  Serial.println("Step 1: センサーをアイドル状態にする");
  userial.write(idle_cmd, sizeof(idle_cmd));
  delay(500);
  
  // Step 2: IMUストリームを無効化（フィルター済みデータのみ使用）
  // ※ コメントアウトしてIMUデータを無効化
  
  // Step 3: AHRSデータフォーマット設定（オイラー角 + Angular Rate）
  uint8_t set_ahrs_format[] = {
    0x75, 0x65,  // Sync bytes
    0x0C,        // Command set
    0x0D,        // Payload length (13バイトに増加)
    0x08,        // Message Format command
    0x02,        // Descriptor set (0x02 = AHRS/Filter)
    0x02,        // Number of fields (2つに変更)
    0x05,        // Field 1: Euler Angles
    0x00, 0x01,  // Rate divider (every packet)
    0x10,        // Field 2: Angular Rate (追加)
    0x00, 0x01,  // Rate divider (every packet)
    0x00, 0x00   // チェックサム
  };
  calculateChecksum(set_ahrs_format, sizeof(set_ahrs_format) - 2, &checksum1, &checksum2);
  set_ahrs_format[sizeof(set_ahrs_format) - 2] = checksum1;
  set_ahrs_format[sizeof(set_ahrs_format) - 1] = checksum2;
  
  Serial.println("Step 3: AHRSフォーマット設定（オイラー角 + Angular Rate @ 100Hz）");
  userial.write(set_ahrs_format, sizeof(set_ahrs_format));
  delay(200);
  
  // Step 4: AHRSストリームのみを有効化
  uint8_t enable_streams[] = {
    0x75, 0x65,  // Sync bytes
    0x0C,        // Command set
    0x05,        // Payload length
    0x11,        // Enable Data Stream
    0x01,        // Function selector
    0x02,        // AHRS(0x82) stream only
    0x01,        // Enable
    0x00, 0x00   // チェックサム
  };
  calculateChecksum(enable_streams, sizeof(enable_streams) - 2, &checksum1, &checksum2);
  enable_streams[sizeof(enable_streams) - 2] = checksum1;
  enable_streams[sizeof(enable_streams) - 1] = checksum2;
  
  Serial.println("Step 4: AHRS(0x82)ストリームのみを有効化");
  userial.write(enable_streams, sizeof(enable_streams));
  delay(200);
  
  Serial.println("✅ センサー設定完了！");
  Serial.println("期待される出力:");
  Serial.println("  - 0x82パケットのみ");
  Serial.println("    - 0x05: オイラー角 (Roll/Pitch/Yaw)");
  Serial.println("    - 0x10: Angular Rate (フィルター済み角速度)");
}

void readSensorData() {
  while (userial.available()) {
    if (bufferIndex >= (int)(sizeof(buffer) - 1)) {
      shiftBuffer();
    }
    
    uint8_t byte = userial.read();
    buffer[bufferIndex++] = byte;
    last_sensor_time = millis();
    
    processBuffer();
  }
}

void processBuffer() {
  int processedBytes = 0;
  
  while (processedBytes < bufferIndex - 3) {
    bool foundPacket = false;
    
    for (int i = processedBytes; i <= bufferIndex - 4; i++) {
      if (buffer[i] == SYNC1 && buffer[i+1] == SYNC2) {
        uint8_t length = buffer[i+3];
        int packetSize = 4 + length + 2;
        
        if (i + packetSize <= bufferIndex) {
          if (verifyChecksum(&buffer[i], packetSize)) {
            processMIPPacket(&buffer[i], packetSize);
          }
          processedBytes = i + packetSize;
          foundPacket = true;
          break;
        } else {
          processedBytes = i;
          foundPacket = false;
          break;
        }
      }
    }
    
    if (!foundPacket) break;
  }
  
  if (processedBytes > 0) {
    memmove(buffer, &buffer[processedBytes], bufferIndex - processedBytes);
    bufferIndex -= processedBytes;
  }
}

void processMIPPacket(uint8_t* packet, int length) {
  uint8_t descriptor = packet[2];
  uint8_t payload_length = packet[3];
  
  if (descriptor == 0x80) {
    // IMUデータ（ジャイロ）
    parseIMUData(&packet[4], payload_length);
  } else if (descriptor == 0x82) {
    // AHRSデータ（オイラー角）
    parseAHRSData(&packet[4], payload_length);
  }
}

void parseIMUData(uint8_t* data, int length) {
  int idx = 0;
  
  while (idx < length - 2) {
    uint8_t field_length = data[idx];
    uint8_t field_descriptor = data[idx + 1];
    
    if (field_length == 0 || field_length > length - idx) break;
    
    // ジャイロデータ (0x05)
    if (field_descriptor == 0x05 && field_length >= 14) {
      gyro_x = parseFloat(&data[idx + 2]);
      gyro_y = parseFloat(&data[idx + 6]);
      gyro_z = parseFloat(&data[idx + 10]);
      
      if (DEBUG_MODE) {
        Serial.print("Gyro: X=");
        Serial.print(gyro_x, 3);
        Serial.print(" Y=");
        Serial.print(gyro_y, 3);
        Serial.print(" Z=");
        Serial.println(gyro_z, 3);
      }
    }
    
    idx += field_length;
  }
}

void parseAHRSData(uint8_t* data, int length) {
  int idx = 0;
  
  while (idx < length - 2) {
    uint8_t field_length = data[idx];
    uint8_t field_descriptor = data[idx + 1];
    
    if (field_length == 0 || field_length > length - idx) break;
    
    // オイラー角 (0x05 in AHRS context)
    if (field_descriptor == 0x05 && field_length >= 16) {
      current_roll = parseFloat(&data[idx + 2]);
      current_pitch = parseFloat(&data[idx + 6]);
      current_yaw = parseFloat(&data[idx + 10]);
      new_sensor_data = true;
      
      if (DEBUG_MODE) {
        Serial.print("Euler: R=");
        Serial.print(current_roll * 180.0 / PI, 1);
        Serial.print("° P=");
        Serial.print(current_pitch * 180.0 / PI, 1);
        Serial.print("° Y=");
        Serial.println(current_yaw * 180.0 / PI, 1);
      }
    }
    // Angular Rate (0x10) - フィルター済み角速度
    else if (field_descriptor == 0x0E && field_length >= 14) {
      gyro_x = parseFloat(&data[idx + 2]);
      gyro_y = parseFloat(&data[idx + 6]);
      gyro_z = parseFloat(&data[idx + 10]);
      
      // デバッグ用（5秒ごとに表示）
      static unsigned long last_gyro_debug = 0;
      if (millis() - last_gyro_debug > 5000) {
        Serial.print("[INFO] フィルター済み角速度取得: X=");
        Serial.print(gyro_x, 4);
        Serial.print(" Y=");
        Serial.print(gyro_y, 4);
        Serial.print(" Z=");
        Serial.print(gyro_z, 4);
        Serial.println(" rad/s");
        last_gyro_debug = millis();
      }
    }
    // デバッグ: 未知のフィールド
    else {
      static unsigned long last_unknown = 0;
      if (millis() - last_unknown > 10000) {  // 10秒ごと
        Serial.print("[DEBUG] AHRSパケット内のフィールド: 0x");
        Serial.print(field_descriptor, HEX);
        Serial.print(" (長さ=");
        Serial.print(field_length);
        Serial.println("バイト)");
        last_unknown = millis();
      }
    }
    
    idx += field_length;
  }
}

void executePDControl() {
  // PD制御計算（ピッチ軸を制御）
  double error = theta_target - current_pitch;
  double torque = -Kp * error - Kd * gyro_y;
  
  // トルクをモータ指令値に変換（mNm単位）
  int motor_torque = (int)(torque * 1000);
  
  // トルクリミット
  if (motor_torque > TORQUE_LIMIT) {
    motor_torque = TORQUE_LIMIT;
  } else if (motor_torque < -TORQUE_LIMIT) {
    motor_torque = -TORQUE_LIMIT;
  }
  
  // モータにトルク指令送信
  sendMotorTorque(motor_torque);
  
  if (SHOW_CONTROL_DATA) {
    static unsigned long last_print = 0;
    if (millis() - last_print > 100) {  // 100msごとに表示
      Serial.print("PD制御: 目標=");
      Serial.print(theta_target * 180.0 / PI, 1);
      Serial.print("° 現在=");
      Serial.print(current_pitch * 180.0 / PI, 1);
      Serial.print("° 誤差=");
      Serial.print(error * 180.0 / PI, 1);
      Serial.print("° 角速度=");
      Serial.print(gyro_y, 3);
      Serial.print("rad/s トルク=");
      Serial.print(motor_torque);
      Serial.println("mNm");
      last_print = millis();
    }
  }
}

// TODO(human): PD制御ゲインの調整ロジックを実装
// この関数では、システムの応答を観察してKpとKdを動的に調整します。
// 振動が発生した場合はKdを増やし、応答が遅い場合はKpを増やすなど、
// 適応的な制御を実現してください。
void adjustPDGains() {
  // ここにゲイン調整ロジックを実装してください
  // 例: 振動検出、応答速度評価、ゲインの自動調整
}

void setupMotorTorqueMode() {
  // 制御モードをトルク制御に設定
  byte controlMode = PMX::ControlMode::Torque;
  byte receiveMode = PMX::ReceiveDataOption::Position;//fullデータを要求からPositionのみに変更
  byte writeOpt = 0;  // トルクONでも強制書き込み（1)から通常の書き込みモード（0）に変更した
  
  uint16_t flag = pmx.setControlMode(SERVO_ID, controlMode, writeOpt);
  if (DEBUG_MODE) {
    Serial.print("setControlMode=");
    Serial.println(flag, HEX);
  }
  
  flag = pmx.setMotorReceive(SERVO_ID, receiveMode, writeOpt);
  if (DEBUG_MODE) {
    Serial.print("setMotorReceive=");
    Serial.println(flag, HEX);
  }
  
  // トルクON
  long receiveData[8];
  flag = pmx.setMotorTorqueOn(SERVO_ID, receiveMode, receiveData, controlMode);
  if (DEBUG_MODE) {
    Serial.print("setMotorTorqueOn=");
    Serial.println(flag, HEX);
  }
}

void sendMotorTorque(int torque_value) {
  // トルク値をPMXフォーマットで送信
  long writeDatas[1] = {torque_value};
  byte controlMode = PMX::ControlMode::Torque;
  byte receiveMode = PMX::ReceiveDataOption::Position;
  long receiveData[8];
  
  uint16_t flag = pmx.MotorWRITE(SERVO_ID, writeDatas, 1, receiveMode, receiveData, controlMode);
  //モータエラー
  if (DEBUG_MODE && flag != 0) {
    Serial.print("Motor write error: ");
    Serial.println(flag, HEX);
  }
}

void handleKeyboardInput() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case '+':
        theta_target += 10.0 * PI / 180.0;  // +10度
        Serial.print("目標角度変更: ");
        Serial.print(theta_target * 180.0 / PI);
        Serial.println("度");
        break;
        
      case '-':
        theta_target -= 10.0 * PI / 180.0;  // -10度
        Serial.print("目標角度変更: ");
        Serial.print(theta_target * 180.0 / PI);
        Serial.println("度");
        break;
        
      case '0':
        theta_target = 0.0;
        Serial.println("目標角度リセット: 0度");
        break;
        
      case 'c':
      case 'C':
        Serial.println("※ SensorConnectで設定済みの場合、再設定は不要です");
        Serial.println("本当に設定を送信する場合は 'f' キーを押してください");
        break;
        
      case 'f':
      case 'F':
        if (sensorConnected) {
          Serial.println("強制的にセンサー設定を送信...");
          configureSensor();
        } else {
          Serial.println("センサーが接続されていません");
        }
        break;
        
      case 's':
      case 'S':
        Serial.println("\n📊 現在の状態:");
        Serial.print("センサー接続: ");
        Serial.println(sensorConnected ? "Yes" : "No");
        Serial.print("USBSerial: ");
        Serial.println(userial ? "検出" : "未検出");
        Serial.println("- 0x80パケット = IMU生データ（ジャイロ）");
        Serial.println("- 0x82パケット = AHRSフィルターデータ（オイラー角）");
        break;
    }
  }
}

void displayStatus() {
  Serial.println("=== システム状態 ===");
  Serial.print("センサー: ");
  Serial.println(sensorConnected ? "接続" : "未接続");
  Serial.print("目標角度: ");
  Serial.print(theta_target * 180.0 / PI);
  Serial.println("度");
  Serial.print("現在角度: Roll=");
  Serial.print(current_roll * 180.0 / PI, 1);
  Serial.print("° Pitch=");
  Serial.print(current_pitch * 180.0 / PI, 1);
  Serial.print("° Yaw=");
  Serial.print(current_yaw * 180.0 / PI, 1);
  Serial.println("°");
  Serial.print("角速度: X=");
  Serial.print(gyro_x, 3);
  Serial.print("rad/s Y=");
  Serial.print(gyro_y, 3);
  Serial.print("rad/s Z=");
  Serial.print(gyro_z, 3);
  Serial.println("rad/s");
}

void shiftBuffer() {
  int shiftAmount = bufferIndex / 2;
  memmove(buffer, &buffer[shiftAmount], bufferIndex - shiftAmount);
  bufferIndex -= shiftAmount;
}

bool verifyChecksum(uint8_t* packet, int length) {
  uint8_t calc_checksum1, calc_checksum2;
  calculateChecksum(packet, length - 2, &calc_checksum1, &calc_checksum2);
  return (packet[length - 2] == calc_checksum1 && 
          packet[length - 1] == calc_checksum2);
}

float parseFloat(uint8_t* bytes) {
  union {
    float f;
    uint8_t b[4];
  } u;
  
  // ビッグエンディアンからリトルエンディアンへ変換
  u.b[0] = bytes[3];
  u.b[1] = bytes[2];
  u.b[2] = bytes[1];
  u.b[3] = bytes[0];
  
  return u.f;
}

void calculateChecksum(uint8_t* data, int length, uint8_t* checksum1, uint8_t* checksum2) {
  uint8_t sum1 = 0;
  uint8_t sum2 = 0;
  
  for (int i = 0; i < length; i++) {
    sum1 = (sum1 + data[i]) & 0xFF;
    sum2 = (sum2 + sum1) & 0xFF;
  }
  
  *checksum1 = sum1;
  *checksum2 = sum2;
}