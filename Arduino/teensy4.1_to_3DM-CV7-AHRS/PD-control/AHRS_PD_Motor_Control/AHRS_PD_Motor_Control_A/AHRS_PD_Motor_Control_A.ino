// 3DM-CV7-AHRS + PMX-SCR-5204HV PD制御プログラム
// Teensy 4.1 USBホスト + Serial1でモータ制御

#include "USBHost_t36.h"
#include <PmxHardSerialClass.h>
#include <DataConvert.h>
#include <math.h>
// SDカードはRaspberry Pi側で管理するため不要

// ========== USBホスト設定 ==========
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBSerial userial(myusb, 1);        // 3DM-CV7-AHRSセンサー用
// Raspberry PiはUSBシリアル（Serial）で通信

// ========== モータ制御設定 ==========
const byte EN_PIN = 31;            // Serial1のENピン（手動制御に戻す）
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
bool raspiConnected = false;         // Raspberry Pi接続状態（USBシリアル経由）

// ========== PD制御パラメータ ==========
const double Kp = 8.0;              // 比例ゲイン
const double Kd = 0.3;              // 微分ゲイン
double theta_target = 0.20;         // 目標角度[rad] (初期値約11.5度)
const double CONTROL_PERIOD = 5;    // 制御周期[ms] (200Hz)
const int TORQUE_LIMIT = 16000;    // トルクリミット[mNm]

// ========== 動力学パラメータ ==========
const double m_torso = 0.450;      // 胴体質量[kg]
const double d_torso = 0.0846;     // 胴体重心位置[m]
const double gravity = 9.81;       // 重力加速度[m/s^2]

// ========== センサーデータ ==========
float current_roll = 0.0;
float current_pitch = 0.0;
float current_yaw = 0.0;
float gyro_x = 0.0;
float gyro_y = 0.0;
float gyro_z = 0.0;
bool new_sensor_data = false;

// ========== エンコーダ・モータ状態 ==========
int encoder_offset = 0;            // エンコーダオフセット
int previous_encoder = 0;          // 前回のエンコーダ値
int encoder_segment = 0;           // エンコーダセグメント（境界処理用）
double theta_motor = 0.0;          // モータ角度[rad]
double omega_motor = 0.0;          // モータ角速度[rad/s]
double previous_theta_motor = 0.0; // 前回のモータ角度

// ========== 支持脚状態（歩行制御用） ==========
double thetaq = 0.0;               // 支持脚と胴体の相対角[rad]
double theta_leg = 0.0;            // 支持脚角度[rad]
double dtheta_leg = 0.0;           // 支持脚角速度[rad/s]
double previous_theta_leg = M_PI/8; // 前回の支持脚角度
double previous_thetaq = 0.0;      // 前回の相対角
int walk_count = 0;                // 歩行ステップカウンタ
int q_segment = 0;                 // 支持脚角度のセグメント管理

// ========== 制御タイミング ==========
unsigned long last_control_time = 0;
unsigned long last_sensor_time = 0;
unsigned long last_sd_log_time = 0;

// ========== 関数プロトタイプ宣言 ==========
void initializeSDCard();
void logDataToSDCard();
void initializeEncoder();
void updateMotorState();
void calculateLegState();
void setupMotorTorqueMode();
void sendMotorTorque(int torque_value);
void manageSensorConnection();
void configureSensor();
void readSensorData();
void processBuffer();
void processMIPPacket(uint8_t* packet, int length);
void parseIMUData(uint8_t* data, int length);
void parseAHRSData(uint8_t* data, int length);
void executePDControl();
void handleKeyboardInput();
void displayStatus();
void shiftBuffer();
bool verifyChecksum(uint8_t* packet, int length);
float parseFloat(uint8_t* bytes);
void calculateChecksum(uint8_t* data, int length, uint8_t* checksum1, uint8_t* checksum2);

// ========== デバッグ設定 ==========
const bool DEBUG_MODE = false;         // falseにしてデバッグ出力を無効化
const bool SHOW_CONTROL_DATA = false;  // 制御データ表示も無効化
const bool ENABLE_RASPI_LOG = true;   // Raspberry Piへのデータ送信有効化
const bool ENABLE_SD_LOG = false;     // SDカードは使用しない（Raspberry Pi側で保存）

// ========== データロギング設定 ==========
unsigned long last_log_time = 0;
const unsigned long LOG_PERIOD = 10;  // ログ送信周期[ms] (100Hz)
static int last_torque_for_raspi = 0;  // Raspberry Pi送信用トルク値

// SDカード機能はRaspberry Pi側で実装
unsigned long sessionStartTime = 0;

void setup() {
  Serial.begin(115200);
  
  // 起動確認LED（5回高速点滅 = プログラム開始）
  pinMode(LED_BUILTIN, OUTPUT);
  for(int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }

  // Serialの初期化待ち時間を短縮（Raspberry Pi接続用）
  delay(500);

  // 起動メッセージを無効化（CSVデータのみ送信）
  // Serial.println("\n===== AHRS + PMX PD制御システム起動 =====");
  // Serial.println("システム初期化中...");
  
  // USBホスト初期化
  // Serial.print("USBホスト初期化...");
  myusb.begin();
  // Serial.println("完了");
  
  // USBホスト接続確認（デバッグ出力を無効化）
  // 重要な設定情報はコメントとして残す
  // 1. センサーはTeensy 4.1のUSBホストポート(5ピンヘッダー)に接続
  // 2. 通常のUSBポート(Type-B)ではありません
  
  // モータ初期化
  // Serial.print("モータ通信初期化...");
  delay(3000);  // モータ起動待ち時間
  pmx.begin();
  // Serial.println("開始");

  // モータ接続確認
  Serial.print("モータ接続テスト...");
  long testData[8];
  uint16_t testFlag = pmx.MotorREAD(SERVO_ID, PMX::ReceiveDataOption::Torque, testData, PMX::ControlMode::Torque);
  if (testFlag == 0) {
    Serial.println("成功");
    Serial.print("  現在トルク: ");
    Serial.print(testData[1]);
    Serial.println("mNm");
  } else {
    Serial.print("失敗 (flag=0x");
    Serial.print(testFlag, HEX);
    Serial.println(")");
    Serial.println("\n[エラー] モータと通信できません！");
    Serial.println("確認項目:");
    Serial.println("  1. モータの電源がONになっているか");
    Serial.println("  2. RS485の配線（Serial1のTX/RX）");
    Serial.println("  3. モータIDが0に設定されているか");
    Serial.println("  4. ボーレートが115200か");
  }

  // モータをトルク制御モードに設定
  setupMotorTorqueMode();

  Serial.println("\n目標角度設定:");
  Serial.println("  '+' : 目標角度 +10度");
  Serial.println("  '-' : 目標角度 -10度");
  Serial.println("  '0' : 目標角度 0度にリセット");
  Serial.println("  'd' : 目標角度 0.2rad（約11.5度）にセット");
  Serial.println("  'c' : センサー設定送信");
  Serial.println("  's' : システム状態表示");
  Serial.println("\nSDカードログ:");
  Serial.println("  'l' : ログ状態確認");
  Serial.println("  'n' : 新しいログファイル開始");
  Serial.println("  'r' : SDカード再初期化");
  Serial.println("\nセンサーを接続してください...");

  // エンコーダ初期化
  delay(500);
  initializeEncoder();
  
  delay(2000);
}

void loop() {
  // 動作確認LED（1秒ごとに点滅）
  static unsigned long last_led_time = 0;
  static bool led_state = false;
  if (millis() - last_led_time > 1000) {
    led_state = !led_state;
    digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);
    last_led_time = millis();
  }

  // USBホストタスク
  myusb.Task();
  
  // センサー接続管理
  manageSensorConnection();
  
  // センサーデータ読み取り
  if (sensorConfigured) {
    readSensorData();
  }

  // Raspberry Pi接続管理（USBシリアル経由）
  if (!raspiConnected && Serial) {
    raspiConnected = true;
    Serial.println("#CONNECTED:TEENSY");
    Serial.println("# Raspberry Pi接続確立");
    sessionStartTime = millis();
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
  
  // 状慎表示（1秒ごと）
  static unsigned long last_display_time = 0;
  if (current_time - last_display_time >= 1000) {
    displayStatus();
    last_display_time = current_time;
  }

  // Raspberry Piへのデータ送信（10ms周期）
  if (ENABLE_RASPI_LOG && raspiConnected) {
    if (current_time - last_log_time >= LOG_PERIOD) {
      sendDataToRaspberryPi();
      last_log_time = current_time;
    }
  }

  // SDカード機能はRaspberry Pi側で実装
}

void manageSensorConnection() {
  // USBデバイス検出デバッグ
  static unsigned long last_check = 0;
  if (millis() - last_check > 2000) {
    Serial.print("USB Host Status: ");
    Serial.print("userial=");
    Serial.print(userial ? "true" : "false");
    // Serial.print(", raspi=");
    // Serial.print(raspiserial ? "true" : "false");
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
  // エンコーダ読み取りと支持脚状態更新
  updateMotorState();
  calculateLegState();

  // PD制御計算（ピッチ軸を制御）+ 重力補償
  double error = theta_target - current_pitch;
  double torque = -Kp * error - Kd * gyro_y
                  - m_torso * gravity * d_torso * sin(current_pitch);

  // トルクをモータ指令値に変換（mNm単位）
  int motor_torque = (int)(torque * 1000);
  last_torque_for_raspi = motor_torque;  // Raspberry Pi送信用に保存
  
  // トルクリミット
  if (motor_torque > TORQUE_LIMIT) {
    motor_torque = TORQUE_LIMIT;
  } else if (motor_torque < -TORQUE_LIMIT) {
    motor_torque = -TORQUE_LIMIT;
  }
  
  // モータにトルク指令送信（通信エラー時はスキップ）
  static int error_count = 0;
  if (error_count < 3) {  // 連続エラー3回まで
    sendMotorTorque(motor_torque);
  } else if (error_count == 3) {
    Serial.println("\n[警告] モータ通信エラーが継続 - トルク出力を停止");
    sendMotorTorque(0);  // 安全のため停止
    error_count++;
  }
  
  if (SHOW_CONTROL_DATA) {
    static unsigned long last_print = 0;
    if (millis() - last_print > 100) {  // 100msごとに表示
      Serial.print("PD: 目標=");
      Serial.print(theta_target * 180.0 / PI, 1);
      Serial.print("° Pitch=");
      Serial.print(current_pitch * 180.0 / PI, 1);
      Serial.print("° 誤差=");
      Serial.print(error * 180.0 / PI, 1);
      Serial.print("°");

      Serial.print(" | 支持脚=");
      Serial.print(theta_leg * 180.0 / PI, 1);
      Serial.print("° dθ_leg=");
      Serial.print(dtheta_leg, 2);

      Serial.print(" | τ=");
      Serial.print(motor_torque);
      Serial.print("mNm 歩数=");
      Serial.println(walk_count);
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
  Serial.println("\n===== モータトルクモード設定 =====");

  // 制御モードをトルク制御に設定
  byte controlMode = PMX::ControlMode::Torque;
  byte receiveMode = PMX::ReceiveDataOption::Position;  // PMXManagerの設定（位置+トルク）に合わせる
  byte writeOpt = 1;  // トルクON状態でも強制書き込み

  uint16_t flag = pmx.setControlMode(SERVO_ID, controlMode, writeOpt);
  Serial.print("1. setControlMode=");
  Serial.print(flag, HEX);
  if (flag == 0) {
    Serial.println(" [OK]");
  } else {
    Serial.println(" [NG] - モータ通信エラー");
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
  byte receiveMode = PMX::ReceiveDataOption::Position;  // 位置データを受信（PMXManager設定に合わせる）
  long receiveData[8];

  uint16_t flag = pmx.MotorWRITE(SERVO_ID, writeDatas, 1, receiveMode, receiveData, controlMode);

  // エラー詳細表示
  if (flag != 0) {
    Serial.print("[ERROR] Motor write failed: ");
    Serial.print("flag=0x");
    Serial.print(flag, HEX);
    Serial.print(" torque=");
    Serial.print(torque_value);
    Serial.println("mNm");

    // エラーコード解析
    if (flag == 0xFF00) {
      Serial.println("  -> タイムアウトエラー: モータ応答なし");
      Serial.println("  -> 確認事項:");
      Serial.println("     1. モータの電源ON?");
      Serial.println("     2. RS485配線OK?");
      Serial.println("     3. モータIDは0?");
    }
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

      case 'd':
      case 'D':
        theta_target = 0.20;  // Raspberry Pi版のデフォルト値
        Serial.println("目標角度: 0.2rad (約11.5度)");
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

      case 'l':
      case 'L':
        Serial.println("\n💾 ログ機能:");
        Serial.println("  データロギングはRaspberry Pi側で実行されています");
        Serial.println("  Raspberry Pi側のログファイルを確認してください");
        break;

      case 'n':
      case 'N':
        Serial.println("ログファイル作成はRaspberry Pi側で管理されています");
        break;

      case 'r':
      case 'R':
        Serial.println("SDカード機能は無効化されています（Raspberry Piでログ記録）");
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

  // IMUセンサー情報
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

  // 歩行制御情報
  Serial.print("支持脚角度: ");
  Serial.print(theta_leg * 180.0 / PI, 1);
  Serial.print("° 角速度: ");
  Serial.print(dtheta_leg, 3);
  Serial.print("rad/s 歩数: ");
  Serial.println(walk_count);

  Serial.print("モータ角度: ");
  Serial.print(theta_motor * 180.0 / PI, 1);
  Serial.print("° 相対角: ");
  Serial.print(thetaq * 180.0 / PI, 1);
  Serial.print("° モータ角速度: ");
  Serial.print(omega_motor, 3);
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

// ラズパイにデータを送信（USBシリアル版）
void sendDataToRaspberryPi() {
  if (!raspiConnected) return;

  // CSV形式でデータ送信
  // Format: timestamp,roll,pitch,yaw,gyro_x,gyro_y,gyro_z,theta_motor,theta_leg,dtheta_leg,walk_count,target,torque

  // タイムスタンプ（ミリ秒）
  Serial.print(millis());
  Serial.print(",");

  // IMUデータ
  Serial.print(current_roll, 4);
  Serial.print(",");
  Serial.print(current_pitch, 4);
  Serial.print(",");
  Serial.print(current_yaw, 4);
  Serial.print(",");
  Serial.print(gyro_x, 4);
  Serial.print(",");
  Serial.print(gyro_y, 4);
  Serial.print(",");
  Serial.print(gyro_z, 4);
  Serial.print(",");

  // モータ・支持脚データ
  Serial.print(theta_motor, 4);
  Serial.print(",");
  Serial.print(theta_leg, 4);
  Serial.print(",");
  Serial.print(dtheta_leg, 4);
  Serial.print(",");

  // 歩数カウント
  Serial.print(walk_count);
  Serial.print(",");

  // PD制御データ
  Serial.print(theta_target, 4);
  Serial.print(",");
  Serial.println(last_torque_for_raspi);
}

// ========== モータエンコーダ関連関数 ==========
void updateMotorState() {
  // PMXモータからエンコーダ値とステータスを読み取り
  long receiveData[8];
  // トルク制御中でも位置情報を取得
  byte receiveMode = PMX::ReceiveDataOption::Position;  // エンコーダ位置を取得
  uint16_t flag = pmx.MotorREAD(SERVO_ID, receiveMode, receiveData, PMX::ControlMode::Torque);

  // デバッグ: 読み取り結果を表示
  if (DEBUG_MODE) {
    static unsigned long last_debug = 0;
    if (millis() - last_debug > 1000) {
      Serial.print("[MOTOR READ] flag=");
      Serial.print(flag, HEX);
      Serial.print(" pos=");
      Serial.print(receiveData[1]);
      Serial.print(" status=");
      Serial.println(receiveData[0]);
      last_debug = millis();
    }
  }

  if (flag == 0) {
    int raw_encoder = (int)receiveData[1];  // 現在位置（エンコーダ値）

    // エンコーダの境界処理（PMXの分解能に応じて調整が必要）
    const int ENCODER_WRAP_THRESHOLD = 2000;
    const int ENCODER_RESOLUTION = 4096;  // PMXモータの分解能に応じて変更

    if (previous_encoder - raw_encoder >= ENCODER_WRAP_THRESHOLD) {
      encoder_segment++;
    }
    if (raw_encoder - previous_encoder >= ENCODER_WRAP_THRESHOLD) {
      encoder_segment--;
    }

    // オフセット済みエンコーダ値の計算
    int offset_encoder = raw_encoder - encoder_offset;
    if (encoder_segment != 0) {
      offset_encoder += ENCODER_RESOLUTION * encoder_segment;
    }

    // 角度への変換
    theta_motor = 2.0 * M_PI * offset_encoder / ENCODER_RESOLUTION;

    // 角速度の計算
    static unsigned long last_encoder_time = 0;
    unsigned long now = millis();
    if (last_encoder_time != 0 && now != last_encoder_time) {
      double dt = (now - last_encoder_time) / 1000.0;
      omega_motor = (theta_motor - previous_theta_motor) / dt;
    }

    previous_encoder = raw_encoder;
    previous_theta_motor = theta_motor;
    last_encoder_time = now;
  }
}

void calculateLegState() {
  // 支持脚と胴体の相対角を計算
  thetaq = 7.0/8.0 * M_PI + theta_motor;

  // セグメント処理（角度の連続性確保）
  if (encoder_segment > 0) {
    thetaq -= 2 * M_PI * encoder_segment;
  }
  if (thetaq < 0) {
    thetaq += 2 * M_PI;
  }

  // 支持脚角度の計算
  theta_leg = M_PI - thetaq - current_pitch + (M_PI/4) * walk_count;

  // 支持脚切り替え検出
  if (previous_thetaq - thetaq > 6.0) {
    q_segment++;
  }
  if (q_segment > 0) {
    theta_leg -= 2 * M_PI * q_segment;
  }

  // 歩行ステップカウント
  if (previous_theta_leg - theta_leg > M_PI/4) {
    walk_count++;
    if (DEBUG_MODE) {
      Serial.print("[歩行] ステップ: ");
      Serial.println(walk_count);
    }
  }

  // 支持脚角速度の計算
  static unsigned long last_leg_time = 0;
  unsigned long now = millis();
  if (last_leg_time != 0 && now != last_leg_time) {
    double dt = (now - last_leg_time) / 1000.0;
    dtheta_leg = -(theta_leg - previous_theta_leg) / dt;
  }

  // リミット処理
  if (theta_leg > M_PI/8) {
    theta_leg = M_PI/8;
  }
  if (theta_leg < -M_PI/8) {
    theta_leg = -M_PI/8;
  }

  // 状態を保存
  previous_theta_leg = theta_leg;
  previous_thetaq = thetaq;
  last_leg_time = now;
}

void initializeEncoder() {
  // エンコーダの初期値を取得
  long receiveData[8];
  byte receiveMode = PMX::ReceiveDataOption::Position;  // 位置情報を取得
  uint16_t flag = pmx.MotorREAD(SERVO_ID, receiveMode, receiveData, PMX::ControlMode::Torque);

  if (flag == 0) {
    encoder_offset = (int)receiveData[1];
    previous_encoder = encoder_offset;
    Serial.print("エンコーダオフセット設定: ");
    Serial.println(encoder_offset);
  }
}
