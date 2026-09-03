/*
  EV-BLACKBOX Firmware v0.4.0

  대상 보드
  - ESP32 DevKit V1, 38핀

  주요 기능
  - PZT 4채널 충격 감지 및 최대값 수집
  - MPU6050을 이용한 진동 교차 확인
  - 충격 위치(상대 좌표 및 구역) 추정
  - 상대 충격수준 및 점검 우선도 분류
  - SSD1306 OLED, LED, 능동 부저 출력
  - DS3231 실제 날짜·시각 기록
  - microSD 요약 CSV와 충격 전후 추적 CSV 기록
  - 데이터 수집용 수동 사건 라벨

  중요
  - PZT는 보호·피크홀드 회로를 거쳐서 연결해야 합니다.
  - PZT를 ESP32 ADC 핀에 직접 연결하면 안 됩니다.
  - 현재 충격수준 기준값은 축소시험용 초기값입니다.
    실제 알루미늄판 실험 데이터로 반드시 다시 보정해야 합니다.
  - 이 펌웨어는 배터리 손상을 진단하지 않습니다.
    출력 등급은 점검 우선도를 정하기 위한 상대 충격등급입니다.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <esp_system.h>

#include <Adafruit_GFX.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>

namespace Config {

// ---------------------------------------------------------------------------
// 펌웨어 및 하드웨어 설정
// ---------------------------------------------------------------------------

constexpr char FIRMWARE_VERSION[] = "0.4.1";
constexpr uint32_t SERIAL_BAUD = 115200;

constexpr size_t PZT_COUNT = 4;
constexpr uint8_t PZT_ALL_CHANNELS_MASK =
    static_cast<uint8_t>((1u << PZT_COUNT) - 1u);

// 센서의 실제 부착 위치와 아래 순서를 반드시 일치시키세요.
// 0: Front-Left, 1: Front-Right, 2: Rear-Left, 3: Rear-Right
constexpr uint8_t PZT_PINS[PZT_COUNT] = {32, 33, 34, 35};
constexpr float PZT_GAIN[PZT_COUNT] = {1.00f, 1.00f, 1.00f, 1.00f};
constexpr float PZT_X[PZT_COUNT] = {0.0f, 1.0f, 0.0f, 1.0f};
constexpr float PZT_Y[PZT_COUNT] = {1.0f, 1.0f, 0.0f, 0.0f};

constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr uint16_t I2C_TIMEOUT_MS = 10;

// DS3231은 0x68을 사용합니다. MPU6050 AD0를 3.3V에 연결해 0x69로
// 사용하는 구성이 우선입니다. RTC가 없을 때의 기존 0x68 MPU도 지원합니다.
constexpr uint8_t MPU_ADDRESS_PRIMARY = 0x69;
constexpr uint8_t MPU_ADDRESS_SECONDARY = 0x68;
constexpr uint8_t RTC_ADDRESS = 0x68;
constexpr uint8_t MPU_ACCEL_XOUT_H_REG = 0x3B;
constexpr uint8_t MPU_ACCEL_CONFIG_REG = 0x1C;
constexpr uint8_t MPU_ACCEL_CONFIG_16G = 0x18;
constexpr size_t MPU_ACCEL_BYTES = 6;
constexpr float MPU_ACCEL_LSB_PER_G = 2048.0f;
constexpr float MPU_SATURATION_G = 15.5f;
constexpr uint8_t MPU_CONFIG_VERIFY_ATTEMPTS = 3;

constexpr uint8_t BUZZER_PIN = 25;
constexpr bool BUZZER_ACTIVE_HIGH = true;

constexpr uint8_t LED_GREEN_PIN = 26;
constexpr uint8_t LED_YELLOW_PIN = 27;
constexpr uint8_t LED_RED_PIN = 14;

constexpr uint8_t SD_SCK_PIN = 18;
constexpr uint8_t SD_MISO_PIN = 19;
constexpr uint8_t SD_MOSI_PIN = 23;
constexpr uint8_t SD_CS_PIN = 13;
constexpr uint32_t SD_SPI_HZ = 4000000;
constexpr char SD_LOG_PATH[] = "/impact_log_v3.csv";
constexpr size_t LOG_QUEUE_CAPACITY = 8;
constexpr uint32_t LOG_WRITE_DELAY_MS = 20;
constexpr uint32_t SD_RETRY_PERIOD_MS = 30000;
constexpr char CSV_HEADER[] =
    "session,event_id,source,event_type,rtc_iso8601,time_source,rtc_valid,trigger_ms,"
    "baseline_fl,baseline_fr,baseline_rl,baseline_rr,"
    "raw_fl,raw_fr,raw_rl,raw_rr,"
    "corrected_fl,corrected_fr,corrected_rl,corrected_rr,"
    "imu_peak_delta_g,x_norm,y_norm,zone,location_quality,"
    "impact_score,inspection_level,latency_ms,valid,saturation_mask,imu_saturated,"
    "active_channel_mask,usable_channel_mask,error_flags,"
    "imu_data_valid,trace_path,trace_samples,trace_saved,firmware_version";
constexpr char TRACE_HEADER[] =
    "relative_us,pzt_fl,pzt_fr,pzt_rl,pzt_rr,"
    "accel_x_g,accel_y_g,accel_z_g,imu_valid";
constexpr size_t TRACE_PATH_SIZE = 48;

constexpr uint8_t OLED_ADDRESS = 0x3C;
constexpr int8_t OLED_RESET_PIN = -1;
constexpr uint8_t OLED_WIDTH = 128;

// 주문한 0.91인치 OLED는 128x32입니다.
// 나중에 128x64 제품으로 바꾸면 32를 64로 수정하세요.
constexpr uint8_t OLED_HEIGHT = 32;

// ---------------------------------------------------------------------------
// 측정 주기 및 상태 시간
// ---------------------------------------------------------------------------

constexpr uint8_t ADC_BITS = 12;
constexpr uint16_t ADC_MAX_COUNTS = 4095;
constexpr uint16_t ADC_SATURATION_COUNTS = 4000;

// 네 채널을 한 세트로 읽는 주기입니다.
constexpr uint32_t PZT_SCAN_PERIOD_US = 1000;
constexpr uint32_t MPU_SAMPLE_PERIOD_US = 5000;
// OLED 전체 버퍼 전송 중 ADC 스캔이 잠깐 멈추므로 평상시는 1초 주기,
// 상태가 바뀌었을 때는 즉시 화면을 갱신합니다.
constexpr uint32_t DISPLAY_PERIOD_MS = 1000;

constexpr uint32_t CALIBRATION_SETTLE_MS = 500;
constexpr uint32_t CALIBRATION_SAMPLE_TIME_MS = 3000;
constexpr uint32_t CALIBRATION_TIME_MS =
    CALIBRATION_SETTLE_MS + CALIBRATION_SAMPLE_TIME_MS;
constexpr uint16_t CALIBRATION_MAX_RANGE_COUNTS = 800;
constexpr uint8_t CALIBRATION_MAX_ATTEMPTS = 3;
constexpr uint8_t CALIBRATION_BAD_STRIKES = 2;
constexpr uint8_t CALIBRATION_COMMON_MOTION_CHANNELS = 3;
constexpr uint32_t PZT_SELF_TEST_CHANNEL_TIMEOUT_MS = 12000;
constexpr uint32_t PZT_SELF_TEST_RESET_STABLE_MS = 100;
// 사건 전 0.5초와 사건 후 1초를 RAM에 보관한 뒤 SD에 기록합니다.
constexpr uint32_t TRACE_PRE_MS = 500;
constexpr uint32_t CAPTURE_TIME_MS = 1000;
constexpr size_t TRACE_PRE_SAMPLES =
    (TRACE_PRE_MS * 1000UL) / PZT_SCAN_PERIOD_US;
constexpr size_t TRACE_POST_SAMPLES =
    (CAPTURE_TIME_MS * 1000UL) / PZT_SCAN_PERIOD_US + 4;
constexpr size_t TRACE_EVENT_CAPACITY =
    TRACE_PRE_SAMPLES + TRACE_POST_SAMPLES;

constexpr uint32_t REARM_MINIMUM_MS = 300;
constexpr uint32_t REARM_STABLE_MS = 100;
constexpr uint32_t REARM_TIMEOUT_MS = 1500;
constexpr uint32_t RESULT_HOLD_MS = 2000;

// ---------------------------------------------------------------------------
// 보정 전 초기 임계값
// ---------------------------------------------------------------------------

// 채널별 트리거 여유값:
// baseline + max(TRIGGER_FLOOR_COUNTS, noise * NOISE_MULTIPLIER)
constexpr float TRIGGER_FLOOR_COUNTS = 120.0f;
constexpr float NOISE_MULTIPLIER = 8.0f;
constexpr float RESET_THRESHOLD_RATIO = 0.40f;

constexpr float IMU_CONFIRM_DELTA_G = 0.20f;
constexpr float STRONG_SINGLE_CHANNEL_RATIO = 2.0f;

// 위치 계산에서 임계값 바로 위의 작은 신호를 제거하는 비율입니다.
constexpr float LOCATION_DEADBAND_RATIO = 0.50f;
constexpr float CENTER_MIN = 0.40f;
constexpr float CENTER_MAX = 0.60f;

// 2026-08-30 파일럿 36회에서 이전 3000 counts / 4 g 기준은 대부분의
// 사건을 점수 상한과 PRIORITY로 몰았습니다. ADC 전체 범위와 파일럿의
// 비포화 IMU 범위에 맞춘 임시 보정값이며, 아날로그 포화 제거 후 다시
// 검증하고 본 실험 전에 동결해야 합니다.
constexpr float PZT_NORMALIZATION_COUNTS = 4095.0f;
constexpr float IMU_NORMALIZATION_DELTA_G = 10.0f;
constexpr float INSPECTION_RECOMMENDED_SCORE = 0.55f;
constexpr float PRIORITY_INSPECTION_SCORE = 0.80f;

constexpr float STANDARD_GRAVITY = 9.80665f;

}  // namespace Config

enum class SystemState : uint8_t {
  BOOT,
  CALIBRATING,
  VERIFYING_PZT,
  ARMED,
  CAPTURING,
  COOLDOWN,
  FAULT
};

enum class InspectionLevel : uint8_t {
  IGNORED,
  RECORD_ONLY,
  INSPECTION_RECOMMENDED,
  PRIORITY_INSPECTION
};

// 실제 분류기는 시험 데이터 수집 후 확정합니다. 현재 자동 감지 사건은
// UNCLASSIFIED_IMPACT로 기록하고, 숫자 명령으로 다음 사건에 시험 라벨을
// 지정할 수 있습니다.
enum class EventType : uint8_t {
  UNCLASSIFIED_IMPACT,
  NORMAL_VIBRATION,
  LOCAL_IMPACT,
  REPEATED_SHOCK,
  SENSOR_NOISE,
  TEST_EVENT
};

enum class TimeSource : uint8_t {
  UPTIME_ONLY,
  RTC,
  BUILD_TIME_SYNC
};

enum class ImpactZone : uint8_t {
  UNKNOWN,
  CENTER,
  FRONT_LEFT,
  FRONT_RIGHT,
  REAR_LEFT,
  REAR_RIGHT
};

enum ErrorFlag : uint16_t {
  ERROR_NONE = 0,
  ERROR_OLED = 1u << 0,
  ERROR_IMU = 1u << 1,
  ERROR_SD = 1u << 2,
  ERROR_ADC_STUCK = 1u << 3,
  ERROR_ADC_BASELINE = 1u << 4,
  ERROR_NO_USABLE_PZT = 1u << 5,
  ERROR_PZT_CAL_UNSTABLE = 1u << 6,
  ERROR_PZT_UNVERIFIED = 1u << 7,
  ERROR_LOG_QUEUE_OVERFLOW = 1u << 8,
  ERROR_RTC = 1u << 9,
  ERROR_TRACE_WRITE = 1u << 10
};

struct ImpactEvent {
  uint32_t id = 0;
  uint32_t triggerMs = 0;
  uint32_t triggerUs = 0;
  uint32_t latencyMs = 0;
  uint16_t rtcYear = 0;
  uint8_t rtcMonth = 0;
  uint8_t rtcDay = 0;
  uint8_t rtcHour = 0;
  uint8_t rtcMinute = 0;
  uint8_t rtcSecond = 0;
  bool rtcValid = false;
  TimeSource timeSource = TimeSource::UPTIME_ONLY;
  uint16_t rawPeak[Config::PZT_COUNT] = {0, 0, 0, 0};
  float correctedPeak[Config::PZT_COUNT] = {0, 0, 0, 0};
  float imuPeakDeltaG = 0.0f;
  float x = 0.5f;
  float y = 0.5f;
  float locationQuality = 0.0f;
  float impactScore = 0.0f;
  uint8_t usableChannelMask = 0x0F;
  uint8_t activeChannelMask = 0;
  uint8_t saturationMask = 0;
  bool imuDataValid = false;
  bool imuSaturated = false;
  uint16_t errorFlags = ERROR_NONE;
  ImpactZone zone = ImpactZone::UNKNOWN;
  InspectionLevel inspectionLevel = InspectionLevel::IGNORED;
  EventType eventType = EventType::UNCLASSIFIED_IMPACT;
  char tracePath[Config::TRACE_PATH_SIZE] = {0};
  uint16_t traceSamples = 0;
  bool traceSaved = false;
  bool valid = false;
  bool manualCapture = false;
  bool testEvent = false;
};

struct TraceSample {
  uint32_t sampleUs = 0;
  uint16_t pzt[Config::PZT_COUNT] = {0, 0, 0, 0};
  int16_t accelMilliG[3] = {0, 0, 1000};
  uint8_t imuValid = 0;
};

struct CalibrationAccumulator {
  uint64_t pztSum[Config::PZT_COUNT] = {0, 0, 0, 0};
  uint16_t pztMin[Config::PZT_COUNT] = {
      Config::ADC_MAX_COUNTS,
      Config::ADC_MAX_COUNTS,
      Config::ADC_MAX_COUNTS,
      Config::ADC_MAX_COUNTS};
  uint16_t pztMax[Config::PZT_COUNT] = {0, 0, 0, 0};
  uint32_t pztSamples = 0;

  double accelSumX = 0.0;
  double accelSumY = 0.0;
  double accelSumZ = 0.0;
  uint32_t accelSamples = 0;
};

struct AlarmController {
  bool active = false;
  bool outputOn = false;
  uint8_t pulsesRemaining = 0;
  uint16_t onTimeMs = 0;
  uint16_t offTimeMs = 0;
  uint32_t changedAtMs = 0;
};

struct QueuedLog {
  ImpactEvent event;
  float baseline[Config::PZT_COUNT] = {0, 0, 0, 0};
  bool serialEmitted = false;
  uint32_t queuedAtMs = 0;
};

Adafruit_SSD1306 oled(
    Config::OLED_WIDTH,
    Config::OLED_HEIGHT,
    &Wire,
    Config::OLED_RESET_PIN,
    Config::I2C_CLOCK_HZ,
    Config::I2C_CLOCK_HZ);
Adafruit_MPU6050 mpu;
RTC_DS3231 rtc;

SystemState systemState = SystemState::BOOT;
uint16_t systemErrorFlags = ERROR_NONE;

bool oledReady = false;
bool imuReady = false;
bool sdReady = false;
bool rtcReady = false;
bool spiStarted = false;
uint8_t mpuAddress = 0;
TimeSource currentTimeSource = TimeSource::UPTIME_ONLY;

uint32_t sessionId = 0;
uint32_t eventCounter = 0;

uint16_t latestPztRaw[Config::PZT_COUNT] = {0, 0, 0, 0};
float pztBaseline[Config::PZT_COUNT] = {0, 0, 0, 0};
float pztNoise[Config::PZT_COUNT] = {0, 0, 0, 0};
float pztTriggerThreshold[Config::PZT_COUNT] = {0, 0, 0, 0};
float pztResetThreshold[Config::PZT_COUNT] = {0, 0, 0, 0};
uint8_t suppressedChannelMask = 0;
uint8_t disabledChannelMask = 0;

float imuReferenceX = 0.0f;
float imuReferenceY = 0.0f;
float imuReferenceZ = Config::STANDARD_GRAVITY;
float latestImuDeltaG = 0.0f;
float latestImuAccelG[3] = {0.0f, 0.0f, 1.0f};
bool latestImuSampleValid = false;
bool latestImuSaturated = false;
uint32_t lastMpuSampleUs = 0;

CalibrationAccumulator calibration;
ImpactEvent currentEvent;
ImpactEvent lastEvent;
bool lastEventAvailable = false;
bool resultVisible = false;

QueuedLog logQueue[Config::LOG_QUEUE_CAPACITY];
size_t logQueueHead = 0;
size_t logQueueCount = 0;
uint32_t logQueueDroppedCount = 0;
uint32_t nextLogServiceMs = 0;
uint32_t nextSdRetryMs = 0;

TraceSample preTriggerTrace[Config::TRACE_PRE_SAMPLES];
TraceSample eventTrace[Config::TRACE_EVENT_CAPACITY];
size_t preTriggerWriteIndex = 0;
size_t preTriggerCount = 0;
size_t eventTraceCount = 0;
EventType nextEventLabel = EventType::UNCLASSIFIED_IMPACT;

AlarmController alarmController;

uint32_t calibrationStartedMs = 0;
bool pztCalibrationValid = false;
uint8_t calibrationAttempt = 0;
uint8_t calibrationBadCount[Config::PZT_COUNT] = {0, 0, 0, 0};
uint8_t calibrationHardFaultSeenMask = 0;
uint8_t calibrationNoisySeenMask = 0;

uint8_t verifiedChannelMask = 0;
uint8_t selfTestFailedMask = 0;
uint8_t selfTestStuckMask = 0;
uint8_t selfTestTarget = 0;
bool selfTestReadyForTap = false;
bool selfTestFinalRelease = false;
uint32_t selfTestChannelStartedMs = 0;
uint32_t selfTestBelowResetStartedMs = 0;

uint32_t captureStartedMs = 0;
uint32_t cooldownStartedMs = 0;
uint32_t belowResetStartedMs = 0;
uint32_t resultVisibleUntilMs = 0;

uint32_t nextPztScanUs = 0;
uint32_t nextMpuSampleUs = 0;
uint32_t nextDisplayMs = 0;

// ---------------------------------------------------------------------------
// 함수 선언
// ---------------------------------------------------------------------------

void initializeOutputs();
void initializeAdc();
void initializeI2cDevices();
bool initializeMpuDevice();
bool initializeRtc();
bool isRtcDateTimeValid(const DateTime &dateTime);
void printRtcStatus();
void captureEventTimestamp(ImpactEvent &event);
bool readMpuRegister8(uint8_t registerAddress, uint8_t &value);
bool readMpuAcceleration(
    float &accelerationX,
    float &accelerationY,
    float &accelerationZ);
void markMpuUnavailable();
bool initializeSd();
bool ensureCsvHeader();
const char *sdCardTypeName(uint8_t cardType);

void startCalibration();
void beginCalibrationAttempt();
void finishCalibration();
void accumulatePztCalibration();
void accumulateImuCalibration(
    float accelerationX,
    float accelerationY,
    float accelerationZ);

void readPztScan();
void processPztScan(uint32_t nowMs, uint32_t nowUs);
TraceSample makeTraceSample(uint32_t nowUs);
void recordPreTriggerSample(uint32_t nowUs);
void appendEventTraceSample(uint32_t nowUs);
void copyPreTriggerTraceToEvent();
bool writeEventTraceCsv(ImpactEvent &event);
void startPztSelfTest();
void beginPztSelfTestChannel(uint32_t nowMs);
void processPztSelfTest(uint32_t nowMs);
void finishPztSelfTest();
void readMpuSample();

void beginCapture(
    uint32_t nowMs,
    uint32_t nowUs,
    bool manualCapture = false);
void updateCapturePeaks();
void finishCapture(uint32_t nowMs);
void processCooldown(uint32_t nowMs);
void enterArmedState();
void enterFaultState();

void calculateEvent(ImpactEvent &event);
void calculateLocation(ImpactEvent &event);
void calculateInspectionLevel(ImpactEvent &event);
void reportEvent(ImpactEvent &event);
void enqueueEventForLog(ImpactEvent &event);
void serviceLogQueue();
bool writeQueuedEventCsv(QueuedLog &queuedLog);
void emitQueuedEventSerial(QueuedLog &queuedLog);
void createTestEvent();

void updateDisplay(uint32_t nowMs);
void updateAlarm(uint32_t nowMs);
void startAlarm(InspectionLevel level, uint32_t nowMs);
void stopAlarm();
void setReadyIndicators();
void setInspectionIndicators(InspectionLevel level);

void handleSerialCommands();
void printHelp();
void printStatus();
void printEventSummary(const ImpactEvent &event);
bool formatEventCsv(
    const ImpactEvent &event,
    const float baseline[Config::PZT_COUNT],
    char *buffer,
    size_t bufferSize);
void formatEventTimestamp(
    const ImpactEvent &event,
    char *buffer,
    size_t bufferSize);

void setErrorFlag(uint16_t flag, bool enabled);
uint8_t calibratedChannelMask();
uint8_t liveUsableChannelMask();
uint8_t countBits(uint8_t value);
bool deadlinePending(uint32_t nowMs, uint32_t deadlineMs);
float clampFloat(float value, float minimum, float maximum);
const char *stateName(SystemState state);
const char *inspectionLevelName(InspectionLevel level);
const char *eventTypeName(EventType type);
const char *timeSourceName(TimeSource source);
const char *zoneName(ImpactZone zone);

// ---------------------------------------------------------------------------
// Arduino 기본 함수
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(Config::SERIAL_BAUD);
  delay(100);

  Serial.println();
  Serial.println("========================================");
  Serial.print("EV-BLACKBOX Firmware v");
  Serial.println(Config::FIRMWARE_VERSION);
  Serial.println("ESP32 boot started");
  Serial.println("========================================");

  sessionId =
      static_cast<uint32_t>(ESP.getEfuseMac() & 0xFFFFFFFFULL) ^
      esp_random() ^
      micros();

  initializeOutputs();
  initializeAdc();
  initializeI2cDevices();
  initializeSd();

  printHelp();
  startCalibration();

  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();
  nextPztScanUs = nowUs;
  nextMpuSampleUs = nowUs;
  nextDisplayMs = nowMs;
}

void loop() {
  // 충격 수집 중에는 시리얼 명령 처리도 미뤄 수집 지연을 줄입니다.
  if (systemState != SystemState::CAPTURING) {
    handleSerialCommands();
  }

  // 명령 처리 중 경과한 시간을 반영해 상태 시간 계산의 역전을 방지합니다.
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  if (static_cast<int32_t>(nowUs - nextPztScanUs) >= 0) {
    nextPztScanUs = nowUs + Config::PZT_SCAN_PERIOD_US;
    readPztScan();
    if (systemState == SystemState::ARMED) {
      recordPreTriggerSample(nowUs);
    } else if (systemState == SystemState::CAPTURING) {
      appendEventTraceSample(nowUs);
    }
    processPztScan(nowMs, nowUs);
  }

  if (imuReady &&
      static_cast<int32_t>(nowUs - nextMpuSampleUs) >= 0) {
    nextMpuSampleUs = nowUs + Config::MPU_SAMPLE_PERIOD_US;
    readMpuSample();
  }

  if (systemState == SystemState::CALIBRATING &&
      nowMs - calibrationStartedMs >= Config::CALIBRATION_TIME_MS) {
    finishCalibration();
    return;
  }

  if (systemState == SystemState::CAPTURING &&
      nowMs - captureStartedMs >= Config::CAPTURE_TIME_MS) {
    finishCapture(nowMs);
    return;
  }

  if (systemState == SystemState::COOLDOWN) {
    processCooldown(nowMs);
  }

  updateAlarm(nowMs);

  if (resultVisible &&
      !deadlinePending(nowMs, resultVisibleUntilMs)) {
    resultVisible = false;
    if (systemState == SystemState::ARMED) {
      setReadyIndicators();
    }
  }

  // OLED 갱신과 SD 기록은 충격 수집 중에는 실행하지 않습니다.
  if (systemState != SystemState::CAPTURING) {
    if (static_cast<int32_t>(nowMs - nextDisplayMs) >= 0) {
      nextDisplayMs = nowMs + Config::DISPLAY_PERIOD_MS;
      updateDisplay(nowMs);
    }

    const bool logServiceAllowed =
        systemState == SystemState::ARMED ||
        systemState == SystemState::COOLDOWN ||
        systemState == SystemState::FAULT;
    if (logServiceAllowed &&
        !sdReady &&
        !deadlinePending(nowMs, nextSdRetryMs)) {
      initializeSd();
    }
    if (logServiceAllowed &&
        logQueueCount > 0 &&
        sdReady &&
        !deadlinePending(nowMs, nextLogServiceMs)) {
      serviceLogQueue();
    }
  }
}

// ---------------------------------------------------------------------------
// 초기화
// ---------------------------------------------------------------------------

void initializeOutputs() {
  pinMode(Config::BUZZER_PIN, OUTPUT);
  pinMode(Config::LED_GREEN_PIN, OUTPUT);
  pinMode(Config::LED_YELLOW_PIN, OUTPUT);
  pinMode(Config::LED_RED_PIN, OUTPUT);

  stopAlarm();
  digitalWrite(Config::LED_GREEN_PIN, LOW);
  digitalWrite(Config::LED_YELLOW_PIN, LOW);
  digitalWrite(Config::LED_RED_PIN, LOW);
}

void initializeAdc() {
  analogReadResolution(Config::ADC_BITS);

  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    pinMode(Config::PZT_PINS[i], INPUT);
    analogSetPinAttenuation(Config::PZT_PINS[i], ADC_11db);
  }

  Serial.println("[OK] ADC1 GPIO32/33/34/35, 12-bit, 11 dB");
}

void initializeI2cDevices() {
  Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN);
  Wire.setTimeOut(Config::I2C_TIMEOUT_MS);
  Wire.setClock(Config::I2C_CLOCK_HZ);

  oledReady = oled.begin(
      SSD1306_SWITCHCAPVCC,
      Config::OLED_ADDRESS,
      true,
      false);

  if (oledReady) {
    setErrorFlag(ERROR_OLED, false);
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setTextWrap(false);
    oled.setCursor(0, 0);
    oled.println("EV-BLACKBOX");
    oled.println("BOOTING...");
    oled.display();
    Serial.println("[OK] OLED 0x3C");
  } else {
    setErrorFlag(ERROR_OLED, true);
    Serial.println("[WARN] OLED not found; Serial operation continues");
  }

  if (initializeMpuDevice()) {
    setErrorFlag(ERROR_IMU, false);
    Serial.printf("[OK] MPU6050 at 0x%02X\n", mpuAddress);
  } else {
    setErrorFlag(ERROR_IMU, true);
    Serial.println("[WARN] MPU6050 not found; PZT-only mode enabled");
  }

  if (initializeRtc()) {
    Serial.printf(
        "[OK] DS3231 RTC 0x%02X time_source=%s\n",
        Config::RTC_ADDRESS,
        timeSourceName(currentTimeSource));
  }

  // 각 라이브러리 초기화가 Wire 설정을 바꿨을 가능성에 대비합니다.
  Wire.setClock(Config::I2C_CLOCK_HZ);
}

bool initializeMpuDevice() {
  imuReady = false;
  mpuAddress = 0;
  latestImuSampleValid = false;
  latestImuSaturated = false;
  latestImuDeltaG = 0.0f;
  latestImuAccelG[0] = 0.0f;
  latestImuAccelG[1] = 0.0f;
  latestImuAccelG[2] = 1.0f;
  lastMpuSampleUs = 0;

  if (mpu.begin(Config::MPU_ADDRESS_PRIMARY, &Wire)) {
    mpuAddress = Config::MPU_ADDRESS_PRIMARY;
  } else if (mpu.begin(Config::MPU_ADDRESS_SECONDARY, &Wire)) {
    mpuAddress = Config::MPU_ADDRESS_SECONDARY;
  } else {
    Wire.setClock(Config::I2C_CLOCK_HZ);
    return false;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  Wire.setClock(Config::I2C_CLOCK_HZ);

  bool rangeVerified = false;
  uint8_t accelConfig = 0;
  for (uint8_t attempt = 0;
       attempt < Config::MPU_CONFIG_VERIFY_ATTEMPTS;
       ++attempt) {
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    delay(2);
    if (readMpuRegister8(
            Config::MPU_ACCEL_CONFIG_REG,
            accelConfig) &&
        accelConfig == Config::MPU_ACCEL_CONFIG_16G) {
      rangeVerified = true;
      break;
    }
  }

  if (!rangeVerified) {
    Serial.println(
        "[WARN] MPU6050 +/-16g configuration verify failed");
    mpuAddress = 0;
    Wire.setClock(Config::I2C_CLOCK_HZ);
    return false;
  }

  imuReady = true;
  setErrorFlag(ERROR_IMU, false);
  return true;
}

bool initializeRtc() {
  rtcReady = false;
  currentTimeSource = TimeSource::UPTIME_ONLY;

  // MPU6050이 아직 0x68이면 DS3231과 주소가 충돌합니다. RTClib은 0x68에
  // 응답하는 장치의 종류를 구분하지 못하므로 이 상태에서는 RTC 접근을
  // 시도하지 않습니다.
  if (imuReady && mpuAddress == Config::RTC_ADDRESS) {
    setErrorFlag(ERROR_RTC, true);
    Serial.println(
        "[WARN] RTC disabled: MPU6050 also uses 0x68. "
        "Connect MPU6050 AD0 to 3.3V (0x69), then reboot.");
    return false;
  }

  if (!rtc.begin(&Wire)) {
    setErrorFlag(ERROR_RTC, true);
    Serial.println("[WARN] DS3231 not found; uptime-only logging enabled");
    return false;
  }

  rtcReady = true;
  bool adjustedFromBuild = rtc.lostPower();
  DateTime now = rtc.now();
  if (!isRtcDateTimeValid(now)) {
    adjustedFromBuild = true;
  }

  if (adjustedFromBuild) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    currentTimeSource = TimeSource::BUILD_TIME_SYNC;
    Serial.println(
        "[WARN] RTC lost power; synchronized once from firmware build time");
  } else {
    currentTimeSource = TimeSource::RTC;
  }

  rtc.writeSqwPinMode(DS3231_OFF);
  setErrorFlag(ERROR_RTC, false);
  return true;
}

bool isRtcDateTimeValid(const DateTime &dateTime) {
  return dateTime.isValid() &&
      dateTime.year() >= 2024 &&
      dateTime.year() <= 2099;
}

void printRtcStatus() {
  if (!rtcReady) {
    Serial.println("[RTC] Not ready; retrying initialization");
    if (!initializeRtc()) {
      Serial.println(
          "[RTC] Unavailable. Check SDA/SCL and keep MPU6050 at 0x69.");
      return;
    }
  }

  const DateTime now = rtc.now();
  if (!isRtcDateTimeValid(now)) {
    setErrorFlag(ERROR_RTC, true);
    rtcReady = false;
    currentTimeSource = TimeSource::UPTIME_ONLY;
    Serial.println("[RTC] Invalid date/time received; RTC marked unavailable");
    return;
  }

  char timestamp[26];
  snprintf(
      timestamp,
      sizeof(timestamp),
      "%04u-%02u-%02u %02u:%02u:%02u",
      static_cast<unsigned>(now.year()),
      static_cast<unsigned>(now.month()),
      static_cast<unsigned>(now.day()),
      static_cast<unsigned>(now.hour()),
      static_cast<unsigned>(now.minute()),
      static_cast<unsigned>(now.second()));
  Serial.printf(
      "[RTC] %s source=%s\n",
      timestamp,
      timeSourceName(currentTimeSource));
}

void captureEventTimestamp(ImpactEvent &event) {
  event.rtcValid = false;
  event.timeSource = TimeSource::UPTIME_ONLY;

  if (!rtcReady) {
    return;
  }

  const DateTime now = rtc.now();
  if (!isRtcDateTimeValid(now)) {
    setErrorFlag(ERROR_RTC, true);
    rtcReady = false;
    currentTimeSource = TimeSource::UPTIME_ONLY;
    return;
  }

  event.rtcYear = now.year();
  event.rtcMonth = now.month();
  event.rtcDay = now.day();
  event.rtcHour = now.hour();
  event.rtcMinute = now.minute();
  event.rtcSecond = now.second();
  event.rtcValid = true;
  event.timeSource = currentTimeSource;
}

bool readMpuRegister8(
    uint8_t registerAddress,
    uint8_t &value) {
  if (mpuAddress == 0) {
    return false;
  }

  Wire.beginTransmission(mpuAddress);
  if (Wire.write(registerAddress) != 1) {
    Wire.endTransmission(true);
    return false;
  }
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t received = Wire.requestFrom(
      mpuAddress,
      static_cast<size_t>(1),
      true);
  if (received != 1 || Wire.available() < 1) {
    while (Wire.available() > 0) {
      Wire.read();
    }
    return false;
  }

  value = static_cast<uint8_t>(Wire.read());
  return true;
}

bool readMpuAcceleration(
    float &accelerationX,
    float &accelerationY,
    float &accelerationZ) {
  if (!imuReady || mpuAddress == 0) {
    return false;
  }

  Wire.beginTransmission(mpuAddress);
  if (Wire.write(Config::MPU_ACCEL_XOUT_H_REG) != 1) {
    Wire.endTransmission(true);
    return false;
  }
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t received = Wire.requestFrom(
      mpuAddress,
      Config::MPU_ACCEL_BYTES,
      true);
  if (received != Config::MPU_ACCEL_BYTES ||
      Wire.available() < static_cast<int>(Config::MPU_ACCEL_BYTES)) {
    while (Wire.available() > 0) {
      Wire.read();
    }
    return false;
  }

  const auto readSigned16 = []() -> int16_t {
    const uint16_t highByte = static_cast<uint8_t>(Wire.read());
    const uint16_t lowByte = static_cast<uint8_t>(Wire.read());
    return static_cast<int16_t>((highByte << 8) | lowByte);
  };

  const int16_t rawX = readSigned16();
  const int16_t rawY = readSigned16();
  const int16_t rawZ = readSigned16();
  const float scale =
      Config::STANDARD_GRAVITY / Config::MPU_ACCEL_LSB_PER_G;

  accelerationX = static_cast<float>(rawX) * scale;
  accelerationY = static_cast<float>(rawY) * scale;
  accelerationZ = static_cast<float>(rawZ) * scale;

  return isfinite(accelerationX) &&
         isfinite(accelerationY) &&
         isfinite(accelerationZ);
}

void markMpuUnavailable() {
  if (imuReady) {
    Serial.println(
        "[WARN] MPU6050 communication lost; PZT-only mode enabled. "
        "Reconnect it and enter c.");
  }

  imuReady = false;
  latestImuSampleValid = false;
  latestImuSaturated = false;
  latestImuDeltaG = 0.0f;
  latestImuAccelG[0] = 0.0f;
  latestImuAccelG[1] = 0.0f;
  latestImuAccelG[2] = 1.0f;
  lastMpuSampleUs = 0;
  setErrorFlag(ERROR_IMU, true);
}

bool initializeSd() {
  nextSdRetryMs = millis() + Config::SD_RETRY_PERIOD_MS;
  pinMode(Config::SD_CS_PIN, OUTPUT);
  digitalWrite(Config::SD_CS_PIN, HIGH);

  if (spiStarted) {
    SD.end();
  } else {
    SPI.begin(
        Config::SD_SCK_PIN,
        Config::SD_MISO_PIN,
        Config::SD_MOSI_PIN,
        Config::SD_CS_PIN);
    spiStarted = true;
  }

  sdReady = SD.begin(
      Config::SD_CS_PIN,
      SPI,
      Config::SD_SPI_HZ);

  if (!sdReady || SD.cardType() == CARD_NONE) {
    sdReady = false;
    setErrorFlag(ERROR_SD, true);
    Serial.println("[WARN] microSD not available; Serial CSV backup enabled");
    return false;
  }

  if (!ensureCsvHeader()) {
    sdReady = false;
    setErrorFlag(ERROR_SD, true);
    Serial.println("[WARN] microSD log file open failed");
    return false;
  }

  setErrorFlag(ERROR_SD, false);
  Serial.printf(
      "[OK] microSD type=%s size=%llu MB log=%s\n",
      sdCardTypeName(SD.cardType()),
      static_cast<unsigned long long>(SD.cardSize() / (1024ULL * 1024ULL)),
      Config::SD_LOG_PATH);
  return true;
}

const char *sdCardTypeName(uint8_t cardType) {
  switch (cardType) {
    case CARD_MMC:
      return "MMC";
    case CARD_SD:
      return "SDSC";
    case CARD_SDHC:
      return "SDHC/SDXC";
    case CARD_NONE:
      return "NONE";
    default:
      return "UNKNOWN";
  }
}

bool ensureCsvHeader() {
  const size_t headerLength =
      sizeof(Config::CSV_HEADER) - 1;
  const size_t expectedHeaderWrite = headerLength + 2;

  if (SD.exists(Config::SD_LOG_PATH)) {
    File existing = SD.open(Config::SD_LOG_PATH, FILE_READ);
    if (!existing) {
      return false;
    }

    const size_t fileSize = existing.size();
    if (fileSize > 0) {
      char firstLine[sizeof(Config::CSV_HEADER) + 2];
      const size_t readLength = existing.readBytesUntil(
          '\n',
          firstLine,
          sizeof(firstLine) - 1);
      firstLine[readLength] = '\0';
      if (readLength > 0 &&
          firstLine[readLength - 1] == '\r') {
        firstLine[readLength - 1] = '\0';
      }

      const bool headerValid =
          strcmp(firstLine, Config::CSV_HEADER) == 0;
      existing.seek(fileSize - 1);
      const int lastByte = existing.read();
      existing.close();

      if (!headerValid) {
        const size_t prefixLength = strlen(firstLine);
        const bool incompleteHeaderPrefix =
            lastByte != '\n' &&
            prefixLength > 0 &&
            prefixLength < headerLength &&
            fileSize <= headerLength + 1 &&
            strncmp(
                Config::CSV_HEADER,
                firstLine,
                prefixLength) == 0;
        if (incompleteHeaderPrefix &&
            SD.remove(Config::SD_LOG_PATH)) {
          File replacement =
              SD.open(Config::SD_LOG_PATH, FILE_APPEND);
          if (!replacement) {
            return false;
          }
          const size_t rewritten =
              replacement.println(Config::CSV_HEADER);
          replacement.flush();
          replacement.close();
          if (rewritten == expectedHeaderWrite) {
            Serial.println("[WARN] Incomplete CSV header recreated");
            return true;
          }
          SD.remove(Config::SD_LOG_PATH);
        }
        Serial.println("[WARN] CSV header is incomplete or incompatible");
        return false;
      }

      if (lastByte != '\n') {
        File repair = SD.open(Config::SD_LOG_PATH, FILE_APPEND);
        if (!repair) {
          return false;
        }
        const char *terminator =
            lastByte == '\r' ? "\n" : "\r\n";
        const size_t expectedRepair =
            lastByte == '\r' ? 1 : 2;
        const size_t repaired = repair.print(terminator);
        repair.flush();
        repair.close();
        if (repaired != expectedRepair) {
          return false;
        }
        Serial.println("[WARN] Incomplete CSV tail isolated before retry");
      }

      return true;
    }

    existing.close();
  }

  File file = SD.open(Config::SD_LOG_PATH, FILE_APPEND);
  if (!file) {
    return false;
  }

  const size_t written = file.println(Config::CSV_HEADER);
  file.flush();
  file.close();
  if (written != expectedHeaderWrite) {
    SD.remove(Config::SD_LOG_PATH);
    return false;
  }

  return true;
}

// ---------------------------------------------------------------------------
// 자동 기준값 보정
// ---------------------------------------------------------------------------

void startCalibration() {
  pztCalibrationValid = false;
  calibrationAttempt = 1;
  memset(calibrationBadCount, 0, sizeof(calibrationBadCount));
  calibrationHardFaultSeenMask = 0;
  calibrationNoisySeenMask = 0;
  suppressedChannelMask = 0;
  disabledChannelMask = 0;
  verifiedChannelMask = 0;
  selfTestFailedMask = 0;
  selfTestStuckMask = 0;
  setErrorFlag(ERROR_ADC_STUCK, false);
  setErrorFlag(ERROR_ADC_BASELINE, false);
  setErrorFlag(ERROR_NO_USABLE_PZT, false);
  setErrorFlag(ERROR_PZT_CAL_UNSTABLE, false);
  setErrorFlag(ERROR_PZT_UNVERIFIED, true);

  beginCalibrationAttempt();
}

void beginCalibrationAttempt() {
  calibration = CalibrationAccumulator();
  calibrationStartedMs = millis();
  latestImuSampleValid = false;
  latestImuSaturated = false;
  lastMpuSampleUs = 0;

  resultVisible = false;
  stopAlarm();
  digitalWrite(Config::LED_GREEN_PIN, LOW);
  digitalWrite(Config::LED_YELLOW_PIN, LOW);
  digitalWrite(Config::LED_RED_PIN, LOW);

  systemState = SystemState::CALIBRATING;

  Serial.println();
  nextDisplayMs = millis();
  Serial.printf(
      "[CAL] Attempt %u/%u, about 4 seconds: do not touch the plate\n",
      calibrationAttempt,
      Config::CALIBRATION_MAX_ATTEMPTS);
}

void accumulatePztCalibration() {
  if (millis() - calibrationStartedMs <
      Config::CALIBRATION_SETTLE_MS) {
    return;
  }

  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    const uint16_t value = latestPztRaw[i];
    calibration.pztSum[i] += value;

    if (value < calibration.pztMin[i]) {
      calibration.pztMin[i] = value;
    }
    if (value > calibration.pztMax[i]) {
      calibration.pztMax[i] = value;
    }
  }

  ++calibration.pztSamples;
}

void accumulateImuCalibration(
    float accelerationX,
    float accelerationY,
    float accelerationZ) {
  if (millis() - calibrationStartedMs <
      Config::CALIBRATION_SETTLE_MS) {
    return;
  }

  calibration.accelSumX += accelerationX;
  calibration.accelSumY += accelerationY;
  calibration.accelSumZ += accelerationZ;
  ++calibration.accelSamples;
}

void finishCalibration() {
  if (calibration.pztSamples == 0) {
    Serial.println("[ERROR] No PZT calibration samples");
    if (calibrationAttempt < Config::CALIBRATION_MAX_ATTEMPTS) {
      ++calibrationAttempt;
      beginCalibrationAttempt();
    } else {
      setErrorFlag(ERROR_PZT_CAL_UNSTABLE, true);
      setErrorFlag(ERROR_NO_USABLE_PZT, true);
      Serial.println("[FAULT] Calibration failed after maximum attempts");
      enterFaultState();
    }
    return;
  }

  float candidateBaseline[Config::PZT_COUNT] = {0, 0, 0, 0};
  float candidateNoise[Config::PZT_COUNT] = {0, 0, 0, 0};
  float candidateTrigger[Config::PZT_COUNT] = {0, 0, 0, 0};
  float candidateReset[Config::PZT_COUNT] = {0, 0, 0, 0};
  uint8_t noisyMask = 0;
  uint8_t hardFaultMask = 0;

  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1u << i);
    const uint16_t calibrationRange =
        calibration.pztMax[i] - calibration.pztMin[i];

    candidateBaseline[i] =
        static_cast<float>(calibration.pztSum[i]) /
        static_cast<float>(calibration.pztSamples);

    const float upperNoise =
        static_cast<float>(calibration.pztMax[i]) -
        candidateBaseline[i];
    const float lowerNoise =
        candidateBaseline[i] -
        static_cast<float>(calibration.pztMin[i]);
    candidateNoise[i] =
        upperNoise > lowerNoise ? upperNoise : lowerNoise;

    const float dynamicMargin =
        candidateNoise[i] * Config::NOISE_MULTIPLIER;
    const float triggerMargin =
        dynamicMargin > Config::TRIGGER_FLOOR_COUNTS
            ? dynamicMargin
            : Config::TRIGGER_FLOOR_COUNTS;

    candidateTrigger[i] =
        candidateBaseline[i] + triggerMargin;
    if (candidateBaseline[i] >= 3800.0f ||
        candidateTrigger[i] >=
            static_cast<float>(Config::ADC_SATURATION_COUNTS - 1)) {
      hardFaultMask |= bit;
    }
    if (candidateTrigger[i] >
        static_cast<float>(Config::ADC_SATURATION_COUNTS - 1)) {
      candidateTrigger[i] =
          static_cast<float>(Config::ADC_SATURATION_COUNTS - 1);
    }

    candidateReset[i] =
        candidateBaseline[i] +
        triggerMargin * Config::RESET_THRESHOLD_RATIO;
    if (candidateReset[i] >= candidateTrigger[i]) {
      candidateReset[i] = candidateTrigger[i] - 1.0f;
    }
    if (candidateReset[i] < candidateBaseline[i]) {
      candidateReset[i] = candidateBaseline[i];
    }

    if (calibrationRange > Config::CALIBRATION_MAX_RANGE_COUNTS) {
      noisyMask |= bit;
      Serial.printf(
          "[CAL] PZT%u moved/noisy: range=%u\n",
          static_cast<unsigned>(i + 1),
          calibrationRange);
    }
  }

  if (countBits(noisyMask) >=
      Config::CALIBRATION_COMMON_MOTION_CHANNELS) {
    Serial.println(
        "[CAL] Plate movement detected on most channels");
    if (calibrationAttempt < Config::CALIBRATION_MAX_ATTEMPTS) {
      ++calibrationAttempt;
      Serial.println("[CAL] Retrying without isolating individual channels");
      beginCalibrationAttempt();
    } else {
      setErrorFlag(ERROR_PZT_CAL_UNSTABLE, true);
      setErrorFlag(ERROR_NO_USABLE_PZT, true);
      Serial.println(
          "[FAULT] Plate did not remain still during calibration; enter c");
      enterFaultState();
    }
    return;
  }

  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1u << i);
    if (((noisyMask | hardFaultMask) & bit) != 0 &&
        calibrationBadCount[i] < UINT8_MAX) {
      ++calibrationBadCount[i];
    }
  }

  calibrationNoisySeenMask |= noisyMask;
  calibrationHardFaultSeenMask |= hardFaultMask;

  const uint8_t currentFaultMask =
      static_cast<uint8_t>(noisyMask | hardFaultMask);
  if (currentFaultMask != 0 &&
      calibrationAttempt < Config::CALIBRATION_MAX_ATTEMPTS) {
    ++calibrationAttempt;
    Serial.printf(
        "[CAL] Channel fault mask=0x%02X; retrying before isolation\n",
        currentFaultMask);
    beginCalibrationAttempt();
    return;
  }

  uint8_t repeatedFaultMask = 0;
  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    if (calibrationBadCount[i] >=
        Config::CALIBRATION_BAD_STRIKES) {
      repeatedFaultMask |= static_cast<uint8_t>(1u << i);
    }
  }

  disabledChannelMask =
      static_cast<uint8_t>(
          currentFaultMask | repeatedFaultMask) &
      Config::PZT_ALL_CHANNELS_MASK;
  setErrorFlag(
      ERROR_ADC_BASELINE,
      (disabledChannelMask & calibrationHardFaultSeenMask) != 0);
  setErrorFlag(
      ERROR_PZT_CAL_UNSTABLE,
      (disabledChannelMask & calibrationNoisySeenMask) != 0);

  Serial.println("[CAL] PZT result");
  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    pztBaseline[i] = candidateBaseline[i];
    pztNoise[i] = candidateNoise[i];
    pztTriggerThreshold[i] = candidateTrigger[i];
    pztResetThreshold[i] = candidateReset[i];

    Serial.printf(
        "  PZT%u baseline=%.1f noise=%.1f trigger=%.1f reset=%.1f %s\n",
        static_cast<unsigned>(i + 1),
        pztBaseline[i],
        pztNoise[i],
        pztTriggerThreshold[i],
        pztResetThreshold[i],
        (disabledChannelMask & static_cast<uint8_t>(1u << i)) != 0
            ? "ISOLATED"
            : "CAL-OK");
  }

  if (disabledChannelMask != 0) {
    Serial.printf(
        "[WARN] Disabled PZT channel mask=0x%02X\n",
        disabledChannelMask);
  }

  const uint8_t usablePztMask =
      static_cast<uint8_t>(~disabledChannelMask) &
      Config::PZT_ALL_CHANNELS_MASK;
  const uint8_t usablePztCount = countBits(usablePztMask);
  setErrorFlag(ERROR_NO_USABLE_PZT, usablePztCount == 0);

  if (imuReady && calibration.accelSamples > 0) {
    imuReferenceX =
        static_cast<float>(
            calibration.accelSumX /
            static_cast<double>(calibration.accelSamples));
    imuReferenceY =
        static_cast<float>(
            calibration.accelSumY /
            static_cast<double>(calibration.accelSamples));
    imuReferenceZ =
        static_cast<float>(
            calibration.accelSumZ /
            static_cast<double>(calibration.accelSamples));

    Serial.printf(
        "[CAL] MPU reference: %.3f, %.3f, %.3f m/s^2\n",
        imuReferenceX,
        imuReferenceY,
        imuReferenceZ);
  }

  pztCalibrationValid = true;
  latestImuDeltaG = 0.0f;

  if (usablePztCount == 0) {
    Serial.println(
        "[FAULT] No usable PZT channel; impact detection is NOT armed");
    Serial.println("[FAULT] Check the PZT wiring/circuit and enter c");
    enterFaultState();
    return;
  }

  startPztSelfTest();
}

// ---------------------------------------------------------------------------
// 센서 읽기
// ---------------------------------------------------------------------------

void readPztScan() {
  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    // ADC 멀티플렉서 채널 전환 직후의 첫 값을 버리고 두 번째 값을 사용합니다.
    analogRead(Config::PZT_PINS[i]);
    latestPztRaw[i] =
        static_cast<uint16_t>(analogRead(Config::PZT_PINS[i]));
  }
}

TraceSample makeTraceSample(uint32_t nowUs) {
  TraceSample sample;
  sample.sampleUs = nowUs;
  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    sample.pzt[i] = latestPztRaw[i];
  }

  for (size_t axis = 0; axis < 3; ++axis) {
    const float milliG = clampFloat(
        latestImuAccelG[axis] * 1000.0f,
        -32768.0f,
        32767.0f);
    sample.accelMilliG[axis] = static_cast<int16_t>(lroundf(milliG));
  }
  sample.imuValid = latestImuSampleValid ? 1u : 0u;
  return sample;
}

void recordPreTriggerSample(uint32_t nowUs) {
  preTriggerTrace[preTriggerWriteIndex] = makeTraceSample(nowUs);
  preTriggerWriteIndex =
      (preTriggerWriteIndex + 1) % Config::TRACE_PRE_SAMPLES;
  if (preTriggerCount < Config::TRACE_PRE_SAMPLES) {
    ++preTriggerCount;
  }
}

void appendEventTraceSample(uint32_t nowUs) {
  if (eventTraceCount >= Config::TRACE_EVENT_CAPACITY) {
    return;
  }
  eventTrace[eventTraceCount++] = makeTraceSample(nowUs);
}

void copyPreTriggerTraceToEvent() {
  eventTraceCount = 0;
  if (preTriggerCount == 0) {
    return;
  }

  const size_t start =
      (preTriggerWriteIndex + Config::TRACE_PRE_SAMPLES - preTriggerCount) %
      Config::TRACE_PRE_SAMPLES;
  for (size_t i = 0;
       i < preTriggerCount && eventTraceCount < Config::TRACE_EVENT_CAPACITY;
       ++i) {
    const size_t index = (start + i) % Config::TRACE_PRE_SAMPLES;
    eventTrace[eventTraceCount++] = preTriggerTrace[index];
  }
}

bool writeEventTraceCsv(ImpactEvent &event) {
  event.traceSaved = false;
  event.traceSamples = static_cast<uint16_t>(
      eventTraceCount > UINT16_MAX ? UINT16_MAX : eventTraceCount);
  event.tracePath[0] = '\0';

  if (event.testEvent || eventTraceCount == 0 || !sdReady) {
    return false;
  }

  char path[Config::TRACE_PATH_SIZE];
  const int pathLength = snprintf(
      path,
      sizeof(path),
      "/tr_%08lX_%06lu.csv",
      static_cast<unsigned long>(sessionId),
      static_cast<unsigned long>(event.id));
  if (pathLength <= 0 ||
      static_cast<size_t>(pathLength) >= sizeof(path)) {
    setErrorFlag(ERROR_TRACE_WRITE, true);
    event.errorFlags |= ERROR_TRACE_WRITE;
    return false;
  }

  if (SD.exists(path)) {
    SD.remove(path);
  }

  File file = SD.open(path, FILE_WRITE);
  if (!file) {
    setErrorFlag(ERROR_TRACE_WRITE, true);
    setErrorFlag(ERROR_SD, true);
    event.errorFlags |= ERROR_TRACE_WRITE | ERROR_SD;
    sdReady = false;
    return false;
  }

  bool writeOk = file.println(Config::TRACE_HEADER) > 0;
  char line[160];
  for (size_t i = 0; i < eventTraceCount && writeOk; ++i) {
    const TraceSample &sample = eventTrace[i];
    const int32_t relativeUs =
        static_cast<int32_t>(sample.sampleUs - event.triggerUs);
    const int written = snprintf(
        line,
        sizeof(line),
        "%ld,%u,%u,%u,%u,%.3f,%.3f,%.3f,%u",
        static_cast<long>(relativeUs),
        sample.pzt[0],
        sample.pzt[1],
        sample.pzt[2],
        sample.pzt[3],
        static_cast<float>(sample.accelMilliG[0]) / 1000.0f,
        static_cast<float>(sample.accelMilliG[1]) / 1000.0f,
        static_cast<float>(sample.accelMilliG[2]) / 1000.0f,
        static_cast<unsigned>(sample.imuValid));
    if (written <= 0 || static_cast<size_t>(written) >= sizeof(line)) {
      writeOk = false;
      break;
    }
    const size_t output = file.println(line);
    if (output == 0) {
      writeOk = false;
    }
  }

  file.flush();
  file.close();

  if (!writeOk) {
    SD.remove(path);
    setErrorFlag(ERROR_TRACE_WRITE, true);
    setErrorFlag(ERROR_SD, true);
    event.errorFlags |= ERROR_TRACE_WRITE | ERROR_SD;
    sdReady = false;
    return false;
  }

  strncpy(event.tracePath, path, sizeof(event.tracePath) - 1);
  event.tracePath[sizeof(event.tracePath) - 1] = '\0';
  event.traceSaved = true;
  setErrorFlag(ERROR_TRACE_WRITE, false);
  Serial.printf(
      "[OK] Event trace: %s (%u samples)\n",
      event.tracePath,
      static_cast<unsigned>(event.traceSamples));
  return true;
}

void processPztScan(uint32_t nowMs, uint32_t nowUs) {
  if (systemState == SystemState::CALIBRATING) {
    accumulatePztCalibration();
    return;
  }

  if (systemState == SystemState::VERIFYING_PZT) {
    processPztSelfTest(nowMs);
    return;
  }

  if (systemState == SystemState::CAPTURING) {
    updateCapturePeaks();
    return;
  }

  if (systemState != SystemState::ARMED) {
    return;
  }

  // 강제로 억제된 채널은 값이 정상 범위로 복귀하면 자동 복구합니다.
  const uint8_t previousSuppressedMask = suppressedChannelMask;
  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1u << i);
    if ((suppressedChannelMask & bit) != 0 &&
        latestPztRaw[i] <= pztResetThreshold[i]) {
      suppressedChannelMask &= static_cast<uint8_t>(~bit);
    }
  }

  if (suppressedChannelMask == 0 &&
      selfTestStuckMask == 0) {
    setErrorFlag(ERROR_ADC_STUCK, false);
  }
  if (suppressedChannelMask != previousSuppressedMask &&
      !resultVisible) {
    setReadyIndicators();
  }

  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1u << i);
    if ((liveUsableChannelMask() & bit) == 0) {
      continue;
    }

    if (latestPztRaw[i] >= pztTriggerThreshold[i]) {
      beginCapture(nowMs, nowUs);
      return;
    }
  }
}

void startPztSelfTest() {
  if (!pztCalibrationValid ||
      calibratedChannelMask() == 0) {
    setErrorFlag(ERROR_NO_USABLE_PZT, true);
    Serial.println(
        "[FAULT] Valid PZT calibration required before self-test; enter c");
    enterFaultState();
    return;
  }

  verifiedChannelMask = 0;
  selfTestFailedMask = 0;
  selfTestStuckMask = 0;
  selfTestTarget = 0;
  selfTestReadyForTap = false;
  selfTestFinalRelease = false;
  selfTestChannelStartedMs = 0;
  selfTestBelowResetStartedMs = 0;
  suppressedChannelMask = 0;
  setErrorFlag(ERROR_ADC_STUCK, false);
  setErrorFlag(ERROR_PZT_UNVERIFIED, true);

  resultVisible = false;
  stopAlarm();
  systemState = SystemState::VERIFYING_PZT;
  digitalWrite(Config::LED_GREEN_PIN, LOW);
  digitalWrite(Config::LED_YELLOW_PIN, HIGH);
  digitalWrite(Config::LED_RED_PIN, LOW);

  Serial.println();
  Serial.println("[TEST] PZT response self-test");
  Serial.println(
      "[TEST] Tap once near each requested sensor; do not tap other areas");
  beginPztSelfTestChannel(millis());
}

void beginPztSelfTestChannel(uint32_t nowMs) {
  const uint8_t availableMask = calibratedChannelMask();
  while (selfTestTarget < Config::PZT_COUNT &&
         (availableMask &
          static_cast<uint8_t>(1u << selfTestTarget)) == 0) {
    ++selfTestTarget;
  }

  if (selfTestTarget >= Config::PZT_COUNT) {
    selfTestFinalRelease = true;
    selfTestReadyForTap = false;
    selfTestChannelStartedMs = nowMs;
    selfTestBelowResetStartedMs = 0;
    nextDisplayMs = nowMs;
    Serial.println(
        "[TEST] Release the plate; waiting for all PZT channels to reset");
    return;
  }

  selfTestReadyForTap = false;
  selfTestChannelStartedMs = nowMs;
  selfTestBelowResetStartedMs = 0;
  nextDisplayMs = nowMs;

  static const char *const labels[Config::PZT_COUNT] = {
      "P1 FRONT-LEFT",
      "P2 FRONT-RIGHT",
      "P3 REAR-LEFT",
      "P4 REAR-RIGHT"};
  Serial.printf(
      "[TEST] Wait for reset, then TAP %s (timeout %lu s)\n",
      labels[selfTestTarget],
      static_cast<unsigned long>(
          Config::PZT_SELF_TEST_CHANNEL_TIMEOUT_MS / 1000));
}

void processPztSelfTest(uint32_t nowMs) {
  if (selfTestFinalRelease) {
    bool allBelowReset = true;
    uint8_t stuckMask = 0;
    for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
      const uint8_t bit = static_cast<uint8_t>(1u << i);
      if ((verifiedChannelMask & bit) == 0) {
        continue;
      }
      if (latestPztRaw[i] > pztResetThreshold[i]) {
        allBelowReset = false;
        stuckMask |= bit;
      }
    }

    if (allBelowReset) {
      if (selfTestBelowResetStartedMs == 0) {
        selfTestBelowResetStartedMs = nowMs;
      } else if (
          nowMs - selfTestBelowResetStartedMs >=
          Config::PZT_SELF_TEST_RESET_STABLE_MS) {
        selfTestFinalRelease = false;
        finishPztSelfTest();
      }
    } else {
      selfTestBelowResetStartedMs = 0;
    }

    if (selfTestFinalRelease &&
        !allBelowReset &&
        nowMs - selfTestChannelStartedMs >=
            Config::PZT_SELF_TEST_CHANNEL_TIMEOUT_MS) {
      selfTestFailedMask |= stuckMask;
      selfTestStuckMask |= stuckMask;
      verifiedChannelMask &=
          static_cast<uint8_t>(~stuckMask);
      setErrorFlag(ERROR_ADC_STUCK, stuckMask != 0);
      Serial.printf(
          "[TEST] Reset timeout; stuck PZT mask=0x%02X\n",
          stuckMask);
      selfTestBelowResetStartedMs = 0;
      selfTestChannelStartedMs = nowMs;
      if (verifiedChannelMask == 0) {
        selfTestFinalRelease = false;
        finishPztSelfTest();
      } else {
        Serial.println(
            "[TEST] Verifying 100 ms stable reset on remaining channels");
      }
    }
    return;
  }

  if (selfTestTarget >= Config::PZT_COUNT) {
    finishPztSelfTest();
    return;
  }

  const uint8_t bit =
      static_cast<uint8_t>(1u << selfTestTarget);

  if (nowMs - selfTestChannelStartedMs >=
      Config::PZT_SELF_TEST_CHANNEL_TIMEOUT_MS) {
    selfTestFailedMask |= bit;
    Serial.printf(
        "[TEST] PZT%u FAIL: no verified response\n",
        static_cast<unsigned>(selfTestTarget + 1));
    ++selfTestTarget;
    beginPztSelfTestChannel(nowMs);
    return;
  }

  const uint16_t raw = latestPztRaw[selfTestTarget];
  if (!selfTestReadyForTap) {
    if (raw <= pztResetThreshold[selfTestTarget]) {
      if (selfTestBelowResetStartedMs == 0) {
        selfTestBelowResetStartedMs = nowMs;
      } else if (
          nowMs - selfTestBelowResetStartedMs >=
          Config::PZT_SELF_TEST_RESET_STABLE_MS) {
        selfTestReadyForTap = true;
        Serial.printf(
            "[TEST] TAP PZT%u NOW\n",
            static_cast<unsigned>(selfTestTarget + 1));
        nextDisplayMs = nowMs;
      }
    } else {
      selfTestBelowResetStartedMs = 0;
    }
    return;
  }

  if (raw >= pztTriggerThreshold[selfTestTarget]) {
    verifiedChannelMask |= bit;
    Serial.printf(
        "[TEST] PZT%u PASS: raw=%u threshold=%.0f\n",
        static_cast<unsigned>(selfTestTarget + 1),
        raw,
        pztTriggerThreshold[selfTestTarget]);
    ++selfTestTarget;
    beginPztSelfTestChannel(nowMs);
  }
}

void finishPztSelfTest() {
  const uint8_t calibratedMask = calibratedChannelMask();
  const uint8_t expectedMask =
      static_cast<uint8_t>(
          calibratedMask & Config::PZT_ALL_CHANNELS_MASK);
  selfTestFailedMask |=
      static_cast<uint8_t>(expectedMask & ~verifiedChannelMask);

  const uint8_t usableMask =
      static_cast<uint8_t>(
          verifiedChannelMask & calibratedMask) &
      Config::PZT_ALL_CHANNELS_MASK;
  const uint8_t usableCount = countBits(usableMask);

  setErrorFlag(
      ERROR_PZT_UNVERIFIED,
      selfTestFailedMask != 0);
  setErrorFlag(ERROR_NO_USABLE_PZT, usableCount == 0);

  if (selfTestFailedMask != 0) {
    Serial.printf(
        "[WARN] PZT self-test failed mask=0x%02X\n",
        selfTestFailedMask);
  }

  if (usableCount == 0) {
    Serial.println(
        "[FAULT] No verified PZT channel; check wiring and enter v");
    enterFaultState();
    return;
  }

  enterArmedState();
  if (usableCount < Config::PZT_COUNT) {
    Serial.printf(
        "[READY] Degraded mode: %u/%u PZT channels verified\n",
        usableCount,
        static_cast<unsigned>(Config::PZT_COUNT));
  } else {
    Serial.println("[READY] All PZT channels verified; impact armed");
  }
  Serial.println();
}

void readMpuSample() {
  float accelerationX = 0.0f;
  float accelerationY = 0.0f;
  float accelerationZ = 0.0f;

  if (!readMpuAcceleration(
          accelerationX,
          accelerationY,
          accelerationZ)) {
    markMpuUnavailable();
    return;
  }

  if (systemState == SystemState::CALIBRATING) {
    accumulateImuCalibration(
        accelerationX,
        accelerationY,
        accelerationZ);
  }

  const float deltaX = accelerationX - imuReferenceX;
  const float deltaY = accelerationY - imuReferenceY;
  const float deltaZ = accelerationZ - imuReferenceZ;

  latestImuDeltaG =
      sqrtf(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ) /
      Config::STANDARD_GRAVITY;
  latestImuAccelG[0] = accelerationX / Config::STANDARD_GRAVITY;
  latestImuAccelG[1] = accelerationY / Config::STANDARD_GRAVITY;
  latestImuAccelG[2] = accelerationZ / Config::STANDARD_GRAVITY;
  latestImuSaturated =
      fabsf(accelerationX) / Config::STANDARD_GRAVITY >=
          Config::MPU_SATURATION_G ||
      fabsf(accelerationY) / Config::STANDARD_GRAVITY >=
          Config::MPU_SATURATION_G ||
      fabsf(accelerationZ) / Config::STANDARD_GRAVITY >=
          Config::MPU_SATURATION_G;
  latestImuSampleValid = true;
  lastMpuSampleUs = micros();

  if (systemState == SystemState::CAPTURING) {
    currentEvent.imuDataValid = true;
    if (latestImuDeltaG > currentEvent.imuPeakDeltaG) {
      currentEvent.imuPeakDeltaG = latestImuDeltaG;
    }
    if (latestImuSaturated) {
      currentEvent.imuSaturated = true;
    }
  }
}

// ---------------------------------------------------------------------------
// 충격 상태 처리
// ---------------------------------------------------------------------------

void beginCapture(
    uint32_t nowMs,
    uint32_t nowUs,
    bool manualCapture) {
  currentEvent = ImpactEvent();
  currentEvent.id = ++eventCounter;
  currentEvent.triggerMs = nowMs;
  currentEvent.triggerUs = nowUs;
  currentEvent.manualCapture = manualCapture;
  currentEvent.usableChannelMask = liveUsableChannelMask();
  currentEvent.eventType = nextEventLabel;
  nextEventLabel = EventType::UNCLASSIFIED_IMPACT;

  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1u << i);
    currentEvent.rawPeak[i] =
        (currentEvent.usableChannelMask & bit) != 0
            ? latestPztRaw[i]
            : 0;
  }

  const uint32_t imuAgeUs = micros() - lastMpuSampleUs;
  const bool freshImuSample =
      imuReady &&
      latestImuSampleValid &&
      imuAgeUs <= Config::MPU_SAMPLE_PERIOD_US * 2;
  if (freshImuSample) {
    currentEvent.imuDataValid = true;
    currentEvent.imuPeakDeltaG = latestImuDeltaG;
    currentEvent.imuSaturated = latestImuSaturated;
  }

  copyPreTriggerTraceToEvent();
  captureEventTimestamp(currentEvent);

  captureStartedMs = nowMs;
  systemState = SystemState::CAPTURING;
}

void updateCapturePeaks() {
  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1u << i);
    if ((currentEvent.usableChannelMask & bit) == 0) {
      continue;
    }

    if (latestPztRaw[i] > currentEvent.rawPeak[i]) {
      currentEvent.rawPeak[i] = latestPztRaw[i];
    }
  }
}

void finishCapture(uint32_t nowMs) {
  currentEvent.latencyMs = nowMs - currentEvent.triggerMs;
  calculateEvent(currentEvent);

  cooldownStartedMs = nowMs;
  belowResetStartedMs = 0;
  systemState = SystemState::COOLDOWN;

  // 상태를 먼저 COOLDOWN으로 바꿔 추적 파일 기록 시간이 새 측정 구간을
  // 침범하지 않도록 합니다. 요약 CSV는 RAM 큐를 거쳐 다음 loop에서 씁니다.
  writeEventTraceCsv(currentEvent);
  currentEvent.errorFlags |= systemErrorFlags;
  reportEvent(currentEvent);
}

void processCooldown(uint32_t nowMs) {
  const uint32_t elapsed = nowMs - cooldownStartedMs;

  bool allBelowReset = true;
  const uint8_t usableMask = liveUsableChannelMask();
  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1u << i);
    if ((usableMask & bit) == 0) {
      continue;
    }

    if (latestPztRaw[i] > pztResetThreshold[i]) {
      allBelowReset = false;
      break;
    }
  }

  if (elapsed >= Config::REARM_MINIMUM_MS && allBelowReset) {
    if (belowResetStartedMs == 0) {
      belowResetStartedMs = nowMs;
    } else if (
        nowMs - belowResetStartedMs >= Config::REARM_STABLE_MS) {
      enterArmedState();
      return;
    }
  } else {
    belowResetStartedMs = 0;
  }

  if (elapsed >= Config::REARM_TIMEOUT_MS) {
    const uint8_t timeoutUsableMask = liveUsableChannelMask();
    for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
      const uint8_t bit = static_cast<uint8_t>(1u << i);
      if ((timeoutUsableMask & bit) == 0) {
        continue;
      }

      if (latestPztRaw[i] > pztResetThreshold[i]) {
        suppressedChannelMask |= bit;
      }
    }

    if (suppressedChannelMask != 0) {
      setErrorFlag(ERROR_ADC_STUCK, true);
      Serial.printf(
          "[WARN] Rearm timeout; suppressed channel mask=0x%02X\n",
          suppressedChannelMask);
    }

    enterArmedState();
  }
}

void enterArmedState() {
  const uint8_t verifiedUsableMask =
      static_cast<uint8_t>(
          verifiedChannelMask & calibratedChannelMask()) &
      Config::PZT_ALL_CHANNELS_MASK;
  if (verifiedUsableMask == 0) {
    setErrorFlag(ERROR_NO_USABLE_PZT, true);
    enterFaultState();
    return;
  }

  systemState = SystemState::ARMED;
  nextDisplayMs = millis();

  if (!resultVisible) {
    setReadyIndicators();
  }
}

void enterFaultState() {
  systemState = SystemState::FAULT;
  resultVisible = false;
  nextDisplayMs = millis();
  stopAlarm();
  digitalWrite(Config::LED_GREEN_PIN, LOW);
  digitalWrite(Config::LED_YELLOW_PIN, LOW);
  digitalWrite(Config::LED_RED_PIN, HIGH);
}

// ---------------------------------------------------------------------------
// 위치, 상대 충격수준 및 점검 우선도 계산
// ---------------------------------------------------------------------------

void calculateEvent(ImpactEvent &event) {
  float maximumTriggerRatio = 0.0f;

  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1u << i);
    if ((event.usableChannelMask & bit) == 0) {
      event.correctedPeak[i] = 0.0f;
      continue;
    }

    float corrected =
        (static_cast<float>(event.rawPeak[i]) - pztBaseline[i]) *
        Config::PZT_GAIN[i];

    if (corrected < 0.0f) {
      corrected = 0.0f;
    }

    event.correctedPeak[i] = corrected;

    if (event.rawPeak[i] >= pztTriggerThreshold[i]) {
      event.activeChannelMask |= bit;
    }

    if (event.rawPeak[i] >= Config::ADC_SATURATION_COUNTS) {
      event.saturationMask |= bit;
    }

    float triggerMargin =
        (pztTriggerThreshold[i] - pztBaseline[i]) *
        Config::PZT_GAIN[i];
    if (triggerMargin < 1.0f) {
      triggerMargin = 1.0f;
    }

    const float triggerRatio = corrected / triggerMargin;
    if (triggerRatio > maximumTriggerRatio) {
      maximumTriggerRatio = triggerRatio;
    }
  }

  const uint8_t activeChannels =
      countBits(event.activeChannelMask);

  const bool pztTriggered = maximumTriggerRatio >= 1.0f;
  const bool pztConfirmed =
      activeChannels >= 2 ||
      maximumTriggerRatio >= Config::STRONG_SINGLE_CHANNEL_RATIO;
  const bool imuConfirmed =
      event.imuDataValid &&
      event.imuPeakDeltaG >= Config::IMU_CONFIRM_DELTA_G;

  event.valid =
      pztTriggered &&
      (pztConfirmed || imuConfirmed || event.testEvent);

  calculateLocation(event);

  // ADC에서 잘린 채널끼리는 상대 크기를 비교할 수 없습니다. 포화된
  // 사건을 특정 모서리로 강제 분류하면 FR/RR 등의 방향 편향처럼 보일 수
  // 있으므로 충격은 유효하게 기록하되 위치 결과는 명시적으로 무효화합니다.
  if (event.saturationMask != 0) {
    event.zone = ImpactZone::UNKNOWN;
    event.locationQuality = 0.0f;
  }

  calculateInspectionLevel(event);

  if (!event.valid) {
    event.zone = ImpactZone::UNKNOWN;
    event.locationQuality = 0.0f;
  }
}

void calculateLocation(ImpactEvent &event) {
  float totalWeight = 0.0f;
  float weightedX = 0.0f;
  float weightedY = 0.0f;
  float maximumWeight = 0.0f;
  const uint8_t usableChannels =
      countBits(event.usableChannelMask);

  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1u << i);
    if ((event.usableChannelMask & bit) == 0) {
      continue;
    }

    const float triggerMargin =
        (pztTriggerThreshold[i] - pztBaseline[i]) *
        Config::PZT_GAIN[i];
    const float deadband =
        triggerMargin * Config::LOCATION_DEADBAND_RATIO;

    float weight = event.correctedPeak[i] - deadband;
    if (weight < 0.0f) {
      weight = 0.0f;
    }

    totalWeight += weight;
    weightedX += weight * Config::PZT_X[i];
    weightedY += weight * Config::PZT_Y[i];

    if (weight > maximumWeight) {
      maximumWeight = weight;
    }
  }

  if (totalWeight <= 0.0f) {
    event.x = 0.5f;
    event.y = 0.5f;
    event.locationQuality = 0.0f;
    event.zone = ImpactZone::UNKNOWN;
    return;
  }

  event.x = clampFloat(weightedX / totalWeight, 0.0f, 1.0f);
  event.y = clampFloat(weightedY / totalWeight, 0.0f, 1.0f);

  float strengthConfidence =
      maximumWeight / Config::PZT_NORMALIZATION_COUNTS;
  strengthConfidence = clampFloat(strengthConfidence, 0.0f, 1.0f);

  if (event.saturationMask != 0) {
    strengthConfidence *= 0.70f;
  }
  strengthConfidence *=
      static_cast<float>(usableChannels) /
      static_cast<float>(Config::PZT_COUNT);
  event.locationQuality = strengthConfidence;

  if (usableChannels <= 2) {
    event.zone = ImpactZone::UNKNOWN;
    return;
  }

  if (event.x >= Config::CENTER_MIN &&
      event.x <= Config::CENTER_MAX &&
      event.y >= Config::CENTER_MIN &&
      event.y <= Config::CENTER_MAX) {
    event.zone = ImpactZone::CENTER;
  } else if (event.y >= 0.5f) {
    event.zone =
        event.x < 0.5f
            ? ImpactZone::FRONT_LEFT
            : ImpactZone::FRONT_RIGHT;
  } else {
    event.zone =
        event.x < 0.5f
            ? ImpactZone::REAR_LEFT
            : ImpactZone::REAR_RIGHT;
  }
}

void calculateInspectionLevel(ImpactEvent &event) {
  if (!event.valid) {
    event.impactScore = 0.0f;
    event.inspectionLevel = InspectionLevel::IGNORED;
    return;
  }

  float maximumNormalized = 0.0f;
  float normalizedSum = 0.0f;
  uint8_t usableChannels = 0;

  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1u << i);
    if ((event.usableChannelMask & bit) == 0) {
      continue;
    }
    ++usableChannels;

    const float normalized = clampFloat(
        event.correctedPeak[i] /
            Config::PZT_NORMALIZATION_COUNTS,
        0.0f,
        1.0f);

    normalizedSum += normalized;
    if (normalized > maximumNormalized) {
      maximumNormalized = normalized;
    }
  }

  const float meanNormalized =
      usableChannels > 0
          ? normalizedSum / static_cast<float>(usableChannels)
          : 0.0f;
  const float pztSeverity =
      0.65f * maximumNormalized +
      0.35f * meanNormalized;

  if (event.imuDataValid) {
    const float imuSeverity = clampFloat(
        event.imuPeakDeltaG /
            Config::IMU_NORMALIZATION_DELTA_G,
        0.0f,
        1.0f);
    event.impactScore =
        0.80f * pztSeverity +
        0.20f * imuSeverity;
  } else {
    event.impactScore = pztSeverity;
  }

  event.impactScore =
      clampFloat(event.impactScore, 0.0f, 1.0f);

  if (event.saturationMask != 0 ||
      event.imuSaturated ||
      event.impactScore >= Config::PRIORITY_INSPECTION_SCORE) {
    event.inspectionLevel = InspectionLevel::PRIORITY_INSPECTION;
  } else if (
      event.impactScore >= Config::INSPECTION_RECOMMENDED_SCORE) {
    event.inspectionLevel = InspectionLevel::INSPECTION_RECOMMENDED;
  } else {
    event.inspectionLevel = InspectionLevel::RECORD_ONLY;
  }
}

// ---------------------------------------------------------------------------
// 결과 보고, SD 및 시리얼
// ---------------------------------------------------------------------------

void reportEvent(ImpactEvent &event) {
  enqueueEventForLog(event);

  lastEvent = event;
  lastEventAvailable = true;
  resultVisible = true;
  resultVisibleUntilMs = millis() + Config::RESULT_HOLD_MS;
  nextDisplayMs = millis();

  setInspectionIndicators(event.inspectionLevel);
  startAlarm(event.inspectionLevel, millis());
  printEventSummary(event);

  if (logQueueCount > 0) {
    const size_t newestIndex =
        (logQueueHead + logQueueCount - 1) %
        Config::LOG_QUEUE_CAPACITY;
    emitQueuedEventSerial(logQueue[newestIndex]);
  }
}

void enqueueEventForLog(ImpactEvent &event) {
  if (logQueueCount >= Config::LOG_QUEUE_CAPACITY) {
    QueuedLog &dropped = logQueue[logQueueHead];
    emitQueuedEventSerial(dropped);
    logQueueHead =
        (logQueueHead + 1) % Config::LOG_QUEUE_CAPACITY;
    --logQueueCount;
    ++logQueueDroppedCount;
    setErrorFlag(ERROR_LOG_QUEUE_OVERFLOW, true);
    event.errorFlags |= ERROR_LOG_QUEUE_OVERFLOW;
    Serial.printf(
        "[WARN] Log queue full; oldest event dropped (total=%lu)\n",
        static_cast<unsigned long>(logQueueDroppedCount));
  }

  if (!sdReady) {
    event.errorFlags |= ERROR_SD;
  }

  const size_t tail =
      (logQueueHead + logQueueCount) %
      Config::LOG_QUEUE_CAPACITY;
  logQueue[tail] = QueuedLog();
  logQueue[tail].event = event;
  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    logQueue[tail].baseline[i] = pztBaseline[i];
  }
  logQueue[tail].queuedAtMs = millis();
  ++logQueueCount;

  if (logQueueCount == 1) {
    nextLogServiceMs =
        logQueue[tail].queuedAtMs +
        Config::LOG_WRITE_DELAY_MS;
  }
}

void serviceLogQueue() {
  if (logQueueCount == 0 || !sdReady) {
    return;
  }

  QueuedLog &queuedLog = logQueue[logQueueHead];
  if (writeQueuedEventCsv(queuedLog)) {
    emitQueuedEventSerial(queuedLog);
    logQueueHead =
        (logQueueHead + 1) % Config::LOG_QUEUE_CAPACITY;
    --logQueueCount;
    nextLogServiceMs =
        millis() + Config::LOG_WRITE_DELAY_MS;
    return;
  }

  queuedLog.event.errorFlags |= ERROR_SD;
  if (lastEventAvailable &&
      lastEvent.id == queuedLog.event.id) {
    lastEvent.errorFlags |= ERROR_SD;
  }
  sdReady = false;
  setErrorFlag(ERROR_SD, true);
  Serial.println(
      "[WARN] microSD write failed; event retained in RAM queue");
  emitQueuedEventSerial(queuedLog);
}

bool writeQueuedEventCsv(QueuedLog &queuedLog) {
  if (!sdReady) {
    return false;
  }

  char csvLine[768];
  if (!formatEventCsv(
          queuedLog.event,
          queuedLog.baseline,
          csvLine,
          sizeof(csvLine))) {
    Serial.println("[ERROR] CSV format buffer too small");
    return false;
  }

  File file = SD.open(Config::SD_LOG_PATH, FILE_APPEND);
  if (!file) {
    return false;
  }

  const size_t lineLength = strlen(csvLine);
  const size_t written = file.println(csvLine);
  file.flush();
  file.close();
  return written == lineLength + 2;
}

void emitQueuedEventSerial(QueuedLog &queuedLog) {
  if (queuedLog.serialEmitted) {
    return;
  }

  char csvLine[768];
  if (formatEventCsv(
          queuedLog.event,
          queuedLog.baseline,
          csvLine,
          sizeof(csvLine))) {
    Serial.print("CSV>");
    Serial.println(csvLine);
  } else {
    Serial.printf(
        "[ERROR] Event #%lu CSV formatting failed\n",
        static_cast<unsigned long>(queuedLog.event.id));
  }
  queuedLog.serialEmitted = true;
}

bool formatEventCsv(
    const ImpactEvent &event,
    const float baseline[Config::PZT_COUNT],
    char *buffer,
    size_t bufferSize) {
  char timestamp[32];
  formatEventTimestamp(event, timestamp, sizeof(timestamp));

  const int written = snprintf(
      buffer,
      bufferSize,
      "%lu,%lu,%s,%s,%s,%s,%u,%lu,"
      "%.1f,%.1f,%.1f,%.1f,"
      "%u,%u,%u,%u,"
      "%.1f,%.1f,%.1f,%.1f,"
      "%.3f,%.4f,%.4f,%s,%.3f,"
      "%.3f,%s,%lu,%u,%u,%u,"
      "%u,%u,%u,%u,%s,%u,%u,%s",
      static_cast<unsigned long>(sessionId),
      static_cast<unsigned long>(event.id),
      event.testEvent
          ? "TEST"
          : (event.manualCapture ? "MANUAL" : "DETECTED"),
      eventTypeName(event.eventType),
      timestamp,
      timeSourceName(event.timeSource),
      event.rtcValid ? 1u : 0u,
      static_cast<unsigned long>(event.triggerMs),
      baseline[0],
      baseline[1],
      baseline[2],
      baseline[3],
      event.rawPeak[0],
      event.rawPeak[1],
      event.rawPeak[2],
      event.rawPeak[3],
      event.correctedPeak[0],
      event.correctedPeak[1],
      event.correctedPeak[2],
      event.correctedPeak[3],
      event.imuPeakDeltaG,
      event.x,
      event.y,
      zoneName(event.zone),
      event.locationQuality,
      event.impactScore,
      inspectionLevelName(event.inspectionLevel),
      static_cast<unsigned long>(event.latencyMs),
      event.valid ? 1u : 0u,
      static_cast<unsigned>(event.saturationMask),
      event.imuSaturated ? 1u : 0u,
      static_cast<unsigned>(event.activeChannelMask),
      static_cast<unsigned>(event.usableChannelMask),
      static_cast<unsigned>(event.errorFlags),
      event.imuDataValid ? 1u : 0u,
      event.tracePath,
      static_cast<unsigned>(event.traceSamples),
      event.traceSaved ? 1u : 0u,
      Config::FIRMWARE_VERSION);

  return written >= 0 &&
         static_cast<size_t>(written) < bufferSize;
}

void formatEventTimestamp(
    const ImpactEvent &event,
    char *buffer,
    size_t bufferSize) {
  if (bufferSize == 0) {
    return;
  }

  if (!event.rtcValid) {
    snprintf(buffer, bufferSize, "NA");
    return;
  }

  // DS3231에는 시간대 정보가 없으므로 한국 현지시각으로 맞춰 사용합니다.
  snprintf(
      buffer,
      bufferSize,
      "%04u-%02u-%02uT%02u:%02u:%02u+09:00",
      static_cast<unsigned>(event.rtcYear),
      static_cast<unsigned>(event.rtcMonth),
      static_cast<unsigned>(event.rtcDay),
      static_cast<unsigned>(event.rtcHour),
      static_cast<unsigned>(event.rtcMinute),
      static_cast<unsigned>(event.rtcSecond));
}

void printEventSummary(const ImpactEvent &event) {
  char timestamp[32];
  formatEventTimestamp(event, timestamp, sizeof(timestamp));

  Serial.println();
  Serial.println("----------------------------------------");
  Serial.printf(
      "Event #%lu  source=%s  type=%s  valid=%s\n",
      static_cast<unsigned long>(event.id),
      event.testEvent
          ? "TEST"
          : (event.manualCapture ? "MANUAL" : "DETECTED"),
      eventTypeName(event.eventType),
      event.valid ? "YES" : "NO");
  Serial.printf(
      "Time: %s  source=%s  uptime=%lu ms\n",
      timestamp,
      timeSourceName(event.timeSource),
      static_cast<unsigned long>(event.triggerMs));
  Serial.printf(
      "PZT raw: %u, %u, %u, %u\n",
      event.rawPeak[0],
      event.rawPeak[1],
      event.rawPeak[2],
      event.rawPeak[3]);
  Serial.printf(
      "Position: %s  x=%.3f y=%.3f quality=%.2f\n",
      zoneName(event.zone),
      event.x,
      event.y,
      event.locationQuality);
  Serial.printf(
      "IMU peak: %.3f g  valid=%s  impact_score=%.3f  inspection=%s\n",
      event.imuPeakDeltaG,
      event.imuDataValid ? "YES" : "NO",
      event.impactScore,
      inspectionLevelName(event.inspectionLevel));
  Serial.printf(
      "PZT saturation=0x%02X  IMU saturation=%s  errors=0x%04X\n",
      event.saturationMask,
      event.imuSaturated ? "YES" : "NO",
      event.errorFlags);
  Serial.printf(
      "Trace: %s  samples=%u  saved=%s\n",
      event.tracePath[0] != '\0' ? event.tracePath : "NONE",
      static_cast<unsigned>(event.traceSamples),
      event.traceSaved ? "YES" : "NO");
  Serial.println("----------------------------------------");
}

// ---------------------------------------------------------------------------
// OLED
// ---------------------------------------------------------------------------

void updateDisplay(uint32_t nowMs) {
  if (!oledReady) {
    return;
  }

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setTextWrap(false);

  if (systemState == SystemState::CALIBRATING) {
    const uint32_t elapsed = nowMs - calibrationStartedMs;
    const uint32_t remaining =
        elapsed >= Config::CALIBRATION_TIME_MS
            ? 0
            : Config::CALIBRATION_TIME_MS - elapsed;

    oled.setCursor(0, 0);
    oled.println("EV-BLACKBOX CAL");
    oled.print("Try ");
    oled.print(calibrationAttempt);
    oled.print("/");
    oled.print(Config::CALIBRATION_MAX_ATTEMPTS);
    oled.print("  ");
    oled.print((remaining + 999) / 1000);
    oled.println("s");
    oled.print("P1:");
    oled.print(latestPztRaw[0]);
    oled.print(" P2:");
    oled.println(latestPztRaw[1]);
    oled.print("P3:");
    oled.print(latestPztRaw[2]);
    oled.print(" P4:");
    oled.print(latestPztRaw[3]);
    oled.display();
    return;
  }

  if (systemState == SystemState::VERIFYING_PZT) {
    static const char *const labels[Config::PZT_COUNT] = {
        "P1 FRONT-LEFT",
        "P2 FRONT-RIGHT",
        "P3 REAR-LEFT",
        "P4 REAR-RIGHT"};
    oled.setCursor(0, 0);
    oled.println("PZT SELF TEST");
    if (selfTestFinalRelease) {
      oled.println("ALL TAPS COMPLETE");
      oled.println("RELEASE THE PLATE");
      oled.println("WAITING FOR RESET");
    } else if (selfTestTarget < Config::PZT_COUNT) {
      oled.println(labels[selfTestTarget]);
      oled.println(
          selfTestReadyForTap
              ? "TAP THIS SENSOR NOW"
              : "WAITING FOR RESET");
      oled.print("RAW:");
      oled.print(latestPztRaw[selfTestTarget]);
      oled.print(" T:");
      oled.print(
          static_cast<int>(
              pztTriggerThreshold[selfTestTarget]));
    }
    oled.display();
    return;
  }

  if (systemState == SystemState::FAULT) {
    oled.setCursor(0, 0);
    oled.println("EV-BLACKBOX FAULT");
    oled.println("NO USABLE PZT");
    oled.print("DIS:0x");
    oled.print(disabledChannelMask, HEX);
    oled.print(" FAIL:0x");
    oled.println(selfTestFailedMask, HEX);
    oled.println("FIX + SEND c OR v");
    oled.display();
    return;
  }

  if (resultVisible && lastEventAvailable) {
    oled.setCursor(0, 0);
    oled.print("#");
    oled.print(lastEvent.id);
    oled.print(" ");
    oled.println(inspectionLevelName(lastEvent.inspectionLevel));

    oled.print(zoneName(lastEvent.zone));
    oled.print(" X:");
    oled.print(static_cast<int>(lastEvent.x * 100.0f));
    oled.print(" Y:");
    oled.println(static_cast<int>(lastEvent.y * 100.0f));

    oled.print("P:");
    float maximumPeak = 0.0f;
    for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
      if (lastEvent.correctedPeak[i] > maximumPeak) {
        maximumPeak = lastEvent.correctedPeak[i];
      }
    }
    oled.print(static_cast<int>(maximumPeak));
    oled.print(" G:");
    oled.println(lastEvent.imuPeakDeltaG, 2);

    const uint8_t resultUsableMask = liveUsableChannelMask();
    if (systemState == SystemState::COOLDOWN) {
      oled.print("WAIT");
    } else if (resultUsableMask == 0) {
      oled.print("HOLD");
    } else if (
        resultUsableMask != Config::PZT_ALL_CHANNELS_MASK) {
      oled.print("DEGR");
    } else {
      oled.print("READY");
    }
    oled.print(" SD:");
    oled.print(sdReady ? "OK" : "--");
    oled.print(" E:");
    oled.print(systemErrorFlags, HEX);
    oled.display();
    return;
  }

  const uint8_t liveUsableMask = liveUsableChannelMask();

  oled.setCursor(0, 0);
  if (liveUsableMask == 0) {
    oled.println("EV-BLACKBOX HOLD");
    oled.println("NO ACTIVE PZT");
    oled.print("SUP:0x");
    oled.print(suppressedChannelMask, HEX);
    oled.print(" DIS:0x");
    oled.println(disabledChannelMask, HEX);
    oled.println("WAIT AUTO RECOVERY");
    oled.display();
    return;
  }

  oled.println(
      liveUsableMask == Config::PZT_ALL_CHANNELS_MASK
          ? "EV-BLACKBOX READY"
          : "EV-BLACKBOX DEGRADED");
  oled.print("P1:");
  oled.print(latestPztRaw[0]);
  oled.print(" P2:");
  oled.println(latestPztRaw[1]);
  oled.print("P3:");
  oled.print(latestPztRaw[2]);
  oled.print(" P4:");
  oled.println(latestPztRaw[3]);
  oled.print("IMU:");
  oled.print(imuReady ? "OK" : "--");
  oled.print(" SD:");
  oled.print(sdReady ? "OK" : "--");
  oled.print(" E:");
  oled.print(systemErrorFlags, HEX);

  if (Config::OLED_HEIGHT >= 64) {
    oled.setCursor(0, 32);
    oled.print("State: ");
    oled.println(stateName(systemState));
    oled.print("dG: ");
    oled.println(latestImuDeltaG, 3);
    oled.print("T1:");
    oled.print(static_cast<int>(pztTriggerThreshold[0]));
    oled.print(" T2:");
    oled.println(static_cast<int>(pztTriggerThreshold[1]));
    oled.print("FW v");
    oled.println(Config::FIRMWARE_VERSION);
  }

  oled.display();
}

// ---------------------------------------------------------------------------
// LED 및 능동 부저
// ---------------------------------------------------------------------------

void setReadyIndicators() {
  const uint8_t usableMask = liveUsableChannelMask();
  const bool allChannelsUsable =
      usableMask == Config::PZT_ALL_CHANNELS_MASK;
  const bool noChannelUsable = usableMask == 0;

  digitalWrite(
      Config::LED_GREEN_PIN,
      allChannelsUsable ? HIGH : LOW);
  digitalWrite(
      Config::LED_YELLOW_PIN,
      !allChannelsUsable && !noChannelUsable ? HIGH : LOW);
  digitalWrite(
      Config::LED_RED_PIN,
      noChannelUsable ? HIGH : LOW);
}

void setInspectionIndicators(InspectionLevel level) {
  digitalWrite(
      Config::LED_GREEN_PIN,
      level == InspectionLevel::RECORD_ONLY ? HIGH : LOW);
  digitalWrite(
      Config::LED_YELLOW_PIN,
      level == InspectionLevel::INSPECTION_RECOMMENDED ? HIGH : LOW);
  digitalWrite(
      Config::LED_RED_PIN,
      level == InspectionLevel::PRIORITY_INSPECTION ? HIGH : LOW);

  if (level == InspectionLevel::IGNORED) {
    digitalWrite(Config::LED_GREEN_PIN, LOW);
    digitalWrite(Config::LED_YELLOW_PIN, LOW);
    digitalWrite(Config::LED_RED_PIN, LOW);
  }
}

void startAlarm(InspectionLevel level, uint32_t nowMs) {
  stopAlarm();

  if (level == InspectionLevel::INSPECTION_RECOMMENDED) {
    alarmController.pulsesRemaining = 2;
    alarmController.onTimeMs = 120;
    alarmController.offTimeMs = 140;
  } else if (level == InspectionLevel::PRIORITY_INSPECTION) {
    alarmController.pulsesRemaining = 4;
    alarmController.onTimeMs = 180;
    alarmController.offTimeMs = 100;
  } else {
    return;
  }

  alarmController.active = true;
  alarmController.outputOn = true;
  alarmController.changedAtMs = nowMs;
  digitalWrite(
      Config::BUZZER_PIN,
      Config::BUZZER_ACTIVE_HIGH ? HIGH : LOW);
}

void updateAlarm(uint32_t nowMs) {
  if (!alarmController.active) {
    return;
  }

  if (alarmController.outputOn) {
    if (nowMs - alarmController.changedAtMs >=
        alarmController.onTimeMs) {
      alarmController.outputOn = false;
      alarmController.changedAtMs = nowMs;
      digitalWrite(
          Config::BUZZER_PIN,
          Config::BUZZER_ACTIVE_HIGH ? LOW : HIGH);

      if (alarmController.pulsesRemaining > 0) {
        --alarmController.pulsesRemaining;
      }

      if (alarmController.pulsesRemaining == 0) {
        alarmController.active = false;
      }
    }
  } else if (
      nowMs - alarmController.changedAtMs >=
      alarmController.offTimeMs) {
    alarmController.outputOn = true;
    alarmController.changedAtMs = nowMs;
    digitalWrite(
        Config::BUZZER_PIN,
        Config::BUZZER_ACTIVE_HIGH ? HIGH : LOW);
  }
}

void stopAlarm() {
  alarmController = AlarmController();
  digitalWrite(
      Config::BUZZER_PIN,
      Config::BUZZER_ACTIVE_HIGH ? LOW : HIGH);
}

// ---------------------------------------------------------------------------
// 시리얼 명령 및 자가시험
// ---------------------------------------------------------------------------

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char command =
        static_cast<char>(tolower(Serial.read()));

    if (command == '\r' || command == '\n' || command == ' ') {
      continue;
    }

    switch (command) {
      case 'h':
        printHelp();
        break;

      case 's':
        printStatus();
        break;

      case 'd':
        printRtcStatus();
        break;

      case 'c':
        if (systemState != SystemState::CAPTURING) {
          if (!imuReady) {
            Serial.println("[INFO] Retrying MPU6050 before calibration");
            if (initializeMpuDevice()) {
              Serial.printf(
                  "[OK] MPU6050 reconnected at 0x%02X\n",
                  mpuAddress);
            } else {
              setErrorFlag(ERROR_IMU, true);
              Serial.println(
                  "[WARN] MPU6050 still unavailable; calibrating PZT only");
            }
          }

          startCalibration();
        }
        break;

      case 'v':
        if (systemState == SystemState::ARMED ||
            systemState == SystemState::FAULT ||
            systemState == SystemState::VERIFYING_PZT) {
          startPztSelfTest();
        } else {
          Serial.println(
              "[INFO] PZT self-test allowed after calibration only");
        }
        break;

      case 't':
        if (systemState == SystemState::ARMED) {
          createTestEvent();
        } else {
          Serial.println("[INFO] Test allowed only in ARMED state");
        }
        break;

      case 'r':
        if (systemState == SystemState::ARMED ||
            systemState == SystemState::FAULT) {
          if (initializeSd()) {
            nextLogServiceMs = millis();
          }
        } else {
          Serial.println(
              "[INFO] SD retry allowed only in ARMED or FAULT state");
        }
        break;

      case 'g':
        if (systemState == SystemState::ARMED) {
          Serial.printf(
              "[CAPTURE] Manual dataset window started; label=%s\n",
              eventTypeName(nextEventLabel));
          beginCapture(millis(), micros(), true);
        } else {
          Serial.println(
              "[INFO] Manual capture allowed only in ARMED state");
        }
        break;

      case '0':
        nextEventLabel = EventType::UNCLASSIFIED_IMPACT;
        Serial.println("[LABEL] Next detected event: UNCLASSIFIED_IMPACT");
        break;

      case '1':
        nextEventLabel = EventType::NORMAL_VIBRATION;
        Serial.println("[LABEL] Next detected event: NORMAL_VIBRATION");
        break;

      case '2':
        nextEventLabel = EventType::LOCAL_IMPACT;
        Serial.println("[LABEL] Next detected event: LOCAL_IMPACT");
        break;

      case '3':
        nextEventLabel = EventType::REPEATED_SHOCK;
        Serial.println("[LABEL] Next detected event: REPEATED_SHOCK");
        break;

      case '4':
        nextEventLabel = EventType::SENSOR_NOISE;
        Serial.println("[LABEL] Next detected event: SENSOR_NOISE");
        break;

      default:
        Serial.println("[INFO] Unknown command; enter h");
        break;
    }
  }
}

void printHelp() {
  Serial.println();
  Serial.println("Serial commands");
  Serial.println("  h : help");
  Serial.println("  s : print status and current sensor values");
  Serial.println("  d : read current RTC time or retry RTC connection");
  Serial.println("  c : recalibrate, then run PZT tap self-test");
  Serial.println("  v : repeat PZT tap self-test");
  Serial.println("  t : create a simulated INSPECT event");
  Serial.println("  r : retry microSD and drain retained events");
  Serial.println("  g : manually capture a labeled 1.5 s dataset window");
  Serial.println("  0 : label next event UNCLASSIFIED (default)");
  Serial.println("  1 : label next event NORMAL_VIBRATION");
  Serial.println("  2 : label next event LOCAL_IMPACT");
  Serial.println("  3 : label next event REPEATED_SHOCK");
  Serial.println("  4 : label next event SENSOR_NOISE");
  Serial.println("  Labels are for dataset collection, not automatic classification.");
  Serial.println();
}

void printStatus() {
  Serial.println();
  Serial.printf(
      "State=%s errors=0x%04X SD=%s OLED=%s MPU=%s RTC=%s\n",
      stateName(systemState),
      systemErrorFlags,
      sdReady ? "OK" : "NO",
      oledReady ? "OK" : "NO",
      imuReady ? "OK" : "NO",
      rtcReady ? "OK" : "NO");
  Serial.printf(
      "MPU address=0x%02X  time_source=%s  next_label=%s\n",
      static_cast<unsigned>(mpuAddress),
      timeSourceName(currentTimeSource),
      eventTypeName(nextEventLabel));
  Serial.printf(
      "PZT raw=%u,%u,%u,%u  suppressed=0x%02X disabled=0x%02X "
      "verified=0x%02X failed=0x%02X self-stuck=0x%02X\n",
      latestPztRaw[0],
      latestPztRaw[1],
      latestPztRaw[2],
      latestPztRaw[3],
      suppressedChannelMask,
      disabledChannelMask,
      verifiedChannelMask,
      selfTestFailedMask,
      selfTestStuckMask);
  Serial.printf(
      "Threshold=%.0f,%.0f,%.0f,%.0f  MPU dG=%.3f\n",
      pztTriggerThreshold[0],
      pztTriggerThreshold[1],
      pztTriggerThreshold[2],
      pztTriggerThreshold[3],
      latestImuDeltaG);
  Serial.printf(
      "Log queue=%u/%u dropped=%lu  pretrace=%u/%u samples\n",
      static_cast<unsigned>(logQueueCount),
      static_cast<unsigned>(Config::LOG_QUEUE_CAPACITY),
      static_cast<unsigned long>(logQueueDroppedCount),
      static_cast<unsigned>(preTriggerCount),
      static_cast<unsigned>(Config::TRACE_PRE_SAMPLES));
}

void createTestEvent() {
  static const uint16_t testCorrected[Config::PZT_COUNT] = {
      900, 1650, 650, 420};

  currentEvent = ImpactEvent();
  currentEvent.id = ++eventCounter;
  currentEvent.triggerMs = millis();
  currentEvent.triggerUs = micros();
  currentEvent.latencyMs = Config::CAPTURE_TIME_MS;
  currentEvent.imuPeakDeltaG = 0.85f;
  currentEvent.imuDataValid = true;
  currentEvent.errorFlags = systemErrorFlags;
  currentEvent.testEvent = true;
  currentEvent.eventType = EventType::TEST_EVENT;
  currentEvent.usableChannelMask = liveUsableChannelMask();
  captureEventTimestamp(currentEvent);

  for (size_t i = 0; i < Config::PZT_COUNT; ++i) {
    float rawValue =
        pztBaseline[i] +
        static_cast<float>(testCorrected[i]) / Config::PZT_GAIN[i];
    rawValue = clampFloat(
        rawValue,
        0.0f,
        static_cast<float>(Config::ADC_MAX_COUNTS));
    currentEvent.rawPeak[i] = static_cast<uint16_t>(rawValue);
  }

  calculateEvent(currentEvent);
  // 자가시험은 실제 임계값과 관계없이 점검 권고 출력 경로를 검사합니다.
  currentEvent.valid = true;
  currentEvent.inspectionLevel =
      InspectionLevel::INSPECTION_RECOMMENDED;
  currentEvent.impactScore = 0.45f;

  cooldownStartedMs = millis();
  belowResetStartedMs = 0;
  systemState = SystemState::COOLDOWN;
  reportEvent(currentEvent);
}

// ---------------------------------------------------------------------------
// 공통 유틸리티
// ---------------------------------------------------------------------------

void setErrorFlag(uint16_t flag, bool enabled) {
  if (enabled) {
    systemErrorFlags |= flag;
  } else {
    systemErrorFlags &= static_cast<uint16_t>(~flag);
  }
}

uint8_t calibratedChannelMask() {
  if (!pztCalibrationValid) {
    return 0;
  }

  return
      static_cast<uint8_t>(~disabledChannelMask) &
      Config::PZT_ALL_CHANNELS_MASK;
}

uint8_t liveUsableChannelMask() {
  return
      static_cast<uint8_t>(
          calibratedChannelMask() &
          verifiedChannelMask &
          static_cast<uint8_t>(~suppressedChannelMask)) &
      Config::PZT_ALL_CHANNELS_MASK;
}

uint8_t countBits(uint8_t value) {
  uint8_t count = 0;
  while (value != 0) {
    count += value & 1u;
    value >>= 1u;
  }
  return count;
}

bool deadlinePending(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(deadlineMs - nowMs) > 0;
}

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

const char *stateName(SystemState state) {
  switch (state) {
    case SystemState::BOOT:
      return "BOOT";
    case SystemState::CALIBRATING:
      return "CAL";
    case SystemState::VERIFYING_PZT:
      return "PZT-TEST";
    case SystemState::ARMED:
      return "ARMED";
    case SystemState::CAPTURING:
      return "CAPTURE";
    case SystemState::COOLDOWN:
      return "WAIT";
    case SystemState::FAULT:
      return "FAULT";
  }
  return "?";
}

const char *inspectionLevelName(InspectionLevel level) {
  switch (level) {
    case InspectionLevel::IGNORED:
      return "IGNORED";
    case InspectionLevel::RECORD_ONLY:
      return "RECORD";
    case InspectionLevel::INSPECTION_RECOMMENDED:
      return "INSPECT";
    case InspectionLevel::PRIORITY_INSPECTION:
      return "PRIORITY";
  }
  return "?";
}

const char *eventTypeName(EventType type) {
  switch (type) {
    case EventType::UNCLASSIFIED_IMPACT:
      return "UNCLASSIFIED_IMPACT";
    case EventType::NORMAL_VIBRATION:
      return "NORMAL_VIBRATION";
    case EventType::LOCAL_IMPACT:
      return "LOCAL_IMPACT";
    case EventType::REPEATED_SHOCK:
      return "REPEATED_SHOCK";
    case EventType::SENSOR_NOISE:
      return "SENSOR_NOISE";
    case EventType::TEST_EVENT:
      return "TEST_EVENT";
  }
  return "?";
}

const char *timeSourceName(TimeSource source) {
  switch (source) {
    case TimeSource::UPTIME_ONLY:
      return "UPTIME_ONLY";
    case TimeSource::RTC:
      return "RTC";
    case TimeSource::BUILD_TIME_SYNC:
      return "BUILD_TIME_SYNC";
  }
  return "?";
}

const char *zoneName(ImpactZone zone) {
  switch (zone) {
    case ImpactZone::UNKNOWN:
      return "UNKNOWN";
    case ImpactZone::CENTER:
      return "CENTER";
    case ImpactZone::FRONT_LEFT:
      return "FRONT-L";
    case ImpactZone::FRONT_RIGHT:
      return "FRONT-R";
    case ImpactZone::REAR_LEFT:
      return "REAR-L";
    case ImpactZone::REAR_RIGHT:
      return "REAR-R";
  }
  return "?";
}
