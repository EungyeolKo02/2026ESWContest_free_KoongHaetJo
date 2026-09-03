# microSD 로그 형식

펌웨어 `v0.4.1`은 이벤트 요약과 상세 파형을 서로 다른 CSV 파일로 저장합니다.

## 1. 이벤트 요약: `/impact_log_v3.csv`

이 파일은 이벤트당 한 행을 기록합니다. 주요 필드는 다음과 같습니다.

| 분류 | 필드 | 설명 |
|---|---|---|
| 식별 | `session`, `event_id` | 부팅 세션과 세션 내 이벤트 번호 |
| 출처 | `source`, `event_type` | 실제 검출·모의시험·수동수집 등 이벤트 생성 경로와 수동 라벨 |
| 시간 | `rtc_iso8601`, `time_source`, `rtc_valid`, `trigger_ms` | RTC 시간 또는 부팅 후 경과 시간과 유효성 |
| 기준값 | `baseline_fl`~`baseline_rr` | 검출 시점의 PZT 채널별 기준 ADC 값 |
| 원신호 | `raw_fl`~`raw_rr` | 이벤트에서 관측한 채널별 원시 최대값 |
| 보정신호 | `corrected_fl`~`corrected_rr` | 기준값·채널 이득을 반영한 상대 크기 |
| 가속도 | `imu_peak_delta_g` | 기준 중력에서 벗어난 최대 가속도 변화량 |
| 위치 | `x_norm`, `y_norm`, `zone`, `location_quality` | 0~1 정규화 좌표, 영역, 위치 신뢰도 |
| 결과 | `impact_score`, `inspection_level`, `latency_ms` | 상대 점수, 점검 우선순위, 처리 지연 |
| 품질 | `valid`, `saturation_mask`, `imu_saturated` | 이벤트 유효성과 센서 포화 정보 |
| 채널 | `active_channel_mask`, `usable_channel_mask` | 반응 채널과 사용 가능한 채널 비트마스크 |
| 오류 | `error_flags`, `imu_data_valid` | 장치·기록 오류 비트와 IMU 데이터 유효성 |
| 파형 | `trace_path`, `trace_samples`, `trace_saved` | 연결된 상세 파형 파일과 저장 결과 |
| 버전 | `firmware_version` | 이벤트를 생성한 펌웨어 버전 |

PZT 비트마스크는 하위 비트부터 `FL`, `FR`, `RL`, `RR` 순서입니다. 예를 들어 `0b0101`은 FL과 RL을 뜻합니다.

## 2. 상세 파형: `/tr_...csv`

각 행은 한 시점의 센서 샘플입니다.

| 필드 | 설명 |
|---|---|
| `relative_us` | 트리거 시점을 0으로 한 상대 시간(µs), 음수는 충격 전 |
| `pzt_fl`, `pzt_fr`, `pzt_rl`, `pzt_rr` | 네 PZT의 원시 ADC 값 |
| `accel_x_g`, `accel_y_g`, `accel_z_g` | MPU6050 3축 가속도(g) |
| `imu_valid` | 해당 행의 IMU 값 유효 여부 |

기본 설정은 트리거 전 0.5초와 후 1초를 저장합니다. PZT는 약 1 ms, MPU6050은 약 5 ms 주기로 갱신되므로 연속 행에서 IMU 값이 반복될 수 있습니다.

## 기록 실패 처리

- 요약 로그 쓰기에 실패하면 CSV 행을 시리얼 모니터에도 출력합니다.
- RAM 큐에 최대 8개 이벤트를 보관합니다.
- microSD가 비정상 상태이면 약 30초마다 재초기화를 시도합니다.
- 시리얼 명령 `r`로 즉시 재초기화와 큐 비우기를 요청할 수 있습니다.
- 전원을 끄면 RAM 큐의 미기록 이벤트는 사라지므로 실험 중 시리얼 로그도 함께 저장하는 것을 권장합니다.

## 분석 전 품질 점검

1. `firmware_version`과 실험 기록의 버전이 일치하는지 확인합니다.
2. `valid=1`, `trace_saved=1`, `imu_data_valid=1` 비율을 확인합니다.
3. `saturation_mask`와 `imu_saturated`가 반복되는 조건은 정량 분석에서 분리합니다.
4. `error_flags`가 0이 아닌 이벤트는 원인을 확인하고 제외 여부를 기록합니다.
5. `trace_samples`와 상대 시간 범위가 기대한 1.5초에 가까운지 확인합니다.
6. 물리 위치와 `zone`, `x_norm`, `y_norm`이 일관되는지 확인합니다.

로그의 점수와 등급은 실제 차량 손상 판정값이 아니라 시험 장치 안의 상대 비교값입니다.
