# 실험·보정 데이터 관리

이 폴더는 EV-BLACKBOX의 실험 데이터를 재현 가능하게 관리하기 위한 공간입니다. 최종 보고서의 75회 시험은 5개 위치, 3개 높이, 조건별 5회 반복으로 구성됩니다.

## 권장 구조

```text
calibration_data/
├─ raw/          # microSD에서 복사한 원본(수정 금지, Git 제외)
├─ processed/    # 정리·필터링·특징 추출 결과
├─ results/      # 표, 그래프, 성능 요약
└─ README.md
```

원본 파일은 복사 후 수정하지 않습니다. 분석 과정에서 새 파일이 필요하면 `processed`에 저장하고, 최종 표와 그래프는 `results`에 저장합니다.

상세 파형은 용량이 커질 수 있으므로 공개 GitHub에는 대표 샘플, 요약표, 결과 그래프만 올리고 전체 원본은 별도 백업 매체에 보관합니다. 이 저장소의 `.gitignore`는 `calibration_data/raw/`를 기본적으로 제외합니다.

## 파일 정리 예시

```text
raw/2026-08-25_pilot01/impact_log_v3.csv
raw/2026-08-25_pilot01/tr_*.csv
processed/2026-08-25_pilot01_event_index.csv
results/2026-08-25_pilot01_summary.csv
```

## 실험 인덱스 필수 항목

펌웨어 로그만으로 알 수 없는 실험 조건을 별도 인덱스 CSV에 기록합니다.

| 항목 | 예시 |
|---|---|
| `experiment_id` | `PILOT01` |
| `event_id` | 펌웨어 로그의 이벤트 ID |
| `label` | 정상진동 / 국부충격 / 반복충격 / 센서잡음 |
| `impact_position` | FL / FR / RL / RR / CENTER |
| `drop_mass_g` | 낙하 물체 질량(g) |
| `drop_height_cm` | 낙하 높이(cm) |
| `support_condition` | 고무발 위치·개수 등 |
| `operator` | 실험자 |
| `firmware_version` | `0.4.1` |
| `notes` | 재시도, 오작동, 환경 변화 등 |

## 재현성을 위해 함께 남길 것

- 판 크기·재질·두께와 지지 조건
- PZT 부착 위치, 방향, 접착 방식, 채널 번호
- 낙하 물체의 질량·재질·형상
- 높이 측정 기준과 낙하 방법
- 펌웨어 커밋 해시와 설정값
- 라이브러리 및 ESP32 보드 패키지 버전
- 실험 일시, 실험자, 이상 상황

실험 순서와 통과 기준은 [`../docs/TEST_PLAN.md`](../docs/TEST_PLAN.md)를 따릅니다.
