# 개발 로그 (Devlog)

> 작업할 때마다 **날짜·시간 + 무엇을 했는지**를 위에서부터 최신순으로 기록합니다.
> 형식: `## YYYY-MM-DD HH:MM (KST) — 한 줄 요약` + 상세 내용.

---

## 2026-05-27 11:55 (KST) — 프로젝트 킥오프 & 6-DOF 시뮬레이터 + PID 호버 베이스라인

**목표 정리 / 의사결정**
- 6개월 후 마하 1, 발사~역추진 착륙, EDF로 시작, PID↔RL 비교 후 최종 RL.
- 물리 검토 결론: **EDF로는 마하 1 불가**(공기 흡입식, 최고 ~200–400 km/h). 마하 1은 고체 로켓 모터 영역.
  → 프로젝트를 **제어 개발(EDF 호버 테스트베드)** 과 **마하 1 비행**으로 분리. 초음속 동력 역추진은 6개월 범위 밖으로 명시.
- 결정: **시뮬레이션 우선**, **6-DOF 시뮬레이터부터** 시작. 하드웨어는 설계 문서로 병행([docs/hardware.md](hardware.md)).

**구현한 것**
- 가상환경 `.venv` + numpy 2.4.6 설치 (Python 3.14.4, arm64).
- `src/rocketsim/quaternion.py` — 쿼터니언 회전/미분/오일러/틸트 유틸.
- `src/rocketsim/vehicle.py` — 차량·환경 파라미터 (EDF 테스트베드 기본값: 2.5 kg, 추력 40 N, 김벌 ±12°).
- `src/rocketsim/dynamics.py` — 14차원 상태 6-DOF 강체 동역학 + RK4. 추력 스풀업 1차 지연, 김벌 추력편향 토크, 2차 항력 모델 포함.
- `src/rocketsim/simulator.py` — 고정 스텝 폐루프 시뮬레이터 + 김벌 레이트 제한 + 궤적 기록.
- `src/rocketsim/controllers/pid.py` — 캐스케이드 PID(위치/고도 PD → 목표 추력벡터 → 자세 PD → 김벌 역산).
- `scripts/run_hover.py` — 호버 데모 (CSV 출력 + 요약).
- `tests/test_dynamics.py` — 자유낙하/호버/김벌 토크/쿼터니언 단위노름 검증.
- `pyproject.toml`, `README.md`, `docs/` (roadmap, hardware, devlog).

**결과 (검증됨)**
- 단위 테스트 5개 전부 통과.
- 호버 데모: 시작 틸트 12.8°·위치 오차 1.8 m → **3.12 s 만에 오차 0.9 cm, 틸트 0.00°** 로 안정화. 평균 스로틀 0.62.

**다음 작업 후보**
- [ ] Gymnasium 환경(`envs/hover_env.py`) 작성 → RL 학습 인터페이스.
- [ ] 착륙(역추진) 시나리오 추가: 하강 후 지면 근처 0 속도 터치다운.
- [ ] PID vs RL 평가 지표 정의(정착 시간, 연료/에너지, 강건성).
- [ ] 시각화(matplotlib 또는 간단한 3D 플롯).
- [ ] EDF 추력 곡선/지연을 실측 데이터로 교정.

---

## 2026-05-27 12:06 (KST) — 착륙(역추진) 시나리오 + PID 착륙 베이스라인

**개념 정리 (왜 시나리오 먼저인가)**
- 착륙 **시나리오 = 과제(task) 정의** (시작 조건·성공/실패 기준·보상·평가지표). PID와 RL이 **공유**하는 공통 기반.
- RL은 환경(Gym)이 필요하고 그 **보상 함수 = 시나리오** → 시나리오 없이는 RL 정의 불가. 그래서 시나리오 + PID 베이스라인 먼저, 그 위에 RL.

**구현한 것**
- `src/rocketsim/scenarios/landing.py` — `LandingScenario`: 초기조건 샘플링(8–12 m 상공, 하강 중, 약간의 오프셋/틸트), 종료(접지/추락/이탈/타임아웃), 연착륙 성공 기준(접지 수직속도<1, 수평속도<0.5, 오프셋<0.5 m, 틸트<8°), RL용 보상함수, 궤적 평가(`TouchdownResult`).
- `src/rocketsim/controllers/landing.py` — `LandingPID`: HoverPID 재사용 + 하강 유도(고도 setpoint를 점차 낮추고 지면 근처에서 감속하는 flare).
- `src/rocketsim/simulator.py` — `run()`에 조기 종료 콜백 `terminate(t,state)` 추가(접지 시 정지, RL 롤아웃에도 활용).
- `scripts/run_landing.py` — 상세 1회(out/landing.csv) + 랜덤 50회 배치 성공률.

**결과 (검증됨)**
- seed=0: 8.52 s에 연착륙, 접지 수직속도 0.21 m/s, 오프셋 0 cm, 틸트 0°. ✅
- 랜덤 50회: **연착륙 50/50 (100%)**, 평균 접지 수직속도 0.21 m/s. 기존 단위 테스트 5개 유지.

**메모 / 다음**
- 현재 시나리오는 PID가 100% 성공 → **베이스라인은 확보**됐지만 난이도가 낮음. RL 비교가 의미 있으려면 난이도 상향 필요: 바람/외란, 센서 노이즈, 부분 관측, 연료/에너지 최소화, 추력 한계 빡빡하게.
- [ ] Gymnasium 환경(`envs/landing_env.py`)으로 시나리오 래핑 → RL 학습.
- [ ] PID vs RL 공정 비교표(성공률·접지속도·에너지·강건성).

## 2026-05-27 13:19 (KST) — GitHub 저장소 연동

- `git init` 후 원격 `https://github.com/seyoung4503/rocket.git`(기존 README만 있던 빈 저장소)에 연결.
- 원격 Initial commit 위에 로컬 작업을 얹어 히스토리 선형 유지. 프로젝트 README로 대체.
- `main` 브랜치 푸시 완료(커밋 `08aabb6`). `.venv/`, `out/`, `__pycache__/`는 `.gitignore`로 제외.

## 2026-05-27 14:03 (KST) — 난이도 상향(외란·도메인 랜덤화) + PID 난이도 곡선

**개념 (사용자 Q&A)**
- "헬리콥터 세우기"는 **고전 제어 벤치마크**지 RL 전용 문제가 아님. 호버/착륙=평형 근처 (준)선형 문제라 PID/LQR이 보통 RL과 대등하거나 우위. RL이 빛난 건 호버가 아니라 곡예기동/시연학습.
- RL은 아직 **미착수**. 단 `reward()`/`check_done()`/`DisturbanceModel`/`Randomization`은 RL 환경의 재료로 이미 작성됨.

**구현한 것**
- `vehicle.py` — `side_area`(크로스윈드 면적), `thrust_misalign`(상수 추력/CG 오정렬=상시 토크 외란) 추가.
- `dynamics.py` — 항력을 **대기 상대속도**(v-wind) 기준으로, `external_force`(돌풍 하중) 주입, 오정렬 반영.
- `scenarios/disturbances.py` — `DisturbanceModel`(평균풍+OU 돌풍+랜덤력), `Randomization`(질량/추력/오정렬/CG 에피소드별 랜덤). 프리셋 `calm/moderate/hard`.
- `simulator.py` — 외란 주입(에피소드마다 reset, 스텝마다 wind/force 적용).
- `controllers/pid.py` — **수평 위치·자세 적분항** 추가(정상 바람·오정렬 제거), 게인 상향, anti-windup.
- `controllers/landing.py` — **착륙 게이트**: 중심·수직·저속이 아니면 하강을 늦추고, **고도가 낮을수록 더 빡세게**(gust-pushed 접지 방지). creep로 최소 하강 보장.
- `scripts/compare_difficulty.py` — calm/moderate/hard 성공률 + 실패원인 분해. 컨트롤러엔 **공칭 모델만** 주고 시뮬엔 **랜덤화 기체**(모델 불일치).

**결과 (검증됨, n=50)**
| 난이도 | 성공률 | 접지 vspeed | 비고 |
|---|---|---|---|
| CALM | 100% | 0.17 | 크래시 0 |
| MODERATE | **98%** | 0.11 | PID가 현실 외란을 설득력 있게 처리 |
| HARD | 58% | 0.09 | 실패=접지순간 돌풍(offset 14·tilt 12·hspeed 6), 타임아웃/수직속도 0 |

- 튜닝 교훈: hard에서 게이트 과도하게 빡세면 타임아웃(8%), 느슨하면 off-nominal 접지(45%) → 균형점 58%. 자세 강성 추가로는 한계 → **고정 게인 PID의 본질적 한계(접지 순간 무작위 돌풍 예측 불가)**. 여기가 RL 동기.

**다음**
- [ ] Gymnasium 환경(`envs/landing_env.py`)으로 시나리오 래핑 → RL(hard에서 PID 58% 돌파 목표).
- [ ] PID vs RL 공정 비교(동일 시나리오·외란·랜덤화).

<!-- 새 항목은 이 줄 위에 추가 -->
