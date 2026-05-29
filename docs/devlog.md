# 개발 로그 (Devlog)

> 작업할 때마다 **날짜·시간 + 무엇을 했는지**를 위에서부터 최신순으로 기록합니다.
> 형식: `## YYYY-MM-DD HH:MM (KST) — 한 줄 요약` + 상세 내용.

---

## 2026-05-29 17:04 (KST) — 음속 돌파/형상/EDF 한계 추가 반영

**문서 반영**
- [docs/2026-05-29_0248_v1_formulas_and_algorithms.md](2026-05-29_0248_v1_formulas_and_algorithms.md)에 추가사항 섹션 작성.
  - 마하 1 근처 동압/항력 식.
  - 천음속 drag rise, 충격파, CP-CG, 노즈콘, 핀, flutter, inlet 문제.
  - EDF가 저속 GNC 연습용이고 마하 1 추진체로는 부적합하다는 분리 원칙.
- [docs/hardware.md](hardware.md)에 Phase 3 추가사항 작성.
  - 음속 돌파용 형상 체크리스트.
  - EDF 한계 표.
  - `EDF = 저속 TVC/GNC/착륙 연습용`, `마하 1 = 별도 고속 비행체 설계 문제`로 명시.

**의사결정**
- 현재 EDF 착륙 시뮬레이션은 유지한다. 이 실험은 저속 GNC/착륙 제어 검증용으로 유효하다.
- 마하 1 목표는 같은 기체/같은 EDF 추진체의 자연 확장이 아니라, 별도 공력·구조·추진 트랙으로 분리한다.

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

## 2026-05-27 14:23 (KST) — RL 진입: Gymnasium 환경 + PPO 학습 파이프라인

**환경 설치 (Python 3.14, arm64)**
- gymnasium 1.3.0 / torch 2.12.0 / stable-baselines3 2.8.0 설치 성공. `pip install -e .` (editable, SubprocVecEnv 워커 import용).

**구현한 것**
- `src/rocketsim/envs/landing_env.py` — `LandingEnv`(Gymnasium). 관측 13차원(pos/vel/**body-z**(자세, 쿼터니언 double-cover 회피)/omega/thrust), 행동 3차원(throttle+gimbal, [-1,1]). 50Hz 제어 + 500Hz 서브스텝(김벌 레이트제한·외란 적용). SB3 `check_env` 통과.
  - `command_to_action()` 제공 → **PID도 동일 env에서 평가**(공정 비교).
- `scripts/evaluate.py` — PID 또는 SB3 모델을 동일 env에서 평가(성공률/접지속도).
- `scripts/train.py` — PPO(MlpPolicy 256x256, 8 env SubprocVecEnv) / SAC 옵션, 체크포인트(100k마다).

**보상 설계 (중요)**
- 1차 시도(per-step 음수 페널티 + sparse +100): **die-fast 함정** — 빨리 추락해 페널티 누적을 끝내는 게 이득이 됨. ep_rew -130대 정체(150k 후 -92로 겨우 상승). 폐기.
- 교체: **포텐셜 기반 shaping** `r = γΦ(s')−Φ(s) + terminal`. `Φ=-(1.0·dist+0.4·speed+2.0·tilt)` → 패드로 다가가고 감속·직립할 때만 보상(hover-farming/die-fast 모두 차단). terminal: soft +100, hard −(10~60), crash −100, timeout −20.

**기준선 (동일 env @ 50Hz)**
- **PID hard = 60%** (29~30/50). 500Hz의 58%와 일치 → RL이 넘어야 할 목표.

**진행 중 / 디버깅**
- PPO hard 1차 학습: ep_rew −130 → +158로 상승했으나 **평가 0/50 착지** — 진단 결과 **공중 호버 farming** 학습(z≈8~10m에서 vz≈0으로 영원히 호버→타임아웃).
- 원인: 포텐셜 shaping `r=γΦ'−Φ`에서 γ=0.99, Φ 상시 음수(~−9) → **고정점 per-step 보너스 `(γ−1)·Φ=+0.09/스텝`**. 1750스텝 호버=+157 > 착지(+100). 내가 만든 함정.

## 2026-05-27 14:36 (KST) — 보상 재설계 #2 (호버 farming 제거) + RL 재학습

- shaping을 **`Φ'−Φ`(γ=1, 포텐셜 차분)** 로 변경 → 고정점 shaping=0(호버 보너스 소멸) + 작은 step 페널티(0.05)로 dithering 억제.
- 검증(새 보상): 호버 모델 return **−104.6**, PID(착지) **+7.6**(5/10 soft) → 착지가 명확히 최적. 보상 수정 확인.
- PPO hard 3M 재학습 시작. ep_len이 1750→300~500으로 떨어지면 착지 학습 신호. 목표: hard에서 PID 60% 돌파.

## 2026-05-27 15:03 (KST) — RL 학습 난항 4종 + 커리큘럼 도입

PPO가 hard 착륙을 못 배우고 막힌 지점들(전부 고전적 RL 실패):
1. **die-fast** (per-step 음수 페널티) → 포텐셜 shaping.
2. **hover-farming** (γ<1 고정점 보너스 `(γ−1)Φ>0`) → `Φ'−Φ`(γ=1)+step penalty.
3. **위험회피 호버** (이진 종단보상 → 그래디언트 없음) → **등급제 종단보상**(착지 품질에 단조, 어떤 착지든 호버보다 우위). 검증: PID +26 vs 호버 −154.
4. **종단 기술 미발견** (12m에서 정밀 연착륙을 무작위로 못 밟음 → calm조차 호버) → **역+난이도 커리큘럼**.

핵심 통찰: 진짜 병목은 알고리즘이 아니라 **soft-touchdown의 발견**. 12m 시작에선 거의 못 밟아봐서 calm에서도 호버 끌개에 빠짐.

**대응 (`scripts/train_curriculum.py`)**: `init_scale`(시작 고도/외란 스케일) 추가. 4단계로 **쉬움→어려움** 웜스타트: ①calm scale 0.12(패드 2m 위) ②calm 0.45 ③moderate 0.75 ④hard 1.0. 결과는 다음 항목에.

## 2026-05-27 15:19 (KST) — 첫 PID vs RL 결과: PID 우세 (hard)

커리큘럼(calm→hard, 2.5M 스텝) 학습 성공 — 호버 끌개 탈출, ep_rew Stage1 +32 → Stage4 +29로 수렴.
최종 평가(hard, n=100, 동일 env @ 50Hz):

| 지표 | PID | RL(PPO, 커리큘럼) |
|---|---|---|
| **soft landing** | **60%** | **10%** |
| 접지 도달 | 100% | 100% (호버/크래시 0) |
| 접지 수직속도(평균) | ~0.09 m/s | 0.84 m/s |

RL 실패 원인(분해, n=80): **offset 61 / hspeed 55** / tilt 19 / vspeed 13.
- RL은 "부드럽게·직립으로 내려 닿기"는 학습했으나 **패드 정밀 중심정렬 실패**(평균 오차 0.88m, 횡속도 0.67).
- PID는 **착륙 게이트(중심·직립 잡힐 때까지 호버 후 하강) + 적분항** 이라는 구조적 사전지식으로 정밀 착지를 공짜로 얻음. RL은 그 "인내" 전략을 발견 못 함 + graded 보상이 near-miss에도 +를 줘서 정밀도 인센티브가 약함.

**결론(잠정):** 이 hard 과제에서 **PID(60%) > RL(10%)**. 평형 근처 + 강한 사전지식이 통하는 영역에서 고전 제어가 더 설득력 있다는 처음 가설을 실증. RL을 더 끌어올리려면 정밀도 보상 강화(offset/hspeed 계수↑, soft 보너스↑) + 재학습 필요.

## 2026-05-27 16:34 (KST) — RL 정밀도 강화 + 안정화: 10% → 27%

정밀도 보상으로 RL을 끌어올리는 과정에서 추가 난항·해결:
- **정밀도 보상 과다** (offset/hspeed 계수 55, soft +80): calm 단계조차 ep_rew −114로 발산 → 계수 30·soft +50로 완화.
- **파국적 붕괴(catastrophic collapse)**: calm에서 ep_rew +4까지 학습 후 −157로 폭락. PPO의 과도한 정책 업데이트. → **target_kl(신뢰영역)** 도입. 0.03은 붕괴는 막았으나 학습 과도 억제(−30 정체) → **0.06 + LR 3e-4**로 균형.

안정화된 커리큘럼(calm→hard, 2.6M, target_kl=0.06) 진행: ep_rew Stage1 −40 → Stage3(moderate) **+62** → Stage4(hard) +30~47 수렴. **붕괴 없이 완주.**

**최종 평가 (hard, n=100, 동일 env):**

| 지표 | PID | RL v1 | **RL v2(정밀도+안정화)** |
|---|---|---|---|
| soft landing | **60%** | 10% | **27%** |
| 접지 도달 | 100% | 100% | 100% |
| 접지 수직속도(평균) | ~0.09 | 0.84 | 0.63 |

**결론:** 정밀도 보상 + 안정화로 RL **10%→27% (2.7배)**. 그러나 여전히 **PID(60%) > RL(27%)**.
- 평형 근처 + 강한 물리 사전지식이 통하는 이 과제에선 고전 제어가 우세 — 처음 가설 재확인.
- RL은 보상설계·탐험·학습안정성을 하나하나 잡아야 했고(난항 6종), PID는 그게 전부 불필요했음.
- 추가 향상 여지: SAC(표본효율), 더 긴 학습, 잔차(residual) RL(PID 위에 RL 보정), 정밀도 보상 추가 튜닝.

RL 트랙 1차 마무리. RL이 *이기는* 영역(모델 미지·곡예기동)은 별도 실험 필요.

## 2026-05-27 18:46 (KST) — 실험 (A) 기억(frame stack) 추가: 가설 검증됨, 성공률은 소폭

"RL이 PID보다 *더 잘* 날 수 있나?"를 정직하게 시험하기 위해 **frame stacking(n_stack=4)** 추가 — 정책이 최근 4관측의 변화로 **보이지 않는 바람을 추론**해 선제 대응 가능. (잔차 RL은 PID보다 낮게 정체해 폐기, 모방학습은 보류.)

`LandingEnv(n_stack=K)` 내장(train/eval 공통). 같은 커리큘럼+target_kl로 학습 → `ppo_mem_hard.zip`.

**평가 (hard, n=100):** soft landing **31%** (기억없는 RL 27% → 시드 변동 범위 내 소폭). 그러나 **실패 양상이 가설대로 변함:**

| 접지 평균 | 기억✗ | 기억✓ |
|---|---|---|
| offset | 0.88 m | **0.60** (−32%) |
| hspeed | 0.67 | **0.53** (−21%) |
| tilt | 5.4° | 6.3° |
| vspeed | 0.85 | 1.12 (한계 초과) |

**결론:** 기억은 **예측한 차원(바람성 수평 드리프트)에서 측정 가능하게 더 잘 날게 했음**(offset −32%, hspeed −21%). 단 RL이 보상 총합을 최적화하며 **수직속도를 희생** → 엄격 4기준 성공률은 소폭(27→31%). 종합으로는 여전히 **PID 60% > RL 31%**.
- "RL이 더 잘 나는가?" → *특정 능력(외란 대응)*은 yes, *종합 성공률*은 아직 no.
- PID 60% 천장의 상당 부분은 작동기 한계(김벌 ±12°·추력지연)상 **누구도 못 막는 접지순간 돌풍**일 가능성 — 기억으로도 안 메워짐.
- 전환 가능성: vspeed 보상 가중↑로 재균형하면 기억의 수평 이득을 성공률로 환산 가능(미시도).

**PID vs RL 최종 비교 (hard):** PID 60% / RL(scratch) 10% / RL(정밀+안정) 27% / RL(+기억) 31%.

## 2026-05-27 20:01 (KST) — 천장 진단: 작동기 아님 = 전략 한계 (RL 여지 있음)

"왜 60%를 못 넘나"를 측정으로 확정 (`scripts/ceiling_test.py`, PID @ hard, n=100):

| 작동기 설정 | PID 성공 |
|---|---|
| stock (김벌12°/rate200/tau0.08) | 53% |
| 김벌 30° | 53% |
| 김벌 rate 2000°/s | 53% |
| 추력지연 tau 0.02 | 53% |
| 전부 이상화 | 52% |

작동기를 **풀어도 변화 없음** → 검증(반대로 조이면 급락: 김벌2°=10%, tau0.3=32%, stock=57%)으로 오버라이드 정상 확인.

**결론: 60% 천장은 작동기 힘 부족이 아니라 *전략/관측* 한계.** 무한 권한을 줘도 고정 게이트는 53% — 접지 순간 돌풍을 *예측·타이밍*하지 못해서. → **더 똑똑한 정책(RL+기억)이면 위로 갈 여지 존재.**

**대응:** 기억 모델이 수평은 개선했으나 vspeed를 희생(1.12)한 걸 막도록 종단보상 재균형(vspeed 25→40, offset 30→40, soft +50→+60). target_kl로 안정성 유지하며 기억+커리큘럼 재학습 중.

## 2026-05-27 20:19 (KST) — VecNormalize로 스케일 붕괴 해결 → RL 50% (PID에 근접!)

**핵심:** 정밀도 보상의 반복 발산은 *보상 공식*이 아니라 *스케일* 문제였음(큰 보상 → critic 목표 폭증 → 붕괴). `VecNormalize(norm_reward=True)`로 PPO를 스케일 무관하게 만들자, 정밀도 보상이 **발산 없이** 학습됨(Stage1 +20 → Stage3 +54 → Stage4 +50 수렴).

**평가 (hard, n=100, 동일 시드):** soft landing **50%** (이전 31% → 50%!), 접지 vspeed 0.44(이전 1.12에서 개선).

**RL 진행: 10% → 27% → 31% → 50%.** 동일 시드 PID는 53% → **거의 동률.**

남은 실패(50% 모델): offset 35(평균 0.44, 한계 0.5 *바로 밑* 꼬리), tilt 19, hspeed 13, vspeed 3, 크래시 3. vspeed·hspeed는 사실상 해결, **남은 병목은 바람성 수평 드리프트.**

50% 모델 백업: `models/ppo_mem50.zip`. 최종 푸시(기억 4→6프레임 + offset 가중 소폭↑ + hard 단계 연장)로 PID 53% 돌파 시도.

## 2026-05-27 20:24 (KST) — 최종 푸시는 역효과 → 50% 모델을 확정 결과로 고정

최종 푸시(n_stack 6 + offset 48/tilt 70 + hard 1.8M)는 **오히려 나빠짐**: stage1이 −40(착지)까지 갔다가 **호버로 서서히 드리프트**(ep_len 44→1000). 더 가혹한 가중치 + 더 큰 관측이 50% 균형을 깨고 호버 끌개로 미끄러짐. (target_kl이 *붕괴*는 막았지만 *드리프트*는 못 막음.)

→ 마진 튜닝이 역효과 구간. **50% 모델(n_stack=4)을 확정**: `ppo_mem50.zip` → `ppo_mem_hard.zip`. 스크립트도 50% 설정으로 되돌림.

### RL 트랙 최종 결론 (hard, 동일 시드 n=100)

| 컨트롤러 | soft landing |
|---|---|
| **PID** (물리 사전지식) | **53%** |
| RL 맨바닥 | 10% |
| RL +정밀보상+안정화 | 27% |
| RL +기억(4) | 31% |
| **RL +기억+VecNormalize+재균형** | **50%** |

- **RL이 10% → 50%로 PID(53%)에 거의 동률까지 따라붙음.** 단, *맨바닥에서* 그 구조를 재발견하느라 보상 6종 디버깅 + 커리큘럼 + 안정화(target_kl, VecNormalize) + 250만 스텝이 들었음.
- 천장 진단: 60%대 천장은 **작동기 아닌 전략/관측 한계**(기억이 offset −32%로 일부 메움).
- **종합 메시지:** 잘 모델링된 평형 문제에서 (a) 고전 제어(PID)는 즉시·견고·해석가능하게 ~53%, (b) RL은 막대한 공을 들여 *비슷한 수준*에 도달 — "도구는 문제에 맞게." RL의 진짜 우위는 모델 미지·곡예 등 미탐색 영역.

## 2026-05-27 21:39 (KST) — 실험 (B) 모델 미지: 가설 반전 — PID가 *더 강함* (97%)

"모델을 모르면 PID가 무너지고 RL이 이긴다"를 시험하려 `model_unknown` 프리셋 추가: 질량 ×0.6~1.8, **관성 ×0.5~2.0**(고정 자세게인 어긋남), TWR 1.4~2.6(호버 스로틀 미지), CG/오정렬 큼, 외란은 약하게(모델 불확실성만 격리). (`Randomization.twr_range`로 TWR 실현가능 보장.)

**측정: PID @ unknown = 97%** (hard의 53%보다 *높음*). 가설 완전 반전.

**왜:** **적분 제어(integral action)가 모델 불확실성을 자동 흡수.** 질량/추력 몰라도 적분이 호버 스로틀을 찾고, 관성 4배여도 게인여유로 수렴. 게이트는 모델 무관. → **"모델 몰라도 견고"는 피드백 제어의 존재 이유.** 모델 불확실성은 PID가 *가장 잘하는* 영역.

**3개 영역 종합:** calm 100% / hard(난기류) 53% / unknown 97%. → PID를 무너뜨리는 건 **모델 불확실성이 아니라 예측 불가 외란**(접지 돌풍). RL도 그건 못 막음.

**결론 갱신 — RL이 PID를 *이기는* 영역은 모델 미지가 아니라:** 좋은 제어구조를 못 쓰는 강한 비선형/대각도, 곡예 회복(거꾸로→복구, 헬리콥터 RL 본무대), 복잡한 목적, raw 센서 end-to-end. (미탐색)

## 2026-05-27 21:47 (KST) — 실험 (C) 곡예 회복: 또 반전 — PID 96%

`recovery` 시나리오 추가(시작 기울기 70~140°=옆~거꾸로, 고도 12~16m, 약한 외란). 버그 1건 수정(시작 틸트>45° 크래시 임계에 즉시 걸림 → 회복 중 틸트-크래시 비활성화, `crash_tilt_deg=200`, 착륙 자세로만 판정).

**측정: PID @ recovery = 96%** (착륙 시 평균 기울기 2.9°). 거꾸로에서 시작해도 복구해 똑바로 착륙.

**왜:** HoverPID의 자세 제어는 소각도가 아니라 **기하학적 대각도 컨트롤러**(err=body_z×z_des)이고, 각가속 여유 α≈L·T/I≈90 rad/s²로 거꾸로→직립 전환이 충분히 빠름 + 12~16m 여유. → 대각도 회복도 피드백이 잘 처리.

**4영역 종합 (PID):** calm 100% / 모델미지 97% / 곡예회복 96% / **난기류(외란) 53%**. → PID(피드백)를 무너뜨리는 유일한 요인은 **예측 불가 외란**뿐. 모델·비선형·대각도는 전부 강함. 이 문제군 자체가 피드백 제어에 매우 적합 → RL이 *전략적으로 파고들 틈이 없음*(그래서 잘해야 동률).

## 2026-05-27 22:07 (KST) — 실험 (D) 센서 노이즈/부분관측: PID가 드디어 무너짐

컨트롤러가 *참값* 대신 **노이즈 측정값**만 보게 함(`LandingEnv.obs_noise`): 위치±5cm·속도±0.2m/s·각속도±0.08rad/s·자세±1.5° ×배율. PID·RL 모두 측정값 사용, 성공 판정은 참값으로.

**PID 노이즈 스윕 (@moderate):** 노이즈 0배 100% / 1배 99% / **2배 21%**.
- 적분 구조는 *적당한* 노이즈는 흡수(1배 99%)하지만, **2배에선 미분(감쇠)항 `kd·vel`,`kd_att·omega`이 노이즈를 증폭 → 떨림 → 21% 붕괴.** PID엔 추정기(필터)가 없음.
- → **드디어 PID가 *전략적으로* 무너지는 영역.** RL은 기억(프레임 스택)으로 노이즈를 시간평균해 추정 가능 = 내장 추정기.

**대응:** `noisy` 난이도(moderate 외란 + 노이즈 2배). 커리큘럼을 init+노이즈 동시 램프(0→2배)로 재구성, 기억 6프레임(노이즈 평균화). RL 학습 중 → PID 21% 돌파가 목표. (연료/에너지 최적도 후속 후보 — RL은 복합목적 직접 최적화 가능.)

## 2026-05-27 22:21 (KST) — 🎯 RL이 드디어 PID를 이김: 노이즈 2배에서 67% vs 21%

노이즈 커리큘럼(init+노이즈 0→2배 램프, 기억 6프레임, VecNormalize+target_kl) 학습 완료. ep_rew: stage3(노이즈1.3) +88 → stage4(노이즈2.0) +74~88 유지(붕괴 없음).

**평가 (noisy = moderate 외란 + 센서노이즈 2배, n=100):**

| | soft landing |
|---|---|
| PID (필터 없음) | **21%** |
| **RL (기억 6프레임)** | **67%** |

**RL 3.2배 승리.** 기억이 **학습된 추정기(필터)** 역할 → 노이즈를 시간평균해 참상태 추정. PID는 미분(감쇠)항이 노이즈 증폭 → 떨림.

**전체 PID vs RL 지형 (확정):**
| 영역 | PID | RL | 승자 |
|---|---|---|---|
| calm | 100% | ~100% | 동률 |
| 모델 미지 | 97% | — | PID(적분) |
| 곡예 회복 | 96% | — | PID(기하 제어) |
| 강한 외란 | 53% | 50% | 동률(물리 천장) |
| **노이즈 센서(부분관측)** | **21%** | **67%** | **RL** ✅ |

**결론:** RL이 고전 제어를 이기는 곳은 **부분관측/나쁜 센서** — 추정+제어를 end-to-end로 학습(기억=내장 필터). 공정성: PID+EKF(추정기)도 회복 가능하나 *우리가 설계*해야 함; RL은 기억에서 공짜로 얻음.
**실세계 연결:** 실제 센서는 노이즈투성이 → 이게 RL의 실질적 가치(벤치마크 승리가 아니라 "실센서에서 도는 것"). 다음: sim 충실도↑(지연·실추력곡선) + 상태추정기, 실기 배포.

## 2026-05-28 00:58 (KST) — MPC 1차 (수직 throttle): calm 연료 −34%, hard 붕괴

샘플링 슈팅 MPC(`controllers/mpc.py`, 솔버 불필요): `SampledVerticalMPC`가 **수직 throttle만** 짧은 수평선으로 최적화, 자세/수평은 HoverPID. `LandingVerticalMPC`로 결합. `scripts/evaluate_mpc.py`로 PID vs PID+MPC 비교.

| 환경 | | 성공 | 에너지 | offset |
|---|---|---|---|---|
| calm | PID | 100% | 5.49 | 0.02 |
| | +수직MPC | 87% | **3.62(−34%)** | 0.27 |
| hard | PID | 60% | 12.79 | 0.33 |
| | +수직MPC | **0%**(17 타임아웃) | 10.29 | 0.91 |

- **calm: MPC 연료 −34%** — 제어비용 직접 최적화(연료최적 강하)의 강점 확인. 단 빠른 강하 → 수평 정렬 시간 부족(offset↑).
- **hard: 0% 붕괴** — 수직 throttle만 보고 수평/외란 무시 → 바람에 횡으로 밀려 offset 0.91·hspeed 0.77, 게이트로 17 타임아웃.
- **결론:** 수직 전용 MPC는 잔잔할 때만 유효. 실제(외란/divert)엔 **추력 벡터(수평+수직 결합) 3-DOF MPC 필요.** → 다음: cvxpy로 SpaceX식 3-DOF convex powered-descent.

## 2026-05-28 17:44 (KST) — 적응형 EKF: hard 회귀 해소 (34→55) + noisy 85 유지

**검증:** 리팩토링은 결백(PID hard true state n=100 = 55%, 이전 53%와 동일). hard 34% 원인은 **EKF의 *고정* 필터 lag**(vel_tau=0.12s)이 노이즈 없는 입력에도 항상 적용되어 PID 미분항을 둔하게 만든 것.

**개선:** `LowPassEstimatorConfig.for_obs_noise(obs_noise)` 추가 — 필터 시간상수를 입력 노이즈에 비례 스케일. obs_noise=0 → scale=0 → passthrough; obs_noise=2 → scale=1 → 기존 필터.

**검증 결과 (n=100):**

| 영역 | 적응 전 | 적응 후 |
|---|---|---|
| PID + EKF @ hard (clean) | 34% | **55%** (PID 단독과 동률) |
| PID + EKF @ noisy (2× noise) | 85% | **85%** (유지) |

→ 비대칭 손실 없이 GNC 스택이 모든 영역에서 PID 단독을 *능가하거나 동률*. 노이즈에선 RL(67%)도 명확히 능가(85%). **미니 SpaceX 1차 베이스라인 확정.**

## 2026-05-29 14:26 (KST) — Actuator-aware MPC A/B 가중치 sweep: 가설 반증

**가설**: actuator-aware MPC가 짐벌 96% 포화시키는 원인이 `q_pos[xy]=1.2` / `q_final_pos[xy]=45` 가중치가 너무 공격적이라서다. 완화하면 짐벌 포화 줄고 성공률 오를 것.

**검정**: `q_pos[xy]`를 0.3 (A), 0.15 (B)로 4×/8× 완화. n=50 estimated 병렬(`--workers 6`), hard·noisy 모두 측정.

| controller   | hard | noisy | noisy timeout |
|--------------|------|-------|---------------|
| pid          | 58%  | 84%   | 0             |
| waypoint     | 52%  | 76%   | 3             |
| actuator     | 46%  | 74%   | 8             |
| actuator_a   | 46%  | 64%   | 10            |
| actuator_b   | 44%  | 58%   | 12            |

**결과**: 가중치 완화 방향으로 **단조 악화**. noisy 성공률 74→64→58%, timeout 8→10→12. 가설 반증.

**재해석**: 짐벌 포화는 비용 가중치 탓이 아니라 **점질량 모델 자체의 한계**. 점질량은 짐벌 회전 lag을 모르고 "즉시 횡가속"을 자유롭게 가정 → 외란이 들어오면 항상 강한 횡가속 명령 → 실제 자세 동역학은 한 박자 늦음 → 짐벌 풀로 일해도 못 따라감. 가중치를 낮추면 MPC가 덜 공격적으로 짜는 만큼 패드 도달 자체에 실패 (timeout↑).

**다음**: 다음 변형을 더 굴리는 것보다 **Step 2 (1차 자세 lag을 MPC 모델에 추가)** 로 가는 ROI가 크다는 결론. 자료: `docs/2026-05-29_1422_v1_actuator_ab_sweep.md` (raw), `docs/2026-05-29_1426_v1_actuator_ab_analysis.md` (분석·가설 위조 노트).

## 2026-05-29 15:30 (KST) — Step 2 (추력 크기 1차 지연) 추가: 개선됐지만 부족

**변경**: `CvxpyActuatorAwareMagLagMPC` + 래퍼 `LandingActuatorAwareMagLagWaypointPID` 추가. Step 1 (슬루 제약) 위에 ① 추력 크기 1차 지연 (`T[k+1] = a·T[k] + (1−a)·T_cmd[k]`, `a = exp(−dt/τ_spool) ≈ 0.082`, τ는 `vehicle.thrust_time_constant`=0.08s 와 매칭), ② 무손실 볼록 완화 `||u||₂ ≤ T[k]` (Açıkmeşe G-FOLD), ③ 비용 `r_Tcmd·(T_cmd−g)²` 로 hover 근처로 끌어당김. **여전히 점질량**. Step 3 이 점질량을 떠나는 첫 단계.

**결과 (n=50, estimated)**:

| controller | hard | noisy |
|---|---|---|
| pid | 58% | **84%** |
| waypoint | 52% | 76% |
| actuator (S1) | 46% | 74% |
| **actuator2 (S2)** | **54%** | **80%** |

Step 1 → Step 2 **hard +8pp, noisy +6pp**. 모든 landed_fail 카테고리에서 호전.

**오픈루프 진단 (`diagnose_mpc_model_mismatch.py`, n=20)**: 짐벌 포화 88~96% — *전혀 변동 없음*. 즉 Step 2 의 개선은 *계획이 더 실현 가능해졌기 때문이 아니라* `r_Tcmd` + lag 가 *참조 신호를 부드럽게* 만들어 PID 가 더 잘 따라간 효과로 해석.

**계획 §8 결정 기준**:
- 짐벌 포화 ≤ 30% ❌ (88~96%)
- PID 능가 ❌ (-4pp)
→ **Step 3 (SCP 6-DOF) 로 진행이 정답.**

자료: `docs/2026-05-29_1514_v1_step2_maglag_results.md` (raw), `docs/2026-05-29_1530_v1_step2_maglag_analysis.md` (분석).

## 2026-05-29 15:50 (KST) — divert 시나리오 추가 + baseline: PID/MPC 가 시나리오마다 *반대* 로 실패

**동기**: Step 1·2 가 hard/noisy 에서 PID 못 이긴 게 *MPC 가 못 한다* 인지 *시나리오가 PID-favorable* 인지 불분명. 저고도 유지 + 실제 hop 시나리오 가까이 + 위 두 변형 (Step 1 vs Step 2 vs Step 3) 의 차이를 부각시키기 위해 **`divert`** (moderate IC + t=2s 에 패드 +10m 이동) 와 **`divert_hard`** (hard IC + 같은 이동) 추가. 구현: `LandingScenario.pad_shift_time/pad_shift_delta_xy`, env step 안에서 *POS 좌표 원점 평행이동* (inertial frame 보존). 자세 설계: `docs/2026-05-29_1531_v1_divert_scenario_plan.md`.

**Baseline 결과 (n=50, estimated)**:

| controller | divert | divert_hard | hard | noisy |
|---|---|---|---|---|
| pid | **78%** | **18%** ⚠️ | 58% | 84% |
| waypoint | 42% | 58% | 52% | 76% |
| actuator (S1) | 16% | 56% | 46% | 74% |
| actuator2 (S2) | 20% | **54%** | 54% | 80% |

**핵심 발견 — 정반대 실패 모드**:
- **divert (mild)**: PID 압도 (78% vs MPC 16–20%). MPC 변형들은 *호버링→타임아웃* (29–42 timeout/50). H1 반증.
- **divert_hard**: PID 가 **37/50 too_high** (위로 박살남, windup) ↔ MPC 변형들 50/50 모두 착륙 (54–58% 성공). H1 *조건부* 검증.

**해석**: "PID 가 hard/noisy 에서 잘하는 것" 은 *외란 약함 + 단거리* 의 PID-favorable 환경이었기 때문. 큰 갑작스러운 오차 (divert + hard 외란) 가 들어가면 PID 의 windup 안전장치 부재가 catastrophic. MPC plan-기반은 *훨씬* 견고. 단 mild divert 의 hover-and-timeout 은 MPC wrapper 의 commit-to-descent 부재 문제로 추정 — Step 3 만으로 안 풀릴 수 있음.

**Step 1 vs Step 2**: divert 16 vs 20%, divert_hard 56 vs 54% — 또 노이즈 안 (McNemar p > 0.3). S1/S2 사실상 구분 불가 결론 유지.

자료: `docs/2026-05-29_1545_v1_divert_baseline_results.md` (raw), `docs/2026-05-29_1550_v1_divert_baseline_analysis.md` (분석·H1 부분 검증/반증).

**다음**: Step 3 (SCP 6-DOF) 구현 → 4 시나리오 × 5 컨트롤러 비교 → H2 평가.

## 2026-05-29 16:34 (KST) — MPC hover 버그 1줄 fix: `lookahead 4 → 10` → MPC 가 *처음으로* PID 능가

**진단** (`scripts/diagnose_hover_bug.py`): mild divert seed=0 의 시계열을 찍어 보니 — t=2.66s (패드 점프 직후), MPC plan **종착점**(`pend`)은 0.93m (≈ 패드, 정확함) 인데 wrapper 가 PID 에 전달한 xy 목표는 **9.12m** (= MPC plan 의 0.8s 시점). 즉 *MPC plan 자체는 멀쩡* — wrapper 가 plan 의 *너무 초반* 만 보고 PID 에 약한 신호 줘서 PID 가 살살 가다가 timeout.

**Fix**: `LandingCvxpyWaypointPID` (+ Step 1/2 wrapper) 의 `lookahead` 기본값 **4 → 10** (= 0.8s → 2s 앞 점). 한 줄.

**Sweep 결과**:

| lookahead | divert | divert_hard | hard | noisy |
|---|---|---|---|---|
| 4 | 16% | 56% | 46% | 74% |
| **10** | **90%** | 52% | **68%** | 78% |
| 19 | 88% | **32%** ⚠️ | 58% | 82% |

→ `10` 이 sweet spot. `19` 는 PID 단독처럼 즉시 패드 노림 → divert_hard 에서 too_high windup 재현 (22/50).

**전체 fix 후 (n=50, estimated)**:

| | PID | waypoint | actuator (S1) | actuator2 (S2) |
|---|---|---|---|---|
| hard | 58% | 54% | **68%** (+10pp) | 64% |
| noisy | **84%** | 72% | 78% | 72% |
| divert | 78% | 94% | 90% | **96%** ✨ |
| divert_hard | 18% ⚠️ | **64%** (+46pp) | 52% | 52% |

→ **MPC 가 3/4 시나리오에서 PID 능가** (divert +18pp, divert_hard +46pp, hard +10pp). noisy 만 PID 가 -6pp 우위 (외란 약함 → reactive PID 의 자연스러운 홈그라운드).

**의미**:
- 이전 "MPC 가 PID 못 이긴다" 결론은 *wrapper 버그* 때문이었음 — MPC 알고리즘이 아니라 통합 layer 의 문제.
- 이제 Step 3 (SCP 6-DOF) 의 *진짜* 효과를 측정할 strong baseline 확보.

**부수 토론** — `docs/2026-05-29_1653_v1_lookahead_vs_spacex_design.md`: 우리 `lookahead` 트릭과 SpaceX 식 *전체 trajectory time-indexed tracking* 의 차이. 우리가 `lookahead` 식으로 간 이유 = *검증된 PID 인터페이스 재사용* (한 점 setpoint 만 받음). 정통 trajectory tracking 은 1~2시간 작업으로 가능 — Step 3 의 plan 정확도 향상이 wrapper 한계에 막히는지 측정 후 결정.

자료: `docs/2026-05-29_1634_v1_hover_bug_fix.md` (진단+fix), `docs/2026-05-29_1634_v1_hover_bug_fix_results.md` (raw), `docs/2026-05-29_1653_v1_lookahead_vs_spacex_design.md` (architecture 비교).

## 2026-05-29 18:45 (KST) — 정통 SpaceX-식 stack 처음부터 구현 → 음의 결과 + 원인 정리

**작업**: `src/rocketsim/spacex/` 새 폴더로 *처음부터* G-FOLD-식 stack 구현. 5 파일 (convex_landing_mpc, trajectory_tracker, attitude_controller, landing_controller, __init__). 기존 PID/MPC 의존성 없음. 알고리즘:

1. **Convex MPC (G-FOLD)**: min-fuel `∑‖u‖` + running z + terminal landing. 제약: 글라이드슬로프 30° (★ 새로 추가), 틸트 콘, 추력 magmin/max, 강하속도 한계. **Shrinking horizon** — `N = ⌊(T_final − sim_t)/dt⌋`, T_final 고정.
2. **Trajectory tracker**: time-indexed `tau = t − plan.start_t`, (p_ref, v_ref, u_ff) 보간, PD+I (HoverPID 게인 재사용). 적분기 anti-windup. **No lookahead, no anticipation, no setpoint hack.**
3. **Attitude controller**: 분리된 클래스. 쿼터니언 PD on body-z alignment → 짐벌 inverse.

자세 알고리즘 + 수식: `docs/2026-05-29_1830_v1_spacex_formulas_and_algorithms.md`.

**결과 (n=50, estimated)**:

| | PID | actuator (la=10) | spacex (default) | spacex (튜닝) |
|---|---|---|---|---|
| hard | 58% | **68%** | 6% | **0%** ⚠️ |
| noisy | **84%** | 78% | 22% | 12% |
| divert | 78% | **90%** | 4% | 16% |
| divert_hard | 18% | **52%** | 6% | 0% |

→ **모든 시나리오에서 *훨씬* 못 함**. 가설 반증.

**원인**:
1. **Hoverslam 패턴 + actuator lag**: MPC 가 coast-then-burn plan 짜지만 EDF thrust τ=0.08s + 짐벌 200°/s 한계로 마지막 burst 가 *늦게* 도달 → touchdown vspeed 3~5 m/s (한계 1.0 의 5배 충돌).
2. **단거리에서 long-horizon advantage 부재**: 5-15m 강하에선 MPC 의 *long look-ahead* 효과 미미. PID 의 reactive 가 충분.
3. **글라이드슬로프 vs divert 충돌**: 패드 +10m 점프 시 글라이드슬로프 30° 안으로 끌어오려고 *위로* 끌어올림 → too_high / out_of_bounds.
4. **PID integrator + landing gate 의 *bundled* 디자인 강점**: 기존 wrapper 의 `HoverPID._z_int`, `LandingGuidance.touchdown_ready` 같은 게이트들이 *우리 EDF + 단거리* 환경에 *극도로 최적화* 되어 있었음.

**교훈**:
- "알고리즘 정통성 ≠ 우리 환경에서 성능". SpaceX 정통은 *그들* 시나리오 (Falcon 9, km 단위, 연료 결정적) 에 맞춤화. 우리 EDF testbed (단거리, TWR>1, hover 가능) 는 *PID+lookahead* 의 홈그라운드.
- **lookahead=10 트릭은 hack 이 아니라 *우리 use case 에 잘 튜닝된* 디자인**.
- 그래도 `src/rocketsim/spacex/` 는 *교과서 reference 구현* 으로 보존. Step 3 (SCP 6-DOF) 의 시작점으로 가치 있음.

자료: `docs/2026-05-29_1753_v1_spacex_style_design.md` (사전 설계), `docs/2026-05-29_1830_v1_spacex_formulas_and_algorithms.md` (수식·알고리즘 통합), `docs/2026-05-29_1830_v1_spacex_results.md` (raw), `docs/2026-05-29_1845_v1_spacex_negative_result.md` (음의 결과 + 원인).

**다음**: Step 3 (점질량 떠난 SCP 6-DOF) 또는 SpaceX-식 stack 에 actuator-aware constraint 포팅. 사용자 결정 대기.

## 2026-05-29 20:45 (KST) — SpaceX MPC + actuator-aware (Step 1·2) 포팅: 두 가지 의외 결과

**작업**: `ConvexLandingMPC` 에 옵션 파라미터 `slew_factor`, `tau_spool` 추가. 설정 시 `u` 를 state 로 승격 + `du` control + 슬루 SOC, T·T_cmd 추가 + 1차 지연 + `||u|| ≤ T` lossless conv. 편의 subclass `ActuatorAwareLandingMPC` / `ActuatorMagLagLandingMPC`. `LandingControllerSpaceX` 에 `planner_cls` 매개변수. eval 에 `spacex_actuator`, `spacex_actuator2` 추가. 부수 fix: descent-rate 제약을 k=0 에서 제외 + soft slack (hard IC 의 -3~-5 m/s 시작 vz 가 v_max_desc=2 와 strict infeasible 였음).

**결과 (n=50, estimated)**:

| | PID | actuator(la=10) | spacex(이전) | **spacex(now)** | spacex_actuator | spacex_actuator2 |
|---|---|---|---|---|---|---|
| hard | 58% | **68%** | 6% | **14%** | 10% | 8% |
| noisy | **84%** | 78% | 22% | **28%** | 10% | 14% |
| divert | 78% | **90%** | 4% | **32%** | 12% | 8% |
| divert_hard | 18% | **52%** | 6% | **24%** | 10% | 10% |

**의외 결과 1**: base spacex 가 descent slack fix 만으로 *부수적* 4-5배 개선. 이전 평가의 *대부분 실패* 가 *알고리즘 한계* 가 아니라 *제약 strict infeasibility* + fallback hover 였음. *진짜* SpaceX 식 성능은 *14-32%* 수준.

**의외 결과 2**: Step 1/2 actuator-aware 가 base 보다 *모든 시나리오에서 나빠짐*. Touchdown 분해 결정적 — Step 1/2 가 *지면 도달율* 은 올렸지만 (14→29 in hard) *soft 비율* 폭락 (50% → 14%). 부드러운 plan + 약한 inner-loop (PID integrator/gate 부재) = 둔한 거동 + 정밀 commit 실패.

**진짜 메커니즘**: plan 정확도 향상의 *방향성* 이 *inner-loop 강도* 에 따라 반대. PID-bundled (la=10) 위에선 약간 + 효과 (+6-8pp), spacex stack 위에선 - 효과 (-4-20pp). `actuator(la=10)` 의 wrapper 가 *우리 EDF + 단거리* 에 *극도로 최적화*.

**최종 평가**: SpaceX 식 stack 은 *구조적으로 깔끔* 한 reference 로 보존. 실용 baseline 은 여전히 `actuator (la=10)`. Step 3 (SCP 6-DOF) 진행 시 *이미 강한* lookahead+PID wrapper 위에서 모델 정확도 효과 측정 권장.

자료: `docs/2026-05-29_1903_v1_spacex_actuator_results.md` (raw), `docs/2026-05-29_2045_v1_spacex_actuator_analysis.md` (분석).

<!-- 새 항목은 이 줄 위에 추가 -->
