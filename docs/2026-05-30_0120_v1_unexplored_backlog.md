# Unexplored Work — Master Backlog Checklist

**작성:** 2026-05-30 01:20 KST  
**버전:** v1  
**목적:** 지금까지 *안 한* 작업의 정직한 체크리스트. 시뮬-아키텍처/제어 풀스택 측면에서 약 60-70 카테고리 중 *15-20* 만 시도. 미래 세션의 backlog. 우선순위는 *임팩트 추정 + 작업량* 기준.

**우선순위 표기**:  
🔥 = high impact, 잘 안 한 (= 다음 세션 후보)  
⚡ = 빠른 작업 (~수시간)  
🛠 = 큰 작업 (~수일)  
📚 = 학습/완성도 (당장 효과는 작음)

---

## ⭐ High-impact Top 7 — 다음 세션 후보

이 7가지가 가장 *적게 작업하고 효과 큰* 부분 — 그동안 *이상하게 빠진* 것들.

- [ ] 🔥⚡ **LQR / LQG baseline** — 정통 *선형 최적 제어*. 우리는 PID 하고 MPC 까지 갔는데 *그 사이* 의 LQR 안 함. ~2-3시간. PID 의 D 항을 *최적* 으로 계산. 정통성 ↑↑.
- [ ] 🔥🛠 **MPC + RL Residual** — MPC plan 위에 RL 보정. Tesla/Waymo/SpaceX 추정 패턴. *기존 RL 인프라* 재사용 가능 (`scripts/train_curriculum.py`). 1-2일. **noisy + divert_hard 직접 효과**.
- [ ] 🔥⚡ **온라인 바람 추정 (Wind observer)** — 외란을 *상태로 augmentation* + Kalman 추정. plant 의 *unknown disturbance* 압축. ~반나절. **hard/divert_hard 핵심**.
- [ ] 🔥⚡ **자동 튜닝 (Bayesian Optimization)** — `optuna` 라이브러리로 MPC/PID 가중치 자동 sweep. 인간 손튜닝 한계 돌파. ~1일. **모든 시나리오 +5-10pp 가능**.
- [ ] 🔥⚡ **Dispatcher (Adaptive Controller Selection)** — 시나리오 감지 + best controller 동적 선택. ~1-2시간. 각 시나리오 best 얻기. 코드 단순.
- [ ] 🔥⚡ **재계획 빈도 sweep** (5Hz → 20Hz → 50Hz) — 외란 응답 속도 4-10배 ↑. cvxpy 솔브 시간 측정 후 결정. ~30분. **divert/divert_hard 직접**.
- [ ] 🔥⚡ **터미널 단계 게인 강화** — 마지막 1m 의 자세 PID 게인 2x. *touchdown but not soft* 실패 풀어줌. ~1시간. **모든 시나리오 +5-15pp**.

---

## A. 컨트롤러 아키텍처 (안 한 게 더 많음)

### A.1 MPC 변형

- [ ] 🔥 **Robust MPC** (uncertainty set 위 worst-case 최적화) — 외란 *uncertainty 직접 다룸*. 학술적 robust 의 정통. 2-3일.
- [ ] 🔥 **Tube MPC** — 명목 궤적 + tube (오차 한계 보장). 안전 인증 가능. 1-2일.
- [ ] 📚 **Stochastic MPC** — 외란 확률 분포 위 *기대값* 최적화. CVaR / chance-constrained 가능. 2-3일.
- [ ] 📚 **Multi-shooting MPC** — 구간 분할로 더 안정적 수렴. 1일.
- [ ] 📚 **Move-blocking MPC** — control horizon < prediction horizon 으로 계산량 ↓. 반나절.
- [ ] 📚 **Distributed MPC** — multi-vehicle 시나리오 대비. N/A 지금은.

### A.2 SCP 변형

- [ ] 📚 **Free-final-time SCP** — 시간 T_final 자체를 결정 변수. shrinking horizon 의 정통. arXiv 1811.10803. 1-2일.
- [ ] 🔥 **iLQR** (Iterative LQR) — 비선형 최적 제어의 *효율적* 대안 SCP. *훨씬* 빠름. **divert_hard 시 SCP 보다 효과적일 수 있음**. 1-2일.
- [ ] 📚 **DDP** (Differential Dynamic Programming) — iLQR 의 정통 변형. 2계 미분 사용. 1-2일.
- [ ] 📚 **GuSTO** / **SCvx*** — 최신 SCP 변형 (수렴 보장 강화). 학술적 가치. 2-3일.

### A.3 학습 기반 (RL)

- [ ] 🔥 **MPC + RL Residual** (top 7 에 있음) — MPC 명령 + RL 보정 = robust + 학습. **가장 임팩트 큰 RL 패턴**.
- [ ] 🔥 **Curriculum + Better Reward** — 이전 PPO 실패 (devlog) 의 *진짜* 원인 (reward 설계). 재설계 + sub-task curriculum. 2-3일.
- [ ] 📚 **Imitation Learning** — MPC trajectory 를 *교사 데이터* 로 NN 모방. inference 빠름. 1-2일.
- [ ] 📚 **Sim-to-Real RL** — 도메인 randomization 으로 sim2real transfer. 진짜 hardware 가서 *그제야 효과*. 2-3일.
- [ ] 📚 **Meta-RL** (MAML 등) — 새 시나리오에 *빠른 적응*. 학술적 frontier. 3-5일.
- [ ] 📚 **Model-based RL** — 학습된 dynamics + planning. 우리는 dynamics 알고있으니 marginal. 2-3일.

### A.4 적응 제어 (Adaptive)

- [ ] 🔥 **Gain Scheduling** — 고도/속도/외란 강도별 게인 자동 전환. ~반나절. **scheduling 자체가 dispatcher 의 fine-grained 버전**.
- [ ] 🔥 **L1 Adaptive Control** — *최신* 적응 제어. NASA 가 hypersonic 에 사용. 1-2일.
- [ ] 📚 **MRAC** (Model Reference Adaptive Control) — 정통 적응 제어. 학술 baseline. 1-2일.
- [ ] 📚 **Direct/Indirect Adaptive** — plant 파라미터 *온라인 식별*. 1-2일.

### A.5 강건 제어 (Robust)

- [ ] 📚 **H∞ Control** — 최악 외란 대비 최적. 정통. 2-3일.
- [ ] 🔥 **Sliding Mode Control** — 외란에 *본질적 robust* (chattering 단점). 1일. **noisy/hard 강함**.
- [ ] 📚 **Backstepping** — 정통 비선형 robust 제어. 1-2일.
- [ ] 📚 **Lyapunov-based control** — 안정성 *증명* 가능. 학술 baseline. 1-2일.

### A.6 최적 제어 (Linear/LQ)

- [ ] 🔥 **LQR** (top 7 에 있음) — *진짜* 안 했음. 정통 baseline 의 *빠진 핵심*.
- [ ] 🔥 **LQG** (LQR + Kalman) — LQR + stochastic state estimation. 정통 패키지.
- [ ] 📚 **Loop transfer recovery (LTR)** — LQG 의 robust 강화. 학술.

### A.7 결합 / Hybrid

- [ ] 🔥 **Dispatcher** (top 7).
- [ ] 🔥 **Ensemble Voting/Blending** — 여러 controller 의 command 가중평균. Bayesian model averaging.
- [ ] 🔥 **Hybrid PID-MPC** — 고고도 MPC + 저고도 PID 같은 *명시적* hybrid. ~1시간 — 이미 우리 lookahead wrapper 가 이런 hybrid 의 *원시 형태*.

---

## B. 튜닝 (거의 안 함)

- [ ] 🔥 **MPC 비용 가중치 sweep** (시나리오별) — q_pos_xy, q_pos_z, q_vel_xy, q_vel_z, q_final_*, r_u, r_du 각각. ~1일.
- [ ] 🔥 **HoverPID 게인 sweep** — kp_pos, kd_pos, ki_pos, kp_z, kd_z, ki_z, kp_att, kd_att, ki_att. 9 차원. Bayesian Opt 필요. ~1일.
- [ ] 🔥 **LandingGuidance 11 파라미터** — gate_alt, creep, tight_frac, v_max, v_min, flare_gain, gate_offset, gate_speed, gate_tilt_deg, commit_*. 모두 기본값. ~반나절.
- [ ] 🔥 **재계획 빈도** (top 7).
- [ ] ⚡ **Lookahead 미세 sweep** (8, 10, 12, 13, 15, 17) — 현재 4/10/19 셋만. ~30분.
- [ ] ⚡ **터미널 가중치 sweep** (q_final_*) — 별도 튜닝. ~1시간.
- [ ] 🔥 **자동 튜닝** (Bayesian Optimization, Optuna) (top 7).
- [ ] 📚 **Hyperband / SHA** — 빠른 hyperparameter search.
- [ ] 📚 **CMA-ES** — 진화 알고리즘 기반 튜닝.

---

## C. 상태 추정 (매우 빈약)

### C.1 위치/속도/자세

- [ ] 🔥 **Kalman Filter** — 정통 (currently 단순 EMA). ~반나절.
- [ ] 🔥 **Extended Kalman Filter (EKF)** — 비선형 정통. ~1일.
- [ ] 🔥 **Unscented Kalman Filter (UKF)** — 더 정확. ~1일.
- [ ] 📚 **Particle Filter** — non-Gaussian noise.
- [ ] 📚 **Information Filter** — 비유 sparse measurements.

### C.2 자세 특화

- [ ] 🔥 **Complementary Filter** — gyro (빠름, drift) + accel (느림, drift 없음) fusion. ~2시간.
- [ ] 🔥 **Mahony / Madgwick Filter** — 정통 자세 estimator (PX4, Ardupilot 사용). ~3-4시간.
- [ ] 📚 **MEKF** (Multiplicative EKF) — quaternion 정통 EKF.

### C.3 외란/플랜트 추정

- [ ] 🔥 **온라인 바람 추정** (Wind observer) (top 7).
- [ ] 🔥 **Mass identification** — thrust-acceleration 관찰로 *실제* 질량 역추정. ~반나절.
- [ ] 🔥 **Drag observer** — drag coefficient 온라인 추정.
- [ ] ⚡ **Bias observer** — thrust_misalign 정상상태 오차에서 역추정. ~1시간.
- [ ] 📚 **Adaptive Kalman** — noise covariance 자동 튜닝.

---

## D. 물리 모델링 (시뮬 현실성)

- [x] EDF roll 물리 (방금 추가)
- [ ] 🔥 **지면 효과 (Ground Effect)** — 가까이에서 추력 +10-30%. **마지막 1m 의 실제 거동 영향 큼**. ~3-4시간.
- [ ] 🔥 **Aero damping** — 회전에 대한 공력 감쇠. roll 안정화에 도움. ~2시간.
- [ ] 📚 **Motor electric dynamics** — 배터리 droop, ESC dynamics, 전류 한계. ~1일.
- [ ] 📚 **Wind shear profiles** — 고도 별 다른 바람. ~3시간.
- [ ] 📚 **Gust profiles** — Dryden, von Karman 표준 난류. ~3시간.
- [ ] 📚 **Terrain effect** — 지표면 반사 + 와류. 복잡.
- [ ] 📚 **Compressibility / Mach effect** — N/A 우리 저속.
- [ ] 📚 **Time-varying thrust misalign** — 온도/시간 변동.
- [ ] 📚 **Blade dynamics** — 블레이드 와류, vortex ring state.

---

## E. 액추에이터 (현재 1개 채널)

- [ ] 🔥 **RCS** (cold gas thruster) — body-z 토크 추가. **EDF roll 물리 + RCS = robust 회복**. ~2-4시간.
- [ ] 🔥 **Reaction Wheel** — 동일 효과, 가스 대신 모터. ~2-4시간.
- [ ] 📚 **Counter-rotating fan** — 명시적 두 팬 모델링.
- [ ] 📚 **Variable pitch blade** — 헬리콥터 식 collective.
- [ ] 📚 **Differential thrust** — 멀티 EDF 시.
- [ ] 📚 **Aerodynamic surfaces** — Grid fins (Falcon 9), canard, air brake.

---

## F. 시나리오 (안 시도)

- [ ] 🔥 **터미널 외란 펄스** — 착륙 직전 ~50cm 에서 강풍 펄스. *현실적 실패 모드*. ~1시간.
- [ ] 🔥 **CG migration** — 연료/배터리 무게 중심 이동. ~2시간.
- [ ] 🔥 **Sensor dropout** — 1-2초간 GPS/IMU 손실. *실제 hardware 흔함*. ~2시간.
- [ ] 🔥 **Plant model mismatch** — vehicle 의 *진짜* 파라미터를 *공칭* 모델과 다르게. **이미 randomization 일부 있지만 sweep 안 함**. ~반나절.
- [ ] 📚 **Thrust mismatch** — 실제 < 명령 (battery drain).
- [ ] 📚 **Gimbal stuck** — failure mode.
- [ ] 📚 **Estimator failure** — Kalman 발산.
- [ ] 📚 **Fuel slosh** (미래 로켓엔진 시).
- [ ] 📚 **Multi-vehicle scenarios**.

---

## G. 평가 (정량 분석 부족)

- [ ] 🔥 **Statistical significance** (McNemar 체계적) — 모든 +pp 차이의 유의성 측정. ~2시간.
- [ ] 🔥 **Confidence intervals** (95% CI) — n=50 의 ±7pp 신뢰구간 명시. ~1시간.
- [ ] 🔥 **Sensitivity analysis** — 어느 파라미터가 가장 critical? Sobol indices / Morris screening. ~반나절.
- [ ] 🔥 **Worst-case scenario generation** — RL adversarial / minimax sampling. ~1일.
- [ ] 🔥 **Pareto frontier** (성공률 vs 연료 vs 정밀도). ~2시간.
- [ ] 🔥 **CDF / PDF of touchdown metrics** — 평균만이 아닌 *분포*. ~1시간.
- [ ] 📚 **Causality / counterfactual analysis** — "만약 X 가 없었다면" 정량.
- [ ] 📚 **A/B test with crossing trials**.

---

## H. 시스템 식별 / 캘리브레이션

- [ ] 🔥 **Plant-model mismatch sweep** — 실제 hardware 의 unmodeled dynamics 영향. ~반나절.
- [ ] 📚 **System ID** — sim 의 *진짜* dynamics 와 MPC 의 *내부 모델* 가 다른 경우. ~1일.
- [ ] 📚 **Sensor calibration** — 단순 노이즈 외 bias, drift, scale factor. ~3-4시간.
- [ ] 📚 **Online parameter estimation** — adaptive 와 결합. ~1일.
- [ ] 🔥 **Sim2Real transfer** — *진짜 hardware 가야* 효과. 별도 큰 프로젝트.

---

## I. 안전 / 비상 처리

- [ ] 🔥 **Failsafe mode** — sensor 실패 / 컨트롤러 실패 시 대응. 진짜 hardware 의 *필수*. ~반나절.
- [ ] 🔥 **Abort criteria** — 착륙 포기 후 ditch. ~2시간.
- [ ] 🔥 **Graceful degradation** — 한 actuator 실패 시 나머지로 진행. ~1일.
- [ ] 📚 **Constraint violation handling** — soft slack 일부만 있음. 체계화. ~3시간.
- [ ] 📚 **Watchdog timer / heartbeat** — hardware 필수.
- [ ] 📚 **Voting redundancy** — triple modular redundancy 등.

---

## J. 학습 인프라 / 도구

- [ ] 🔥 **RL 환경 재정비** — 이전 PPO 실패 후 폐기. 재활성 + 새 reward. ~1일.
- [ ] 🔥 **Bayesian Optimization 자동화** (top 7).
- [ ] 📚 **Distillation** — RL/MPC → 작은 NN으로 inference 빠르게.
- [ ] 📚 **Symbolic regression** — 학습된 policy 를 식으로 추출.
- [ ] 📚 **MLOps** — 실험 추적 (W&B, MLflow).

---

## K. 하드웨어 준비 (진짜 비행 가야 의미)

- [ ] 🔥 **HIL (Hardware-in-the-Loop)** — 실제 IMU/모터 + 시뮬 plant.
- [ ] 🔥 **SIL (Software-in-the-Loop)** — 컨트롤러 코드를 *임베디드 형식* 으로 변환.
- [ ] 🔥 **Active roll control 채널** (top 7 의 RCS/Reaction Wheel) — EDF 의 진짜 비행 필수.
- [ ] 📚 **Communication protocol** — telemetry 설계.
- [ ] 📚 **Logging / replay infrastructure** — 비행 데이터 분석.
- [ ] 📚 **Mission planner UI** — pad 좌표, 시나리오 입력.

---

## L. 진단 / 디버깅 도구

- [ ] 🔥 **Plot infrastructure** — 시계열 시각화 자동 (matplotlib 통합). ~3시간.
- [ ] 🔥 **3D animation** — vehicle 궤적 동영상. ~3시간.
- [ ] 📚 **Replay tool** — 저장된 시드 재실행.
- [ ] 📚 **A/B comparison UI** — 두 컨트롤러 동시 시각화.
- [ ] 📚 **Profiling** — cvxpy 솔브 시간 / 메모리.

---

## M. 문서 / 메타

- [ ] 📚 **Algorithm survey doc** — 시도한 알고리즘 transformer-식 요약.
- [ ] 📚 **API documentation** — Sphinx / mkdocs.
- [ ] 📚 **CONTRIBUTING.md** — 다른 개발자 onboarding.
- [ ] 📚 **Architecture decision records (ADR)** — 왜 PID + lookahead 선택했나 등.

---

## 카테고리별 진행도 요약

| 카테고리 | 시도/전체 | % |
|---|---|---|
| A. 컨트롤러 아키텍처 | 6 / 30 | 20% |
| B. 튜닝 | 1 / 9 | 11% |
| C. 상태 추정 | 1 / 12 | 8% |
| D. 물리 모델링 | 6 / 12 | 50% |
| E. 액추에이터 | 1 / 7 | 14% |
| F. 시나리오 | 4 / 9 | 44% |
| G. 평가 | 1 / 8 | 13% |
| H. 시스템 식별 | 0 / 5 | 0% |
| I. 안전 처리 | 0 / 6 | 0% |
| J. 학습 인프라 | 1 / 5 | 20% |
| K. 하드웨어 준비 | 0 / 6 | 0% |
| L. 진단 도구 | 1 / 5 | 20% |
| M. 문서 | 0 / 4 | 0% |
| **총** | **22 / 118** | **19%** |

= **약 80% 영역 미시도**.

---

## 다음 세션 우선순위 제안

세션 *임팩트 vs 노력* 비율 기준:

### 1순위 (즉시 가시화)
- [ ] 자동 튜닝 (Bayesian Opt with Optuna) — *모든* 튜닝 작업의 어머니
- [ ] Dispatcher / Ensemble — 빠른 worst-case ↑
- [ ] 재계획 빈도 sweep — 30분 작업, 외란 응답 ↑

### 2순위 (큰 효과)
- [ ] LQR / LQG baseline — 정통 빠진 핵심
- [ ] 온라인 바람 추정 — 외란 영역 직접
- [ ] 터미널 단계 게인 + 지면 효과 — *touchdown but not soft* 풀어줌

### 3순위 (장기 가치)
- [ ] MPC + RL Residual — 진짜 frontier
- [ ] Active roll control 채널 — EDF 실제 비행 필수
- [ ] HIL 환경 — 진짜 hardware 단계

---

## 기록

각 미시도 카테고리를 *언제, 누가, 어떤 결과* 로 했는지 향후 채우기. 다음 세션의 *backlog 청산* 추적.
