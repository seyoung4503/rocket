# 로드맵

## 2026-05-27 다음 스텝 — GNC-first로 방향 수정

이번 단계의 결론:

- **RL-first는 메인 방향이 아니다.**
- 프로젝트 중심축은 **GNC(Guidance, Navigation, Control)** 로 둔다.
- RL은 버리지 않고, **PID/MPC를 대체하는 주 제어기**가 아니라 **보조 제어기, residual, 외란/노이즈 보정기, 비교 실험 대상**으로 쓴다.
- EDF는 마하 1용 추진체가 아니라, **저속 VTVL 제어 테스트베드**로 사용한다.
- 마하 1 트랙은 EDF 제어 트랙과 분리하고, 당장은 시뮬레이션 프로파일로 준비한다.

즉 목표 문장을 다음처럼 바꾼다.

> 6개월 안에 마하 1 역추진 착륙 로켓을 완성한다.

에서:

> 6개월 안에 EDF 기반 VTVL 제어 스택을 만들고, 같은 GNC 구조를 마하 1 로켓 시뮬레이션으로 확장한다.

## 목표

### 단기 목표: EDF VTVL 제어 테스트베드

- 6-DOF 시뮬레이션에서 로버스트한 수직 이착륙/착륙 제어 스택을 만든다.
- EDF + TVC 짐벌 + 센서 노이즈 + 바람 + 모델 오차를 포함한다.
- PID/LQR/MPC 기반 기준선을 먼저 만든다.
- RL은 다음 용도로 비교/결합한다.
  - PID/MPC residual correction
  - 센서 노이즈/부분관측 보정
  - 바람/외란 대응
  - 착륙 타이밍 또는 guidance 정책
  - PID/MPC 대비 성능 비교
- 이후 실제 보드/센서/EDF 테스트베드에서 tethered hover와 저고도 자동 착륙을 검증한다.

### 장기 목표: 마하 1 + 회수/착륙 트랙

- EDF로 마하 1은 불가능하므로 별도 추진 트랙으로 분리한다.
- 마하 1은 고체/하이브리드/액체 로켓 추진, 구조, 공력, 법규, 시험장 문제가 포함된다.
- 초음속 비행체는 우선 시뮬레이션과 회수 시스템 중심으로 설계한다.
- 역추진 착륙과 결합하는 것은 EDF VTVL 제어 스택이 안정화된 뒤의 장기 목표다.
- 콜드 런치는 추진/점화/안전 체계가 안정화된 뒤 검토할 장기 확장이다. 초기에는 실제 장치가 아니라 시뮬레이션 이벤트와 안전 요구사항 정의부터 시작한다.

## 핵심 현실 점검

| 항목 | 결론 |
|---|---|
| EDF로 마하 1 | 불가능. EDF는 저속 제어 테스트베드용이다. |
| 고체연료로 역추진 착륙 | 부적합. 스로틀 조절/재점화/정밀 제어가 어렵다. |
| RL 단독 제어 | 실제 하드웨어 메인 제어기로는 위험하고 비효율적이다. |
| SpaceX식 접근 | 공개 자료 기준으로 RL-first보다 optimal control, convex optimization, robust GNC에 가깝다. |
| 콜드 런치 | 추진과 분리된 발사/점화 sequence 문제다. 실제 구현은 고위험이므로 시뮬레이션과 안전 요구사항부터 다룬다. |
| 현실적 6개월 산출물 | EDF VTVL 시뮬레이터 + GNC 기준선 + RL 보조 실험 + tethered hover 준비/검증. |

## 권장 제어 구조

```text
Sensors
  ↓
Navigation
  상태추정: IMU, 고도계, GPS/vision, EKF/필터
  ↓
Guidance
  목표 고도, 목표 속도, 착륙 궤적, landing gate
  ↓
Control
  PID/LQR/MPC로 스로틀 + TVC 짐벌 명령 생성
  ↓
RL Assist
  residual correction 또는 외란/노이즈 보정
  ↓
Safety Filter
  추력, 짐벌, 기울기, 속도, 고도 제한
  ↓
EDF + TVC
```

최종 명령 구조:

```text
u_final = safety_filter(u_model_based + u_rl_residual)
```

## 단계

### 2026-05-28 다음 구현 작업 — Navigation-first GNC 정리

MPC 실험 결론:

- 단순 3D point-mass MPC는 PID보다 약했다.
- MPC를 waypoint/guidance로 제한하고 기존 PID가 추종하면 PID 안정성은 유지된다.
- 다음 병목은 MPC 튜닝이 아니라 **상태추정/센서/terminal mode 계층 부재**다.

따라서 다음 구현 순서는 아래처럼 고정한다.

1. **Sensor model 추가**
   - 파일 후보: `src/rocketsim/navigation/sensors.py`
   - IMU gyro/accel noise
   - attitude noise
   - altitude/range noise
   - position/velocity measurement noise
   - bias/drift는 2차 단계로 둔다.

2. **Simple estimator 추가**
   - 파일 후보: `src/rocketsim/navigation/estimator.py`
   - 시작은 EKF가 아니라 low-pass/complementary filter로 한다.
   - 입력: noisy measurement
   - 출력: controller가 사용할 estimated state
   - 목적: PID/MPC/RL이 모두 같은 추정 상태를 보게 만드는 것

3. **Environment control-state 분리**
   - 현재 true state, noisy measured state, estimated state의 역할을 분리한다.
   - controller 입력은 기본적으로 estimated state가 되도록 한다.
   - 성공 판정은 true state로 유지한다.

4. **평가 스크립트 추가**
   - 파일 후보: `scripts/evaluate_navigation.py`
   - 비교 대상:
     - PID on true/noisy state
     - PID on estimated state
     - MPC waypoint + PID on estimated state
   - 난이도:
     - `calm`
     - `moderate`
     - `noisy`
     - `hard`

5. **Guidance/terminal mode 분리**
   - 파일 후보: `src/rocketsim/guidance/landing.py`
   - `LandingPID` 안에 섞여 있는 landing gate를 별도 guidance state machine으로 분리한다.
   - 모드:
     - `approach`
     - `settle`
     - `flare`
     - `commit`
     - `touchdown/abort`

6. **MPC 재평가**
   - estimator + guidance state machine 위에서 다시 비교한다.
   - 비교 대상:
     - PID baseline
     - Cvxpy waypoint guidance + PID tracking
     - Cvxpy guidance + tracking feedback
   - 목표는 당장 PID를 이기는 것이 아니라, 같은 GNC 계층 위에서 공정 비교하는 것이다.

7. **RL residual은 마지막에 추가**
   - 기본 구조가 안정화된 뒤에만 진행한다.
   - 제한:
     - throttle residual: 작은 범위
     - gimbal residual: 작은 범위
   - RL의 목표는 direct control이 아니라 모델오차/노이즈/외란 보정이다.

성공 기준:

- `calm`, `moderate`에서 estimated-state PID가 true-state PID와 거의 같은 성공률을 유지한다.
- `noisy`에서 estimated-state PID가 raw noisy PID보다 개선된다.
- MPC 계열은 같은 estimated state와 같은 terminal mode를 사용해 평가된다.
- 모든 결과는 `docs/mpc_experiments.md` 또는 별도 `docs/navigation_experiments.md`에 기록한다.

### 2026-05-28 추가 실험 결과 — point-mass MPC 한계 확인

질문:

> 현재 문제가 MPC가 점질량으로 계산해서 로켓이 실패하는 것인가?

실험 결론:

- **맞다. 큰 원인 중 하나다.**
- 현재 시뮬레이터는 점질량이 아니라 6-DOF rigid body다.
  - quaternion attitude
  - 관성행렬
  - 엔진 오프셋
  - TVC gimbal
  - gimbal rate limit
  - thrust spool-up lag
  - drag/wind/external force
  - episode별 mass/thrust/inertia/misalignment randomization
- 반면 convex MPC는 position/velocity만 보는 point-mass 모델이다.
- `scripts/diagnose_mpc_model_mismatch.py`로 점질량 계획을 6-DOF 시뮬레이터에서 no-wind rollout 해보니, 계획은 4-5 m 고도에 남아 있어야 하는데 실제 6-DOF 차량은 지면을 통과했다.
- gimbal saturation이 87-94%로 나타났다. 즉 point-mass solver가 요구하는 lateral acceleration이 실제 짐벌 로켓으로는 그 시간 안에 추종 불가능하다.
- oracle plant model, 즉 episode별 실제 mass/thrust/inertia를 알려줘도 이 mismatch가 사라지지 않았다. 문제는 단순 파라미터 오차가 아니라 모델 구조 차이다.

추가 구현:

- `scripts/diagnose_mpc_model_mismatch.py`
  - point-mass convex MPC 계획과 6-DOF rollout mismatch를 측정한다.
  - 결과 저장: `docs/mpc_model_mismatch.md`
- `LandingFullDynamicsMPC`
  - `dyn.rk4_step` 기반 6-DOF sampled nonlinear MPC
  - PID 명령 주변의 residual 후보를 full dynamics로 rollout한다.
  - retune 후에는 PID safety-filter 형태로만 작은 보정을 허용한다.
  - 결과 저장: `docs/full_dynamics_mpc_experiments.md`

현재 결론:

- convex point-mass MPC는 **outer guidance**로는 유지할 수 있다.
- 하지만 직접 착륙 authority를 주면 현재 6-DOF 로켓이 추종하지 못한다.
- 실제 SpaceX식 구조에 더 가까운 다음 단계는:
  1. point-mass convex guidance
  2. attitude/actuator feasibility check
  3. full-dynamics tracking controller
  4. safety filter
  5. estimator 기반 상태 입력
- full 6-DOF sampled MPC는 연구/진단용으로는 유용하지만, 현재 Python 구현은 너무 느려서 50 Hz onboard controller 후보가 아니다.

### 2026-05-28 구조 정리 — Shared Guidance + MPC+PID

이번 정리의 목적:

- PID 안에 있던 landing gate를 분리한다.
- PID와 MPC가 같은 착륙 판단을 공유하게 만든다.
- MPC는 단독 제어기가 아니라 waypoint/guidance 제안자로 쓴다.
- PID는 최종 throttle/gimbal tracking을 계속 담당한다.

구현:

- `src/rocketsim/guidance/landing.py`
  - `LandingGuidance`
  - `LandingGuidanceConfig`
  - `LandingGuidanceOutput`
  - mode: `approach`, `settle`, `flare`, `commit`
- `LandingPID`
  - 기존 landing gate 로직을 제거하고 `LandingGuidance` target만 추종하도록 변경
- `LandingCvxpyWaypointPID`
  - 기존 ad hoc landing gate 대신 shared `LandingGuidance` 사용
- `LandingFeasibleWaypointMPC`
  - point-mass MPC가 제안한 XY waypoint를 attitude/actuator feasibility envelope 안으로 projection
  - PID가 projected target을 추종
- `scripts/evaluate_navigation.py`
  - `feasible` controller 추가

실험:

`docs/feasible_waypoint_experiments.md`, n=100, estimated state, nominal plant model

| difficulty | PID | waypoint MPC+PID | feasible MPC+PID |
|---|---:|---:|---:|
| noisy | 85/100 | 75/100 | 79/100 |
| hard | 34/100 | 33/100 | 34/100 |

해석:

- **MPC+PID는 사용한다.**
- 다만 MPC가 throttle/gimbal을 직접 제어하는 방식이 아니라, PID가 따라갈 waypoint를 제안하는 방식이다.
- n=10에서는 hard 개선처럼 보였지만, n=100에서는 통계적으로 유지되지 않았다.
- feasible 버전은 raw waypoint보다 noisy 안정성이 낫지만 PID보다 좋지는 않다.
- hard에서는 PID와 거의 동률이다.
- 남은 문제는 vertical speed가 아니라 terminal hspeed/offset/tilt다.
- 다음 개선은 단순 waypoint clipping이 아니라 terminal guidance/trajectory timing 자체를 바꿔야 한다.

### Phase 0 — 현재 코드 정리: 착륙 시뮬레이션 기준선

완료/진행 중:

- 6-DOF 강체 동역학 + RK4
- EDF 차량 모델
- TVC 짐벌/추력 지연
- PID 호버/착륙 기준선
- Gymnasium RL 착륙 환경
- 바람/외란/도메인 랜덤화
- PID vs RL 평가 스크립트

다음 작업:

- 현재 `landing_env`를 GNC 구조 기준으로 재정리한다.
- 평가 결과를 `hard`, `noisy`, `unknown`, `recovery`로 분리해 문서화한다.
- 기존 결론을 명확히 적는다.
  - `hard`: PID 우세
  - `noisy`: RL/frame stack이 PID보다 유리
  - 따라서 RL은 주 제어기보다 보조 제어기로 적합

### Phase 1 — Navigation/센서 모델 추가

목표:

- 실제 보드 실험으로 이어질 수 있는 센서 모델을 만든다.

작업:

- IMU gyro/accel noise, bias, drift 모델
- 고도계/거리센서 노이즈와 지연
- 상태 참값과 측정값 분리
- 간단한 complementary filter 또는 EKF
- PID/MPC/RL 모두 같은 추정 상태를 사용하도록 통일

성공 기준:

- 참값 상태 제어가 아니라, 노이즈 측정/추정 상태만으로 hover/landing 성공률을 측정한다.

### Phase 2 — Guidance 분리

목표:

- 착륙 정책을 controller 내부 ad hoc 로직이 아니라 guidance 모듈로 분리한다.

작업:

- 목표 고도/속도 profile 생성
- landing gate 분리
- hover, descent, flare, touchdown mode 정의
- horizontal divert 목표 생성
- 착륙 실패 원인별 metric 정리

성공 기준:

- 같은 guidance를 PID, LQR, MPC, RL-residual이 공유할 수 있다.

### Phase 3 — PID/LQR 기준선 강화

목표:

- RL이 비교할 만한 강한 model-based baseline을 만든다.

작업:

- 기존 PID gain 정리
- LQR 또는 geometric attitude controller 추가 검토
- anti-windup, actuator saturation, rate limit 명시
- sensor filter 포함한 실전형 PID 평가

성공 기준:

- calm/moderate에서 높은 성공률
- noisy/hard에서 실패 원인을 일관되게 재현

### Phase 4 — MPC/최적제어 추가

목표:

- SpaceX식 접근에 가까운 guidance/control 실험을 시작한다.

작업:

- 단순 1D vertical landing MPC부터 시작
- 이후 2D/3D divert landing으로 확장
- 제약 포함:
  - 추력 한계
  - 짐벌 한계
  - 기울기 한계
  - 착륙 속도 한계
  - 지면 충돌 방지
- 비용함수:
  - 착륙 오차
  - 속도
  - 연료/에너지
  - 제어 입력 변화량

성공 기준:

- PID보다 착륙 timing과 fuel/energy tradeoff가 좋아지는지 확인한다.
- RL은 이 MPC를 대체하기보다 보조하거나 모방하는 대상으로 둔다.

### Phase 5 — RL Assist

목표:

- RL을 직접 제어기보다 안전한 위치에 넣는다.

실험 순서:

1. PID + residual RL
2. MPC + residual RL
3. RL guidance + PID inner loop
4. imitation learning: MPC/PID teacher policy 모방
5. direct RL control은 시뮬레이션 비교용으로만 유지

성공 기준:

- RL이 다음 영역 중 하나에서 명확한 개선을 보인다.
  - 센서 노이즈
  - 부분관측
  - 외란 추정
  - 모델 오차
  - landing timing
  - energy/fuel tradeoff

### Phase 6 — 실제 EDF 테스트베드

목표:

- 시뮬레이션의 EDF VTVL 제어 스택을 실제 하드웨어로 옮긴다.

순서:

1. EDF thrust stand
2. thrust curve 측정
3. spool-up/down 지연 측정
4. TVC 짐벌 응답 측정
5. IMU/고도계 노이즈 측정
6. tethered hover
7. 낮은 높이 제한 자동 착륙

성공 기준:

- 보드 위에서 센서 추정, PID/LQR/MPC 제어, safety limiter가 실제로 동작한다.
- RL은 처음부터 실제 모터 명령을 직접 내리지 않는다.

### Phase 7 — 마하 1 시뮬레이션 트랙

목표:

- EDF 트랙과 별도로 고속 로켓 모델을 준비한다.

작업:

- 로켓 추진 profile
- 질량 변화
- 고속 항력
- 안정성/CP/CG
- fin control 또는 TVC
- 회수 시스템
- 법규/시험장 요구사항 조사

성공 기준:

- 실제 제작 전, 시뮬레이션에서 마하 1 ascent profile과 회수 전략을 검토한다.

### Phase 8 — 콜드 런치 개념 검토

목표:

- 추진 시스템이 안정화된 뒤, 발사 sequence를 엔진 점화와 분리하는 콜드 런치 개념을 장기 후보로 검토한다.

범위:

- 실제 장치 설계가 아니라 시뮬레이션과 요구사항 정의부터 시작한다.
- 콜드 런치 phase, ignition transition, powered ascent phase를 분리해 모델링한다.
- ejection 직후 자세 안정화, 점화 전후 상태추정, abort 조건, 안전 envelope을 정의한다.
- GNC 입장에서는 다음 상태 전이를 다루는 문제로 본다.

```text
stored / prelaunch
  ↓
cold launch separation
  ↓
attitude stabilization window
  ↓
main propulsion ignition
  ↓
powered ascent
```

필수 전제:

- 추진/점화/구조/법규/시험장 요구사항이 먼저 정리되어야 한다.
- 실제 구현은 전문가 검토와 합법적인 시험 환경 없이 진행하지 않는다.
- 이 로드맵에서는 구체적인 고에너지 발사장치 설계나 제작 절차를 다루지 않는다.

성공 기준:

- 시뮬레이션에서 콜드 런치 전후 상태 전이와 GNC 요구사항을 정의한다.
- 콜드 런치가 전체 목표에 주는 이점과 추가 위험을 trade study로 비교한다.

## 6개월 산출물 정의

필수:

- EDF VTVL 6-DOF 시뮬레이터
- GNC 구조 문서화
- PID/LQR 기준선
- 센서 노이즈/외란/모델오차 포함 평가
- MPC 또는 단순 최적제어 prototype
- RL assist 실험 결과
- 실제 EDF 테스트베드 설계와 thrust/TVC 계측

도전:

- tethered hover
- 저고도 자동 착륙
- RL residual이 특정 조건에서 PID/MPC를 개선하는 결과

장기 준비:

- 마하 1 로켓 시뮬레이션 profile
- 추진/구조/공력/법규 조사
- 콜드 런치 개념 trade study

## 제어 전략 요약

| 전략 | 역할 |
|---|---|
| PID | 첫 기준선. 단순하고 강하며 실제 테스트에 적합. |
| LQR/geometric control | 자세 안정화 품질 개선 후보. |
| MPC/convex optimization | 제약을 고려한 landing guidance/control. SpaceX식 접근에 가까움. |
| RL direct control | 시뮬레이션 비교용. 실제 하드웨어 메인 제어기로는 보류. |
| RL residual | 가장 현실적인 RL 사용처. PID/MPC 위에 작은 보정만 더함. |
| Safety filter | 하드웨어 적용 전 필수. RL 포함 모든 명령을 제한. |

## EDF 관련 주의

- EDF는 마하 1 추진체가 아니다.
- EDF는 저속 VTVL 제어, 센서, TVC, 시뮬레이션-실기 전환을 검증하기 위한 테스트베드다.
- EDF는 추력 응답 지연이 크므로 자세 제어는 TVC 짐벌 중심으로 설계한다.
- 단일 김벌 EDF는 roll 제어가 약하거나 불가능할 수 있다. 필요하면 반작용휠, 핀, 카운터 로터, 다중 EDF 등을 검토한다.
- 실제 테스트에서는 반드시 tether, soft kill switch, thrust limit, tilt limit, geofence를 둔다.
