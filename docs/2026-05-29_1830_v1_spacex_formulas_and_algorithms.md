# SpaceX-Style Landing — 수식·알고리즘 통합 정리

**날짜**: 2026-05-29 18:30 KST  
**버전**: v1  
**참고**: 기존 [`2026-05-29_0248_v1_formulas_and_algorithms.md`](./2026-05-29_0248_v1_formulas_and_algorithms.md) 는 *우리 PID + MPC + lookahead* 식 스택을 정리. 이 문서는 *SpaceX 식 (G-FOLD lineage)* 만 떼서 *처음부터 끝까지* 수식·알고리즘 트리로 정리.

---

## 0. 좌표·기호 약속

- 월드 프레임: `x, y` = 수평, `z` = 위 (+). 패드는 원점 (0, 0, 0).
- 바디 프레임: 차량 종축 = `body-z`.
- `g_world = (0, 0, -g)`, g = 9.81 m/s² (지구 표면).
- 상태: `p ∈ R³` 위치, `v ∈ R³` 속도, `q ∈ R⁴` 쿼터니언, `ω ∈ R³` 각속도, `T` 추력 크기.
- 제어 입력: `u ∈ R³` 월드 프레임 *추력 가속도* (m/s²) — 즉 `u = T·n̂ / m` (n̂ 은 추력 방향).
- Command: `(throttle, gimbal_x, gimbal_y)`.

---

## 1. 알고리즘 트리 (한 그림)

```
┌──────────────────────────────────────────────────────────────────────┐
│  Layer 1: Convex Landing MPC      (G-FOLD style)                     │
│            Solve every replan_dt:                                    │
│            min  fuel(u) + landing(p[N], v[N]) + running(p_z)         │
│            s.t. dynamics, glideslope, thrust cone, descent rate      │
│       →  MpcPlan(p, v, u, start_t, dt)                               │
├──────────────────────────────────────────────────────────────────────┤
│  Layer 2: Trajectory Tracker      (Time-indexed PD + I)              │
│            tau = sim_t - plan.start_t                                │
│            (p_ref, v_ref, u_ff) = plan.at_time(tau)   ★ 보간          │
│            a_des = u_ff + K_p·(p_ref-p) + K_d·(v_ref-v) + K_i·∫err   │
│       →  a_des ∈ R³                                                  │
├──────────────────────────────────────────────────────────────────────┤
│  Layer 3: Attitude Controller     (Quaternion PD on body-z alignment)│
│            F_des = m·a_des                                           │
│            throttle = ‖F_des‖ / T_max                                │
│            z_des = F_des / ‖F_des‖                                   │
│            err = R(q)ᵀ · (bz_world × z_des)                          │
│            τ_des = I · (k_p·err - k_d·ω + k_i·∫err)                  │
│            (g_x, g_y) = -τ_des[xy] / (L·T_eff)                       │
│       →  Command(throttle, gimbal_x, gimbal_y)                       │
└──────────────────────────────────────────────────────────────────────┘
```

각 레이어 내부 식들 본문에서 상세히.

---

## 2. Layer 1 — Convex Landing MPC (G-FOLD)

### 2.1 결정 변수 (cvxpy)

| 기호 | 차원 | 의미 |
|---|---|---|
| `p[:, 0..N]` | (3, N+1) | 위치 상태 |
| `v[:, 0..N]` | (3, N+1) | 속도 상태 |
| `u[:, 0..N-1]` | (3, N) | 추력 가속도 (제어) |
| `s[0..N-1]` | (N,) | `‖u[:, k]‖₂ ≤ s[k]` 슬랙 (lossless conv) |
| `soft[0..N-1]` | (N,) | 글라이드슬로프 슬랙 (높은 페널티) |

여기서 N 은 *남은 horizon 길이*: `N = ⌊(T_final − sim_t) / dt⌋` (**Shrinking horizon**).

### 2.2 비용 함수 (Objective)

```
minimize:
    Σ_{k=0..N-1} [ w_fuel · s[k]                  (≈ ∫‖u‖dt: min-fuel)
                 + w_running_z · p[2, k]²        (running z: 빨리 내려와)
                 + w_soft · soft[k]²             (글라이드슬로프 위반 억제)
                 ]
  + w_term_p · ‖p[:, N]‖²                          (terminal: 패드 도착)
  + w_term_v · ‖v[:, N]‖²                          (terminal: 정지)
```

기본값:
- `w_fuel = 1`
- `w_running_z = 0.5`
- `w_term_p = 200`, `w_term_v = 200`
- `w_soft = 1000`

#### 비교 — 우리 기존 비용 (참고용)

| | SpaceX 식 (이 문서) | 우리 기존 (`CvxpyPointMassMPC`) |
|---|---|---|
| In-flight 비용 | min ‖u‖ + 작은 running z | regulator: q_pos·‖p‖² + q_vel·‖v‖² + r_u·‖u-hover‖² + r_du·‖du‖² |
| Terminal | `p[N]=0, v[N]=0` 강제 페널티 | 같음 |
| 본질 | **fuel + 시간 최적화** | **state regulation (안정 추적)** |

→ 이게 plan 의 *공격성* 차이의 *근원*. 우리 기존은 smoothness 우선 → 호버 트랩. SpaceX 는 fuel 최소화 우선 → coast-and-burn 패턴.

### 2.3 제약 (Constraints)

#### 2.3.1 동역학 (Euler 적분)

각 k = 0..N-1 에 대해:
```
p[:, k+1] = p[:, k] + dt · v[:, k] + ½ dt² · (u[:, k] + g_world)
v[:, k+1] = v[:, k] + dt · (u[:, k] + g_world)
```

#### 2.3.2 추력 / 자세 콘

```
‖u[:, k]‖₂  ≤  s[k]                  (lossless-conv 매그니튜드 슬랙)
s[k] ∈ [a_min, a_max]                (추력 가속도 범위)
u_z[k]  ≥  0                          (one-sided thrust)
‖u_xy[k]‖₂  ≤  tan(θ_max) · u_z[k]    (틸트 콘 = 짐벌 각도 한계)
```

`a_max = T_max / m`, `a_min = 0` (EDF 는 셔트오프 가능; SpaceX 식 hard constraint 인 `a_min ≥ 40% T_max` 와 다름).

#### 2.3.3 ★ 글라이드슬로프 (G-FOLD 의 *간판* 제약)

```
‖p_xy[k]‖₂  ≤  tan(γ_glide) · p[2, k]  +  soft[k]
```

= 어느 고도 z 든, 수평 거리는 *z 의 tan(γ) 이하* — vehicle 이 패드를 향한 *원뿔* 안에서 움직임. 기본 γ = 30°. **bouncing / orbiting 방지의 *핵심*** — 우리 기존 MPC 에는 *없었던* 제약.

`soft[k]` 슬랙은 noisy IC 에서 발생 가능한 *초기* 위반만 허용 (heavily penalized via w_soft).

#### 2.3.4 지면 + 최대 강하

```
p[2, k]  ≥  0                          (지면 아래로 안 감)
v[2, k]  ≥  -v_max_desc                (최대 강하속도 한계, 기본 4 m/s)
soft[k]  ≥  0
```

#### 2.3.5 Boundary

```
p[:, 0] = p₀                           (현재 상태 = parameter)
v[:, 0] = v₀
p[2, N] ≥ 0
v[2, N] ≥ -0.35                        (gentle terminal vz)
```

terminal `p[:, N] = 0, v[:, N] = 0` 은 *equality* 가 아니라 *비용 페널티* 로. 슬랙 풀어두면 옵티마이저가 *대체로 도달* 하되 *불가능한 IC* 에선 적당히 양보.

### 2.4 Shrinking Horizon (receding 와 비교)

- **Receding** (우리 기존): 매번 "*지금부터* 4초" plan. 데드라인 따라옴. → hover 트랩.
- **Shrinking** (이 문서): `T_final` 은 *고정* (예: 12초), `N = ⌊(T_final − sim_t)/dt⌋` 으로 매번 감소. *데드라인 절대값* 정해짐 → commit 강제.

`T_final` 선택 기준: 시뮬 `timeout` (예: 20초) 보다 조금 짧게 (12초). 차량이 그 안에 *반드시* 도착해야 함.

### 2.5 G-FOLD 의 *수학적* 핵심 — Lossless Convexification

원래 hoverslam:
```
min  ∫ ‖u‖ dt        (← norm, 볼록)
s.t. ‖u‖ ≥ a_min     (★ non-convex when a_min > 0)
```

`‖u‖ ≥ a_min` 은 *볼록 집합의 *외부*** — 비볼록. 옛날엔 SQP/IPOPT 같은 비선형 솔버 필요.

Açıkmeşe (2007): **변수 변환** 으로 동등한 볼록 문제 만듦.
- 슬랙 변수 `s` 도입, `‖u‖ ≤ s` (볼록), `a_min ≤ s ≤ a_max` (볼록)
- 비용 `∫ s dt` (볼록)
- **결과 1**: optimum 에서 `‖u‖ = s` 자동 성립 (= lossless)
- **결과 2**: SOCP 로 풀림, 매우 빠름 (100Hz onboard 가능)

→ 이 트릭으로 SpaceX 의 *실시간 onboard MPC* 가 *비로소 가능* 해졌음.  
우리 코드도 같은 슬랙 `s` 사용. `a_min=0` 이라 비볼록성 없지만 패턴은 동일.

---

## 3. Layer 2 — Trajectory Tracker (Time-Indexed PD + I)

### 3.1 Time-Indexed Reference

매 컨트롤 step (50Hz) 에:
```
tau  =  max(0, sim_t − plan.start_t)
idx  =  tau / plan.dt
alpha = idx - ⌊idx⌋
p_ref(tau) = (1 − α) · plan.p[:, ⌊idx⌋] + α · plan.p[:, ⌊idx⌋+1]   (선형 보간)
v_ref(tau) = (1 − α) · plan.v[:, ⌊idx⌋] + α · plan.v[:, ⌊idx⌋+1]
u_ref(tau) = (1 − α) · plan.u[:, ⌊idx⌋] + α · plan.u[:, ⌊idx⌋+1]
```

`tau ≥ horizon` 일 땐 terminal 값 반복.

### 3.2 PD + I 피드백 + Feed-Forward

```
err_p     =  p_ref − p
err_v     =  v_ref − v
err_p_int += err_p · dt                (★ anti-windup)
clip(err_p_int_xy,  ±xy_int_limit)
clip(err_p_int_z,   ±z_int_limit)

a_des  =  u_ref                        (FEED-FORWARD: MPC 의 plan)
       +  K_p · err_p                  (P)
       +  K_d · err_v                  (D)
       +  K_i · err_p_int              (I)
```

기본 게인 (HoverPID 와 동일, EDF 에 검증됨):
- `K_p = diag(1.6, 1.6, 6.0)`
- `K_d = diag(2.4, 2.4, 4.0)`
- `K_i = diag(0.6, 0.6, 1.5)`
- `xy_int_limit = 3 m·s`, `z_int_limit = 5 m·s`

### 3.3 핵심 차이 — lookahead 트릭과의 비교

| | lookahead 트릭 | Time-indexed (이 문서) |
|---|---|---|
| reference 위치 | 항상 *plan[k=lookahead]* (지금 + N·dt) | *plan[t=tau]* (지금) |
| velocity ref 사용 | 무시 | **사용** |
| feed-forward (u) | 무시 | **사용** (MPC 가 의도한 thrust) |
| 게인 튜닝 | 한 점 setpoint 추적 | trajectory 추적 |
| `anticipation` / `lookahead` 노브 | *있음* (튜닝 필요) | *없음* |

→ time-indexed 가 MPC plan 의 *정보 전부* 사용. plan 의 *velocity/acceleration* 까지 추적.

---

## 4. Layer 3 — Attitude Controller (Body-z PD + Gimbal Inverse)

### 4.1 Thrust Vector → Throttle + Body-z 목표

```
F_des     =  m · a_des                  (요청된 월드 프레임 추력)
F_mag     =  ‖F_des‖
throttle  =  clip(F_mag / T_max, 0, 1)
```

`F_mag < ε` 일 땐 throttle=0, body-z = world-z (fallback).

자세 desired:
```
z_des     =  F_des / ‖F_des‖            (단위 벡터)
tilt      =  acos(z_des · ẑ)
if tilt > θ_max:                        (틸트 안전 한계, 기본 12°)
    z_des = [sin(θ_max)·x̂_horiz, cos(θ_max)]   (콘 안으로 끌어옴)
```

### 4.2 Body-z 정렬 오차 (쿼터니언 PD)

```
bz_world  =  R(q) · ẑ                    (현재 body-z 의 월드 표현)
err_world =  bz_world × z_des            (외적 = ‘axis · sin θ_err’)
err_body  =  R(q)ᵀ · err_world           (바디 프레임으로)
```

PD + I (자세 통합기):
```
err_int += err_body · dt
clip(err_int, ±0.5)
τ_des  =  I · ( k_p · err_body − k_d · ω + k_i · err_int )
```

기본:
- `k_p = 130, k_d = 24, k_i = 55`
- `I` = 차량 관성 텐서

### 4.3 Torque → Gimbal Inverse Kinematics

추력이 짐벌 노즐을 통해 작용하므로, 토크 = 노즐 위치 × 추력. 노즐이 차량 바디-z 축 아래로 `L = engine_offset` 떨어져 있으면:
```
torque_x  =  − L · T_actual · gimbal_y   (gimbal_y → x-축 회전)
torque_y  =  + L · T_actual · gimbal_x   (gimbal_x → y-축 회전)
```

(부호는 좌표계 약속에 따라.)

역산:
```
T_eff     =  max(throttle · T_max, 0.25 · T_max)   (수치 안정 floor)
L_eff     =  engine_offset · T_eff
g_x       =  clip(− τ_des[x] / L_eff,  ±gimbal_limit)
g_y       =  clip(− τ_des[y] / L_eff,  ±gimbal_limit)
```

`gimbal_limit = ±12°` (EDF 스펙).

### 4.4 출력

```
return Command(throttle, gimbal_x = g_x, gimbal_y = g_y)
```

시뮬레이터는 이 명령을 받아:
1. throttle → thrust 명령 → 1차 lag (τ = 0.08s) 으로 실제 thrust 진화
2. gimbal → 200°/s rate 제한으로 실제 gimbal 각도 진화
3. 그 결과 thrust 벡터 + 토크 가 차량에 적용

---

## 5. 전체 흐름 (Step-by-Step)

매 컨트롤 step:

```
1. 현재 state 받음: (p, v, q, ω, T) ∈ R¹⁴
2. (if t − last_plan_t ≥ replan_dt OR plan == None):
   a. N = ⌊(T_final − t) / dt⌋
   b. cvxpy 문제 *현재 N* 으로 빌드
   c. p₀ = p, v₀ = v (parameter 로)
   d. 솔브 → MpcPlan(p, v, u, start_t = t, dt)
3. Tracker:
   a. tau = t − plan.start_t
   b. (p_ref, v_ref, u_ref) = plan.at_time(tau)
   c. err_p, err_v 계산
   d. err_p_int += err_p · dt (clip)
   e. a_des = u_ref + K_p·err_p + K_d·err_v + K_i·err_p_int
4. Attitude:
   a. F_des = m · a_des, throttle = ‖F_des‖ / T_max (clip)
   b. z_des 계산, 틸트 콘 안으로 끌어옴
   c. err_body 계산, err_int += err_body·dt (clip)
   d. τ_des = I·(k_p·err_body − k_d·ω + k_i·err_int)
   e. (g_x, g_y) = inverse from τ_des
5. return Command(throttle, g_x, g_y)
```

매 step 5번이 시뮬레이터에 전달, 5번 → 시뮬 → 다음 state.

---

## 6. *왜* 이 알고리즘인가 — 의사결정 트레이드오프

(상세 분석: `docs/2026-05-29_1753_v1_spacex_style_design.md`)

| 결정 | 선택 | 근거 |
|---|---|---|
| MPC 비용 | min-fuel (∫‖u‖) | SpaceX 의 *진짜* 비용 — 연료 그대로. smoothness 비용은 호버 트랩 만듦. |
| Horizon | shrinking (T_final fixed) | receding 은 데드라인 따라옴 → 영원히 미룸. |
| Glideslope | tan(γ)·z ≥ ‖p_xy‖ | bouncing 방지의 *수학적* 가드 — 게이트 hack 불필요. |
| Tracker | time-indexed PD+I | plan 의 *velocity, accel* 정보 *전부* 사용. lookahead 의 한 점은 *손실*. |
| Attitude | 기존 PD (재사용) | 자세 동역학은 어디서나 동일 — 재발명 가치 없음. |
| a_min | 0 (EDF) | EDF 셔트오프 가능. SpaceX 식 `a_min ≥ 40%` 는 액체엔진 hardware 제약 — 우리에겐 없음. |

---

## 7. 문헌 인용

기본 G-FOLD 식 + lossless convexification:
- Açıkmeşe, B., Ploen, S. R. (2007). "Convex Programming Approach to Powered Descent Guidance for Mars Landing." *Journal of Guidance, Control, and Dynamics*, 30(5).
- Blackmore, L., Açıkmeşe, B., Scharf, D. P. (2010). "Minimum-Landing-Error Powered-Descent Guidance for Mars Landing Using Convex Optimization." *JGCD*, 33(4).
- Açıkmeşe, B., Carson, J. M., Blackmore, L. (2013). "Lossless Convexification of Nonconvex Control Bound and Pointing Constraints of the Soft Landing Optimal Control Problem." *IEEE Transactions on Control Systems Technology*, 21(6).

SCP 6-DOF 확장 (= 우리 Step 3 의 향후 방향):
- Szmuk, M., Açıkmeşe, B. (2018). "Successive Convexification for 6-DoF Mars Rocket Powered Landing with Free-Final-Time." *AIAA SciTech*. [arXiv:1811.10803](https://arxiv.org/abs/1811.10803)
- Reynolds, T. P., Szmuk, M., Malyuta, D., Mesbahi, M., Açıkmeşe, B., Carson, J. M. III (2020). "Dual Quaternion-Based Powered Descent Guidance with State-Triggered Constraints." *JGCD*, 43(9). [arXiv:1901.02181](https://arxiv.org/abs/1901.02181)

---

## 8. 파일 매핑

```
src/rocketsim/spacex/
  __init__.py              # public API
  convex_landing_mpc.py    # §2 (Layer 1)
  trajectory_tracker.py    # §3 (Layer 2)
  attitude_controller.py   # §4 (Layer 3)
  landing_controller.py    # §5 (전체 흐름)
```

evaluate_navigation.py 의 `spacex` 컨트롤러 = `LandingControllerSpaceX`.

---

## 9. 다음 확장 가능성

1. **Step 3 (SCP 6-DOF) 통합**: §2 의 MPC 를 자세 state 까지 모델링하는 비선형 → SCP 반복 변환. attitude_controller 는 그대로 / 약화 (MPC 가 자세까지 책임).
2. **Free-final-time**: T_final 자체를 결정 변수로. 초기 reduction (`T_final` 시간 grid 의 normalize) + bilinear-to-convex 변환.
3. **Plant-Mismatch Bias Estimator**: `accel_bias` 파라미터를 MPC 에 넣음 — 진짜 EDF 의 자기 캘리브레이션.
4. **LQR Inner Loop**: §3 의 PD+I 를 *진짜 LQR* (Riccati 해) 로 교체. 코우플드 게인.

---

## 10. 기록 흐름 갱신

```
... 이전 19개 ...
20. 2026-05-29_1753_v1_spacex_style_design.md           — algorithm diff + 설계
21. 2026-05-29_1830_v1_spacex_formulas_and_algorithms.md — 이 문서 (SpaceX-식 수식 통합)
22. 2026-05-29_1830_v1_spacex_results.md                 — 실험 raw (다음 background eval)
```
