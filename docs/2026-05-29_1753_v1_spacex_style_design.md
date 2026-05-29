# SpaceX-Style Landing — Algorithm Differences & Design

**날짜**: 2026-05-29 17:53 KST  
**버전**: v1  
**Predecessor**: [`2026-05-29_1712_v1_trajectory_tracking_analysis.md`](./2026-05-29_1712_v1_trajectory_tracking_analysis.md)

## 동기

지난 시도들 (`tracking_*`, `full_*`) 은 *기존 PID-식 구조 위에 부분 부분만 SpaceX 식 으로 바꿔본* 하이브리드였고, 모두 실패. 사용자 요청대로 **새 폴더에 처음부터 정통 SpaceX 식 으로** 작성. 그 전에 *알고리즘 차이* 부터 명확히 정리.

## 한 줄 요약

**SpaceX 식 = "minimum-fuel/minimum-time 비용 + 글라이드슬로프 제약 + LQR 또는 PD trajectory tracking"** — 우리는 매 단계에서 *다른* 트레이드오프를 선택했음.

## 5가지 핵심 차이

### 1. MPC 의 *비용 함수* 가 본질적으로 다름

| | 우리 (`CvxpyPointMassMPC` 등) | SpaceX 식 (G-FOLD, Açıkmeşe) |
|---|---|---|
| **비용** | `q_pos·\|p\|² + q_vel·\|v\|² + r_u·\|u-hover\|² + r_du·\|du\|²` (LQR/regulator) | **`∑ \|\|u[k]\|\|`** (min-fuel) 또는 **`min T_final`** (min-time) |
| **의도** | "패드 *근처* 에서 *부드럽게*" | "**연료/시간 *최소화* 하면서 *정확히* 패드 위에**" |
| **결과 plan** | smoothness 우선 → *천천히 부드럽게* 강하 | aggressive committed descent — *최단* 또는 *최저-연료* 궤적 |

→ **smoothness 비용이 호버 버그의 *근본 원인***. min-fuel 비용은 thrust 를 *덜* 쓰려는 인센티브 → 빨리 내려와서 끝내려 함.

### 2. *글라이드슬로프 제약* 의 유무

SpaceX 의 *간판 제약*:
```
||p_xy[k]|| ≤ tan(γ) · p_z[k]      (γ ≈ 30° 같이 고정)
```
= 어느 고도 z 에 있든, *수평 거리는 z·tan(γ) 이하* — vehicle 이 패드를 향한 *원뿔* 안에서만 움직임. **이게 *bouncing/orbiting* 방지** 의 핵심.

우리는:
- 현재 *틸트* 콘 (`||u_xy|| ≤ tan(max_tilt)·u_z`) 만 있고 — 추력 벡터의 *방향* 제한
- *위치* 글라이드슬로프 없음 — 그래서 옆으로 벗어났다가 돌아오는 hover-and-recover 패턴 가능

→ Step 추가하면서도 *원래 G-FOLD 제약* 은 *빠져있었음*. 이게 hover 가능성을 열어둔 두 번째 구조 요인.

### 3. *수평선 (horizon)* 처리

| | 우리 | SpaceX 식 |
|---|---|---|
| **horizon 길이** | 고정 4초 receding | **shrinking 또는 free T_final** |
| **재계획 시** | "*지금부터* 4초" — 데드라인 따라옴 | "*총 T_final 시점* 까지" — 데드라인 *고정* |
| **결과** | "4초 뒤 도착" 영원히 미뤄짐 (= hover 버그) | "정해진 시점에 무조건 도착" — commit 강제 |

free-final-time:
- T_final 도 *결정 변수* 로
- 비용에 `+ w_T · T_final` (시간도 비용)
- bilinear 항이 생겨서 *직접* 비선형, 단 변환 가능 (Açıkmeşe lossless conv)

shrinking-horizon (간단한 대안):
- T_final 고정 (예: 시작 시 10s)
- 매 replan 마다 horizon N = (T_final − sim_t)/dt 로 감소
- 마지막에 N=1 이 됨 — 강제 commit

→ **이게 hover 트랩의 *진짜 해결***. 우리는 lookahead 트릭으로 *우회만* 했음.

### 4. *Inner-loop* 의 구조

| | 우리 (`LandingCvxpyWaypointPID`) | SpaceX 식 |
|---|---|---|
| **무엇** | PID setpoint 트래커 + xy 한 점 lookahead | LQR (또는 정밀 PD) + 전체 trajectory tracking |
| **MPC plan 활용** | xy 한 점만 (lookahead 시점) | (p, v, a) 전체 시계열 |
| **시간 인덱싱** | 없음 — setpoint 한 점 받음 | `tau = sim_t - plan_start_t` 로 ref(tau) 보간 |
| **자세 처리** | PID 자세 (PID 가 thrust→tilt→gimbal 변환) | LQR or thrust → quaternion → gimbal 직접 |
| **안전 게이트** | 별도 `LandingGuidance` (descent ladder, commit) | MPC 의 *제약* 안에 통합 |

→ 우리 inner-loop 의 모든 *구조* 가 PID 호환성 (single setpoint, integrator, gate) 에 맞춰져 있어서 plan trajectory 의 *velocity·acceleration 정보* 가 *체계적으로 손실됨*.

### 5. *안전 로직* 의 위치

| | 우리 | SpaceX 식 |
|---|---|---|
| max descent rate | 별도 `creep` / `gate` 게이트로 | MPC constraint: `\|v_z\| ≤ v_max` |
| 패드 근접 lockdown | 별도 `touchdown_ready` 게이트 | MPC final state: `p[N]=0, v[N]=0` |
| 글라이드슬로프 | 없음 | MPC constraint (위 §2) |
| 짐벌 한계 | 별도 후처리 (clip) | MPC constraint: `\|\|u_xy\|\| ≤ tan(θ)·u_z` |
| 추력 minmax | 후처리 clip | MPC constraint |

우리 코드는 안전 로직이 *분산* (pid.py, guidance/landing.py, wrapper) — 그래서 *override 디자인* 같은 어색한 hack 이 생김.  
SpaceX 식은 *전부 MPC constraint 로* — wrapper 가 "할 일이 없어짐".

## 정리 표

| 차원 | 우리 (현재) | SpaceX 식 |
|---|---|---|
| MPC 비용 | regulator (q·\|p\|² 등) | min-fuel `∑\|u\|` 또는 min-time |
| 글라이드슬로프 | 없음 | 핵심 제약 |
| Horizon | receding 4s | shrinking 또는 free T_final |
| Inner loop | PID setpoint | LQR / 전체 traj track |
| 시간 인덱싱 | 없음 (lookahead 한 점) | `tau` 보간 |
| 안전 로직 위치 | 분산 (PID gate, guidance) | MPC constraint |
| u 사용 | 한 시점 1차원 | 전체 (p, v, a) |

## 새 폴더 설계 — `src/rocketsim/spacex/`

```
spacex/
  __init__.py                 # public exports
  convex_landing_mpc.py       # 정통 G-FOLD 식 convex MPC
  trajectory_tracker.py       # 시간 인덱스 trajectory tracker (PD+I, no setpoint hack)
  attitude_controller.py      # thrust 벡터 → quaternion 추적 (PD on quaternion error)
  landing_controller.py       # top-level: MPC → tracker → attitude → command
README.md                     # design + 사용법
```

### `convex_landing_mpc.py` 의 핵심 설계

- 변수: p (3×N+1), v (3×N+1), u (3×N) — control 은 u 만
- 동역학: `p[k+1] = p[k] + dt·v[k] + 0.5·dt²·(u[k]+g)` 등
- 비용: **`w_fuel · ∑ \|\|u[k]\|\|₂ + w_term · \|p[N]\|² + w_termv · \|v[N]\|²`** (min-fuel + 강한 terminal)
- 제약:
  - `\|\|u[k]\|\|₂ ≤ a_max` (추력 magnitude)
  - `u[2,k] ≥ a_min` (one-sided, ≥ 최소 추력)
  - `\|\|u_xy[k]\|\|₂ ≤ tan(θ_max) · u[2,k]` (틸트 콘)
  - **`\|\|p_xy[k]\|\|₂ ≤ tan(γ_glide) · p[2,k]`** (글라이드슬로프 — 새로 추가)
  - `p[2,k] ≥ 0` (지면)
  - `v[2,k] ≥ -v_max_desc` (최대 강하속도)
  - Terminal: `p[N] = 0, v[N] = 0`
- **Shrinking horizon**: N_used = round((T_final - sim_t)/dt), N_used ≥ 2. plan 은 N_used 점만 의미.
  - 구현: cvxpy 변수는 max-N 으로 fix, 끝쪽 step 들은 추가 비용 없이 두거나 weight=0

### `trajectory_tracker.py` 의 핵심 설계

```python
class TrajectoryTracker:
    def step(self, t, state, plan):
        tau = t - plan.start_t
        p_ref, v_ref, a_ref = plan.at_time(tau)  # 보간
        p_err = p_ref - pos
        v_err = v_ref - vel
        self._p_int += p_err * dt  # 적분기 + anti-windup
        self._p_int = clip(self._p_int, ±lim)
        a_des = a_ref + K_p @ p_err + K_v @ v_err + K_i @ self._p_int
        return a_des
```

- **PID/LQR 게인** 은 HoverPID 의 검증된 값 재사용 (kp=[1.6,1.6,6.0], kd=[2.4,2.4,4.0], ki=[0.6,0.6,1.5])
- **시간 인덱싱** `tau = sim_t - plan_start_t` — 항상 "*지금* 시점 의 ref"
- **No setpoint hack**: a_ref 가 직접 feed-forward 가 됨

### `attitude_controller.py` 의 핵심 설계

기존 `_command_from_thrust_accel` 의 자세 PID 로직을 *별도 클래스* 로 분리. 함수만 옮기되 SpaceX 식 quaternion-error PD 도 옵션:

```python
def thrust_accel_to_command(self, state, a_des):
    # 1. a_des → thrust vector (in world)
    # 2. thrust direction → desired body-z (with tilt limit)
    # 3. body-z error (quaternion-based) → desired torque (PD on error)
    # 4. torque → gimbal angle (inverse kinematics)
    # 5. magnitude → throttle
```

같은 알고리즘이지만 *명확한 layered API* 로 정리.

### `landing_controller.py` — top-level

```python
class SpaceXLandingController:
    def __init__(self, vehicle, env, T_final=10.0, ...):
        self.mpc = ConvexLandingMPC(vehicle, env, T_final=T_final, ...)
        self.tracker = TrajectoryTracker(...)
        self.attitude = AttitudeController(vehicle, env)
        self._plan = None
        self._plan_start_t = -inf
    
    def __call__(self, t, state):
        # Replan periodically (or every step at fast rate)
        if t - self._plan_start_t >= self.replan_dt or self._plan is None:
            self._plan = self.mpc.solve(t, state)
            self._plan_start_t = t
        a_des = self.tracker.step(t, state, self._plan)
        return self.attitude.thrust_accel_to_command(state, a_des)
```

**No setpoint hack, no guidance ladder, no override** — 순수 plan → tracker → attitude.

## 검증 계획

1. **Smoke test**: 1 시드, divert 시나리오, 시계열 print. timeout 없이 도착하는지 확인.
2. **n=50 비교**: hard/noisy/divert/divert_hard 에서 4 컨트롤러 비교
   - PID (baseline)
   - actuator (la=10) (현재 best)
   - **spacex (new)**
3. **Open-loop 진단**: G-FOLD plan 의 짐벌 포화 / 추적 오차 측정. 우리 lookahead 와 같은 진단.

## 의미 — 이 시도가 왜 다를까

기존 시도들 (`tracking_*`, `full_*`) 은 *기존 PID 스택의 위* 에 *얇은 layer 만* SpaceX 식 으로 바꿈. 그래서:
- *PID setpoint 인터페이스* 그대로 → 한 점 setpoint
- *guidance ladder* 그대로 → override 디자인 문제
- *MPC cost* 그대로 → smoothness 우선

이번엔 *근본 부분* 부터 새로:
- **MPC cost: min-fuel + 글라이드슬로프 + shrinking horizon**
- **Tracker: 시간 인덱스 + 적분기 + LQR/PD (setpoint 아님)**
- **Attitude: 분리된 layered API**

→ 가능성 있음. 단 디버그 시 *기존 코드에 의존 없이* 처음부터 작동하는 게 핵심.

## 다음 단계

1. ❑ `src/rocketsim/spacex/` 폴더 생성, 4 파일 작성
2. ❑ evaluate_navigation 에 `spacex` 컨트롤러 추가
3. ❑ Smoke test → n=50 비교 → 진단
4. ❑ 문서 (results + analysis)
5. (옵션) MPC 의 정확한 G-FOLD 식 검증 — codex/문헌 cross-check
6. *그 다음에* Step 3 (점질량 떠나는 SCP 6-DOF)

## 기록 흐름

```
... 이전 17개 ...
18. 2026-05-29_1712_v1_trajectory_tracking_analysis.md — 음의 결과 + 원인
19. 2026-05-29_1730_v1_full_tracker_fixed_results.md   — min/max 수정 결과 (raw)
20. 2026-05-29_1753_v1_spacex_style_design.md          — 이 문서 (algorithm diff + 설계)
```
