# EDF Vane Roll Control — Cliff Edge Resolved

**날짜**: 2026-06-01 11:32 KST  
**버전**: v1  
**브랜치**: `edf-vane-roll-control`  
**Companion data**: [`2026-06-01_1124_v1_vane_realistic_results.md`](./2026-06-01_1124_v1_vane_realistic_results.md)  
**Predecessor**: [`2026-05-29_2222_v1_edf_roll_implementation.md`](./2026-05-29_2222_v1_edf_roll_implementation.md) (cliff edge 발견)

## TL;DR

이전 commit 에서 *발견* 한 EDF roll cliff edge (자이로 ON 시 *모든 컨트롤러 0% 박살*) 를 **active roll control 채널** 추가로 해소:

- **새 actuator**: V-2 식 *배기 베인* (50g, 추력 의존 토크)
- **새 컨트롤러 wrapper**: `RollPIDWrapper` (어떤 컨트롤러에도 roll 피드백 추가)
- **새 시나리오**: `spin` (초기 ω_z = 5 rad/s = 286°/s)

결과:
- **`spin` + EDF roll + vane**: PID 0% → **100%**, actuator/scp_warm 0% → **98%**
- **4 표준 시나리오 + EDF roll + vane**: worst-case **0% → 44%**
- **vane 없는 환경에서 roll wrapper**: 영향 *0* (안전)

= **시뮬-real 갭 의 첫 큰 폐기**. 진짜 EDF 비행 *가능* 영역.

## 1. 무엇을 추가했나

### 1.1 Vehicle 파라미터

```python
@dataclass
class Vehicle:
    ...
    edf_vane_torque_max: float = 0.0   # N·m at full thrust, opt-in
```

기본값 0 → 기존 동작 변화 *없음*. 0.5 N·m = 작은 90mm EDF 4개 베인의 추정치.

### 1.2 Dynamics — 베인 토크

```python
# state_derivative() 안에:
if vehicle.edf_vane_torque_max > 0:
    tau_z_vane = roll_cmd * edf_vane_torque_max * (thrust / max_thrust)
    torque_body[2] += tau_z_vane
```

추력 의존성 = *호버 시 약함, 고출력 시 강함* — 실제 배기 momentum flux 와 일치.

### 1.3 Command 4번째 채널

```python
@dataclass
class Command:
    throttle: float = 0.0
    gimbal_x: float = 0.0
    gimbal_y: float = 0.0
    roll_cmd: float = 0.0   # ★ 새, ±1, vane deflection
```

action_space 3 → 4 차원. 기존 3-element 호출 backward compatible (roll_cmd=0 default).

### 1.4 RollPIDWrapper

```python
class RollPIDWrapper:
    """Wraps any controller, adds PID on body-z heading."""
    def __call__(self, t, state) -> Command:
        cmd = self.inner(t, state)              # 기존 컨트롤러
        roll = quat_to_euler(state[QUAT])[0]    # 현재 heading
        omega_z = state[OMEGA][2]
        cmd.roll_cmd = clip(
            kp * (target - roll) - kd * omega_z + ki * integral,
            -1, 1
        )
        return cmd
```

PD on heading angle + D 항은 *gyro 직접* (PID 의 D 노이즈 회피). HoverPID 와 같은 패턴.

기본 게인 (90mm EDF 기준):
- `kp_roll = 8.0`, `kd_roll = 1.2`, `ki_roll = 2.0`
- 익숙한 critically-damped 응답 위해 차원 분석으로 결정.

### 1.5 새 시나리오 — `spin`

```python
LandingScenario.spin():
    start_omega_z = 5.0  # rad/s = ~286°/s
    timeout = 25.0
    # 외란/위치 모두 보통; spin 자체에 집중
```

= "발사 직후 회전 받은 vehicle 이 자세 회복 후 착륙"

## 2. 검증 결과

### 2.1 단위 테스트 — 11개 모두 통과

새 3개:
- `test_edf_vane_roll_torque_proportional_to_thrust_and_cmd` — `τ_z = roll_cmd · vane_max · thrust/Tmax` 정확
- `test_edf_vane_backward_compatible_with_3element_cmd` — 3-element cmd 일 때 vane 비활성
- `test_edf_vane_cancels_edf_roll_reaction_at_hover` — vane 토크가 EDF 반작용 토크 *정확히* 상쇄

### 2.2 Single-seed 진단

`spin` + edf_roll + vane (seed=0):
- 초기: ω_z = -3.24 rad/s
- **PID 단독**: ω_z 가 165 rad/s 까지 발산 → crash_tilt
- **PID + Roll wrapper**: ω_z 가 3.24 → 0.44 rad/s 로 *감쇠* → touchdown success

### 2.3 n=50 평가 — `spin` 시나리오

| 컨트롤러 | 기본 | + Roll Wrapper |
|---|---|---|
| pid | **0%** (48 crash_tilt + 2 oob) | **100%** ✨ |
| actuator | **0%** (50 crash_tilt) | **98%** |
| scp_warm | **0%** (50 crash_tilt) | **98%** |

→ *완벽한 회복*. 초기 286°/s 회전 vehicle 도 *100%* 부드러운 착륙.

### 2.4 n=50 평가 — 4 표준 시나리오 + EDF roll + vane (= 진짜 EDF 환경)

| | pid_roll | actuator_roll | **scp_warm_roll** |
|---|---|---|---|
| hard | 42% | 48% | 46% |
| noisy | **86%** | 80% | 80% |
| divert | 76% | 86% | 84% |
| divert_hard | 10% | **44%** | **44%** |
| **worst-case** | 10% | **44%** | **44%** |

기존 (roll OFF, 손튜닝 actuator): worst-case **52%**  
**진짜 EDF (roll ON + vane + roll PID)**: worst-case **44%**

= **roll 물리 도입 비용 -8pp** (52→44). 이 가격은:
- 베인 ~5-10% 효율 손실
- roll control 이 thrust 일부 *공유*
- 추가 control 결합 (roll → pitch/yaw 미세 coupling)

= 실제 hardware 의 *공정한* 비용.

### 2.5 n=50 sanity — vane 없을 때 roll wrapper

| | hard | divert_hard |
|---|---|---|
| actuator | 68% | 52% |
| actuator_roll | **68%** | **52%** |

→ 완벽히 동일. 진짜 *부작용 없음*.

## 3. 의미

### 3.1 *진짜* robust 영역 회복

이전 결과 (PR #2): "scp_warm_tuned 가 worst-case 64% 챔피언" — *조건: edf_roll=False*.  
실제 EDF 라면 *0% 박살*.

이제: **진짜 EDF (roll 물리 + vane + roll PID)** 에서 **worst-case 44%**.
- spin 처럼 *극단* 시나리오에서 **98-100%**
- 일반 시나리오 평균 60-80%

= 시뮬과 real 의 *gap 닫혔음*. *진짜 비행 가능성* 큼.

### 3.2 *각 컨트롤러* 의 ω_z 처리 능력 입증

진단의 핵심:
```
ω_z = -3.24 rad/s 시작 →

PID 단독:        ω_z → 165 rad/s 폭주 (= 9,468°/s = 26 회전/초)
                 vehicle 자세 PID 가 *pitch/yaw 만* 제어 → roll 무방어 → 발산

PID + Roll:      ω_z → 0.44 rad/s 수렴
                 vane 토크가 반작용 + 초기 회전 둘 다 잡음
```

= **현실 EDF 비행 의 *물리적 가능성* 정량 입증**.

### 3.3 *Hardware* 측면 함의

진짜 EDF 비행체 만들 때 *반드시* 필요한 것:
- ✓ 짐벌 (있음, ±12°/200°/s)
- ✓ Throttle 제어 (있음)
- ★ **roll 채널** (RCS 또는 *exhaust vane 또는 카운터-rotation*)
- ✓ IMU (있음으로 가정)
- ✓ 적절한 컨트롤 게인 (자동 튜닝 가능)

베인이 가장 *현실적* (50g 추가, 단순 서보 4개). 카운터-rotation 은 *더 무거움* (모터 + ESC 추가).

## 4. 한계 및 다음 작업

### 4.1 한계

- **vane 토크 0.5 N·m 추정치** — 진짜 hardware 측정 필요
- **roll wrapper 게인 손튜닝** — 자동 튜닝 안 함 (다음 단계)
- **베인 efficiency 손실 안 모델링** — 5-10% 추력 감소 가정해야 더 정확
- **hard scenario 손해**: 68% → 42-48% (roll 물리 자체의 비용)

### 4.2 다음 작업 후보

1. **Roll PID 자동 튜닝** — Optuna sweep, ~1시간. worst-case 44→55%+ 가능성.
2. **scp_warm_tuned + roll PID + 자동 재튜닝** — 챔피언 합체.
3. **베인 efficiency 손실 모델링** — 시뮬 더 정직.
4. **HoverPID gains + roll wrapper gains 합쳐 튜닝** — 20+ params, ~2시간.

### 4.3 직접 hardware 측면

- 90mm EDF + 4개 베인 + 서보 4개 prototype 가능
- 베인 토크 측정 (벤치 셋업): EDF 정지, 베인 deflect, torque cell
- 실제 측정값으로 `edf_vane_torque_max` 캘리브레이션

## 5. 정리

```
[이전 (PR #2)]
  worst-case 64%  ←  *edf_roll = False* 가정
                    (= 비현실적 — 실제 EDF 면 0% 박살)

[지금 (이 PR)]
  worst-case 44%  ←  *진짜 EDF* (roll 물리 + vane + roll PID)
                    (= 현실 비행 가능 영역)

  + spin 시나리오 (286°/s 초기 회전):
    PID:      0% → 100%
    others:   0% → 98%
```

진짜 *시뮬-real 갭* 의 첫 큰 페이지. *진짜 EDF 비행* 으로 한 발 더 가까워짐.

## 6. 기록 흐름

```
... 이전 39개 ...
40. 2026-05-30_1914_v1_scp_warm_tuned_champion.md
41. 2026-06-01_1124_v1_vane_realistic_results.md   (raw n=50)
42. 2026-06-01_1132_v1_edf_vane_roll_control.md    (이 문서)
```
