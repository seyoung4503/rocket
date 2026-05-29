# EDF Roll Physics — Implementation + Cliff-Edge Result

**날짜**: 2026-05-29 22:22 KST  
**버전**: v1  
**Predecessor**: [`2026-05-29_2108_v1_edf_roll_physics_design.md`](./2026-05-29_2108_v1_edf_roll_physics_design.md)

## TL;DR

설계 doc 의 EDF roll 물리 (팬 반작용 토크 + 자이로 세차) 를 `dynamics.py` 에 **opt-in 파라미터** 로 구현. 단위 테스트 3개 추가, 모두 통과.

평가 결과 — **scale = 0.05 (95% 카운터-rotation 상쇄) 만으로도 *모든* 컨트롤러 0%**. 즉 *카운터-rotation 만으로는 불충분*, **액티브 roll 제어 채널 (RCS 또는 반작용 휠) 이 *필수***.

## 1. 구현 — `Vehicle` opt-in 파라미터

`src/rocketsim/vehicle.py`:
```python
edf_roll_coeff: float = 0.0   # N·m per N thrust  (Q ≈ 0.012 · T 추천)
edf_fan_inertia: float = 0.0  # kg·m² (작은 EDF: ~1e-4)
edf_fan_omega_max: float = 0.0  # rad/s at full thrust (~4000)
```

기본값 0.0 → **기존 동작 유지**.

`src/rocketsim/dynamics.py` — `state_derivative` 에 추가:
```python
if vehicle.edf_roll_coeff > 0.0:
    # 반작용 토크 (body-z 축, 팬 회전 반대 방향)
    torque_body += [0, 0, -edf_roll_coeff * thrust]

if vehicle.edf_fan_inertia > 0.0 and vehicle.edf_fan_omega_max > 0.0:
    # 자이로 세차 — H_fan = I_fan · ω_fan_z
    omega_fan = edf_fan_omega_max * sqrt(thrust / max_thrust)
    H_fan_body = [0, 0, edf_fan_inertia * omega_fan]
    torque_body -= cross(omega_body, H_fan_body)
```

## 2. Env 통합

`make_landing_env(edf_roll=True, edf_roll_scale=1.0)` — 양쪽 다 켜기.

`edf_roll_scale` 은 *카운터-rotation 부분 상쇄* 모델링용:
- 1.0 = 싱글 팬 (완전한 반작용 토크)
- 0.1 = 카운터-rotating 팬으로 90% 상쇄 → 잔여 10%
- 0.0 = 완벽 상쇄 (= edf_roll=False 와 동일)

`evaluate_navigation.py` 에 `--edf-roll`, `--edf-roll-scale` CLI 플래그 추가.

## 3. 단위 테스트 (3개 추가, 8개 전부 통과)

- `test_edf_roll_off_means_zero_body_z_torque`: 기본값에선 z 토크 0
- `test_edf_roll_on_produces_reaction_torque`: coeff=0.012 → 정확한 ω̇_z 계산
- `test_edf_fan_gyro_couples_yaw_and_pitch`: pitch rate × fan H → yaw torque

## 4. 평가 결과 — Cliff edge

**`hard` 시나리오, n=20, estimated, actuator vs scp_warm**:

| scale | actuator | scp_warm | 비고 |
|---|---|---|---|
| 0.00 | 70% | 70% | EDF roll 없음 (= 기존) |
| **0.05** | **5%** | **0%** | 95% 상쇄 — 이미 붕괴 |
| 0.10 | 0% | 0% | 완전 0 |
| 0.20 | 0% | 0% | — |
| 0.50 | 0% | 0% | — |
| **1.00** | **0%** | **0%** | 싱글 팬 (full) |

→ **scale 0.00 → 0.05 의 *cliff*** — 70% 에서 0% 로 *떨어짐*.

실패 모드: 거의 모두 `crash_tilt` 또는 `out_of_bounds`. vehicle 이 *roll 발산* 으로 회전하다 자세 한계 (45°) 넘기거나 폭주.

## 5. 수치 감각 — 왜 5% 도 못 견디나

EDF 단일 팬 (full, scale=1.0):
- Q_react = 0.012 × 24.5 N = 0.29 N·m (hover 추력)
- I_zz (roll 관성) = 0.004 kg·m²
- α_roll = Q / I = 0.29 / 0.004 = **73 rad/s²**

scale=0.05 잔여:
- Q_res = 0.0145 N·m  
- α_roll = 3.6 rad/s²
- 5초 후 ω_z ≈ 18 rad/s (= 175°/s)
- 10초 후: 36 rad/s

자세 PID 가 roll 토크에 *접근 권한 없음* (짐벌은 pitch/yaw 만 만들고, gimbal × thrust 의 z 성분 = 0) → vehicle 이 잡지 못함 → 누적 발산.

= **roll 채널 없으면 *물리적으로* 안정화 불가능**.

## 6. 의미 — *진짜* hardware-realistic 시뮬

이게 *전체 baseline 의 의미를 바꿈*:

| | Roll 물리 없음 (현재까지) | Roll 물리 있음 (실제) |
|---|---|---|
| worst-case 성공률 | 60% (PID 18%) | **0%** ⚠️ |
| 어느 controller 가 robust? | scp_warm, actuator + smooth | **모두 실패** |
| 결론 | 알고리즘으로 *충분* | **하드웨어 (roll 채널) 필수** |

= **시뮬 마이크로-튜닝 의 한계** 명확히 드러남. 실제 hardware 로 가려면:
1. Counter-rotating fan (97%+ 상쇄, 어려움)
2. RCS (가스 분출) — 새 actuator
3. 반작용 휠 — 새 actuator
4. 블레이드 가변 피치 — 복잡

## 7. 다음 작업 후보

1. **Active roll control 채널 추가** — `Command` 에 `roll_torque` 추가, 컨트롤러 3곳 수정 (PID, MPC waypoint, SCP attitude). roll PID 추가. ~2-4시간.
2. **카운터-rotation 완성도 sweep** — scale 더 미세 (0.01, 0.02, 0.03) 으로 *진짜* cliff 위치 찾기. 30분.
3. **시뮬-real 캘리브레이션** — `edf_roll_coeff`, `edf_fan_inertia` 의 실제 측정값 (EDF 벤치) 으로 보정. hardware 단계.
4. **랩업** — 시뮬 마이크로-튜닝 한계 도달, 하드웨어 / HIL 단계로.

## 의미 정리

이전 baseline (worst-case 60%) 는 *roll 물리 없는* 가정 아래. 실제 EDF 단일 팬으로 hardware 가져가면 **0%**. 진짜 robust 한 시스템에 가려면:
- *Counter-rotation* 만으로는 부족
- *Active roll control* 채널 필수
- 또는 *EDF 가 아닌 추진* (단일축 토크 안 만드는 로켓 엔진)

이게 *시뮬 가지고 놀던 단계* 와 *진짜 비행* 의 가장 큰 갭. 솔직히 측정됨.

## 기록 흐름

```
34. 2026-05-29_2222_v1_edf_roll_implementation.md  — 이 문서 (구현 + cliff)
```
