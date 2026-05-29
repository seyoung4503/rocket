# SpaceX MPC + Actuator-Aware Constraints — Two Surprises

**날짜**: 2026-05-29 20:45 KST  
**버전**: v1  
**Companion data**: [`2026-05-29_1903_v1_spacex_actuator_results.md`](./2026-05-29_1903_v1_spacex_actuator_results.md)  
**Predecessor**: [`2026-05-29_1845_v1_spacex_negative_result.md`](./2026-05-29_1845_v1_spacex_negative_result.md)

## TL;DR

Step 1 (슬루) 과 Step 1+2 (슬루 + mag-lag) 를 SpaceX-식 `ConvexLandingMPC` 에 포팅. 두 가지 *예상 밖* 결과:

1. **base spacex 가 *부수적으로* 크게 개선됨**. Plan-feasibility fix (descent-rate soft slack + skip v_max constraint at k=0) 가 *진짜* 음의 결과 였던 이전 평가의 부분을 해결: divert **4 → 32%** (+28pp), divert_hard **6 → 24%** (+18pp).
2. **그러나 actuator-aware 추가 (Step 1/Step 1+2) 는 *오히려* 나빠짐**. 가설 (lag 인지 → terminal hoverslam 부드러워짐) 반증.

## 정리 표 (n=50, estimated)

| | PID | actuator(la=10) | spacex(이전) | **spacex(now)** | spacex_actuator | spacex_actuator2 |
|---|---|---|---|---|---|---|
| hard | 58% | **68%** | 6% | **14%** | 10% | 8% |
| noisy | **84%** | 78% | 22% | **28%** | 10% | 14% |
| divert | 78% | **90%** | 4% | **32%** | 12% | 8% |
| divert_hard | 18% | **52%** | 6% | **24%** | 10% | 10% |

→ **base spacex 가 부수 fix 로 *4-5배* 개선**. 단 여전히 `actuator(la=10)` 압도적 우위.

## 발견 1 — base spacex 의 *부수* 개선 (Plan feasibility fix)

### 원인 진단

`v_max_desc = 2.0 m/s` 인데 hard IC 의 시작 vz 가 -3 ~ -5 m/s. 제약 `v[2, k] >= -v_max_desc` 가 k=0 (=초기 조건 = -3.37) 에서 *strictly infeasible*. CLARABEL 이 None 반환 → controller fallback (hover) → vehicle 그냥 떠있음 → timeout.

이전 평가 (`docs/2026-05-29_1830_v1_spacex_results.md`) 의 *대부분의 fail* 은 **알고리즘 음의 결과 가 아니라 *제약 InfeasibilityFault*** 였음.

### Fix

```python
if k > 0:
    constraints.append(
        v[2, k] + self.v_max_desc + desc_slack[k] >= 0
    )
cost = cost + self.w_soft * cp.square(desc_slack[k])
```

- k=0 skip → 초기 조건 강제 violation 해소
- soft slack → 초기 vz 가 -5 인 시드도 *몇 step 안에* -2 로 ramp 가능

### 효과

이전 진짜 음의 결과 였던 패턴 (모든 seed 가 거의 fall-through hover) 이 *해소*. 실제 plan 이 풀리고 vehicle 이 vehicle 답게 거동.

| | 이전 | 후 | 차이 |
|---|---|---|---|
| hard | 6% | 14% | +8pp |
| noisy | 22% | 28% | +6pp |
| divert | 4% | **32%** | +28pp |
| divert_hard | 6% | **24%** | +18pp |

→ 이전 "SpaceX 식이 우리 환경에 안 맞음" 결론은 *부분 과장*. 실제 *알고리즘 한계* 와 *코드 버그* 가 섞여있었음. 현재는 정직한 비교 가능.

## 발견 2 — Actuator-aware 추가가 *오히려* 나빠짐

### 데이터

| | spacex | spacex_actuator | spacex_actuator2 |
|---|---|---|---|
| hard | **14%** | 10% | 8% |
| noisy | **28%** | 10% | 14% |
| divert | **32%** | 12% | 8% |
| divert_hard | **24%** | 10% | 10% |

→ Step 1/2 가 모든 시나리오에서 base 보다 *못 함*.

### 가설 vs 결과

**가설**: 슬루 제약 + mag-lag → MPC plan 이 *실현 가능* 한 hoverslam burst 짠다 → vehicle 정확히 추적 → 부드러운 착지.

**결과**: 반대. plan 이 *더 부드러워짐* → vehicle 도 *더 부드럽게* 거동 → 횡 수렴 느림 + 강하 commit 느림 → 더 자주 timeout, touchdown 도 더 거침.

### Touchdown 분해 — 결정적 신호

hard 시나리오:

| controller | touchdown | soft | 실패 (hard landings) |
|---|---|---|---|
| spacex (base) | 14/50 (28%) | 7 (50%) | 7 (50%) |
| spacex_actuator (Step 1) | 27/50 (54%) | 5 (19%) | **22 (81%)** |
| spacex_actuator2 (Step 1+2) | 29/50 (58%) | 4 (14%) | **25 (86%)** |

→ Step 1/2 이 *지면 도달율* 은 올렸지만 (14→29) *soft landing 비율* 폭락 (50%→14%).

해석: 부드러운 plan + 시간 인덱스 tracker 가 *더 일찍 commit* 하지만 *vsspeed 정밀 제어* 못 함 → 빠르게 부딪힘.

### 왜 정반대 결과인가 — 진짜 메커니즘

이전 진단 (`docs/2026-05-29_1845_v1_spacex_negative_result.md`) 에서 정확히 지적한 것:

> "PID integrator + landing gate 의 *bundled* 디자인 강점" — lookahead wrapper 가 *우리 EDF + 단거리* 환경에 *극도로 최적화* 되어 있음.

이번 결과가 그걸 *직접* 보여줌:
- `actuator(la=10)` 의 wrapper 는 `HoverPID._z_int`, `_xy_int`, `LandingGuidance.touchdown_ready` 가 *함께* 작동. Plan 이 부드러우면 integrator 가 *적분 누적* 으로 *plan 이상* 추적.
- spacex stack 의 `TrajectoryTracker` 는 *integrator 약하고, gate 없음*. Plan 이 부드러우면 vehicle 도 *문자 그대로* 부드러움 → 횡 수렴 못 따라잡음.

→ **Plan 정확도 향상 (Step 1/2) + 약한 inner-loop = 더 못 함**. 같은 plan 정확도 향상이 *PID 강한 inner-loop 위* 에선 *살짝* 도움 (기존 데이터: actuator (la=10) 가 점질량 (la=10) 보다 +6-8pp). 두 효과의 *방향성* 이 *inner-loop 강도* 에 따라 다름.

### 더 깊은 의미 — 비용 함수 vs 시간 인덱스 추적

base spacex 는 *min-fuel + 강한 terminal* 비용 → coast-then-burn (hoverslam) plan. 마지막 burst 가 *날카로움*. tracker 는 그걸 ff 으로 받아 *그대로* burst 명령.

slew 제약 추가 시:
- plan 의 burst 가 *둔해짐* (du 제약으로 ramp 시간 필요)
- ramp 가 *plan 안에 그려짐* → MPC 는 *더 일찍* burst 시작하는 plan 짬
- 그러나 *transition 자체가 부드러워서* tracker 의 FF + PD 가 *명확한 trigger* 못 받음
- 결과: 강하 commit *지연* → 더 늦게 도달 → 더 큰 vspeed

이건 SpaceX 식 stack 의 *내재적* 트레이드오프 — *plan 정확도* 와 *tracker 의 commit 강제력* 이 *반비례* 관계일 수 있음.

## 결론

### 부분 검증된 것

- **base SpaceX 식이 *우리 환경에서 30% 정도* 동작** (이전 < 10% 가 아니라). 단 actuator(la=10) baseline (60-90%) 와는 여전히 큰 격차.
- 격차의 *주된 원인* 은 SpaceX tracker 의 *integrator/gate 부재*.

### 반증된 것

- "actuator-aware constraint 가 plan 정확도 올려서 closed-loop 성능 향상" — 우리 환경에서 *반대*. tracker 가 약하면 *부드러운 plan = 둔한 거동*.

### 실용적 결론

`actuator(la=10)` 가 여전히 strong baseline. SpaceX 식은 *구조적으로 깔끔* 한 reference 구현 으로 보존하되 *우리 시나리오* 에선 lookahead wrapper 가 정통식보다 우위.

진정한 SpaceX 식 stack 으로 가려면 tracker 에 *강한 integrator + commit gate* 추가 필요 — 이미 한 번 시도 (`LandingTrajectoryTrackingFullMPC`) 했고 실패 (`docs/2026-05-29_1845_v1_*`). 같은 방향으로 한 번 더 가는 건 ROI 낮음.

## 다음 작업 후보

1. **Step 3 (SCP 6-DOF) 로 진행**: actuator(la=10) baseline 위에서 *모델 정확도* 의 진짜 효과 측정. *이미 강한 inner-loop* 위에서 평가 가능.
2. **SpaceX tracker 에 integrator 강화**: kp_i 키우고 anti-windup 조정 — 한 sweep 가치 있을 수 있음 (반나절).
3. **랩업 + 하드웨어**: 시뮬에서 충분한 baseline (68/78/90/52%) 확보됐다고 판단, 다음 단계.

## 기록 흐름

```
24. 2026-05-29_1903_v1_spacex_actuator_results.md  — raw (5 ctrl × 4 scene)
25. 2026-05-29_2045_v1_spacex_actuator_analysis.md — 이 문서 (두 가지 발견)
```
