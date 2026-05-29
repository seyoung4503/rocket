# SpaceX-Style Stack — Honest Negative Result

**날짜**: 2026-05-29 18:45 KST  
**버전**: v1  
**Companion data**: [`2026-05-29_1830_v1_spacex_results.md`](./2026-05-29_1830_v1_spacex_results.md)  
**Predecessor**: [`2026-05-29_1753_v1_spacex_style_design.md`](./2026-05-29_1753_v1_spacex_style_design.md), [`2026-05-29_1830_v1_spacex_formulas_and_algorithms.md`](./2026-05-29_1830_v1_spacex_formulas_and_algorithms.md)

## TL;DR

`src/rocketsim/spacex/` 에 *처음부터* 정통 G-FOLD-식 stack 구현 완료 (convex MPC + 시간 인덱스 tracker + 자세 PD). **그런데 우리 EDF 시뮬에서 *훨씬 못 함*** — lookahead 식 wrapper (`actuator` la=10) 의 1/10 수준.

| | PID | actuator (la=10) | spacex (default) | spacex (튜닝) |
|---|---|---|---|---|
| hard | 58% | **68%** | 6% | **0%** ⚠️ |
| noisy | **84%** | 78% | 22% | 12% |
| divert | 78% | **90%** | 4% | 16% |
| divert_hard | 18% | **52%** | 6% | 0% |

**튜닝**: `v_max_desc 4→2 m/s, kd_z 4→6` — tilt 평균은 좋아졌지만 vspeed 더 나빠짐 (5.30 m/s, 한계 1.0 의 5배).

## 원인 — 시뮬과 EDF 의 구조적 제약

### 1. Hoverslam 패턴 vs Actuator Lag

SpaceX-식 MPC 는 min-fuel 비용 + shrinking horizon 으로 *coast-then-burn* (자살 강하) plan 짬:
```
plan z: 12 → 9 → 5 → 1 → 0  (3.6s 동안 -4 m/s 강하)
plan u: 6.75 → 9.87 → 9.82 → 16.0 → 9.49 (마지막 단계 max thrust 으로 *순간* 감속)
```

이게 *수학적으로 fuel-optimal*. 그러나:
- EDF thrust 시간 상수 **τ = 0.08s** — throttle=1 명령 받아도 max thrust 도달하려면 ~0.2s
- 짐벌 rate 한계 **200°/s**
- → **마지막 0.2s 의 "deceleration burst" 가 *늦게* 도달** → vehicle 이 *큰 vz 로 지면 충돌*

실제 측정 (hard, vspeed 평균):
- PID: 0.09 m/s ✅
- actuator (la=10): 0.10 m/s ✅
- **spacex: 3.44~5.30 m/s** ❌ (한계 1.0 의 3~5배 — 충돌)

### 2. 짧은 거리에서 long-horizon advantage 부재

- Falcon 9: km 단위 강하, plan 정확도가 *결정적*
- 우리: 5~15m, 5~6초 — plan 의 *처음 1초만* 중요
- → MPC 의 *long-horizon* 강점 효과 미미

### 3. PID Integrator + Landing Gate 의 *적분 기여*

기존 `LandingCvxpyWaypointPID` 는 *외란 누적 보정* + *commit 게이트* 가 *셋트*:
- `HoverPID._z_int`: 무게 mis-estimate 흡수
- `HoverPID._xy_int`: 정상풍 흡수  
- `LandingGuidance` 의 `touchdown_ready` 게이트: 마지막 50cm 의 *gentle commit*

SpaceX-식 stack 의 PD+I tracker 도 적분기 있긴 하지만:
- *plan time-indexed 라서* tau≈0 에서 err 작음 → 적분 누적 안 됨
- 게이트 없음 → 마지막 commit 부재

### 4. 글라이드슬로프 vs 단거리 divert

divert 시나리오에서 패드 +10m 이동:
- MPC 가 *글라이드슬로프 30°* 안에 vehicle 을 끌어오려고 함
- 10m 옆으로 가려면 z ≥ 10·tan(30°) = 17.3m 가 *필요*
- 시작 z = 8-12m → 글라이드슬로프 *영역 밖* → soft slack 활성
- 활성 slack 페널티 w=1000 으로 옵티마이저가 *위로 끌어올려* 슬랙 해소 시도
- → too_high 실패 (9/50 in divert)

같은 문제로 divert_hard 에서 **10/50 이 out_of_bounds** (수평으로 폭주).

## 의미 — 가설 vs 데이터

### 가설 (`docs/2026-05-29_1753_v1_spacex_style_design.md` 의 H)

> "구조적 차이 (min-fuel + glideslope + time-indexed + 적분기 + 게이트 분리) 가 lookahead 트릭보다 *robust* 한 거동을 만들 것."

**반증** — 모든 시나리오에서 lookahead 식이 *훨씬* 우수.

### 더 깊은 이유

| 차원 | SpaceX 식 강점 | 우리 환경에서 |
|---|---|---|
| long-horizon planning | 분 단위 강하에 결정적 | *5초 강하* 라 효과 미미 |
| min-fuel | 연료 톤 단위 절감 | *전력 = 무한* 가정에선 의미 없음 |
| 글라이드슬로프 | bouncing 방지 | *단거리* 라 잔효과 + divert 와 충돌 |
| 정밀 hoverslam | TWR > 1 hover 불가 시 *유일* 해법 | *우리 TWR 1.6* — hover 가능, slam 불필요 |
| Time-indexed tracking | plan 정보 100% 활용 | actuator lag 으로 timing 정밀 불가능 |
| 적분기 분리 | 전체 stack 깔끔 | PID integrator + gate 의 *bundled* 디자인이 *우리 EDF 에 최적화* |

→ **우리 환경 (EDF + 단거리 + TWR>1) 은 SpaceX 식의 *각 장점이 적용 안 되는* 영역**. 반면 lookahead+PID 의 *bundled* 디자인은 우리 환경에 *과적합* 으로 잘 작동.

### 솔직한 자기비판

처음에 SpaceX 식을 "정통" 이라고 표현했지만 — **정통이 맞다 ≠ 우리 시나리오에 최적이다**.  
SpaceX 의 정통성은 *그들 시나리오* (Falcon 9, km 단위, 연료 결정적) 에 *맞춰* 진화한 것. 우리 EDF testbed 에선 *다른 최적화* 가 필요.

## 그래도 SpaceX 식 *코드는 보존*

`src/rocketsim/spacex/` 는 *교과서 같은 reference 구현* — 5개 파일, 깔끔한 layer 분리, 문헌 그대로:
- `convex_landing_mpc.py`: G-FOLD constraint set + lossless conv
- `trajectory_tracker.py`: 시간 인덱스 PD+I
- `attitude_controller.py`: 분리된 자세 PD
- `landing_controller.py`: 3 layer 통합

향후 *비교 baseline* 또는 *Step 3 (SCP 6-DOF) 의 시작점* 으로 가치 있음.

## 다음 작업 옵션

1. **(권장)** `actuator (la=10)` 을 strong baseline 으로 인정, **Step 3 (점질량 떠난 SCP 6-DOF) 로 진행**. Step 3 MPC 를 *기존 wrapper* 에 끼우는 방향.
2. SpaceX 식 stack 에 **actuator-aware constraint 포팅** (Step 1/2 의 슬루 + 매그 lag 을 spacex MPC 에 추가). 가능성 있음 (vspeed 3.44 의 원인 부분 해결 기대) 단 추가 작업 필요.
3. SpaceX 식 stack 의 **landing gate / commit logic 추가** — 정통성 일부 양보하되 우리 EDF 에 맞춤. 실용적이지만 "purity" 손실.

## 메타 — 이 작업으로 무엇을 배웠나

1. **알고리즘 정통성 ≠ 성능**: SpaceX-식 코드가 *클린* 하다고 *우리 환경에서 좋은 게 아님*.
2. **각 시나리오 마다 적절한 알고리즘 다름**: km 강하 → G-FOLD, 단거리 hop → PID+lookahead. *우리* 의 use case 가 후자.
3. **"hack 인 줄 알았던 트릭"이 사실 잘 *튜닝된* 디자인 이었음**: `lookahead=10` + HoverPID 통합기 + LandingGuidance 게이트는 *우리 시나리오에 *진짜로* 좋은 조합*.
4. **새 알고리즘 시도의 가치는 *negative result* 에도 있음**: SpaceX-식이 *왜* 안 되는지 *정량적으로* 측정 → 우리 시스템의 *진짜 bottleneck (actuator lag)* 부각.

## 기록 흐름 갱신

```
20. 2026-05-29_1753_v1_spacex_style_design.md          — 설계 (사전)
21. 2026-05-29_1830_v1_spacex_formulas_and_algorithms.md — 수식 통합
22. 2026-05-29_1830_v1_spacex_results.md               — raw (default 파라미터)
23. 2026-05-29_1845_v1_spacex_negative_result.md       — 이 문서 (음의 결과 + 원인)
```
