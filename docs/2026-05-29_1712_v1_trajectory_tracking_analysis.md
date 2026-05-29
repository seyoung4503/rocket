# SpaceX-Style Trajectory Tracking — Counterintuitive Negative Result

**날짜**: 2026-05-29 17:12 KST  
**버전**: v1  
**Companion data**: [`2026-05-29_1700_v1_trajectory_tracking_results.md`](./2026-05-29_1700_v1_trajectory_tracking_results.md)  
**Predecessor**: [`2026-05-29_1653_v1_lookahead_vs_spacex_design.md`](./2026-05-29_1653_v1_lookahead_vs_spacex_design.md)

## TL;DR

"정통 SpaceX 식" time-indexed trajectory tracking 을 구현해서 우리 `lookahead=10` 트릭과 비교 → **모든 시나리오에서 *훨씬* 나쁨**. 이게 *처음엔 충격적*이지만 **사실은 우리 lookahead 트릭이 *MPC plan 의 게으름을 가리던 hack* 이었다** 는 것을 직접 보여줌. 깨끗한 분석.

## 결과 (n=50, estimated, 모두 같은 시드 풀)

| | PID | waypoint (la=10) | actuator (la=10) | actuator2 (la=10) | tracking_pointmass | tracking_actuator | tracking_actuator2 |
|---|---|---|---|---|---|---|---|
| hard | 58% | 54% | **68%** | 64% | 26% | 10% | **4%** ⚠️ |
| noisy | **84%** | 72% | 78% | 72% | 48% | 40% | 26% |
| divert | 78% | 94% | 90% | **96%** | 46% | 20% | 18% |
| divert_hard | 18% ⚠️ | **64%** | 52% | 52% | 12% | **0%** ⚠️ | 2% |

**두 가지 충격적인 패턴**:
1. **모든 시나리오에서 tracking < lookahead**. 50-70pp 하락.
2. **Actuator-awareness 더할수록 tracking 성능 떨어짐** (pointmass > actuator > actuator2). 정확히 *반대 방향*.

## 왜 이렇게 나쁜가 — 원인 분석

### 1. MPC plan 자체가 *천천히* 가는 plan

진단 (`docs/2026-05-29_1634_v1_hover_bug_fix.md`) 에서 mild divert seed=0 의 시계열 봤을 때:
- MPC plan 의 *0.8초 시점* (= `lookahead=4`): 시작 위치에서 거의 안 움직임
- MPC plan 의 *4초 시점* (= horizon end): 패드 근접

즉 MPC 가 "4초 동안 *천천히 부드럽게* 패드로 가자" plan 을 짬. **이게 plan 의 *정상* 모습이다 — q_pos·q_vel·r_u·r_du 가 smoothness 를 강제하는 결과.**

### 2. Lookahead 트릭이 *plan 의 미래* 를 *현재 PID 목표* 로 사용했음

`lookahead=10` (2초 앞) → PID 가 받은 setpoint = "2초 후에 있어야 할 곳". 현재 시간보다 *앞선* 점.  
→ PID 가 *지금부터 그곳까지* 빠르게 가도록 강하게 명령 → 실제 vehicle 이 plan 보다 *빠르게* 움직임.

### 3. Time-indexed tracking 은 *지금 시간* 의 plan ref 만 봄

`tracking_*` 은 `tau = sim_t - plan_t0` 에서 plan 보간:
- tau=0 (방금 replan): ref = plan[0] = 현재 vehicle 상태 → **오차 0, PD 안 함**
- tau=0.1: ref = plan 의 0.5 step ≈ 약간 미래
- tau=0.2 (다음 replan 직전): ref = plan[1] = 0.2초 후

따라서 tracker 는 plan 의 *지금 시점* 만 보고 *그대로* 추적. **MPC plan 이 게을러면 vehicle 도 게으르게 감.** PID 처럼 *plan 의 미래를 보고 가속* 하는 트릭이 없음.

### 4. Actuator-awareness 더할수록 plan 이 *더* 게을러짐

이게 *반전 방향* 결과의 원인:
- `CvxpyPointMassMPC`: 슬루 제약 없음 → plan 이 비교적 공격적 가능
- `CvxpyActuatorAwareMPC` (Step 1): 슬루 제약 `||du|| ≤ β·u_z` 추가 → plan 이 *부드러워짐*
- `CvxpyActuatorAwareMagLagMPC` (Step 2): + 추력 크기 1차 지연 → plan 이 *더* 부드러워짐

Lookahead 트릭은 plan 의 *미래* 만 보니까 plan smoothness 가 안 보임. Time-indexed tracking 은 plan 의 *현재* ref 를 따라가므로 smoothness 가 *바로* 추적 성능에 반영. → S1, S2 가 추적 성능 *떨어뜨림*. **정확히 모델링한 만큼 vehicle 도 천천히 감.**

## 의미 — 우리가 새로 알게 된 것

### A. `lookahead=10` 의 진짜 정체

> **공식적으로는** "MPC plan 의 2초 앞 점을 PID 에 setpoint 로" — 시계열 추적의 *간소화*.  
> **실질적으로는** "MPC plan 의 *진행 속도* 를 *2초 빨리 감기* 한 ref 를 PID 에 줘서 PID 가 *plan 보다 앞서가게* 강요". 즉 plan 의 *제약* 을 *우회*하는 hack.

→ "PID + lookahead 트릭" = "MPC plan 보고도 그것보다 더 빨리 가는" 메커니즘. 이게 우리 좋은 결과의 *진짜 원천*.

### B. SpaceX-style tracking 이 정상 작동 하려면 *MPC plan 이 더 공격적* 이어야 함

문헌 (Blackmore, Açıkmeşe) 의 SpaceX-style landing 은 minimum-time / minimum-fuel cost 로 최적화. 즉 plan 자체가 *시간 효율적* 으로 설계됨. 우리 MPC 의 q_pos / q_vel / r_u·r_du 조합은 *smoothness 우선* — 같은 코스트 구조가 아님.

→ time-indexed tracking 을 살리려면 **MPC 의 cost 를 minimum-time 식으로 재설계** 가 필요. 단순히 "wrapper 만 SpaceX 식" 은 작동 안 함.

### C. Actuator-aware MPC 가 *추적* 면에선 마이너스

이건 우리 hierarchical_mpc_plan 의 의도와 모순:
- *원래 의도*: actuator-aware MPC 가 *plan 정확성* 을 올린다 (짐벌 못 따라가는 plan 안 짬)
- *실제*: 정확한 만큼 *느린* plan → tracker 가 따라하면 vehicle 도 느려짐
- *lookahead 트릭 덕분에* 그동안 가려져 있었음 — lookahead 가 plan 의 *전체* smoothness 무시.

즉 Step 1, Step 2 의 *plan 정확성 향상* 은 lookahead 트릭과 *공존* 할 때만 효과를 봄. SpaceX-style 로 가면 actuator-awareness 가 *발목 잡힘*.

## 결론

1. **`lookahead=10` 트릭은 *의도된 디자인* 보다 *더 강력한* 우회법이었음**. 의도: setpoint 단순화. 실제: plan 게으름 우회.
2. **SpaceX-style trajectory tracking 은 *우리 MPC plan 위에서* 작동 안 함**. MPC plan 의 비용 함수를 minimum-time 으로 재설계해야 가능. ~반나절-1일 작업.
3. **actuator-aware MPC 가 tracking 면에선 마이너스** — 정확한 만큼 plan 이 게을러짐. lookahead 트릭이 이를 가리고 있었음.
4. **현실적 다음 단계**:
   - **(권장)** lookahead=10 유지. Step 3 로 진행. Step 3 plan 의 *모델 정확도* 가 lookahead 식 wrapper 와도 잘 어울리는지 측정.
   - (옵션) MPC cost 를 minimum-time 식 재설계 후 tracking 재실험. 정통 SpaceX 식 완성도 ↑.
   - (옵션) tracking 의 `anticipation` 파라미터 (lookahead 의 부드러운 버전) 로 sweet spot 찾기. 일종의 hybrid.

## 깨달음 - 시뮬보다 *실세계가 더 쉬운 경우* 가 있다?

사용자가 한 말: "*시뮬이 현실보다 훨씬 쉬울 텐데도 못한다*". 부분적으로는 *시뮬 자체가 쉽지 않다* — touchdown gate 가 빡빡함 (수직속도≤1, 수평속도≤0.5, 오프셋≤0.5m, tilt≤8°). 평균 인간 헬리콥터 조종사도 이 기준은 매우 어려움.

또: 우리 *제어 구조* 의 한계가 드러난 것 — MPC plan 게으름 + integrator-less tracker + 외란. 진짜 SpaceX 처럼 *최소 시간 비용 + 빠른 inner loop + LQR 통합* 까지 다 구축하면 더 좋아질 가능성 큰데, 그 작업이 클 뿐. 시뮬이 "쉬운데 못한다" 가 아니라 **우리 컨트롤러 구현이 미완성** 인 상태.

## 다음 작업 추천

1. ❑ (선택) `anticipation` sweep — tracking 의 *부드러운 lookahead 변형* 으로 sweet spot 탐색 (~30분). 만약 anticipation=1.0s 정도가 lookahead=10 과 동급이면 "둘은 같은 것의 다른 표현" 이라 결론.
2. ❑ Step 3 (`CvxpyScp6DofMPC`) 진행 — *지금까지의 strong baseline* (PID/waypoint/actuator/actuator2 at la=10) 위에서 정통 6-DOF MPC 의 효과 측정.
3. ❑ 따로 (장기) minimum-time MPC cost 변형 + tracking 재시험.

## 기록 흐름

```
... 이전 16개 ...
17. 2026-05-29_1700_v1_trajectory_tracking_results.md  — raw (7 ctrl × 4 scene)
18. 2026-05-29_1712_v1_trajectory_tracking_analysis.md — 이 문서 (음의 결과 + 원인 분석)
```
