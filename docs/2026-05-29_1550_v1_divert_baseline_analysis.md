# Divert Baseline — Surprising Asymmetry between PID and MPC

**날짜**: 2026-05-29 15:50 KST  
**버전**: v1  
**Companion data**: [`2026-05-29_1545_v1_divert_baseline_results.md`](./2026-05-29_1545_v1_divert_baseline_results.md)  
**Predecessor**: [`2026-05-29_1531_v1_divert_scenario_plan.md`](./2026-05-29_1531_v1_divert_scenario_plan.md)

## TL;DR

새 `divert` / `divert_hard` 시나리오 baseline (Step 3 추가 전):

| controller          | divert (mild) | divert_hard | hard | noisy |
|---------------------|---------------|-------------|------|-------|
| pid                 | **78%**       | **18%** ⚠️  | 58%  | 84%   |
| waypoint            | 42%           | 58%         | 52%  | 76%   |
| actuator (S1)       | 16%           | 56%         | 46%  | 74%   |
| actuator2 (S2)      | 20%           | **54%**     | 54%  | 80%   |

- **divert (mild)**: **PID 압도** (78% vs MPC 16–20%) — 가설 H1 (MPC favorable) **반증**.
- **divert_hard**: **MPC 변형들이 PID 압도** (54–58% vs PID 18%) — H1 *부분적으로* 검증.

**둘은 정반대의 실패 모드** 를 가짐. 그게 핵심 발견.

## 정확히 무엇이 일어났나

### divert (mild, moderate IC + moderate 외란)

| | landings | reasons                     | landed_fail               |
|---|---|---|---|
| pid       | 40/50 (39 soft) | touchdown:40, **too_high:10**       | offset:1                  |
| waypoint  | 21/50           | touchdown:21, **timeout:29**         | —                         |
| actuator  | 8/50            | touchdown:8, **timeout:42**          | —                         |
| actuator2 | 10/50           | touchdown:10, **timeout:40**         | —                         |

**MPC 변형은 *호버링 / 타임아웃*** — 패드 이동 후 새 패드 위에서 commit-to-descent 를 못 함. PID 는 단순히 새 목표로 추종하면서 착륙.

### divert_hard (hard IC + hard 외란 + divert)

| | landings | reasons                                    | landed_fail              |
|---|---|---|---|
| pid       | 13/50 (9 soft)  | touchdown:13, **too_high:37**                       | hspeed:3, tilt:2        |
| waypoint  | 50/50           | touchdown:50                                | offset:13, hspeed:10, tilt:14 |
| actuator  | 50/50           | touchdown:50                                | offset:16, hspeed:7, tilt:14  |
| actuator2 | 49/50           | touchdown:49, too_high:1                    | offset:14, hspeed:11, tilt:13 |

**PID 의 37/50 too_high** — 강 외란 + divert 의 *큰 수평 오차* 에 PID 가 통합기 windup + 공격적 자세 명령으로 *위로* 날아감 (max_altitude=20m 초과 추정). MPC 변형은 *전부* 착륙은 함 (성공률은 50~58%, 부드러운 착륙 실패는 따로).

## 의미와 해석

### H1 (MPC structural advantage) — *경계 조건에서만* 참

가정: "패드 이동 = receding-horizon replanning 우위". 결과:
- **외란이 약하면** (divert mild): MPC 의 plan-and-track 패턴이 commit 시점을 못 잡음. PID 의 단순 reactive 가 *더 잘 함*. 가설 반증.
- **외란이 강하면** (divert_hard): PID 의 windup 이 catastrophic. MPC 의 plan 기반이 *훨씬 robust*. 가설 검증.

즉 "*MPC vs PID*" 가 단일 차원이 아니라 **시나리오의 외란 강도와 우위가 교차** 함. 결정적 발견.

### Step 1 vs Step 2 — *또* 동률

| | divert | divert_hard |
|---|---|---|
| actuator (S1)  | 16% | 56% |
| actuator2 (S2) | 20% | 54% |

평소처럼 노이즈 안. McNemar p > 0.3. → 일관되게 "S1/S2 는 통계적으로 구분 불가".

### waypoint (베이스라인 MPC) ≈ actuator (S1) ≈ actuator2 (S2)

세 변형이 divert_hard 에서 58% / 56% / 54% — 사실상 동일. **MPC 변형 *내부*** 의 차이는 미미; **PID vs MPC** 가 진짜 분기점.

## 실패 모드 분석 — MPC 의 hover-and-timeout 원인

divert (mild) 에서 MPC 가 timeout 하는 이유 추정:

1. **commit logic 부재**: waypoint MPC 는 *현재 위치 → 패드* 의 receding-horizon plan 을 매번 새로. 패드까지 가는 path 가 항상 4초 미래로 push 됨. 진짜 강하 commit 신호가 없음.
2. **PID 게이트는 분리됨**: LandingPID 의 *landing gate* (특정 alt+approach 조건 만족 시 강제 hover→commit 전환) 가 wrapper 에 들어가 있긴 하지만, MPC plan 이 "내려가지 않는" 시계열을 만들면 게이트가 발동 안 함.
3. **q_pos[xy] = 1.2 vs q_pos[z] = 0.08**: 수평 비용이 *수직 비용의 15배*. 큰 수평 오차 (Δ=10m) 일 때 수직 강하를 *희생* 해서 수평을 우선.

→ MPC 의 divert(mild) 실패는 *모델 정확성* 문제가 아니라 *컨트롤러 아키텍처* 문제. Step 3 (정확한 모델) 로 갈수록 *덜* 풀릴 가능성도 있음 — Step 3 plan 도 같은 비용 구조에서 같은 hover-bias 를 가질 수 있음.

## PID 의 divert_hard 실패 모드 — `too_high`

37/50 episode 가 max_altitude 초과. 추정:
- divert 시점 (t=2s) 에 갑자기 10m 수평 오차 발생.
- PID 의 P term 이 큰 자세 명령 → 강한 수평 가속 + 수직 thrust 감소를 위해 큰 tilt → vehicle 이 수평 가속 받음.
- 동시에 hard 외란이 위로 미는 펄스를 겹치면 → upward 가속.
- max_altitude=20m 초과 → too_high reason 으로 종료.

PID 의 windup 안전장치 없는 게 드러남. 이건 PID 의 *컨트롤러 수준* 문제 (게인·통합 한도 조정으로 *부분* 해결 가능, 그러나 시나리오별 튜닝 필요).

## 결정적 발견 — 우리가 비교하던 것이 무엇이었나

기존 hard/noisy 에선 *우리도 모르는 사이* "외란 약함 + 단거리 강하" 환경에서 평가하고 있었음. 이건 PID 의 *최적 시나리오*. divert_hard 같은 *큰 갑작스러운 오차* 가 들어가니 PID 가 무너짐.

→ "MPC 가 PID 못 이긴다" 는 *기존 시나리오에서만* 참. 더 일반적인 robust performance 측면에선 MPC 가 더 안정적일 가능성.

## H2/H3 평가 미정 — Step 3 가 더 흥미로워짐

원래 H2: "Step 3 가 추가되면 divert 에서 의미 있게 개선". 지금 데이터로 보면:

- **divert_hard 에서**: MPC 가 이미 PID 압도. Step 3 가 *얼마나 더* 끌어올리느냐 — 흥미. 50~58% → 70%+ 가능성?
- **divert(mild) 에서**: MPC 의 실패는 hover commit 부재. Step 3 plan 정확도가 *이걸 풀어줄지* — 불확실. Plan 의 비용 구조가 같으면 같은 hover-bias.

H3 (기존 hard/noisy 결론 유지) 는 여전히 합리적 — 그 환경에선 PID 가 이미 좋음.

## 다음 작업

1. ❑ Step 3 (`CvxpyScp6DofMPC`) 구현 — 정통 SCP 6-DOF.
2. ❑ 4 시나리오 × 4 컨트롤러 (PID, waypoint, actuator, actuator2) + Step 3 = 5 컨트롤러. 동일 시드 비교.
3. ❑ 분석: 
   - divert 에서 Step 3 가 hover-and-timeout 풀어주는가?
   - divert_hard 에서 Step 3 가 50~58% → 70%+ 가는가?
   - hard/noisy 에서 Step 3 가 PID 와 동률에 가까워지나?
4. ❑ (옵션) MPC wrapper 의 *commit-to-descent* logic 점검 — divert 의 hover-timeout 패턴이 Step 3 에서도 보이면 Step 3 와 별개로 wrapper 수정 필요.

## 기록 흐름 (사용자 요청)

이 시리즈의 문서 흐름:
1. `2026-05-29_0213_v1_hierarchical_mpc_plan.md` — 3단계 로드맵
2. `2026-05-29_0248_v1_formulas_and_algorithms.md` — 수식·알고리즘 정리
3. `2026-05-29_1422_v1_actuator_ab_sweep.md` — Step 1 가중치 sweep raw
4. `2026-05-29_1426_v1_actuator_ab_analysis.md` — H 반증
5. `2026-05-29_1514_v1_step2_maglag_results.md` — Step 2 raw
6. `2026-05-29_1530_v1_step2_maglag_analysis.md` — Step 2 분석 (closed-loop ≠ open-loop)
7. `2026-05-29_1531_v1_divert_scenario_plan.md` — divert 설계·가설
8. `2026-05-29_1545_v1_divert_baseline_results.md` — divert raw (현재)
9. `2026-05-29_1550_v1_divert_baseline_analysis.md` — 이 문서

다음: Step 3 구현 계획 문서 → 구현 → 결과 → 분석.
