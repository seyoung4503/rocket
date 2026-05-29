# Step 2 (Magnitude Lag) — Analysis

**날짜**: 2026-05-29 15:30 KST  
**버전**: v1  
**Companion data**: [`2026-05-29_1514_v1_step2_maglag_results.md`](./2026-05-29_1514_v1_step2_maglag_results.md)  
**Predecessor**: [`2026-05-29_1426_v1_actuator_ab_analysis.md`](./2026-05-29_1426_v1_actuator_ab_analysis.md), [`2026-05-29_0213_v1_hierarchical_mpc_plan.md`](./2026-05-29_0213_v1_hierarchical_mpc_plan.md)

## 변경 요약

`CvxpyActuatorAwareMagLagMPC` (Step 2) 추가. Step 1 (`CvxpyActuatorAwareMPC`)
대비 두 가지가 추가됨:

1. **추력 크기 1차 지연**: 스칼라 상태 `T[k]`, 스칼라 입력 `T_cmd[k]`, 동역학
   `T[k+1] = a·T[k] + (1−a)·T_cmd[k]` with `a = exp(−dt/τ_spool)`. 기본
   `τ_spool` = `vehicle.thrust_time_constant` (= 0.08s) — 시뮬레이터의 실제
   EDF 스풀업 상수와 *동일*하게 매칭.
2. **무손실 볼록 완화**: `||u[k]||₂ ≤ max_acc` → `||u[k]||₂ ≤ T[k]` 
   (Açıkmeşe G-FOLD 식). `T_cmd ∈ [0, max_acc]` 은 상한만 두고 비용 `r_Tcmd·
   (T_cmd − g)²` 로 hover 근처로 끌어당김. 스모크 테스트에서 `||u|| = T`
   타이트하게 성립하는 것을 확인 (→ 완화가 무손실).

Step 1 의 슬루 제약 (`||du|| ≤ β·u_z + slack`) 과 틸트 콘은 그대로 유지.
**여전히 점질량**.

## 결과 (n=50, mode=estimated, plant=nominal, 6 워커 병렬)

### Closed-loop 성능

| controller          | hard       | noisy     | noisy timeout |
|---------------------|------------|-----------|---------------|
| pid                 | 58%        | **84%**   | 0             |
| waypoint            | 52%        | 76%       | 3             |
| actuator (Step 1)   | 46%        | 74%       | 8             |
| **actuator2 (Step 2)** | **54%** | **80%**  | 7             |

**Step 1 → Step 2 개선**: hard **+8pp** (46 → 54), noisy **+6pp** (74 → 80).

**Step 2 vs PID**: hard **−4pp** (54 vs 58), noisy **−4pp** (80 vs 84). 
*개선됐지만 아직 PID 못 이김.*

**Landed-fail 분해 (hard)**: offset 12→10, hspeed 16→11, tilt 15→11 — *모든*
카테고리에서 호전. 평균 tilt 도 5.8°→5.4°.

**Landed-fail 분해 (noisy)**: hspeed 4→2, tilt 1→1. 더 부드러운 접지.

### Open-loop 진단 (`diagnose_mpc_model_mismatch.py`, n=20)

오픈루프 진단은 *MPC 계획을 TVC 트래커로 따라가게 한 뒤* 외란·피드백 없이
얼마나 빗나가는지 측정. 즉 "계획 자체의 *물리적 실현 가능성*" 만 본다.

| MPC               | hard pos_err | hard gimbal_sat | noisy gimbal_sat |
|-------------------|--------------|-----------------|------------------|
| pointmass         | 6.21 m       | 93%             | 93%              |
| actuator (Step 1) | 6.18 m       | 96%             | 88%              |
| actuator2 (Step 2)| **6.31 m**   | **96%**         | **88%**          |

**모든 오픈루프 메트릭이 거의 동일.** Step 2 의 계획도 점질량 + 무손실
완화일 뿐, 짐벌 입장에서 실현 가능성은 *전혀 안 개선됨*.

## 해석 — 왜 닫힘 루프는 좋아졌나

오픈루프 진단과 클로즈드루프 결과의 갭이 핵심 단서:

> Step 2 의 효과는 *더 실현 가능한 계획* 이 아니라 *더 부드러운 참조 신호*
> 였다.

근거:
- 짐벌 포화 96% → 그대로. 추력 벡터의 *방향* 변화는 여전히 한계에 박힘.
- 한편 `T_cmd` 비용 `r_Tcmd·(T_cmd − g)²` 와 lag 동역학이 *추력 크기 시퀀스
  를 부드럽게* 짐. PID 레이어가 짧은 시간 스케일에서 따라가기 쉬워짐.
- 접지 평균 tilt 가 줄고 hspeed/tilt 실패가 줄어든 패턴은 "참조가 덜 흔들려서
  PID 가 자세를 더 안정적으로 잡았다" 와 일치.

즉 Step 2 는 *MPC를 더 정직하게* 만든 게 아니라 *PID 가 좋아하는 모양으로
계획을 다듬어준* 셈이다. 부수적 효과인지, 우리가 원하던 "actuator-aware"
효과인지는 모호하다.

## 계획상 결정 기준

`2026-05-29_0213_v1_hierarchical_mpc_plan.md` §8 의 결정 트리:

> 충분 (포화 ≤ 30%, PID 능가) → 끝. hop_test 로 이동  
> 부족 (포화 50%+, PID 못 이김) → Step 3

현재 상태:
- 짐벌 포화 88~96% ≫ 30% — **부족**
- PID 못 이김 — **부족**

→ **두 기준 모두 부족 → Step 3 (SCP 6-DOF) 로 가는 게 계획대로의 결정.**

## 그래도 Step 2 를 남기는 이유

코드는 보존 원칙대로 유지 (`CvxpyActuatorAwareMagLagMPC` /
`LandingActuatorAwareMagLagWaypointPID`). 이유:

1. **벤치마크**: Step 3 가 정말로 Step 2 보다 나은지 *같은 환경*에서 비교
   해야 의미 있음. Step 1 / Step 2 / Step 3 를 한 표에 둘 수 있어야 한다.
2. **PID 친화적 부수 효과는 자체로 유용**: Step 3 가 잘 안 되면 백업.
3. **Step 3 가 필요 없을 수도 있는 시나리오 대비**: 외란이 약한 calm/
   moderate 에선 Step 2 만으로도 PID 동률·능가 가능성. 별도 검증 가치.

## 부산물 — `r_Tcmd` 의 영향 미확인

Step 2 의 개선이 *lag 자체* 때문인지 *`r_Tcmd` 평활화 비용* 때문인지 안
가렸음. 후속 ablation:

- `actuator2_r0`: `r_Tcmd = 0` (lag 만, 평활화 비용 없음)
- `actuator2_r0.1`: `r_Tcmd = 0.1` (5× 강한 평활화)

Step 3 가기 전 30분 정도면 끝나는 sweep. 이번 세션에 같이 끼우거나 Step 3
첫 검증과 함께 묶기.

## 다음 작업

1. ❑ (선택) `r_Tcmd` ablation — 만약 평활화가 주효과면 *Step 3 까지 안 가도
   참조 평활화로 PID 동률 가능* 시나리오 검토 여지가 생긴다.
2. ❑ Step 3: SCP 6-DOF MPC (`CvxpyScp6DofMPC` 후보 이름). cvxpy 한 번에
   못 풀어서 trust-region SCP 반복 필요. 문헌: arXiv 1811.10803, 1901.02181.
3. ❑ 같은 4 메트릭 (성공률, 짐벌 포화, plan-vs-rollout pos_err, 접지 평균)
   으로 비교 표 갱신.
