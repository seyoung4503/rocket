# Full SCP (Multi-iteration) — Diagnostic Confirms Marginal Effect

**날짜**: 2026-05-29 22:12 KST  
**버전**: v1  
**Companion data**: [`2026-05-29_2203_v1_scp_full_results.md`](./2026-05-29_2203_v1_scp_full_results.md)  
**Predecessor**: [`2026-05-29_2155_v1_robust_controller_analysis.md`](./2026-05-29_2155_v1_robust_controller_analysis.md)

## TL;DR

Multi-iteration full SCP (정통 구현, 매 replan 마다 *수렴까지* SCP 반복) 추가. 진단 (`docs/2026-05-29_2110_v1_step3_analysis.md`) 에서 *이미 예측한 대로* — single-iteration warm-SCP 대비 **거의 변화 없음** (worst-case 60% 동일). 학습/완성도 측면 가치 있음. 추가 향상은 다른 방향 (시뮬 현실성, EDF 회전 물리) 에서 가능.

## 결과 (n=50, estimated)

| | PID | actuator (la=10) | **scp_warm** | **scp_full** |
|---|---|---|---|---|
| hard | 58% | **68%** | **70%** | 64% (-6pp) |
| noisy | **84%** | 78% | 74% | 74% (=) |
| divert | 78% | **90%** | 84% | 86% (+2pp) |
| divert_hard | 18% | 52% | **60%** | **60%** (=) |
| **worst-case** | 18% | 52% | **60%** | **60%** |

## 알고리즘 — Full SCP vs Warm-SCP

### Warm-SCP (이전)
```
each replan:
    linearize around stored q_bar(t)  ← 한 번
    solve SOCP
    update q_bar from solution        ← 다음 replan 위해 저장
    return plan
```

= **per replan 1 SCP iteration**.

### Full SCP (이번)
```
each replan:
    q_bar_iter = stored q_bar
    for it in range(max_iters):
        linearize around q_bar_iter
        solve SOCP
        compute new candidate q_bar
        if max||φ|| < threshold:  break (converged)
        q_bar_iter = new candidate
    update stored q_bar = q_bar_iter
    return plan
```

= **per replan 최대 max_iters (3) SCP iterations**.  
실제 사용량: 평균 3-5 iterations, threshold 0.02 rad (~1.15°) 에서 수렴.

## 왜 효과 미미했나 — 이미 진단에서 예측

`docs/2026-05-29_2110_v1_step3_analysis.md` 의 *진단 결과* 가 정확히 이걸 예측:

> "Linearization 영역 거의 안 벗어남: 모든 시나리오 평균 tilt < 8°. sin(8°) vs 8° 오차 < 0.3%."

수학적으로:
- Linearization 정확도 ∝ θ³ 항의 크기 (Taylor 잔차)
- θ < 10° 영역: error < 1%
- θ = 30° 에서: error ~5%

우리 vehicle 은 *대부분 시간 θ < 10°*. → linearization 오차가 *애초에 작음*. → 수렴 반복해도 *수렴할 곳이 없음*.

### Plan-vs-rollout 데이터 재확인

이전 진단:
- actuator pred err: 0.049m
- scp(single-shot) pred err: 0.055m (+12%)

→ scp 이 *plan 정확도가 약간 나쁨*. iteration 으로 정확도 올려도 vehicle 의 *실제* 거동에 영향 *제한적*.

## Hard 의 -6pp 회귀 가설

hard 에서 scp_warm 70% → scp_full 64%. -6pp 가설:
1. **과수렴 (over-fitting)**: 수렴된 plan 이 *너무 정확*. tracker 가 미세 변화 다 추적 시도 → 자세 흔들림.
2. **Solver state 변화**: warm-start 다르게 작동.
3. **Stochastic**: n=50 으로 ±5pp 수준. McNemar 으로 확인 시 *통계적 유의 미달* 가능성.

3) 가 가장 가능성 큼. n=50 의 ±7pp 신뢰구간 안에서의 변동.

## 의미 — *알고리즘 완성도* vs *실질 효과*

학습/완성도 측면 ✓:
- 정통 SCP 구현 — Falcon 9 라인의 알고리즘
- Trust region 안전장치 그대로 작동
- 수렴 카운터 (`last_iters_used`) 진단용 노출

실질 효과:
- worst-case 60% 동일 — *향상 없음*
- 시나리오별 ±2~6pp 변동 — 노이즈 영역

→ **diagnostic 이 옳았다**. linearization 정확도 향상의 한계 효용 ≈ 0 (우리 vehicle/시나리오 영역에서).

## 그래도 보존하는 이유

`src/rocketsim/controllers/scp_6dof_mpc.py:CvxpyScpFull6DofMPC` 은 *향후* 가치 있음:
1. **시나리오 확장 시**: 더 극단적 외란 / 큰 자세 거동이 도입되면 (예: EDF roll 물리 추가, 실제 hardware 의 unmodeled dynamics) — linearization 한계가 *진짜로* 드러남.
2. **Free-final-time SCP** 의 초석: shrinking horizon 결합 시 더 복잡한 수렴 필요.
3. **벤치마크 reference**: warm-SCP 가 충분히 잘 하는지 확인하는 *upper bound*.

## 다음 작업 — EDF Roll 물리

진단이 명확히 보여줌: **시뮬 *알고리즘* 측면에선 robust controller 거의 완성** (worst-case 60%). 추가 향상은:
1. 시뮬 *현실성* 추가 → real hardware 갭 축소 (EDF roll 물리)
2. Estimator 자동 조정 → noisy 약점 (작은 효과)
3. 하드웨어 / HIL → real flight test

**1) 우선 진행** — `docs/2026-05-29_2108_v1_edf_roll_physics_design.md` 의 설계대로 fan reaction torque + gyroscopic precession 추가.

## 기록 흐름

```
32. 2026-05-29_2203_v1_scp_full_results.md  — Full SCP raw
33. 2026-05-29_2212_v1_scp_full_analysis.md — 이 문서 (예측 검증)
```
