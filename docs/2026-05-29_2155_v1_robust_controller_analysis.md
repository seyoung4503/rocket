# Robust Controller — Full SCP + Smoothing Sweep

**날짜**: 2026-05-29 21:55 KST  
**버전**: v1  
**Companion data**:
- `2026-05-29_2126_v1_scp_warm_results.md` — single-shot vs warm-SCP raw
- 본 문서 표 (smoothing sweep) 는 인라인

**Predecessor**: [`2026-05-29_2110_v1_step3_analysis.md`](./2026-05-29_2110_v1_step3_analysis.md)

## TL;DR — *Robust* controller 가 출현

이전 결과: 시나리오별 우승자 다 다름, 최약 시나리오 PID 18% (divert_hard).

이번 작업으로:
1. **Warm-start SCP** (시간-변동 linearization, 한 SCP 반복/replan): 단발 linearization 의 정확도 한계를 *trust region* 추가로 극복.
2. **xy_ref EMA smoothing**: noisy 가설은 위조됐지만 divert_hard 의 actuator 성능 +8pp.

결과 — **worst-case 60% 의 *진짜 robust* 컨트롤러 두 개 출현**:

| controller | hard | noisy | divert | divert_hard | **worst-case** |
|---|---|---|---|---|---|
| PID (baseline) | 58% | **84%** | 78% | 18% ⚠️ | **18%** |
| actuator (la=10) | **68%** | 78% | 90% | 52% | 52% |
| scp (single-shot) | 58% | 74% | 84% | 56% | 56% |
| **scp_warm + TR** | **70%** | 74% | 84% | **60%** | **60%** ✨ |
| **actuator + smooth_05** | 60% | 76% | **92%** | **60%** | **60%** ✨ |

PID 18% → 60% = **3.3× 향상**. 어떤 시나리오에서도 *최소 60% 성공*.

## 1. Warm-start SCP — 진짜 SCP 의 단일 반복 구현

### 알고리즘

```
for each replan:
    1. q̄(t) = previous_plan_q(t)  # 이전 plan 의 자세를 reference 로
    2. φ in MPC = rotation FROM q̄(t) TO actual q(t)  (small angle 가정)
    3. n̂_bar[k] = R(q̄[k]) · ê_z   →  cvxpy Parameter
       M_φ_bar[k] = R(q̄[k]) · skew_ê_z   →  cvxpy Parameter
    4. solve(): u = (T/m)·n̂_bar + (T_bar/m)·M_φ_bar·φ + g_world
    5. update q̄ ← q̄ ⊗ quat_from_phi(φ*)  for receding window
```

= 정통 SCP 의 *한 반복* per replan. cvxpy 문제는 *한 번만* 컴파일, 매 replan 마다 *parameter 만* 업데이트 (빠름).

### 첫 측정 — 큰 회귀 발생

| | scp (단발) | scp_warm (TR 없이) |
|---|---|---|
| hard | 58% | **66%** ✨ |
| noisy | 74% | 78% |
| divert | 84% | **70%** ⚠️ (-14pp) |
| divert_hard | 56% | **18%** ⚠️ (-38pp 붕괴!) |

진단: divert event (t=2s, 패드 +10m) 시 q̄(t) 는 *event 전* 의 plan (수직 강하 가정) 이고, 실제 q 는 *event 후* 수평 회복 시도 → φ[0] = rotation from q̄[0] to actual q 가 *큼* → linearization 신뢰 영역 벗어남 → MPC 가 *틀린* plan → vehicle 위로 박살 (`too_high` 24/50 on divert_hard).

### Trust Region 한 줄 fix

```python
if ||φ[0]|| > 0.30 rad (~ 17°):
    # 신뢰 영역 벗어남 → reference 리셋
    self._ref_q[:] = q_actual_now
    phi[0] = 0
```

이게 SCP 의 trust region — *linearization 정확 영역 안에서만* 사용. 큰 외란 (divert 패드 점프) 직후엔 자동 리셋.

### TR 추가 후

| | scp_warm (TR 없이) | **scp_warm (TR)** |
|---|---|---|
| hard | 66% | **70%** ✨ |
| noisy | 78% | 74% |
| divert | 70% ⚠️ | **84%** (회복) |
| divert_hard | 18% ⚠️ | **60%** ✨ |

한 줄 fix 가 *divert_hard 의 -38pp 붕괴를 +4pp 우위로 전환*. SCP 의 trust region 이 *알고리즘의 핵심* 임이 데이터로 확인됨.

## 2. xy_ref EMA Smoothing — noisy 가설 위조, divert_hard 부수 효과

### 가설 (위조됨)

> MPC 가 noisy estimated state 로 *흔들리는* plan 짬 → PID 의 D 항이 noise 반응 → noisy 약점 (74% vs PID 84%).

### Fix 시도

`LandingCvxpyWaypointPID` 에 `xy_ref_alpha` 파라미터 추가. EMA between consecutive replan's xy waypoint:
```python
self._xy_ref = alpha * new_ref + (1 - alpha) * self._xy_ref
```

alpha=1.0 = 무평활화 (기본), alpha=0.5 = "50% 새 plan + 50% 이전".

### Sweep 결과

| | actuator | smooth_05 | smooth_07 | scp_warm | scp_warm_smooth_05 | scp_warm_smooth_07 |
|---|---|---|---|---|---|---|
| hard | **68%** | 60% | 60% | 70% | 70% | 70% |
| noisy | **78%** | 76% | 78% | 74% | 74% | 74% |
| divert | 90% | **92%** | **92%** | 84% | 84% | 84% |
| divert_hard | 52% | **60%** | 58% | **60%** | **60%** | **60%** |

**noisy 가설 위조**: smoothing 이 noisy 의 6pp 갭을 *전혀* 줄이지 못함. PID 의 noisy 우위는 *MPC plan jitter* 가 원인이 아님.

**그러나** divert_hard 에서 actuator 의 +8pp (52→60%) — divert event 의 큰 xy 점프 시 smoothing 이 PID 의 over-correction 완화. 부수 효과.

scp_warm 은 이미 SCP/TR 메커니즘이 plan 안정화 → smoothing 으로 *더 안 좋아지지도, 더 좋아지지도 않음*.

## 3. *Robust* 컨트롤러 — 두 후보

worst-case 60% 두 컨트롤러 분석:

### `scp_warm + TR` (Step 3 정통 + 안전장치)

**프로파일**: 70/74/84/60. **균형형**. hard 와 divert_hard 에 강하고 noisy 와 divert 는 중상.  
**근거**: 자세 직접 모델링 + 시간-변동 linearization + 신뢰영역 → 어느 시나리오든 *원리적으로* 적합.  
**약점**: noisy 가 74% (PID 84% 대비 10pp 차) — 정직히 마이너스.  
**ML / 학습 가치**: SCP 의 정통 구현 — Falcon 9 의 알고리즘 라인.

### `actuator + smooth_05` (Step 1 + EMA)

**프로파일**: 60/76/92/60. **divert 형**. 패드 이동 시나리오에 *특히* 강함 (92% 최고).  
**근거**: 슬루 제약이 hard 에선 약함 (-8pp from actuator) 인데 smoothing 이 divert 의 plan jitter 완화.  
**약점**: hard 60% — actuator (la=10) 68% 보다 8pp 낮음.

### 어느 걸 추천하나

**`scp_warm + TR`**:
1. 균형성 (모든 시나리오 ≥ 60%, 4개 중 3개 ≥ 70%)
2. 정통 알고리즘 — 미래 확장 (full multi-iter SCP, free-final-time) 의 토대
3. 자세 모델링 명시 — 외란 더 강해지면 자연스럽게 활용 가능

**`actuator + smooth_05`**:
1. divert (패드 이동) 시나리오가 dominant 라면 최고
2. 구현 단순 — Step 1 + 한 줄 smoothing

용도에 따라 다름. *진짜 미래 비행 시나리오 분포* 모르면 **scp_warm + TR** 의 균형이 더 안전.

## 4. 진짜 발전한 부분 — *Worst-case* 메트릭

이전: best controller 의 worst scenario = actuator 의 divert_hard = 52%  
지금: 두 컨트롤러 worst-case 60% = **+8pp 향상**

PID 대비:
- PID worst-case: 18% (divert_hard)
- 우리 best: 60% (divert_hard)
- = **3.33× robust**

이게 *진짜* "어떤 상황에서도 잘 작동하는 컨트롤러" 의 정량적 증거.

## 5. 추가 가능성 — 멈춰도 되는 근거 + 더 갈 수 있는 길

### 멈춰도 되는 근거

- worst-case 60% 가 hop_test 으로 가기 충분한 baseline
- 다음 단계 (하드웨어, HIL, real flight) 가 더 큰 unknown 을 가져옴
- 시뮬 마이크로-튜닝의 마지널 효용 빠르게 감소

### 더 갈 수 있는 길

1. **Multi-iteration full SCP**: 매 replan 마다 trust-region 안에서 *여러* 반복 (수렴까지). divert event 같은 큰 변화 시 더 정확한 plan. 1-2일 작업.
2. **Plan-tracker 통합 노이즈 처리**: noisy 의 6pp 갭은 *측정 노이즈* 가 *PID 추적기* 까지 전파. estimator (LowPassEstimatorConfig) 의 시나리오별 자동 조정 시도. 30분-1시간.
3. **Adaptive controller selection**: 외란 강도 / 패드 이동 감지 후 시나리오별 best controller 동적 선택. 정통은 아니지만 실용적. 1일.
4. **Hybrid PID-MPC blending**: 시나리오 unknown 일 때 *둘 다* 돌리고 가중평균. 1-2시간.
5. **EDF roll 물리 추가** (`docs/2026-05-29_2108_v1_*`): 시뮬 현실성. 진짜 hardware 대비.

### 다음 우선순위 추천

1) **Step 3 의 multi-iter SCP** (정통 완성도 + divert_hard 추가 향상 가능성) → 또는 **5) EDF roll 물리** (시뮬 vs 진짜 hardware 갭 줄이기)
2) 둘 다 의미 있는 작업. 학습 가치는 1, hardware 가까이 가려면 5.

## 기록 흐름

```
28. 2026-05-29_2057_v1_step3_results.md   — single-shot SCP raw
29. 2026-05-29_2110_v1_step3_analysis.md  — single-shot 분석
30. 2026-05-29_2126_v1_scp_warm_results.md — warm-SCP raw (TR 없이)
31. 2026-05-29_2155_v1_robust_controller_analysis.md — 이 문서 (TR + smoothing + robust)
```
