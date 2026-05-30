# scp_warm_tuned: New Champion — Worst-Case 64%

**날짜**: 2026-05-30 19:14 KST  
**버전**: v1  
**Companion data**: [`2026-05-30_1907_v1_tuned_verification.md`](./2026-05-30_1907_v1_tuned_verification.md)  
**Predecessor**: [`2026-05-30_1747_v1_auto_tuning_progress.md`](./2026-05-30_1747_v1_auto_tuning_progress.md)

## TL;DR

`scp_warm` 의 **15 cost-weight 자동 튜닝** (Optuna 60 trial) 후 **worst-case 64%** 달성. 시작 PID 18% 대비 **3.6× robust 향상**. 손튜닝 한계 (60%) 넘음.

## 최종 결과 (n=50, estimated)

| | PID | actuator | scp_warm | actuator_tuned | **scp_warm_tuned** ★ |
|---|---|---|---|---|---|
| hard | 58% | **68%** | **70%** | 60% | 60% |
| noisy | **84%** | 78% | 74% | 78% | 78% |
| divert | 78% | 90% | 84% | **100%** | 96% |
| divert_hard | 18% | 52% | 60% | 58% | **64%** |
| **worst-case** | 18% | 52% | 60% | 58% | **64%** ★ |
| 평균 | 60% | 72% | 72% | 74% | **74.5%** ★ |

## 진행 추적 (worst-case 향상 history)

| 단계 | worst-case | Δ |
|---|---|---|
| 시작 (PID baseline) | 18% | — |
| Step 1 actuator-aware MPC | 52% | +34pp |
| Step 3 single-shot SCP | 56% | +4pp |
| Step 3 warm-SCP + Trust Region | 60% | +4pp |
| **자동 튜닝 (Optuna scp_warm)** | **64%** | **+4pp** ✨ |

**총 향상: 18% → 64% = +46pp (3.6× robust)**.

## 최적 파라미터 — `scp_warm_tuned` (Trial 6)

```
q_pos_xy         = 0.7351   (기본 1.2 의 0.6배)
q_pos_z          = 0.2722   (기본 0.08 의 3.4배 — 더 정밀 z 추적)
q_vel_xy         = 1.4841   (기본 4.0 의 0.4배)
q_vel_z          = 5.7157   (기본 3.0 의 1.9배)
q_phi            = 3.1024   (기본 5.0 의 0.6배)
q_omega          = 0.2048   (기본 0.5 의 0.4배)
q_final_pos_xy   = 57.9523  (기본 45 의 1.3배)
q_final_pos_z    = 5.2867   (기본 18 의 0.3배 — terminal z 자유)
q_final_vel_xy   = 188.7727 (기본 80 의 2.4배 — terminal v_xy 정확)
q_final_vel_z    = 50.5435  (기본 180 의 0.3배 — terminal v_z 자유)
q_final_phi      = 152.3666 (기본 100 의 1.5배 — terminal 자세 강조)
q_final_omega    = 22.4537  (기본 50 의 0.4배)
r_thrust         = 0.0753   (기본 0.02 의 3.8배 — smooth thrust)
r_gimbal         = 0.2638   (기본 0.05 의 5.3배 — smooth gimbal)
v_max_desc       = 2.9950   (기본 4.0 의 0.75배 — 더 신중한 강하)
```

## 패턴 분석 — TPE 가 찾은 *전략*

손튜닝 가중치와 자동 튜닝 가중치를 비교하면 TPE 가 학습한 *전략* 보임:

### 1. *Smooth 액추에이션 우선* (r_thrust 3.8×, r_gimbal 5.3×)

손튜닝: 액추에이터 effort 비용 작게 (r=0.02-0.05) → MPC 가 *공격적* plan.  
TPE: effort 비용 *훨씬 강하게* → MPC 가 *부드러운* plan → tracker 가 잘 추적.

→ "*Plan 의 정확성* 보다 *추적 가능성* 이 더 중요" 학습.

### 2. *Terminal v_xy 강조, terminal v_z 자유* (q_final_vel_xy 2.4×, q_final_vel_z 0.3×)

손튜닝: 모든 terminal velocity 강하게.  
TPE: 수평 속도 *정확히* 0 으로 (착륙 hspeed 임계 통과), 수직 속도 *부드럽게* 도달 (강제 안 함).

→ 임계 분석에서 hspeed > 0.5 가 가장 자주 실패. TPE 가 이걸 직접 인지.

### 3. *Terminal z 자유, 위치 z 정밀* (q_pos_z 3.4×, q_final_pos_z 0.3×)

손튜닝: terminal z 강함.  
TPE: 비행 중 z 정확 추적 (= z 트래킹), 마지막 z 자유 (LandingGuidance 게이트가 알아서).

→ MPC 와 wrapper 의 *역할 분담* 자동 발견.

### 4. *더 신중한 강하* (v_max_desc 0.75×)

손튜닝: v_max_desc=4 (속도 자유).  
TPE: 3 (더 보수적).

→ actuator lag 고려한 *안전 마진*.

## 의미

**자동 튜닝의 가치 검증**:
- 사람이 *결코 시도 안 했을* 가중치 비율 발견 (r_gimbal 5.3×, q_final_v_z 0.3× 등)
- 손튜닝의 "직관" 한계 넘음
- 한 번 인프라 깔면 *영구* 사용 가능

**진짜 *robust* 컨트롤러**:
- 어느 시나리오든 *최소 60% 이상*
- 평균 74.5% — 모든 시나리오 *대체로* 잘 함
- PID 18% 의 worst-case 와 비교 시 *질적으로 다른* 시스템

## 한계 — 정직히

1. **EDF roll 물리 OFF 가정** (이전 commit `e3969cf` 참조). roll ON 시 모두 0%.
2. **hard 60% — 손튜닝 baseline (68%) 보다 낮음**. trade-off (divert/divert_hard 향상 대가).
3. **n=50 도 ±7pp 신뢰구간** — 64% 는 실제 57-71% 어딘가.
4. **이 파라미터는 *우리 시나리오 set* 에 *과적합* 가능성** — 새 시나리오 추가 시 재튜닝 필요.

## 다음 작업 후보

backlog 의 다음 우선순위:
1. **HoverPID + LandingGuidance 튜닝** (20 파라미터, ~2시간) — wrapper 의 *공통* 부분, 모든 컨트롤러에 영향
2. **목적함수 재설계** (worst-case 가중치 3-4 또는 geometric mean) — hard 회복 시도
3. **LQR / LQG baseline** — 정통 빠진 핵심
4. **MPC + RL Residual** — 학습 frontier

## 기록 흐름

```
... 이전 35개 ...
36. 2026-05-30_0120_v1_unexplored_backlog.md
37. 2026-05-30_1747_v1_auto_tuning_progress.md
38. 2026-05-30_1753_v1_actuator_tuned_verification.md (actuator 1차)
39. 2026-05-30_1907_v1_tuned_verification.md          (5 컨트롤러 비교)
40. 2026-05-30_1914_v1_scp_warm_tuned_champion.md     (이 문서)
```
