# 세션 핸드오프 — 2026-06-01 11:45 KST

다음 세션을 위한 *현재 상태 + 컨텍스트 + 재개 가이드*. 이 문서 하나로 다른 사람 (또는 미래의 나) 이 *지금 어디까지* 했고 *어디부터* 가야 할지 알 수 있게.

---

## 1. 현재 상태 한 줄

> **EDF 자이로 cliff edge 를 vane 으로 풀어서 worst-case 0% → 44% 회복. PR #4 review 대기. main 기준 새 시작 가능.**

---

## 2. 브랜치 / PR 상태

### 원격

| 브랜치 | 상태 | PR |
|---|---|---|
| `main` | 모든 머지된 작업 통합 | — |
| `actuator-mpc-spacex-experiments` | merged → revert 안 함 (PR #1) | #1 (merged) |
| `automated-tuning` | OPEN, review 대기 | **#3** ⏸️ |
| `edf-vane-roll-control` | OPEN, review 대기 | **#4** ⏸️ |
| `rocket-engine-regime`, `actuator-mpc-spacex-hoverslam`, `edf-fuel-hoverslam` | 로컬만, 다른 agent 작업 (확인 필요) | — |

### main 의 *현재* HEAD

```
7d694d1 Merge PR #1 (actuator-mpc-spacex-experiments)
```

= **PR #2 머지 *되돌려졌음*** (실수로 머지 후 revert). PR #3 의 변경은 *main 에 없음*.  
PR #3, PR #4 둘 다 **사용자 명시 시까지 머지 금지**. (`memory/pr-merge-explicit.md` 참조.)

---

## 3. 지금까지 누적 진행 — worst-case 그래프

```
시작 (PID baseline, edf_roll=False):                        18%
  ↓ +34pp  Step 1 actuator-aware MPC
PR #1: actuator (la=10):                                    52%
  ↓ +8pp   Step 3 + Trust Region + warm-SCP
PR #1: scp_warm:                                            60%
  ↓ +4pp   Optuna 15-param 자동 튜닝
PR #3: scp_warm_tuned:                                      64%   ← 시뮬 (이상화)
  ↓
[현실 EDF 적용 — edf_roll 물리 ON]                            0%   ← 충격: 모두 박살
  ↓ +44pp  vane + roll PID wrapper
PR #4: actuator_roll, scp_warm_roll:                        44%   ← 진짜 EDF 회복
```

**핵심**: 64% 는 *가상의 cliff 가 없는* 시뮬, 44% 는 *진짜* EDF.  
→ *실제 hardware 비행* 의 *현실적* 베이스라인 = **44%**.

### 시나리오별 현재 best (n=50, estimated)

**edf_roll=False (이상화 시뮬)**:
| | 챔피언 | 성공률 |
|---|---|---|
| hard | scp_warm | 70% |
| noisy | PID | 84% |
| divert | actuator_tuned | 100% |
| divert_hard | **scp_warm_tuned** | **64%** ★ |

**edf_roll=True + vane (진짜 EDF)**:
| | 챔피언 | 성공률 |
|---|---|---|
| hard | actuator_roll | 48% |
| noisy | pid_roll | 86% |
| divert | actuator_roll | 86% |
| divert_hard | actuator_roll / scp_warm_roll | **44%** ★ |
| **spin** | pid_roll | **100%** ✨ |

---

## 4. 작업한 큰 줄기 (시간순)

### Session 1 (2026-05-29) — PR #1 ✅ merged

1. **Step 1+2 actuator-aware MPC** — 슬루 제약 + 추력 lag
2. **Divert 시나리오** — 패드 +10m 점프
3. **Hover bug fix** — lookahead 4→10 (mild divert 16→90%)
4. **Trajectory tracker 실험** (negative result)
5. **SpaceX-style stack** (G-FOLD) — 작동하나 우리 환경엔 부적합
6. **Step 1+2 SpaceX 식으로 포팅** — 추가 효과 없음
7. **Step 3 (Linearized 6-DOF MPC)** — 자세 모델링
8. **Warm-SCP + Trust Region** — divert_hard 60% 달성
9. **Full SCP (multi-iter)** — single-shot 과 거의 동일
10. **EDF roll 물리** (opt-in, default off) — **cliff edge 발견**

### Session 2 (2026-05-30) — PR #2 (revert) / PR #3 ⏸️ review 대기

11. **Optuna 자동 튜닝 인프라** — `scripts/tune_controller.py`
12. **actuator 자동 튜닝** (6 param) — worst 52→58%
13. **scp_warm 자동 튜닝** (15 param) — worst 60→64% ★
14. PR #2 실수 머지 → revert → PR #3 재생성
15. (다른 agent) **로켓엔진 regime 계획** (구현 X)

### Session 3 (2026-06-01) — PR #4 ⏸️ review 대기

16. **EDF vane 물리** — `edf_vane_torque_max` + 4번째 Command 채널
17. **RollPIDWrapper** — 어느 컨트롤러에도 roll PID 추가
18. **`spin` 시나리오** — 초기 286°/s 회전
19. n=50 검증 — *cliff edge 풀림*, worst 0→44%

---

## 5. 코드 인프라 — 신규 사용 가능

### 컨트롤러

```
src/rocketsim/controllers/
  mpc.py
    CvxpyPointMassMPC           (Step 0)
    CvxpyActuatorAwareMPC       (Step 1, slew)
    CvxpyActuatorAwareMagLagMPC (Step 2, +mag lag)
    LandingActuatorAwareWaypointPID
    LandingActuatorAwareMagLagWaypointPID
  scp_6dof_mpc.py
    CvxpyScp6DofMPC             (Step 3, single-shot)
    CvxpyScpWarm6DofMPC         (Step 3, warm-start SCP + TR)
    CvxpyScpFull6DofMPC         (Step 3, multi-iter SCP)
    LandingScp*6DofWaypointPID  (각각의 wrapper)
  roll_wrapper.py    ★ 새
    RollPIDWrapper              (어느 컨트롤러에도 roll PID 추가)
  trajectory_tracker.py         (음의 결과, 보존)

src/rocketsim/spacex/    ★ SpaceX 스택
  convex_landing_mpc.py
    ConvexLandingMPC                  (min-fuel + glideslope)
    ActuatorAwareLandingMPC           (+ slew)
    ActuatorMagLagLandingMPC          (+ mag lag)
  trajectory_tracker.py
  attitude_controller.py
  landing_controller.py
```

### 시나리오

```
make_landing_env(difficulty, **kwargs):
  difficulty: calm | moderate | hard | unknown | recovery | noisy
              | divert | divert_hard | spin
  edf_roll: bool = False        ← 반작용 토크 + 자이로 ON
  edf_roll_scale: float = 1.0   ← 카운터-로테이션 효과 (0.05 cliff)
  edf_vane: bool = False        ← 베인 roll 제어 ON
  edf_vane_torque_max: float = 0.5  ← N·m at full thrust
```

### 자동 튜닝

```bash
.venv/bin/python scripts/tune_controller.py <target> \
    --trials N --episodes N --workers 6 --storage path.db

# target: actuator (6 params) | scp_warm (15 params)
# 새 target 추가: TARGETS dict 에 한 entry
# DB resume 가능 (--storage 같이)
```

### 평가

```bash
.venv/bin/python scripts/evaluate_navigation.py \
    --difficulties hard,noisy,divert,divert_hard,spin \
    --controllers pid,actuator,scp_warm,scp_warm_tuned,scp_warm_roll \
    --modes estimated \
    --episodes 50 --workers 6 \
    [--edf-roll] [--edf-vane] [--edf-roll-scale 0.05]
```

### 단위 테스트

```bash
.venv/bin/python tests/test_dynamics.py
# 11 tests pass
```

---

## 6. 미해결 / 진행 중

### 즉시 해야 할 일 (있음)

없음. 모든 commit 됐고, working tree clean, 2개 PR review 대기.

### Backlog 의 *next* 우선순위 (`docs/2026-05-30_0120_v1_unexplored_backlog.md`)

내가 추천하는 *임팩트 vs 노력* 순서:

#### ⚡ Quick wins (몇 시간)

1. **Roll PID 자동 튜닝** (Optuna sweep, ~1시간)
   - `tune_controller.py` 의 TARGETS 에 `actuator_roll` 추가
   - 6 params (slew_factor, slack_weight, q_pos_xy, q_final_pos_xy, xy_ref_alpha, lookahead) + 3 roll params (kp_roll, kd_roll, ki_roll)
   - 예상: worst-case 44 → 50-55%

2. **HoverPID 게인 자동 튜닝** (~1.5시간)
   - 9 게인 (kp_pos, kd_pos, ki_pos, kp_z, kd_z, ki_z, kp_att, kd_att, ki_att)
   - 모든 wrapper 의 *공통* — 큰 영향
   - 예상: 모든 시나리오 +3-5pp

3. **scp_warm_tuned + roll PID + 자동 재튜닝** — 챔피언들 합체
   - 15 (MPC) + 3 (roll) = 18 params
   - ~2시간

#### 🛠 Medium effort (반나절~1일)

4. **LQR / LQG baseline** — 정통 빠진 핵심 (~3-4시간)
5. **온라인 바람 추정** (Kalman) — hard/divert_hard 직접 (~반나절)
6. **MPC + RL Residual** — RL 인프라 *이미 있음* (이전 PPO 시도). 1-2일

#### 📚 Bigger experiments

7. **Multi-iteration SCP w/ varying iteration count** — Full SCP 의 trial 개수 sweep
8. **Robust MPC / Tube MPC** — 외란 uncertainty 명시 처리
9. **`spin` 외 더 어려운 회전 시나리오** — 빠른 spin, divert + spin

### 외부 dependency

- `optuna` 설치됨 (`.venv` 안)
- 다른 agent 가 만든 로컬 브랜치 (`rocket-engine-regime` 등) — 합치고 싶으면 확인 필요

---

## 7. 환경 주의사항

### Gitignore

```
.venv/, out/, models/, runs/, *.zip
.tuning/   ← Optuna SQLite 저장 (개인 실험 데이터)
```

`.tuning/` 의 DB 들은 *재실행 가능* — 결과 docs 에 best params 박혀있음.

### Python / 의존성

```
Python 3.14.4 arm64 (macOS)
.venv 안에:
  numpy 2.4.6
  cvxpy + CLARABEL (MPC 솔버)
  optuna 4.8.0    ← 자동 튜닝
  gymnasium 1.3.0, torch 2.12.0, stable-baselines3 2.8.0  (RL, 미사용)
```

### 워크플로 규칙 (메모리)

- `docs/` 새 파일: `YYYY-MM-DD_HHMM_v<n>_<topic>.md` 형식
- `docs/devlog.md` 에 세션마다 항목 추가 (이번 세션엔 *안 했음 — 핸드오프 후 추가*)
- ★ **PR 머지는 명시적 "머지/merge" 키워드 있을 때만**. "PR 보내" = push + create *only*. 자동 머지 *금지*. (이전 세션에서 실수 → revert)

---

## 8. 다음 세션 재개 — 명령어

```bash
cd /Users/seyoung/workspace/rocket
git fetch origin
git checkout main
git pull origin main
git log --oneline -5     # 어디까지 갔는지 확인

# Open PR 확인
gh pr list

# 새 브랜치 펴기 (다음 작업)
git checkout -b <next-feature-name>

# 가상환경 활성화
source .venv/bin/activate   # 또는 .venv/bin/python 직접 사용

# 단위 테스트 sanity
.venv/bin/python tests/test_dynamics.py
```

---

## 9. 알아두면 좋은 *맥락* (코드 못 보고 알기 어려운 것)

### 9.1 컨트롤러 설계 철학

- **PID + lookahead wrapper** 가 *우리 EDF 환경에 진짜 최적화* 되어 있음. SpaceX-style G-FOLD 가 *수학적으로 더 정통* 이지만 *우리 시나리오엔 안 맞음*. → "정통 알고리즘 ≠ 우리 환경 최적".
- **Trust Region** 이 SCP 의 *수학적 필수 안전장치*. 빼면 divert_hard 에서 -38pp 붕괴.
- **자동 튜닝이 사람보다 비직관적 가중치 찾음** — r_gimbal 5.3×, q_final_v_xy/v_z 비율 등.
- **MPC + LandingGuidance 의 *역할 분담* 이 핵심**. wrapper 가 *bundling* 되어 있어서 분리하면 성능 떨어짐.

### 9.2 시뮬-real 갭

- **edf_roll 물리** 가 *진짜 EDF 의 가장 큰* 미모델 효과. 추가하니 모든 알고리즘 0%.
- **vane** 추가로 회복 — 진짜 hardware 도 *roll 채널 필수*.
- 시뮬에선 64% 가능, **진짜 EDF 면 44% (현재)**.

### 9.3 결정적 진단들

- `docs/2026-05-29_2222_v1_edf_roll_implementation.md` — cliff edge 발견
- `docs/2026-05-29_1634_v1_hover_bug_fix.md` — wrapper 버그 vs 알고리즘 버그
- `docs/2026-05-29_2110_v1_step3_analysis.md` — linearization 오차 사소함
- `docs/2026-05-30_1914_v1_scp_warm_tuned_champion.md` — Optuna 가 사람보다 잘함
- `docs/2026-06-01_1132_v1_edf_vane_roll_control.md` — cliff edge 풀림

---

## 10. 정직한 한 줄 평가

> **64% (이상화) / 44% (현실 EDF) 의 worst-case 는 *알고리즘 한계 아님*. 백로그의 ~80% 가 *아직 안 시도*.** 자동 튜닝 한 사이클 더, 또는 LQR baseline, 또는 RL residual 추가하면 *상당히 더* 갈 수 있음.

---

## 11. 다음 세션 첫 메시지 추천

```
"main 받아서 docs/2026-06-01_1145_v1_session_handoff.md 읽었어.
[X 작업] 부터 시작하자."
```

[X] 자리에 추천:
- "Roll PID 자동 튜닝"   ← 가장 가성비 (1시간, +5-10pp)
- "HoverPID 자동 튜닝"   ← 큰 영향 (1.5h, 모든 컨트롤러 영향)
- "scp_warm_tuned + roll 합체"  ← 챔피언 합체
- "LQR baseline"        ← 정통성 빠진 핵심
- "MPC + RL residual"   ← 학습 frontier

---

이 문서가 다음 세션의 *anchor* 입니다. 여기서 시작하면 *맥락 손실 없이* 진행 가능.
