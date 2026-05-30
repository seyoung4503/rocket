# 자동 튜닝 작업 — 현재까지 진행 상황

**날짜**: 2026-05-30 17:47 KST  
**버전**: v1  
**브랜치**: `automated-tuning` (main 으로부터 분기)  
**Predecessor**: [`2026-05-30_0120_v1_unexplored_backlog.md`](./2026-05-30_0120_v1_unexplored_backlog.md) (튜닝이 *top 7 임팩트 항목* 중 첫 번째)

---

## 1. 자동 튜닝 = *컴퓨터가 최적 손잡이 찾기*

### 사람 vs 컴퓨터

**컨트롤러 (예: actuator) 의 손잡이 6개**:
| 손잡이 | 무엇 조절 | 범위 |
|---|---|---|
| `slew_factor` | 짐벌 변화율 한계 | 0.3 ~ 1.4 |
| `slack_weight` | 슬랙 페널티 강도 | 10 ~ 200 |
| `q_pos_xy` | 수평 위치 추적 강도 | 0.3 ~ 5 |
| `q_final_pos_xy` | terminal 수평 강도 | 10 ~ 200 |
| `xy_ref_alpha` | EMA 평활화 | 0.3 ~ 1.0 |
| `lookahead` | MPC plan 의 몇 step 앞 | 5 ~ 15 |

→ 6차원 공간. 가능한 조합 *무한*.

**사람 손튜닝의 한계**:
- 보통 3-5개 값 시도 → "1.2 vs 1.5 어느 게 나아?" 단순 비교
- 6차원 *상호작용* 못 봄 ("q_pos_xy 올리면 q_final 같이 내려야 하나?" 직관 없음)
- 한 번 튜닝에 *수 시간 ~ 수 일*

**Bayesian Optimization (Optuna TPE)**:
- 매 trial 마다 *지금까지 결과* 보고 *다음 시도할 위치* 자동 선택
- 6차원 공간을 *효율적* 으로 탐색 (랜덤보다 *10배+* 빠름)
- 컴퓨터가 *자동* — 사람은 *지켜만 봄*

### 한 trial 안에서 무슨 일

```
Trial k:
  1. TPE 가 "이번엔 손잡이를 X 위치로" 결정
  2. X 위치로 컨트롤러 만듦
  3. 4 시나리오 × 30 episode = 120번 비행 시뮬
     (6 워커 병렬, ~80초)
  4. 각 시나리오 성공률 측정
  5. 점수 계산: mean + 2 × min  ← worst-case 우선
  6. TPE 에 결과 보고 → 다음 trial 의 X 위치 결정
```

**총 작업**: 40 trial × 120 episode = **4,800 비행 시뮬**.  
**총 시간**: ~50분 (사람 손으로는 *수 일*).

---

## 2. 점수 (예: 2.025) 의미

```
score = mean(4 시나리오 성공률) + 2 × min(worst-case)
```

**왜 worst-case 에 2배 가중치?**
- 목표 = "*어느 시나리오든 *최소* 잘 됨*" (robust)
- *평균만* 최적화하면 한 시나리오 100%, 나머지 50% 도 OK → 균형 깨짐
- worst-case 가중치 2배 → 옵티마이저가 *최약점* 끌어올리는데 집중

**예 — Trial #21**:
- hard 63%, noisy 77%, divert 100%, divert_hard 63%
- mean = 76%, min = 63%
- score = 0.76 + 2 × 0.63 = **2.025**

= "*평균 76%, 어느 시나리오든 최소 63%*" 의미.

---

## 3. 인프라 — `scripts/tune_controller.py`

### 구조

```python
TARGETS = {
    "actuator": TuningTarget(suggest=..., build=...),
    "scp_warm": TuningTarget(suggest=..., build=...),
}

# suggest 함수 = 손잡이 6개의 range 정의
# build 함수  = 손잡이 값 → 실제 컨트롤러 인스턴스
```

새 컨트롤러 튜닝하고 싶으면 *한 entry 만* 추가하면 됨.

### 주요 기능

- **TPE Sampler** (Tree-structured Parzen Estimator) — 정통 Bayesian Opt
- **SQLite Storage** (`./.tuning/<target>.db`) — *중단 후 resume 가능*
- **ProcessPool Parallel** — trial 안 episode 6개 동시
- **Per-scenario user_attrs** — 각 trial 의 시나리오별 성공률 DB 저장 → 사후 분석 가능

### CLI 사용

```bash
# 40 trial × 30 episode/시나리오 sweep
python scripts/tune_controller.py actuator --trials 40 --episodes 30 --workers 6

# Resume (같은 storage 사용)
python scripts/tune_controller.py actuator --trials 60  # 기존 + 20 추가
```

---

## 4. 1차 sweep 결과 — actuator (40 trial 중 36 완료)

### 실행

- 시작: 01:28:40  
- 중단: ~02:15 (백그라운드 shell 끊김, 36 trial 완료)
- Storage: `.tuning/actuator_run1.db` (보존됨, resume 가능)

### 최고 trial — #21

**최적 손잡이 위치**:
```
slew_factor      = 1.39    (기본 0.9 보다 *더 공격적*)
slack_weight     = 197     (기본 50 의 *4배 — 슬랙 강한 페널티*)
q_pos_xy         = 3.71    (기본 1.2 의 *3배 — 정밀 추적*)
q_final_pos_xy   = 195     (기본 45 의 *4배 — 마지막에 정확히*)
xy_ref_alpha     = 0.75    (기본 1.0 보다 *살짝 부드럽게*)
lookahead        = 10      (기본과 동일)
```

**시나리오별 성공률 (n=30)**:
| | 기존 손튜닝 | **자동 튜닝** | Δ |
|---|---|---|---|
| hard | 68% | **63%** | -5pp |
| noisy | 78% | **77%** | -1 |
| divert | 90% | **100%** | +10pp |
| divert_hard | 52% | **63%** | **+11pp** ✨ |
| **worst-case** | **52%** | **63%** | **+11pp** ✨ |

= **worst-case +11pp** (52% → 63%). 손튜닝 한계 돌파.

### Top 5 trial — 수렴 패턴

```
#21  score=2.025  divert=100, divert_hard=63, hard=63, noisy=77
#26  score=2.017  divert=97,  divert_hard=63, hard=67, noisy=73
# 3  score=1.958  divert=100, divert_hard=60, hard=60, noisy=83
#27  score=1.958  divert=100, divert_hard=60, hard=70, noisy=73
#12  score=1.950  divert=100, divert_hard=60, hard=67, noisy=73
```

**관찰**:
- `divert` 는 모두 96-100% — *완전 해결*
- `divert_hard` 60-63% — 새 천장
- `hard` 60-70% — 약간 변동
- `noisy` 73-83% — 약간 변동
- TPE 가 *수렴 중* (top 5 가 비슷한 영역)

---

## 5. ⚠️ 검증 미진 — n=30 의 불확실성

n=30 episode 의 95% 신뢰구간 ≈ ±18pp. 즉 63% 측정 = *진짜* 값은 45-80% 어딘가.

→ **best trial 의 *n=50 재측정* 필요** (아직 안 함).

n=50 으로 다시 측정해야 +11pp 가 *진짜* 인지 확인.

---

## 6. 아직 안 한 것 (TODO)

- [ ] 남은 4 trial 완료 (40 까지)
- [ ] best trial (#21) 의 *n=50 재측정* — *진짜* +11pp 인지
- [ ] `scp_warm` 튜닝 sweep (15 파라미터, ~60 trial)
- [ ] HoverPID + LandingGuidance 튜닝 (20 파라미터)
- [ ] 결과 통합 commit + devlog 항목
- [ ] 더 큰 시나리오 set (calm, moderate, recovery 도)
- [ ] `--edf-roll` 환경에서도 튜닝

---

## 7. 인프라 가치 — 한 번 깔면 *영구* 사용

`tune_controller.py` 가 일반화 되어있어서:
- 새 컨트롤러 추가 → 5분 코드 → 자동 튜닝 됨
- 시나리오 추가 → 인자만 바꿈
- 목적함수 변경 → `objective_score` 한 줄 수정

→ *이번 세션* 만 효과가 아니라 *모든 다음 세션* 의 효율 ↑.

---

## 8. 다음 작업 흐름

```
[지금]  검증 + 튜닝 마무리
   ↓
1. 남은 4 trial 완료 (~7분)
2. Best trial n=50 재측정 (~5분)
   → +11pp 가 진짜인지 확인
3. Commit (인프라 + 1차 결과 + 검증)
   ↓
[이후]  더 큰 효과
4. scp_warm 60 trial sweep (~1.5시간)
5. HoverPID 튜닝 80 trial sweep (~2시간)
6. 종합 비교 + 결정
```

---

## 9. 기록 흐름

```
... 이전 33개 ...
34. 2026-05-30_0120_v1_unexplored_backlog.md
35. 2026-05-30_1747_v1_auto_tuning_progress.md  ← 이 문서
```
