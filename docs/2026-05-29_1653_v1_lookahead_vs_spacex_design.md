# `lookahead` vs SpaceX-Style Trajectory Tracking — Design Discussion

**날짜**: 2026-05-29 16:53 KST  
**버전**: v1  
**Predecessor**: [`2026-05-29_1634_v1_hover_bug_fix.md`](./2026-05-29_1634_v1_hover_bug_fix.md)

## 동기

방금 hover bug 를 `lookahead=4 → 10` 한 줄로 풀었음 (mild divert 16% → 90%, 다른 시나리오도 동시 개선). 그러면서 자연스럽게 떠오른 질문:

> "이 `lookahead` 라는 트릭이 SpaceX 같은 정통 GNC 에서도 쓰는 방식인가?"  
> "왜 우리는 SpaceX 식으로 안 했나? PID 때문인가?"

답: **둘 다 No / Yes** — SpaceX 는 `lookahead` 같은 트릭 *안 쓰는 것으로 알려져 있고*, 우리가 그쪽으로 안 간 *주된* 이유가 **PID 의 기존 인터페이스를 재사용한** 엔지니어링 선택이었음.

이 문서는 그 차이를 명확히 정리.

## 두 방식 비교

### 우리 방식 (`lookahead`)

```
MPC 가 plan 짬:  ●●●●●●●●●●●●●●●●●●●●     (20점, 시간 0..4s)
                              ↑
                          #10 만 골라서
                              ↓
PID 에 setpoint 로 전달:  "여기로 가라"  (위치 한 점, 속도 무시)
```

매 0.2초마다 *plan 갱신 + 같은 인덱스 (#10) 뽑기*. PID 는 항상 "지금부터 2초 후 위치" 를 목표로 추격.

특징:
- plan 의 *한 점* 만 사용 (위치만)
- 그 점은 *항상 미래* (지금 + lookahead × dt)
- `lookahead` 가 튜닝 노브 (4 → 10 으로 바꿔서 버그 풀림)

### SpaceX 식 (정통 trajectory tracking)

```
MPC 가 plan 짬:  ●●●●●●●●●●●●●●●●●●●●     (20점, 시간 0..4s)
                  ↑
              지금 시간 (t=0) 의 reference
                  ↓
inner-loop:  "실제 (x, v) - reference[0] = 오차"
              "오차 * K = 보정 명령"

0.2초 뒤:    "실제 (x, v) - reference[1] = 오차" → 보정
0.4초 뒤:    reference[2] → 보정
... 시간이 흐르면서 자연스럽게 다음 점 따라감
```

매 step *현재 시간 인덱스* 의 reference 추적. plan 자체는 별도 주기로 (보통 더 빠름) 갱신.

특징:
- plan 의 *모든 시점* 사용 (시간이 흐르면서)
- 위치 + 속도 (+ 가속도) 함께 추적
- `lookahead` 같은 노브 없음 — 시간이 알아서 함

## 표

| | 우리 (`lookahead=10`) | SpaceX 식 |
|---|---|---|
| plan 의 몇 점 사용? | **1점** (인덱스 #lookahead) | **모든 시점** (시간 인덱스) |
| 어느 시점? | 항상 *미래* (지금+lookahead·dt) | *지금* 시점 |
| 무엇 사용? | 위치만 | 위치 + 속도 (+ 가속도) |
| 튜닝 노브 `lookahead`? | 있음 | 없음 |
| 코드 구조 | PID + setpoint 한 점 | trajectory-tracking 컨트롤러 (LQR/MPC inner) |
| 코드 복잡도 | 단순 | 복잡 (시간 인덱스 관리 + 다변량 게인) |
| MPC plan 의 *velocity* 정보 | 버림 | 사용 |
| Plan 정확도 향상 시 효과 | 한 점만 좋아짐 | 전체 궤적이 좋아짐 |

## 왜 우리는 SpaceX 식이 아니고 `lookahead` 식으로 했나

> *Yes, 주된 이유는 PID 였습니다.*

### 1. PID 가 *이미* 매우 잘 작동했음

프로젝트 진행 순서가 이랬어요:
1. **먼저 `LandingPID` 구현** — cascaded PID + 통합기 + landing gate. hard 58%, noisy 84% 의 강한 baseline.
2. **그 다음 MPC 시도** — "PID 위에 MPC 를 얹을 수 있을까?"

이 시점에 PID 는 **그 자체로 *잘 작동하는 검증된 컨트롤러*** 였음. 다 갈아엎고 trajectory tracking 인너 루프 다시 짜기보다는, *PID 의 setpoint 입력 인터페이스* 에 MPC 가 골라 준 한 점을 끼워넣는 것이 자연스러운 통합.

### 2. PID 의 setpoint 인터페이스가 *한 점* 만 받음

```python
class HoverPID:
    def __init__(self, ..., target=(0,0,0)):
        self.target = target  # 위치 한 점만 받음
```

PID 는 본질적으로 *오차 = target − 현재* 의 비례·적분·미분 보정. target 이 한 점이라 wrapper 가 MPC plan 에서 *한 점* 만 골라 줘야 함. 그 점이 *어디인지* 의 결정 = `lookahead`.

SpaceX 식이면 PID 가 아니라 trajectory-tracking 컨트롤러가 필요:
```python
class TrajectoryTracker:
    def __init__(self, ..., K_pos, K_vel):
        ...
    def step(self, t, state, ref_traj):
        p_ref = interpolate(ref_traj.p, t)  # 시간 인덱스
        v_ref = interpolate(ref_traj.v, t)
        return K_pos @ (state.pos - p_ref) + K_vel @ (state.vel - v_ref)
```

→ 새로 짜야 함. 게인 튜닝도 새로.

### 3. 보수적 split — "MPC 가 *틀려도* PID 가 최후 보루"

우리 hierarchical_mpc_plan 의 §4 에서 이 결정을 명시:

> Layer 1·3 에선 공존, Layer 2 에선 *둘 중 하나만*. 우리 변형들이 Layer 2 를 MPC 로 *교체* 함 → PID 의 외란 흡수 (적분·게이트) 사라짐 → 성능 ↓.

→ Layer 2 (위치 → 추력벡터) 는 PID 가 보존, MPC 는 *guidance* (Layer 1) 에서 setpoint 만 제공. 이게 우리 `LandingCvxpyWaypointPID` 의 구조.

SpaceX 처럼 *MPC 가 Layer 1+2 다 가져가면* PID 보루가 사라짐. 그러면 MPC plan 이 정확해야만 됨 → 우리가 *MPC 자체를 못 믿는* 단계에선 위험.

### 4. 개발 속도

| 방식 | 작업량 |
|---|---|
| PID + MPC setpoint (lookahead) | 1시간 (wrapper + 한 점 추출) |
| 정통 trajectory tracking | 1~2일 (인너 루프 새로 + 게인 튜닝 + 검증) |

빠른 iteration 사이클 (Step 1 → 측정 → Step 2 → 측정 → ...) 을 위해 *덜 invasive* 한 선택이 합리적이었음.

## 왜 SpaceX 는 정통 trajectory tracking 으로 가나

(공개 정보 + 추정 — 정확한 내부 구조는 비공개)

### 1. *처음부터* GNC stack 을 새로 설계함

SpaceX 는 PID-from-legacy 가 없었음. Falcon 9 landing 은 *최초* 사례 (재사용 로켓). GNC 팀이 처음부터 convex MPC + trajectory tracking 구조로 설계 가능.

### 2. *검증된 PID 가 없었음* → 처음부터 정통

우리처럼 "잘 작동하는 PID 위에 살짝" 이라는 경로가 *애초에 없었음*. 새 알고리즘에 매달리는 게 자연스러움.

### 3. 인너 루프 갱신 속도가 *훨씬* 빠름

추정: SpaceX 인너 루프 ≥ 100Hz, MPC plan 갱신 10~50Hz. 우리는 control_dt=0.02s (50Hz) PID + replan 5Hz MPC. *상대적* 으로 비슷한 비율인데, 그들은 절대적으로 더 빠르고 latency 가 작음.

### 4. *trajectory tracking 의 장점* 이 그들 시나리오에서 큼

수십 km 강하 + 정확한 좌표 착륙 → plan 의 *모든 시점* 정보가 가치 있음. 우리의 6초짜리 hop 에선 plan 의 중간점 정보 가치 < 단순 setpoint 만 줘도 *충분히* 정확.

## 우리가 정통으로 갈 수 있나

가능. ~1~2시간 작업:

1. **새 컨트롤러** `LandingTrajectoryTracker(K_pos, K_vel)` 작성 — `HoverPID` 대신.
2. **wrapper 수정** — `LandingCvxpyWaypointPID` 가 `lookahead` 한 점 대신 *전체 plan + plan_start_time* 을 tracker 에 넘김.
3. **시간 인덱스 보간** — `ref_at(t) = linear_interp(p_plan, v_plan, t - plan_start_t)`.
4. **게인 튜닝** — K_pos, K_vel 을 hard / divert 에서 재측정.

위험: 게인 튜닝이 까다로움. PID 의 *통합기* + *landing gate* 가 사라지므로 외란 흡수가 약해질 수 있음 (특히 noisy).

## 갈지 말지 결정 기준

권장 흐름:

```
[지금]  lookahead=10 베이스라인 (3/4 시나리오 PID 능가) — 충분히 좋음
   ↓
[Step 3]  SCP 6-DOF MPC 구현 + 측정
   ↓ 결과 보고 결정 ↓
   │
   ├─ Step 3 가 모든 시나리오에서 명확히 우위 → 끝. trajectory tracking 굳이 필요 없음.
   └─ Step 3 의 plan 정확도가 좋아도 closed-loop 결과 정체 →
        ★ *원인이 wrapper 의 한 점 추출 한계일 가능성*
        → 정통 trajectory tracking 으로 업그레이드
```

이렇게 가는 이유:
1. Plan 의 *모든 시점* 정보가 필요할 만큼 *plan 정확도* 가 충분히 좋아야 trajectory tracking 의 장점이 살아남.
2. Step 3 가 plan 정확도를 끌어올려 줄 *진짜* 단계. 그 시점에 wrapper 한계가 새 병목이 되면 그때 정통 갈 가치.
3. 지금 정통으로 가면 *plan 도 약한 (점질량) MPC + trajectory tracking* 조합 — 둘 다 약해서 효과 측정 어려움.

## 의미 정리

- **우리의 `lookahead` 트릭** = *PID 의 setpoint-한-점 인터페이스* 에 *MPC plan 의 한 점* 끼워넣기. 빠른 통합 위한 *공학적 단순화*.
- **SpaceX 식 trajectory tracking** = *정통* 방식. plan 의 전체 정보 사용. 인너 루프 새로 설계 필요.
- **차이의 근본 이유** = 우리는 *검증된 PID 위에 점진적 확장*, SpaceX 는 *처음부터 정통 GNC stack 설계*.
- **`lookahead` 노브가 *존재 자체로* 우리 방식이 정통 아님을 드러냄** — SpaceX 식이면 이런 노브 필요 없음.

## 다음 작업

- ❑ Step 3 (`CvxpyScp6DofMPC`) 구현 — 점질량 *진짜* 떠남.
- ❑ Step 3 결과 보고 trajectory tracking 업그레이드 필요성 평가.
- ❑ (옵션) divert_hard 의 PID `too_high` 별도 분석.

## 기록 흐름

```
... 이전 13개 ...
14. 2026-05-29_1634_v1_hover_bug_fix.md             — 진단 + lookahead fix
15. 2026-05-29_1634_v1_hover_bug_fix_results.md     — la=10 raw
16. 2026-05-29_1653_v1_lookahead_vs_spacex_design.md — 이 문서 (architecture 비교)
```
