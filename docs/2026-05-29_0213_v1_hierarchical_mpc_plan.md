# Hierarchical MPC 계획서 — 사고 과정 정리

- **날짜**: 2026-05-29 02:13 KST
- **버전**: v1
- **목적**: "왜 우리 MPC가 PID를 못 이기는지" 진단부터 "어떻게 SpaceX식 계층형으로 갈지" 결정까지의 사고 과정을 빠짐없이 정리. 미래의 나/협업자가 *왜 이 길을 택했는지* 알 수 있게.

---

## 1. 출발점 — 현재 우리가 가진 것

- **시뮬**: 6-DOF 강체 (질량 2.5 kg, 관성 텐서, 김벌 ±12°/200°/s, 추력 40 N, 스풀업 τ=0.08s, 항력)
- **컨트롤러 4가지**: 
  - `LandingPID` (캐스케이드 + 적분 + 게이트). 가장 견고.
  - `CvxpyPointMassMPC` (점질량 convex). MPC 핵심.
  - `LandingCvxpyMPC`, `GuidancePID`, `WaypointPID` 등 PID 변형
  - `LandingResidualMPC` 계획 (보류)
- **상태 추정**: 적응형 EKF (`LowPassStateEstimator.for_obs_noise(...)`)
- **평가**: `evaluate_navigation.py`로 *동일 시드*에서 PID/MPC 변형들 비교

## 2. 발견된 문제

| n=50, estimated 상태 | hard | noisy |
|---|---|---|
| **PID 단독** | **58%** | **84%** |
| Cvxpy Raw | 0% | 0% |
| Cvxpy Guidance | 42% | 0% |
| Cvxpy Waypoint | 52% | 76% |

**모든 MPC 변형이 PID 단독에게 짐.** 직관에 어긋남(MPC가 *위층*인데 왜?).

## 3. 진단 — 왜 MPC가 못 이기나

`docs/mpc_model_mismatch.md`의 정량 진단:
- MPC 점질량 계획 vs 실제 6-DOF 롤아웃: **위치 오차 4~6m**
- **김벌 87~94% 시간 한계에 박힘** (saturated)
- Oracle 모델(정확한 질량) 줘도 안 풀림 → *구조* 문제, 튜닝 문제 아님

원인: **MPC의 내부 모델이 점질량**. *"추력 벡터를 즉시 어디로든 만들 수 있다"* 가정. 실제론:
1. 노즐 틀어야 하고 (서보 시간)
2. 몸체가 회전해야 하고 (관성)
3. 추력 방향이 따라 바뀜
4. 가속도가 생김

총 ~0.2초 *순차* 지연. MPC는 0초로 가정 → *불가능한 횡가속 요청* → 김벌 saturated.

## 4. 첫 혼동 — "공존(공존)" vs "교체(교체)"

사용자: *"MPC와 PID 공존한다고 했는데 왜 교체라는 말이 나와?"*

대답:
- **Layer 1 (계획)·Layer 3 (자세) 에선 공존** (MPC가 계획, PID가 자세 안정)
- **Layer 2 (위치 → 추력벡터) 에선 *둘 중 하나만***
- 우리 변형들이 Layer 2를 *MPC로 교체* 함 → PID의 외란 흡수(적분·게이트) 사라짐 → 성능 ↓

→ 같은 *시스템*에서 *층마다* 공존/교체가 섞여 있었음. 표현이 부정확했음을 인정.

## 5. 두 가지 분기 — Residual vs Hierarchical

### Residual MPC
```
u_final = u_PID(전체) + clip(u_MPC, ±작은 한계)
```
- 장점: PID 하한 *구조적* 보장 (MPC=0이면 = PID)
- 단점: MPC 모델 문제는 *덮지만* 안 *고침*. 보조 패턴.
- codex 검증: "PARTIALLY TRUE — 하한 보장은 *순간*에만 참. 작동 중엔 saturation/integrator windup 가능. anti-windup 같은 추가 장치 필요"

### Hierarchical (SpaceX식)
```
MPC: 미래 궤적 publish      ← 위층
PID/LQR: 그 궤적 추종 + 짧은 외란 즉시 반응  ← 아래층
```
- 장점: 진짜 정통 GNC 패턴. SpaceX가 쓰는 길.
- 단점: MPC가 *틀리면* 보호장치 없음. **MPC가 정확해야 작동.**
- codex 검증: "convex 가이던스 부분 확정. 단 SpaceX 내부 컨트롤러 세부는 *공개 안 됨* (PID/LQR 추정)"

### 사용자 선택
**"Residual 말고 Hierarchical로. MPC 자체를 고치자."**

이유:
- Residual은 *증상 완화*, Hierarchical+MPC 수정은 *원인 해결*
- 진짜 SpaceX식으로 가고 싶음
- MPC 점질량이 *근본 잘못*이라 *그것부터 고치는 게 맞음*

→ **이 결정으로 작업 방향 확정: MPC를 *실현 가능한 계획*을 짜도록 업그레이드.**

## 6. 세 단계 업그레이드 — 비유로 설명

**MPC = 운전자/장군. 점질량 = "체스 말처럼 즉시 이동 가능"하다고 *거짓 가정* 한 상태.**
한 단계씩 *현실*을 알려준다:

### Step 1 — "추력 방향은 *천천히만* 바뀐다"
```
MPC 추가 룰:  ||u[k+1] - u[k]||₂ ≤ J_eff · dt   (하드 제약)
```
- 추력 가속도 벡터를 *상태*로 승격
- 한 스텝에 변할 수 있는 최대 크기 제한
- **여전히 convex** → cvxpy 그대로 사용
- **무엇을 모델링하나**: "몸체 자세가 갑자기 못 바뀌니 추력 방향도 갑자기 못 바뀜"을 *간접 근사*
- 작업량: ~1.5시간

### Step 2 — "추력 *크기* 도 즉시 안 바뀐다"
```
MPC 추가 상태: T (추력 크기)
            동역학: dT/dt = (T_cmd − T) / 0.08s   (EDF 스풀업)
```
- EDF 1차 지연 반영
- 여전히 convex
- 작업량: ~30분

### Step 3 — "*몸 자체가* 회전하는 진짜 물리"
```
MPC 추가 상태: 자세(quaternion), 각속도(ω)
            동역학:  자세' = ω
                    ω' = (김벌 토크 − ω×Iω) / I
```
- **자세를 *진짜 상태*로 모델링**. 정통 SpaceX 6-DOF guidance.
- **비선형** → cvxpy 못 풀음 → **SCP (Successive Convex Programming)** 필요
- 참고 문헌: Szmuk, Reynolds, Açıkmeşe 라인. [arXiv 1811.10803](https://arxiv.org/abs/1811.10803), [arXiv 1901.02181](https://arxiv.org/abs/1901.02181)
- 작업량: ~1~2일

## 7. "야매(Step 1+2)" vs "정통(Step 3)" — 무슨 의미인가

codex가 "Step 1+2는 *standard-adjacent*"이라고 한 의미:

| | 야매 (Step 1+2) | 정통 (Step 3) |
|---|---|---|
| **자세 동역학** | 무시. 추력 벡터 변화 속도로 *간접 근사* | 자세를 *상태*로 정확히 모델링 |
| **slew 값** | 추정/캘리브레이션 (J_eff) | 자세 운동방정식에서 *유도* |
| **솔버** | cvxpy (빠름) | SCP (반복적, 느림) |
| **정확도** | ~70~80% (대체로 OK) | ~100% (진짜 물리) |
| **작업량** | 2시간 | 1~2일 |

비유:
```
야매:  "보통 차로 30분 걸려" (대충 룰)
정통:  실제 도로·교통·신호등 계산해 *정확한* 도착 시간
```

**왜 야매부터?**: *충분히* 잘 되면 Step 3 안 해도 됨. 데이터로 결정 가능.

## 8. 검증 전략 — 순차 + 4 메트릭

codex 조언: *"하나하나 검증해야 어디서 문제 생겼는지 안다."*

### 흐름
```
[현재] 점질량 MPC                              ← 기준선
  │   메트릭: 성공 52%, 김벌 포화 87%
  ↓ Step 1 추가 (Step 2/3 *안 함*)
[Step 1 only]                                  ← 새 클래스 CvxpyActuatorAwareMPC
  │   검증 ① n=50 hard·noisy → 4 메트릭
  │   비교: 점질량 vs Step 1
  │   기대: 김벌 포화 ↓, 성공률 유지/↑
  ↓ Step 2 추가
[Step 1+2]
  │   검증 ② n=50 → 4 메트릭
  │   비교: Step 1 vs Step 1+2
  ↓ *데이터로* 결정
  │
  ├─ 충분 (포화 ≤ 30%, PID 능가) → 끝. hop_test로 이동
  └─ 부족 (포화 50%+, PID 못 이김) → Step 3
                                       ↓ 새 클래스 CvxpyScp6DofMPC
                                    [Step 3]
                                       검증 ③ n=50 → 4 메트릭
```

### 4 메트릭 (단순 성공률만 보면 안 됨)

1. **성공률** — 최종 결과
2. **김벌 포화 %** — MPC 계획이 *실현 가능*했는가
3. **계획 vs 실제 위치 오차** (m) — MPC가 얼마나 거짓말?
4. **접지 vspeed/offset/tilt 평균** — *어디서* 망가지나

### 이전 코드 보존 원칙

```
src/rocketsim/controllers/mpc.py:
  CvxpyPointMassMPC        ← 현재. 절대 *수정/삭제 안 함*
  CvxpyActuatorAwareMPC    ← Step 1+2. 새 클래스로 추가.
  CvxpyScp6DofMPC          ← Step 3. 새 클래스 (필요시).
```

→ 언제든 *이전 버전으로 롤백* 가능. A/B 비교도 가능.

## 9. 함정 — codex가 미리 알려준 것

Step 1+2 구현 시 *반드시 반영* 해야 할 4가지:

1. **slew 값(J_eff)을 *김벌 스펙*만 보지 말 것**. `J_eff = Jmax × dt`로, 자세 루프 대역폭 + 현재 추력 레벨 + 안전 마진 모두 반영해 계산.
2. **Slack variable 필수**. 강한 외란 회복 중 infeasibility 방지. 비용에 큰 가중치 주어 평소엔 0이지만 위급 시 *살짝* 위반 가능.
3. **World-frame slew는 *상위 근사***. 실제 자세 동역학을 통한 실현 가능성은 *별개* 문제. 김벌 토크/관성 한계가 따로 있음.
4. **Slew 권한은 *추력 크기 의존***. 추력 작을 때 횡 권한도 작음. 가능하면 J_eff를 추력 크기에 *비례하게*.

## 10. 다음 작업

```
Day 1 오전 (1.5h):  CvxpyActuatorAwareMPC 코드 (Step 1, 함정 4개 반영)
Day 1 오후 (1h):    검증 ① — n=50 hard·noisy + 4 메트릭
                     결과를 docs/<날짜>_<시간>_v1_step1_results.md 에 기록
Day 2 오전 (30m):   Step 2 추가 (추력 magnitude lag)
Day 2 오전 (1h):    검증 ② → 기록
                     ↓
                  결정: Step 3 가나? *데이터로 판단.*

(필요 시) Day 2~3: Step 3 — SCP 6-DOF MPC
                   문헌: arXiv 1811.10803 30분 훑기 → 골격 구현 → 검증
```

## 11. 사고 과정 핵심 요약

1. MPC가 PID 못 이기는 *근본 원인* = MPC 점질량 모델 (자세 동역학 무시)
2. 해결 *방향*에 두 가지 길: Residual(증상 완화) vs Hierarchical(원인 해결)
3. 사용자 선택: **Hierarchical** — MPC 자체를 고친다
4. MPC 업그레이드는 *세 단계*: 추력 벡터 슬루(1) → 추력 크기 지연(2) → 자세 상태(3)
5. Step 1+2 = *야매(빠르고 대체로 OK)*, Step 3 = *정통(진짜 6-DOF)*
6. **순차 검증** — 한 단계씩 추가하며 4 메트릭 측정. 이전 코드 보존.
7. 함정 4개 (codex 지적) 반드시 반영

---

## 부록: 파일명 규칙

이 문서부터 적용:
```
docs/YYYY-MM-DD_HHMM_v<버전>_<주제>.md
```
예: `docs/2026-05-29_0213_v1_hierarchical_mpc_plan.md`

- 정렬 가능
- 같은 주제의 다음 버전은 `_v2` 등으로 증가
- 검증 결과·후속 사고는 새 파일에 추가 (이 파일 *덮어쓰지 않음*)
