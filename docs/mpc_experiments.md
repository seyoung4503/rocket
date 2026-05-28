# MPC 실험 기록

## 2026-05-28 (KST) — 빠른 MPC 실험 결과

목표:

- PID 착륙 기준선 위에 MPC/볼록 최적화 계열을 붙였을 때 성능이 좋아지는지 확인한다.
- 처음부터 full 6-DOF 최적화를 하지 않고, 단계적으로 다음 구조를 비교한다.

## 구현한 컨트롤러

파일:

- `src/rocketsim/controllers/mpc.py`
- `scripts/evaluate_mpc.py`

컨트롤러:

| 이름 | 설명 |
|---|---|
| `LandingVerticalMPC` | 1D 수직 throttle-only sampled MPC. 수평/자세는 기존 PID. |
| `LandingCvxpyMPC` | 3D point-mass convex MPC가 즉시 추력 가속도 벡터를 생성. |
| `LandingCvxpyGuidancePID` | convex MPC가 궤적/waypoint를 만들고 tracking feedback + TVC가 추종. |
| `LandingCvxpyWaypointPID` | convex MPC는 XY waypoint만 제안하고, 기존 HoverPID + landing gate가 추종. |

`cvxpy`는 optional dependency로 추가했다.

```toml
mpc = ["cvxpy>=1.6"]
```

## 실험 명령

예시:

```bash
.venv/bin/python scripts/evaluate_mpc.py --difficulty hard --episodes 20 --controllers pid,guidance,waypoint
```

검증:

```bash
.venv/bin/python -m py_compile src/rocketsim/controllers/mpc.py scripts/evaluate_mpc.py
.venv/bin/python tests/test_dynamics.py
```

결과:

- `py_compile` 통과
- dynamics 테스트 통과

## 주요 결과

### 1D Vertical MPC

초기 실험 결과:

| 난이도 | PID | PID + VerticalMPC | 관찰 |
|---|---:|---:|---|
| calm, n=20 | 20/20 | 17/20 | 에너지는 줄었지만 수평 정렬 전 접지 |
| moderate, n=20 | 20/20 | 6/20 | timeout/offset 실패 증가 |
| hard, n=20 | 11/20 | 0/20 | 수평/자세와 분리한 수직 MPC는 부적합 |

결론:

- 수직 throttle planning만 따로 떼면 안 된다.
- 착륙 문제는 수직속도보다 **수평 정렬 + 접지 타이밍**이 핵심이다.

### Raw 3D Convex MPC

`LandingCvxpyMPC`: 3D point-mass convex MPC가 직접 원하는 추력 가속도 벡터를 만든다.

| 난이도 | PID | Raw Cvxpy3DMPC |
|---|---:|---:|
| calm, n=10 | 10/10 | 10/10 |
| moderate, n=20 | 20/20 | 12/20 |
| hard, n=20 | 11/20 | 3/20 |

관찰:

- calm에서는 잘 된다.
- moderate/hard에서 실제 6-DOF 동역학과 MPC 내부 point-mass 모델의 mismatch가 커진다.
- MPC는 즉시 원하는 가속도 벡터를 만들 수 있다고 가정하지만, 실제 기체는 자세/TVC/추력 지연을 거쳐야 한다.

### MPC Guidance + Tracking

`LandingCvxpyGuidancePID`: MPC가 궤적을 만들고 tracking feedback + TVC 루프가 추종한다.

| 난이도 | PID | CvxpyGuidancePID |
|---|---:|---:|
| calm, n=10 | 10/10 | 10/10 |
| moderate, n=20 | 20/20 | 16/20 |
| hard, n=20 | 11/20 | 7/20 |
| noisy, n=20 | 2/20 | 1/20 |

관찰:

- raw MPC보다 실제 GNC 구조에 가까워지면서 성능이 개선됐다.
- 하지만 hard에서는 여전히 PID보다 낮다.
- noisy에서는 상태추정기 없이 noisy measurement를 쓰면 MPC가 더 흔들린다.

### MPC Waypoint + 기존 PID

`LandingCvxpyWaypointPID`: MPC는 waypoint만 제안하고, 기존 PID/landing gate가 실제 추종과 접지 판단을 담당한다.

| 난이도 | PID | CvxpyWaypointPID |
|---|---:|---:|
| calm, n=20 | 20/20 | 20/20 |
| moderate, n=20 | 20/20 | 20/20 |
| hard, n=20 | 11/20 | 11/20 |
| noisy, n=20 | 2/20 | 1/20 |

관찰:

- PID 안정성을 깨지 않는 가장 보수적인 구조다.
- hard에서 PID와 같은 성공률까지 회복했다.
- 다만 PID를 이기지는 못했고, throttle integral은 더 컸다.

hard 세부:

| 컨트롤러 | soft landing | reached ground | throttle integral mean | touchdown mean |
|---|---:|---:|---:|---|
| PID | 11/20 | 20/20 | 12.66 | offset 0.32 m, vspeed 0.09 m/s, hspeed 0.33 m/s, tilt 5.6 deg |
| CvxpyWaypointPID | 11/20 | 20/20 | 14.28 | offset 0.38 m, vspeed 0.10 m/s, hspeed 0.33 m/s, tilt 6.0 deg |

## 외란 Bias 추정 실험

상수 가속도 bias를 추정해 MPC 내부 모델에 넣는 실험도 했다.

결과:

- raw velocity difference 기반 bias 추정은 오히려 성능을 악화시켰다.
- 센서 노이즈와 추력/자세 지연이 섞여 있어, 단순 차분으로는 외란만 분리되지 않는다.
- 기본값은 `use_bias_estimator=False`로 꺼두었다.

결론:

- 외란 추정은 필요하지만, EKF/필터 없이 넣으면 안 된다.

## 현재 결론

MPC 방향 자체는 맞다.

하지만 현재 구현한 3D point-mass convex MPC는 아직 PID보다 약하다. 이유는 MPC가 틀린 세계 모델을 보고 판단하기 때문이다.

현재 PID가 가진 강점:

- 실제 6-DOF 상태를 매 순간 피드백으로 보정
- landing gate로 수평/자세 정렬 전 접지 방지
- 적분항으로 바람/추력 오정렬/모델오차 흡수
- actuator delay와 TVC 한계를 실제 루프에서 계속 보정

현재 MPC에 부족한 것:

- Navigation/EKF 또는 필터링된 상태추정
- actuator-aware prediction
  - thrust lag
  - gimbal rate limit
  - attitude tracking lag
- robust terminal state machine
- 신뢰할 수 있는 disturbance estimation
- 6-DOF와 point-mass 모델 사이의 mismatch 보정

## 다음 작업

우선순위:

1. 센서/Navigation 계층 추가
2. EKF 또는 complementary filter로 position/velocity/attitude 추정
3. MPC 입력을 참값/노이즈값이 아니라 추정 상태로 통일
4. actuator-aware MPC 또는 conservative reachable set 추가
5. terminal mode를 controller와 분리된 guidance state machine으로 정리
6. 그 다음 MPC vs PID vs RL residual을 다시 비교

한 줄 요약:

> 현재 MPC는 방향은 맞지만, 아직 실제 로켓 GNC에 필요한 상태추정/작동기 모델/terminal mode가 부족해서 PID보다 못하다.
