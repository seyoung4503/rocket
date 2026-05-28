# GNC, MPC, PID, RL 역할 정리

## 핵심 결론

MPC, PID, RL은 같은 자리에 놓고 경쟁시키는 알고리즘이 아니다.

추천 구조:

```text
Navigation / 상태추정
  ↓
MPC / Guidance
  ↓
PID 또는 LQR / Tracking Control
  ↓
RL residual 보정
  ↓
Safety Filter
  ↓
Throttle + TVC Gimbal
```

한 줄 요약:

> MPC는 미래 궤적을 계획하고, PID/LQR는 그 계획을 실제 기체가 따라가게 하며, RL은 남는 모델오차/외란을 작은 범위에서 보정한다.

## PID와 MPC의 차이

### PID

PID는 현재 오차를 보고 즉시 보정한다.

```text
error = target - current
command = Kp * error + Ki * integral(error) + Kd * derivative(error)
```

장점:

- 빠르다.
- 단순하다.
- 실제 하드웨어에 강하다.
- 작은 외란에 즉각 반응한다.
- 적분항이 있으면 바람, 추력 오차, 질량 오차를 어느 정도 흡수한다.

단점:

- 미래 제약을 직접 보지 않는다.
- 추력 지연, 남은 거리, 착륙 타이밍 같은 미래 tradeoff를 명시적으로 풀지 않는다.

### MPC

MPC는 현재 상태에서 미래 몇 초를 예측하고, 제약을 만족하는 가장 좋은 계획을 계속 다시 계산한다.

```text
현재 상태를 기준으로
앞으로 N초 동안의 위치/속도/입력을 예측
제약을 만족하면서 비용이 가장 낮은 계획 선택
첫 번째 조각만 실행
다음 순간 다시 계산
```

MPC가 예측하는 것:

- 미래 위치
- 미래 속도
- 미래 고도
- 미래 하강속도
- 미래 수평속도
- 미래 연료/에너지 사용
- 미래 제약 위반 여부

MPC가 고려할 수 있는 제약:

- 추력 한계
- 짐벌 한계
- 기울기 한계
- 하강속도 한계
- 지면 충돌 방지
- 착륙점 오차
- 연료/에너지 사용

## "방향타를 돌리면 그 방향으로 가는 것 아닌가?"

로켓/EDF는 방향타를 돌린다고 즉시 그 방향으로 이동하지 않는다.

실제 순서:

```text
짐벌을 꺾음
  ↓
토크가 생김
  ↓
몸통 각속도가 생김
  ↓
몸통이 기울어짐
  ↓
추력 방향이 바뀜
  ↓
가속도가 생김
  ↓
속도가 바뀜
  ↓
위치가 바뀜
```

즉 지연과 누적 효과가 있다.

MPC는 이 지연과 누적을 모델로 예측하려고 한다.

## Receding Horizon

MPC는 5초 계획을 세워도 5초 전체를 그대로 실행하지 않는다.

```text
t = 0.0
  0~5초 계획 계산
  첫 0.1~0.2초만 실행

t = 0.2
  센서 다시 읽음
  0.2~5.2초 계획 다시 계산
  첫 0.1~0.2초만 실행
```

이 방식을 receding horizon이라고 한다.

핵심:

> 길게 보고, 짧게 실행하고, 계속 다시 계획한다.

## MPC와 PID는 어떻게 충돌하지 않게 하나?

나쁜 구조:

```text
MPC: throttle = 0.70, gimbal_x = 3도
PID: throttle = 0.62, gimbal_x = -1도
```

이렇게 같은 레벨의 명령을 동시에 내면 충돌한다.

좋은 구조:

```text
MPC:
  목표 위치 p_ref
  목표 속도 v_ref
  목표 가속도 a_ref

PID/LQR:
  p_ref, v_ref, a_ref를 실제 기체가 따라가도록 보정
```

tracking control 예:

```text
a_cmd =
    a_ref
  + Kp * (p_ref - p_now)
  + Kd * (v_ref - v_now)
  + Ki * integral_error
```

그 다음:

```text
a_cmd → 목표 추력벡터
목표 추력벡터 → throttle + TVC gimbal
```

즉 MPC와 PID는 블렌딩하는 것이 아니라 계층을 나눈다.

```text
MPC = 목표 생성
PID/LQR = 목표 추종
```

## RL은 어디에 들어가나?

RL을 직접 주 제어기로 쓰면 위험하다.

나쁜 구조:

```text
RL(state) → throttle/gimbal
```

추천 구조:

```text
u_base = PID/LQR tracking output
u_rl = small residual correction
u_final = safety_filter(u_base + u_rl)
```

예:

```text
PID/MPC tracking 결과:
  throttle = 0.62
  gimbal_x = 2.0 deg
  gimbal_y = -1.0 deg

RL residual:
  throttle_delta = +0.03
  gimbal_x_delta = +0.4 deg
  gimbal_y_delta = -0.2 deg

최종:
  throttle = 0.65
  gimbal_x = 2.4 deg
  gimbal_y = -1.2 deg
```

RL residual은 반드시 제한한다.

예:

```text
throttle_delta ∈ [-0.05, +0.05]
gimbal_delta ∈ [-1 deg, +1 deg]
```

그리고 safety filter가 마지막으로 제한한다.

## 왜 MPC 단독이 PID보다 항상 좋지 않은가?

MPC는 미래를 보지만, 모델이 틀리면 틀린 미래를 본다.

현재 프로젝트의 단순 MPC는 다음을 충분히 반영하지 못한다.

- 자세 지연
- 짐벌 rate limit
- 추력 지연
- 바람/gust
- 센서 노이즈
- 추력/CG misalignment
- 실제 6-DOF 동역학
- terminal landing state machine

반대로 현재 PID는 단순해 보여도 다음을 이미 갖고 있다.

- 빠른 feedback
- landing gate
- 적분항
- 실제 6-DOF 상태에 대한 지속 보정

그래서 현재 실험에서는 PID가 강하다.

정확한 결론:

> MPC가 나쁜 것이 아니라, 현재 MPC 내부 모델과 GNC 계층이 아직 불완전하다.

## 미니 SpaceX식 구조

목표 구조:

```text
Sensors
  ↓
Navigation / EKF
  상태 추정: position, velocity, attitude, angular rate
  ↓
MPC / Convex Guidance
  목표 궤적, 목표 속도, 목표 가속도 계산
  ↓
PID/LQR/Geometric Tracking
  목표 추력벡터를 실제 자세/짐벌/스로틀로 추종
  ↓
RL Residual
  바람, 모델오차, 노이즈에 대한 작은 보정
  ↓
Safety Filter
  추력, 짐벌, 기울기, 속도 제한
  ↓
Vehicle
```

이 구조에서 MPC는 두뇌 전체가 아니다.

전체 두뇌는 GNC이고, MPC는 그 안의 guidance/planning 엔진이다.

## 현재 프로젝트에 적용할 다음 순서

1. Navigation/상태추정 추가
2. MPC 입력을 참값/노이즈값이 아니라 추정 상태로 통일
3. actuator-aware prediction 추가
4. terminal landing state machine 분리
5. MPC guidance + PID/LQR tracking 재평가
6. RL residual은 마지막에 작은 보정으로 추가

## 정리

현재 이해해야 할 핵심:

```text
MPC는 "미래를 보고 큰 계획을 세우는 계층"
PID는 "그 계획을 빠르게 따라가는 계층"
RL은 "모델이 틀린 부분을 작게 보정하는 계층"
Safety filter는 "최종 명령을 안전하게 자르는 계층"
```

MPC와 PID를 같은 명령으로 섞지 않는다.

대신:

```text
trajectory = MPC(state_estimate)
u_base = PID_or_LQR_track(trajectory, state_estimate)
u_rl = RL_residual(history, state_estimate, trajectory, u_base)
u_final = safety_filter(u_base + u_rl)
```

## 다음 구현 체크리스트

세부 작업 목록은 `docs/roadmap.md`의 `2026-05-28 다음 구현 작업 — Navigation-first GNC 정리` 섹션을 기준으로 한다.

우선순위:

1. `src/rocketsim/navigation/sensors.py`
2. `src/rocketsim/navigation/estimator.py`
3. controller 입력을 estimated state로 통일
4. `scripts/evaluate_navigation.py`
5. `src/rocketsim/guidance/landing.py`
6. estimator + guidance 위에서 MPC/PID 재평가
7. RL residual 추가
