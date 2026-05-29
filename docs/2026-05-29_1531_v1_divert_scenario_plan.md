# Divert Scenario — Design & Hypothesis

**날짜**: 2026-05-29 15:31 KST  
**버전**: v1  
**Predecessor**: [`2026-05-29_1530_v1_step2_maglag_analysis.md`](./2026-05-29_1530_v1_step2_maglag_analysis.md)

## 동기 — 왜 새 시나리오가 필요한가

지금까지 결과 정리:

| controller | hard | noisy | 비고 |
|---|---|---|---|
| pid | 58% | 84% | **베이스라인** |
| waypoint | 52% | 76% | |
| actuator (S1) | 46% | 74% | |
| actuator2 (S2) | 54% | 80% | |

핵심 발견 (`2026-05-29_1530_v1_step2_maglag_analysis.md` deeper diagnostic):
1. Step 1 vs Step 2 의 명령 시계열·자세 메트릭이 **사실상 동일** (∆ throttle RMS ≈ 0).
2. 성공률 차이(+4 / +3 seed)는 McNemar 검정 p > 0.2 — **노이즈 안**.
3. 현재 hard/noisy 시나리오는 5~6초 짧은 강하 + 즉발 외란 — **MPC 의 look-ahead 가 별로 안 쓰이는 환경**. PID 의 reactive 강점이 자연스럽게 이기는 setup.

→ "MPC 가 PID 못 이긴다" 는 결론을 내리기 전에, **MPC 의 구조적 강점이 실제로 작동하는** 시나리오에서 한 번 더 검증해야 공정함.

## 사용자 제약

1. **저고도 유지**: 실제 hop 실험은 5~15m 시작. 고고도 (30m+) 는 실험 환경과 다르므로 *기본* 시나리오로 채택하지 않음. (단, 컨트롤러가 robust 하다면 고고도도 통과해야 한다는 *추가* 검증 의미는 있음 — 후순위.)
2. **추가만, 재정의 금지**: calm/moderate/hard/noisy/recovery/unknown 모두 그대로 둠. 새 시나리오는 *추가*.
3. **Step 3 효과 가시화**: 새 시나리오에서 MPC 트랙 (Step 1/2/3) 과 PID 가 *명확히 갈라져야* Step 3 추가의 가치 측정 가능.
4. **Hard 위에서도 robust**: 새 시나리오의 *외란 강한 버전* 까지 견뎌야 진짜 robust.

## 시나리오: `divert` (그리고 `divert_hard`)

### 한 줄 정의

> 비행 중 (t = 2.0s) 에 **패드 목표가 옆으로 +10m 이동** 한다. 컨트롤러는 새로운 패드 위에 부드럽게 착륙해야 한다.

### 왜 MPC favorable 한가 (구조적 이유)

1. **PID 의 약점**: 통합기가 *이전* 목표 기준으로 적분된 누적 오차를 갖고 있다. 패드가 갑자기 이동하면 통합기 windup → 큰 과조정 → 과진동.
2. **MPC 의 강점**: receding-horizon plan 은 *지금* 의 상대 위치에서 패드까지의 *전체 궤적* 을 매 스텝 새로 짠다. 패드 이동 = 다음 plan() 호출에서 즉시 반영 — windup 자체가 없음.
3. **저고도 유지**: 시작 IC 는 기존 *moderate* 와 동일 (alt 8-12m). divert 자체는 수평 이동.
4. **Hard 변형**: `divert_hard` = hard IC + divert 이동 → 외란 + 비행 중 목표 변경 조합. Step 3 이 *진짜* 필요한 시나리오 확인.

### 실제 hop 실험과의 연결

실제 hop 실험에서 비슷한 상황:
- "원래 패드 A 에 착륙하려 했는데 마지막 순간에 패드 B 로 변경" (다중 패드 운용)
- "착륙 중 안전요소 (사람, 장애물) 발견 시 즉시 회피 후 새 지점 착륙"
- "GPS 갱신으로 패드 좌표가 1m 보정됨" — 작은 divert (Δ=1m) 도 같은 메커니즘

→ 단순 픽션 아니라 실제 hop 자동화의 *현실적 use case*.

## 구현 설계

### 데이터 구조

`LandingScenario` 에 두 필드 추가:
```python
pad_shift_time: float | None = None  # 초; None = 시프트 없음
pad_shift_delta_xy: tuple[float, float] = (0.0, 0.0)  # m, world-frame 이동
```

### 동작

`LandingEnv.step()` 의 sim sub-loop 안에서, `self.t` 가 `pad_shift_time` 을 *처음* 넘는 순간:
```python
self.state[dyn.POS][:2] -= np.array(pad_shift_delta_xy)
```
즉 **state 를 "새 패드 기준 상대 좌표"로 변환**. 차량의 실제 world 위치는 안 변함 (gravity·물리 다 안 변함, inertial frame 그대로). 이후 모든 check_done / soft_landing / potential 은 자동으로 새 패드 기준 — **컨트롤러 코드 무수정**.

원리: pad-relative frame 도 inertial (패드가 즉시 점프하지 가속 안 함) — gravity·wind 그대로 적용 가능. shift 는 *좌표 원점만* 옮긴 것이라 dynamics 보존.

### Sanity 한계

- 기존 `bounds_radius = 15.0m`. shift Δ=10m + 초기 offset 3m (hard IC) = 최대 ~13m → 14m 마진 정도. OK.
- `timeout = 20s`. shift 가 t=2s 라 회복 시간 18s — 충분.
- 큰 Δ (15m+) 는 향후 `divert_long` 같은 별도 시나리오로.

### 두 변형

| 시나리오     | 기반 IC          | 외란    | shift Δ      | 의도                          |
|--------------|------------------|---------|---------------|-------------------------------|
| `divert`     | moderate (기본) | moderate | (+10, 0) m   | MPC favorable 의 기본 테스트 |
| `divert_hard`| hard            | hard    | (+10, 0) m   | 외란 + divert 의 robust 테스트 |

shift 방향은 +X 단일 (대칭성으로 그래도 됨, seed 변화로 다양성 확보).

## 가설

### H1 (MPC 의 구조적 우위)

> divert 시나리오에서 MPC 변형 (actuator2 이상) 이 PID 를 *통계적으로 유의하게* 이긴다 (n=50, McNemar p < 0.05).

증거가 될 패턴:
- divert: actuator2 ≥ PID + 8pp 이상, p < 0.05
- divert_hard: 격차 더 크면 좋고, 아니면 적어도 *비등*

### H2 (Step 3 의 효과 명확화)

> Step 3 (SCP 6-DOF) 를 추가하면, divert/divert_hard 에서 *closed-loop 성공률 + 자세 안정성* 양쪽 모두에서 Step 2 대비 의미 있는 개선.

증거가 될 패턴:
- 짐벌 포화율 감소 (open-loop 진단)
- 성공률 5pp+ 추가 개선, 통계적 유의

### H3 (현 시나리오의 결론은 그대로)

> 기존 hard/noisy 에서는 Step 3 도 PID 를 의미 있게 이기지 않을 가능성. 이건 *fine* — "PID 가 그 환경에선 충분하다" 결론을 강화할 뿐.

## 검증 전략

```
[지금]  divert / divert_hard 구현 + smoke test
   ↓
[기준선 측정]  pid, waypoint, actuator(S1), actuator2(S2) on divert + divert_hard
   ↓
[기록]  docs/<time>_v1_divert_baseline_results.md (raw + 해석)
   ↓
[Step 3 구현]  CvxpyScp6DofMPC (별도 클래스, 기존 코드 보존)
   ↓
[Step 3 측정]  4 변형 모두 hard, noisy, divert, divert_hard
   ↓
[비교 종합]  Step 1/2/3 의 시나리오별 효과 표 — H1, H2, H3 평가
```

각 단계마다 *새로운* 분석 문서 + devlog 항목.

## 다음 한 줄

`scenarios/landing.py` 에 `pad_shift_*` + classmethod 추가 → `envs/landing_env.py` 에 sub-loop 안 shift → `make_landing_env` 매핑 → smoke test → n=50 baseline.
