# Plan: 로켓엔진 regime — "호버 불가"에서 hoverslam이 유일 해법이 되는가

**날짜:** 2026-05-30 17:53 (KST)
**버전:** v1
**상태:** 계획 (구현 미착수 — 승인 후 별도 워크트리에서 진행)
**Predecessor:** [`2026-05-30_0117_v1_spacex_hoverslam_ab_results.md`](./2026-05-30_0117_v1_spacex_hoverslam_ab_results.md) (EDF에선 hoverslam이 baseline에 짐), [`2026-05-29_1845_v1_spacex_negative_result.md`](./2026-05-29_1845_v1_spacex_negative_result.md)

## 동기

hoverslam A/B 실험의 결론: **EDF에선 hoverslam이 안 맞는다** — EDF가 호버 가능(TWR 1.6, 스로틀 0까지)이라 천천히 내려오는 `actuator(la=10)` baseline(70~97%)이 이기고, 공격적 hoverslam은 actuator lag로 소프트 착륙 4조건을 동시에 못 채움(병목이 vspeed→tilt→hspeed로 행진).

그런데 SpaceX G-FOLD/hoverslam은 *그들 regime*에 맞춰 진화한 것이고, 그 regime의 정의적 특징은 **호버 불가**다. 이번 트랙은 그 regime을 시뮬에 만들어 **"적용 가능성이 regime에 따라 뒤집힌다"는 thesis를 정량 검증**한다. 이번 hoverslam stack을 *살리는* 방향.

> 주의: 이건 **로켓엔진 역추진 *착륙* GNC** regime이다(프로젝트의 "역추진 착륙" 축). devlog 2026-05-29 17:04에서 분리한 *Mach 1 비행*(별도 공력·구조 트랙)과는 다름.

## 핵심 가설

> **min-throttle 추력 > 무게 → 엔진 켜면 무조건 상승 → 호버 물리적 불가 → hoverslam이 유일 해법.**

이 regime에서:
- **PID / actuator baseline은 *실패*한다** — 이들은 "추력을 무게에 맞춰 호버하며 천천히 내려온다"를 전제. min-throttle T/W>1이면 그 전제가 깨짐(throttle 내려도 계속 가속 상승 → 호버 setpoint 추종 불가).
- **G-FOLD hoverslam stack은 *작동*한다** — coast(엔진 off)→단일 burn으로 고도0=속도0 타이밍 맞추기. min-thrust 하한이 있는 lossless convexification이 바로 이 문제를 푸는 도구.

**예상: EDF에서의 순위가 뒤집힌다** (actuator ≫ spacex → spacex ≫ actuator 또는 actuator 실패).

## 확정된 regime 파라미터 (tight Falcon식, 2026-05-30)

m=2.5/60N 대신 **가볍고 빡빡한 Falcon식**으로 확정:

| 항목 | 값 | 근거 |
|---|---|---|
| mass | **1.5 kg** | 무게 W = 14.7 N |
| max_thrust | **21 N** | 최대 TWR = **1.43** (Falcon 착륙 1.2~1.4급) |
| min_throttle | **0.72** | 최소 추력 15.1N, 무게보다 **겨우 +0.4N** → 호버 불가, 아슬 |
| 짐벌·τ | **EDF 값 유지** (±12°, 200°/s, τ=0.08s) | "정직하게 어렵게" — lag 교체 안 함 |

거동:
- 엔진 끔(coast): 자유낙하 −9.8 m/s²
- 최소 점화: +0.27 m/s² (간신히 상승) → **호버 불가 확정**
- 최대 점화: **+4.2 m/s² 감속 (≈0.43g)** → 타이밍 창이 좁은 진짜 자살강하

**⚠️ 의도된 난이도**: 감속 권한 0.43g + EDF lag(τ=0.08s) → 타이밍이 극도로 빡셈. 성공률이 낮게 나와도 *그게 결과* — "이 regime이 얼마나 빡센지 + baseline(호버 의존)이 여기서 깨지는지"를 정직하게 측정. (lag 교체 = 공정화 옵션은 후속.)

## 최소 구현 (대부분 이미 존재)

1. **로켓엔진 vehicle 프리셋** (`vehicle.py`)
   - `min_throttle: float = 0.0` 필드 추가 (EDF=0 유지 → 기존 동작 불변).
   - `rocket_lander()` 팩토리:
     ```python
     Vehicle(mass=1.5, max_thrust=21.0, min_throttle=0.72)
     # 최소 추력 0.72*21 = 15.1N > 무게 14.7N → 호버 불가
     ```
2. **dynamics throttle 하한** (`dynamics.py`)
   - 엔진 점화 상태에서 throttle을 `[min_throttle, 1]`로 clamp(또는 0). 현재는 `[0,1]` clamp. 정확한 모델은 `{0} ∪ [min_throttle, 1]`(비연속)이나, 착륙 burn 구간에선 항상 점화라 `[min_throttle, 1]`로 충분.
3. **G-FOLD MPC는 이미 준비됨** (`spacex/convex_landing_mpc.py`)
   - `a_min_frac` 파라미터가 이미 있음(현재 EDF=0). 로켓엔진은 `a_min_frac = 0.72`(= min_throttle)로 주면: `a_max = 21/1.5 = 14.0`, `a_min = 0.72 × 14.0 = 10.1 m/s² > g(9.8)` → G-FOLD 하한 추력 제약(`s[k] ≥ a_min > g`) 활성 → 호버 가속(=g) 계획 불가 → **coast-burn 구조 강제**. **컨트롤러 코드 변경 거의 0.**
4. **시나리오/eval**
   - 기존 시나리오 재사용 가능(필요시 시작 고도↑로 burn 여유). eval `make_controller`가 `plant_model`에 따라 vehicle을 고르므로, 로켓엔진 vehicle을 주입하는 경로 추가.
   - 대조군: 같은 regime에서 `pid`, `actuator`, `spacex`, `spacex_hoverslam_*` 비교.
5. **(후속, 선택) 추진제 질량 감소** — thrust 적분으로 mass 감소 → min-fuel 목적이 진짜 의미. 1차 컷에선 생략(고정 질량).

## 실험 매트릭스

```
# 호버 불가 로켓엔진 regime에서 순위 역전 확인
controllers: pid, actuator, spacex, spacex_hoverslam, spacex_hoverslam_commit
difficulties: (로켓엔진 IC — 시작 고도/속도 재설정 필요할 수 있음)
modes: true (+추후 estimated)
--workers 16   # ★ 기본 1=직렬이라 반드시 명시 (지난 실험 교훈)
```

핵심 지표: success%, **baseline이 실제로 실패하는가**(reasons에 호버-불가 발산/too_high/timeout), spacex가 유일하게 착륙하는가.

## 성공 기준 (가설 검증)

- **순위 역전 확인** = thesis 입증: EDF에서 졌던 spacex/hoverslam이 이 regime에선 baseline을 *이기거나*, baseline이 *구조적으로 실패*.
- 역전이 안 나오면 그것도 의미 있는 결과(왜 안 뒤집히는지 → min-throttle 모델/스케일 점검).

## 최소 → 리얼리스틱 조건

| 조건 | 1차 컷 | 후속 |
|---|---|---|
| min-throttle 호버 불가 | ✅ 핵심 | — |
| 추진제 질량 감소 | ❌ 고정질량 | min-fuel 의미화 |
| `estimated` 네비 | ❌ true | 노이즈 robust |
| 스케일(고고도·고속) | ❌ 기존 IC | km·수십초로 확장 시 long-horizon 이점 |
| EDF roll 물리 | 별도 트랙 | [[edf-roll-physics-separate-world]] |
| HW(실 엔진 추력곡선·min-throttle) | ❌ | 캘리브레이션 |

## 리스크 / 오픈 질문

- **throttle 하한 모델의 비연속성**(`{0}∪[min,1]`) — 착륙 burn은 항상 점화라 `[min,1]`로 근사. coast 구간(엔진 off)을 어떻게 다룰지: G-FOLD는 `a_min`을 하한으로 두므로 "항상 점화" 가정. coast 허용하려면 정수계획(비convex) — 1차 컷은 "항상 점화" 단순화.
- **시작 IC** — 호버 불가면 burn 타이밍이 빡세서, 시작 고도/속도가 burn 여유와 직결. 튜닝 필요.
- baseline(PID/actuator)이 이 regime에서 *우아하게 실패*하는지(발산 vs 그냥 hard landing) 확인.

## 다음 (승인 후)

1. 새 워크트리 `rocket-engine-regime` 분기 (hoverslam과 분리, HEAD에서).
2. 최소 구현(vehicle 프리셋 + dynamics 하한 + a_min_frac 배선 + eval 주입).
3. `--workers 16` 병렬 비교 실험 → 순위 역전 확인.
4. 결과 문서 + devlog.

## 기록 흐름
```
30. 2026-05-30_1753_v1_rocket_engine_regime_plan.md  — 이 문서 (계획)
```
