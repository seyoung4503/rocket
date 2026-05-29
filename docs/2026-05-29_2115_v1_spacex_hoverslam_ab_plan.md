# Plan: EDF에서 SpaceX hoverslam — 순정 vs actuator-aware A/B

**날짜:** 2026-05-29 21:15 (KST)
**버전:** v1
**상태:** 계획 (구현 미착수 — 승인 후 진행)
**Predecessor:** [`2026-05-29_1845_v1_spacex_negative_result.md`](./2026-05-29_1845_v1_spacex_negative_result.md), [`2026-05-29_2045_v1_spacex_actuator_analysis.md`](./2026-05-29_2045_v1_spacex_actuator_analysis.md), [`2026-05-29_2110_v1_step3_analysis.md`](./2026-05-29_2110_v1_step3_analysis.md)

## 동기

SpaceX G-FOLD stack은 EDF 단거리 hop regime에 구조적으로 안 맞고 본인 데이터로 이미 반증됨(6~32% vs baseline 58~90%). 그럼에도 **"드론이 아니라 로켓"** 이라는 정체성과 낭만으로, EDF에서 *실제로 착륙하는* hoverslam을 만들어본다. 목표는 "정통 알고리즘 포팅"이 아니라 **"정통 알고리즘으로 진짜 내려앉히기"**.

이 작업의 know-how(actuator-aware 플래닝)는 나중 진짜 로켓엔진 단계로 전이됨.

## 0. 철학 (락된 것)

- **드론 아님 = 호버 금지.** 두 변형 모두 commit하는 로켓 무브(coast→burn). 지속 호버 진입 절대 없음.
- **EDF 호버 능력 = 안전망**, 착륙 전략 아님. 일부러 hoverslam을 강제해 로켓 GNC를 연습.
- **G-FOLD 영혼 유지**: min-fuel + shrinking horizon + glideslope + time-indexed tracking. 이건 안 건드림.
- 비교가 답할 질문: **"MPC에 액추에이터 lag(τ=0.08s)+슬루를 알려주면, 진짜 hoverslam이 laggy EDF에서 성립하는가?"**

## 1. 두 변형 정의

| | **A — 순정 hoverslam (reference)** | **B — actuator-aware hoverslam** |
|---|---|---|
| 플래너 | `ConvexLandingMPC` (naive) | `ActuatorMagLagLandingMPC` (슬루+스풀 lag) |
| 강하 | 공격적 `v_max_desc≈4~5` (진짜 coast→kill) | 동일하게 공격적 (호버슬램 유지) |
| 터미널 | **보정 없음** — 정직한 reference | **commit flare** (v→0 at z→0, 호버 아님) |
| 기대 | EDF lag로 고vspeed 충돌 — 문제를 드러냄 | burn을 일찍 시작·성형해 실제 착지 |

> 핵심: 현재 base spacex는 `v_max_desc`를 4→2로 낮춰 "반쪽 hoverslam"이 돼 있음(`landing_controller.py:62`). A는 이걸 **다시 공격적으로 올려 순정 hoverslam의 정직한 reference**를 만들고, B는 lag을 모델링해 공격적 강하를 *성립*시킴.

## 2. 파일별 변경 (구현 시)

### `src/rocketsim/spacex/convex_landing_mpc.py`
- 변경 거의 없음 — `ActuatorMagLagLandingMPC`(슬루+스풀 lag) 이미 존재(L420~447), B에 그대로 재사용.
- (선택) `a_min_frac=0` + 공격적 `v_max_desc` 조합이 순정 hoverslam임을 주석으로 명시.

### `src/rocketsim/spacex/trajectory_tracker.py` — *핵심 작업*
- **터미널 commit flare** 추가 (B 전용, 플래그 `commit_gate`):
  - `z < z_gate`(예 0.6m) 또는 `tau ≥ plan 끝`이면 plan 추종 대신 직접 감속 법칙:
    ```
    v_target(z) = -min(v_cap, sqrt(2 · a_decel · z))   # z→0에서 v→0
    a_des_z = g + kd_commit · (v_target − vz)
    ```
  - **호버 아님**: 계속 하강하되 속도를 z에 묶어 부드럽게 0으로 commit. 로켓 flare.
  - 근거: 본인 데이터상 지배적 실패가 *플랜*이 아니라 *터미널 commit*(vspeed 3.44~5.30, 한계 1.0). 최고 레버리지.

### `src/rocketsim/spacex/landing_controller.py`
- `commit_gate`, `z_gate`, `a_decel` 매개변수 추가 → tracker로 전달. 기본 off(=A는 영향 없음).

### `scripts/evaluate_navigation.py`
- 컨트롤러 2개 등록:
  - `spacex_hoverslam` → A (naive, 공격적 강하, commit off)
  - `spacex_hoverslam_aware` → B (`ActuatorMagLagLandingMPC` + commit on)
- `CONTROLLERS` 리스트(L73~77 근처) + `make_controller` 분기(L135~152 패턴) 추가.

## 3. 실험 매트릭스

```
python scripts/evaluate_navigation.py \
  --difficulties hard,noisy,divert,divert_hard \
  --controllers actuator,spacex,spacex_hoverslam,spacex_hoverslam_aware \
  --episodes 50 --modes true
```

- baseline: `actuator(la=10)`(58~90%), `spacex`(base, 14~32%)와 비교.
- 핵심 지표: **touchdown vspeed**(한계 1.0), success%, reason 분해.

## 4. 실험 성공 기준 (낭만의 정량화)

- **A**: vspeed가 크게 나와도 OK — "순정 hoverslam은 lag 때문에 박는다"를 정량 입증하면 성공.
- **B**: A 대비 vspeed 유의미 감소(목표 <1.5, 이상적 <1.0) + success가 base spacex(14~32%) 대비 상승. baseline(actuator) 근접이 stretch goal.
- 산출물: A vs B vs base의 vspeed/success 표 → "actuator-aware가 hoverslam을 성립시키는가"에 명확한 답.

## 5. 리스크 / 오픈 질문

- commit flare의 `z_gate`/`a_decel` 튜닝 민감할 수 있음 → 1~2회 스윕 필요.
- B의 actuator-aware 플랜이 과거(20:45 분석) 단독으론 역효과였음 → **이번엔 commit flare와 *세트*로 가는 게 가설**. 둘이 시너지인지 확인이 실험의 묘미.
- glideslope vs divert 충돌(10m divert에 z≥10·tan30°=17.3m 요구)은 별도 — 필요시 γ 적응형은 *후속*으로 분리(이번 A/B 범위 밖).

## 6. devlog/docs

- 실험 후 `docs/YYYY-MM-DD_HHMM_v1_spacex_hoverslam_ab.md`(결과+분석) + devlog 항목. 컨벤션 준수.

## 기록 흐름

```
28. 2026-05-29_2115_v1_spacex_hoverslam_ab_plan.md  — 이 문서 (계획)
```
