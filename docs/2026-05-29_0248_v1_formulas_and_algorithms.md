# 프로젝트 전체 수식·알고리즘 통합 정리

- **날짜**: 2026-05-29 02:48 KST
- **버전**: v1
- **목적**: 이 시뮬레이터·컨트롤러·평가에 들어간 *모든* 수식과 알고리즘을 위치(=물리적·논리적 어느 부품인지) + 유도 + 사용처 + 우려·보강과 함께 한 문서로.
- **읽는 법**: 먼저 §1 큰 그림과 §2 ASCII 도해 보고, 관심 부분의 상세 절로 점프.

---

## 1. 큰 그림

```
[로켓 (6-DOF 강체)]  ←─ 외란(바람/돌풍/랜덤힘) ──┐
   │                                              │
   ↓ 센서 측정 (노이즈 섞임)                       │
[Navigation: 상태추정 EKF/Low-pass]                │
   │                                              │
   ↓ 추정 상태                                     │
[Guidance: 어디로 갈지 (MPC) 또는 즉시 반응 (PID)]│
   │                                              │
   ↓ 목표 가속/궤적                                │
[Control: 자세 안정화 (PID 내부 루프)]             │
   │                                              │
   ↓ throttle + gimbal 명령                        │
[Actuator: 서보(±12°,200°/s) + EDF(τ=0.08s)]      │
   │                                              │
   └──→ 다시 로켓 ──────────────────────────────┘
        매 0.002초 (500Hz) 물리 적분 RK4
        매 0.02초  (50Hz)  컨트롤러 결정
```

이 문서는 위 화살표마다 *어떤 수식*이 작용하는지 분해.

---

## 2. 로켓 ASCII 도해 — 어디에 어떤 알고리즘이?

```
                              ╔═════════════╗
                              ║             ║       ▲
                              ║   노즈콘    ║       │ 항력 (drag)
                              ║   (간략)    ║       │ F_drag = -½·ρ·Cd·A_side·|v_air|·v_air
                              ║             ║       │  ρ=1.225, Cd=0.5
                              ╟─────────────╢       │  A_side=0.045 m²
                              ║             ║       │  v_air = v_world - v_wind
                              ║             ║       │
                              ║             ║       │ §5 외란
                  ┌─── [IMU] ─║   본체      ║
                  │           ║             ║
        §6 측정   │           ║  ★ CG ★    ║◄──── 질량 m = 2.5 kg
        s_meas =  │           ║  (원점)     ║      관성텐서 I = diag(0.12, 0.12, 0.004)
        s + N(0,σ)│           ║             ║      §3 동역학
                  │           ║             ║
        §7 추정   │ Low-pass  ║             ║      ──→ 중력 F_g = (0, 0, -m·g)
        s_est =   │ EKF       ║             ║              g = 9.80665 m/s²
        adaptive  │           ║   EDF       ║              §4 중력
        filter    │           ║   (모터)    ║◄──── 추력 dT/dt = (T_cmd - T)/τ
                  │           ║             ║      T_max = 40 N, τ = 0.08 s
                  ▼           ╟─────────────╢      §4 추력 지연
                              ║  ⚙   ⚙      ║◄──── 김벌 서보 ×2
        §8 PID                ║  김벌 (TVC) ║      ±12°, 200°/s
        §9 MPC                ║             ║      §4 김벌→토크
        §10 RL                ╟ ─── ▼ ──── ╢
        §11 Step 1            └──── 노즐 ───┘
                                   │
                                   │ 추력 F_t = T · R(q) · d_body
                                   │ d_body = [sy, -sx·cy, cx·cy]   (gimbal_x=x, gimbal_y=y)
                                   ↓
                              월드 z축 ↓ (지면)
                              착륙 패드 (목표)
                              
                              §12 성공 기준:
                                offset ≤ 0.5 m
                                vspeed ≤ 1.0 m/s
                                hspeed ≤ 0.5 m/s
                                tilt ≤ 8°
                                (4개 동시 만족 = AND)
```

---

## 3. 6-DOF 강체 동역학 (시뮬레이터 핵심)

**위치**: `src/rocketsim/dynamics.py`
**호출**: 매 물리 스텝(0.002s) `rk4_step()`

### 3.1 상태 변수 (총 14차원)
| 기호 | 의미 | 차원 | 단위 |
|---|---|---|---|
| $\mathbf{p}$ | 위치 (월드) | 3 | m |
| $\mathbf{v}$ | 속도 (월드) | 3 | m/s |
| $\mathbf{q}$ | 자세 쿼터니언 (body→world) | 4 | — |
| $\boldsymbol{\omega}$ | 각속도 (몸체) | 3 | rad/s |
| $T$ | 실제 추력 크기 | 1 | N |

### 3.2 운동방정식 (연속 시간)
$$
\begin{aligned}
\dot{\mathbf{p}} &= \mathbf{v} \\
\dot{\mathbf{v}} &= \frac{1}{m}\left( \mathbf{F}_\text{thrust} + \mathbf{F}_g + \mathbf{F}_\text{drag} + \mathbf{F}_\text{ext} \right) \\
\dot{\mathbf{q}} &= \tfrac{1}{2}\, \mathbf{q} \otimes [\,0, \boldsymbol{\omega}\,] \\
I\,\dot{\boldsymbol{\omega}} &= \boldsymbol{\tau} - \boldsymbol{\omega} \times (I\,\boldsymbol{\omega}) \\
\dot{T} &= \frac{T_\text{cmd} - T}{\tau_\text{thr}}
\end{aligned}
$$

- $\mathbf{F}_\text{thrust} = T \cdot R(\mathbf{q}) \cdot \mathbf{d}_\text{body}$ — 추력. $R(\mathbf{q})$는 자세 회전 행렬, $\mathbf{d}_\text{body}$는 노즐 방향 단위 벡터(§4.4).
- $\mathbf{F}_g = (0, 0, -m\,g)$ — 중력.
- $\boldsymbol{\tau} = \mathbf{r}_e \times \mathbf{F}_\text{thrust, body}$ — 김벌이 만드는 토크. $\mathbf{r}_e = (0,0,-L)$ (CG에서 엔진까지), $L = 0.45$ m.

### 3.3 적분기 — RK4 (4차 Runge-Kutta)
$$
\mathbf{x}_{k+1} = \mathbf{x}_k + \frac{h}{6}(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4)
$$
$h = 0.002$s. 자세 쿼터니언은 매 스텝 정규화: $\mathbf{q} \leftarrow \mathbf{q}/\|\mathbf{q}\|$.

**유도**: 표준 수치해석. 6-DOF 강체 → 비선형 ODE → RK4가 정확/안정의 표준 절충.

---

## 4. 힘 — 자유물체도와 통합 뉴턴 방정식

### 4.0 한눈에 — 매 순간 로켓에 작용하는 모든 힘

```
                       ▲ 추력 F_thrust = T·R(q)·d_body         (방향: 노즐이 향한 쪽)
                       │   T ≤ 40 N, 노즐 ±12° 김벌
                       │
                       │
                       │
                  ┌────┴────┐
                  │   로켓   │  ← ★ 모든 힘이 *질량중심* 한 점에 가해진다고 가정 (점질점 모델)
                  │   (CG)  │     (실제론 다른 지점에 작용 → 토크 발생 → §4.4 참조)
                  └────┬────┘
                       │  ← (외란) ←──── 가스트 / 랜덤 힘 F_ext  (가우시안 OU, ±2 N 수준)
                       │
                       │ 바람 →─── 항력 F_drag = -½ρ·Cd·A·|v_air|·v_air
                       │            (v_air = v − v_wind, 대기 상대속도)
                       │
                       ▼ 중력 F_g = (0, 0, -m·g) ≈ -24.5 N (m=2.5 kg, g=9.8)
                       
                       ↓ 지면 (z=0)
                       
              ★ 현재 모델링 *안* 됨: 지면 반작용 N (지면 접촉 시), 다리 강성/감쇠
```

### 4.1 통합 뉴턴 제2법칙 (병진)

매 순간 모든 힘의 *벡터 합*을 질량으로 나눠 가속도를 얻음:

$$
\boxed{
\dot{\mathbf{v}} = \frac{1}{m} \Big( \underbrace{\mathbf{F}_{\text{thrust}}}_{\text{추력}}
                              + \underbrace{\mathbf{F}_g}_{\text{중력}}
                              + \underbrace{\mathbf{F}_{\text{drag}}}_{\text{항력}}
                              + \underbrace{\mathbf{F}_{\text{ext}}}_{\text{외란/돌풍}}
                              + \underbrace{\mathbf{F}_{\text{contact}}}_{\text{(미모델링)}}
                        \Big)
}
$$

각 항이 어떻게 *동시에* 작용하는지 핵심:

```
호버 상태에서:    F_thrust↑ ≈ -F_g↓     → 가속도 ≈ 0      → 떠 있음
하강 중:         F_thrust < |F_g|       → 가속도 ↓         → 떨어짐
바람 옆에서:     F_drag 옆 + F_ext 옆   → 옆으로 가속      → 드리프트
강한 추력:       F_thrust > |F_g|       → 가속도 ↑        → 상승
```

### 4.2 회전에 대한 뉴턴 (Euler 방정식)

추력이 *CG에서 떨어진 점에* 작용 → 토크 → 각가속도:

$$
\boxed{
\dot{\boldsymbol{\omega}} = I^{-1} \Big( \boldsymbol{\tau}_{\text{thrust}}
                                   - \boldsymbol{\omega} \times (I\,\boldsymbol{\omega}) \Big)
}
$$
- $\boldsymbol{\tau}_{\text{thrust}} = \mathbf{r}_e \times \mathbf{F}_{\text{thrust,body}}$ — §4.6
- $\boldsymbol{\omega} \times (I\boldsymbol{\omega})$ — *자이로 결합 항* (회전관성). 정확한 6-DOF엔 필수.

→ **항력·중력·외란은 *CG에 작용*하므로 토크 안 만듦**(점질점 가정). 추력만 토크 생성.

### 4.3 중력 (상세)

#### 수식
$$
\mathbf{F}_g = (0,\ 0,\ -m\,g), \quad g = 9.80665 \text{ m/s}^2
$$
- *항상* 세계 좌표 -z 방향. 자세 변해도 *변하지 않음*.
- 크기 = $24.5\text{ N}$ (m=2.5 kg일 때). 호버 시 추력이 *정확히* 이만큼 위로 받쳐줘야 균형.

#### 무엇을 의미하나
- **무게 = 중력**. 흔히 같이 씀.
- **추력대중량비 (TWR)** = $T_{\text{max}} / (m\,g) = 40 / 24.5 \approx 1.63$.
  - TWR > 1: 위로 가속 가능 → 이륙·복귀 가능
  - TWR < 1: 무엇을 해도 떨어짐 (호버 불가)
  - 우리 EDF: 1.63 → 여유 있음

#### 유도와 가정
- **뉴턴 만유인력**의 *근거리 근사*: $F = G\frac{Mm}{r^2} \approx mg$ (지표 근처, 고도 변동 무시)
- **회전 좌표계 효과 (Coriolis·원심)**: $\mathbf{F}_{\text{cor}} = -2m\boldsymbol{\Omega}\times\mathbf{v}$ — 지구 자전 보정. *무시함* (저속·단시간)
- **고도 변동**: $g(h) \approx g_0 (1 - 2h/R_E)$. 100m 고도에서 변동 ~0.003% → *무시*

#### 우려·보강
| 항목 | 영향 | 보강 |
|---|---|---|
| 위도에 따라 $g \in [9.78,\ 9.83]$ | TWR 추정 0.5% 오차 | 발사장 위도별 $g$ 입력 |
| 마하 1 비행 시 고도 차이 큼 | $g$ 변동 누적 | 고도 의존 $g(h)$ |
| 우주 진공 (장기 목표) | 항력 0, $g$ 거의 0 | 다른 모델 필요 |

### 4.4 추력 (자세히)

#### 발생 원리 (EDF)
- 전기 모터가 팬을 회전 → 공기를 *뒤로* (아래로) 가속
- 뉴턴 3법칙: 공기에 가한 힘 = 로켓이 받는 힘 (반대 방향, *위로*)
- 추력 크기 ≈ (질량유량) × (배기속도)

→ 시뮬은 *기계는 무시*하고 *결과인 추력 벡터*만 모델:
$$
\mathbf{F}_{\text{thrust,body}} = T \cdot \mathbf{d}_{\text{body}}
$$

#### 추력 크기 동역학 (스풀업 지연)
$$
\frac{dT}{dt} = \frac{T_{\text{cmd}} - T}{\tau_{\text{thr}}}, \quad \tau_{\text{thr}} = 0.08 \text{ s}
$$
- 명령 즉시 안 따라옴. 약 0.16초(2τ) 후 95% 도달.
- **유도**: 1차 시스템 (RC 회로 같은). EDF의 *전기적+기계적 관성* 근사.

#### 방향 — 김벌 각도
$$
\mathbf{d}_{\text{body}} = R_x(\alpha) R_y(\beta) \cdot \hat{z}
= \begin{pmatrix} \sin\beta \\ -\sin\alpha\cos\beta \\ \cos\alpha\cos\beta \end{pmatrix}
$$
- 김벌 0이면 $\mathbf{d}_{\text{body}} = \hat{z}$ (몸체 z축, 즉 "위")
- 김벌 12°면 $\mathbf{d}_{\text{body}}$ 가 21% 옆으로 기울어짐

#### 월드 좌표로 변환
$$
\mathbf{F}_{\text{thrust,world}} = R(\mathbf{q})\,\mathbf{F}_{\text{thrust,body}}
$$
- 자세가 바뀌면 *같은 김벌·throttle*이어도 월드 힘 방향 다름.
- 거꾸로 서 있으면 추력이 *지면 향함* → 더 빨리 추락.

### 4.5 항력 (자세히)

#### 수식
$$
\mathbf{F}_{\text{drag}} = -\tfrac{1}{2}\,\rho\,C_d\,A_{\text{eff}}\, |\mathbf{v}_{\text{air}}|\, \mathbf{v}_{\text{air}}
$$
- $\mathbf{v}_{\text{air}} = \mathbf{v}_{\text{world}} - \mathbf{v}_{\text{wind}}$ (대기 *상대* 속도)
- 방향: 대기 상대 속도의 *반대*. 즉 항상 *움직임을 방해*.
- 크기: 속도의 *제곱*에 비례 → 빠를수록 *훨씬* 큼.

#### 면적 $A_{\text{eff}}$ — 정면/측면 보간
$$
A_{\text{eff}} = A_{\text{ref}} + (A_{\text{side}} - A_{\text{ref}}) \cdot \frac{\|\mathbf{v}_{\text{air,xy}}\|}{\|\mathbf{v}_{\text{air}}\|}
$$
- 수직 강하 → $A_{\text{ref}} = 0.0079$ m² (지름 10 cm)
- 수평 비행 → $A_{\text{side}} = 0.045$ m² (45 cm 길이 측면)
- *근사*. 실제론 받음각 따라 곡선.

#### 작용 위치 — *압력 중심 (CP)*
- **실제론 항력이 CG가 아니라 CP에 작용** → 토크 발생
- 안정 비행: CP가 CG보다 *아래* 있어야 (화살 같은 형태)
- 우리 sim: **CP=CG 가정**, 항력 토크 *0* — *단순화*. 실제 핀 단 로켓엔 *큼*.

#### 크기 추정 (저속)
- v = 5 m/s, 측면적 → 항력 ≈ 0.5 × 1.225 × 0.5 × 0.045 × 25 ≈ 0.34 N
- v = 20 m/s → 항력 ≈ 5.5 N (무게의 22%!)
- → **저속 호버에선 작지만 고속에선 무시 못 함**

### 4.6 추력 → 토크 (회전 만드는 메커니즘)

#### 수식
$$
\boldsymbol{\tau}_{\text{thrust,body}} = \mathbf{r}_e \times \mathbf{F}_{\text{thrust,body}}
$$
- $\mathbf{r}_e = (0,\ 0,\ -L)$ — CG에서 엔진까지 (아래로 $L=0.45$ m)

#### 풀어 쓰기 (소각 김벌)
김벌 $\alpha$ (body x축 회전), $\beta$ (body y축 회전):
$$
\mathbf{F}_{\text{thrust,body}} \approx T\,(\beta,\ -\alpha,\ 1)
$$
$$
\boldsymbol{\tau} = (0,0,-L) \times T(\beta,-\alpha,1) = T\,L\,(-\alpha,\ -\beta,\ 0)
$$
- 김벌 x → 토크 -x (pitch)
- 김벌 y → 토크 -y (yaw)
- **z(roll) 토크 ≈ 0** — *단일 김벌로 roll 제어 불가*

#### 의미
- 김벌은 *작은 옆구리 힘* + *큰 토크* 만듦 (lever arm L 덕)
- 직접 가속은 작지만 *회전을 통해* 결국 추력 방향이 바뀌어 옆으로 감
- 이게 §4.7의 "회전 후에야 옆으로 가속" 사슬의 시작

### 4.7 전체 사슬 — *옆으로 가는* 데 걸리는 시간

```
0초:    김벌 명령 5°
0.005초: 서보 슬루로 김벌 *실제로* 1° 도달 (200°/s × 5ms)
0.01초:  김벌 5° 다 도달
        → 추력 옆 성분 = T·sin(5°) ≈ 0.087·T ≈ 2.1 N
        → 토크 = L·F_lateral ≈ 1 N·m
0.05초: 각속도 ω 쌓임 (α=τ/I ≈ 8 rad/s²)
        → ω ≈ 0.4 rad/s
0.1초:  자세 기울기 ≈ 2.3°
        → 추력 *월드* x성분 ≈ T·sin(2.3°) ≈ 1.0 N
0.2초:  자세 ≈ 9° 기울어짐
        → 가속도 옆 성분 ≈ 0.4 m/s² (이제 진짜 의미 있음)
```

→ **"옆으로 가야지"라고 결정한 순간부터 *실제 옆 가속*이 의미 있어지기까지 ~0.2초**. 이게 점질량 MPC가 *무시한* 핵심.

### 4.8 외란 — 가스트·랜덤력 (CG 작용 가정)

§5 참조. 모델링상 *CG에 작용*하는 가속도로 합쳐짐. 실제론 *압력 중심에 작용*해 토크도 만들지만 단순화.

### 4.9 *현재 모델링 안 된* 힘들

| 힘 | 우리 sim | 실제 |
|---|---|---|
| **지면 반작용 N** | z=0에서 *이벤트로 종료*. 다리·튕김 모델 없음 | 다리 강성·감쇠·마찰 + 자세 모멘트 |
| **양력 (lift)** | 0 (가정) | 핀·날개 달리면 큼. 받음각에 비례 |
| **CP 토크** | 0 (가정, CP=CG) | 실제: 핀이 있으면 안정 모멘트, 없으면 불안정 모멘트 |
| **자기력/유도** | 0 | 보통 무시 가능 |
| **Coriolis (지구자전)** | 0 | 저속·단시간 무시 가능 |
| **Magnus (회전체+측풍)** | 0 | 로켓이 굴러 회전(spin) 시 발생 |
| **충격파/압축성** | 0 | 마하 1 근처 *결정적* |
| **추력에 의한 충격 반동** | 무시 (T 연속) | 실제 솔리드 점화는 *충격* |

---

## (원래 4번 내용은 아래로 이동 — 개별 힘 상세)



### 4.1 중력
$$
\mathbf{F}_g = (0, 0, -m \cdot g), \quad g = 9.80665 \text{ m/s}^2
$$
- **유도**: 뉴턴 만유인력. 저고도라 $g$ 상수.
- **사용**: 매 스텝, 항상 작용.
- **우려**: 진짜 발사장은 위도/고도에 따라 $g \in [9.78, 9.83]$ — 0.5% 변동. 우리 sim엔 작은 영향.

### 4.2 추력 크기 (스풀업 1차 지연)
$$
\frac{dT}{dt} = \frac{T_\text{cmd}(t) - T(t)}{\tau_\text{thr}}, \quad \tau_\text{thr} = 0.08 \text{ s}
$$
$T_\text{cmd} = \text{throttle} \times T_\text{max}$, $T_\text{max} = 40$ N.

- **유도**: 모터/팬은 관성·전기적 시상수 때문에 즉시 못 따라옴. 표준 1차 시스템 근사.
- **이산화**: $T_{k+1} = \alpha T_\text{cmd} + (1-\alpha) T_k,\ \alpha = 1 - e^{-h/\tau}$. (Step 2 MPC에서 사용)
- **사용**: 매 RK4 substep.
- **우려·보강**:
  - 실제 EDF: 배터리 전압 강하(sag), 모터 ESC 비선형, 정·역방향 추력 비대칭 — 모두 *모델링 안 됨*
  - 실제 *액체/하이브리드 로켓*: 추력 지연 더 큼, 점화 시퀀스, 챔버 압력 동역학 — 완전 다른 모델 필요
  - 솔리드 로켓: 점화 후 *연소 곡선*이 미리 정해짐, 스로틀 *불가* — 우리 모델 부적합

### 4.3 항력 (대기 상대속도 기준)
$$
\mathbf{F}_\text{drag} = -\tfrac{1}{2}\,\rho\,C_d\,A_\text{eff}\, |\mathbf{v}_\text{air}|\, \mathbf{v}_\text{air}
$$
$\mathbf{v}_\text{air} = \mathbf{v} - \mathbf{v}_\text{wind}$. $\rho = 1.225$ kg/m³, $C_d = 0.5$.
$A_\text{eff}$ 는 정면적 → 측면적으로 선형 보간 (수평 비율에 따라):
$$
A_\text{eff} = A_\text{ref} + (A_\text{side} - A_\text{ref}) \cdot \frac{\|\mathbf{v}_\text{air,xy}\|}{\|\mathbf{v}_\text{air}\|}
$$
$A_\text{ref} = 0.0079$ m² (지름 10 cm), $A_\text{side} = 0.045$ m² (측면).

- **유도**: 표준 2차 항력 공식. 저속 + 비점성 가정.
- **사용**: 매 RK4 substep.
- **우려·보강**:
  - **실제 항력은 받음각·Reynolds 수에 따라 변함** — 우리 모델 단순함
  - **마하 1 이상 압축성·충격파 미모델링** — 마하 1 트랙으로 가면 *완전 다른* 공력 모델 필요 (CFD 또는 실험 데이터)
  - 양력(lift) 0 가정 — 실제 핀/날개 달리면 큼

### 4.4 김벌 → 추력 방향 + 토크
**노즐 방향** (몸체 좌표, gimbal_x=$\alpha$, gimbal_y=$\beta$):
$$
\mathbf{d}_\text{body} = R_x(\alpha)\,R_y(\beta) \cdot \hat{z} = (\sin\beta,\ -\sin\alpha\cos\beta,\ \cos\alpha\cos\beta)
$$
$$
\mathbf{F}_\text{thrust, body} = T \cdot \mathbf{d}_\text{body}
$$
$$
\boldsymbol{\tau}_\text{body} = \mathbf{r}_e \times \mathbf{F}_\text{thrust, body}, \quad \mathbf{r}_e = (0,0,-L)
$$
$L = 0.45$ m (CG → 엔진).

- **유도**: 강체 회전: $\boldsymbol{\tau} = \mathbf{r} \times \mathbf{F}$. 노즐이 CG보다 아래라 김벌→토크 직접 연결.
- **사용**: 매 RK4 substep, dynamics.state_derivative().
- **우려·보강**:
  - **CG가 *원점*이라는 가정** — 사용자 지적대로 실제 조립 시 정확한 CG 모름. 작은 오차도 *상수 토크 외란*으로 작용(=`thrust_misalign` 으로 부분 모델링)
  - 김벌 마운트의 강성/유격은 *0 가정* — 실제는 진동 가능
  - 단일 김벌은 **roll(z축 회전) 제어 불가** — 우리 sim은 roll을 제어 안 함(자연 안정성 가정)

### 4.5 서보 (김벌 슬루) — 시뮬레이터 강제
명령 김벌 각 $\alpha_\text{cmd}$가 들어와도, 실제 적용 각:
$$
\alpha_{k+1} = \alpha_k + \text{clip}(\alpha_\text{cmd} - \alpha_k,\ -\Delta_\text{max},\ +\Delta_\text{max})
$$
$\Delta_\text{max} = \dot\alpha_\text{max} \cdot h = 200\,\text{°/s} \cdot 0.002\,\text{s} = 0.4\text{°/스텝}$.

- **유도**: 서보 슬루율 한계. 단순 비례 한계.
- **사용**: `simulator.py`, `LandingEnv.step()` 의 서브스텝 루프.
- **우려·보강**:
  - **서보 데드밴드/백래시 0 가정** — 실제 1~3° 오차 있을 수 있음
  - **서보 내부 PID 응답 (수십 ms)** 없음 — 즉시 명령 추종 가정
  - **PWM 양자화** 없음 — 실제 1~10 μs 단위
  - **토크 한계 vs 부하** — 우리는 슬루만, 실제 김벌이 *공기력 부하 받으면* 느려짐

---

## 5. 외란 모델 (Disturbance)

**위치**: `src/rocketsim/scenarios/disturbances.py`

### 5.1 평균풍 + Ornstein-Uhlenbeck 돌풍
$$
\mathbf{v}_\text{wind}(t) = \mathbf{v}_\text{mean} + \mathbf{g}(t)
$$
$$
\mathbf{g}_{k+1} = a \cdot \mathbf{g}_k + \mathcal{N}\!\left(0,\ \sigma_g \sqrt{1-a^2}\right), \quad a = e^{-h/\tau_g}
$$

- **유도**: OU 프로세스. 시간 상관관계가 있는(꼬리가 있는) 랜덤 변동. 평균회귀.
- **시상수** $\tau_g$ = 0.7~1.5 s (시나리오별)
- **사용**: 매 스텝 wind 벡터 생성 → 항력 식의 $\mathbf{v}_\text{wind}$.

### 5.2 랜덤 외부 힘 (가스트 충격 근사)
$$
\mathbf{F}_\text{ext}(t) = \text{OU}(0, \sigma_f, \tau_f)
$$
- 추가 모델링: 단순 항력으로 표현 어려운 *돌풍 충격*.
- **사용**: 동역학식의 $\mathbf{F}_\text{ext}$.

### 5.3 도메인 랜덤화 (에피소드당 기체 변경)
매 에피소드 새 기체:
$$
m = m_0 \cdot U(0.85, 1.15), \quad T_\text{max} = T_{0} \cdot U(0.9, 1.1), \quad I = I_0 \cdot U(\ldots)
$$
- **유도**: 실제 제작 시 부품 편차. sim-to-real 격차 완화.
- **사용**: `env.reset()` 시 한 번 적용.

---

## 6. 센서 모델

**위치**: `src/rocketsim/navigation/sensors.py`

```
측정값 = 참값 + 가우시안 노이즈
```
- 위치: $\sigma_p = 0.05$ m
- 속도: $\sigma_v = 0.20$ m/s
- 각속도: $\sigma_\omega = 0.08$ rad/s
- 자세: small-angle 회전 $\sigma_\theta = 1.5$°

`obs_noise` 배율로 전체 강도 조절.

- **유도**: 단순 IMU 노이즈 가정. 실제는 *훨씬* 복잡.
- **우려·보강 (사용자 지적과 동일)**:
  - **IMU bias drift** — 실제 자이로 bias는 시간에 따라 *천천히 표류* (수 deg/min)
  - **온도 의존성** — 실제 센서 bias는 온도에 따라 변함
  - **scale factor 오차** — 1 m/s² 측정이 실제로 0.98 m/s² 일 수 있음
  - **IMU misalignment** — 센서 축이 *기체 축*과 정확히 일치 안 함
  - **고도 측정**: 우리 sim은 *완벽한 z* 가정. 실제는 기압계(저고도 약함) + 거리계(ToF, 지면 반사 의존)
  - **GPS** 미모델링 — 실외에선 위치 추정 핵심

---

## 7. 상태 추정 (Navigation Filter)

**위치**: `src/rocketsim/navigation/estimator.py`

### 7.1 Low-Pass 보조 필터 (현재 사용)
각 변수별:
$$
\hat{x}_{k+1} = (1 - \alpha)\,\hat{x}_k + \alpha\, x_\text{meas}, \quad \alpha = 1 - e^{-h/\tau}
$$

| 변수 | $\tau$ (s) |
|---|---|
| 위치 | 0.04 |
| 속도 | 0.12 |
| 자세 (nlerp) | 0.001 |
| 각속도 | 0.003 |

자세는 쿼터니언 nlerp:
$$
\hat{\mathbf{q}}_{k+1} = \text{normalize}\!\left( (1-\alpha)\,\hat{\mathbf{q}}_k + \alpha\, \mathbf{q}_\text{meas} \right)
$$

### 7.2 적응형 (2026-05-28 추가)
$$
\tau_\text{eff} = \tau_0 \cdot \frac{\text{obs\_noise}}{2.0}
$$
노이즈 0 → 필터 통과(lag 없음), 노이즈 2 → 기본 필터.

- **유도**: "필터 강도 = 노이즈 강도에 비례" — 추정기-제어기 정합. EKF의 *측정 공분산 적응*과 유사.
- **사용**: `evaluate_navigation.py` 매 reset.
- **우려·보강**:
  - 진짜 EKF가 아님 (공분산 없음, innovation 없음)
  - 동역학 *예측 단계* 없음 — 추력·중력으로 *forward predict* 안 함. 실제 EKF는 dynamics와 measurement를 함께 융합.
  - 자이로 적분으로 자세 *propagate* 후 가속도계로 보정 — 표준 attitude EKF — 미구현

---

## 8. PID 컨트롤러 (LandingPID / HoverPID)

**위치**: `src/rocketsim/controllers/pid.py` + `landing.py` + `guidance/landing.py`

### 8.1 캐스케이드 구조
```
위치 오차 → 목표 가속도 → 목표 추력벡터(크기+방향) → 자세 PID → 토크 → 김벌
              §8.2              §8.3                 §8.4    §8.5    §8.6
```

### 8.2 외측 (위치) — PID + 적분
수평:
$$
\mathbf{a}_{xy}^\text{des} = K_p^\text{pos}\,(\mathbf{p}_\text{target,xy} - \mathbf{p}_\text{xy}) - K_d^\text{pos}\,\mathbf{v}_\text{xy} + K_i^\text{pos}\!\int (\mathbf{p}_\text{target,xy} - \mathbf{p}_\text{xy})\,dt
$$
수직:
$$
a_z^\text{des} = K_p^z\,(z_\text{target} - z) - K_d^z\,v_z + K_i^z\!\int (z_\text{target} - z)\,dt
$$
**적분 누적량 클램프** (anti-windup): $[-3, 3]$ (수평), $[-5, 5]$ (수직).

게인 (튜닝됨): $K_p^\text{pos}=1.6,\ K_d^\text{pos}=2.4,\ K_i^\text{pos}=0.6$, $K_p^z=6,\ K_d^z=4,\ K_i^z=1.5$.

- **유도**: 표준 PID. 적분이 정상상태 외란(바람, 추력 오정렬, 질량 오차) 흡수.
- **사용**: 매 50Hz 제어 사이클.

### 8.3 목표 추력벡터
$$
\mathbf{F}_\text{des} = m \cdot \left( a_{xy}^\text{des,x}, a_{xy}^\text{des,y}, a_z^\text{des} + g \right)
$$
크기 → throttle (정규화):
$$
\text{throttle} = \text{clip}\!\left( \frac{\|\mathbf{F}_\text{des}\|}{T_\text{max}},\ 0,\ 1 \right)
$$
방향 → 목표 body z축 $\hat{\mathbf{z}}_\text{des} = \mathbf{F}_\text{des} / \|\mathbf{F}_\text{des}\|$.

**최대 기울기 cone 적용**: $\hat{\mathbf{z}}_\text{des}$ 의 기울기가 `max_tilt=25°` 넘으면 cone 표면으로 사영.

### 8.4 내측 (자세) — 기하 제어
현재 body z축 (월드): $\hat{\mathbf{z}}_\text{now} = R(\mathbf{q})\,\hat{z}$.
**오차 회전 벡터** (월드):
$$
\mathbf{e}_\text{world} = \hat{\mathbf{z}}_\text{now} \times \hat{\mathbf{z}}_\text{des}
$$
(작은 각도에서 $\mathbf{e}_\text{world} \approx$ 회전축·각). 몸체 좌표로 변환:
$$
\mathbf{e}_\text{body} = R(\mathbf{q})^T\,\mathbf{e}_\text{world}
$$
적분 누적 후:
$$
\boldsymbol{\tau}_\text{des} = I \cdot ( K_p^\text{att}\,\mathbf{e}_\text{body} - K_d^\text{att}\,\boldsymbol{\omega} + K_i^\text{att}\!\int \mathbf{e}_\text{body}\,dt )
$$
$K_p^\text{att}=130,\ K_d^\text{att}=24,\ K_i^\text{att}=55$.

- **유도**: 기하학적 자세 제어 (Lee/Loh, "Geometric tracking control" 류). 쿼터니언/SO(3) 위에서 직접 작동, 소각 가정 *아님* → 대각도 회복 가능.
- **사용**: 모든 PID 변형 내부.

### 8.5 토크 → 김벌 (역산)
소각 모델:
$$
\tau_x \approx -L \cdot T \cdot \alpha_\text{gimbal}, \quad \tau_y \approx -L \cdot T \cdot \beta_\text{gimbal}
$$
→ 역산:
$$
\alpha_\text{gimbal} = \text{clip}\!\left( \frac{-\tau_x^\text{des}}{L \cdot T},\ \pm 12° \right)
$$
$T$는 현재 *추정* 추력 (또는 명령 throttle × T_max). 0 추력시 발산 방지: $T \geq 0.25\,T_\text{max}$.

- **유도**: §4.4 토크 식 역연산. 작은 김벌 각도라 $\sin \alpha \approx \alpha$.
- **우려·보강**: $T$가 *작을* 때 (착륙 직전 감속) 김벌 권한도 작음 → 모델은 알지만 PID 게인은 *고정*. Step 1 MPC의 thrust-magnitude-scaled slew 가 이걸 다룸.

### 8.6 착륙 게이트 (LandingGuidance)
**준비도** (0..1):
$$
g_\text{ready} = \exp\!\left(-\frac{r^2}{w_r^2}\right) \exp\!\left(-\frac{v_h^2}{w_v^2}\right) \exp\!\left(-\frac{\theta^2}{w_\theta^2}\right)
$$
$r$=수평거리, $v_h$=횡속도, $\theta$=기울기. 게이트 폭 $w_*$은 *고도에 비례 축소*:
$$
w_*(z) = w_*^\text{nom} \cdot \left( \text{tight\_frac} + (1-\text{tight\_frac}) \cdot \min(z/z_\text{gate},\ 1) \right)
$$
하강 속도: $v_\text{desc} = \text{clip}(\text{flare\_gain} \cdot z_\text{set},\ v_\text{min},\ v_\text{max}) \cdot (g_\text{ready}\,\text{or creep})$.

- **유도**: 휴리스틱. "중심·자세 안 잡히면 호버, 잡히면 하강". 손튜닝.
- **사용**: 모든 LandingPID/Waypoint 변형.

---

## 9. MPC — Convex Point-Mass (현재 옛 버전)

**위치**: `src/rocketsim/controllers/mpc.py`, 클래스 `CvxpyPointMassMPC`

### 9.1 상태/제어
| 기호 | 의미 | 차원 |
|---|---|---|
| $\mathbf{p}_k$ | 위치 (state) | 3 |
| $\mathbf{v}_k$ | 속도 (state) | 3 |
| $\mathbf{u}_k$ | 추력 가속도 벡터 (control) | 3 |

수평선 $N = 20$ (4초 / 0.2s 스텝).

### 9.2 동역학 (점질량, 즉시 추력 가능 *가정*)
$$
\begin{aligned}
\mathbf{p}_{k+1} &= \mathbf{p}_k + h\mathbf{v}_k + \tfrac{1}{2}h^2(\mathbf{u}_k + \mathbf{g} + \mathbf{b}) \\
\mathbf{v}_{k+1} &= \mathbf{v}_k + h(\mathbf{u}_k + \mathbf{g} + \mathbf{b})
\end{aligned}
$$
$\mathbf{g} = (0,0,-9.8)$, $\mathbf{b}$: 추정 외란 bias (옵션).

### 9.3 제약 (모두 convex)
- $\|\mathbf{u}_k\|_2 \le T_\text{max}/m$ — 추력 magnitude 한계 (SOC)
- $\|\mathbf{u}_{k,xy}\|_2 \le \tan(\theta_\text{max}) \cdot u_{k,z}$ — 기울기 cone (SOC)
- $u_{k,z} \ge 0$ — 양의 방향만
- $p_{k,z} \ge 0$ — 지면 위
- $v_{k,z} \ge -4$ — 최대 하강속도
- $v_{k,z} + 0.45\,p_{k,z} \ge -0.45$ — 글라이드슬로프
- (옛 버전) $\|\mathbf{u}_{k+1} - \mathbf{u}_k\|_2 \le \text{du\_max}$ — 소프트 슬루

### 9.4 비용함수
$$
J = \sum_{k=0}^{N-1}\left[ \mathbf{p}_k^T Q_p \mathbf{p}_k + \mathbf{v}_k^T Q_v \mathbf{v}_k + r_u\,\|\mathbf{u}_k - \mathbf{u}_\text{hover}\|^2 + r_{du}\,\|\Delta \mathbf{u}_k\|^2 \right] + \mathbf{p}_N^T Q_p^f \mathbf{p}_N + \mathbf{v}_N^T Q_v^f \mathbf{v}_N
$$

- **유도**: 표준 quadratic regulator + 최종 상태 페널티 + 평활도.
- $Q_p^f$, $Q_v^f$가 매우 큼 → 최종 상태에 패드·정지 강제.

### 9.5 풀이 + Receding Horizon
- **솔버**: CLARABEL (1차 SOC), 실패 시 SCS
- **재계획**: 매 0.2초마다 풀어 $\mathbf{u}_0$만 적용
- **유도**: 표준 MPC. cvxpy로 problem 한 번 만들고 parameter만 갱신.

### 9.6 우려·보강 — 점질량의 *근본* 한계
- ✘ 자세 동역학 무시: 노즐 틀어 → 실제로 가속까지 *시간 걸림*. MPC는 *즉시* 가정.
- ✘ 추력 지연 무시
- ✘ 김벌 한계 *간접*만 (cone constraint)
- → **김벌 87~94% 시간 한계에 박힘** (실증)
- 보강: §11 Step 1 + Step 2 + Step 3

---

## 10. RL 컨트롤러 (PPO)

**위치**: `src/rocketsim/envs/landing_env.py` + `scripts/train_curriculum.py`

### 10.1 정책 (Gaussian MLP)
$$
\pi_\theta(\mathbf{a}\,|\,\mathbf{s}) = \mathcal{N}(\mu_\theta(\mathbf{s}),\ \sigma_\theta(\mathbf{s}))
$$
- $\mathbf{s}$ = 최근 6프레임 관측 (78차원)
- $\mathbf{a} \in [-1, 1]^3$ = (throttle, gimbal_x, gimbal_y) 정규화
- MLP 256×256 + tanh 출력

### 10.2 PPO 손실
$$
L = \mathbb{E}\!\left[\min\!\left( r_t(\theta)\,\hat{A}_t,\ \text{clip}(r_t(\theta), 1-\epsilon, 1+\epsilon)\,\hat{A}_t \right)\right] - c_v\,L_\text{value} - c_e\,\mathcal{H}
$$
$r_t = \pi_\theta / \pi_\text{old}$, $\hat{A}_t$ = GAE 우위, $\epsilon = 0.2$.

- **유도**: Schulman 2017. 정책 업데이트 *너무 크게* 못 가게 클리핑 → 안정.

### 10.3 보상 (포텐셜 기반 shaping + 종단)
$$
r_t = (\Phi_{t+1} - \Phi_t) - \text{step\_penalty} + r_\text{terminal}
$$
포텐셜:
$$
\Phi = -(w_\text{alt}\,z + w_\text{horiz}\,\|\mathbf{p}_{xy}\| + w_\text{speed}\,\|\mathbf{v}\| + w_\text{tilt}\,\theta)
$$
종단: 연착륙 +100, 하드랜딩 0~−40, 크래시 −100, 타임아웃 −20.

- **유도**: Ng (1999) 포텐셜 기반 보상. 정책 불변성 (이론).
- **함정 6종 디버깅 거침**: die-fast, hover-farming, 위험회피, 발견실패, 붕괴, 보상스케일 발산 (모두 `docs/devlog.md` 기록).

### 10.4 학습 안정화
- **VecNormalize**: reward를 running std로 정규화 → 스케일 무관
- **target_kl = 0.06**: 정책 업데이트 너무 큰 경우 조기 중단
- **커리큘럼**: calm→hard 단계적 (init_scale + 외란 ramp)

---

## 11. Step 1 — Actuator-Aware MPC (방금 추가)

**위치**: `mpc.py` 클래스 `CvxpyActuatorAwareMPC` (2026-05-29 v1)

### 11.1 핵심 차이 — $\mathbf{u}$를 *상태*로 승격
| | 점질량 MPC | Step 1 MPC |
|---|---|---|
| $\mathbf{u}$ | 제어 (즉시 어디로든) | **상태** |
| $\mathbf{du}$ | (없음) | **제어** (변화율) |
| 슬루 | (없음 또는 소프트) | **하드 SOC** |
| slack | (없음) | **있음** (infeasibility 흡수) |

### 11.2 추가 동역학
$$
\mathbf{u}_{k+1} = \mathbf{u}_k + h \cdot \mathbf{du}_k
$$

### 11.3 추가 제약 (★ 핵심)
$$
\|\mathbf{du}_k\|_2 \le \beta \cdot u_{k,z} + s_k, \quad s_k \ge 0
$$
$\beta = 0.6$ — 슬루 권한이 *추력 magnitude에 비례* (codex caveat #4 반영).

### 11.4 추가 비용
$$
J \mathrel{+}= w_\text{slack} \sum_k s_k^2, \quad w_\text{slack} = 50
$$
slack이 평소엔 0, 외란 회복 중에만 살짝 위반.

### 11.5 유도
- "추력 벡터가 *순간 텔레포트* 불가" → 변화율에 한계
- $\beta = J_\text{eff}/T_\text{hover}$ where $J_\text{eff}=(T/m) \cdot \omega_\text{achievable}$
- 호버 추력 $T/m = 9.8$, 도달가능 각속도 $\omega \approx 3$ rad/s → $J_\text{eff} \approx 29$ m/s³ → $J_\text{eff} \cdot h \approx 5.9$ m/s² @ h=0.2 → $\beta \approx 0.6$

### 11.6 초기 검증 결과 (n=50, estimated)
| | hard | noisy |
|---|---|---|
| PID | 58% | 84% |
| 옛 waypoint (점질량) | 52% | 76% |
| **Step 1 actuator** | **46%** | **74%** |

→ **현재 약간 *나빠짐*** (왜인지 §13 우려에 정리). 튜닝 필요.

---

## 12. 평가 기준

### 12.1 연착륙 4 기준 (AND)
$$
\text{success} = (r \le 0.5\text{m}) \land (v_z \le 1.0\text{m/s}) \land (v_h \le 0.5\text{m/s}) \land (\theta \le 8°)
$$

### 12.2 에너지 지표
$$
E = \int_0^{t_\text{end}} \text{throttle}(t)\, dt
$$
연료 소모 *대용*. 같은 성공률에서 작을수록 효율적.

### 12.3 실패 분해
실패한 touchdown마다 어느 기준 깨졌나 카운트 → 어디 고쳐야 할지 *진단*.

---

## 13. 우려 / 보강 필요한 점들 (전체 리스트)

### A. 시뮬레이션 자체의 단순화 (sim-to-real 격차)

| 영역 | 현재 sim 가정 | 실제와 다른 점 |
|---|---|---|
| **무게중심(CG)** | 기체 좌표 원점 (정확히) | 조립마다 다름, ±수 cm. 상수 토크 외란으로 부분 모델링 (thrust_misalign) |
| **관성텐서** | 대각 (roll축 분리) | 비대각 항 가능 (배터리·전선 비대칭), 시간 변동 (연료 소모 시) |
| **질량** | 상수 | 실제는 연소·연료 소모로 *감소* (액체/하이브리드). EDF는 거의 일정 |
| **추력 곡선** | 1차 지연만 | 실제 EDF: 배터리 전압 sag (시간 지남에 따라 출력 ↓), 모터 효율 곡선 |
| **항력** | 단순 2차 | 실제: 받음각 의존, Re수 의존, 압축성 (마하 1↑) |
| **양력** | 0 | 핀 달리면 큼 |
| **김벌 서보** | 위치+속도 한계만 | 데드밴드, 백래시, 토크 한계 (공기력 부하), PWM 양자화, 내부 PID 지연 |
| **센서 노이즈** | 가우시안 white | bias drift, 스케일 오차, 온도 의존, 미스얼라인먼트 |
| **GPS/외부 위치** | 사용 안 함 | 실외에선 필수 |
| **지면 접촉** | z=0 이벤트로 종료 | 실제: 다리 강성/감쇠/마찰, 튕김, 쓰러짐 |
| **바람** | OU 프로세스 균일 | 실제: 고도·시간·지형에 따라 *변동* 큼 |
| **자세 ω 적분** | 정확 (RK4) | 실제: 자이로 적분만으로는 drift |

### B. EDF 로켓 ↔ 진짜 로켓 (사용자 지적)

| 영역 | EDF (현재) | 진짜 로켓 |
|---|---|---|
| **추진** | 전기 + 공기 흡입식 (대기 필요) | 화학 (자기 추진제) |
| **마하 1** | 불가 (최대 ~200~400 km/h) | 가능 (고체 모터 1초 안에) |
| **스로틀** | 자유롭게 (전기 모터) | 솔리드 *불가*, 액체/하이브리드만 |
| **재점화** | 자유 | 솔리드 거의 불가 |
| **상승 vs 착륙** | 같은 모터로 가능 | **다른 문제로 풀어야 함** ★ 사용자 지적 |
| **연료 소모** | 배터리 (전력만) | 연료 + 산화제 → 질량 변화 → CG 이동 → 관성 변화 |
| **점화 시퀀스** | 즉시 가능 | 수 초 ~ 수 십 초 (액체) |
| **챔버 압력 동역학** | 없음 | 매우 빠르고 복잡 |
| **노즐 열변형** | 없음 | 있음 (고체) |
| **배터리 sag / 추진제 압력 변화** | 없음 | 있음 |

### C. 상승 vs 착륙 — 다른 문제 (사용자 핵심 지적)

```
[상승]
- 솔리드: 점화 후 *고정 추력 곡선* 따라감. 컨트롤러는 *조향*만 (TVC).
  추력 *조절 불가*.
- 액체: 조향 + 스로틀 둘 다 가능. 더 복잡한 GNC.
- 회수: 낙하산 (저속), 또는 *동력 착륙*(SpaceX).

[착륙 (역추진)]
- 솔리드: 별도 *착륙 모터*, 정확한 점화 *타이밍* 으로 0속도 도달 (suicide burn).
  스로틀 없어 *튜닝 불가* → 매우 어려움.
- 액체: 호버까지 가능. 정밀 착륙 가능. → SpaceX Falcon 9.
- EDF: 전기라 자유롭게 스로틀. *착륙 제어 *연습*용*.

★ 우리 현재 sim: 한 모터로 *양쪽* 가능 (EDF 특성). 
   진짜 로켓엔 *완전 다른 알고리즘*이 양쪽에 필요할 수 있음.
```

### D. 현재 컨트롤러의 한계

| 한계 | 영향 | 보강 방향 |
|---|---|---|
| **PID 게인 고정** | 추력·고도·외란에 따라 *최적치* 변하지만 고정 | Gain scheduling 또는 LQR |
| **MPC 점질량** | 김벌 87% 포화 | Step 1+2+3 (진행 중) |
| **MPC 슬루 βu_z** | 추력 *작을 때* 권한도 작음 — 인지는 함. *얼마나 작아야 위험*한지 부정확 | 자세 상태 명시 (Step 3) |
| **RL roll 제어 안 함** | 단일 김벌이라 roll 제어 *불가* | 카운터-로터 또는 핀 (하드웨어) |
| **추정기 = 저역통과** | 진짜 EKF 아님, 동역학 예측 없음 | 진짜 EKF (IMU 적분 + 가속도계 보정) |
| **착륙 게이트 = 휴리스틱** | 손튜닝, 일반화 약함 | 최적 정지 시간 / 학습된 게이트 |

### E. 평가 자체의 한계

| 한계 | 영향 |
|---|---|
| **고정 시드 0~49** | 표본 편향. 다른 시드 집합으론 다른 결과 가능 |
| **AND 4기준 성공** | 임의 임계. 다른 임계로 *순위 바뀔 수 있음* |
| **에너지 = 단순 ∫throttle** | 실제 배터리 소모와 다름 (효율 곡선 무시) |
| **하나의 시나리오만** | hop test, divert, 회복 등 *각각* 별도 검증 필요 |
| **실세계 외란 분포 모름** | hard·noisy가 실제 EDF 실험과 *얼마나* 일치할지 미지 |

## 추가사항 — 2026-05-29 17:04 KST — 음속 돌파, 형상, EDF 한계

사용자 추가 질문: "음속 돌파만 하더라도 형상에 대한 고민이 필요한가? EDF로도 음속 돌파는 할 수 있을 것 같다."

결론:

> **마하 1 근처부터는 제어 문제 이전에 공력 형상, 구조, 추진 방식이 병목이다. EDF로 음속 돌파를 목표로 삼는 것은 현재 Phase 1 EDF 착륙 제어 실험과 별도 트랙으로 분리해야 한다.**

### 13.F 음속 근처에서 힘이 어떻게 달라지는가

현재 저속 시뮬의 항력은:

$$
\mathbf{F}_\text{drag} = -\tfrac{1}{2}\rho C_d A |\mathbf{v}_\text{air}| \mathbf{v}_\text{air}
$$

마하 1 근처에서도 기본 동압은:

$$
\bar{q} = \tfrac{1}{2}\rho V^2
$$

해수면에서 $V \approx 343\ \text{m/s}$, $\rho \approx 1.225\ \text{kg/m}^3$ 라고 하면:

$$
\bar{q} \approx 0.5 \times 1.225 \times 343^2 \approx 72{,}000\ \text{Pa}
$$

즉 기체 표면과 핀에는 **약 72 kPa 급 동압**이 걸린다. 문제는 여기서 끝나지 않는다. 마하 0.8~1.2 천음속 구간에서는 충격파가 생기면서 $C_d$ 자체가 증가한다.

$$
D = \bar{q} C_d(M,\alpha,\text{shape}) A
$$

- $M$: 마하수
- $\alpha$: 받음각
- `shape`: 노즈콘, 동체 세장비, 핀, 흡입구, 단면 변화

현재 시뮬은 $C_d=0.5$ 상수와 단순 면적 보간만 사용한다. 따라서 **천음속 drag rise, wave drag, shock-induced separation은 전혀 모델링하지 않는다.**

### 13.G 음속 돌파에 필요한 형상 검토

음속 돌파를 별도 비행체 목표로 둔다면 최소한 아래가 필요하다.

| 항목 | 왜 필요한가 | 현재 sim 상태 | 보강 방향 |
|---|---|---|---|
| **노즈콘** | 둥근 전면은 wave drag 증가. 긴 ogive/conical 계열이 유리 | 단순 원통/면적 가정 | 노즈콘 형상별 $C_d(M)$ 테이블 |
| **동체 직경/단면적** | 항력 $D \propto A$ | 지름 10cm 고정 가정 | 직경, 길이, 세장비 trade study |
| **핀 형상** | CP 위치, 안정성, flutter에 결정적 | 양력/CP 토크 없음 | 핀 lift/drag/moment 모델 |
| **CP-CG 관계** | 고속 안정성은 CP가 CG 뒤쪽이어야 함 | 항력은 CG에 작용한다고 가정 | CP 위치 계산, static margin |
| **받음각 공력** | 작은 자세 오차도 큰 횡력/모멘트 생성 | lift=0 | $C_L(\alpha,M)$, $C_m(\alpha,M)$ |
| **천음속 압축성** | 마하 0.8~1.2에서 충격파/박리 | 없음 | compressible aero 모델 또는 OpenRocket/CFD 데이터 |
| **구조/플러터** | 핀과 마운트가 고동압에서 진동/파손 가능 | 구조 유연성 0 | fin flutter margin, FEA/실험 |
| **열/표면** | 짧은 시간이라도 고속 표면 가열과 진동 고려 | 없음 | 목표 속도/시간별 열 검토 |

현재 EDF 착륙 테스트베드는 저속 GNC 검증용이다. 이 형상 검토는 **Phase 3 마하 1 비행체**의 별도 설계 문제다.

### 13.H EDF로 음속 돌파가 어려운 이유

EDF는 공기 흡입식 팬이다. 저속 호버에서는 정지 추력을 만들기 쉽지만, 마하 1 근처에서는 다음 문제가 생긴다.

| 문제 | 설명 |
|---|---|
| **흡입구(inlet)** | 마하 근처 유입 공기를 팬이 처리할 수 있게 감속/압축해야 함. 잘못하면 충격파와 유동 박리 발생 |
| **팬 tip Mach** | 팬 블레이드 끝단 속도 + 유입 속도가 transonic/supersonic으로 가면 효율 급락 |
| **덕트 충격파** | 덕트 내부 압력파/충격파로 압력 손실과 실속 발생 가능 |
| **배기속도 한계** | 음속 돌파에는 높은 exhaust velocity와 큰 power가 필요 |
| **전력 밀도** | 배터리+EDF 조합의 에너지/출력 밀도는 화학 로켓보다 낮음 |
| **추력 유지** | 고속에서 정지추력 수치가 그대로 유지되지 않음. 실제 net thrust가 급감할 수 있음 |

따라서 EDF는 **저속 TVC/GNC 연습용**으로는 좋지만, **마하 1 추진체 후보로는 부적합**하다고 보는 것이 보수적이다. 마하 1은 고체 로켓 모터, 액체 로켓, 또는 터보제트/램제트 계열 문제로 분리해야 한다.

### 13.I 현재 로드맵에 반영할 분리 원칙

```
Phase 1: EDF 저속 테스트베드
  목적: 자세 제어, 호버, hop, 착륙 GNC
  핵심: 추정기 + PID/MPC + TVC + 착륙 판단
  속도: 저속, 비압축성 가정 가능

Phase 2: 아음속 로켓 TVC 호퍼
  목적: 화학 추진, TVC, 추력곡선, 연료/질량 변화
  핵심: 상승과 착륙을 분리해서 모델링
  속도: 아음속

Phase 3: 마하 1 비행체
  목적: 음속 돌파, 고속 안정성, 회수
  핵심: 형상/공력/구조/법규/안전
  착륙: 우선 낙하산 회수. 동력 역추진은 장기 별도 트랙
```

마하 1 목표는 현재 EDF 착륙 GNC 실험을 무효화하지 않는다. 다만 **같은 기체/같은 추진체로 자연스럽게 확장되는 목표가 아니다.**

---

## 14. 한눈에 — 알고리즘 *어디서* 작동하나

```
시간 ──────────────────────────────────────────────────────────────────────────────→
        매 0.002s (500 Hz)                  매 0.02s (50 Hz)
       ┌──────────────────┐               ┌─────────────────────┐
       │ 시뮬 RK4 적분    │               │ 컨트롤러 결정      │
       │ §3 동역학        │ 50번 마다 →   │ §6 측정 받음       │
       │ §4 힘·토크       │               │ §7 상태 추정       │
       │ §4.5 서보 슬루   │               │ §8/9/10/11 제어    │
       │ §5 외란 적용     │               │ → throttle, gimbal │
       └──────────────────┘               └─────────────────────┘
              ↓                                      ↓
              └─────────────명령 전달───────────────┘
              (controller가 결정 → env가 다음 0.02s 동안 동일 명령으로 substep 10번)
```

- 컨트롤러는 *느린 루프* (50Hz, 사람 머리에 가까움)
- 물리는 *빠른 루프* (500Hz, 수치 정확성)
- 그 사이에 *서보 슬루* 가 들어가 *명령을 부드럽게 적용*

---

## 15. 한 문장 정리

> 우리 sim은 **6-DOF 강체 + 김벌 추력 + 1차 지연 추력 + 2차 항력 + OU 외란 + 가우시안 노이즈** 의 *상당히 단순화된* 모델 위에서, **PID·MPC·RL** 세 가지 컨트롤러를 *서로 다른 강점*으로 비교 중. EDF라 *상승·착륙 같은 모터로* 가능하지만, 진짜 로켓으로 가면 **상승 (TVC 솔리드/액체) vs 착륙 (재점화 가능 액체)** 이 *다른 알고리즘 영역*이 됨.

---

## 부록 A — 파일 위치 색인

| 알고리즘 | 파일 | 핵심 함수/클래스 |
|---|---|---|
| §3 동역학 | `src/rocketsim/dynamics.py` | `state_derivative`, `rk4_step` |
| §4 힘 (추력/김벌/항력) | `dynamics.py` + `vehicle.py` | `state_derivative` 내부 |
| §4.5 서보 슬루 | `simulator.py`, `envs/landing_env.py` | `_rate_limit_gimbal` |
| §5 외란/랜덤화 | `scenarios/disturbances.py` | `DisturbanceModel`, `Randomization` |
| §6 센서 | `navigation/sensors.py` | `SensorModel` |
| §7 추정기 | `navigation/estimator.py` | `LowPassStateEstimator` |
| §8 PID | `controllers/pid.py`, `controllers/landing.py`, `guidance/landing.py` | `HoverPID`, `LandingPID`, `LandingGuidance` |
| §9 점질량 MPC | `controllers/mpc.py` | `CvxpyPointMassMPC` |
| §10 RL | `envs/landing_env.py`, `scripts/train_curriculum.py` | `LandingEnv`, train_curriculum |
| §11 Step 1 MPC | `controllers/mpc.py` (2026-05-29 추가) | `CvxpyActuatorAwareMPC` |
| §12 평가 | `scripts/evaluate*.py` | `evaluate_navigation.py` |

## 부록 B — 다음 보강 우선순위 (요약)

1. **CG 불확실성 모델링 강화** (사용자 지적): `thrust_misalign` 외에도 *시간 변동 CG* 옵션 (연료 소모 emulation).
2. **EKF 도입**: 자이로 적분 + 가속도계 보정. 진짜 propagate-update.
3. **지면 접촉 모델**: 다리·감쇠·튕김·쓰러짐.
4. **연료 예산**: $\int \text{throttle} \le$ 한계 + 고갈 시 추력 0.
5. **하강·상승 분리 모델링**: 상승 단계에선 *솔리드 추력 곡선* 옵션 등 (마하 1 트랙 대비).
6. **MPC Step 2+3**: 추력 magnitude 지연 → 자세·각속도 상태화.
7. **로깅**: §부록 — 솔버 상태, EKF innovation, safety filter 액션 (현재 트레이스 CSV에 미포함).
8. **실기 시스템 식별**: 진짜 EDF로 추력 곡선, 김벌 응답, IMU bias 직접 측정 → sim 파라미터 *교정*.
9. **마하 1 별도 공력 트랙**: 노즈콘/핀/CP-CG/천음속 $C_d(M)$ / flutter / inlet 검토. EDF 저속 테스트베드와 분리.
