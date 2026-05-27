# rocket 🚀

추력편향(TVC) 로켓의 **발사 → 역추진 착륙** 제어를 개발하는 프로젝트.
제어기를 **PID vs RL** 로 비교하고 최종적으로 RL을 목표로 한다.
초기 단계는 안전한 저속 **EDF(전기 덕트 팬) 호버 테스트베드**(= 헬리콥터 호버 RL 예제와 같은 문제 구조)로 진행한다.

> **마하 1 참고:** EDF로는 마하 1이 불가능하다(공기 흡입식, 최고 ~200–400 km/h).
> 마하 1은 고체 로켓 추진 영역으로, 제어 개발 트랙과 **분리**해 진행한다.
> 자세한 내용은 [docs/roadmap.md](docs/roadmap.md).

## 구조
```
src/rocketsim/        6-DOF 시뮬레이터 패키지
  quaternion.py       쿼터니언 유틸
  vehicle.py          차량/환경 파라미터 (EDF 테스트베드)
  dynamics.py         6-DOF 강체 동역학 + RK4
  simulator.py        폐루프 시뮬레이터 (김벌 레이트 제한)
  controllers/pid.py  캐스케이드 PID 호버 베이스라인
scripts/run_hover.py  호버 데모
tests/                동역학 단위 테스트
docs/
  roadmap.md          단계별 로드맵
  hardware.md         하드웨어 설계
  devlog.md           작업 기록 (날짜·시간 + 내용)
```

## 빠른 시작
```bash
python3 -m venv .venv
.venv/bin/pip install numpy            # 또는: .venv/bin/pip install -e .
.venv/bin/python tests/test_dynamics.py
.venv/bin/python scripts/run_hover.py  # -> out/hover.csv + 요약
```

## 현재 상태
- ✅ 6-DOF 동역학 + RK4, 단위 테스트 통과
- ✅ PID 호버 베이스라인 (틸트 12.8°·오차 1.8 m → 3.1 s 내 오차 <1 cm, 틸트 0°)
- ⏳ Gymnasium RL 환경 / 착륙 시나리오 / PID↔RL 평가

작업 진행 기록은 [docs/devlog.md](docs/devlog.md)에서 따라갈 수 있다.
