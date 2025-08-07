# 34일차 - PID 제어 기법 및 응용
https://apm.byu.edu/prism/index.php/Site/OnlineCourses


## PID 제어 기본과 디지털 구현
1. 배경 : 현실의 물리 시스템 중 질량- 스프링 - 댐퍼 시스템
- 뉴턴 제2법칙에서 유도된 m x``(t) + c x`(t) + k x(t) = u(t) (질량, 댐퍼, 스프링 순)
- 제어 문제란, 에러 e(t) = Setpoint - x(t) = 0 으로 만다는 제어력 u(t)를 설계하는 것.

<img width="1043" height="638" alt="image" src="https://github.com/user-attachments/assets/1def3da2-3c55-41eb-89ee-7f81da3d2cfb" />

2. PID 제어 파라미터 값에 대한 정리.
| 파라미터   | 증가시 효과                                                                           | 감소시 효과                                          |
| ------ | -------------------------------------------------------------------------------- | ----------------------------------------------- |
| **Kp** | - 상승 시간(Rise time) 감소<br/>- 과도(overshoot) 증가 가능<br/>- 정착 시간(settling time) 약간 증가 | - 상승 시간 증가<br/>- 과도·진동 감소<br/>- 정착까지 시간 증가      |
| **Ki** | - 영점 편차(steady-state error) 감소 (영구 오차 제거)<br/>- 과도·진동 증가<br/>- 정착 시간 증가          | - 영구 오차 남을 수 있음<br/>- 진동·불안정성 완화<br/>- 정착 시간 단축 |
| **Kd** | - 과도 응답(overshoot) 억제<br/>- 진동 감소, 안정성 향상<br/>- 상승 속도(초기 반응) 느려질 수 있음            | - 과도 억제 기능 약화 → 오버슈트 증가<br/>- 진동 및 불안정성 증가      |

## SISO(시소), MIMO(마이모) 시스템을 위한 PID 튜닝
1. 단일 입출력 시스템인 SISO(시소) 시스템 PID 튜닝 전략
- 지글러-니콜스 방법 : 수학적 모델링 없이, 실험을 통해 초기 PID 이득 값을 찾는 경험적(Heuristic) 방법

2. MIMO 시스템 PID 튜닝 전략 3가지
- 분산 제어 : 상호 간섭은 무시하고, 각각의 입출력 쌍에 대해 독립적인 SISO PID제어기를 설계함.<br> ( 상호 간섭이 약한 MIMO 일때 효과적 )
- 순차적 루프 튜닝 : 가장 빠르거나 중요한 루프를 먼저 튜닝하고, 튜닝된 루프를 작동시킨 상태에서 다음 루프를 튜닝해나감.<br> ( 느린 루프 튜닝시, 앞선 빠른 루프들의 영향을 미리 고려하는 것 )
- 디커플링 : 가장 고급 방식이고 모델 기반의 접근 방식, "시스템 이득 행렬의 역행렬"과 유사한 "디커플러" 블록을 설계하여 상호 간섭을 수학적으로 상쇄시킨후 독립적인 SISO 제어기들을 튜닝.<br> ( 간섭을 없앤 후 분산 제어 적용 )

## PID 고급 개념(최적 제어)과 견고한 구현
1. 선형-이차 조절기(LQR)
- LQR : 수학적으로 정의된 비용함수 'J'를 최소화 하는것이 "최적"이라고 간주하며, 제어 법칙 u(t)를 찾는것.
- 이득 스케쥴링 : 선형 제어 기법인 PID, LQR을 비선형 시스템 적용하는 강력한 기법.

2. 미분 킥 (Derivative Kick)
- 순간적으로 Setpoint가 변한다면, `error = setpoint – PV` 계산에서 에러변화율이 매우커져 미분항이 매우 커진다.
- 미분 대상을 `derivative = (pv - self.previous_pv) / dt` pv로 바꿔서 해결가능.
        `d_term = -self.Kd * derivative`
- Setpoint가 변경되는 5초 시점에서 Controller Output이 비정상적으로 크게 튀는 것을 볼 수 있습니다. 이것이 바로 '미분 킥'입니다.<br>
  <img width="1007" height="541" alt="image" src="https://github.com/user-attachments/assets/ffa8b40c-ff9e-4daf-b205-1f197daacaaf" />

3. 적분 와인드업 기법들 ( Anti-windup ) : 액추에이터가 포화상태일때, I항이 더이상 누적되지 않도록 막는 매커니즘.
- PV가 처음 0에서 출발할 때부터 적분항에 (integral += error·dt) 오차가 계속 누적되어 결과값이 +100까지 커졌다.
- 10초에 Setpoint가 0으로 돌아왔음에도 불구하고, PV가 0을 훨씬 지나쳐 심각한 언더슈트 발생.<br> 이는 거대하게 '감겨버린(wound up)' Integral 값 때문.
  <img width="1022" height="531" alt="image" src="https://github.com/user-attachments/assets/f10a83a7-3902-4a03-bc60-c0fd1f5fc692" />

  - 적분항 클램핑(Integral Clamping) : 적분 누적값을 미리 정의한 상한(I_max)/하한(I_min) 범위 내로 제한하여 해결가능.<br>`integral = min(max(integral, I_min), I_max)`
  - 조건부 적분(Conditional Integration)	: 오차가 일정 임계값을 초과하거나 제어 출력이 포화 상태일 때 적분 누적을 일시 중지
