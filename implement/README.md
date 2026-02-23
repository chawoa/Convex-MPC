## 파일 구조 및 역할
### 파일명	역할 및 설명
| 파일명 (File Name) | 역할 및 설명 | 관련 논문 근거 (Reference) |
| :--- | :--- | :--- |
| main_mpc.m | 전체 시뮬레이션의 메인 루프. 각 모듈을 호출하여 MPC를 수행하고 결과를 시각화함. | 논문 전체 알고리즘 흐름 담당 |
| get_continuous_dynamics.m | 연속시간 동역학 모델 생성. 로봇 물리 파라미터와 현재 상태로 Ac​,Bc​ 행렬 계산. | Eq.(16), (17) |
| discretize_zoh.m | 연속시간 동역학을 Zero-Order Hold 방식으로 이산화. (Ac​,Bc​→Ad​,Bd​) | Eq.(25), (26) |
| build_qp.m | QP(Quadratic Programming) 문제 구성. 비용함수 및 제약조건 행렬 생성. | Eq.(27) ~ (32) |
| gait_scheduler.m | 보행 패턴에 따라 각 발의 접촉 시퀀스(contact sequence) 생성. | Section IV-B |
| plan_foot_positions.m | 각 발의 목표 위치 및 COM→발 벡터 계산. Stance/Swing 구분 및 Heuristic 적용. | Section V |
| generate_reference_trajectory.m | 명령 속도에 따라 참조 궤적 생성. 상태별 추적값 반영. | Section IV-B |

### 실행 방법
MATLAB에서 모든 파일을 동일 폴더에 위치시킵니다.
main_mpc.m을 실행하면 전체 시뮬레이션이 동작하며, 결과 그래프와 성능 지표가 출력됩니다.
quadprog 함수는 Optimization Toolbox 필요
