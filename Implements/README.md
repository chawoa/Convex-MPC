파일 구조 및 역할
파일명	역할 및 설명
main_mpc.m	전체 시뮬레이션의 메인 루프. 각 모듈을 호출하여 MPC를 수행하고 결과를 시각화함. 논문 전체 알고리즘의 흐름을 담당.
get_continuous_dynamics.m	논문 Eq.(16)(17) 기반의 연속시간 동역학 모델 생성. 로봇의 물리 파라미터와 현재 상태를 받아 Ac, Bc 행렬을 계산.
discretize_zoh.m	논문 Eq.(25)(26) 기반, 연속시간 동역학을 Zero-Order Hold 방식으로 이산화. Ac, Bc → Ad, Bd 변환.
build_qp.m	논문 Eq.(27)~(32) 기반, QP(Quadratic Programming) 문제를 구성. 비용함수, 제약조건 행렬 생성.
gait_scheduler.m	논문 Section IV-B 기반, 보행(gait) 패턴에 따라 각 발의 contact sequence를 생성. 다양한 gait 지원.
plan_foot_positions.m	논문 Section V 기반, 각 발의 목표 위치 및 COM→발 벡터 계산. stance/swing 구분 및 heuristic 적용.
generate_reference_trajectory.m	논문 Section IV-B 기반, 명령 속도에 따라 참조 궤적 생성. 상태별로 추적할 값만 반영.

실행 방법
MATLAB에서 모든 파일을 동일 폴더에 위치시킵니다.
main_mpc.m을 실행하면 전체 시뮬레이션이 동작하며, 결과 그래프와 성능 지표가 출력됩니다.
별도의 외부 toolbox 없이 동작합니다. (단, quadprog 함수는 Optimization Toolbox 필요)