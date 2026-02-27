## 파일 구조 및 역할
### 파일명	역할 및 설명
| 파일명 (File Name) | 역할 및 설명 | 관련 논문 근거 (Reference) |
| :--- | :--- | :--- |
| main_mpc.m | 전체 시뮬레이션의 메인 루프. 각 모듈을 호출하여 MPC를 수행하고 결과를 시각화함. | 논문 전체 알고리즘 흐름 담당 |
| get_continuous_dynamics.m | 연속시간 동역학 모델 생성. 로봇 물리 파라미터와 현재 상태로 Ac​,Bc​ 행렬 계산. | Eq.(16), (17) |
| discretize_zoh.m | 연속시간 동역학을 Zero-Order Hold 방식으로 이산화. (Ac​,Bc​→Ad​,Bd​) | Eq.(25), (26) |
| build_qp.m | QP 문제 구성. 비용함수 및 제약조건 행렬 생성. | Eq.(27) ~ (32) |
| gait_scheduler.m | 보행 패턴에 따라 각 발의 접촉 시퀀스(contact sequence) 생성. | Section IV-B |
| generate_reference_trajectory.m | 명령 속도에 따라 참조 궤적 생성. 상태별 추적값 반영. | Section IV-B |
| plan_foot_positions.m | 각 발의 목표 위치 및 COM→발 벡터 계산. Stance/Swing 구분 및 Heuristic 적용. | Section V |


### 실행 방법
MATLAB에서 모든 파일을 동일 폴더에 위치.  
main_mpc.m을 실행하면 전체 시뮬레이션이 동작하며, 결과 그래프와 성능 지표가 출력.  
quadprog 함수(QP solver)는 Optimization Toolbox 필요.

**Flow and Data Interfaces**

- **Entry point — main loop:** [Implements/main_mpc.m](main_mpc.m#L1) : 시뮬레이션 및 제어의 시작점. `params`, 초기 상태 `x`와 시뮬레이션 설정을 읽고 반복 루프를 돌며 각 모듈을 호출.
- **Gait scheduling:** [Implements/gait_scheduler.m](gait_scheduler.m#L1) : 입력 `t`, `dt_mpc`, `k_horizon`, `gait_type` → 출력 `contact_seq` (4×k_horizon). `main_mpc`가 매 반복마다 호출하여 향후 horizon에서의 접촉(스탠스/스윙)을 제공.
- **Foot placement planner:** [Implements/plan_foot_positions.m](plan_foot_positions.m#L1) : 입력 현재 상태 `x`, 명령 속도 `cmd_vel`, `contact_seq`, `params` → 출력 `r_feet` (3×4, COM→각 발 벡터) 및 `p_des` (3×4, 발 목표 위치). `r_feet`는 동역학(관성 및 선운동량 기여)을 계산할 때 `get_continuous_dynamics`에 전달.
- **Continuous dynamics (linearized):** [Implements/get_continuous_dynamics.m](get_continuous_dynamics.m#L1) : 입력 `psi`(yaw), `r_feet`, `params` → 출력 `Ac` (13×13), `Bc` (13×12). 이 연속행렬들은 MPC에서 상태-입력 선형 모델로 사용.
- **Discretization (ZOH):** [Implements/discretize_zoh.m](discretize_zoh.m#L1) : 입력 `Ac`, `Bc`, `dt_mpc` → 출력 `Ad`, `Bd`. `main_mpc`는 horizon 전체에 동일한 `Ad`, `Bd`를 복제(`Ad_list`, `Bd_list`)하여 예측 모델에 사용.
- **Reference trajectory:** [Implements/generate_reference_trajectory.m](generate_reference_trajectory.m#L1) : 입력 `x0`, `cmd_vel`, `k_horizon`, `dt_mpc` → 출력 `x_ref` (13·k_horizon × 1). 예측 horizon 동안의 목표 상태(위치·속도·yaw 등)를 생성.
- **QP construction:** [Implements/build_qp.m](build_qp.m#L1) : 입력 `Ad_list`, `Bd_list`, `x0`, `x_ref`, `contact_seq`, `Q_weights`, `alpha`, `params`, `k_horizon` → 출력 `H`, `g_vec`, `C_ineq`, `lb`, `ub`. 여기서 예측 모델과 가중치, 마찰/힘 제약을 모아 표준 QP 형태로 생성.
- **QP solve & apply:** `main_mpc`에서 `quadprog`로 `H`, `g_vec`, `C_ineq`, `lb`, `ub`를 풀어 최적 입력 `U_opt`(forces over horizon)을 획득. 그중 첫 12개(`u1`)를 실제로 적용(지면 반력)하고 상태를 `x_{k+1} = Ad*x_k + Bd*u1`로 업데이트.

이 흐름에서 각 모듈의 주요 데이터 전달 경로: `main_mpc` → (`gait_scheduler` → `contact_seq`) → (`plan_foot_positions` → `r_feet`, `p_des`) → (`get_continuous_dynamics` → `Ac,Bc`) → (`discretize_zoh` → `Ad,Bd`) → (`generate_reference_trajectory` → `x_ref`) → (`build_qp` → `H,g,C,lb,ub`) → (`quadprog` → `U_opt`) → 상태 업데이트 및 로그.
