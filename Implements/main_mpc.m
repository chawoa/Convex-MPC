% main_mpc.m

clc; clear; close all;

%  1. 로봇 파라미터 (논문 Table I)────────────────────────────────────
params.m          = 43; % 질량
params.I_b        = diag([0.41, 2.1, 2.1]); % 각 축에 대한 관성
params.mu         = 0.6; % 마찰 계수
params.g          = -9.8; % 중력 가속도
params.fmin       = 10; % 최소 출력
params.fmax       = 666; % 최대 출력
params.leg_length = 0.5;   % 다리 길이 (m)

%  2. MPC 설정────────────────────────────────────
dt_mpc    = 1/30;   % MPC 주기 30Hz
k_horizon = 10;     % prediction horizon 몇 스텝을 뽑아낼 건지
params.dt_mpc = dt_mpc;

% 상태 가중치 Q (논문 Table I)
% [roll, pitch, yaw, px, py, pz, wx, wy, wz, vx, vy, vz, g]
Q_weights = [1,  1,  1, ...   % Θ: roll, pitch, yaw 동일하게 판단
             1,  1,  50, ...  % p: px, py, pz (z 높이 강조)
             0,  0,  1, ...   % ω: yaw rate만 추적
             1,  1,  0, ...   % v: vx, vy
             0];              % gravity state

alpha = 1e-6;  % force 가중치 값이 작을수록 force를 자유롭게 쓰고, 값이 크면 force 사용을 억제하려는 경향 QP의 목적식은 상태 오차 + force 크기를 최소화하기 때문

%  3. 속도 명령 (운영자 입력)────────────────────────────────────
% [vx_cmd (m/s), vy_cmd (m/s), yaw_rate_cmd (rad/s)]
% 현재 시나리오: 0→1초 제자리, 1→4초 전진 0.5 m/s, 4→6초 전진+좌회전

sim_time = 6.0; % 시뮬레이션 시간
N_steps  = floor(sim_time / dt_mpc); % 시뮬레이션 시간 동안 mpc가 몇 회 반복되는지

gait_type = 'bound';  % 'stand', 'trot', 'bound' 어떻게 보행할 것인지 gait_scheduler와 연동

%  4. 초기 상태────────────────────────────────────
% x = [roll, pitch, yaw, px, py, pz, wx, wy, wz, vx, vy, vz, g] 13
x = [0; 0; 0;          % Θ 자세
     0; 0; 0.5;        % p 위치 (pz=0.5m 높이에서 시작) 왜냐하면 다리 길이가 0.5m이기 때문
     0; 0; 0;          % ω 각속도-회전속도
     0; 0; 0;          % v 선속도-이동 속도
     -9.8];            % g

%  5. 로깅 변수────────────────────────────────────
X_log        = zeros(13, N_steps+1);
X_log(:, 1)  = x;
CMD_log      = zeros(3,  N_steps);
Contact_log  = zeros(4,  N_steps);
Force_log    = zeros(12, N_steps);
t_log        = (0:N_steps-1) * dt_mpc;

%  6. 메인 제어 루프────────────────────────────────────
fprintf('시뮬레이션 시작: %.1f초, %d 스텝\n', sim_time, N_steps);

for step = 1:N_steps
    t = (step-1) * dt_mpc; % 현재 시간

    % 6-1. 속도 명령 ────────────────────────────────────
    cmd_vel = get_cmd_vel(t);  % [vx; vy; yaw_rate] 어떤 방향으로 속도를 내야하는지 + 회전 속도, 현재 로컬 함수로 조작
    CMD_log(:, step) = cmd_vel;

    % 6-2. Gait Scheduler ───────────────────────────────
    contact_seq = gait_scheduler(t, dt_mpc, k_horizon, gait_type); % 선택한 보행 기법에 따라 어느 발이 지면과 접촉하는지에 대한 contact 시퀀스 생성
    Contact_log(:, step) = contact_seq(:, 1);

    % 6-3. 발 위치 계획 (논문 Eq.33) ───────────────────
    [r_feet, p_des] = plan_foot_positions(x, cmd_vel, contact_seq, params); % r_feet는 로봇의 중심(COM)으로 부터 각 발의 상대 위치, p_des는 각 발의 목표 위치를 계산

    % 6-4. 연속 dynamics → 이산화 ──────────────────────
    psi = x(3);  % 현재 yaw
    [Ac, Bc] = get_continuous_dynamics(psi, r_feet, params);
    [Ad, Bd] = discretize_zoh(Ac, Bc, dt_mpc);

    % horizon 전체에 같은 Ad, Bd 사용 horizon 내에서 동역학 행렬을 일정하게 가정
    Ad_list = repmat({Ad}, 1, k_horizon);
    Bd_list = repmat({Bd}, 1, k_horizon);

    % 6-5. 참조 궤적 생성 ──────────────────────────────
    x_ref = generate_reference_trajectory(x, cmd_vel, k_horizon, dt_mpc);

    % 6-6. QP 구성 및 풀기 ─────────────────────────────
    [H, g_vec, C_ineq, lb, ub] = build_qp(Ad_list, Bd_list, ...
        x, x_ref, contact_seq, Q_weights, alpha, params, k_horizon);

    % quadprog: min 0.5*x'Hx + g'x  s.t. C*x<=ub, lb<=x
    opts = optimoptions('quadprog', 'Display', 'off', ...
                        'Algorithm', 'interior-point-convex');

    [U_opt, ~, flag] = quadprog(sparse(H), g_vec, ...
                                sparse(C_ineq), ub, ...
                                [], [], lb, [], [], opts);

    if flag ~= 1
        fprintf('QP 실패 이전 입력 유지\n'); % 디버깅 및 안정성을 위함
        if step > 1
            U_opt = [Force_log(:, step-1); zeros(12*(k_horizon-1), 1)];
        else
            U_opt = zeros(12*k_horizon, 1);
        end
    end

    % 6-7. 첫 번째 timestep force 적용 ─────────────────
    u1 = U_opt(1:12);
    Force_log(:, step) = u1;

    % 6-8. 상태 업데이트 ───────
    x = Ad * x + Bd * u1;
    % 외란 테스트 코드
    if step == floor(3.0 / dt_mpc)
        x(11) = x(11) + 2.0; % vy 속도(옆방향 속도)를 강제로 증가
    end
    X_log(:, step+1) = x;
end

fprintf('시뮬레이션 완료.\n');

%  7. 결과 시각화
t_plot = [t_log, t_log(end)+dt_mpc];  % X_log에 맞게 N_steps+1

figure('Name', 'CONVEX-MPC 시뮬레이션 결과', 'Position', [100 100 1200 800]);

% (1) 속도 추적
subplot(3,3,1);
plot(t_plot, X_log(10,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_log,  CMD_log(1,:), 'r--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('vx (m/s)');
title('X 속도 추적'); legend('실제','명령'); grid on;

subplot(3,3,2);
plot(t_plot, X_log(11,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_log,  CMD_log(2,:), 'r--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('vy (m/s)');
title('Y 속도 추적'); legend('실제','명령'); grid on;

subplot(3,3,3);
plot(t_plot, rad2deg(X_log(9,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(t_log,  rad2deg(CMD_log(3,:)), 'r--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('yaw rate (deg/s)');
title('Yaw Rate 추적'); legend('실제','명령'); grid on;

% (2) 자세 (orientation)
subplot(3,3,4);
plot(t_plot, rad2deg(X_log(1,:)), 'r-', 'LineWidth', 1.5); hold on;
plot(t_plot, rad2deg(X_log(2,:)), 'g-', 'LineWidth', 1.5);
plot(t_plot, rad2deg(X_log(3,:)), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Angle (deg)');
title('자세 (Euler Angles)'); legend('Roll','Pitch','Yaw'); grid on;

% (3) 위치
subplot(3,3,5);
plot(t_plot, X_log(4,:), 'r-', 'LineWidth', 1.5); hold on;
plot(t_plot, X_log(5,:), 'g-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Position (m)');
title('수평 위치'); legend('px','py'); grid on;

subplot(3,3,6);
plot(t_plot, X_log(6,:), 'b-', 'LineWidth', 1.5); hold on;
yline(0.5, 'r--', '목표 높이', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('pz (m)');
title('높이 제어 (pz)'); grid on; ylim([0, 1.0]);

% (4) Ground Reaction Forces
subplot(3,3,7);
leg_names = {'FL','FR','RL','RR'};
colors = {'r','b','g','k'};
for i = 1:4
    fz_idx = i*3;  % z-force index
    plot(t_log, Force_log(fz_idx,:), colors{i}, 'LineWidth', 1.2); hold on;
end
xlabel('Time (s)'); ylabel('Fz (N)');
title('Ground Reaction Force (Z)'); legend(leg_names); grid on;

% (5) Contact sequence
subplot(3,3,8);
imagesc(t_log, 1:4, Contact_log);
colormap([1 1 1; 0.2 0.6 0.2]);  % 흰=공중, 초록=지면
yticks(1:4); yticklabels(leg_names);
xlabel('Time (s)'); title('Contact Sequence (초록=지면)');
colorbar('Ticks',[0,1],'TickLabels',{'Swing','Stance'});

% (6) XY 궤적
subplot(3,3,9);
plot(X_log(4,:), X_log(5,:), 'b-', 'LineWidth', 2);
hold on; plot(X_log(4,1), X_log(5,1), 'go', 'MarkerSize', 8, 'LineWidth', 2);
plot(X_log(4,end), X_log(5,end), 'rs', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('px (m)'); ylabel('py (m)');
title('XY 궤적'); legend('궤적','시작','끝'); grid on; axis equal;

sgtitle(sprintf('MPC - %s gait, vx=%.1f m/s', ...
        gait_type, max(abs(CMD_log(1,:)))));

% 8. 성능 지표 출력
fprintf('\n═══ 성능 지표 ═══\n');
fprintf('최대 vx:    %.3f m/s\n', max(X_log(10,:)));
fprintf('최대 |roll|: %.2f deg\n', max(abs(rad2deg(X_log(1,:)))));
fprintf('최대 |pitch|:%.2f deg\n', max(abs(rad2deg(X_log(2,:)))));
fprintf('pz 평균:    %.3f m (목표: 0.5 m)\n', mean(X_log(6,2:end)));
fprintf('pz 표준편차:%.4f m\n', std(X_log(6,2:end)));

% 함수: 속도 명령 스케줄 ────────────────────────────────────
function cmd = get_cmd_vel(t)
% 시간에 따른 명령 속도 [vx; vy; yaw_rate]
    if t < 1.0
        cmd = [0.0; 0.0; 0.0];       % 정지
    elseif t < 4.0
        cmd = [0.5; 0.0; 0.0];       % 전진 0.5 m/s
    elseif t < 5.0
        cmd = [0.5; 0.0; 0.5];       % 전진 + 좌회전
    else
        cmd = [0.0; 0.0; 0.0];       % 정지
    end
end
