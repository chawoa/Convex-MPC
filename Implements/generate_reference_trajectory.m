function x_ref = generate_reference_trajectory(x0, cmd_vel, k_horizon, dt_mpc)
%
% xy-velocity, xy-position, z position, yaw, yaw rate만 값을 가짐
%
% 입력:
%   x0        : 현재 상태 [13x1]
%   cmd_vel   : 명령 속도 [vx_cmd; vy_cmd; yaw_rate_cmd]
%   k_horizon : prediction horizon 길이
%   dt_mpc    : MPC timestep (sec)
%
% 출력:
%   x_ref : 13*k_horizon x 1 참조 궤적 벡터

% 명령 속도 파싱
vx_cmd       = cmd_vel(1);
vy_cmd       = cmd_vel(2);
yaw_rate_cmd = cmd_vel(3);

% 현재 상태에서 초기값 추출
yaw_0 = x0(3);
px_0  = x0(4);
py_0  = x0(5);

% 목표 높이 ~0.5m
pz_ref = 0.5;

x_ref = zeros(13 * k_horizon, 1);

yaw_k = yaw_0;
px_k  = px_0;
py_k  = py_0;

for k = 1:k_horizon
    % yaw 적분
    % 이산적으로 처리를 했기 때문에 적분을 하게 되면 아래와 같이 구현 가능
    yaw_k = yaw_k + yaw_rate_cmd * dt_mpc;

    % xy 위치 적분 (world frame)
    % 이산적으로 처리를 했기 때문에 적분을 하게 되면 아래와 같이 구현 가능
    px_k = px_k + vx_cmd * dt_mpc;
    py_k = py_k + vy_cmd * dt_mpc;

    % 참조 상태 벡터 구성 (13개 요소)
    % [roll, pitch, yaw, px, py, pz, wx, wy, wz, vx, vy, vz, g]
    x_k = zeros(13, 1);
    x_k(1) = 0;           % roll = 0
    x_k(2) = 0;           % pitch = 0
    x_k(3) = yaw_k;       % yaw 추적
    x_k(4) = px_k;        % x 위치
    x_k(5) = py_k;        % y 위치
    x_k(6) = pz_ref;      % z 높이 유지
    x_k(7) = 0;           % roll rate = 0
    x_k(8) = 0;           % pitch rate = 0
    x_k(9) = yaw_rate_cmd; % yaw rate 추적
    x_k(10) = vx_cmd;     % vx 추적
    x_k(11) = vy_cmd;     % vy 추적
    x_k(12) = 0;          % vz = 0
    x_k(13) = -9.8;       % gravity

    idx = (k-1)*13 + 1 : k*13;
    x_ref(idx) = x_k;
end

end
