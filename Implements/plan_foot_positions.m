function [r_feet, p_des] = plan_foot_positions(x, cmd_vel, contact_seq, params)
% PLAN_FOOT_POSITIONS 발 위치 heuristic
%
% 논문 원문 (Section V):
%   p_des = p_ref + v_com * Δt/2
%   where Δt = time foot will spend on ground
%   p_ref = location on ground beneath hip of robot
%
% 입력:
%   x           : 현재 상태 [13x1] Θ, p, ω, v
%   cmd_vel     : 명령 속도 [vx; vy; yaw_rate] (world frame)
%   contact_seq : 4 x k_horizon contact sequence
%   params      : 로봇 파라미터 구조체
%
% 출력:
%   r_feet : COM에서 발까지의 벡터 (3x4), 상대 위치
%   p_des  : 발 목표 위치 (3x4), 절대 위치

% 현재 상태 파싱
roll  = x(1); pitch = x(2); yaw = x(3);
p_com = x(4:6);   % COM 위치
v_com = x(10:12); % COM 속도

% 회전 행렬 * world frame으로 월드기반 절대 좌표로 변환 가능
Rz = [cos(yaw) -sin(yaw) 0;
      sin(yaw)  cos(yaw) 0;
      0         0        1];

% ── hip 위치 정의 (body frame 기준) ──────────────────────────────────
% [FL, FR, RL, RR] 좌우 26cm, 앞뒤 60cm라는 가정하에
hip_body = [ 0.3,  0.3, -0.3, -0.3;  % 앞/뒤 (x)
             0.13, -0.13,  0.13, -0.13;  % 좌/우 (y)
             0.00,  0.00,  0.00,  0.00]; % 위/아래 (z) - hip joint 높이

% hip 위치 world frame 변환
hip_world = zeros(3, 4);
for i = 1:4
    hip_world(:,i) = p_com + Rz * hip_body(:,i);
end

% ── stance 시간 계산 ─────────────────────────────────────────────────
% 현재 timestep에서 각 발의 stance가 남은 시간 추정
dt_mpc   = params.dt_mpc;
k_horizon = size(contact_seq, 2);

T_stance = zeros(4, 1);
for leg = 1:4
    count = sum(contact_seq(leg, :));  % 앞으로 stance할 스텝 수
    T_stance(leg) = count * dt_mpc;
end

% ── 목표 발 위치 (논문 Eq.33) ────────────────────────────────────────
p_des = zeros(3, 4); % ri를 구하기 위함
for i = 1:4
    % v_com: 현재 velocity 사용
    % 논문: "p_des = p_ref + v_CoM * Δt/2"
    v_ref = [x(10); x(11); 0];  % xy 평면 속도만 사용

    p_des(:,i) = hip_world(:,i) + v_ref * T_stance(i) / 2;

    % 발은 지면에 닿아야 하므로 z = 0
    p_des(3,i) = 0;
end

% ── r_feet: COM → 발 벡터 (MPC에서 ri로 사용) COM으로 부터 각 발의 거리 ──────────────────────
% ri = foot_pos - COM_pos, 각 운동량을 구하기 위함
r_feet = zeros(3, 4);
for i = 1:4
    if contact_seq(i, 1) == 1
        % 지면에 있는 발의 위치 = 현재 목표 위치라고 가정
        r_feet(:,i) = p_des(:,i) - p_com;
    else
        % 공중에 있는 발: MPC에서 force=0이므로 값은 중요하지 않음
        % hip 아래 기본 위치로 설정
        r_feet(:,i) = hip_world(:,i) - p_com;
        r_feet(3,i) = -params.leg_length;
    end
end

end
