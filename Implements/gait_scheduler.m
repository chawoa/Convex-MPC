function contact_seq = gait_scheduler(t, dt_mpc, k_horizon, gait_type)
% 논문 Section IV-B의 contact sequence 생성 (논문에서 1=지면, 0=공중) 각 발에 대해 생성
%
% 입력:
%   t          : 현재 시간
%   dt_mpc     : MPC timestep
%   k_horizon  : prediction horizon 길이
%   gait_type  : 'trot', 'stand', 'bound', 'pace'
%
% 출력:
%   contact_seq : 4 x k_horizon (1=지면, 0=공중)
%                 발 순서: [FL, FR, RL, RR]

contact_seq = zeros(4, k_horizon);

switch gait_type
    case 'stand'
        % 모든 발 지면 접촉
        contact_seq = ones(4, k_horizon);
    case 'trot'
        % Trot: 대각선 발 쌍이 교대로 접촉
        % FL+RR 동시, FR+RL 동시
        T_gait   = 0.5;    % 1 gait cycle 보행 사이클 공중 시간 + 지면 시간
        T_stance = 0.25;   % 한 발이 지면에 있는 시간 (duty = 0.5)

        % 발 위상 오프셋 (gait cycle 비율)
        phase_offset = [0.0;  % FL
                        0.5;  % FR
                        0.5;  % RL
                        0.0]; % RR

        for k = 1:k_horizon
            t_k = t + (k-1) * dt_mpc;
            for leg = 1:4
                phase = mod(t_k / T_gait + phase_offset(leg), 1.0);
                if phase < (T_stance / T_gait)
                    contact_seq(leg, k) = 1;
                else
                    contact_seq(leg, k) = 0;
                end
            end
        end
        
    case 'bound'
        % 논문: bound는 2.5 m/s 달성, gait freq 3.3 Hz
        T_gait   = 1/3.3;  % ≈ 0.30 sec
        T_stance = 0.09;   % 90 ms stance

        phase_offset = [0.0;   % FL
                        0.0;   % FR
                        0.5;   % RL
                        0.5];  % RR

        for k = 1:k_horizon
            t_k = t + (k-1) * dt_mpc;
            for leg = 1:4
                phase = mod(t_k / T_gait + phase_offset(leg), 1.0);
                if phase < (T_stance / T_gait)
                    contact_seq(leg, k) = 1;
                else
                    contact_seq(leg, k) = 0;
                end
            end
        end
end

end
