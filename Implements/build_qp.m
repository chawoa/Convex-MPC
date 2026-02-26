function [H, g_vec, C_ineq, lb, ub] = build_qp(Ad_list, Bd_list, ...
    x0, x_ref, contact_seq, Q_weights, alpha, params, k_horizon)
%
%
% 입력:
%   Ad_list, Bd_list : k_horizon 길이 cell array
%   x0              : 현재 상태 (13x1)
%   x_ref           : 참조 궤적 (13*k_horizon x 1)
%   contact_seq     : 4 x k_horizon
%   Q_weights       : 상태 가중치 (1x13)
%   alpha           : force 가중치 (scalar)
%   params          : 로봇 파라미터 (mu, fmin, fmax)
%   k_horizon       : horizon 길이
%
% 출력:
%   H, g_vec : QP 비용함수 행렬
%   C_ineq   : 부등식 제약 행렬 (C·U <= ub)
%   lb, ub   : 변수 하한/상한 및 제약 상한

n_s = 13;
n_u = 12; 

mu   = params.mu;
fmin = params.fmin;
fmax = params.fmax;

%% ── Aqp, Bqp 구성 (논문 Eq.27) ──────────────────────────
% X = Aqp·x0 + Bqp·U
% Aqp[i] = A[i]·A[i-1]·...·A[1]
% Bqp[i,j] = A[i]·...·A[j+1]·B[j]  (j <= i)

Aqp = zeros(n_s * k_horizon, n_s);
Bqp = zeros(n_s * k_horizon, n_u * k_horizon); % 입력이 k_horizon 갯수 만큼 나오기 때문

% Aqp 계산 (순차 곱)
A_pow = eye(n_s);
A_powers = cell(1, k_horizon);
for i = 1:k_horizon
    A_pow = Ad_list{i} * A_pow;
    A_powers{i} = A_pow;
    Aqp((i-1)*n_s+1:i*n_s, :) = A_pow;
end

% Bqp 계산
for i = 1:k_horizon
    for j = 1:i
        row = (i-1)*n_s+1 : i*n_s;
        col = (j-1)*n_u+1 : j*n_u;
        if j == i
            Bqp(row, col) = Bd_list{j}; % 가장 최근의 입력에 대해서는 B만 넣어주는 형태
        else
            % A[i]·...·A[j+1] = A_powers[i] / A_powers[j]
            A_prod = eye(n_s);
            for k = j+1:i
                A_prod = Ad_list{k} * A_prod;
            end
            Bqp(row, col) = A_prod * Bd_list{j};
        end
    end
end

%% ── 가중치 행렬 (논문 Eq.31, 32) ────────────────────────
L = kron(eye(k_horizon), diag(Q_weights));  % 13x13
K = alpha * eye(n_u * k_horizon);           % 12x12

% H = 2(Bqp'·L·Bqp + K)
H = 2 * (Bqp' * L * Bqp + K);
H = (H + H') / 2;  % 수치 대칭성 보장 (경고로 인한 추가)

% g = 2·Bqp'·L·(Aqp·x0 - x_ref)
g_vec = 2 * Bqp' * L * (Aqp * x0 - x_ref);

%% ── 변수 경계 초기화 ─────────────────────────────────────
lb_u = -inf(n_u * k_horizon, 1);
ub_u =  inf(n_u * k_horizon, 1);

%% ── 제약조건 구성 (논문 Eq.21~24) ───────────────────────
%    총 6개의 제약
%    -fz ≤ -fmin   (fz ≥ fmin)
%    fz ≤  fmax
%    fx - μ·fz ≤ 0
%   -fx - μ·fz ≤ 0
%    fy - μ·fz ≤ 0
%   -fy - μ·fz ≤ 0
% 공중 발: force = 0 (lb=ub=0)

n_stance = 0;
for i = 1:k_horizon
    n_stance = n_stance + sum(contact_seq(:,i));
end

C_ineq = zeros(6 * n_stance, n_u * k_horizon);
ub     = zeros(6 * n_stance, 1);

row_c = 1;
for i = 1:k_horizon
    for j = 1:4
        base = (i-1)*n_u + (j-1)*3 + 1;
        fx_idx = base;
        fy_idx = base + 1;
        fz_idx = base + 2;

        if contact_seq(j, i) == 0
            % 공중 발: force 0 강제 (논문 Eq.21)
            lb_u(base:base+2) = 0;
            ub_u(base:base+2) = 0;
        else
            % 지면 발: 마찰 제약 (논문 Eq.22~24)

            % fz ≥ fmin  →  -fz ≤ -fmin
            C_ineq(row_c, fz_idx) = -1;
            ub(row_c) = -fmin;

            % fz ≤ fmax
            C_ineq(row_c+1, fz_idx) = 1;
            ub(row_c+1) = fmax;

            % fx ≤ μ·fz  →  fx - μ·fz ≤ 0
            C_ineq(row_c+2, [fx_idx, fz_idx]) = [1, -mu];

            % -fx ≤ μ·fz  →  -fx - μ·fz ≤ 0
            C_ineq(row_c+3, [fx_idx, fz_idx]) = [-1, -mu];

            % fy ≤ μ·fz
            C_ineq(row_c+4, [fy_idx, fz_idx]) = [1, -mu];

            % -fy ≤ μ·fz
            C_ineq(row_c+5, [fy_idx, fz_idx]) = [-1, -mu];

            row_c = row_c + 6;
        end
    end
end

% 실제 사용된 행만 유지
C_ineq = C_ineq(1:row_c-1, :);
ub     = ub(1:row_c-1);

% quadprog의 lb (변수 하한)
lb = lb_u;

end
