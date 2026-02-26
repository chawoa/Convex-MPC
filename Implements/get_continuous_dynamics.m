function [Ac, Bc] = get_continuous_dynamics(psi, r_feet, params)
% 단순화 근거 (논문 Section III):
%   1. 다리 질량 무시 (전체의 10%)
%   2. roll/pitch 소각도 근사 → R = Rz(ψ)  [Eq.15]
%   3. ω×(Iω) = 0  [Eq.13]
%
% 입력:
%   psi    : 현재 yaw 각도 (rad)
%   r_feet : COM→발 벡터 (3x4)
%   params : 로봇 파라미터 구조체 (m, I_b)
%
% 출력:
%   Ac : 13x13 연속시간 상태행렬
%   Bc : 13x12 연속시간 입력행렬

m   = params.m;
I_b = params.I_b;
n_feet = 4;

% 논문 Eq.(15): I_world ≈ Rz * I_body * Rz'
Rz = [cos(psi) -sin(psi) 0;
      sin(psi)  cos(psi) 0;
      0         0        1];

I_world = Rz * I_b * Rz'; % 결과 3x3
I_inv   = I_world \ eye(3);  % 3x3 대각행렬을 통한 I_inv 계산 A \ B는 A * X = B가 되는 행렬을 구하는 방법

% ── Ac 행렬 (13x13) ──────────────────────────────────────
Ac = zeros(13, 13);
%  오일러각 변화율: Θ̇ = Rz'(ψ)·ω  [논문 Eq.12]
Ac(1:3,  7:9)  = Rz'; % 1:3 행, 7:9 열 자리에 들어감 논문과 동일
%  위치 변화율: ṗ = v
Ac(4:6, 10:12) = eye(3); % 4:6행, 10:12열 자리에 대각 행렬로 들어감 속도 값이 3x1 이므로 3x3 대각 행렬을 통해 pos(3x1)가 나오도록
%  중력 항: v̇z += g  (13번째 상태가 gravity -9.8)
Ac(10:12, 13)  = [0; 0; 1];

% ── Bc 행렬 (13x12) ──────────────────────────────────────
Bc = zeros(13, 3 * n_feet); % 입력 값이 12차원이므로
for i = 1:n_feet
    ci = (i-1)*3 + 1 : i*3; % 적용할 열을 정하기 위함 논문 형식처럼
    ri = r_feet(:, i); % r_feet(:, 1) ... r_feet(:, 4) 그러나 값이 3x1 벡터 이므로 skew가 필요

    % 각운동량 기여: ω̇ = I_inv*[ri]x*fi  논문 Eq.16
    Bc(7:9,  ci) = I_inv * skew(ri);

    % 선운동량 기여: v̇ = fi/m  논문 Eq.5
    Bc(10:12, ci) = eye(3) / m;
end

end

function S = skew(v)
% 벡터 v의 skew-symmetric 행렬, 외적을 행렬곱으로 연산하기 위해 변환
S = [ 0,    -v(3),  v(2);
      v(3),  0,    -v(1);
     -v(2),  v(1),  0   ];
end
