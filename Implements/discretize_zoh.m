function [Ad, Bd] = discretize_zoh(Ac, Bc, dt)
% Zero-Order Hold 이산화
%
% 입력:
%   Ac : n×n 연속시간 상태행렬
%   Bc : n×m 연속시간 입력행렬
%   dt : timestep (sec)
%
% 출력:
%   Ad : n×n 이산시간 상태행렬
%   Bd : n×m 이산시간 입력행렬

n = size(Ac, 1); % 13
m = size(Bc, 2); % 12

% 확장 행렬 구성
M  = [Ac, Bc; zeros(m, n+m)]; % M = [[Ac, Bc], [0, 0]]
eM = expm(M * dt);

Ad = eM(1:n, 1:n);
Bd = eM(1:n, n+1:n+m);

end
