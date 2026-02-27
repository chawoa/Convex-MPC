# Convex-MPC

## CONTROL ARCHITECTURE
Swing Leg Control과 Ground Force Control  
### Swing Leg Control
공중에 있을때의 다리를 목표 궤적으로 그리며 날아가도록 관절 모터의 힘을 계산.  
피드백 + 피드 포워드로 구성.  
$$\tau_i = J_i^\top [K_p(^B p_{i,\text{ref}} - ^B p_i) + K_d(^B v_{i,\text{ref}} - ^B v_i)] + \tau_{i,\text{ff}}$$  
  
- 피드백(대괄호): 목표 위치/속도($\text{ref}$)와 현재 위치/속도의 차이(오차)를 계산. ($K_p$는 강성, $K_d$는 감쇠력)  
- 자코비안 ($J_i^\top$): 발끝에서 내야 하는 3차원 직선 방향의 힘을 각 관절(고관절, 무릎 등)의 회전력(토크)으로 변환.  
    오일러 각 변화량($\dot{\Theta} = [\dot{\phi}, \dot{\theta}, \dot{\psi}]^\top$)은 축이 3번 연속으로 꺾이면서(Z축 돌고, Y축 돌고, X축 돌고) 측정되는 값. 반면 각속도($\omega$)는 고정된 절대 공간(World)에서의 회전 속도입니다.따라서 비틀어진 축 기준의 속도($\dot{\Theta}$)를 절대 공간 기준의 속도($\omega$)로 변환해 주는 역할의 행렬.  
- 피드포워드 토크 ($\tau_{i,\text{ff}}$): 다리 자체의 무게, 관성, 다리를 휘두를 때 생기는 원심력 힘 등을 수학 모델을 통해 미리 계산.  
- 동적 강성 조절: 로봇 다리가 펴져 있을 때와 굽혀져 있을 때, 모터가 느끼는 다리의 관성이 달라짐. 이 식을 통해 $$K_{p,i} = \omega_i^2 \Lambda_{i,i}$$ 다리의 자세가 변해서 관성($\Lambda_{i,i}$)이 변하더라도, 다리 제어의 고유 진동수($\omega_i$, 즉 반응하는 템포)가 일정하게 유지되도록 강성($K_p$)을 실시간으로 조절.  

### Ground Force Control
발이 땅에 닿아 있는 동안, 로봇의 무거운 몸통을 지탱하고 원하는 방향으로 밀어내기 위해 관절 토크를 계산.  
$$\tau_i = J_i^\top R_i^\top f_i$$  
  
- $f_i$ (목표 지면 반력): 로봇 몸통의 밸런스를 잡고 앞으로 나아가기 위해 '발끝이 땅을 얼마나 세게 밀어야 하는가'를 나타내는 3차원 힘 벡터.  
- $R_i^\top$: 로봇 몸통 기준의 방향을 절대 세계(World) 좌표계로 변환하는 회전 행렬.  
- $J_i^\top$ (자코비안): 스윙 페이즈와 마찬가지로, 발끝에서 발생해야 하는 3차원 힘 벡터($f_i$)를 각 다리 관절 모터가 내야 할 회전력($\tau_i$)으로 변환.  
  
## SIMPLIFIED ROBOT DYNAMICS
복잡한 계산을 실시간으로 처리하기 위해 로봇을 Single Rigid Body로 단순화합니다. (다리 무게는 전체에 미치는 영향이 적어 무시)    
### 기본 법칙  
- 위치 이동: $\ddot{p} = \frac{\sum f_i}{m} - g$  
    (가속도 = 힘/질량 - 중력. 로봇 몸통이 지면 반력 $f_i$를 받아 앞뒤좌우로 움직이는 법칙), 지면 반력으로 위치 이동을 하기 때문.  
- 회전: $\frac{d}{dt}(I\omega) = \sum r_i \times f_i$  
    $f_i$ (지면 반력): 발끝이 땅을 미는 힘 (예: 10kg의 힘으로 땅을 참).  
    $r_i$ (모멘트 암): 로봇의 무게중심(COM)에서 발끝(힘이 작용하는 곳)까지의 3차원 거리 벡터.  
    $r_i \times f_i$ (외적, Cross Product): 거리와 힘을 곱해 '토크(회전력)'를 생성 (예: 문을 열 때, 문의 경첩(무게중심)에서 손잡이까지의 거리($r_i$)가 멀수록, 그리고 손으로 미는 힘($f_i$)이 셀수록 문이 더 잘 도는(토크가 커지는) 것과 같은 원리  
    $I$ (관성 텐서): 물체가 회전하기 싫어하는 정도(무게와 모양에 따라 다름)  
    $\omega$ (각속도): 로봇이 도는 속도  
    $I\omega$ (각운동량): 회전하는 물체가 가진 힘
    
- 각도 변화: $$\dot{R} = [\omega]_\times R$$  
    $R$ (회전 행렬): 로봇이 현재 월드 좌표계(절대 세계)에서 어느 방향으로 비틀어져 있는지를 나타내는 3x3 행렬.
    $$[ω]_×$$ (외적 행렬, Skew-symmetric matrix): 각속도 벡터 $\omega = [\omega_x, \omega_y, \omega_z]^\top$를 3×3 행렬 모양으로 변환한 행렬. (외적을 하기 위함)
  
### Approximated Angular Velocity Dynamics
- 로봇은 심하게 기울어지지 않음 (방정식 8 ~ 12)  
    방정식 9와 10은 각속도($\omega$)와 오일러 각의 변화량($\dot{\Theta}$)을 변환하는 식.  
    여기서 보행 로봇이 바닥과 평행하게 걷는다고 가정하기 때문에 Roll과 Pitch는 0에 가깝게 아주 작다고 가정.   
    따라서 $\sin(0) = 0$, $\cos(0) = 1$, 방정식 (11)과 (12)처럼 Yaw($\psi$) 하나만 남은 아주 단순한 곱셈 식으로 변환.  
  
- 로봇의 회전이 빠르지 않음 (방정식 13)  
    방정식 (6)의 원본을 풀면 $\frac{d}{dt}(I\omega) = I\dot{\omega} + \omega \times (I\omega)$  
    여기서 뒤쪽의 $\omega \times (I\omega)$는 회전이 빠를때 생기는 힘.   
    로봇이 걷거나 뛸 때는 이 값이 0에 가까움.  
    따라서 $I\dot{\omega}$ 로 변환.  

- 관성 모멘트도 Yaw만 고려 (방정식 14, 15)  
    로봇이 기울어질 때마다 관성(회전하기 힘든 정도, $I$) 행렬이 변하는데 (방향 등이 변함)  
    이또한 Roll과 Pitch가 작다고 가정하고, Yaw 회전($$R_z(\psi)$$)에 대한 관성만 계산.  
  
### Simplified Robot Dynamics
행렬 합체 (방정식 16, 17)  
현재 상태 + 입력으로 인한 상태 변화 + 중력 = 예측된 상태  
방정식 16을 요약하면 다음과 같은 형태.  
$\frac{d}{dt} x_{old} = A_{old} x_{old} + B u + G$  
여기서 각 항목의 크기(차원)는 다음과 같습니다.  
$x_{old}$: 크기가 $12 \times 1$ 인 기존 상태 벡터 (각도, 위치, 속도 등)  
$A_{old}$: 크기가 $12 \times 12$ 인 시스템 행렬.  
$G$: 크기가 $12 \times 1$ 인 중력 벡터.  
  
아래와 같은 과정을 거쳐 방정식 17로 변형.  
항상 값이 1인 가상의 상태를 하나 추가해서 13차원의 새로운 상태 벡터 $x_{new}$를 생성.  
```math
x_{new} = \begin{bmatrix} x_{old} \\ 1 \end{bmatrix}
```  
이때, 상수인 1을 시간에 대해 미분하면 0이 되므로, 좌변의 미분 벡터는 다음과 같이 변경.  
```math
\frac{d}{dt} x_{new} = \begin{bmatrix} \dot{x_{old}} \\ 0 \end{bmatrix}
```  
행렬 재배치: 중력을 A행렬 안으로 병합. 이제 늘어난 상태 벡터($13 \times 1$)에 곱해질 수 있도록, $$A_{old}$$ 행렬($12 \times 12$)의 크기를 $13 \times 13$으로 변형.  
```math
A_{new} = \begin{bmatrix} A_{old} & G \\ 0 \cdots 0 & 0 \end{bmatrix}
```  
결과적으로 아래와 같은 식으로 정리  
$\frac{d}{dt} x_{new} = A_{new} x_{new} + B u$    
```math
\frac{d}{dt} x_{new} = \begin{bmatrix} A_{old} & G \\ 0 \cdots 0 & 0 \end{bmatrix} \begin{bmatrix} x_{old} \\ 1 \end{bmatrix} + B u
```  
  
$x(t)$ (상태): 로봇의 현재 기울기, 위치, 속도 등 (결과)  
$u(t)$ (입력): 4개의 발끝에서 땅을 밀어내는 힘($f_1, f_2, f_3, f_4$) (원인)  
$A_c$, $B_c$ (시스템 행렬): 로봇이 현재 바라보는 방향(Yaw, $\psi$)과 발 딛는 위치($r_i$)에만 영향을 받는 행렬.  

## MODEL PREDICTIVE CONTROL  
### Force Constraints
- 방정식 22 ($f_{\text{min}} \le f_z \le f_{\text{max}}$): Z축(위아래) 힘의 한계.  
    로봇 발은 땅을 밀어낼 수만 있지, 땅을 잡고 당길 수는 없음. (최소 힘 $\ge 0$). 또한 모터가 낼 수 있는 최대 힘 한계도 존재.  
- 방정식 23, 24 (마찰): X, Y 방향으로 미는 힘($f_x, f_y$)은 누르는 힘($f_z$)에 마찰 계수($\mu$)를 곱한 것보다 작아야 발이 미끄러지지 않음. 이를 컴퓨터가 계산하기 쉽게 원뿔 모양이 아닌 Square pyramid 형태의 선형 부등식으로 단순화.  
  
### Reference Trajectory Generation
$x, y$ 속도와 Yaw(회전)만 명령을 따르며, Roll, Pitch, $Z$ 속도는 무조건 0으로 세팅.  

### Linear Discrete Time Dynamics
연속 -> 이산  
$A_c, B_c$ -> $\hat{A}, \hat{B}$  
  
### QP Formulation
MPC의 핵심은 컴퓨터가 가장 빨리 풀 수 있는 이차 계획법(Standard QP: $\min \frac{1}{2} U^\top H U + U^\top g$) 형태로 문제를 변환하는 것.
- 상태(X)를 소거 (방정식 27):   
    아래와 같이 전개하면, 미래의 모든 상태($X$)는 굳이 변수로 둘 필요 없이 초기 상태($x_0$)와 미래의 힘($U$)들만의 곱셈으로 치환 가능.  
    $$x_1 = Ax_0 + Bu_0$$  
    $$x_2 = A(Ax_0 + Bu_0) + Bu_1 = A^2x_0 + ABu_0 + Bu_1$$  
    $$x_3 = A^3x_0 + A^2Bu_0 + ABu_1 + Bu_2$$  
    이를 통해 $A_{qp}$ 행렬과 $B_{qp}$ 행렬 생성.  
- 비용 함수(Cost Function) 정리:  
    목표 궤적과의 오차를 줄이고 에너지($U$) 소모를 최소화하는 식(방정식 28)으로 전개  
    ```math
      J(U) = \|A_{qp}x_0 + B_{qp}U - y\|_L^2 + \|U\|_K^2
    ```
    방정식 28을 전개한 후 정리하면, 방정식 29의 Standard QP (Quadratic Programming) 폼인 $\frac{1}{2} U^\top H U + U^\top g$ 형태가 생성.  
    2차항 ($U^2$ 항):  
    ```math
      U^\top (B_{qp}^\top L B_{qp} + K) U
    ```
    1차항 ($U$ 항):  
    ```math
      U^\top (2 B_{qp}^\top L (A_{qp}x_0 - y))
    ```  
    여기서 최적화(미분해서 0이 되는 지점 찾기)를 할 때 상수항은 어차피 사라지므로 버림.  
    아재 다시 방정식 29와 같은 형태로 정리하면, $$\min \frac{1}{2} U^\top H U + U^\top g$$  
    방정식 31 ($H$ 행렬): $2차항 계수 = \frac{1}{2} H \implies \mathbf{H = 2(B_{qp}^\top L B_{qp} + K)}$  
    방정식 32 ($g$ 벡터): $1차항 계수 = g \implies \mathbf{g = 2 B_{qp}^\top L (A_{qp}x_0 - y)}$  
