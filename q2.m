% =========================================================================
% Vehicle Steering System - Analytical Controller & Observer Design
% Reference: Astrom & Murray (AM09), Examples 6.4 & 7.3
% =========================================================================
clear; clc; close all;

% 1. System Parameters
gamma = 0.5; % Ratio of distance to center of mass vs wheelbase
A = [0 1; 
    0 0];
B = [gamma; 
    1];
C = [1 0];   
D = 0;

% 2. Controller Design (Using Analytical Equations from Text)
% Define desired closed-loop natural frequency and damping ratio
wc = 1;     % Controller natural frequency (rad/s)
zc = 0.707; % Controller damping ratio (critically damped)

% Apply equations from Example 6.4
k1 = wc^2;
k2 = 2*zc*wc - gamma*wc^2;
K = [k1, k2];

% Feedforward gain from Equation 6.13
kr = k1; 

% 3. Observer Design (Using Analytical Equations from Example 7.3)
% Make observer faster than controller (e.g., twice as fast)
wo = 2 * wc;  % Observer natural frequency
zo = 0.707;   % Observer damping ratio

% For A = [0 1; 0 0] and C = [1 0], det(sI - A + LC) = s^2 + l1*s + l2
l1 = 2*zo*wo;
l2 = wo^2;
L = [l1; l2];

% =========================================================================
% 4. SIMULINK INITIALIZATION VARIABLES
% =========================================================================
A_obs = A - L*C;
B_obs = [B, L];           % Muxed input takes both 'u' and 'y'
C_obs = eye(size(A, 1));  % Outputs all estimated states to feed into K
D_obs = zeros(size(C_obs, 1), size(B_obs, 2));

% =========================================================================
% 5. MATLAB lsim Simulation (for standalone plotting)
% =========================================================================
A_cl = [A,           -B*K;
    L*C,  A - L*C - B*K];
B_cl = [B*kr; 
    B*kr];
C_cl = [C, 0, 0]; 
D_cl = 0;

sys_cl = ss(A_cl, B_cl, C_cl, D_cl);
t = 0:0.01:20;
r = sin(0.5 * t); 

[y, t_out, x_out] = lsim(sys_cl, r, t);

figure('Name', 'Vehicle Steering Control');
plot(t, r, '--k', 'LineWidth', 1.5); hold on;
plot(t, y, 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Lateral Deviation (y)', 'FontWeight', 'bold');
title('Vehicle Steering: Tracking Sinusoidal Trajectory (Analytical)');
legend('Desired Trajectory (Reference)', 'Actual Trajectory (Output)', 'Location', 'best');
grid on;

disp('Analytical variables loaded successfully. You can run Simulink now.');