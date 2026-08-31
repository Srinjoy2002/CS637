% =========================================================================
% Vehicle Steering System (Linearized) - Controller & Observer Design
% Reference: Astrom & Murray (AM09), Examples 6.4 & 7.3
% =========================================================================
clear; clc; close all;

% 1. System Parameters (Normalized Kinematic Bicycle Model)
gamma = 0.5; % Ratio of distance to center of mass vs wheelbase
A = [0 1; 
    0 0];
B = [gamma; 
    1];
C = [1 0];   % Output is the first state (lateral deviation)
D = 0;

% 2. Controller Design (Example 6.4)
% Place closed-loop controller poles (e.g., at s = -1 ± i)
p_ctrl = [-1+1i, -1-1i];
K = place(A, B, p_ctrl);

% Feedforward gain (kr) to ensure steady-state tracking of the reference
% We want the DC gain of the closed-loop system to be 1
kr = 1 / (C * inv(B*K - A) * B);

% 3. Observer Design (Example 7.3)
% Place observer poles faster than controller poles (e.g., at s = -2 ± 2i)
p_obs = [-2+2i, -2-2i];
L = place(A', C', p_obs)';

% =========================================================================
% MATLAB Simulation (Mathematical Equivalent of Simulink)
% =========================================================================
% Let augmented state X = [x; x_hat]
% dx/dt = Ax - BK*x_hat + B*kr*r
% dx_hat/dt = LCx + (A - LC - BK)*x_hat + B*kr*r

A_cl = [A,           -B*K;
    L*C,  A - L*C - B*K];
B_cl = [B*kr; 
    B*kr];
C_cl = [C, 0, 0]; % Extract actual lateral deviation y = x1
D_cl = 0;

% Create State-Space system
sys_cl = ss(A_cl, B_cl, C_cl, D_cl);

% Define Time and Sinusoidal Reference Trajectory
t = 0:0.01:20;
r = sin(0.5 * t); 

% Simulate
[y, t_out, x_out] = lsim(sys_cl, r, t);

% Plot the Output
figure('Name', 'Vehicle Steering Control');
plot(t, r, '--k', 'LineWidth', 1.5); hold on;
plot(t, y, 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Lateral Deviation (y)', 'FontWeight', 'bold');
title('Vehicle Steering: Tracking Sinusoidal Trajectory (with Observer)');
legend('Desired Trajectory (Reference)', 'Actual Trajectory (Output)', 'Location', 'best');
grid on;