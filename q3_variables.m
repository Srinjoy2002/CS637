% setup_q3.m - Initialization script for Thermostat Hysteresis Model
clear; clc;

% Define central target temperature (Setpoint)
T = Simulink.Parameter;
T.Value = 24;            
T.DataType = 'double';

% Define hysteresis tolerance (+/- 2 degrees)
e = Simulink.Parameter;
e.Value = 2;            
e.DataType = 'double';

disp('Variables T and e loaded successfully into the workspace.');