% Patrick Serafin
% March 19, 2021 
% EP707 - Dr. Drakunov
clear all; clc; close all;

%% Configuration
fprintf('Configuring Simulation Parameters...\n')

% Environment Parameters
Re = 6378.14; % [km] radius of Earth
mu = 3.986005*10^5; % 

% Spacecraft Parameters
thrust = 1;%400;    % [N] engine thrust 
ISP = 3000*0.99;    % [s] specific impulse engine
    
% Initial Conditions -- Circular Orbit
Xi = 9000;
v  = sqrt(mu/Xi);
w0 = [-Xi 0 0 ... % Position
      0 -v 0  ... % Velocity
      100];      % Mass

% Desired State 
r_des = 35786; % Geostationary Orbit
v_des = sqrt(mu/r_des);
theta = linspace(0,2*pi,100);
x_des = r_des*cos(theta);
y_des = r_des*sin(theta);
z_des = zeros(1,length(theta));

% Options - Simulation
simTime = 60*60*24; % min

% Path
addpath('plotting')

%% Run Simulation
fprintf('Running Simulation...\n')
out = sim('orbitalTransferSim',simTime);

%% Plot Simulation Results
fprintf('Plotting Simulation Results...\n')
plotSimResults
