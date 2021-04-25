% Patrick Serafin
% March 19, 2021 
% EP707 - Dr. Drakunov
clear all; clc; close all;

%% Configuration
fprintf('Configuring Simulation Parameters...\n')

% Environment Parameters
Re = 6378.14;       % [km] radius of Earth
mu = 3.986005*10^5; % [m^3/s^2] Earth's gravitational parameter

% Spacecraft Parameters
M = 100;            % [kg] initial mass
thrust = 5;         % [N] engine thrust 
ISP = 2100*0.99;    % [s] specific impulse engine

% Initial Conditions -- Circular Orbit
Xi = 9000;
v  = sqrt(mu/Xi);
IC = [-Xi 0 0 ... % Position
      0 -v 0  ... % Velocity
      M];         % Mass

theta_0 = 24*pi;%+pi/180;

% Desired State 
r_des = 35786+Re; % 42,164 [km] Geostationary Orbit
v_des = sqrt(mu/r_des);
theta = linspace(0,2*pi,100);
x_des = r_des*cos(theta);
y_des = r_des*sin(theta);
z_des = zeros(1,length(theta));

% Options - Simulation
simTime = 60*60*24*14; % [s]

% Path
addpath('plotting')

%% Calculate Reference Trajectory
stepSize= 0.25;

%theta_ref1 = (24*pi:stepSize:37*pi);
theta_ref1 = (24*pi:stepSize:45*pi);
a_const1 = -0.2;
n_const1 = 0.40345;
r_ref1 = a_const1.*theta_ref1.^(1./n_const1);
[x_ref1,y_ref1,z_ref1] = pol2cart(theta_ref1,r_ref1,zeros(1,length(r_ref1)));

% theta_ref2 = (18*pi:stepSize:30*pi);
% a_const2 = 3500;
% n_const2 = 2;
% r_ref2 = a_const2.*theta_ref2.^(1./n_const2);
% [x_ref2,y_ref2,z_ref2] = pol2cart(theta_ref2,r_ref2,zeros(1,length(r_ref2)));

r_ref = [r_ref1];
theta_ref = [theta_ref1];

x_ref = [x_ref1];
y_ref = [y_ref1];
z_ref = [z_ref1];

% r_ref = [r_ref1 r_ref2];
% theta_ref = [theta_ref1 theta_ref2];
% 
% x_ref = [x_ref1 x_ref2];
% y_ref = [y_ref1 y_ref2];
% z_ref = [z_ref1 z_ref2];

%figure
%plot3(x_ref,y_ref,z_ref)

%% Run Simulation
fprintf('Running Simulation...\n')
out = sim('orbitalTransferSim_v2',simTime);

%% Plot Simulation Results
fprintf('Plotting Simulation Results...\n')
plotSimResults
