close all

%% Get Data From Simulation

% Environment Parameters
Re = 6378.14;       % [km] radius of Earth
mu = 3.986005*10^5; % 

% Desired State 
r_des = 35786; % Geostationary Orbit
v_des = sqrt(mu/r_des);
theta = linspace(0,2*pi,100);
x_des = r_des*cos(theta);
y_des = r_des*sin(theta);
z_des = zeros(1,length(theta));

x = out.Spacecraft_Trajectory.signals(1).values;
y = out.Spacecraft_Trajectory.signals(2).values;
z = out.Spacecraft_Trajectory.signals(3).values;
t = out.tout;

%% Plot Simulation Results 
figure;
hold all;

% Plot Desired Orbit
plot3(x_des, y_des, z_des, 'k--')

% Plot Trajectory
plot3(x, y, z, 'b')
plot3(x(end), y(end), z(end), 'Marker','o')

% Plot Earth
[earth_x,earth_y,earth_z] = sphere;
mesh(earth_x*Re,earth_y*Re,earth_z*Re);

% Figure Settings
legend('Desired Orbit', 'Trajectory', 'Spacecraft') 
grid on;
rotate3d on;
axis equal;
axis([-r_des-100 r_des+100 -r_des-100 r_des+100])
view(45, 45);
