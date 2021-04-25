close all

%% Get Data From Simulation
t = out.tout;
x = out.simout.Data(:,1);
y = out.simout.Data(:,2);
z = out.simout.Data(:,3);
r_pos = out.des_vs_actual.signals(1).values;
r_guidance = out.des_vs_actual.signals(2).values;
theta_guidance = out.theta.signals.values;
deltaV_Req = out.deltaV_req.signals.values;
engine_switching = out.engineSwitch.signals.values;

% Downsample and Get Cartesion Form for Guidance Trajectory
r_pos = r_pos(1:100:end)./1000;
r_guidance = r_guidance(1:100:end)./1000;
theta_guidance = theta_guidance(1:100:end);
early_obsTime = (60*60*12)/0.25;
time_h = t(1:10:early_obsTime)/(3600);
time_d = t(1:100:end)./(3600*24);
deltaV_Req_wholeDownsampled = deltaV_Req(1:100:end).*1000;
deltaV_Req_earlySim = deltaV_Req(1:10:early_obsTime).*1000;

% Get Cartesion Form for Guidance Trajectory
[x_ref, y_ref] = pol2cart(r_guidance, theta_guidance);
z_ref = zeros(1,length(x_ref));

% Derive Desired Orbit Trajectory
x_des = r_des*cos(theta);
y_des = r_des*sin(theta);
z_des = zeros(1,length(theta));

% Derive Position Error
error_pos = (r_guidance-r_pos).*1000;

% Derive Velocity Error

%% Plot Simulation Results 
figure;
hold on;

% Plot Desired Orbit
%plot3(x_ref, y_ref, z_ref, 'r--', 'LineWidth',1)
plot3(x_des, y_des, z_des, 'r--', 'LineWidth',3)

% Plot Trajectory
plot3(x, y, z, 'b', 'LineWidth',1)
plot3(x(end), y(end), z(end), 'Marker','o')

% Plot Earth
[earth_x,earth_y,earth_z] = sphere;
mesh(earth_x*Re,earth_y*Re,earth_z*Re);

% Figure Settings
legend('Desired Orbit', 'Spacecraft Trajectory') 
grid on;
grid minor;
rotate3d on;
axis equal;
axis([-r_des-300 r_des+300 -r_des-300 r_des+300])
view(45, 45);


figure
plot(time_h,deltaV_Req_earlySim)
title('\DeltaV Required to Stay on Course -- First 12 Hours')
xlabel('Time [hours]')
ylabel('\Delta V  [m/s]')
grid on;
grid minor;

figure
hold on
plot(time_d,r_guidance, 'r--','LineWidth',2)
plot(time_d,r_pos, 'b', 'LineWidth',2)
title('Guidance vs Actual Trajetory')
xlabel('Time [Days]')
ylabel('Orbital Radius [km]')
legend('Guidance','Trajectory')
grid on;
grid minor;

figure
hold on
plot(time_d,error_pos, 'r','LineWidth',2)
title('Position Error')
xlabel('Time [Days]')
ylabel('Orbital Radius [m]')
legend('Guidance','Trajectory')
grid on;
grid minor;

figure
subplot(2,1,1)
plot(time_d,deltaV_Req_wholeDownsampled)
title('\DeltaV Required to Stay on Course')
xlabel('Time [days]')
ylabel('\Delta V  [m/s]')
grid on;
grid minor;
% Switching
subplot(2,1,2)
hold on
plot(t./(3600*24),engine_switching, 'r','LineWidth',1)
title('Engine Switching')
xlabel('Time [Days]')
ylabel('Engine Status')
legend('Engine Command')
grid on;
