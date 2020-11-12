close all;
clc; clear;

addpath('./lib');

%Constants
R2D = 180/pi; %Radians to degree
D2R = pi/180; %Degree to radians

%Init parameters
%Specifications
drone1_params = containers.Map({'mass', 'armLength', 'Ixx', 'Iyy', 'Izz'}, ...
    {1.25, 0.265, 0.0232, 0.0232, 0.0468});

%State Variables
drone1_intitStates = [0, 0, -6, ...     % X, Y, Z
    0, 0, 0,  ...                                    % Velocities of dX, dY, dZ
    0, 0, 0,  ...                                    % Euler angles phi, theta, psi
    0, 0, 0,]';                                     % Angular rate p, q, r 

%Control Variables - total thrust and moments on each control axis.
drone1_initInputs = [0, 0, 0, 0]';     % u1, u2, u3, u4 (T, M1, M2, M3)

drone1_body = [ 0.265,          0,       0,   1;  ...      % Arm 1
                                  0,  -0.265,       0,   1;  ...      % Arm 2
                          -0.265,          0,        0,  1;  ...      % Arm 3
                                  0,   0.265,        0,  1;   ...      % Arm 4
                                  0,          0,        0,  1;   ...      % Payload
                                  0,          0,  -0.15,  1]';         % Body
                           
drone1_gains = containers.Map(...
    {'P_phi', 'I_phi', 'D_phi', ...
     'P_theta', 'I_theta', 'D_theta', ...
     'P_psi', 'I_psi', 'D_psi', ...
     'P_zdot', 'I_zdot', 'D_zdot'}, ...
    {0.2, 0.0, 0.15, ...
     0.2, 0.0, 0.15, ...
     0.4, 0.0, 0.3, ...
     10.0, 0.4, 0.0});  % Change values to tune the PID_controls
 
simulationTime = 2;
 
drone1 = Drone(drone1_params, drone1_intitStates, drone1_initInputs, drone1_gains, simulationTime);
 
%Init 3D figure
fig1 =figure('pos', [0 200 800 800]);
h = gca;
view(3);
fig1.CurrentAxes.ZDir = 'Reverse' ;
fig1.CurrentAxes.YDir = 'Reverse' ;

axis equal;
grid on;

xlim([-5 5]); ylim([-5 5]); zlim([-8 0]);
xlabel('X[m]'); ylabel('Y[m]'); zlabel('Z[m]');

hold(gca, 'on');
drone1_state = drone1.GetState();
%Homogeneous transform matrix
wHb = [RPY2Rot(drone1_state(7:9))' drone1_state(1:3);
           0 0 0 1];
drone1_world =wHb * drone1_body;
drone1_atti = drone1_world(1:3, :);

fig1_ARM13 = plot3(gca, drone1_atti(1, [1 3]), drone1_atti(2, [1 3]), drone1_atti(3, [1 3]), '-ro', 'MarkerSize', 5); % Connecting drone arm 1 & 3
fig1_ARM24 = plot3(gca, drone1_atti(1, [2 4]), drone1_atti(2, [2 4]), drone1_atti(3, [2 4]), '-bo', 'MarkerSize', 5); % Connecting drone arm 2 & 4
fig1_payload = plot3(gca, drone1_atti(1, [5 6]), drone1_atti(2, [5 6]), drone1_atti(3, [5 6]), '-k', 'LineWidth', 3); % Payload from drone1_body
fig1_shadow = plot3(gca, 0, 0, 0, 'xk', 'LineWidth', 3); 

hold(gca, 'off');

%Init Data figure
fig2 = figure('pos' ,[800 500 800 450]);
subplot(2, 3, 1)
title('phi[deg]');
grid on;
hold on;
subplot(2, 3, 2)
title('theta[deg]');
grid on;
hold on;
subplot(2, 3, 3)
title('psi[deg]');
grid on;
hold on;
subplot(2, 3, 4)
title('x[m]');
grid on;
hold on;
subplot(2, 3, 5)
title('y[m]');
grid on;
hold on;
subplot(2, 3, 6)
title('zdot[m/s]');
grid on;
hold on;

%
commandSig(1) = 10.0 * D2R;    % Change values to tune
commandSig(2) = -10.0 * D2R;    % Change values to tune
commandSig(3) = 10.0 * D2R;    % Change values to tune
commandSig(4) = 1.0;                 % Change values to tune

for i = 1:simulationTime/0.01
    drone1.AttitudeCtrl(commandSig);
    drone1.UpdateState();
    drone1_state = drone1.GetState();
    
    %checking 3D plot
    figure(1)
    wHb = [RPY2Rot(drone1_state(7:9))' drone1_state(1:3); 0 0 0 1];
    drone1_world =wHb * drone1_body;
    drone1_atti = drone1_world(1:3, :);
    
    set(fig1_ARM13, ...
           'xData', drone1_atti(1,[1 3]), ...
           'yData', drone1_atti(2,[1 3]), ...
           'zData', drone1_atti(3,[1 3]));
    set(fig1_ARM24, ...
           'xData', drone1_atti(1,[2 4]), ...
           'yData', drone1_atti(2,[2 4]), ...
           'zData', drone1_atti(3,[2 4]));
    set(fig1_payload, ...
           'xData', drone1_atti(1,[5 6]), ...
           'yData', drone1_atti(2,[5 6]), ...
           'zData', drone1_atti(3,[5 6]));
    set(fig1_shadow, ...
           'xData', drone1_state(1), ...
           'yData', drone1_state(2), ...
           'zData', 0);
    
    figure(2) 
    subplot(2, 3, 1)
        plot(i/100, drone1_state(7)*R2D, '.');
    subplot(2, 3, 2)
        plot(i/100, drone1_state(8)*R2D, '.');
    subplot(2, 3, 3)
        plot(i/100, drone1_state(9)*R2D, '.');
    subplot(2, 3, 4)
        plot(i/100, drone1_state(1)*R2D, '.');
    subplot(2, 3, 5)
       plot(i/100, drone1_state(2)*R2D, '.');
    subplot(2, 3, 6)
        plot(i/100, drone1_state(6)*R2D, '.');
    
    
    drawnow;
    
    %Crash alert
    if (drone1_state(3) >= 0)
        msgbox('Drone Crashed!', 'Error', 'error');
        break;
    end
end    



                              