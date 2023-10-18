clc;
clf;
clear all;

% initial value
global T; 
T = 0.5;
global num_states;
num_states = 80; 

% Make a starting guess at the solution
X0 = zeros(5*num_states+3,1);  

% prediction horizon
global N;
N = 4; 

% generate the desired trajectory
X_desired = desired_trajectory();                                    
global x_initial;               % starting point of actual robot
x_initial = [-0.5;0;pi/2];

%% lower bounds and upper bounds
x_min=-10;
x_max=10;
y_min=-10;
y_max=10;
theta_min=-pi;
theta_max=pi;

v_min=-0.5;
v_max=0.5;
w_min=-pi;
w_max=pi;

LB = []; 
UB = [];
for i=1:N
    LB = [LB; x_min; y_min; theta_min; v_min; w_min];
    UB = [UB; x_max; y_max; theta_max; v_max; w_max];
end
LB = [LB; x_min; y_min; theta_min];
UB = [UB; x_max; y_max; theta_max];

%% call fmincon
start_ = 1;                         %starting poses for each moving horizon
while (1)                    
    end_ = start_ + (N*5 + 3) - 1;
    
    if (end_ < 5*num_states+3)

        X_desired_window = X_desired(start_:end_);
        save X_desired_window X_desired_window
        options = optimoptions('fmincon','Algorithm','active-set');
        [X(start_:end_),fval] = fmincon('objfun_WMR',X0(start_:end_),[],[],[],[],LB,UB,'confun_WMR',options);
        % break
    else
        end_ = length(X0);
        X_desired_window = X_desired(start_:end_);
        save X_desired_window X_desired_window
        N = fix((end_ - start_)/5);

        options = optimoptions('fmincon','Algorithm','active-set');
        [X(start_:end_),fval] = fmincon('objfun_WMR',X0(start_:end_),[],[],[],[],LB,UB,'confun_WMR',options);
        break
    end
    start_ = start_ + 5;                % update for the next horizon
    x_initial = (X(start_:start_+2))';  % update for the next horizon
end

%% for drawing of the robot heading
arrow_length = 0.1; % length of the arrow (for drawing robot orientation)

% load the desired trajecotory for drawing
load X_desired

%% draw the figure
frames = [];
fig = figure()

hold on;
axis('equal')
axis([-6 6 -6 6]);
xlabel('X position (m)')
ylabel('Y position (m)')
title('Trajectories of the robot. Red: desired, Blue: actual')

% plot the two trajectories

for j=0:num_states
    % initial position of robot
    x(1) = X(5*j+1);
    y(1)= X(5*j+2);
    phi(1) = X(5*j+3);
    
    % initial position of the desired robot position
    x_d(1) = X_desired(5*j+1);
    y_d(1)= X_desired(5*j+2);
    phi_d(1) = X_desired(5*j+3);
    
    % draw the vehicle position and orientation
    
    plot(x(1),y(1),'bo') % global coordinate
    plot(x_d(1),y_d(1),'rd') % global coordinate
    quiver(x(1),y(1), arrow_length*cos(phi(1)), arrow_length*sin(phi(1)), 0, 'Color', 'b') 
    quiver(x_d(1),y_d(1), arrow_length*cos(phi_d(1)), arrow_length*sin(phi_d(1)), 0, 'Color', 'r')
    
    % Capture the frame and save as an image
    frame = getframe(fig);
    frames = [frames, frame];
end

% Create a GIF
filename = 'robot_trajectory.gif';
for i = 1:length(frames)
    [A, map] = rgb2ind(frame2im(frames(i), fig), 256);
    if i == 1
        imwrite(A, map, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.2);
    else
        imwrite(A, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.2);
    end
end