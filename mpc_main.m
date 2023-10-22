clc;
clf;
clear all;

% initial value
global T; 
T = 0.5;
global num_states;
num_states = 80; 

% starting point of actual robot
global x_initial;               
x_initial = [-0.3;0;pi/2];

% Make a starting guess at the solution
x_start = zeros(23,1);
% X0 = zeros(5*num_states+3,1);  

% prediction horizon
global N;
N = 4; 

% noise add decision
noise_switch = 1; 

% generate the desired trajectory
X_desired = desired_trajectory();     

%% lower bounds and upper bounds
x_min=-10;
x_max=10;
y_min=-10;
y_max=10;
theta_min=-1.e1000;
theta_max=1.e1000;

v_min=-1;
v_max=1;
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
% x_initial = (X(start_:start_+2))';

while (1)                    
    end_ = start_ + (N*5 + 3) - 1;
    
    if (end_ < 5*num_states+3)

        X_desired_window = X_desired(start_:end_);
        save X_desired_window X_desired_window;             
        options = optimoptions('fmincon','Algorithm','active-set');
        [X(start_:end_),fval] = fmincon('objfun_WMR',x_start,[],[],[],[],LB,UB,'confun_WMR',options);
         % adding noise
        if (noise_switch) 
            X(start_:end_) = X(start_:end_)+gen_noise();
        end

    else
        % last horizon window
        end_ = length(X_desired);
        X_desired_window = X_desired(start_:end_);
        save X_desired_window X_desired_window;
        N = fix((end_ - start_)/5);

        options = optimoptions('fmincon','Algorithm','active-set');
        [X(start_:end_),fval] = fmincon('objfun_WMR',x_start,[],[],[],[],LB,UB,'confun_WMR',options);
     
        break
    end

    x_start = X(start_:end_)';          % update for a starting guess for each horizon step
    start_ = start_ + 5;                % update for the next horizon
    x_initial = (X(start_:start_+2))';  % update for the next horizon
    
end

%% for drawing of the robot heading
arrow_length = 0.1; % length of the arrow (for drawing robot orientation)

% load the desired trajecotory for drawing
load X_desired

%% draw the figure
figure(1)

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

end
