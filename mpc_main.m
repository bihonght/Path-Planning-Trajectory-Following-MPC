clc;
clf;
clear all;

% initial value
global T; 
T = 0.5;
global num_states;
num_states = 35; 

X0 = zeros(5*num_states+3,1);     % Make a starting guess at the solution

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
for i=1:num_states
    LB = [LB; x_min; y_min; theta_min; v_min; w_min];
    UB = [UB; x_max; y_max; theta_max; v_max; w_max];
end
LB = [LB; x_min; y_min; theta_min];
UB = [UB; x_max; y_max; theta_max];

% call fmincon
options = optimoptions('fmincon','Algorithm','active-set');

[X,fval] = fmincon('objfun_WMR',X0,[],[],[],[],LB,UB,'confun_WMR',options)


%% for drawing of the robot heading
arrow_length = 0.1; % length of the arrow (for drawing robot orientation)

% load the desired trajecotory for drawing
load X_desired

%% draw the figure
figure(1)

hold on;
axis('equal')
axis([-3 3 -3 3]);
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
