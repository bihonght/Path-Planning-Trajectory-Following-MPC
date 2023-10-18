function [X_desired] = desired_trajectory()
%% desired trajectory
% initial position and orientation
x_r_0=[0;0;pi/2];
% desired velocity
v_r = 0.3;
w_r = 0;

% time interval in each step
global T;     

% Define the number of samples
global num_states;

% Initialize the robot's initial pose
X_desired = [x_r_0; v_r; w_r];
% Define the range for the turn rate
min_turn_rate = -pi/4;
max_turn_rate = pi/4;

for j=1:num_states
    x = X_desired(end-4:end-2);
    u = X_desired(end-1:end);

    [phi_next,x_next,y_next] = compute_next_pose_old(x(3),x(1),x(2),u(1),u(2),T);

    x_r_next = [x_next;y_next;phi_next];

    if j<num_states
        % Define control inputs (random values for turn rate and velocity)
        turn_rate = -pi/6; % Random turn rate between -1 and 1
        % velocity = 0.2 + 0.4 * rand(); % Random velocity between 0.2 and 0.6 (adjust as needed)
        velocity = 0.3;
        if j>num_states/8 && j<num_states/5
            turn_rate = pi/7;
            velocity = 0.4;
        elseif j>num_states/5 && j<num_states/3
            turn_rate = -pi/4;
            velocity = 0.25;
        elseif j>num_states/3 && j<num_states/2
            turn_rate = pi/4.5;
            velocity = 0.3;
        elseif j>num_states/2 && j<num_states*0.55
            turn_rate = 0;
            velocity = 0.5;
        elseif j>num_states*0.7 && j<num_states*0.85
            turn_rate = -pi/5;
            velocity = 0.4;
        elseif j>num_states*0.85 && j<num_states*0.95
            turn_rate = pi/9;
            velocity = 0.4;
        elseif j>num_states*0.95
            turn_rate = -pi/9;
            velocity = 0.55;
        end
        X_desired = [X_desired; x_r_next; velocity; turn_rate];
    else
        X_desired = [X_desired; x_r_next];
    end
end


% save the desired trajectory for ploting later
save X_desired X_desired

end