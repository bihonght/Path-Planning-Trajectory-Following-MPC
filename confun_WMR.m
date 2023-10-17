function [c, ceq] = confun_WMR(X)

%% desired trajectory
global num_states;
% initial position and orientation
% x_initial=[-1;-1;0];
x_initial=[0;0;pi/2];
% time interval in each step
global T;

% Nonlinear inequality constraints
c = [];

% Nonlinear equality constraints
ceq = [];
x_next_pred = [0;0;0];
for i=0:num_states
    x = X(5*i+1:5*i+3);          % actual trajectory from X                      
    if (i==0)  
        ceq(i+1,:) = x-x_initial;
    else
        ceq(i+1,:) = x-x_next_pred;
    end
    
    if i<num_states
        u = X(5*i+4:5*i+5);
        [phi_next,x_next,y_next] = compute_next_pose_old(x(3),x(1),x(2),u(1),u(2),T); 
        x_next_pred = [x_next;y_next;phi_next];
    end
end

% ceq_0 = x_0-x_initial;
% 
% [phi_next,x_next,y_next]=compute_next_pose_old(x_0(3),x_0(1),x_0(2),u_0(1),u_0(2),1);
% x_1_pred=[x_next;y_next;phi_next];
% 
% ceq_1 = x_1-x_1_pred;
% 
% [phi_next,x_next,y_next]=compute_next_pose_old(x_1(3),x_1(1),x_1(2),u_1(1),u_1(2),1);
% x_2_pred=[x_next;y_next;phi_next];
% 
% ceq_2 = x_2-x_2_pred;
% 
% [phi_next,x_next,y_next]=compute_next_pose_old(x_2(3),x_2(1),x_2(2),u_2(1),u_2(2),1);
% x_3_pred=[x_next;y_next;phi_next];
% 
% ceq_3 = x_3-x_3_pred;
% 
% ceq = [ceq_0;ceq_1;ceq_2;ceq_3];


