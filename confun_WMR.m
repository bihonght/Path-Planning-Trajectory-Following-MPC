function [c, ceq] = confun_WMR(X)

%% desired trajectory
global N;
% initial position and orientation

global x_initial;

% time interval in each step
global T;

% Nonlinear inequality constraints
c = [];

% Nonlinear equality constraints
ceq = [];
x_next_pred = [0;0;0];
for i=0:N
    X(5*i+3) = wrapToPi(X(5*i+3));
    x = X(5*i+1:5*i+3);          % actual trajectory from X                      
    if (i==0)  
        ceq(i+1,:) = x-x_initial;
        ceq(i+1,3) = wrapToPi(x(3)-x_initial(3));
    else
        ceq(i+1,:) = x-x_next_pred;
        ceq(i+1,3) = wrapToPi(x(3)-x_next_pred(3));
    end
    
    if i<N
        u = X(5*i+4:5*i+5);
        [phi_next,x_next,y_next] = compute_next_pose_old(x(3),x(1),x(2),u(1),u(2),T); 
        x_next_pred = [x_next;y_next;phi_next];
    end
end
