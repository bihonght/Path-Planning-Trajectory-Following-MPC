function f = objfun_WMR(X)

global N;
%% weights
Q=[1, 0, 0;
    0, 1, 0;
    0, 0, 1];     % the weight of Q was 0.5
R = [0.1, 0; 0, 0.1];
%% desired trajectory

load X_desired_window X_desired_window

f = 0;
for i=0:N

    x = X(5*i+1:5*i+3);     %% actual trajectory from X
    x_r = X_desired_window(5*i+1:5*i+3);  
    
    dx = [x(1)-x_r(1);x(2)-x_r(2); wrapToPi(x(3)-x_r(3))];

    if i==N
        % f = f + (x-x_r)'*Q*(x-x_r);
        f = f + dx'*Q*dx;
    elseif i==0
        u = X(4:5);
        u_r = X_desired_window(4:5);
        f = f + (u-u_r)'*R*(u-u_r);
    else 
        u = X(5*i+4:5*i+5);
        u_r = X_desired_window(5*i+4:5*i+5);
        % f = f + (x-x_r)'*Q*(x-x_r) + (u-u_r)'*R*(u-u_r);
        f = f + dx'*Q*dx + (u-u_r)'*R*(u-u_r);
    end

end

% %% actual trajectory from X
% x_0=X(1:3);
% u_0=X(4:5);
% x_1=X(6:8);
% u_1=X(9:10);
% % actual control
% x_2=X(11:13);
% u_2=X(14:15);
% x_3=X(16:18);
% 
% %% objective function
% f = (x_1-x_r_1)'*Q*(x_1-x_r_1)+(x_2-x_r_2)'*Q*(x_2-x_r_2)+(x_3-x_r_3)'*Q*(x_3-x_r_3)...
%     +(u_0-u_r_0)'*R*(u_0-u_r_0)+(u_1-u_r_1)'*R*(u_1-u_r_1)+(u_2-u_r_2)'*R*(u_2-u_r_2);
