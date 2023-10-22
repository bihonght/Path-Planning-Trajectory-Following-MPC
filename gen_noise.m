function [noise] = gen_noise()
%GEN_NOISE Summary of this function goes here
%   Detailed explanation goes here
global N;

% noise on the velocity and angular velocity
V_noise = 0.05;   % uncertainty on velocity 
Omega_noise = 0.05;   % uncertainty on angular velocity 


% noise level on pose estimation
xy_noise=0.03; % xy error in meter
theta_noise=0.03; % orientation errot in radius

noise = [];
for j=1:N
    noise = [noise; randn(1,1)*xy_noise; randn(1,1)*xy_noise; randn(1,1)*theta_noise; randn(1,1)*V_noise; randn(1,1)*Omega_noise];
end
noise = [noise; randn(1,1)*xy_noise; randn(1,1)*xy_noise; randn(1,1)*theta_noise]';
end

