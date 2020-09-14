function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
Kp=[500;500;500];Kd=[20;20;20];
rdes_ddot=des_state.acc+Kp.*(des_state.pos-state.pos)+Kd.*(des_state.vel-state.vel);
F=params.mass*(params.gravity+rdes_ddot(3));

% Moment
M = zeros(3,1);
Kp_ang=[100;100;100];
Kd_ang=[2;2;2];
phi_des=(rdes_ddot(1)*sin(des_state.yaw)-rdes_ddot(2)*cos(des_state.yaw))/params.gravity;
theta_des=(rdes_ddot(1)*cos(des_state.yaw)+rdes_ddot(2)*sin(des_state.yaw))/params.gravity;
rot_des=[phi_des;theta_des;des_state.yaw];%角度
omega_des=[0;0;des_state.yawdot];%角速度
M=Kp_ang.*(rot_des-state.rot)+Kd_ang.*(omega_des-state.omega);

if F>params.maxF
    F=params.maxF;
end

% =================== Your code ends here ===================

end
