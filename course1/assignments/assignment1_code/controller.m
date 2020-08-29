function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

kp=55;kd=10;
u = params.mass*(0+sum([kp;kd].*(s_des-s))+params.gravity);
if u>params.u_max
    u=params.u_max;
end

% FILL IN YOUR CODE HERE


end

