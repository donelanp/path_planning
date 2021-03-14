%--------------------------------------------------------------------------
% purpose: update the state of the mobile robot
%   input:      x = current state
%               u = control input [linear velocity; angular velocity]
%               Q = process noise covariance matrix
%               a = distance from center of wheel axle to front of robot
%              ts = sample time
%  output: x_next = next state of mobile robot
%               F = state transition matrix evaluated at (x,u)
%--------------------------------------------------------------------------
function [x_next, F] = state_transition(x, u, Q, a, ts)
n = numel(x);

% orientation of robot
theta = x(3);

% model uncertainty
w = normrnd(zeros(size(x)), diag(Q));

% kinematic model
f = [cos(theta) -a * sin(theta); sin(theta) a * cos(theta); 0 1];

xdot_robot = f * u;

% next state
x_next_robot = x(1:3) + xdot_robot * ts;
x_next = [x_next_robot; x(4:end)] + w;

% state transition matrix
F_robot = [1 0 (-sin(theta) * u(1) - a * cos(theta) * u(2)) * ts; 0 1 (cos(theta) * u(1) - a * sin(theta) * u(2)) * ts; 0 0 1];
F = blkdiag(F_robot, eye(n-3));
end
%--------------------------------------------------------------------------