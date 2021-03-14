%--------------------------------------------------------------------------
% purpose: use bug0 algorithm for path planning
%  input:        x_est = estimated state
%            pos_final = desired position
%              z_lidar = lidar measurements
%            ang_lidar = lidar angles
%           col_buffer = closest distance allowed to obstacle to prevent collision
%           max_linear = maximum linear velocity
%          max_angular = maximum angular velocity
% output:            u = control input
%--------------------------------------------------------------------------
function [u] = bug0_moving_obstacle(x_est, pos_final, z_lidar, ang_lidar, col_buffer, max_linear, max_angular)
% determine heading to desired position
delta_pos = pos_final - x_est(1:2);
ang_desired = mod(atan2(delta_pos(2), delta_pos(1)) - x_est(3), 2*pi);

% find lidar angle closest to desired angle
[dist_desired, ind_desired] = min(abs(ang_desired - ang_lidar));

% forward facing lidar angles if facing in desired heading
fwd_desired = mod(abs(ang_lidar - ang_lidar(ind_desired)), 2*pi) < pi/2;

% distance to obstacle in desired heading
dist_obstacle = min(z_lidar(fwd_desired));

if dist_obstacle < col_buffer
    % too close to obstacle to travel in desired direction
    % find closest point of contact
    [~, ind_closest] = min(z_lidar);
    
    % desired heading is now tangential to closest point of contact
    ang_desired = mod(ang_lidar(ind_closest) + pi / 2, 2*pi);
    
    % move forward
    dist_desired = max_linear;
end

% put desired angle in [-pi, pi)
if ang_desired > pi
    ang_desired = ang_desired - 2*pi;
end

% commands to rotate to desired angle
num_max_turn = floor(abs(ang_desired) / max_angular);
final_turn = rem(abs(ang_desired), max_angular);
u_ang = [0; max_angular];
u_ang = sign(ang_desired) * [repmat(u_ang, 1, num_max_turn), [0; final_turn]];

% command to move forward
u_lin = [min(dist_desired, max_linear); 0];

% total commands
u = [u_ang, u_lin];
end
%--------------------------------------------------------------------------