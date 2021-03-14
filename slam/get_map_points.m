%--------------------------------------------------------------------------
% purpose: get map points using lidar measurements and estimated state
%   input:      x_est = estimated state
%              x_true = true state
%                   M = number of lidar directions
%                   R = lidar noise covariance
%             occ_map = 2D binary occupancy map
%  output: map_points = 2D map points from lidar measurements
%             z_lidar = lidar measurements
%           ang_lidar = lidar angles
%--------------------------------------------------------------------------
function [map_points, z_lidar, ang_lidar] = get_map_points(x_est, x_true, M, R, occ_map)
z_lidar = [];

if M > 0
    % lidar range
    range = 5;
    
    % scan directions
    ang_lidar = 2 * pi * (0:M-1)' / M;
    
    % get lidar measurements
    end_points = rayIntersection(occ_map, x_true(1:3), ang_lidar, range);
    z_lidar = vecnorm(end_points - x_true(1:2)', 2, 2);
    
    % add noise
    z_lidar = normrnd(z_lidar, diag(R));
    
    % remove invalid measurements
    out_range = isnan(z_lidar);
    z_lidar(out_range) = [];
    ang_lidar(out_range) = [];
    
    % create map points
    map_points = x_est(1:2)' + z_lidar .* [cos(x_est(3) + ang_lidar) sin(x_est(3) + ang_lidar)];
end
end
%--------------------------------------------------------------------------