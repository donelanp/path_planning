clear all; close all;

%--------------------------------------------------------------------------
% setup simulation
%--------------------------------------------------------------------------
% axes
ez = [0; 0; 1];

% number of lidar scan directions
M = 16;

% lidar noise covariance
R_lidar = 0.1 * eye(M);

% initial state covariance matrix
P_rob = 0.1  * eye(3);
P_range = 0.05 * eye(3 * 4);
P_bear = 0.05 * eye(3);
P = blkdiag(P_rob, P_range, P_bear);

% process noise covariance matrix
Q_rob = 0.001 * eye(3);
Q_range = zeros(3 * 4);
Q_bear = zeros(3);
Q = blkdiag(Q_rob, Q_range, Q_bear);

% observation noise covariance matrix
R_range = 0.1 * eye(4);
R_bear = 0.1;
R = blkdiag(R_range, R_bear);

% obstacles
colobj = setup_environment();

% true initial state
x_robot = [1; 2; 0];
x_range = [0 0 4 0 10 4 10 10 4 10 0 4]';
x_bear = colobj{14}.Pose(1:3,4);
x_true = [x_robot; x_range; x_bear];

% mobile robot
robot = setup_robot(x_robot);
a = 0;

% simulation window
sim1 = display_simulation(robot, colobj, 1);

% let figure update
pause(1);

% used to manually quit simulation
key = ' ';
key_pressed = 0;
set(sim1,'KeyPressFcn',@get_key);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% control robot
%--------------------------------------------------------------------------
% sample time
ts = 1;

% maximum velocities
max_linear = 0.5;
max_angular = 0.5;

% initial state estimate
x_est = normrnd(x_true, diag(P));

% history of true and estimated states
x_true_hist = x_true;
x_est_hist = x_est;

% create occupancy map of environment
width = 12;
height = 12;
res = 100;
N = 1000;
occ_map = create_occupancy(width, height, res, N, colobj);

% robot free space
free_space = generate_free_space(0.2, [0 10 0 10], 50, occ_map);

% get initial map points using lidar
[new_points, z_lidar, ang_lidar] = get_map_points(x_est, x_true, M, R_lidar, occ_map);
map_points = new_points;

% path deviation tolerance
path_epsilon = 0.1;

% target position
pos_final = [4 8];

% path planning algorithm
alg = 'bug 0';

switch alg
    case 'bug 0'
        path = pos_final';
        vertices = [];
        edges = [];
    case 'potential field'
        path = pos_final';
        vertices = [];
        edges = [];
        gain_att = 1;
        gain_rep = 1;
        col_buffer = 1;
        distance_thresh  = 5;
        alpha = 0.1;
    case 'brush fire'
        path = pos_final';
        vertices = [];
        edges = [];
        res = 3;
        brush_fire_grid = nan(height * res, width * res);
        brush_fire_grid = update_brush_fire_grid(brush_fire_grid, height, width, res, pos_final(2), pos_final(1), 0);
    case 'prm'
        num_sample = floor(0.25 * size(free_space, 1));
        num_neighbors = 4;
        [vertices, edges, path] = probability_road_map(free_space, occ_map, x_est(1:2)', pos_final, num_sample, num_neighbors);
        path = path';
    case 'rrt'
        num_sample = floor(0.25 * size(free_space, 1));
        [vertices, edges, path] = rapidly_exploring_random_tree(free_space, occ_map, x_est(1:2)', pos_final, num_sample);
        path = path';
end

% index of desired position along path
path_ind = 1;

% index of moving obstacle
move_ind = 9;

% equation of motion for moving obstacle
A = 1;
T = 5;
move_fun = @(t) [2; 7] - A * sin(2 * pi * t / T) * [1; 0];

% number of movement iterations
num_iter = 0;

% exit by pressing 'q' or when path is completed
while ~strcmp(key, 'q') && path_ind <= size(path, 2)
    switch alg
        case 'bug 0'
            col_buffer = 0.5;
            u_list = bug0_moving_obstacle(x_est, path(:, path_ind), z_lidar, ang_lidar, col_buffer, max_linear, max_angular);
        case 'potential field'
            u_list = potential_field_moving_obstacle(x_est, path(:, path_ind), new_points, max_linear, max_angular,...
                gain_att, gain_rep, col_buffer, distance_thresh, alpha);
        case 'brush fire'
            [u_list, brush_fire_grid] = brush_fire(x_est, brush_fire_grid, height, width, res, new_points, max_linear, max_angular);
        case {'prm', 'rrt'}
            col_buffer = 0;
            u_list = bug0_moving_obstacle(x_est, path(:, path_ind), z_lidar, ang_lidar, col_buffer, max_linear, max_angular);
    end
    
    % update states with control inputs
    for ii=1:size(u_list,2)
        num_iter = num_iter + 1;
        
        u = u_list(:,ii);
        
        % state transition
        [x_true, F] = state_transition(x_true, u, Q, a, ts);
        
        % update robot object
        robot.Pose(1:2,4) = x_true(1:2);
        robot.Pose(1:3,1:3) = rot(ez, x_true(3));
        
        % update the true state history
        x_true_hist = [x_true_hist, x_true];
        
        % state estimation using extended kalman filter
        [x_est, x_est_hist, P] = ekf(x_est, x_est_hist, x_true_hist, u, Q, P, R, a, ts);
        
        % update moving obstacle
        if ~isempty(move_ind)
            old_pos = colobj{move_ind}.Pose(1:2,4);
            new_pos = move_fun(num_iter * ts);
            [colobj{move_ind}, occ_map] = update_moving_obstacle(colobj{move_ind},...
                occ_map, old_pos, new_pos, width, height, res, N);
            delete(sim1.CurrentAxes.Children(2));
            show(colobj{move_ind});
        end
        
        % get map points using lidar
        [new_points, z_lidar, ang_lidar] = get_map_points(x_est, x_true, M, R_lidar, occ_map);
        map_points = [map_points; new_points];
        
        % update simulation frame
        figure(1);
        delete(sim1.CurrentAxes.Children(2));
        [~,patchObj] = show(robot);
        patchObj.FaceColor = [1 0 0];
        patchObj.EdgeColor = 'none';
        drawnow;
        
        frame = getframe(sim1);
        images{num_iter} = frame2im(frame);
        
        % desired position reached
        if norm(abs(x_est(1:2) - path(:, path_ind))) < path_epsilon
            path_ind = path_ind + 1;
            break;
        end
    end
end

% save images to gif
filename = [alg, ' moving obstacle.gif'];
for ii=1:num_iter
   [A, map] = rgb2ind(images{ii}, 256); 
   if ii == 1
        imwrite(A, map, filename,'gif', 'LoopCount', Inf, 'DelayTime', 0.1);
    else
        imwrite(A, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    end
end

% plot path results
% plot_analysis(x_est_hist, x_true_hist, pos_final, free_space, occ_map,...
%     vertices, edges, path, alg, 2);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% purpose: display simulation
%   input:    robot = collision box representing mobile robot
%            colobj = cell array of collision boxes for environment
%           fig_num = number of figure
%  output:      sim = figure handle for simulation
%--------------------------------------------------------------------------
function [sim] = display_simulation(robot, colobj, fig_num)
sim = figure(fig_num);
clf(sim);

% display walls
col = rand(1,3);
[~,patchObj] = show(colobj{1});
patchObj.FaceColor = col;
patchObj.EdgeColor = 'none';
hold on;
for ii=2:4
    [~, patchObj] = show(colobj{ii});
    patchObj.FaceColor = col;
    patchObj.EdgeColor = 'none';
end

% display desks
[~,patchObj] = show(colobj{5});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{6});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';

% display stools
[~,patchObj] = show(colobj{7});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{8});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
% [~,patchObj] = show(colobj{9});
% patchObj.FaceColor = rand(1,3);
% patchObj.EdgeColor = 'none';

% display other obstacles
[~,patchObj] = show(colobj{10});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{11});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{12});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{13});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{14});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';

% display robot
[~,patchObj] = show(robot);
patchObj.FaceColor = [1 0 0];
patchObj.EdgeColor = 'none';

sim.CurrentAxes.Children(end).Visible = 'off';

view(-90,90);
axis([-1 11 -1 11 0 4]);

rotate3d off;
zoom off;
brush off;
datacursormode off;
pan off;
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% purpose: create collision box representing mobile robot
%   input:    q0 = initial configuration
%  output: robot = collision box representing mobile robot
%--------------------------------------------------------------------------
function [robot] = setup_robot(q0)
% z-axis
ez = [0; 0; 1];

% circular mobile robot
robot = collisionCylinder(0.2, 0.2);

% robot position
robot.Pose(1:2, 4) = q0(1:2);
robot.Pose(3, 4) = 0.1;

% robot orientation
robot.Pose(1:3, 1:3) = rot(ez, q0(3));
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% purpose: setup environment collision boxes
%   input:
%  output: colobj = cell array of collision boxes for environment
%--------------------------------------------------------------------------
function [colobj] = setup_environment()
ez=[0;0;1];

% walls
wL=10;wW=.5;wH=4;
colobj{1}=collisionBox(wL,wW,wH);
colobj{1}.Pose(1:3,4)=[wL/2;-wW/2;wH/2];
colobj{2}=collisionBox(wL,wW,wH);
colobj{2}.Pose(1:3,4)=[wL/2;wL+wW/2;wH/2];
colobj{3}=collisionBox(wL,wW,wH);
colobj{3}.Pose(1:3,1:3)=rot(ez,pi/2);
colobj{3}.Pose(1:3,4)=[-wW/2;wL/2;wH/2];
colobj{4}=collisionBox(wL,wW,wH);
colobj{4}.Pose(1:3,1:3)=rot(ez,pi/2);
colobj{4}.Pose(1:3,4)=[wL+wW/2;wL/2;wH/2];

% desks
dL=1;dW=2;dH=.9;
desk1loc=[5;5];desk2loc=[2;8];
colobj{5}=collisionBox(dL,dW,dH);
colobj{6}=collisionBox(dL,dW,dH);
colobj{5}.Pose(1:3,4)=[desk1loc;dH/2];
colobj{6}.Pose(1:3,4)=[desk2loc;dH/2];
colobj{6}.Pose(1:3,1:3)=rot(ez,pi/2);

% stools
sR=.25;sL=.4;
colobj{7}=collisionCylinder(sR,sL);
colobj{8}=collisionCylinder(sR,sL);
colobj{9}=collisionCylinder(sR,sL);
colobj{7}.Pose(1:3,4)=[4.2;4.2;sL/2];
colobj{8}.Pose(1:3,4)=[4.2;5.8;sL/2];
colobj{9}.Pose(1:3,4)=[2;7;sL/2];

% other obstacles
colobj{10}=collisionBox(1,2,2);
colobj{11}=collisionBox(1,2,2);
colobj{12}=collisionBox(2,1,2);
colobj{13}=collisionBox(2,1.5,2);
colobj{14}=collisionCylinder(.5,1.5);
colobj{10}.Pose(1:3,4)=[9.5;8;1];
colobj{11}.Pose(1:3,4)=[9.5;1;1];
colobj{12}.Pose(1:3,4)=[4;.5;1];
colobj{13}.Pose(1:3,4)=[6;.5;1];
colobj{14}.Pose(1:3,4)=[8;5;.75];
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% purpose: get pressed key
%--------------------------------------------------------------------------
function get_key(~, event)
assignin('base', 'key', event.Key);
assignin('base', 'key_pressed', 1);
end
%--------------------------------------------------------------------------