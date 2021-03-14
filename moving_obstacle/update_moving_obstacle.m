function [obj, occ_map] = update_moving_obstacle(obj, occ_map, old_pos, new_pos, width, height, res, N)
% update occupancy map
if isa(obj, 'collisionCylinder')
    % create a map for the old position
    old_map = binaryOccupancyMap(width, height, res);
    setOccupancy(old_map, old_pos', 1);
    inflate(old_map, obj.Radius);
    [occ_row_old, occ_col_old] = find(getOccupancy(old_map));
    
    % create a map for the new position
    new_map = binaryOccupancyMap(width, height, res);
    setOccupancy(new_map, new_pos', 1);
    inflate(new_map, obj.Radius);
    [occ_row_new, occ_col_new] = find(getOccupancy(new_map));
    
    % remove old position
    setOccupancy(occ_map, [occ_row_old, occ_col_old], 0, 'grid');
    
    % add new position
    setOccupancy(occ_map, [occ_row_new, occ_col_new], 1, 'grid');
else
    % create generic rectangle
    [~, ang] = R2kth(obj.Pose(1:3,1:3));
    rect_x = linspace(-obj.X / 2, obj.X / 2, N);
    rect_y = linspace(-obj.Y / 2, obj.Y / 2, N);
    [rect_x, rect_y] = meshgrid(rect_x, rect_y);
    rect_gen = [rect_x(:), rect_y(:)];
    rect_gen = rect_gen * [cos(ang) -sin(ang); sin(ang) cos(ang)]';
    
    % remove old position
    old_pts = rect_gen + old_pos';
    setOccupancy(occ_map, old_pts, 0);
    
    % add new position
    new_pts = rect_gen + new_pos';
    setOccupancy(occ_map, new_pts, 1);
end

% update object with new position
obj.Pose(1:2,4) = new_pos;
end