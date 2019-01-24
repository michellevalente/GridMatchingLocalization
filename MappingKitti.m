grid_matching = 1;
clear changes;

%% LIDAR mapping
% load velodyne points
fid = fopen(sprintf('%s/velodyne/%06d.bin',base_dir,frame),'rb');
velo = fread(fid,[4 inf],'single')';
fclose(fid);

velo = extractLayer(velo);

obj_aging=Reference;
obj_aging.X=aging_lidar; clear aging_lidar;
obj_grid_lidar=Reference;
obj_grid_lidar.X=lidar_grid; clear lidar_grid;
[lidar_grid, aging_lidar, changes_lidar, limits,position] = lidarMappingKitti(velo, current_pose, origin, size_grid_x, ...
    size_grid_y, resolution, obj_aging, obj_grid_lidar, frame);

%% Add Free space
[yaw, pitch, roll]  = dcm2angle(current_pose(1:3,1:3));
theta = 0 + radtodeg(yaw);  
obj_lidar_grid=Reference; obj_lidar_grid.X=lidar_grid; clear lidar_grid;
obj_aging_lidar=Reference; obj_aging_lidar.X=aging_lidar; clear aging_lidar;
[lidar_grid, aging_lidar, new_changes] = rayTracer(obj_lidar_grid, obj_aging_lidar, changes_lidar, position, frame, theta, resolution);
changes = union(changes_lidar, new_changes, 'rows');

%% Grid Matching
no_fusion = 0;
if frame > initial_frame && grid_matching == 1 

    [new_lidar_grid, new_pose, bb,converged] = gridMatching(lidar_grid, total_grid, position,1, current_pose, resolution,origin, aging_lidar, frame);
    delta_new = new_poses{frame-1} \ new_pose ;
    delta_matching = delta_new \ delta_error;
    [~,~,angle] = dcm2angle( delta_matching(1:3,1:3));
    error_trans_limit = 1.3;
    dx = delta_matching(1,4);
    dy = delta_matching(2,4);
    d = sqrt(dx*dx + dy*dy);
    a = delta_matching(1,1);
    b = delta_matching(2,2);
    c = delta_matching(3,3);
    d2 = 0.5 * (a+b+c-1.0);
    r = acos(max(min(d2,1.0),-1.0));
    if (abs(round(d,1)) > 3.0) || round(r,2) > 0.5 || abs(round(dx,1)) > 1.8
        new_pose = current_pose;
        no_fusion = 1;
        count_no_fusion = count_no_fusion + 1;
        if count_no_fusion == 4
            no_fusion = 0;
            count_no_fusion = 0;
        end
    else
        current_pose = new_pose;
        no_fusion = 0;
        lidar_grid = new_lidar_grid;
        count_no_fusion= 0;
    end
    new_poses{frame} = current_pose;
else
    new_poses{frame} = current_pose;
end

if frame > initial_frame
    if no_fusion == 0
    %% Time Fusion
        obj=Reference;
        obj.X=total_grid; clear total_grid;
        obj_lidar_grid = Reference; obj_lidar_grid.X=lidar_grid;clear lidar_grid;
        [total_grid, ~] = timeFusion(obj_lidar_grid, obj, changes);
    end
else
    total_grid(:,:,1:4) = lidar_grid(:,:,1:4);
end
%% Perception layer mapping
obj_total_grid=Reference;
obj_total_grid.X=total_grid; clear total_grid;
obj_aging_lidar=Reference;
obj_aging_lidar.X=aging_lidar; clear aging_lidar;
[total_grid, aging_lidar] = perceptionGridKitti( obj_total_grid, obj_aging_lidar,frame,limits);  

lidar_grid = zeros(round(size_grid_x/resolution), round(size_grid_y/resolution),5);
lidar_grid(:,:,4) = 1.0;