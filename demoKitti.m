close all;

%% Parameters
use_odometry = false;
data_set = '03';
root_dir = '/media/daniele/Backup/Dataset/kitti/odometry/sequences/';
base_dir = fullfile(root_dir, data_set);
image_dir = fullfile(root_dir,[data_set '/image_0/' ]);
images   = dir(fullfile(image_dir,'*.png'));
nt_frames = length(images) -1;
velo_dir = [root_dir,data_set,'/velodyne/'];

%% Read ground truth poses for comparision and add error for simulated IMU
if exist('pose','var') == 0 
    vo_bench_poses_fpath = [base_dir '/' data_set,'.txt'];
    vo_bench_poses = dlmread(vo_bench_poses_fpath);
    for i = 1:size(vo_bench_poses,1)
        pose{i} = [vo_bench_poses(i, 1) vo_bench_poses(i, 3) vo_bench_poses(i, 7) vo_bench_poses(i, 12);...
                    vo_bench_poses(i, 9) vo_bench_poses(i,11) vo_bench_poses(i, 2) -vo_bench_poses(i, 4);
                    vo_bench_poses(i, 10) vo_bench_poses(i,5) vo_bench_poses(i, 6) -vo_bench_poses(i, 8);
                    0 0 0 1];
    end
    pose_error = addErrorPoses(pose);
end 

%% Grid initialization
size_grid_x = 1600;
size_grid_y = 1600;
resolution = 0.5;
size_x = round(size_grid_x/resolution);
size_y = round(size_grid_y/resolution);
lidar_grid = zeros(size_x, size_y,4);
lidar_grid(:,:,4) = 1.0;
aging_lidar = zeros(size_x, size_y,1);
total_grid = zeros(size_x, size_y,7);
total_grid(:,:,4) = 1.0;
total_grid(:,:,5) = 0;
origin = [ size_grid_x/(2*resolution), size_grid_y / (2.0*resolution)];

positions_no_error = [];
result = [];
initial_frame = 1;
final_frame = nt_frames;

for frame = initial_frame:1:final_frame
    positions_no_error = [positions_no_error;[pose{frame}(1,4), pose{frame}(2,4)]];
end

%% Main Loop
tic
for frame = initial_frame:1:final_frame
    if frame > initial_frame
        delta_error = pose_error{frame-1} \ pose_error{frame};
        if use_odometry == true
            current_pose = current_pose * delta_error;
        end
    else
        if use_odometry == true
            current_pose = pose_error{frame};
        else
            current_pose = eye(4);
        end
    end
    
    %% Mapping
    MappingKitti();

    %% Visualization
    result = [result;[current_pose(1,4), current_pose(2,4)]];
    if mod(frame,10) == 0
        toc
        tic
        figure(1);
        imagesc(flipud(total_grid(:,:,5)));
        axis equal;
        figure(2);
        plot(positions_no_error(:,1), positions_no_error(:,2),'LineWidth',2,'Color', 'g');
        hold on;
        plot(result(:,1), result(:,2),'LineWidth',2,'Color', 'b');
        axis equal;
    end
    if mod(frame,10) == 0
        frame
    end

end

frame;
