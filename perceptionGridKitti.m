function [ grid, aging_lidar, aging_stereo] = perceptionGridKitti( obj, obj_aging_stereo, obj_aging_lidar,time,limits)
% Perception : 1- free, 2-occ, 3-conf, 4-unk
% % 1 - free/currently_free , 2 - free/unk
% % 3 - occ/fixed , 4 - occ/currently
% % 0 - unk
grid=obj.X; obj.X=[];
aging_lidar=obj_aging_lidar.X; obj_aging_lidar.X=[];
aging_stereo=obj_aging_stereo.X; obj_aging_stereo.X=[];
calculate_entropy = 0;
size_grid_i = size(grid,1);
size_grid_j = size(grid,2);

% current_state_grid = zeros(size_grid_i, size_grid_j,2);
% current_state_grid(:,:,1) = 4;
% perception_grid = zeros(size_grid_i, size_grid_j,2);
% perception_grid(:,:,1) = 5;
entropy_grid = zeros(size_grid_i, size_grid_j,1);
specificity_grid = zeros(size_grid_i, size_grid_j,1);
thr_occ = 10;

% previous_state = grid(:,:,5);
% grid(:,:,5) = 4;

new_states = 1;
i_min = limits(3) - 50;
if i_min < 1
    i_min = 1;
end
i_max = limits(4) + 50;
if i_max > size(grid,1)
    i_max = size(grid,1);
end
j_min = limits(1) - 50;
if j_min < 1
    j_min = 1;
end
j_max = limits(2) + 50;
if j_max > size(grid,1)
    j_max = size(grid,1);
end
time_th = 10;

for j = j_min:j_max
    for i = i_min:i_max
        if grid(i,j,4) ~= 1.0
            if new_states == 0
                [~,I] = max([grid(i,j,1),grid(i,j,2),grid(i,j,3),grid(i,j,4)]);
                grid(i,j,5) = I;
            else
                last_time_lidar = aging_lidar(i,j);
                last_time_stereo = aging_stereo(i,j);
                if last_time_lidar > last_time_stereo
                    last_time = last_time_lidar;
                else
                    last_time = last_time_stereo;
                end
                if (time - last_time) < time_th + 10
                    if grid(i,j,4) == 1.0
                        if grid(i,j,5) == 1
                            if (time - last_time) > time_th
                                grid(i,j,5) = 2;
                            else
                                grid(i,j,5) = 1;
                            end
                        elseif grid(i,j,5) == 4
                            grid(i,j,5) = 0;
                        end
                    else
%                         [~,I] = max([grid(i,j,1),grid(i,j,2),grid(i,j,3),grid(i,j,4)]);
                        %                         previous_state = grid(i,j,5);
                        %                         if grid(i,j,5) == 0
%                         i_1 = grid(i,j,1);
%                         i_2 = grid(i,j,2);
%                         i_3 = grid(i,j,3);
%                         i_4 = grid(i,j,4);
                        if grid(i,j,1) > 0.5
                            if (time - last_time) > time_th 
                                grid(i,j,5) = 2;
                            else
                                grid(i,j,5) = 1;
                            end
                            grid(i,j,6) = 0;
                        elseif grid(i,j,2) > 0.5
                            if (time - last_time) < time_th
                                %                             if move > 7.0
                                %                                 grid(i,j,6) = grid(i,j,6) + 2;
                                %                             elseif move > 4 && move <= 6

                                %                             end
                                if grid(i,j,6) > thr_occ
                                    grid(i,j,5) = 3;
                                else
                                    grid(i,j,5) = 4;
                                    grid(i,j,6) = grid(i,j,6) + 1;
                                   
                                end
                            else
                                if grid(i,j,5) == 4
                                    grid(i,j,5) = 0;
                                end
                            end

                        elseif grid(i,j,3) > 0.5
                            grid(i,j,5) = 4;
                           
                        else
                            if grid(i,j,5) == 1
                                if (time - last_time) > time_th
                                    grid(i,j,5) = 2;
                                else
                                    grid(i,j,5) = 1;
                                end
                            elseif grid(i,j,5) == 4
                                grid(i,j,5) = 0;
                            end
                        end
                    end
                else
                    if grid(i,j,5) == 4
                        grid(i,j,5) = 0 ;
                    end
                    %                         if previous_state(i,j) ~= 4
                    %                             grid(i,j,5) = previous_state(i,j);
                    %                         else
                    %                             grid(i,j,5) = 0;
                    %                         end
                end
                %             else
                %                 grid(i,j,5) = 0;
                %             end
            end
        end
    end
end

% if mod(time, 100) == 0
%     candidates_lidar = find(time - aging_lidar(:,:) > 200 & aging_lidar(:,:) ~= 0);
%     candidates_stereo = find(time - aging_stereo(:,:) > 200 & aging_stereo(:,:) ~= 0);
%     [i1,j1] = ind2sub(size(aging_lidar),candidates_lidar);
%     [i2,j2] = ind2sub(size(aging_stereo),candidates_stereo);
%     for i = 1:size(i1)
% %        grid(i1(i),j1(i),1:4) = [0 0 0 1];
%        grid(i1(i),j1(i),5:6) = 0;
%        aging_lidar(i1(i),j1(i)) = 0;
%     end
%     for i = 1:size(i2)
% %        grid(i2(i),j2(i),1:4) = [0 0 0 1];
%        grid(i2(i),j2(i),5:6) = 0;
%        aging_stereo(i2(i),j2(i)) = 0;
%     end
% end

% if mod(time,1) == 0
%     part_grid = grid(i_min:i_max,j_min:j_max,:);
%     obj_grid=Reference;
%     obj_grid.X=part_grid; clear part_grid;
%     part_grid = clusterAndUpdateState(obj_grid);
%     grid(i_min:i_max,j_min:j_max,:) = part_grid;
% end

if calculate_entropy == 1
    average_entropy = mean2(entropy_grid);
    average_specificity = mean2(specificity_grid);
else
    average_entropy = 0;
    average_specificity = 0;
end
end