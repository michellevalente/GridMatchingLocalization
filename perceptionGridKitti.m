function [ grid, aging_lidar] = perceptionGridKitti( obj, obj_aging_lidar,time,limits)
% Perception : 1- free, 2-occ, 3-conf, 4-unk
% % 1 - free/currently_free , 2 - free/unk
% % 3 - occ/fixed , 4 - occ/currently
% % 0 - unk
grid=obj.X; obj.X=[];
aging_lidar=obj_aging_lidar.X; obj_aging_lidar.X=[];
calculate_entropy = 0;
size_grid_i = size(grid,1);
size_grid_j = size(grid,2);

entropy_grid = zeros(size_grid_i, size_grid_j,1);
specificity_grid = zeros(size_grid_i, size_grid_j,1);
thr_occ = 10;

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
                last_time = aging_lidar(i,j);
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
                        if grid(i,j,1) > 0.5
                            if (time - last_time) > time_th 
                                grid(i,j,5) = 2;
                            else
                                grid(i,j,5) = 1;
                            end
                            grid(i,j,6) = 0;
                        elseif grid(i,j,2) > 0.5
                            if (time - last_time) < time_th
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
                end
            end
        end
    end
end

if calculate_entropy == 1
    average_entropy = mean2(entropy_grid);
    average_specificity = mean2(specificity_grid);
else
    average_entropy = 0;
    average_specificity = 0;
end
end