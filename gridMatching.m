function [output, new_pose, bb,converged] = gridMatching(new_grid,previous_grid, position, curr_occ, pose_camera, resolution, origin, aging_lidar, time)
    copy_aging_lidar = aging_lidar;
    [fixed,moving,aging_lidar,new_position_cut,bb] = cropMaps(previous_grid,new_grid,aging_lidar,position);
    
    candidates_lidar_new = find(time - aging_lidar(:,:) > 30 & aging_lidar(:,:) ~= 0);
    [i1,j1] = ind2sub(size(aging_lidar),candidates_lidar_new);
    for i = 1:size(i1)
       fixed(i1(i),j1(i)) = 0;
    end
   clean_fixed = 0;
   [output, new_pose, converged] = match(moving,fixed, new_grid, new_position_cut, bb, pose_camera, resolution, origin, clean_fixed,1.0);
   
   if mod(time,30) == 0
       [fixed,moving,aging_lidar,new_position_cut,bb] = cropMaps(previous_grid,output,copy_aging_lidar,position);
       clean_fixed = 1;
       [output, new_pose, converged] = match(moving,fixed, output, new_position_cut, bb, pose_camera, resolution, origin, clean_fixed,1.0);
   end
   
end
    

