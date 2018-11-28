function [grid_evidential, aging_lidar, changes, limits,position] = lidarMappingKitti(pointcloud,pose,origin, grid_x, ...
    grid_y, resolution, obj, obj2, time, Tr_velo_to_cam)
 aging_lidar=obj.X; obj.X=[]; 
  grid_evidential=obj2.X; obj2.X=[]; 
%% Transform scan to camera frame
%    pointcloud = pose_camera * pose_camera_lidar * G_ins_laser  * [pointcloud; ones(1, size(pointcloud,2))];
%    pointcloud = pose_camera * [pointcloud; ones(1, size(pointcloud,2));
%    pose_lidar = [pose_camera(1,4), pose_camera(2,4)] + [G_ins_laser(1,4), G_ins_laser(2,4)];

%    pointcloud = Tr_velo_to_cam * pointcloud;
   translate_pc = [ [1,0,0;0,1,0;0,0,1;0,0,0] [Tr_velo_to_cam(3,4);Tr_velo_to_cam(2,4);0;1]];
%    pointcloud = translate_pc * pointcloud;
   pointcloud = pose  * translate_pc* pointcloud;
   new_pose = pose * translate_pc;
   pose_lidar = [new_pose(1,4), new_pose(2,4)];
   position = round(pose_lidar/resolution + origin);
   
%    
% position = origin;
distance_max = 100 / resolution;
%% Limits current map
  limit_x_min = round(position(1) - distance_max);
  if limit_x_min < 1
      limit_x_min = 1;
  end
  
  limit_x_max = round(position(1) + distance_max);
  if limit_x_max > grid_x / resolution
      limit_x_max = grid_x / resolution;
  end
  
  limit_y_min = round((position(2) - distance_max) );
  if limit_y_min < 1
      limit_y_min = 1;
  end
  
  limit_y_max = round((position(2) + distance_max)) ;
  if limit_y_max > grid_y / resolution
      limit_y_max = grid_y / resolution;
  end
  
  limits = [limit_x_min,limit_x_max, limit_y_min, limit_y_max ];
  
%% LIDAR Mapping
  show_pointcloud = 0;
  confidence_lidar_occ = 0.8;  
  confidence_lidar_free = 0.4;
  if show_pointcloud == 1
    showPointcloud(pointcloud);
  end


  res = resolution;
  grid_size_x = grid_x / res;
  grid_size_y = grid_y / res;
 
  size_pc = size(pointcloud);
  changes = zeros(1,2);
  count_changes = 1;
  pointcloud = sortrows(pointcloud', 1)';
  
  angles = [];
  previous_free_x = [];
  previous_free_y = [];
  for i = 1:size_pc(2)
      p = pointcloud(:,i);
      
      row = ceil((p(1))/res + origin(1));
      col = ceil((p(2))/res + origin(2));
      free = [];
      
      
      if(row >= limits(1) && row <= limits(2) && col >= limits(3) && col <= limits(4) && ...
              grid_evidential(col,row,2) ~= confidence_lidar_occ)
        
        changes(count_changes,:) = [col,row];
        count_changes = count_changes + 1;
        grid_evidential(col,row,1) = 0.0;
        grid_evidential(col,row,2) = confidence_lidar_occ;
        grid_evidential(col,row,3) = 0.0;
        grid_evidential(col,row,4) = 1 - confidence_lidar_occ;
        aging_lidar(col,row,1) = time; 
        
      end
      
  end

end
