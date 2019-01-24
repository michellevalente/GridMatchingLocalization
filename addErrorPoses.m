
function [new_poses] = addErrorPoses(poses)

positions_x = zeros(size(poses,2));
positions_y = zeros(size(poses,2));
positions_x_error = zeros(size(poses,2));
positions_y_error = zeros(size(poses,2));
new_deltas = cell(size(poses,2),1);
new_poses = poses;
position3d = [ 1 1 1 1]';

for i = 1:size(poses,2)
    positions_x(i) = poses{i}(1,4);
    positions_y(i) = poses{i}(2,4);
end
std_x = 0.03;
std_y = 0.03;
std_yaw = 0.03;

previous_pose = poses{1};

for i = 2:size(poses,2)
    
   
   delta = poses{i-1} \ (poses{i});
   
   x_error = randn(1,1) * std_x + delta(1,4);
   y_error = randn(1,1) * std_y + delta(2,4);
   
   z = atan2(delta(2,1), delta(1,1));
   new_angle = randn(1,1) * std_yaw + z;

   new_dcm = eul2rotm([new_angle, 0 ,0 ]);
   new_delta = [new_dcm(1,1) new_dcm(1,2) delta(1,3) x_error; ...
           new_dcm(2,1) new_dcm(2,2) new_dcm(2,3) y_error; ...
           new_dcm(3,1) new_dcm(3,2) new_dcm(3,3) 0;
           0 0 0 1];
   
   new_deltas{i} = new_delta;
   new_pose = previous_pose * new_delta;
      
   previous_pose = new_pose;   
end

previous_pose = new_poses{1};
positions_x(1) = poses{1}(1,4);
positions_y(1) = poses{1}(2,4);
positions_x_error(1) = positions_x(1);
positions_y_error(1) = positions_y(1);
for i = 2:size(poses,2)

   new_pose = previous_pose * new_deltas{i};
   position = poses{i} * position3d;

   position_error = new_pose * position3d;
   previous_pose = new_pose;

   new_poses{i} = new_pose;
   positions_x(i) = position(1);
   positions_y(i) = position(2);
   positions_x_error(i) = position_error(1);
   positions_y_error(i) = position_error(2);

end

% figure(1);
% plot(positions_x, positions_y, 'r');
% hold on;
% plot(positions_x_error, positions_y_error,'g');
end
