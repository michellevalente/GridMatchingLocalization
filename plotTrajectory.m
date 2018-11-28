function [] = plotTrajectory(poses_gt,poses_new, poses_error,size_trajectory)
positions_no_error = [];
positions_fixed = [];
positions_error = [];

for frame = 1:size_trajectory
    positions_no_error = [positions_no_error;[poses_gt{frame}(1,4), poses_gt{frame}(2,4)]];
    positions_fixed = [positions_fixed;[poses_new{frame}(1,4), poses_new{frame}(2,4)]];
    positions_error = [positions_error;[poses_error{frame}(1,4), poses_error{frame}(2,4)]];
end

figure(2);
plot(positions_no_error(:,1), positions_no_error(:,2),'LineWidth',2,'Color', 'g');
hold on;
plot(positions_fixed(:,1), positions_fixed(:,2),'LineWidth',2,'Color', 'b');
% plot(positions_error(:,1),  positions_error(:,2),'r');
axis equal;
end

