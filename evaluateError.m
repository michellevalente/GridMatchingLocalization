load poses_10.mat
close all;
sequence_error = struct;
count = 1;
% for sequence = 0:10
%     if sequence == 1
%         continue;
%     end
%     if sequence < 10 
%         seq = ['0' num2str(sequence)];
%     else
%         seq = num2str(sequence);
%     end
%     clear pose;
%     clear new_poses;
%     clear pose_error;
%     clear timestamps;
%     load(['poses' seq '.mat']);
    
    %% Get distances
    timeframes = frame;
    dist = zeros(timeframes,1);
    for i = 2:size(pose,2)-1
        P1 = pose{i-1};
        P2 = pose{i};
        dx = P1(1,4) - P2(1,4);
        dy = P1(2,4) - P2(2,4);
        dz = P1(3,4) - P2(3,4);
        dist(i) = dist(i-1) + sqrt(dx*dx + dy*dy + dz*dz);    
    end

    plotTrajectory(pose,new_poses, pose_error,timeframes)
    error_delta_result = zeros(timeframes,1);
    error_delta_noise = zeros(timeframes,1);
    % 
    % for frame = 2:timeframes
    %     pose_delta_gt = pose{frame} \ pose{frame-1};
    % %     pose_delta_result = new_poses{frame} \ new_poses{frame-1};
    % %     pose_delta_noise = pose_error{frame} \ pose_error{frame-1};
    %     pose_delta_result =  new_poses{frame} \ pose{frame};
    %     pose_delta_noise =  pose_error{frame} \ pose{frame};
    % %     error_delta_result(frame) =  translationError(pose_delta_gt \ pose_delta_result);
    % %     error_delta_noise(frame) =  translationError(pose_delta_gt \ pose_delta_noise);
    % error_delta_result(frame) =  translationError(pose_delta_result);
    %     error_delta_noise(frame) =  translationError(pose_delta_noise);
    % 
    % end
    % 
    % figure;plot(1:timeframes, error_delta_result ,'b'); 
    % hold on;
    % plot(1:timeframes, error_delta_noise,'r'); 
    % %% Sequence errors

    % every second
    step_size = 10;

    % segment lenghts
    num_lengths = 8;
    lengths = [100,200,300,400,500,600,700,800];
t_err_length
    % For all start positions do
    for first_frame = 1:step_size:size(pose,2)-1
        for l = 1:num_lengths
            len = lengths(l);
            last_frame = lastFrameFromSegmentLength(dist,first_frame,len);
            if last_frame == -1
                continue;
            end

            pose_delta_gt = inv(pose{first_frame}) * pose{last_frame};
            pose_delta_result = inv(new_poses{first_frame}) * new_poses{last_frame} ;
            pose_error_delta = inv(pose_delta_result) * pose_delta_gt ;
            r_err = rotationError(pose_error_delta);
            t_err = translationError(pose_error_delta);

            % compute speed
            num_frames = last_frame-first_frame+1;
            speed = len / (0.1 * num_frames);

            sequence_error(count).first_frame = first_frame;
            sequence_error(count).r_err = r_err/len;
            sequence_error(count).t_err = t_err/len;
            sequence_error(count).len = len;
            sequence_error(count).speed = speed;
            count = count + 1;

        end
    end
% end
t_err_length = zeros(num_lengths,1);
r_err_length = zeros(num_lengths,1);
for i = 1:num_lengths
    t_err = 0;
    r_err = 0;
    num = 0;
    
    for j = 1:size(sequence_error,2)
        if(abs(sequence_error(j).len - lengths(i) < 1.0))
            t_err = t_err + sequence_error(j).t_err;
            r_err = r_err + sequence_error(j).r_err;
            num = num + 1;
        end
    end
    if num > 2
        t_err_length(i) = t_err / num;
        r_err_length(i) = r_err / num;
    end
end
figure;plot(lengths, r_err_length ); ylim([0 0.01]);
figure;plot(lengths, t_err_length * 100); ylim([0 10]);
%% Functions
function lastFrame = lastFrameFromSegmentLength(dist, frame, len)
    lastFrame = -1;
    for d = 1:size(dist)
       if dist(d) > dist(frame) + len
           lastFrame = d;
           return;
       end 
    end
end

function r = rotationError(pose_error_delta)
    a = pose_error_delta(1,1);
    b = pose_error_delta(2,2);
    c = pose_error_delta(3,3);
    c = 1.0;
    d = 0.5 * (a+b+c-1.0);
    r = acos(max(min(d,1.0),-1.0));
end

function t = translationError(pose_error_delta)
    dx = pose_error_delta(1,4);
    dy = pose_error_delta(2,4);
    dz = 0;
    t = sqrt(dx*dx+dy*dy+dz*dz);
end