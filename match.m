function [output, new_pose, converged] = match(moving,fixed, new_grid,new_position_cut,bb, pose_camera, resolution, origin, clean_fixed,use_weights)
 %% Normalize colors
    weights = fixed;
    use_weights = 1.0;
    % % 1 - free/currently_free , 2 - free/unk
    % % 3 - occ/fixed , 4 - occ/currently
    % % 0 - unk

    weights(weights == 0) = 0.0;
    weights(weights == 1) = 0.6;
    weights(weights == 2) = 0.6;
    weights(weights == 3) = 1.0;
    weights(weights == 4) = 0.6;

    fixed(fixed == 1) = 255;
    fixed(fixed == 2) = 255;
    fixed(fixed == 0) = 192;
    fixed(fixed == 3 ) = 0;
    fixed(fixed == 4) = 0;
    
    fixed = imgaussfilt(fixed);
    if clean_fixed == 1
        fixed = medfilt2(fixed);
    end

    moving(moving == 0.0) = 0.25;
    moving(moving >= 1.0) = 0;
    moving(moving == 0.5) = 255;
    moving(moving == 0.25) = 192;
    moving = imgaussfilt(moving);

    if use_weights == 1
        [t, alpha, new_image,converged] = register_optimized((fixed),(moving),[0;0], 0,0,weights);
    else
        [t, alpha, new_image,converged] = register_optimized((fixed),(moving),[0;0], 0,0);
    end

    ca = cos(alpha) ;
    sa = sin(alpha) ;
    R = [ca -sa ; sa ca];
    new_t = (R) * -t ;
  
    T = [1 0 0;0 1 0;new_t(1) new_t(2) 1];
    R2 = [ca -sa 0;sa ca 0;0 0 1];

    TR = T*R2;
    tform2 = affine2d(TR);

    new_grid_crop = new_grid(bb(2):bb(2)+bb(4),bb(1):bb(1)+bb(3),:);
    outputView = imref2d(size(new_grid_crop));

    warped_new_grid = imwarp(new_grid_crop,tform2,'OutputView',outputView);

    for i = 1:size(warped_new_grid,1)
        for j = 1:size(warped_new_grid,2)
            if(warped_new_grid(i,j,2) > 0.0)
                warped_new_grid(i,j,1:4) = [0, 0.7, 0, 0.3];
            elseif (warped_new_grid(i,j,1) > 0.0)
                warped_new_grid(i,j,1:4) = [0.7, 0, 0, 0.3];
            elseif warped_new_grid(i,j,4) ~= 1.0
                warped_new_grid(i,j,1:4) = [0, 0, 0, 1.0];
            end
        end
    end
    output = zeros(size(new_grid));
    output(bb(2):bb(2)+bb(4),bb(1):bb(1)+bb(3), :) = warped_new_grid;
    [x, y] = transformPointsForward(tform2,new_position_cut(1),new_position_cut(2));
    delta_position = [(round(x)-new_position_cut(1));(round(y)-new_position_cut(2))];
    delta_t = pose_camera(1:2,1:2) \ [delta_position*resolution];
    dcm = [ca sa 0;-sa ca 0; 0 0 1;0 0 0];
    se3_pose = [dcm, [delta_t(1) delta_t(2) 0 1]'];
    new_pose = pose_camera * se3_pose;
    new_position = round([new_pose(1,4) , new_pose(2,4)]/resolution + origin);
    score_matching = sum(new_image(:) <= 128) / sum(moving(:) <= 128);
end

