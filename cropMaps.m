function [fixed,moving,aging_lidar,new_position_cut,bb] = cropMaps(previous_grid,new_grid,aging_lidar,position)

    total_moving = new_grid(:,:,2);
    total_moving(total_moving > 0 ) = 1.0;
    props = regionprops(total_moving,'BoundingBox');
    bb = props.BoundingBox;
    bb(1) = round(bb(1) - 10);
    if bb(1) < 1
        bb(1) = 1.0;
    end
    bb(2) = round(bb(2) - 10);
    if bb(2) < 1
        bb(2) = 1.0;
    end
   
    bb(3) = round(bb(3) + 20);
    bb(4) = round(bb(4) + 20);
    
    if bb(2) + bb(4) > size(total_moving,2)
        bb(4) = size(total_moving,2) - bb(2);
    end
    if bb(1) + bb(3) > size(total_moving,1)
        bb(3) = size(total_moving,1) - bb(1);
    end
    
    %% Crop the image to the matching region
    
    x_cut_before = bb(1);
    y_cut_before = bb(2);
    moving_free = double(imcrop(new_grid(:,:,1), bb));
    moving_occ = double(imcrop(new_grid(:,:,2), bb));
    aging_lidar = double(imcrop(aging_lidar, bb));
    fixed = double(imcrop(previous_grid(:,:,5), bb));
    new_position_cut = round([position(1)-x_cut_before , position(2)-y_cut_before]);
    moving_free(moving_free >= 0.1) = 0.5;
    moving_free(moving_free < 0.1) = 0.0;
    moving_occ(moving_occ >= 0.5) = 1.0;
    moving_occ(moving_occ < 0.5) = 0.0;
    moving = moving_free + moving_occ;
end

