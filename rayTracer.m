function [grid,aging, new_changes] = rayTracer(obj_grid,obj_aging, changes, position,time, theta,resolution)

grid=obj_grid.X; obj_grid.X=[]; 
aging=obj_aging.X; obj_aging.X=[]; 
% changes=obj_changes.X; obj_changes.X=[]; 
new_changes = zeros(500,2);
count_changes = 1;

x1 = position(2);
y1 = position(1);
[M,N] = size(grid);

distance = 75 / resolution;

for angle = -180:1:180
    x2 = round(x1 + distance*cosd(theta + angle + 90));
    y2 = round(y1 + distance*sind(theta + angle + 90));
    [free_x,free_y] = bresenham(x1,y1,x2,y2);
    if size(free_x) ~= 0
        for f = 1:size(free_x)
            
            x = free_x(f);
            y = free_y(f);
            d = sqrt( (x-x1)^2 + (y-y1)^2);
            if x < M && y < N 
                if x > 1 && y > 1 
                    if(grid(x,y,2) == 0) && (grid(x+1,y,2) == 0) && (grid(x,y+1,2) == 0) && (grid(x-1,y,2) == 0) && (grid(x,y-1,2) == 0) && (grid(x+1,y+1,2) == 0) && (grid(x-1,y-1,2) == 0)
                        if grid(x,y,1) == 0
%                             if d > 20
                                grid(x,y,1:4) = [0.7,0.0,0.0,0.3];
                                aging(x,y) = time;
                                changes(end,1:2) = [x y];
                                new_changes(count_changes,1:2) = [x y];
                                count_changes = count_changes + 1;
%                             end
                        end
                    else
                        break;
                    end
                end
            else
                break;
            end
        end
    end
end
end

