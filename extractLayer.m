function [new_velo] = extractLayer(velo)
    azimuths = [];
    layers = {};
    prev_azimuth = 0;
    cnt = 1;

    new_velo = [];
    for i = 1:size(velo,1)
        
       x = velo(i,1);
       y = velo(i,2);
       azimuth = double(atan2(double(y),double(x)));
       
       if azimuth < 0
           azimuth = 2*pi + azimuth;
       end
       if((azimuth - prev_azimuth) < -0.2)
           cnt = cnt + 1;
       end
       if cnt == 1
          new_velo = [new_velo [velo(i,1:3) 1]'];
       else
           break;
       end
       prev_azimuth = azimuth;
    end
end

