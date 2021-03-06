function [t,alpha,im_warped,converged] = register_optimized(fixed,moving,t0,alpha0,optWeight,weight)
%[t,alpha,im_warped] = register(fixed,moving,t0,alpha0,optWeight,weight)
%
% Make registration between fixed and moving, with a matrix weights
% Tip: this function uses "inverse compositional" method to limit the
% number of computation of the image gradient
%
%
% The function to minimize is:
%  ||ï¿½fixed(w-1(pi')) - moving(pi') ||^2_weightï¿½
%  with pi' = w(pi) and pi is all the pixels of an image
%
% optWeight = 1 to set the weights of unknown area to zero
max_itr = 200;
momentum = 0;
incrOld = [0;0;0];
show_images = 0;
neutralValue = 192 ;
s = size(moving);

if (nargin==5)
    weight = ones(s);
elseif (nargin~=6)
    error('Bad number of arguments');
end
    

[gx,gy] = gradient(fixed);

% Optimization:
remove = find(gx.^2+gy.^2==0);
keep = find(gx.^2+gy.^2 ~=0);
N = length(keep);
gx = gx(keep) ;
gy = gy(keep) ;
weight(remove) = [] ;

i = (1:N)';
j1 = (i-1)*2+1 ;
j2 = (i-1)*2+2 ;

imG = sparse([i;i],[j1;j2],[gx(:);gy(:)],N,2*N);


[px,py] = meshgrid(1:s(2),1:s(1));
p = [px(:).';py(:).'] ;
p(:,remove) = [];

converge = false ;

grad_p = zeros(2*N,3); % Correspond to dp'/d(t,alpha)
grad_p(1:2:end,1) = -1 ;
grad_p(2:2:end,2) = -1 ;
im_warped  = zeros(s);
t = t0 ;
alpha = alpha0 ;



nbIter = 0;
while (~converge)
 
   % Compute the warped coordinates
   ca = cos(alpha) ;
   sa = sin(alpha) ;
   R = [ca -sa ; sa ca];
   R_deriv = [-sa -ca ; ca -sa] ; 
   p_prime = p;
   p_prime(1,:) = p_prime(1,:) + t(1);
   p_prime(2,:) = p_prime(2,:) + t(2);
   p_prime = R*p_prime ;
   
   % Compute the gradient of "w-1"(p')
   % Note that: w-1: p'-->  p = R'p' - t
   interm = reshape(R_deriv.'*p_prime(:,:),2*N,1) ;
   grad_p(:,3) = interm ;
   J = imG*grad_p ; % Final gradient
   
   % Compute the warped values and error
   p_prime2 = round(p_prime);
   id_ok = find(p_prime2(1,:)<=s(2) & p_prime2(1,:)>0 &...
       p_prime2(2,:)<=s(1) & p_prime2(2,:) >0) ;
   idx = sub2ind(s,p_prime2(2,id_ok).',p_prime2(1,id_ok).');
   im_warped(:)   = neutralValue ;
   im_warped(keep(id_ok)) = moving(idx);
   try err = fixed(keep) - im_warped(keep);
   catch
       keyboard;
   end
   
   % Compute the new weights
   weight2 = weight ;
   if optWeight
       weight2(fixed(keep)==neutralValue | im_warped(keep)==neutralValue) = 0 ;
   end

   w_sp = sparse(1:N,1:N,weight2(:),N,N);
   err = w_sp*err ;
   J = w_sp*J ; %weighted gradient
   
   % Compute the increment of the solution
   
   incr = -J \ err ;

   incr = incrOld*momentum + (1-momentum)*incr;
   t = t+ incr(1:2) ;
   alpha = alpha + incr(3);


   if (round(norm(incr(1:2)),2) < round(length(moving)/5000,2)  && abs(incr(3))<0.01*pi/180*2 )
       converge  = 1;
       converged = 1;
   else  
       nbIter = nbIter + 1;
       if nbIter == max_itr
           converge  = 1;converged = 1;
           if (round(norm(incr(1:2)),3) > round(length(moving)/1000,3)  && abs(incr(3))>0.02*pi/180*2 )
                converged = 0;
           end
       end
   end
end
