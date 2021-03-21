clc;
close all;
clear;

% k = [0.5 0.9 0.3]';
% k = k / norm(k);
% theta = pi/4;
% 
% R1 = axang2rotm([k' theta])
% R2 = axang2rotm_(k, theta)

% jpos = [-1.3470,   0.0316,   0.3502];
jpos = [pi/4,   0, 0];


R =[ 0.9388  -0.3429   0.0316
     0.0472   0.2190   0.9746
    -0.3411  -0.9135   0.2218];


alpha = jpos(1);
beta = jpos(2);
gamma = jpos(3);

R2 = wristAng2rotm(jpos(1), jpos(2), jpos(3));

Rc = [0 0 1; 1 0 0; 0 1 0];
R3 = Rc*gMat(jpos(1)+pi, 0)*gMat(jpos(2)+pi, pi)*gMat(jpos(3), pi);

R
R2
R3

function R = wristAng2rotm(alpha, beta, gamma)

    Rx_alpha = rotx(alpha*180/pi);

    y1 = Rx_alpha*[0 1 0]';
    z1 = Rx_alpha*[0 0 1]';

    %Ry1_beta = axang2rotm([y1', beta]);
    Ry1_beta = axang2rotm_(y1,beta);

    z2 = Ry1_beta * z1;

    Rz2_gamma = axang2rotm([z2', gamma]);

    R = Rz2_gamma * Ry1_beta * Rx_alpha;

end

function R = axang2rotm_(axis, angle)

  x=axis(1);
  y=axis(2);
  z=axis(3);
  c = cos(angle);
  s = sin(angle);
  t=1-c;
    
  R = [ t*x*x + c,	  t*x*y - z*s,    t*x*z + y*s  ;
        t*x*y + z*s,     t*y*y + c,	  t*y*z - x*s  ;
        t*x*z - y*s,   t*y*z + x*s,      t*z*z + c ];

end

function R = gMat(theta, a)

 sin_th = sin(theta);
 cos_th = cos(theta);
 sin_a = sin(a);
 cos_a = cos(a);

  R = [        cos_th,       -sin_th,       0  ;
         sin_th*cos_a,  cos_th*cos_a,  -sin_a  ;
         sin_th*sin_a,  cos_th*sin_a,   cos_a ];
     
%  R2 = rotx(a*180/pi)*rotz(theta*180/pi);
%  norm(R-R2)
%  pause

end
  
  
  
  
  
