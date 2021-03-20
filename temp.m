clc;
close all;
clear;

% oa = -1.34834;
% ob = 0.0338511;
% og = 0.349775;
% 
% R2 = eul2rotm([og, ob, og], 'ZYX')

jpos = [-1.3485, 0.0310, 0.3483];


R =[ 0.9395  -0.3412   0.0310
     0.0468   0.2176   0.9749
    -0.3394  -0.9145   0.2204];


alpha = jpos(1);
beta = jpos(2);
gamma = jpos(3);

x = [1 0 0]';
y = [0 1 0]';
z = [0 0 1]';

Rx_alpha = rotx(alpha*180/pi);

y1 = Rx_alpha*y;
z1 = Rx_alpha*z;

Ry1_beta = axang2rotm([y1', beta]);

z2 = Ry1_beta * z1;

Rz2_gamma = axang2rotm([z2', gamma]);

R2 = Rz2_gamma * Ry1_beta * Rx_alpha;

R
R2
  
  
  
  
  
