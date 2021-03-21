clc;
close all;
clear;

path = '../data/';

filename = [path 'rec_data.bin'];

fid = FileIO(filename, FileIO.in);

fid.printHeader();

Time = fid.read('Time');
Pos_data = fid.read('Pos');
Quat_data = fid.read('Quat');
Wrist_joints_data = fid.read('Wrist_joints');


pos_ylabels = {'$x$', '$y$','$z$'};
figure;
for i=1:3
   subplot(3,1,i);
   plot(Time, Pos_data(i,:), 'LineWidth',2);
   ylabel(pos_ylabels{i}, 'interpreter','latex', 'fontsize',15);
   axis tight;
   if (i==1), title('Cartesian Position', 'fontsize',16); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
end 

pos_ylabels = {'$j_0$', '$j_1$','$j_2$'};
figure;
for i=1:3
   subplot(3,1,i);
   plot(Time, Wrist_joints_data(i,:), 'LineWidth',2);
   ylabel(pos_ylabels{i}, 'interpreter','latex', 'fontsize',15);
   axis tight;
   if (i==1), title('Wrist Joints', 'fontsize',16); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
end 
