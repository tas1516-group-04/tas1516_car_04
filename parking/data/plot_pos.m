close all;
R = importdata('pos.txt');
plot(R(:,1),R(:,2));
hold on;
%C = importdata('corners.txt');
%plot(C(:,1),C(:,2));
title('Robot Position by IMU');
xlabel('x / m');
ylabel('y / m');




figure;
plot(R(:,3));
title('Robot orientation by IMU')
xlabel('Timestep');
ylabel('theta / rad')

