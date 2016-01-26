close all;

%yaw = importdata('~/yaw/19_01_2016_yaw-1.txt');
yaw = importdata('~/yaw/19_01_2016_yaw-2.txt');
%yaw = importdata('~/yaw/19_01_2016_yaw.txt');

r_yaw = -1.50221;
    
yaw = yaw(3:end);

plot(yaw, 'LineWidth',2);
hold on;
plot(106, yaw(40), 'o', 'LineWidth',2);
plot(158, yaw(158), 'o', 'LineWidth',2);
plot(182, yaw(182),'o', 'LineWidth',2);
%plot(102, r_yaw, 'x', 'LineWidth',2);
ylim([min(yaw)-0.8 max(yaw)+0.2]);
legend('IMU yaw', 'capture yaw from IMU', 'start backward parking', 'steering reversal point (triggered by yaw)');
xlabel('timestep'); ylabel('yaw / rad');