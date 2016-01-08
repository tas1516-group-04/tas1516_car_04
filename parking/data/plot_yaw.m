close all;

yaw = importdata('yaw.txt');

r_yaw = -1.50221;

plot(yaw);
hold on;
plot(110, -1.68, 'x', 'LineWidth',2);
plot(129, -2.114, 'x', 'LineWidth',2);
%plot(102, r_yaw, 'x', 'LineWidth',2);
legend('IMU yaw', 'start backward parking', 'steering reversal point (triggered by yaw)');
xlabel('timestep'); ylabel('yaw / rad');