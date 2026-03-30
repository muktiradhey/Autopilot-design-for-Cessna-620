% run_and_plot.m
close all; clc;

P = cessna620_parameters();
assignin('base', 'P', P);
sim('mavsim_trim');

figure('Position', [100 100 1000 700]);

subplot(4,2,1); plot(tout, xout(:,4), 'b', 'LineWidth', 1.5);
ylim([xout(1,4)-2, xout(1,4)+2]);
xlabel('Time (s)'); ylabel('u (m/s)'); title('u'); grid on;

subplot(4,2,2); plot(tout, xout(:,5), 'b', 'LineWidth', 1.5);
ylim([-1, 1]);
xlabel('Time (s)'); ylabel('v (m/s)'); title('v'); grid on;

subplot(4,2,3); plot(tout, xout(:,6), 'b', 'LineWidth', 1.5);
ylim([xout(1,6)-2, xout(1,6)+2]);
xlabel('Time (s)'); ylabel('w (m/s)'); title('w'); grid on;

subplot(4,2,4); plot(tout, xout(:,7)*180/pi, 'b', 'LineWidth', 1.5);
ylim([30, 40]);
xlabel('Time (s)'); ylabel('\phi (deg)'); title('Roll angle \phi'); grid on;

subplot(4,2,5); plot(tout, xout(:,8)*180/pi, 'b', 'LineWidth', 1.5);
ylim([xout(1,8)*180/pi - 2, xout(1,8)*180/pi + 2]);
xlabel('Time (s)'); ylabel('\theta (deg)'); title('Pitch angle \theta'); grid on;

subplot(4,2,6); plot(tout, xout(:,9)*180/pi, 'b', 'LineWidth', 1.5);
ylim([-5, 5]);
xlabel('Time (s)'); ylabel('\psi (deg)'); title('Heading \psi'); grid on;

subplot(4,2,7); plot(tout, xout(:,11)*180/pi, 'b', 'LineWidth', 1.5);
ylim([2, 4]);
xlabel('Time (s)'); ylabel('q (deg/s)'); title('Pitch rate q'); grid on;

subplot(4,2,8); plot(tout, -xout(:,3), 'b', 'LineWidth', 1.5);
ylim([495,505])
xlabel('Time (s)'); ylabel('h (m)'); title('Altitude h'); grid on;


sgtitle('Trim Verification — \gamma=0, Cl in range:0.7-1.0, n=1.2.', 'FontSize', 13);