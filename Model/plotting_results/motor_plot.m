close all;
figure;
plot(out.tout,out.actual(:,1),'linewidth',1.5)
hold on;
plot(out.tout,out.ref(:,1),'linewidth',1.5)
legend('Actual Motor Angle','Reference Motor Angle');
ylim([0,1.2]);
hold off;
xlabel('Time (in sec)','fontsize',12);
ylabel('Motor Angle of Rotation (in rad)','fontsize',12);
title('Step Response','fontsize',14);

figure;
plot(out.tout,out.actual(:,2),'linewidth',1.5)
hold on;
plot(out.tout,out.ref(:,2),'linewidth',1.5)
legend('Actual Motor Angle','Reference Motor Angle');
ylim([-1.5,1.5]);
hold off;
xlabel('Time (in sec)','fontsize',12);
ylabel('Motor Angle of Rotation (in rad)','fontsize',12);
title('Response to Sine Wave ref. signal','fontsize',14);
