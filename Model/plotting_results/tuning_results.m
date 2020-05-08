% Result plots
% Cross-verification plots
% ode 1 (Step:e-3) (10;0;10)
figure;
hold on;
grid on;
for col = 1:18
 plot(out.tout(1:5001),out.simout(1:5001,col),'linewidth',1.5)
end

hold off;
xlabel('Time (in sec)','fontsize',12);
ylabel('theta (in rad)','fontsize',12);
title('Model Response - All joints - Last link controlled','fontsize',13);
legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5','Joint 6','Joint 7','Joint 8','Joint 9','Joint 10',...
       'Joint 11','Joint 12','Joint 13','Joint 14','Joint 15','Joint 16','Joint 17','Joint 18');
figure;
hold on;
plot(out.tout(1:5001),out.single(1:5001),'linewidth',1.5)
plot(out.tout(1:5001),out.single2(1:5001),'linewidth',1.5)
xlabel('Time (in sec)','fontsize',12);
ylabel('theta (in rad)','fontsize',12);
title('Model Response - Joint 1 and 2 - Last link controlled','fontsize',13);
legend('Joint 1','Joint 2');
hold off;

% plot(out.tout,out.sim_out,'linewidth',1.5)
