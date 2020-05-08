% Cross-verification plots
figure;
plot(out.tout,out.phi_p_in(:,1),'linewidth',1.5)
hold on;
plot(out.tout,out.phi_p_out(:,1),'linewidth',1.5)
legend('Input','Output');
hold off;
xlabel('Time (in sec)','fontsize',12);
ylabel('\phi_p (in rad)','fontsize',12);
title('Driving space to Joint space - \phi_p','fontsize',14);

figure;
plot(out.tout,out.phi_d_in(:,1),'linewidth',1.5)
hold on;
plot(out.tout,out.phi_d_out(:,1),'linewidth',1.5)
legend('Input','Output');
hold off;
xlabel('Time (in sec)','fontsize',12);
ylabel('\phi_d (in rad)','fontsize',12);
title('Driving space to Joint space - \phi_d','fontsize',14);
