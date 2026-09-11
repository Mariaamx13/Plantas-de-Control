Ts = 0.02;
t = (0:length(YAW1)-1)' * Ts;
idx = t <= 4;

yy1 = lsim(mod1, E_PITCH, Tiempo);
yy2 = lsim(mod2, E_YAW1(idx), t(idx));
yy3 = lsim(mod3, E_PITCH(idx), t(idx));

figure;

subplot(3,1,1)
plot(Tiempo, PITCH, Tiempo, yy1)
legend('Experimento', 'Modelo')
title('Pitch / E\_PITCH')
xlabel('Tiempo (s)'); ylabel('Ángulo (rad)')

subplot(3,1,2)
plot(t(idx), YAW1(idx), t(idx), yy2)
legend('Experimento', 'Modelo')
title('YAW / E\_YAW')
xlabel('Tiempo (s)'); ylabel('Ángulo (rad)')

subplot(3,1,3)
plot(t(idx), YAW(idx), t(idx), yy3)
legend('Experimento', 'Modelo')
title('YAW / E\_PITCH')
xlabel('Tiempo (s)'); ylabel('Ángulo (rad)')