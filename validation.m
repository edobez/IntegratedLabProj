
sp = 1; % Tensione dello step

step(sp*W2*rad2rpm,1);
hold on;
plot(sim_shaftSpeed);
hold off;