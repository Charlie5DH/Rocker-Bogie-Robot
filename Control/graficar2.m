function [] =  graficar2(large,wide,Re,path_x,path_y,dt,ts,xref,yref,x,y,scaleMap,cita,citaref,u1,u2,ex,ey,ecita)
figure(1);
spy(scaleMap,10);
hold on;
plot(path_x,path_y,'k');
grid on;
hold on;
plot(xref,yref,'r');
hold on;
plot(x,y,'b');
set(gca,'YDir','reverse');

title('Trayectoria');
legend('estados posibles','camino planificado','camino generado','trayectoria real');
xlabel('posicion(cm)');
ylabel('posicion(cm)');
xticks(0:Re:large);
yticks(0:Re:wide);
axis([0 large 0 wide]);
hold off;

figure(2);
p2 = plot(xref,yref,'r--');
p2.LineWidth = 1.2;
hold on;
plot(x,y,'b');
grid on;
title('trayectoria de referencia y seguimiento de tray');
hold off;

figure(3);
t = 0:dt:ts-2*dt;
plot(t,ex,'k');
title('error en x');
legend('error x');
grid on;
xlabel('tiempo(s)');
ylabel('cm');

figure(4);
t = 0:dt:ts-2*dt;
plot(t,ey,'r');
title('error en y');
grid on;
legend('error y');
xlabel('tiempo(s)');
ylabel('cm');

figure(5);
t = 0:dt:ts-2*dt;
plot(t,cita*180/pi +180);
title('cita');
hold on;

plot(t,citaref*180/pi);
title('cita ref Vs cita real');
legend('orientaciones reales','orientaciones de referencia');
hold off;

figure(6);
tu1 = 0:dt:(length(u1)-1)*dt;
plot(tu1,u2);
hold on;
title('señal de control');
legend('vel lineal(u2)');

figure(7);                            %Graficar las orientaciones de referencia             
t = 0:dt:ts-2*dt;
p2 = plot(t,citaref*180/pi -180);
p2.LineWidth = 1;
title('variación de la orientación de referencia');
legend('orientaciones de referencia');
hold off;
xlabel('tiempo(s)');
ylabel('grados');
yticks(-60:15:100);
grid on;

figure(8);
spy(scaleMap,10);
hold on;
plot(path_x,path_y,'k');
grid on;
hold on;
plot(xref,yref,'r');
set(gca,'YDir','reverse');

title('Trayectoria');
legend('estados posibles','camino planificado','camino generado');
xlabel('posicion(cm)');
ylabel('posicion(cm)');
xticks(0:Re:large);
yticks(0:Re:wide);
axis([0 large 0 wide]);
hold off;
end

