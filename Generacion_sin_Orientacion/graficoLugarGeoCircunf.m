P1 = [2 2];
Pv = [3 2];
P2 = [4 3];

pathx = [2 3 4];
pathy = [2 2 3];

cita1 = atan2(Pv(2)-P1(2),Pv(1)-P1(1));
cita2 = atan2(P2(2)-Pv(2),P2(1)-Pv(1));
D = cot((cita2-cita1)/2);
Pc = [ (P1(1)+P2(1)-D*(P2(2)-P1(2)))/2 , (P1(2)+P2(2)+D*(P2(1)-P1(1)))/2];
r = sqrt(((Pc(2)- P1(2))^2) + ((Pc(1)-P1(1))^2));

gamma1 = atan2(Pc(2)-P1(2),Pc(1)-P1(1));
gamma2 = atan2(Pc(2)-P2(2),Pc(1)-P2(1));

theta = linspace(0,2*pi);
x = r*cos(theta)+Pc(1);
y = r*sin(theta)+Pc(2);
plot(x,y);axis equal,grid on;
hold on;
plot(pathx,pathy);

q = [3.635 2.647];
citaq = atan2(q(2)-P1(2),q(1)-P1(1))*2 - cita1;
gamma = atan2(Pc(2)-q(2),Pc(1)-q(1));

