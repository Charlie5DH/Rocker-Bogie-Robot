function [q] = getQPoint(Pini,Pfin,cita)

%esta funcion es para obtener el punto de union entre las clotoides
%configuracion del pto de partida
x1 = Pini(1);
y1 = Pini(2);
cita_1 =cita(1); %orientacion de salida

%configuracion del pto de llegada
x2 = Pfin(1);
y2 = Pfin(2);
cita_2 = cita(2); %orientacion de llegada

[cita_1] = AngRango0a2pi(cita_1);
[cita_2] = AngRango0a2pi(cita_2);
if(cita_1 == 0 & cita_2 > pi) cita_1 = 2*pi;end

beta = atan2((y2-y1),(x2-x1));  %angulo entre puntos inicio y fin
if(beta < 0) beta = 2*pi + beta; end

%parametrso circunferencia que pasa por ambos puntos inicio y fin
C = cot((cita_2 - cita_1)/2);
Xc = 1/2*(x1+x2+c*(y1-y2));
Yc = 1/2*(y1+y2+c*(x2-x1));
Rc = (sqrt((xc-x1)*(xc-x1)+(yc-y1)*(yc-y1))+sqrt((xc-x2)*(xc-x2)+(yc-y2)*(yc-y2)))/2;

%eleccion del punto q.
% los ptos q se van escogiendo separados a una distancia fija uno del otro
% y nos quedamos con el de menor costo. El lugar geometrico del punto q es
% la circunferencia de centro x,yc y radio distancia_1c. 
% Dividimos la circunferencia en 36 parte (cada 10 grados ubicamos un punto).
alfa = (cita_2-cita_1)/2;

gamma1 = atan2((y1-Yc),(x1-Xc));   
gamma2 = atan2((y1-Yc),(x1-Xc));    
[gamma1] = AngRango0a2pi(gamma1);
[gamma2] = AngRango0a2pi(gamma2);
if(gamma1 > pi & gamma2 == 0)
    gamma2 = 2*pi; 
end

%eleccion del paso de incremento del angulo... 
paso = 1*pi/180;
if(alfa > 0)
    if(gamma2 < gamma1) 
        gamma2 = gamma2+2*pi; 
    end 
    d_ang = gamma1 + paso:paso:gamma2-paso;     %todos los pntos desde ang_pi hasta ang_p2
else
    if(gamma2 > gamma1) 
        gamma2 = gamma2-2*pi; 
    end 
    d_ang = gamma2 + paso:paso:gamma1-paso;
end

for i = 1: length(d_ang)    
    q(i,1) = Rc*cos(d_ang(i)) + Xc;     %q_x
    q(i,2) = Rc*sin(d_ang(i)) + Yc;     %q_y
end

end

