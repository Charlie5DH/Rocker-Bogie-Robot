function [xr,yr,citar,fir,kr,sigma_C2,L_C2,sigma_max_c2,f_C2] = clotoideBtwPoint(Pini,Pfin,cita,dr,fi_max,vr,dt)
%esta funcion devuelve la curva entre 2 puntos

%configuracion del pto de partida
x1 = Pini(1);
y1 = Pini(2);
cita_1 =cita(1); %orientacion de salida

%configuracion del pto de llegada
x2 = Pfin(1);
y2 = Pfin(2);
cita_2 = cita(2); %orientacion de llegada

cita_1 = AngRango0a2pi(cita_1);
cita_2 = AngRango0a2pi(cita_2);
if(cita_1 == 0 & cita_2 > pi) cita_1 = 2*pi;end

beta = atan2((y2-y1),(x2-x1));  %angulo entre puntos inicio y fin
if(beta < 0) beta = 2*pi + beta; end

%parametrso circunferencia que pasa por ambos puntos inicio y fin
C = cot((cita_2 - cita_1)/2);
Xc = 1/2*(x1+x2+C*(y1-y2));
Yc = 1/2*(y1+y2+C*(x2-x1));
Rc = (sqrt((Xc-x1)*(Xc-x1)+(Yc-y1)*(Yc-y1))+sqrt((Xc-x2)*(Xc-x2)+(Yc-y2)*(Yc-y2)))/2;

%eleccion del punto q.
% los ptos q se van escogiendo separados a una distancia fija uno del otro
% y nos quedamos con el de menor costo. El lugar geometrico del punto q es
% la circunferencia de centro x,yc y radio distancia_1c. 
% Dividimos la circunferencia en 36 parte (cada 10 grados ubicamos un punto).
alfa = (cita_2-cita_1)/2;

gamma1 = atan2((y1-Yc),(x1-Xc));   
gamma2 = atan2((y2-Yc),(x2-Xc));    
if( gamma1 < 0) gamma1 = 2*pi + gamma1; end
if( gamma2 < 0) gamma2 = 2*pi + gamma2; end
if(gamma1 > pi & gamma2 == 0) gamma2 = 2*pi; end

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

for i = 1: 1: length(d_ang)    
    
    q(i,1) = Rc*cos(d_ang(i)) + Xc;     %q_x
    q(i,2) = Rc*sin(d_ang(i)) + Yc;     %q_y
  
    %calculando la trayectoria que pasa por q
    %primer tramo
    %OOOOOOJOOOOOOO
    %cuando se admite que q tome el valor de uno de los extremos hay que
    %validar alfa cero para que no se divida por cero en sigma....
    
    beta_1q = atan2( (q(i,2)-y1),(q(i,1)-x1) );
    if(beta_1q < 0) 
        beta_1q = 2*pi + beta_1q; 
    end        
    alfa = (beta_1q - cita_1); % variacion en la orientacion
    sigma_max1(i) = 1/(2*dr^2*alfa)*(tan(fi_max))^2;
        
    % Calculo de las integrales de Fresnel.
    %esta aproximacion es buena, se puede comprobal con el
    %comando de MATLAB C = mfun('FresnelC',t)
    t = sqrt(2*abs(alfa)/pi);
    RT = (0.506*t+1)/(1.79*t^2+2.054*t+sqrt(2));
    AT = 1/(0.803*t^3+1.886*t^2+2.524*t+2);
    FC = 1/2 - RT*sin(1/2*pi*(AT-t^2));
    FS = 1/2 - RT*cos(1/2*pi*(AT-t^2)); 
%   FC = mfun('FresnelC',t);
%   FS = mfun('FresnelS',t);
    
    D = cos(alfa)*FC + sin(alfa)*FS ;
    
    %r1q = distance(x1,y1,q(i,1),q(i,2));
    r1q = sqrt((x1-q(i,1))*(x1-q(i,1))+(y1-q(i,2))*(y1-q(i,2)));
    sigma(i,1) = 4*pi*sign(alfa)*(D/r1q)^2;
    l(i,1) = 2*sqrt(2*alfa/sigma(i,1));
    
    %segundo tramo-------------------------------
    
    beta_q2 = atan2((y2 - q(i,2)),(x2 - q(i,1)));
    if(beta_q2 < 0) beta_q2 = 2*pi + beta_q2; end
    
    cita_q = (beta_1q - cita_1) + beta_1q;
    if(cita_q > 2*pi) cita_q = cita_q-2*pi; end
    if(cita_q < 0) cita_q = 2*pi + cita_q; end
    alfa =  beta_q2 - cita_q;
%   alfa = (cita_2 - beta_q2);  % variacion en la orientacion

    sigma_max2(i)=1/(2*dr^2*alfa)*(tan(fi_max))^2;
    
    %Calculo de las integrales de Fresnel.
    %esta aproximacion es buena, se puede comprobal con el
    %comando de MATLAB C = mfun('FresnelC',t)
    t = sqrt(2*abs(alfa)/pi);
    RT = (0.506*t+1)/(1.79*t^2+2.054*t+sqrt(2));
    AT = 1/(0.803*t^3+1.886*t^2+2.524*t+2);
    FC = 1/2 - RT*sin(1/2*pi*(AT-t^2));
    FS = 1/2 - RT*cos(1/2*pi*(AT-t^2));
    
    D = cos(alfa)*FC + sin(alfa)*FS ;
    
    rq2 = sqrt((x2-q(i,1))*(x2-q(i,1))+(y2-q(i,2))*(y2-q(i,2))); %distance(x2,y2,q(i,1),q(i,2));
    sigma(i,2) = 4*pi*sign(alfa)*(D/rq2)^2;
    l(i,2) = 2*sqrt(2*alfa/sigma(i,2));
    
    costo(i) = l(i,1) + l(i,2); 
    
    f(i,1) = sqrt(sigma(i,1)/sigma_max1(i)); 
    f(i,2) = sqrt(sigma(i,2)/sigma_max2(i));
    i = i+1; 
end

%se selecciona el punto q(sera el que minimice el costo entre todos los
%calculados anteriormente)
% [valor,index] = min(costo);
% [valor,index] = max(costo);
min_costo = inf;
for i = 1: length(f)
    costo(i) = f(i,1)^2 + f(i,2)^2;
    if(costo(i) < min_costo)
        min_costo = costo(i);
        index = i;
    end
end
% [valor,index] = min(f(:,1).^2+f(:,2).^2);
%-----------------------------------------------PARAMETROS DE LA CURVA

sigma_C2 = [sigma(index,1), sigma(index,2)];
L_C2 = [l(index,1) , l(index,2)];
sigma_max_c2 = [sigma_max1(index), sigma_max2(index)];
f_C2 = [f(index,1), f(index,2)];

%---------------------------------------------------ESTOS SE UTILIZARAN
%PARA LA CURVA 2 SOLAMENTE

dl = vr*dt;             %seleccion del paso de integracion
s1 = 0:dl:l(index,1);   %trayectoria para curva 1
s2 = 0:dl:l(index,2);   %trayectoria para curva 2

%obtencion de la trayectoria que marca la curva
%valores iniciales primer tramo
kr(1) = 0;              %curvatura inicial siempre sera 0
citar(1) = cita_1;      %angulo inicial corresponde con el angulo de partida
xr(1) = x1;             %x inicial corresponde con la de partida
yr(1) = y1;             %y inicial corresponde con la de partida
fir(1) = 0;             

for i=2:length(s1)      
    if(s1(i) < l(index,1)/2)
        kr(i) = sigma(index,1)*s1(i);
    else
        kr(i) = sigma(index,1)*(l(index,1) - s1(i));
    end
    
    citar(i) = citar(i-1) + (kr(i)+kr(i-1))/2*dl;
    %citar(i) =  0.5*sigma(index,1)*s1(i)^2 + cita_1;
    xr(i,1) = xr(i-1) + (cos(citar(i)) + cos(citar(i-1)))/2*dl;
    yr(i,1) = yr(i-1) + (sin(citar(i)) + sin(citar(i-1)))/2*dl;
    
    fir(i) = atan(dr*kr(i)); 
end

% valores iniciales segundo tramo
offset = length(s1);
kr(1+offset) = kr(length(s1));
citar(1+offset) = citar(length(s1));
xr(1+offset) = xr(length(s1));
yr(1+offset) = yr(length(s1));

fir(1+offset) = fir(length(s1)); 

for i = 2+offset: length(s2)+offset
    if( s2(i-offset) < l(index,2)/2 )
        kr(i) = sigma(index,2)*s2(i-offset);
    else
        kr(i) = sigma(index,2)*(l(index,2) - s2(i-offset));
    end
    
    citar(i) = citar(i-1) + (kr(i)+kr(i-1))/2*dl;
    %citar(i) =  0.5*sigma(index,2)*s2(i-offset)^2 + citar(1+offset);
    xr(i) = xr(i-1) + (cos(citar(i)) + cos(citar(i-1)))/2*dl;
    yr(i) = yr(i-1) + (sin(citar(i)) + sin(citar(i-1)))/2*dl;
    fir(i) = atan(dr*kr(i));
end

