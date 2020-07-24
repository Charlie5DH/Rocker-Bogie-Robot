clear
clc
%trayectoria planificada Inicio-Objeto 1
xtp1 = [
44.000000
66.000000
88.000000
110.000000
132.000000
154.000000
176.000000
198.000000
220.000000
242.000000
264.000000
286.000000
308.000000
330.000000
]';

ytp1 = [
44.000000
66.000000
66.000000
88.000000
88.000000
88.000000
88.000000
110.000000
110.000000
110.000000
110.000000
88.000000
66.000000
44.000000
]';

%trayectoria planificada Objeto 1- Objeto 2
xtp2 = [
330.000000
308.000000
286.000000
264.000000
242.000000
220.000000
198.000000
176.000000
154.000000
132.000000
132.000000
154.000000
176.000000
198.000000
220.000000
242.000000
264.000000
286.000000
308.000000
330.000000
352.000000
]';
ytp2 = [
44.000000
66.000000
88.000000
110.000000
132.000000
154.000000
176.000000
176.000000
176.000000
198.000000
220.000000
242.000000
242.000000
242.000000
242.000000
242.000000
242.000000
242.000000
242.000000
242.000000
242.000000    
]';

%trayectoria planificada Objeto 2 -Vertedero
xtp3 = [
352.000000
330.000000
308.000000
286.000000
264.000000
242.000000
220.000000
198.000000
176.000000
154.000000
132.000000
110.000000
88.000000
66.000000
44.000000
]';
ytp3 = [
242.000000
264.000000
286.000000
308.000000
308.000000
308.000000
308.000000
330.000000
352.000000
352.000000
352.000000
352.000000
352.000000
330.000000
330.000000    
]';


% xtp1 = xtp1 - 200;
% ytp1 = 200 -  ytp1;

%parametros de diseño
kp = 5;
k1 = 3;

%valores de la trayectoria de referencia
dr = 10;
vr = 5;
dt = 0.02;

x=0;
y=0;

% %orientacion inicial del robot
% cita_ir = 0*pi/180;  
% 
% %orientacion final del robot
% cita_fr = 135*pi/180;
% 
% %realizando maniobra de giro desde cita_ir hasta cita_fr. 
% [xr,yr,citar,fir,kr,signmov]=GiroOrientacion(x,y,cita_ir,cita_fr,dr,vr,dt);

%PRIMER TRAMO. INICIO - Objeto1. 

%orientacion inicial del robot.
cita_ir = 0*pi/180;
%orientacion final del primer tramo (orientacion con que termina la trayectoria, no la deseada)
cita_f =  atan2((ytp1(length(ytp1))- ytp1(length(ytp1)-1)),(xtp1(length(xtp1))-xtp1(length(ytp1)-1)));
if( cita_f < 0) 
    cita_f = 2*pi + cita_f; 
end

[xr,yr,citar,fir,kr,signmov] = GeneracionCompleta(xtp1,ytp1,cita_ir,cita_f,dr,vr,dt);

%SEGUNDO TRAMO. Objeto1 - Objeto2.

%orientacion inicial
cita_i = cita_f;    %orientacion con que termina el primer tramo
%buscacndo continuidad
if(citar(length(citar))<0)
    cita_i = -2*pi + cita_i;
end

%orientacion final del segundo tramo (orientacion con que termina la trayectoria, no la deseada)
cita_f =  atan2((ytp2(length(ytp2))- ytp2(length(ytp2)-1)),(xtp2(length(xtp2))-xtp2(length(ytp2)-1)));
if( cita_f < 0) 
    cita_f = 2*pi + cita_f; 
end

[xt,yt,citat,fit,kt,signmovt] = GeneracionCompleta(xtp2,ytp2,cita_i,cita_f,dr,vr,dt);

%copiando valores para el vector de referencia..
offset = length(xr);
for j=offset+1:offset+length(xt)
    xr(j) = xt(j-offset);
    yr(j) = yt(j-offset);
    citar(j) = citat(j-offset);
    fir(j) = fit(j-offset);
    kr(j) = kt(j-offset);
    signmov(j) = signmovt(j-offset);
end

%TERCER TRAMO. Objeto2 - Vertedero.

%orientacion inicial
cita_i = cita_f;    %orientacion con que termina el segundo tramo
%buscacndo continuidad
if(citat(length(citat))<0)
    cita_i = -2*pi + cita_i;
end

%orientacion final del tercer tramo (la deseada)
cita_f =  0;

[xt,yt,citat,fit,kt,signmovt] = GeneracionCompleta(xtp3,ytp3,cita_i,cita_f,dr,vr,dt);

%copiando valores para el vector de referencia..
offset = length(xr);
for j=offset+1:offset+length(xt)
    xr(j) = xt(j-offset);
    yr(j) = yt(j-offset);
    citar(j) = citat(j-offset);
    fir(j) = fit(j-offset);
    kr(j) = kt(j-offset);
    signmov(j) = signmovt(j-offset);
end


%CONTROL DE  SEGUIMIENTO


u2r = vr;

%valores iniciales de la trayectoria real
x(1) = xtp1(1)-1;  % x-0.1        %m
y(1) = ytp1(1)-3;  % y %    %m
cita(1) = cita_ir;        %rad
fi(1) = 0;       %rad
saturacion = 60/90;
u1(1) = 0;          %rad/s
u2(1)= 0;           %m/s

%otros valores iniciales
ex(1) = 0;
ey(1) = 0;
ecita(1) = 0;
efi(1) = 0;
fid(1) = 0;
citad(1) = 0;
cita_p1 = 0;

%parametrizacion del tiempo
dt = 0.02;         %s

ts =(length(fir)+1)*dt;             %s. tiempo de duracion de la trayectoria
tf = ts + 5;            %s. tiempo total de simulacion.

for k = 1:tf/dt+1        
    % MIDO...
    if k*dt < ts
        u2r = signmov(k)*vr;
    end
    
    if( k > 1)
        %valores de referencia de la trayectoria.
        %variacion de la tray. ref
        if(k == 153)
            parar = 1;
        end
            
        if k*dt < ts
            dxr = (xr(k)-xr(k-1))/dt; 
            dyr = (yr(k)-yr(k-1))/dt; 
            
            %pie en la marcha atras. producto de la no union bien en los
            %empates de tramos...
            if( signmov(k-1)~=signmov(k))
                %cambio de sentido de movimiento.
                if sign(xr(k-1)-xr(k-2)) == sign(dxr)
                    dxr = -dxr;
                end
                if sign(yr(k-1)-yr(k-2)) == sign(dyr)
                    dyr = -dyr;
                end
            end
        else
            u1(k) = 0;
            u2(k) = 0;
            continue;
        end
    
        %valores de la tray. real
        [dx,dy,dcita,dfi] = ModeloRobot(x(k-1),y(k-1),cita(k-1),fi(k-1),u1(k-1),u2(k-1));
        x(k) = dx*dt + x(k-1); 
        y(k) = dy*dt + y(k-1); 
        cita(k) = dcita*dt + cita(k-1);
        fi(k) = dfi*dt + fi(k-1);
    
        if fi(k) < -saturacion*pi/2
            fi(k) = -saturacion*pi/2;
        end
        if fi(k) > saturacion*pi/2
            fi(k) = saturacion*pi/2;
        end
    else
        dxr = 0;
        dyr = 0;
    end
    %CALCULO DEL VECTOR DE CONTROL
    
    %calculo del error
    ex(k) = xr(k) - x(k);
    ey(k) = yr(k) - y(k);
    ecita(k) = citar(k) - cita(k);
    efi(k) = fir(k) - fi(k);
    
    %otros parametros...
    h4 = kp*ey(k) + dyr;
    h3 = kp*ex(k) + dxr;
    
    %orientacion auxiliar. algoritmo en forma continua...
    cita_p2 = atan2(sign(u2r)*h4,sign(u2r)*h3);
    if k > 1
        citad(k) = CitaContinua(cita_p2,cita_p1,citad(k-1));
    else
        citad(k) = cita_p2;
        if(citad(k) < 0) citad(k) = 2*pi + citad(k);end
    end
    cita_p1 = cita_p2;

    %error de la orientacion auxiliar
    ecitad(k) = citad(k) - cita(k);
    if ecitad(k)/(2*pi) == floor(ecitad(k)/(2*pi)) 
        ecitad(k) = 0;
    end
    %variacion de la orientacion aux.
    if( k > 1)
        dcitad = (citad(k) - citad(k-1))/dt;
    else    
        dcitad = 0;
    end
    h2 = kp*ecitad(k) + dcitad;
    
    %control u2
    u2(k) = (h2*tan(fi(k)) + h3*cos(cita(k)) + h4*sin(cita(k)))/(1+tan(fi(k))^2);
    
    %angulo de direccion aux.
    fid(k) = atan(h2/(abs(u2(k))*sign(u2r)));
    if fid(k) < -0.8*pi/2
        fid(k) = -0.8*pi/2;
    end
    if fid(k) > 0.8*pi/2
        fid(k) = 0.8*pi/2;
    end
    % variacion del ang. de direcc. aux.
    if( k > 1)
        dfid = (fid(k) - fid(k-1))/dt;
    else
        dfid = 0;
    end
    %control u1
    u1(k) = k1*(fid(k)-fi(k))+ dfid;     
end

%GRAFICO
figure(1);
plot(x,y);
title('Trayectoria')
set(gca,'YDir','reverse');
hold on
plot(xr,yr,'r--')
hold off
figure(2)
tu2 = 0:dt:(length(u2)-1)*dt;
plot(tu2,u2)
hold on;
plot(tu2,u1);
title('u2')
figure(3)
t = 0:dt:ts-2*dt;
plot(t,cita*180/pi)
title('cita')
figure(4)
plot(t,citar*180/pi)
title('cita ref')
