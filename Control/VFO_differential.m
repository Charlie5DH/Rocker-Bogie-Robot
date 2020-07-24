function [u1,u2,x,y,cita,ex,ey,err_cita,rWheelVel,lWheelVel] = VFO_differential(Xt,Yt,cita_ir,Xr,Yr,citar,signmov,Vt,kp,k_cita,dt,ts,b,r)

u2r = Vt;
%valores iniciales de la trayectoria real
x(1) = Xt(1)+30;  % x-0.1        %m
y(1) = Yt(1)-40;  % y %    %m
cita(1) = cita_ir;        %rad
u1(1) = 0;          %rad/s
u2(1)= 0;           %m/s

%otros valores iniciales
ex(1) = 0;
ey(1) = 0;
err_cita(1) = 0;
cita_der(1) = 0;
cita_aux = 0;

%parametrizacion del tiempo
tf = ts + 5;                        %s. tiempo total de simulacion.
for k = 1:tf/dt+1
    % MIDO...
    if k*dt < ts
        u2r = signmov(k);
    end
    if(k > 1)
        if(k == 153)
            parar = 1;
        end
        if(k*dt < ts)
            dxr = (Xr(k)-Xr(k-1))/dt;
            dyr = (Yr(k)-Yr(k-1))/dt;
            %pie en la marcha atras. producto de la no union bien en los
            %empates de tramos...
            if(signmov(k-1)~=signmov(k))
                %cambio de sentido de movimiento.
                if sign(Xr(k-1)-Xr(k-2)) == sign(dxr)
                    dxr = -dxr;
                end
                if sign(Yr(k-1)-Yr(k-2)) == sign(dyr)
                    dyr = -dyr;
                end
            end
        else
            u1(k) = 0;
            u2(k) = 0;
            continue;
        end
        
        %valores de la tray. real
        [dx,dy,dcita] = ModeloDiferencial(cita(k-1),u1(k-1),u2(k-1));
        x(k) = dx*dt + x(k-1);
        y(k) = dy*dt + y(k-1);
        cita(k) = dcita*dt + cita(k-1);       

    %CALCULO DEL VECTOR DE CONTROL
    
    %calculo del error
    ex(k) = Xr(k) - x(k);
    ey(k) = Yr(k) - y(k);
    
    %otros parametros...
    h2 = kp*ex(k) +  dxr;   %(u2r*cos(fir(k))*cos(citar(k)));
    h3 = kp*ey(k) +  dyr;   %(u2r*cos(fir(k))*sin(citar(k)));
    
    %orientacion auxiliar. algoritmo en forma continua...

    cita_p2 = atan2(sign(u2r)*h3,sign(u2r)*h2);
    if (k > 1)
        cita_der(k) = CitaContinua(cita_p2,cita_aux,cita_der(k-1));
    else
        cita_der(k) = cita_p2;
        if(cita_der(k) < 0) 
            cita_der(k) = 2*pi + cita_der(k);
        end
    end
    cita_aux = cita_p2;
    
    err_cita(k) = cita_aux - cita(k);
    %control u2
    u2(k) = h2*cos(cita(k)) + h3*sin(cita(k));
    u1(k) = k_cita*err_cita(k) + cita_aux;
    
    rWheelVel(k) = (2*u2(k) + u1(k)*b)/(2*r);
    lWheelVel(k) = (2*u2(k) - u1(k)*b)/(2*r);
end
end

