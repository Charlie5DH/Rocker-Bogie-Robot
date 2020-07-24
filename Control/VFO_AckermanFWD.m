function [u1,u2,x,y,cita,ex,ey,ecita,efi,rWheelVel,lWheelVel] = VFO_AckermanFWD(Xt,Yt,cita_ir,Xr,Yr,citar,fir,signmov,Vt,kp,k_cita,k_fi,dt,ts,L,wheel_base,wheel_radi)

u2r = Vt;
%valores iniciales de la trayectoria real
x(1) = Xt(1)+30;  % x-0.1        %m
y(1) = Yt(1)-40;  % y %    %m
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
        [dx,dy,dcita,dfi] = modeloAckermanFWD(x(k-1),y(k-1),cita(k-1),fi(k-1),u1(k-1),u2(k-1),L);
        x(k) = dx*dt + x(k-1); 
        y(k) = dy*dt + y(k-1); 
        cita(k) = dcita*dt + cita(k-1);
        fi(k) = dfi*dt + fi(k-1);
    
        if fi(k) < -saturacion*pi/2     %maximo angulo de giro
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
    ex(k) = Xr(k) - x(k);
    ey(k) = Yr(k) - y(k);
    ecita(k) = citar(k) - cita(k);
    efi(k) = fir(k) - fi(k);
    
    %otros parametros...
    h2 = kp*ex(k) +  dxr;   %(u2r*cos(fir(k))*cos(citar(k)));
    h3 = kp*ey(k) +  dyr;   %(u2r*cos(fir(k))*sin(citar(k)));
    
    %orientacion auxiliar. algoritmo en forma continua...
    sigma = sign(u2r);      %sign(u2r*cos(fir(k)))
    cita_p2 = atan2(sigma*h3,sigma*h2);
    if (k > 1)
        citad(k) = CitaContinua(cita_p2,cita_p1,citad(k-1));
    else
        citad(k) = cita_p2;
        if(citad(k) < 0) citad(k) = 2*pi + citad(k);end
    end
    cita_p1 = cita_p2;

    %error de la orientacion auxiliar
    ecitad(k) = citad(k) - cita(k);
    if (ecitad(k)/(2*pi) == floor(ecitad(k)/(2*pi))) 
        ecitad(k) = 0;
    end
    %variacion de la orientacion aux.
    if(k > 1)
        dcitad = (citad(k) - citad(k-1))/dt;
    else    
        dcitad = 0;
    end
    
    h1 = k_cita*ecitad(k) + dcitad; %v1  Kcita ganancia controlador ecita
    v1(k) = h1;
    v2(k) = h2*cos(cita(k)) + h3*sin(cita(k));    
    
    %control u2
    u2(k) = v2(k)*cos(fi(k)) + L*v1(k)*sin(fi(k));
    
    %angulo de direccion aux.
    fid(k) = atan(L*v1(k)/(v2(k)));       
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
    
    u1(k) = k_fi*(fid(k)-fi(k))+ dfid;      %controlador de giro
 
    rWheelVel(k) = (2*u2(k) + u1(k)*wheel_base)/(2*wheel_radi);
    lWheelVel(k) = (2*u2(k) - u1(k)*wheel_base)/(2*wheel_radi);
    end
end

