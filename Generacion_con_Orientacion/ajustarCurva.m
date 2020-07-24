function[xref,yref,citaref,firef,kref] = ajustarCurva(x,y,escala,cita,vr,dt,dr)
% x,y: los puntos del intervalo P1, Pv, P2.
% cita: angulo de orientacion inicial
% escala: factor de escala. 
% vr: velocidad del robot
% dt: paso de tiempo.
% Esta funcion genera los valores de referencia de la curva que se ajusta
% al intervalo.
   
delta = vr*dt;

    %obteniendo el tipo de curva que se ajusta al intervalo
    [tipo,invertir,signo] = TipoCurva(x,y,escala);
    
    %valores plantillas para el tipo de curva dado
    switch(tipo)
        case 1
            sigma1 = 2.2305;
            sigma2 = 0;
            l1  = 1.6784;
            l2  = 0;
            f = 1.09*dr;
        case 2
            sigma1 = 0.8460;
            sigma2 = -0.0520;
            l1  = 1.9298;
            l2  = 0.4180;
            f = 0.4713*dr;
        case 3
            sigma1 = 1.1152;
            sigma2 = 0;
            l1  = 2.3736;
            l2  = 0;
            f = 0.7642*dr;
        case 4
            sigma1 = 0;
            sigma2 = 0;
            l1 = sqrt((x(1)-x(2))^2+(y(1)-y(2))^2);
            l2 = sqrt((x(3)-x(2))^2+(y(3)-y(2))^2);
            f = 1;
    end
      
    if(invertir == 1 & tipo~=3)
        % diagonal + corta. Invertir orden de los parametros.
        % esto es valido solo para los dos casos anteriores.
        sigma_temp = sigma1;
        sigma1 = sigma2;
        sigma2 = sigma_temp; 
        l_temp = l1;
        l1 = l2;
        l2 = l_temp;
    end
    
    %ajustando a la escala
    sigma1 = sigma1/f^2;
    sigma2 = sigma2/f^2;
    l1 = l1*f;
    l2 = l2*f;

%ajustando a la escala para no utilizar  p1 prima ni p2 prima
%     sigma1 = sigma1/escala^2;
%     sigma2 = sigma2/escala^2;
%     l1 = l1*escala;
%     l2 = l2*escala;

   % reajustando los puntos de salida y entrada (P1' y P2').
    
    if(tipo~=4)
        %determinando p1x y p2x
        if(x(2)-x(1)==0)
            p1x = x(2);
        elseif(x(2)-x(1)>0)
            p1x = x(2) - f;
        else
            p1x = x(2) + f;
        end
        if(x(3)-x(2)==0)
            p2x = x(2);
        elseif(x(3)-x(2)>0)
            p2x = x(2) + f;
        else
            p2x = x(2) - f;
        end
        %determinando p1y y p2y
        % ecuacion P1Pv
        if(x(1)-x(2)==0)
            if(y(2)-y(1)>0)
                p1y = y(2)-f;
            else
                p1y = y(2)+f;
            end
        else
            m = (y(1)-y(2))/(x(1)-x(2));
            n = y(2)-m*x(2);
            p1y = m*p1x+n;
        end
        % ecuacion PvP2
        if(x(3)-x(2)==0)
            if(y(3)-y(1)>0)
                p2y = y(2)+f;
            else
                p2y = y(2)-f;
            end
        else
            m = (y(3)-y(2))/(x(3)-x(2));
            n = y(2)-m*x(2);
            p2y = m*p2x+n;
        end
    else
        p1x = x(1);
        p1y = y(1);
        p2x = x(3);
        p2y = y(3);
    end

    
    % valores del primer tramo de la curva
      
    %tramo recto 
    l = sqrt((x(1)-p1x)^2+(y(1)-p1y)^2);
    s = 0:delta:l;
        
    kref(1) = 0;
    citaref(1) = cita;
    xref(1) = x(1);
    yref(1) = y(1);
    firef(1) = 0;
    
    for j = 2:length(s)
        kref(j) = 0;
        citaref(j) = citaref(j-1) + (kref(j)+kref(j-1))/2*delta; 
        xref(j) = xref(j-1) + (cos(citaref(j))+cos(citaref(j-1)))/2*delta;
        yref(j) = yref(j-1) + (sin(citaref(j)) + sin(citaref(j-1)))/2*delta; 
        firef(j) = atan(dr*kref(j));
    end
    
    %comienza la curva.
    %primer tramo
    p = length(s)-1;%el -1 es ca;ona...
    if(p == 0)p =1; end
    s = 0:delta:l1; 
    for j=1:length(s)
       if(s(j) < l1/2)
         kref(j+p) = signo*sigma1*s(j);
       else
         kref(j+p) = signo*sigma1*(l1 - s(j));
       end
           
       citaref(j+p) = citaref(j+p-1) + (kref(j+p)+kref(j+p-1))/2*delta; 
       xref(j+p) = xref(j+p-1) + (cos(citaref(j+p))+cos(citaref(j+p-1)))/2*delta;
       yref(j+p) = yref(j+p-1) + (sin(citaref(j+p)) + sin(citaref(j+p-1)))/2*delta; 
       firef(j+p) = atan(dr*kref(j+p));
    end
   
    % segundo tramo    
    p = p + length(s);
    s = 0:delta:l2;
    for j=1:length(s)
       if(s(j) < l2/2)
         kref(j+p) = signo*sigma2*s(j);
       else
         kref(j+p) = signo*sigma2*(l2 - s(j));
       end
           
       citaref(j+p) = citaref(j+p-1) + (kref(j+p)+kref(j+p-1))/2*delta; 
       xref(j+p) = xref(j+p-1) + (cos(citaref(j+p))+cos(citaref(j+p-1)))/2*delta;
       yref(j+p) = yref(j+p-1) + (sin(citaref(j+p)) + sin(citaref(j+p-1)))/2*delta; 
       firef(j+p) = atan(dr*kref(j+p));
    end
    
%     %tramo recto de nuevo
    offset2 = length(s);
    p = p + offset2;
    l = sqrt((x(3)-p2x)^2+(y(3)-p2y)^2);
    s = 0:delta:l;
    for j=1:length(s)
       kref(j+p) = 0;           
       citaref(j+p) = citaref(j+p-1) + (kref(j+p)+kref(j+p-1))/2*delta; 
       xref(j+p) = xref(j+p-1) + (cos(citaref(j+p))+cos(citaref(j+p-1)))/2*delta;
       yref(j+p) = yref(j+p-1) + (sin(citaref(j+p)) + sin(citaref(j+p-1)))/2*delta; 
       firef(j+p) = atan(dr*kref(j+p));
    end


    