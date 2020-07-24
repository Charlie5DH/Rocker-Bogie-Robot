function[xr,yr,kr,citar,fir,m] = CurvaDeEntradaSalidaAlGiroOrientacion(x,y,cita,entrada,dr,vr,dt)

%buscando el angulo de giro plantilla que debe tener...
anguloGiro = AnguloNotable(cita);


%Definiendo si es maniobra de entrada o salida
if(entrada == 1)
    cita_1 = cita;
    cita_2 = anguloGiro;
    
    x1 = x;
    y1 = y;
    
    x2 = 1.1*dr*sqrt(2)*cos(anguloGiro);
    if(x2 == 0)
        y2 = 1.1*dr*sin(anguloGiro);
    else
        y2 = 1.1*dr*sqrt(2)*sin(anguloGiro);
        if(y2 == 0)
            x2 = 1.1*dr*cos(anguloGiro);
        end
     end
    [xr,yr,kr,citar,fir] = CurvaClotoideEntrePuntos([x1 x2],[y1 y2],[cita_1 cita_2],dr,vr,dt);
    m = zeros(1,length(xr));
else
    cita_2 = cita;
    cita_1 = anguloGiro;
    
    x1 = x;
    y1 = y;
 
    % 1.3 para que sigma < sigma_max
%     x2 = 1.3*dr*cos(cita);
%     y2 = 1.3*dr*sin(cita);
    x2  = 0;
    y2  = 0;
    
    [xr,yr,kr,citar,fir] = CurvaClotoideEntrePuntos([x1 x2],[y1 y2],[cita_1 cita_2],dr,vr,dt);
    m = zeros(1,length(xr));
    
    %tramo en marcha atras. correccion de la orientacion virtual..
    offset =length(citar);
    for j=offset+1:offset+length(citar)
        citar(j) = citar(j-offset)+pi;
    end
    
%     xtramo = [x2, (x2+x1)/2, x1];
%     ytramo = [y2, (y2+y1)/2, y1];
%     %valores de referencia para el tramo. Tramo en marcha atras.
%     %Orientacion virtual
%     [xt,yt,citat,fit,kt] = ajuste(xtramo,ytramo,1.1*dr,cita-pi,vr,dt,dr);
% 
%     %copiando valores para los vectores de trayectoria
%     offset =length(xr);
%     for j=offset+1:offset+length(xt)
%         xr(j) = xt(j-offset);
%         yr(j) = yt(j-offset);
%         %tramo en marcha atras. correccion de la orientacion virtual..
%         citar(j) = citat(j-offset)+pi;
%         fir(j) = fit(j-offset);
%         kr(j) = kt(j-offset);
%         m(j) = 1;
%     end
end




