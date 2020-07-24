function[xg,yg,citag,fig,kg,m]=GiroOrientacion(x,y,cita_i,cita_f,dr,vr,dt)
% Determina la trayectoria para realizar un giro de orientacion.
% Devuelve:
% xp: coordenas x de la trayectoria
% yp: coordenas y de la trayectoria
% citag: variacion en el angulo de orientacion.
% m: indica si el proximo movimiento en en marcha atras. m=1--> marcha
% atras.
% Recibe:
% x,y posicion actual del robot. Al realizar los giros debe terminar 
% en esta misma posicio pero con otra orientacion.
% cita_i,cita_f: angulos de orientacion inicial y fina respectivamente.


%angulo notable inicial para comenzar maniobra de giro. 
cita_in = AnguloNotable(cita_i);
%angulo notable final para la maniobra de giro. 
cita_fn = AnguloNotable(cita_f);

[xp,yp,citap,m] = PlanificarGiro(x,y,cita_i,cita_f,dr);

%aproximandomea un angulo notable...si la dif. es mayor
%que 1 grado
if(abs(cita_in - cita_i)>1*pi/180 & abs(cita_in - cita_i) ~= 2*pi)
    [xg,yg,citag,fig,kg] = CurvaClotoideEntrePuntos([xp(1) xp(2)],[yp(1) yp(2)],[citap(1) citap(2)],dr,vr,dt);
else
    %generar recta hasta el punto 2...
    xtramo = [xp(1), (xp(2)+xp(1))/2, xp(2)];
    ytramo = [yp(1), (yp(2)+yp(1))/2, yp(2)];

    %valores de referencia para el tramo recto
    [xg,yg,citag,fig,kg]  = ajustarCurva(xtramo,ytramo,1.3*dr,cita_in,vr,dt,dr);
end

[xt,yt,citat,fit,kt] = GiroBasico(xp,yp,x,y,m,citap,dr,vr,dt);

offset =length(xg);
for j=offset+1:offset+length(xt)
    xg(j) = xt(j-offset);
    yg(j) = yt(j-offset);
    citag(j) = citat(j-offset);
    kg(j) = kt(j-offset);
    fig(j) = fit(j-offset);
end
    
%girando hasta el angulo final deseado..si la dif. es mayor
%que 1 grado

if(abs(cita_f - cita_fn) > pi/180 & abs(cita_f - cita_fn) ~= 2*pi )
    if(m(length(m)-1) == 1)
        cita_temp = [citap(length(citap)-1)+pi citap(length(citap))+pi];
    else
        cita_temp = [citap(length(citap)-1) citap(length(citap))];
    end
    [xt,yt,citat,fit,kt] = CurvaClotoideEntrePuntos([xp(length(xp)-1) xp(length(xp))],[yp(length(yp)-1) yp(length(yp))],cita_temp, dr,vr,dt);
else
    %generar recta hasta el punto n...
    xtramo = [xp(length(xp)-1) (xp(length(xp)-1)+xp(length(xp)))/2 xp(length(xp))];
    ytramo = [yp(length(yp)-1) (yp(length(yp)-1)+yp(length(yp)))/2 yp(length(yp))];
    if(m(length(m)-1) == 1)
        cita_fn = cita_fn +pi; %orientacion virtual...
    end
    %valores de referencia para el tramo recto
    [xt,yt,citat,fit,kt]  = ajustarCurva(xtramo,ytramo,1.3*dr,cita_fn,vr,dt,dr);
end
%copiando trayectoria descrita
offset =length(xg);
for j=offset+1:offset+length(xt)
    xg(j) = xt(j-offset);
    yg(j) = yt(j-offset);
    if(m(length(m)-1) == 1)% correccion de la orientacion virtual...
        citag(j) = citat(j-offset)-pi;
    else
        citag(j) = citat(j-offset);
    end
    kg(j) = kt(j-offset);
    fig(j) = fit(j-offset);
end