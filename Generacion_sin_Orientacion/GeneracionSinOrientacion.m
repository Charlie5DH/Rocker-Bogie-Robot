function[xref,yref,citaref,firef,kref,signmov] = GeneracionSinOrientacion(x,y,cita_i,cita_f,dr,vr,dt)
%Algoritmo de generacion completo, incluye los giros de orientacion al
%inicio y final de la trayectoria.

% (dm_min) distancia min. por los ejes que puede haber entre dos estados
% consecutivos.
f = 1.1*dr; 
% Resolucion min para poder trabajar con el metodo propuesto.
% Re = 2*f;

%angulo inicial de salida de la trayectoria planificada. 
%(si no coincide con cita_i hay que realizar un giro para alcanzarlo)
cita_inicial =  atan2((y(2)- y(1)),(x(2)-x(1)));
if( cita_inicial < 0) 
    cita_inicial = 2*pi + cita_inicial; 
end

%ajustando los angulos inicial y final al rango 0 - 2pi. En caso de
%necesidad de ajuste despues hay que corregir para lograr una continuidad
%en el angulo.
cita_ip = AngRango0a2pi(cita_i);
correccion = false;
if(cita_ip~=cita_i)
    correccion = true;
end

%comprobando si hay que reorientarse.(Realizar giro).
Reorientacion_Inicial = 0;
% if(abs(cita_inicial - cita_i)>1*pi/180 & abs(cita_inicial - cita_i) ~= 2*pi)
%     [xref,yref,citaref,firef,kref,signmov]=GiroOrientacion(x(1),y(1),cita_ip,cita_inicial,dr,vr,dt);
%     Reorientacion_Inicial = 1;
% end

% tramo inicial
xtramo = [x(1), x(1)+(x(2)-x(1))/4, (x(2)+x(1))/2];
ytramo = [y(1), y(1)+(y(2)-y(1))/4, (y(2)+y(1))/2];

%valores de referencia para el tramo recto
[xt,yt,citat,fit,kt,signmovt] = ajuste(xtramo,ytramo,f,cita_inicial,vr,dt,dr);
cita_inicial = citat(length(citat));

%copiando valores para los vectores de trayectoria
if(Reorientacion_Inicial == 1)
    offset =length(xref); 
else
    offset = 0;
end
for j=offset+1:offset+length(xt)
    xref(j) = xt(j-offset);
    yref(j) = yt(j-offset);
    citaref(j) = citat(j-offset);
    firef(j) = fit(j-offset);
    kref(j) = kt(j-offset);
    signmov(j) = signmovt(j-offset);
end

% tramos de ajuste de curvas...
for i=2:length(x)-1
   
    xtramo = [x(i-1)+(x(i)-x(i-1))/2 x(i) x(i)+(x(i+1)-x(i))/2];
    ytramo = [y(i-1)+(y(i)-y(i-1))/2 y(i) y(i)+(y(i+1)-y(i))/2];
    %valores de referencia para el tramo 
    [xt,yt,citat,fit,kt,signmovt] = ajuste(xtramo,ytramo,f,cita_inicial,vr,dt,dr);
    %copiando valores para los vectores de trayectoria
    offset =length(xref); 
    for j=offset+1:offset+length(xt)
        xref(j) = xt(j-offset);
        yref(j) = yt(j-offset);
        citaref(j) = citat(j-offset);
        firef(j) = fit(j-offset);
        kref(j) = kt(j-offset);
        signmov(j) = signmovt(j-offset);
    end
    
    cita_inicial = citaref(length(citaref));
end

% tramo final
xtramo = [(x(length(x)-1)+x(length(x)))/2, (x(length(x)-1)+3*x(length(x)))/4, x(length(x))];
ytramo = [(y(length(y)-1)+y(length(y)))/2, (y(length(y)-1)+3*y(length(y)))/4, y(length(y))];
%valores de referencia para el tramo
[xt,yt,citat,fit,kt,signmovt] = ajuste(xtramo,ytramo,f,cita_inicial,vr,dt,dr);

%copiando valores para los vectores de trayectoria
offset =length(xref);
for j=offset+1:offset+length(xt)
    xref(j) = xt(j-offset);
    yref(j) = yt(j-offset);
    citaref(j) = citat(j-offset);
    firef(j) = fit(j-offset);
    kref(j) = kt(j-offset);
    signmov(j) = signmovt(j-offset);
end

%angulo de llegada ideal
cita_llegada_ideal =  atan2((y(length(y))- y(length(y)-1)),(x(length(x))-x(length(y)-1)));
if( cita_llegada_ideal < 0) 
    cita_llegada_ideal = 2*pi + cita_llegada_ideal; 
end
%angulo de llegada 
cita_llegada = citaref(length(citaref)); 
cita_llegada = AngRango0a2pi(cita_llegada); %pudiera pensasrse en corregir cuadrantes....para lograr continuidad...
%comprobando el error. Si es menor que 0.01 se puede asumir el ideal
if(abs(cita_llegada_ideal - cita_llegada)< 2*pi/180)
    cita_llegada = cita_llegada_ideal;      
elseif(cita_llegada_ideal == 0)
    if(abs(2*pi - cita_llegada)< 0.01)
        cita_llegada = 2*pi;
    end    
end
%comprobando si el angulo de llegada coincide con el final.
% sin no coinciden hay que Reorientarse.

% if(abs(cita_llegada - cita_f)>1*pi/180 & abs(cita_llegada - cita_f) ~= 2*pi)
%     [xt,yt,citat,fit,kt,signmovt]=GiroOrientacion(x(length(x)),y(length(y)),cita_llegada ,cita_f,dr,vr,dt);
%     %copiando valores para los vectores de trayectoria
%     offset =length(xref);
%     for j=offset+1:offset+length(xt)
%         xref(j) = xt(j-offset);
%         yref(j) = yt(j-offset);
%         citaref(j) = citat(j-offset);
%         firef(j) = fit(j-offset);
%         kref(j) = kt(j-offset);
%         signmov(j) = signmovt(j-offset);
%     end
% end

%correcion en la orientacion para lograr continuidad
if correccion
    deltacorreccion = cita_i - cita_ip;
    for j= 1: length(citaref)
        citaref(j) = deltacorreccion + citaref(j);
    end
end
    