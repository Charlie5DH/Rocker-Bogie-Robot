%Aqui se realiza el proceso completo de planificacion y generacion de 
%trayectoria.
%es necesario definir una resolucion para el entorno
%las dimensiones estaran dadas en cm
%el largo del robot es de aproximadamente 30 cm y el ancho de 25cm
%la distancia entre los estados es la distancia minima entre un edstado
%libre y uno ocupado. 
%factor de expansion del entorno de 30cm, el robot sera modelado como una
%circunferencia de radio 30cm
%la distancia entre las ruedas delanteras y las traseras es de 30cm
%el angulo de direccion maximo de las ruedas sera tomado igual a pi/3

%primero se realiza la descomposicion en celdas del entorno. Cada vertice
%de una celda corresponde a un estado(1 permitido, 0 prohibido). El mapa
%tendra forma cuadrada.
%la resolucion tiene que ser mayor que el factor de expansion del entorno,
%por lo menos 2*dmmin
clear;
clc;

large = 400;%cm
wide = 400;%cm
dr = 10;%cm
Vr = 5;%cm/s
dt = 0.001;
fi_max = 60*pi/180;%rad
f = 1.1*dr;
Re = 2*f;%cm

x_init = 2; y_init = 2;
mapMatrix = genEntorno(Re,large,wide,[x_init y_init]);
%se obtiene un terreno de 20*20 en el que cada 50cm hay un estado posible
%para el robot. Estas dimensiones tienen en cuanta el factor de expansion
%del entorno.
%una vez generado el mapa se procede a planificar una trayectoria mediante
%el algoritmo de programacion dinamica de Bellman Ford.
%Debido a la descomposicion en celdas el robot solo pudede realizar 8
%movimientos. El costo de un vertice a otro(pesos) dependeran de la
%distancia en los movimientos.
%del planificador se obtiene una trayectoria

x_fin = 1; y_fin = 1;
while(mapMatrix(x_fin,y_fin) == 0)
    x_fin =  2 + ceil(( floor((large/Re -1)) -2)* rand());
    y_fin =  2 + ceil(( floor((large/Re -1)) -2)* rand());
end

[dist_max,path_x,path_y] = planificadorBellmanFord(mapMatrix,Re,x_init,y_init,x_fin,y_fin);
if(path_x(1) == 0)
    disp('No hay camino posible');      %mostrar que no se encontro camino

else  
    for i = 1: length(mapMatrix)
        for j = 1: length(mapMatrix)
            scaleMap(i*Re,j*Re) = mapMatrix(i,j);
        end
    end
    for i = 1: length(path_x)
        path_x(i) = path_x(i)*Re;
        path_y(i) = path_y(i)*Re;
    end
    
    
    %ahora es necesario obtener los paramteros de las curvas basicas para las
    %dimensiones del robot
    [SIGMA,SIGMA_rest_max,L,f] = parametrosCurvasBas(1,fi_max,dr,Vr,dt);      %curva 1 _|
    C1 = [SIGMA,SIGMA_rest_max,L,f];
    [SIGMA,SIGMA_rest_max,L,f] = parametrosCurvasBas(3,fi_max,dr,Vr,dt);      %curva 3 >
    C3 = [SIGMA,SIGMA_rest_max,L,f];
    [SIGMA,SIGMA_rest_max,L,f] = parametrosCurvasBas(4,fi_max,dr,Vr,dt);      %curva 4 --
    C4 = [SIGMA,SIGMA_rest_max,L,f];
    [SIGMA,SIGMA_rest_max,L,f] = parametrosCurvasBas(2,fi_max,dr,Vr,dt);      %curva 2 _/
    C2 = [SIGMA,SIGMA_rest_max,L,f];
    fci = [max(C1(7),C1(8)), max(C2(7),C2(8)) , max(C3(7),C3(8))];
    dm_min = max(fci);
    
    %orientaciones inicial y final
    cita_ini_rob = 0;
    cita_fin_rob =  atan2((path_y(length(path_y))- path_y(length(path_y)-1)), (path_x(length(path_x))-path_x(length(path_x)-1)));
    if(cita_fin_rob < 0)
        cita_fin_rob = 2*pi + cita_fin_rob;
    end
    [xref,yref,citaref,firef,kref] = GeneracionConOrientacion(path_x,path_y,cita_ini_rob,cita_fin_rob,dr,Vr,dt);
    
    figure(1);
    spy(scaleMap,10);
    hold on;
    p = plot(path_x,path_y,'k');
    p.LineWidth = 1.2;
    grid on;
    hold on;
    p2 = plot(xref,yref,'r');
    
    legend('estados posibles','camino planificado','camino generado');
    
    xticks(0:Re:large);
    yticks(0:Re:wide);
    axis([0 large 0 wide]);
end
