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

wheel_radi = 0.065/2;             %cm
wheel_base = 30;                %cm 
large = 400;          %cm
wide = 400;           %cm
dr = 10;               %cm
Vr = 0.5;                %m/s
dt = 0.02;             %s
fi_max = 60*pi/180;    %rad
f = 1.1*dr;            %cm
Re = 2*f;              %cm


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
    
%     load('xtp1 ytp1');
%     path_x = xtp1;
%     path_y = ytp1;
    
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
    [xref,yref,citaref,firef,kref,signmov] = GeneracionCompleta(path_x,path_y,cita_ini_rob,cita_fin_rob,dr,Vr,dt);
    %CONTROL DE TRAYECTORIA---------------------------------------
    %una vez obtenidas las coordenadas de las trayectorias de referencia, es
    %decir, la trayectoria generada por el proceso de generacion de
    %trayectoria, se pasa al control del robot.
    %El controlador del robot debe seguir los puntos de la trayectoria y dar
    %los valores de u1,u2, que son las señales de control (velocidad angular y
    %velocidad linear del robot, no de las ruedas)
    %el controlador ademas realiza la estabilizacion del punto de
    %referencia
    %el control se realiza mediante el VFO (Vector Field Orientation). Este
    %metodo se puede utilizar en robot con configuracion triciclo y con
    %configuracion ackerman(en este caso con direccion trasera).
    
    Kp = 0.2;                                      %(1/seg) parametro del controlador
    %K1 = 10;                                    %parametro del controlador para TRM
    k_cita = 5;                                  %parametro del controlador
    k_fi = 10;                                   %parametro del controlador
    L = 0.3;                                     %longitud entre las ruedas delanteras y traseras en (m)
    ts =(length(firef)+1)*dt;                    %tiempo de duracion de la trayectoria
    
    %a mediada que aumenta el largo del vehiculo hay que disminuir los
    %valores del controlador
    
    path_x = ordenarArreglo(path_x);
    path_y = ordenarArreglo(path_y);
    xref = ordenarArreglo(xref);
    yref = ordenarArreglo(yref);
    citaref = ordenarArreglo(citaref);
   
    %[u1,u2,x,y,cita,ex,ey,ecita,efi,rWheelVel,lWheelVel] = controladorVFO(path_x,path_y,cita_ini_rob,xref,yref,citaref,firef,signmov,Vr,Kp,K1,dt,ts);
    [u1,u2,x,y,cita,ex,ey,ecita,efi] = frontWheelVFO(path_x,path_y,cita_ini_rob,xref,yref,citaref,firef,signmov,Vr,Kp,k_cita,k_fi,dt,ts,L);
    
    %u1 y u2 senales de control. el robot es modelado 
    %u1 velocidad angular de giro de la rueda.
    %u2 velocidad longitudinal
    
    Vr = 0;                                     %velocidad inicial de la rueda derecha
    Vl = 0;                                     %velocidad inicial de la rudea izquierda
    
    %para obtener las velocidades reales de las ruedas para el movimiento
    %diferencial hay que mapear las entradas de control para las que se
    %disenaron en el modelo real del carro.
    %para un robot de movimiento diferencial las velocidades de las ruedas
    %seran Vr = (2*v + w*l)/(2*R); Vl = (2*v - w*l)/(2*R); donde v = vel
    %lineal, w = vel angular;
    
    
    graficar(large,wide,Re,path_x,path_y,dt,ts,xref,yref,x,y,scaleMap,cita,citaref,u1,u2,ex,ey,ecita,efi); %GRAFICAR
    
end







