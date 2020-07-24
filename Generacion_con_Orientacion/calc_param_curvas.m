%para formar las curvas fundamentales C1 C2 C3 C4(linea recta) hay que 
%calcular los parametros de %las clotoides simetricas que las conforman.
%cita_TA contiene los angulos intermedios. Los elementos impares son los
%primeros(cita_1) y los pares los segundos(cita_2)
%los movimientos se realizan entre los puntos de la trayectoria auxiliar.
%El punto P1 sera j-1, Pv sera j y P2 sera j+1 de la trayectoria auxiliar
%TA.
%BETA sera atan2 de P2 y P1
%se puede observar que los angulos que se forman siempre son de 45,90 o 180
%grados, esto se debe al tipo de representacion del entorno seleccionada

dr = 1;                                                     %distancia entre las ruedas delanteras y traseras
fi_max = pi/3;                                                    %angulo de direccion maximo de la rueds
radio_r = 1.22/2;                                              %radio de las ruedas

%alfa = (cita_2 - cita_1)/2; 
%alfa_1 = beta-cita1_1;                                           %variacion en la orientacion de la primera clotoide
%alfa_2 = cita_2-beta;                                            %variacion en la orientacion de la segunda clotoide

%-------------------------------------------------------------------------

K_rest_max = (1/dr)*tan(fi_max);
trayGen_X = TA_x;
trayGen_Y = TA_y;

for i = 1: 1: length(2)*2 -1-2 
    BETA(i) = atan2(TA_y(i+2)-TA_y(i),TA_x(i+2)-TA_x(i));
    %BETA(i) = rad2deg(BETA(i));
    
    %ALFA(i) = (CITA_TA(i+1) - CITA_TA(i))/2;
    ALFA_1q = BETA(i) - CITA_TA(i);
    ALFA_q2 = CITA_TA(i+1) - BETA(i);
    
    t1q = sqrt(2*abs(ALFA_1q)/pi);
    tq2 = sqrt(2*abs(ALFA_q2)/pi);    
    
    R1q = (0.506*t1q + 1) / ( (1.79*(t1q^2))+2.054*t1q + sqrt(2));
    Rq2 = (0.506*tq2 + 1) / ( (1.79*(tq2^2))+2.054*tq2 + sqrt(2));
    A1q = 1/(0.803*(t1q^3) + 1.886*(t1q^2) + 2.52*t1q + 2);
    Aq2 = 1/(0.803*(tq2^3) + 1.886*(tq2^2) + 2.52*tq2 + 2);     
    
    Fc_1q = 0.5 - (R1q*sin(0.5*pi*(A1q-t1q^2)));                        %aproximacion de la integral coseno de Fresnel
    Fc_q2 = 0.5 - (Rq2*sin(0.5*pi*(Aq2-tq2^2)));
    Fs_1q = 0.5 - (R1q*cos(0.5*pi*(A1q-t1q^2)));
    Fs_q2 = 0.5 - (Rq2*cos(0.5*pi*(Aq2-tq2^2)));

    D1q = cos(ALFA_1q)*Fc_1q + sin(ALFA_1q)*Fs_1q;
    Dq2 = cos(ALFA_q2)*Fc_q2 + sin(ALFA_q2)*Fs_q2;
    r(i) = sqrt(((TA_y(i+2)-TA_y(i))^2) + ((TA_x(i+2)-TA_x(i))^2)); %distancia euclidiana
        
    sharpness1(i) = 4*pi*sign(ALFA_1q)*((D1q^2)/(r(i)^2));
    sharpness2(i) = 4*pi*sign(ALFA_q2)*((Dq2^2)/(r(i)^2));
    L1(i) = 2*sqrt(2*ALFA_1q/sharpness1(i));
    L2(i) = 2*sqrt(2*ALFA_q2/sharpness2(i));
    
    sharpness1_rest_max_1(i) = (1*(tan(fi_max)^2))/(2*ALFA_1q*(dr^2));
    sharpness2_rest_max_2(i) = (1*(tan(fi_max)^2))/(2*ALFA_q2*(dr^2));
    
    f1(i) = sqrt(sharpness1(i)/sharpness1_rest_max_1(i));                          %factor de escala curva 1
    f2(i) = sqrt(sharpness2(i)/sharpness2_rest_max_2(i));                          %factor de escala curva 2
    Costo_q(i) = f1(i)^2 + f2(i)^2;                                        %funcion cde costo para suavizar el camino
    
    
end
clear ALFA_1q ALFA_q2 t1q tq2 R1q Rq2 A1q Aq2 Fc_1q Fc_q2 Fs_1q Fs_q2 D1q Dq2 


