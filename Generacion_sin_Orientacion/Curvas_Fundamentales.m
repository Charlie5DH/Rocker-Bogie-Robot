
C = 1;
if(C == 1)
  Pinit = [2 2];
  Pfin = [3 3];
  Pv = [3 2];
end
if (C == 3)
  Pinit = [2 2];
  Pfin = [4 2];
  Pv = [3 3];
end
if (C == 4)
  Pinit = [2 2];
  Pfin = [4 2];
  Pv = [3 2];
end
if (C == 2)
  Pinit = [2 2];
  Pfin = [4 3];
  Pv = [3 2];
end

fi_max = pi/3;
dr = 1;
K_rest_max = (1/dr)*tan(fi_max);

var_cita = atan2(Pfin(2) - Pv(2),Pfin(1)-Pv(1)) - atan2(Pv(2) - Pinit(2),Pv(1)-Pinit(1));

if(C == 1 || C == 3 || C == 4)
    BETA = atan2(Pfin(2) - Pinit(2),Pfin(1)-Pinit(1));
    ALFA = ( atan2(Pfin(2) - Pv(2),Pfin(1)-Pv(1)) - atan2(Pv(2) - Pinit(2),Pv(1)-Pinit(1)) )/2;
    if(ALFA < 0)
       ALFA = ALFA + pi/2; 
    end
    t = sqrt(2*abs(ALFA)/pi);
    
    R = (0.506*t + 1) / ( (1.79*(t^2))+2.054*t + sqrt(2));
    A = 1/(0.803*(t^3) + 1.886*(t^2) + 2.52*t + 2);
    Fc = 0.5 - (R*sin(0.5*pi*(A-t^2)));
    Fs = 0.5 - (R*cos(0.5*pi*(A-t^2)));
    D = cos(ALFA)*Fc + sin(ALFA)*Fs;
    r = sqrt(((Pfin(2)-Pinit(2))^2) + ((Pfin(1)-Pinit(1))^2));                    %distancia euclidiana
    SIGMA = 4*pi*sign(ALFA)*((D^2)/(r^2));
    SIGMA_rest_max = (1*(tan(fi_max)^2))/(2*ALFA*(dr^2));
    L1 = 2*sqrt(2*ALFA/SIGMA);
    f1 = sqrt(SIGMA/SIGMA_rest_max);
    fc1 = f1;
    
else
    BETA = atan2(q(2) - Pinit(2),q(1)-Pinit(1));
    ALFA_1q = BETA - atan2(Pv(2) - Pinit(2),Pv(1)-Pinit(1));
    ALFA_q2 = atan2(Pfin(2) - Pv(2),Pfin(1)-Pv(1)) - BETA;
    
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
    r = sqrt(((Pfin(2)-Pinit(2))^2) + ((Pfin(1)-Pinit(1))^2));                    %distancia euclidiana
        
    SIGMA_1 = 4*pi*sign(ALFA_1q)*((D1q^2)/(r^2));
    SIGMA_2 = 4*pi*sign(ALFA_q2)*((Dq2^2)/(r^2));
    L1 = 2*sqrt(2*ALFA_1q/SIGMA_1);
    L2 = 2*sqrt(2*ALFA_q2/SIGMA_2);
    
    SIGMA_rest_max_1 = (1*(tan(fi_max)^2))/(2*ALFA_1q*(dr^2));
    SIGMA_rest_max_2 = (1*(tan(fi_max)^2))/(2*ALFA_q2*(dr^2));
    
    f1 = sqrt(SIGMA_1/SIGMA_rest_max_1);                          %factor de escala curva 1
    f2 = sqrt(SIGMA_2/SIGMA_rest_max_2);                          %factor de escala curva 2
    Costo_q = f1^2 + f2^2;                                        %funcion cde costo para suavizar el camino
    fc2 = max(f1,f2);
end
    
%     s = 0:0.01:10;
%     k = ((s>=0) & (s<L1/2)).*(SIGMA_1*s) + ((s >= L1/2) &(s < L1)).*(SIGMA_1*(L1-s));
%     plot(s,k);
%clear ALFA_1q ALFA_q2 t1q tq2 R1q Rq2 A1q Aq2 Fc_1q Fc_q2 Fs_1q Fs_q2 D1q Dq2
