function[SIGMA,SIGMA_rest_max,L,f] = parametrosCurvasBas(C,fi_max,dr,Vr,dt)
	
%se calculan los parametros de las curvas fundamentales
% con las formas siguientes C1 = _|; C2 = _/; C3 = >; C4 = --
    
switch(C)
    case 1
        Pini = [2 2];
        Pfin = [3 3];
        Pv = [3 2];
    case 2
        Pini = [2 2];
        Pfin = [4 3];
        Pv = [3 2];
    case 3
        Pini = [2 2];
        Pv = [3 3];
        Pfin = [2 4];
    case 4
        Pini = [2 2];
        Pfin = [4 2];
        Pv = [3 2];
end

%K_rest_max = (1/dr)*tan(fi_max);
%var_cita = atan2(Pfin(2) - Pv(2),Pfin(1)-Pv(1)) - atan2(Pv(2) - Pinit(2),Pv(1)-Pinit(1));

if(C == 1 || C == 3)                                           %parametros de las curvas fundamentales
    BETA = atan2(Pfin(2) - Pini(2),Pfin(1)-Pini(1));
    BETA = AngRango0a2pi(BETA);
    ALFA = ( atan2(Pfin(2) - Pv(2),Pfin(1)-Pv(1)) - atan2(Pv(2) - Pini(2),Pv(1)-Pini(1)) )/2;
    ALFA = AngRango0a2pi(ALFA);
    
    t = sqrt(2*abs(ALFA)/pi);
    
    R = (0.506*t + 1) / ( (1.79*(t^2))+2.054*t + sqrt(2));
    A = 1/(0.803*(t^3) + 1.886*(t^2) + 2.52*t + 2);
    Fc = 0.5 - (R*sin(0.5*pi*(A-t^2)));
    Fs = 0.5 - (R*cos(0.5*pi*(A-t^2)));
    D = cos(ALFA)*Fc + sin(ALFA)*Fs;
    r = sqrt(((Pfin(2)-Pini(2))^2) + ((Pfin(1)-Pini(1))^2));                    %distancia euclidiana
    SIGMA1 = 4*pi*sign(ALFA)*((D^2)/(r^2));
    SIGMA_rest_max1 = (1*(tan(fi_max)^2))/(2*ALFA*(dr^2));
    f1 = sqrt(SIGMA1/SIGMA_rest_max1);
    
    L1 = 2*sqrt(2*ALFA/SIGMA1);
    L2 = 0;    
    SIGMA2 = 0;
    f2 = 0;
    SIGMA_rest_max2 = 0;
end
if(C == 4) %parametros de la recta
    L1 = sqrt(((Pfin(2)-Pini(2))^2) + ((Pfin(1)-Pini(1))^2));
    L2 = 0;
    SIGMA1 = 0;
    SIGMA2 = 0;
    f1 = 1;
    f2 = 0;
    SIGMA_rest_max1 = 0;
    SIGMA_rest_max2 = 0;
end
if(C == 2)
    cita = [atan2(Pv(2) - Pini(2),Pv(1)-Pini(1)), atan2(Pfin(2) - Pv(2),Pfin(1)-Pv(1))];
    [xr,yr,citar,fir,kr,sigma_C2,L_C2,sigma_max_C2,f_C2] = clotoideBtwPoint(Pini,Pfin,cita,dr,fi_max,Vr,dt);
    SIGMA1 = sigma_C2(1);
    SIGMA2 = sigma_C2(2);
    L1 = L_C2(1);
    L2 = L_C2(2);
    SIGMA_rest_max1 = sigma_max_C2(1);
    SIGMA_rest_max2 = sigma_max_C2(2);
    f1 = f_C2(1);
    f2 = f_C2(2);
end
    L = [L1 L2];
    SIGMA = [SIGMA1 SIGMA2];
    SIGMA_rest_max = [SIGMA_rest_max1 SIGMA_rest_max2];
    f = [f1 f2];
end