function[cita_k] = CitaContinua(cita_p2,cita_p1,cita_k_1)
if abs(cita_p2)>5*pi/180
    if cita_p2 < 0  
        cita_k = ceil(cita_k_1/(2*pi))*2*pi +cita_p2;
    else
        cita_k = floor(cita_k_1/(2*pi))*2*pi +cita_p2;
    end
else
    cita_k = cita_k_1 + cita_p2 - cita_p1;
end
       