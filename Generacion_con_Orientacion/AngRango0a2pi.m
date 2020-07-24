function[cita] = AngRango0a2pi(cita)
if(cita > 2*pi)
    cita = cita - floor(cita/(2*pi))*2*pi;
end
if(cita < 0)
    cita =  ceil(-cita/(2*pi))*2*pi + cita;
end