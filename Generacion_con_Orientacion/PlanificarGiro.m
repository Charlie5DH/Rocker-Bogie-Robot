function [xp,yp,citap,m] = PlanificarGiro(x,y,cita_ir,cita_fr,dr)
% Planifica los puntos por los que debe pasar el robot para realizar el giro de orientacion.
% El giro de orientacion siempre se da respecto al punto x,y. 
% m: indica si el proximo movimiento en en marcha atras. m=1--> marcha
% atras.

% distancia para que todas las curvas de los giros sean realizables y poder
% unirlas con la primera y tercera etapa de los giros.
dm = 1.3*dr; % 

cita_i = AnguloNotable(cita_ir);
cita_f = AnguloNotable(cita_fr);

% calculo de Delta cita para determinar sentido y tipo de giro. 
citaTemp = cita_i;

if(0 < cita_i&&cita_i <= pi/2 && 3*pi/2 <= cita_f&&cita_f < 2*pi) % caso de I y IV cuadrante
    Dcita = 2*pi - cita_f + cita_i;
elseif(0 < cita_f&&cita_f <= pi/2 && 3*pi/2 <= cita_i&&cita_i < 2*pi) % caso de I y IV cuadrante
    Dcita = 2*pi - cita_i + cita_f;
else
    if(cita_i == 0 && pi<cita_f&&cita_f<2*pi)
        cita_i = 2*pi;
    end
    if(cita_f == 0 && pi<cita_i&&cita_i<2*pi)
        cita_f = 2*pi;
    end
    if(cita_i == 2*pi && 0<cita_f&&cita_f<pi)
        cita_i = 0;
    end
    if(cita_f == 2*pi && 0<cita_i&&cita_i<pi)
        cita_f = 0;
    end
    Dcita = abs(cita_i-cita_f); 
end
cita_i = citaTemp;

% calculo del sentido del giro(simetria).
if(cita_i == 0)
    cita_itemp = 2*pi;
else
    cita_itemp = cita_i;
end
if(cita_f + Dcita > 2*pi)
    cita_temp = cita_f + Dcita - 2*pi;
else
    cita_temp = cita_f + Dcita;
end

if( cita_temp  == cita_itemp)
    sentido = -1;
else
    sentido = 1;
end


% Determinacion del tipo de giro y la trayectoria planificada.
xp(1) = x;
yp(1) = y;
citap(1) = cita_ir;
m(1) = 0;


switch Dcita
    case {pi/4,pi/2}
        % Giro 1, Giro 2. Se realizan con el mismo procedimiento.
        % paso 1. Adelantar 
        if(abs(cos(cita_i)) ~= 1)
            xp(2) = x + dm*sqrt(2)*cos(cita_i);
        else
            xp(2) = x + dm*cos(cita_i);
        end

        if(abs(sin(cita_i)) ~= 1)
            yp(2) = y + dm*sqrt(2)*sin(cita_i);
        else
            yp(2) = y + dm*sin(cita_i);
        end
        m(2) = 1;
        citap(2) = cita_i;
         
        % paso 2. Marcha atras en curva.
        cita_itemp = cita_f - pi; % para determinar elpunto al que tengo que ir
        [i,j] = DireccionIJ(cita_itemp);
        xp(3) = x + dm*i; %Re_min*i;
        yp(3) = y + dm*j; %Re_min*j;
        
        % sin limitar cita entre 0 y 2pi
        if(sentido == -1 && cita_i < Dcita && cita_f > 0) 
            cita_f = -(2*pi - cita_f);
        end
        if(sentido == 1 && 2*pi-cita_i < Dcita && cita_i > 0 && cita_i<2*pi) 
            cita_f = 2*pi + cita_f;
        end
        if(sentido == 1 && cita_f == 0)
            cita_f = 2*pi;
        end
        citap(3) = cita_f;        
        m(3) = 0;
        
%         % paso 3. Adelantar en linea recta.
        xp(4) = x;
        yp(4) = y;
        %ajustando angulo final con el mismo signo
        if(cita_f < 0 && cita_fr > 0)
            citap(4) = -2*pi + cita_fr;
        else
            citap(4) = cita_fr;
        end
        m(4) = 0;      
    case {3*pi/4,pi}
        % Giro 3. Giro4 
        
        % paso 1. Adelantar en linea recta.
        if(abs(cos(cita_i)) ~= 1)
            xp(2) = x + dm*sqrt(2)*cos(cita_i);
        else 
            xp(2) = x + dm*cos(cita_i);
        end

        if(abs(sin(cita_i)) ~= 1)
            yp(2) = y +  dm*sqrt(2)*sin(cita_i);
        else
            yp(2) = y + dm*sin(cita_i);
        end
        citap(2) = cita_i;
        m(2) = 1;
        
        % paso 2. Marcha atras en curva.
        cita_itemp = cita_f + sentido*pi/2;
        [i,j] = DireccionIJ(cita_itemp);
        xp(3) = x + dm*i;
        yp(3) = y + dm*j;
        
        if(Dcita == pi)
            citap(3) = cita_i + sentido*pi/2;
        else
            citap(3) = cita_i + sentido*pi/4; 
        end
        m(3) = 0;
        
        % paso 3. Adelantar en curva.
        [i,j] = DireccionIJ(cita_f);
        xp(4) = x + dm*i; %Re_min*i;
        yp(4) = y + dm*j; %Re_min*j;
        
        % sin limitar la disminucion de cita entre 0 y 2pi
        if(sentido == -1 && cita_i < Dcita && cita_f > 0) 
            cita_f = -(2*pi - cita_f);
        end
        % sin limitar el aumento de cita entre 0 y 2pi
        if(sentido == 1 && 2*pi-cita_i < Dcita && cita_i > 0 && cita_i<2*pi) 
            cita_f = 2*pi + cita_f;
        end
        citap(4) = cita_f;
        
        m(4) = 1;
        
%         % paso 4. Marcha atras en recta.
        xp(5) = x;
        yp(5) = y;
        %ajustando angulo final con el mismo signo
        if(cita_f  < 0  && cita_fr > 0)
            citap(5) = -2*pi + cita_fr;
        else
            citap(5) = cita_fr;
        end
        m(5) = 0;
end
