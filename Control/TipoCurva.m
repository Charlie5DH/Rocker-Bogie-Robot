function[tipo,invertir,signo] = TipoCurva(x,y,escala)
% x,y: los puntos del intervalo P1, Pv, P2.
% Esta funcion determina cual es el tipo de curva que se ajusta al
% intervalo dado devolviendolo en el argumento tipo.
% Tipos de curva: C1 _| (1) C2 _/ (2) C3 > (3) C4 -- (4) 
% invertir: indica si hay que invertir los valores de sigma y longitud de
% las curvas plantillas.
% signo: indica el signo de sigma.
    
    invertir = 0;
        
    %determinacion de la orientacio
    cita1 = atan2((y(2)-y(1)),(x(2)-x(1)));
    if( cita1 < 0) cita1 = 2*pi + cita1; end
    cita2 = atan2(y(3)-y(2),x(3)-x(2));
    if( cita2 < 0) cita2 = 2*pi + cita2; end
    
    
    

    %determinacion del orden de los parametros y el tipo de curva
    recta1 = sqrt((x(1)-x(2))^2+(y(1)-y(2))^2);
    recta2 = sqrt((x(3)-x(2))^2+(y(3)-y(2))^2);
    c = sqrt((x(3)-x(1))^2+(y(3)-y(1))^2);
       
    alfa = acos(-(c^2-recta1^2-recta2^2)/(2*recta1*recta2));
    
    %el valor de alfa indica cual de los tres tipo de curva usar.
    % los posibles valores son : C1 y  C3(pi/2) C2 (3*pi/4) C4(0, pi y 2*pi)
    if(abs(alfa-3*pi/4) < 0.01)
        tipo = 2;
    end
    
    if(abs(alfa-pi)< pi/60 |abs(alfa-2*pi)< pi/60 |abs(alfa)< pi/60)
        tipo = 4;
    end
    
    if(abs(alfa-pi/2) < 0.01)
        if(abs(recta1-1*escala) > 0.1)
            tipo = 3;
        else
            tipo = 1;
        end
    end
    
    if(recta1 > recta2 & abs(alfa-pi/2) > 0.01)
        invertir = 1;
    end    
    
   
    %determinacion del signo de la curvaturta
    if(cita1 < cita2) 
        signo = 1; 
    else 
        signo = -1; 
    end
    if((3*pi/2 <= cita1&cita1<= 2*pi) & (0<=cita2&cita2<=pi/2))
        signo = 1;
    end
    if((3*pi/2 <= cita2&cita2<= 2*pi) & (0<=cita1&cita1<=pi/2))
        signo = -1;
    end
%     if(cita1 == 0 & cita2 > pi) 
%         signo = -1; 
%     end
%     if(tipo == 3)
%         if(3*pi/2<cita1&cita1<2*pi & 0<cita2&cita2<pi/2)
%             signo = 1;
%         end
%         if(3*pi/2<cita2&cita2<2*pi & 0<cita1&cita1<pi/2)
%             signo = -1;
%         end
%     end
%     if(tipo == 2)
%         if(cita1>=3*pi/2 & (cita2 == 2*pi|cita2 == 0))
%            signo = 1;
%         end
%         if(cita1 == 2*pi & cita2 == pi/4)
%             signo = 1;
%         end
%         if(cita1 == 0 & cita2 == 3*pi/2)
%             signo = -1;
%         end
%     end
%     if(tipo == 1)
%         if(cita1 ==3*pi/2 & (cita2 == 2*pi|cita2 == 0))
%            signo = 1;
%         end
%     end
%     return
end