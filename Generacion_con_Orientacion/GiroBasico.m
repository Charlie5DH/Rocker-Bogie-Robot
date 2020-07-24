function[xg,yg,citag,fig,kg] = GiroBasico(xp,yp,xv,yv,m,citap,dr,vr,dt)

% distancia para que todas las curvas de los giros sean realizables y poder
% unirlas con la primera y tercera etapa de los giros.
dm = 1.3*dr; % 

% planificacion de los punto por los que debe pasar el robot para realizar
% el giro.

if(citap(2) ~= citap(length(citap)-1))
    %tramos curvas
    for i=2:length(xp)-2
        xtramo = [xp(i) xv xp(i+1)]; % Pv siempre es x,y. Todos los giros se realizan alredor de x,y
        ytramo = [yp(i) yv yp(i+1)];
    
        if(m(i) == 1) % marcha atras. utilizo angulo de salida virtual
            cita_inicial = citap(i) - pi;
        else
            cita_inicial = citap(i);
        end
    
        %valores de referencia para el tramo 
        [xt,yt,citat,fit,kt] = ajustarCurva(xtramo,ytramo,dm,cita_inicial,vr,dt,dr);
        %copiando valores para los vectores de trayectoria
        if(i == 2)
            offset = 0;
        else
            offset = length(xg); 
        end
        for j=offset+1:offset+length(xt)
            xg(j) = xt(j-offset);
            yg(j) = yt(j-offset);
            if(m(i)==1) %marcha atras. corregir angulo. se utilizo uno virtual.
                citag(j) = citat(j-offset) + pi ;
            else
                citag(j) = citat(j-offset);
            end
            fig(j) = fit(j-offset);
            kg(j) = kt(j-offset);
        end
    end
else
    xg = 0;
    yg = 0;
    citag = citap(1);
    fig = 0;
    kg = 0;
    m = 0;
end
    