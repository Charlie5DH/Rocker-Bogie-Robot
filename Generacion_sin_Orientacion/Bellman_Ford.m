


% Este algoritmo determina la trayectoria minima entre un punto inicial y
% todos los restantes puntos del entorno
% 
% el mapa tiene una resolucion de 0.5m lo que significa que la distancia
% entre estados es de 0.5m con los sig 4 movimientos:
% (i = i+1, j = j) (i = i-1, j = j) (i = i, j = j-1)   (i = i, j = j+1)
% el costo de un estado a otro es 0.5m
% 
% cuando se avanza con un movimiento diagonal(ang = 45) la distancia entre
% estados sera sqr(2*Re^2)->costo del movimiento. Esto es con los
% movimientos m1,m3,m4,m6
% se asume inicio en i=2,j=2

n = M_size(1)*M_size(2);                       %cantidad de estados
weightsMatrix = mapMatrix;                              %Conjunto de todos los estados
Edges = (n - l_terreno*4)*8;                   %cantidad de aristas
costoMov1 = Re;                                %distancia horizontal
costoMov2 = sqrt(2*(Re^2));                    %distancia diagonal

v_end_col = 63;                                %coordenadas de destino
v_end_row = 65;

for i = 1: 1:(M_size(1))                       %inicializando pesos con infinito
    for j = 1: 1: (M_size(2))
        weightsMatrix(i,j) = inf;
    end
end 
Parent_col = weightsMatrix;                 %matriz que almacena las coordenadas de las columnas de los padres
Parent_row = weightsMatrix;                 %matriz que almacena las coordenadas de las filas de los padres

source = [2 2];
weightsMatrix(source(1),source(2)) = 0;           %coordenadas de inicio

for k = 0: 1: n-1
    for i = 2: 1: M_size(1) -1
        for j = 2: 1: M_size(2) -1
            if( mapMatrix(i, j+1) ==1 && (weightsMatrix(i, j+1) > weightsMatrix(i,j) + costoMov1))
                weightsMatrix(i, j+1) = weightsMatrix(i,j) + costoMov1;
                Parent_col(i,j+1) = j;
                Parent_row(i,j+1) = i;
            end
            if( mapMatrix(i, j-1) ==1 && weightsMatrix(i, j-1) > weightsMatrix(i,j) + costoMov1)
                weightsMatrix(i, j-1) = weightsMatrix(i,j) + costoMov1;
                Parent_col(i, j-1) = j;
                Parent_row(i, j-1) = i;
            end
            if( mapMatrix(i+1,j) ==1 && weightsMatrix(i+1, j) > weightsMatrix(i,j) + costoMov1)
                weightsMatrix(i+1, j) = weightsMatrix(i,j) + costoMov1;
                Parent_col(i+1, j) = j;
                Parent_row(i+1, j) = i;
            end
            if( mapMatrix(i-1, j) ==1 && weightsMatrix(i-1, j) > weightsMatrix(i,j) + costoMov1)
                weightsMatrix(i-1, j) = weightsMatrix(i,j) + costoMov1;
                Parent_col(i-1, j) = j;
                Parent_row(i-1, j) = i;
            end
            if( mapMatrix(i+1, j+1) ==1 && weightsMatrix(i+1, j+1) > weightsMatrix(i,j) + costoMov2)
                weightsMatrix(i+1, j+1) = weightsMatrix(i,j) + costoMov2;
                Parent_col(i+1, j+1) = j;
                Parent_row(i+1, j+1) = i;
            end
            if( mapMatrix(i-1, j+1) ==1 && weightsMatrix(i-1, j+1) > weightsMatrix(i,j) + costoMov2)
                weightsMatrix(i-1, j+1) = weightsMatrix(i,j) + costoMov2;
                Parent_col(i-1, j+1) = j;
                Parent_row(i-1, j+1) = i;
            end
            if( mapMatrix(i+1, j-1) ==1 && weightsMatrix(i+1, j-1) > weightsMatrix(i,j) + costoMov2)
                weightsMatrix(i+1, j-1) = weightsMatrix(i,j) + costoMov2;
                Parent_col(i+1, j-1) = j;
                Parent_row(i+1, j-1) = i;
            end
            if( mapMatrix(i-1, j-1) ==1 && weightsMatrix(i-1, j-1) > weightsMatrix(i,j) + costoMov2)
                weightsMatrix(i-1, j-1) = weightsMatrix(i,j) + costoMov2;
                Parent_col(i-1, j-1) = j;
                Parent_row(i-1, j-1) = i;
            end
        end
    end
end

dist_max = weightsMatrix(v_end_row,v_end_col);          %distancia del recorrido

C = 2;
path_x(1) = v_end_col;
path_y(1) = v_end_row;
i = v_end_row;                              %coordenadas fila pto final
j = v_end_col;                              %coordenadas col pto final

while(i ~= source(1) && j ~= source(2))                        %conformacion del camino
    path_x(C) = Parent_col(i,j);
    path_y(C) = Parent_row(i,j);
    aux = [i,j];
    i = Parent_row(aux(1),aux(2));
    j = Parent_col(aux(1),aux(2));
    C = C+1;
end
path_x(C) = Parent_col(i,j);
path_y(C) = Parent_row(i,j);

clear C;
spy(mapMatrix);
hold on;
p = plot(path_x,path_y,'k');
p.LineWidth = 1.5;
grid on;
xticks(0:1:a_terreno/Re);
yticks(0:1:a_terreno/Re);
axis([0 a_terreno/Re 0 a_terreno/Re]);

