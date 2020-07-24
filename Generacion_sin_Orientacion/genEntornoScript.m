clear all;
clc;
%se asume que el robot se mueve en un entorno bidimensional
%se considera como funcion de costo la distancia
%en el mapa 1 significara espacio permitido, 0 prohibido
%todas las medidas en cm
large = 1200;%cm
wide = 1200;%cm
dr = 30;%cm
f = 1.1*dr;
Re = 2*f;%cm

Pini = [2 2];

for i = 1: 1:large/Re
    for j = 1: 1: wide/Re
        mapMatrix(i,j) = 1;
    end
end

M_size = size(mapMatrix);
x_init = Pini(1);
y_init = Pini(2);

%definir obstaculos
for i = 1: 1:(M_size(1))
    for j = 1: 1: (M_size(2))
        if(i == 1 || j == 1 || i == M_size(1) || j == M_size(2))
            mapMatrix(i,j) = 0;
        end 
        if(j == M_size(2)/2 && i < M_size(1)/2)
            mapMatrix(i,j) = 0;
        end
        if(j == M_size(2)/4 -1 && i < M_size(1)/4)
            mapMatrix(i,j) = 0;
        end
        if(i > M_size(1) -7 && j == M_size(2) -6)
            mapMatrix(i,j) = 0;
        end
        if(i == M_size(1) -12 && j > M_size(2) -4)
            mapMatrix(i,j) = 0;
        end
        if(i >= 13 && i <=14 && j > 4 && j < 12)
            mapMatrix(i,j) = 0;
        end
        if(j == x_init && i == y_init)
            mapMatrix(i,j) = 1;
        end
        if(j == 3 && i >= 7 && i <= 8)
            mapMatrix(i,j) = 0;
        end
        if(j >= 13 && j <= 14 && i == 10)
            mapMatrix(i,j) = 0;
        end
    end
end

spy(mapMatrix);
grid on;
xticks(0:1:large/Re);
yticks(0:1:wide/Re);
axis([0 wide/Re 0 large/Re]);
