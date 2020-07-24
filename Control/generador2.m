%se asume que el robot se mueve en un entorno bidimensional
%se considera como funcion de costo la distancia
%en el mapa 1 significara espacio permitido, 0 prohibido
%todas las medidas en cm
large = 10;
wide = 10;
for i = 1: 1:large
    for j = 1: 1: wide
        mapMatrix(i,j) = 0;
    end
end

%definir obstaculos
for i = 1: 1:(M_size(1))
    for j = 1: 1: (M_size(2))
         if(rand() > 0.85)
             mapMatrix(i,j) = rand();
         end
    end
end