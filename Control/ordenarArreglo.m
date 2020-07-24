function [array] = ordenarArreglo(array)

size = length(array);
C = 0;
for i = 1: size
    aux(i) = array(size-C);
    C = C+1;
end
array = aux;

end

