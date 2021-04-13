classdef path < handle

    properties
        centro_ter = [0 0]
        raio_ter = 20 %em km
        dist_max_drone = 4 %em km
        raio_camera = 
    end

    methods
        function def_points(obj)
            compmax = 2*pi*obj.raiomax
            for part in range(1:1:raiomax)
                if raiomax/dist < dist_max_drone
                    num_part_raio = raiomax/dist
                    break

        end
    end

input:
    centro
    raio do terreno
    raio do campo de visão da camera
    dist_max_drone %obs pode ser substituido por desempenho da bateria
