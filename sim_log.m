classdef sim_log < handle
    properties
        %% log de posicoes
        posx_log = [];
        posy_log = [];
        posz_log = [];
        %% log de velocidades lineares
        velx_log = [];
        vely_log = [];
        velz_log = [];
        %% log de aceleracoes lineares
        accx_log = [];
        accy_log = [];
        accz_log = [];
        %% log de velocidades angulares dos motores
        omg1_log = [];
        omg2_log = [];
        omg3_log = [];
        omg4_log = [];
        %% log de energias
        ener_pot = [];
        ener_cin = [];
        ener_mec = [];
        %%matriz principal
        mat_log = [];
        %%indice de iteracao
        ind = 0;
    end

    methods
        function record(obj, drone)
            obj.ind = uint64(drone.simt/0.05);

            obj.posx_log(obj.ind) = drone.X;
            obj.posy_log(obj.ind) = drone.Y;
            obj.posz_log(obj.ind) = drone.Z;

            obj.velx_log(obj.ind) = drone.X_1;
            obj.vely_log(obj.ind) = drone.Y_1;
            obj.velz_log(obj.ind) = drone.Z_1;

            obj.accx_log(obj.ind) = drone.X_2;
            obj.accy_log(obj.ind) = drone.Y_2;
            obj.accz_log(obj.ind) = drone.Z_2;

            obj.omg1_log(obj.ind) = drone.omega_1;
            obj.omg2_log(obj.ind) = drone.omega_2;
            obj.omg3_log(obj.ind) = drone.omega_3;
            obj.omg4_log(obj.ind) = drone.omega_4;

            obj.ener_pot(obj.ind) = drone.m*drone.g*drone.Z;
            mod_vel = (obj.velx_log(obj.ind).^2 + obj.vely_log(obj.ind).^2 + obj.velz_log(obj.ind).^2).^(0.5);
            obj.ener_cin(obj.ind) = (drone.m*mod_vel.^2)/2;
            obj.ener_mec(obj.ind) = obj.ener_pot(obj.ind) + obj.ener_cin(obj.ind);
        end

        function save_2_mat(obj, drone)
            obj.mat_log(obj.ind, 1) = drone.simt;

            obj.mat_log(obj.ind, 2) = obj.posx_log(obj.ind);
            obj.mat_log(obj.ind, 3) = obj.posy_log(obj.ind);
            obj.mat_log(obj.ind, 4) = obj.posz_log(obj.ind);

            obj.mat_log(obj.ind, 5) = obj.velx_log(obj.ind);
            obj.mat_log(obj.ind, 6) = obj.vely_log(obj.ind);
            obj.mat_log(obj.ind, 7) = obj.velz_log(obj.ind);

            obj.mat_log(obj.ind, 8) = obj.accx_log(obj.ind);
            obj.mat_log(obj.ind, 9) = obj.accy_log(obj.ind);
            obj.mat_log(obj.ind, 10) = obj.accz_log(obj.ind);

            obj.mat_log(obj.ind, 11) = obj.omg1_log(obj.ind);
            obj.mat_log(obj.ind, 12) = obj.omg2_log(obj.ind);
            obj.mat_log(obj.ind, 13) = obj.omg3_log(obj.ind);
            obj.mat_log(obj.ind, 14) = obj.omg4_log(obj.ind);

            obj.mat_log(obj.ind, 15) = obj.ener_pot(obj.ind);
            obj.mat_log(obj.ind, 16) = obj.ener_cin(obj.ind);
            obj.mat_log(obj.ind, 17) = obj.ener_mec(obj.ind);
        end

        function save_2_txt(obj)
            fileID = fopen('log.txt','wt');
            fprintf(fileID, ['time posx posy posz velx vely velz accx accy accz omg1 omg2 omg3 omg4 epot ecin emec \r\n']);
            fprintf(fileID,'%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f \r\n', obj.mat_log');
            fclose(fileID);
        end

        function show_graph(obj, drone, graph_var)
            if graph_var == 1
                y1 = obj.posx_log;
                y2 = obj.posy_log;
                y3 = obj.posz_log;
                title1 = 'posicao x pelo tempo';
                title2 = 'posicao y pelo tempo';
                title3 = 'posicao z pelo tempo';
            elseif graph_var == 2
                y1 = obj.velx_log;
                y2 = obj.vely_log;
                y3 = obj.velz_log;
                title1 = 'velocidade x pelo tempo';
                title2 = 'velocidade y pelo tempo';
                title3 = 'velocidade z pelo tempo';
            elseif graph_var == 3
                y1 = obj.accx_log;
                y2 = obj.accy_log;
                y3 = obj.accz_log;
                title1 = 'aceleracao x pelo tempo';
                title2 = 'aceleracao y pelo tempo';
                title3 = 'aceleracao z pelo tempo';
            elseif graph_var == 4
                y1 = obj.ener_pot;
                y2 = obj.ener_cin;
                y3 = obj.ener_mec;
                title1 = 'energia potencial pelo tempo';
                title2 = 'energia cinetica pelo tempo';
                title3 = 'energia mecanica pelo tempo';
            end

            if graph_var < 5
                subplot(3,1,1);
                x = 0:0.05:drone.simt;
                plot(x,y1)
                title(title1)

                subplot(3,1,2);
                plot(x,y2)
                title(title2)

                subplot(3,1,3);
                plot(x,y3)
                title(title3)
            end

            if graph_var == 5
                subplot(4,1,1);
                x = 0:0.05:drone.simt;
                plot(x,obj.omg1_log)
                title('omega1 pelo tempo')

                subplot(4,1,2);
                plot(x,obj.omg2_log)
                title('omega_2 pelo tempo')

                subplot(4,1,3);
                plot(x,obj.omg3_log)
                title('omega_3 pelo tempo')

                subplot(4,1,4);
                plot(x,obj.omg4_log)
                title('omega_4 pelo tempo')
            end
        end
    end
end
