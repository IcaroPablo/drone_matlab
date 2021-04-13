classdef fly_toward < handle

    properties
        z_last_error = [0 0 0];
        a_last_error = [0 0 0];
        ang_last_error = [0 0; 0 0; 0 0];
        last_pos_error = [0 0; 0 0; 0 0];

        p_max = 0.1;
        z_max = 1.8;
        r_max = 1;
        ang_max = 10;
    end

    methods

        function [mov_p] = p_pid(obj, act_pos, des_pos, act_angle, param, droneID)
            if param == 1
                % ganhos do PID para o angulo ideal dependente da distancia
                d_kp = 0.686;%0.686
                d_ki = 0.48;%0.48
                d_kd = 100;%100

                % ganhos do PID para mov_x dependente do angulo ideal
                a_kp = 40;
                a_ki = 15;
                a_kd = 600;
            elseif param == 2
                % ganhos do PID para o angulo ideal dependente da distancia
                d_kp = 0.696;%0.686
                d_ki = 0.48;%0.48
                d_kd = 100;%100

                % ganhos do PID para mov_x dependente do angulo ideal
                a_kp = 40;
                a_ki = 15;
                a_kd = 600;
            end

            % angulo de inclinacao no eixo "proporcional" a distancia
            pos_error = (des_pos - act_pos)*d_kp;
            pos_error_sum = (pos_error + obj.last_pos_error(droneID, param))*d_ki;
            pos_error_dif = (pos_error - obj.last_pos_error(droneID, param))*d_kd;
            obj.last_pos_error(droneID, param) = pos_error;

            ideal_ang = pos_error + pos_error_sum + pos_error_dif;

            % definicao de limites superior e inferior para o angulo ideal
            if ideal_ang > obj.ang_max
                ideal_ang = obj.ang_max;
            elseif ideal_ang < -obj.ang_max
                ideal_ang = -obj.ang_max;
            end

            ang_error = (ideal_ang - act_angle)*a_kp;
            ang_error_sum = (ang_error + obj.ang_last_error(droneID, param))*a_ki;
            ang_error_dif = (ang_error - obj.ang_last_error(droneID, param))*a_kd;
            obj.ang_last_error(droneID, param) = ang_error;

            % calculo do adicional de omega proporcional ao PID do angulo ideal
            mov_p = ang_error + ang_error_sum + ang_error_dif;
            mov_p = mov_p/200;

            if mov_p > obj.p_max
                mov_p = obj.p_max;
            elseif mov_p < -obj.p_max
                mov_p = -obj.p_max;
            end
        end

        function [mov_z] = z_pid(obj, actualZ, desired_z, droneID)

            z_param = 1.566;
            z_kp = 0.5;
            z_ki = 0.1;
            z_kd = 8;

            z_error = (desired_z - actualZ); % erro no eixo z
            z_error_sum = z_error + obj.z_last_error(droneID);
            z_error_dif = (z_error - obj.z_last_error(droneID));
            obj.z_last_error(droneID) = z_error;

            mov_z = z_param + z_kp*z_error + z_ki*z_error_sum + z_kd*z_error_dif;

            if mov_z > obj.z_max
                mov_z = obj.z_max;
            end
        end

        function [mov_r] = a_pid(obj, actualA, desired_a, droneID)

            a_kp = 0.3;
            a_ki = 0.135;
            a_kd = 14;

            a_error = (desired_a - actualA); % erro no angulo (direcao)
            a_error_sum = a_error + obj.a_last_error(droneID);
            a_error_dif = (a_error - obj.a_last_error(droneID));
            obj.a_last_error(droneID) = a_error;

            mov_r = a_kp*a_error + a_ki*a_error_sum + a_kd*a_error_dif;

            if mov_r > obj.r_max
                mov_r = obj.r_max;
            elseif mov_r < -obj.r_max
                mov_r = -obj.r_max;
            end
        end

        function go_to(obj, drone, pos_vec, droneID)
          comb_x = norm([drone.theta -drone.phi]);
          comb_y = norm([drone.theta drone.phi]);

          [mov_x] = obj.p_pid(drone.X, pos_vec(1), drone.theta, 1, droneID);
          [mov_y] = obj.p_pid(drone.Y, pos_vec(2), -drone.phi, 2, droneID);
          [mov_z] = obj.z_pid(drone.Z, pos_vec(3), droneID);
          [mov_r] = obj.a_pid(drone.psi, pos_vec(4), droneID);

          mov_r = 0;

          omega_1 = mov_z - mov_r - mov_x - mov_y;
          omega_2 = mov_z + mov_r - mov_x + mov_y;
          omega_3 = mov_z - mov_r + mov_x + mov_y;
          omega_4 = mov_z + mov_r + mov_x - mov_y;

          drone.def_omega(omega_1, omega_2, omega_3, omega_4);
        end
    end
end
