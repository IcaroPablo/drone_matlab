classdef drone_prop < handle

    properties
        %% posicoes em E
        X = 0;
        Y = 0;
        Z = 0;
        %% velocidades lineares em E
        X_1 = 0;
        Y_1 = 0;
        Z_1 = 0;
        %% aceleracoes lineares em E
        X_2 = 0;
        Y_2 = 0;
        Z_2 = 0;
        %% angulos em B
        phi = 0;
        theta = 0;
        psi = 0;
        %% velocidades angulares em B
        p = 0; % phi_1
        q = 0; % theta_1
        r = 0; % psi_1
        %% aceleracoes angulares em B
        p_1 = 0; % phi_2
        q_1 = 0; % theta_2
        r_1 = 0; % psi_2
        %% velocidade angular dos motores
        omega_1 = 0;
        omega_2 = 0;
        omega_3 = 0;
        omega_4 = 0;
        %% constantes
        simt = 0; %tempo de simulacao atual
        t = 0.05; % tempo de atualizacao das posicoes
        c = 1; % constante de proporcionalidade entre omega e U
        d = 1; % constante de arrasto
        l = 5; % comprimento do braco
        m = 1; % massa
        g = 9.8; % aceleracao da gravidade
        I_x = 1; % momento de inercia no eixo x
        I_y = 1; % momento de inercia no eixo y
        I_z = 1; % momento de inercia no eixo z
        J_t = 1; % momento de inercia total
    end

    methods
        function def_omega(obj, v1, v2, v3, v4)
            %% recebendo valores de velociades angulares

            if v1 < 0
                v1 = 0;
            end
            if v2 < 0
                v2 = 0;
            end
            if v3 < 0
                v3 = 0;
            end
            if v4 < 0
                v4 = 0;
            end

            obj.omega_1 = v1;
            obj.omega_2 = v2;
            obj.omega_3 = v3;
            obj.omega_4 = v4;
          end

        function drone_init(obj)
            %% calculo dos impulsos
            U1 = obj.c*(obj.omega_1.^2 + obj.omega_2.^2 + obj.omega_3.^2 + obj.omega_4.^2);
            U2 = obj.c*obj.l*(obj.omega_4.^2-obj.omega_2.^2);
            U3 = obj.c*obj.l*(obj.omega_3.^2-obj.omega_1.^2);
            U4 = obj.d*(-obj.omega_1.^2+obj.omega_2.^2-obj.omega_3.^2+obj.omega_4.^2);

            %% calculo das aceleracoes lineares
            obj.X_2 = (sind(obj.psi)*sind(obj.phi)+cosd(obj.psi)*sind(obj.theta)*cosd(obj.phi))*U1/obj.m;
            obj.Y_2 = (-cosd(obj.psi)*sind(obj.phi)+sind(obj.psi)*sind(obj.theta)*cosd(obj.phi))*U1/obj.m;
            obj.Z_2 = -obj.g + (cosd(obj.theta)*cosd(obj.phi))*U1/obj.m;

            %% calculo das aceleracoes angulares
            obj.p_1 = ((obj.I_y - obj.I_z)/obj.I_x)*obj.q*obj.r - (obj.J_t/obj.I_x)*obj.q*(-obj.omega_1+obj.omega_2-obj.omega_3+obj.omega_4) + U2/obj.I_x;
            obj.q_1 = ((obj.I_z - obj.I_x)/obj.I_y)*obj.p*obj.r + (obj.J_t/obj.I_y)*obj.p*(-obj.omega_1+obj.omega_2-obj.omega_3+obj.omega_4) + U3/obj.I_y;
            obj.r_1 = ((obj.I_x - obj.I_y)/obj.I_z)*obj.p*obj.q + U4/obj.I_z;

            %% atualizar velocidades lineares
            obj.X_1 = obj.X_1 + obj.X_2*obj.t;
            obj.Y_1 = obj.Y_1 + obj.Y_2*obj.t;
            obj.Z_1 = obj.Z_1 + obj.Z_2*obj.t;

            %% atualizar velocidades angulares
            obj.p = obj.p + obj.p_1*obj.t;
            obj.q = obj.q + obj.q_1*obj.t;
            obj.r = obj.r + obj.r_1*obj.t;

            % atualizar posicoes lineares
            obj.X = obj.X + obj.X_1*obj.t + (obj.X_2*obj.t.^2)/2;
            obj.Y = obj.Y + obj.Y_1*obj.t + (obj.Y_2*obj.t.^2)/2;
            obj.Z = obj.Z + obj.Z_1*obj.t + (obj.Z_2*obj.t.^2)/2;

            % atualizar posicoes angulares
            obj.phi = obj.phi + obj.p*obj.t + (obj.p_1*obj.t.^2)/2;
            obj.theta = obj.theta + obj.q*obj.t + (obj.q_1*obj.t.^2)/2;
            % obj.psi = obj.psi + obj.r*obj.t + (obj.r_1*obj.t.^2)/2;
            obj.psi = 45;

            if obj.Z < 0
                obj.Z = 0;
            end

            % set_pos_drone(obj.X, obj.Y, obj.Z, obj.phi, obj.theta, obj.psi)
            obj.simt = obj.simt + obj.t;
        end
    end
end
