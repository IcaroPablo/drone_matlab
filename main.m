tempo = 50;
log = sim_log();
drone1 = drone_prop;
drone2 = drone_prop;
drone3 = drone_prop;
fly = fly_toward;

% tempo1 = 0:0.05:tempo;
% desired_pos = [x_path y_path z_path 0];

desired_pos = [5 5 10 0];

while drone1.simt < tempo %true
    if drone1.simt > 30
        desired_pos = [7 7 10 0];
    end

    fly.go_to(drone1, desired_pos, 1)
    drone1.drone_init()
    fly.go_to(drone2, [4 5 10 0], 2)
    fly.go_to(drone3, [4 3 10 0], 3)

    drone2.drone_init()
    drone3.drone_init()
    log.record(drone1)

    set_pos_drone(drone1, drone2, drone3)
    % plot3(log.posx_log(1:log.ind), log.posy_log(1:log.ind), log.posz_log(1:log.ind));
    drawnow;

    log.save_2_mat(drone1)
end
log.show_graph(drone1, 1)
log.save_2_txt()

% plot(tempo1, posyvec)

%       1
%   4   0   2
%       3

%   x
%   ^
%   |
%   1       2
%       0
%   4       3 --->-y
