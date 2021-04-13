function set_pos_drone(drone, drone2, drone3)
    clf;
    hold on
    view(120, 120) % angulo de visao
    tam = 10;

    % axis([-tam+x0 tam+x0 -tam+y0 tam+y0 0 3*tam]); % tamanho da janela de visao
    axis([-tam tam -tam tam 0 2*tam]);
    light
    grid on
    rotate3d off
    zoom off

    x0 = drone.X;
    y0 = drone.Y;
    z0 = drone.Z;
    R0 = drone.phi;
    P0 = drone.theta;
    Y0 = drone.psi;

    x02 = drone2.X;
    y02 = drone2.Y;
    z02 = drone2.Z;
    R02 = drone2.phi;
    P02 = drone2.theta;
    Y02 = drone2.psi;

    x03 = drone3.X;
    y03 = drone3.Y;
    z03 = drone3.Z;
    R03 = drone3.phi;
    P03 = drone3.theta;
    Y03 = drone3.psi;

    object = stlread('body_quadcoptero.stl');
    object2 = stlread('body_quadcoptero.stl');
    object3 = stlread('body_quadcoptero.stl');

    robot = patch(object, 'FaceColor', [0 0 1]);
    robot2 = patch(object2, 'FaceColor', [0 0 1]);
    robot3 = patch(object3, 'FaceColor', [0 0 1]);

    coord_x = get(robot, 'XData') + x0;
    coord_y = get(robot, 'YData') + y0;
    coord_z = get(robot, 'ZData') + z0;

    set(robot, 'XData', coord_x);
    set(robot, 'YData', coord_y);
    set(robot, 'Zdata', coord_z);

    rotate(robot, [1 0 0], R0);
    rotate(robot, [0 1 0], P0);
    rotate(robot, [0 0 1], Y0);

    coord_x2 = get(robot2, 'XData') + x02;
    coord_y2 = get(robot2, 'YData') + y02;
    coord_z2 = get(robot2, 'ZData') + z02;

    set(robot2, 'XData', coord_x2);
    set(robot2, 'YData', coord_y2);
    set(robot2, 'Zdata', coord_z2);

    rotate(robot2, [1 0 0], R02);
    rotate(robot2, [0 1 0], P02);
    rotate(robot2, [0 0 1], Y02);


    coord_x3 = get(robot3, 'XData') + x03;
    coord_y3 = get(robot3, 'YData') + y03;
    coord_z3 = get(robot3, 'ZData') + z03;

    set(robot3, 'XData', coord_x3);
    set(robot3, 'YData', coord_y3);
    set(robot3, 'Zdata', coord_z3);

    rotate(robot3, [1 0 0], R03);
    rotate(robot3, [0 1 0], P03);
    rotate(robot3, [0 0 1], Y03);
end
