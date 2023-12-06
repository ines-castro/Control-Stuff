% initial positions
x0 = 2;
y0 = 1.5;
psi0 = 0;
me_x = [];
me_y = [];

% related to trajectory
r = 1;
v = 2;

% speed profile
u = 0.1;

% tunning parameter
delta_h = 0.1;   % 8 works for 0.1

% start to visualize something
[xt, yt, ts, dt] = eightPath();
path = [xt', yt', ts'];

figure()
plot(path(:,1), path(:,2), 'Color', 'k')
hold on

xlabel("x (m)")
ylabel("y (m)")
title("Trajectory")
xlim([-2, 3])
ylim([-2, 3])
axis equal

for t = ts(1):dt:ts(end)
    me_x = [me_x, x0];
    me_y = [me_y, y0];
    plot(me_x, me_y, 'Color', '#A2142F');   % trajectory
    hold on
    
    vehicle = [x0, y0];
    prev_p = [10,10];
    [P, tP] = findClosestPoint2(path, vehicle, prev_p);
    prev_p = P;
    disp(P)
     
    [T, N, ~] = frameFrenetSerret8(tP);

    [psi_p, e1] = geometricCalculations(T, vehicle, P);
    psi_los = calculatePsiLos(psi_p, e1, delta_h);

    disp(['Psi los: ', num2str(rad2deg(psi_los))]);

    if t > ts(1)
        delete(h_tangent);
        delete(h_vehicle);
        delete(h_closest);
        delete(h_heading);
        delete(h_y1);
    end
    h_tangent = quiver(P(1), P(2), T(1), T(2), 'Color', '#0072BD', 'LineWidth', 2); 
    hold on
    h_vehicle = plot(x0, y0, 'pentagram', 'MarkerSize', 15, 'MarkerFaceColor', '#A2142F', 'MarkerEdgeColor', '#A2142F');
    hold on
    h_closest = plot(P(1), P(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', '#0072BD', 'MarkerEdgeColor', '#0072BD');
    hold on
    h_heading = quiver(x0, y0, cos(psi_los), sin(psi_los), 'Color', '#4DBEEE', 'LineWidth', 2);
    hold on
    h_y1 = quiver(P(1), P(2), e1(1), e1(2), 'Color', '#EDB120', 'LineWidth', 2);

    drawnow;

    %pause(0.5);

    % update position of the vehicle
    psi0 = psi_los;

    [x0, y0] = calculatePosition(x0, y0, u, psi0);

    % de buuuuu ger
    %disp(['psi_e = ', num2str(psi_e), ' | psi_los = ', num2str(psi_los), ' | psi_p = ', num2str(psi_p)]);
    disp('___________________________')

end

%%
function [p, tp] = findClosestPoint(path, vehicle)
    min_dist = Inf;
    for i = 1:size(path, 1)

        pos = [path(i, 1), path(i, 2)]; % position of the point im testing
        distance = norm(vehicle - pos);

        if distance < min_dist
            min_dist = distance;
            p = pos;
            tp = path(i,3);
        end
    end
    disp(['distance of P (', num2str(p), ') is ', num2str(min_dist)]);
end

% batota
function [p, tp] = findClosestPoint2(path, vehicle, prev_p)
    min_dist = Inf;
    for i = 1:size(path, 1)
        pos = [path(i, 1), path(i, 2)]; % position of the point im testing
        distance = norm(vehicle - pos);
    
        if distance < min_dist && distance < norm(prev_p - vehicle)
            min_dist = distance;
            p = pos;
            tp = path(i,3);
        end
    end
    disp(['distance of P (', num2str(p), ') is ', num2str(min_dist)]);
end


% for the circle path 

function [xt, yt] = calculatePosition(x0, y0, u, psi)
    xt = x0 + u * cos(psi);
    yt = y0 + u * sin(psi);
end

function [xt, yt, ts, dt] = circlePath()
    tspan = [0 10];
    numSteps = 5000;
    dt = (tspan(2) - tspan(1)) / numSteps;
    r = 1;
    xt = [];
    yt = [];
    ts = [];
    v = 2;

    for i = 1:numSteps
        t = tspan(1) + (i - 1)* dt;
        x_now = r * cos(v * t);
        y_now = r * sin(v * t);
        ts = [ts, t];
        xt = [xt, x_now];
        yt = [yt, y_now];
    end
end

function [T, N, B] = frameFrenetSerret(t)
    % disp(['Calculating frame for time = ', t]);
    r = 1;
    v = 2;
    vx = -r * v * sin(v * t); 
    vy = r * v * cos(v * t); 
    v = [vx, vy];
    
    % T -> unit tangent vector
    T = v / norm(v);
    dT = [-vy, vx];

    % N -> normal unit vector
    N = dT / norm(dT);

    % B -> binormal unit vector
    B = [0, 0, 1];
end

% for the eight path

function [xt, yt, ts, dt] = eightPath()
    tspan = [0 4];
    numSteps = 10000;
    dt = (tspan(2) - tspan(1)) / numSteps;
    r = 1;
    xt = [];
    yt = [];
    ts = [];
    v = 2;

    for i = 1:numSteps
        t = tspan(1) + (i - 1)* dt;
        x_now = r * cos(v * t);
        y_now = r * sin(2 * v * t);
        ts = [ts, t];
        xt = [xt, x_now];
        yt = [yt, y_now];
    end
end

function [T, N, B] = frameFrenetSerret8(t)
    % disp(['Calculating frame for time = ', t]);
    r = 1;
    v = 2;
    vx = -r * v * sin(v * t); 
    vy = 2 * r * v * cos(2 * v * t); 
    v = [vx, vy];
    
    % T -> unit tangent vector
    T = v / norm(v);
    dT = [-vy, vx];

    % N -> normal unit vector
    N = dT / norm(dT);

    % B -> binormal unit vector
    B = [0, 0, 1];
end

function [psi_p, e1] = geometricCalculations(T, vehicle, P)
    psi_p = atan2(T(2), T(1));
    % matriz de rotacao de I para P
    R_itop = [cos(psi_p) sin(psi_p);
              -sin(psi_p) cos(psi_p)];

    % error
    e1 = R_itop * (vehicle - P)';
    disp(['e1: ', mat2str(e1)]);
end

function psi_los = calculatePsiLos(psi_p, e1, delta_h)
    y1 = e1(2);
    disp(['psi_p: ', num2str(rad2deg(psi_p))]);

    psi_los = psi_p + atan(-y1/delta_h);
end













