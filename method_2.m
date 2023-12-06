%% ASV control - Method 2 - Lapierre 

clear
close all
clc

%% Simulation time definition
ts = 1e-4;
times = (0:ts:50)';

% Simulation parameters
a = 2;
pd = @(t) [a*sqrt(2)*cos(t)./(sin(t).^2+1);...
            a*sqrt(2)*cos(t).*sin(t)./(sin(t).^2+1)];

% syms t; pd(t) % derivatives computed symbolocally
pd_dot = @(t) [                                        - (2*2^(1/2).*sin(t))./(sin(t).^2 + 1) - (4*2.^(1/2)*cos(t).^2.*sin(t))./(sin(t).^2 + 1).^2;
                (2*2.^(1/2).*cos(t).^2)./(sin(t).^2 + 1) - (2*2.^(1/2).*sin(t).^2)./(sin(t).^2 + 1) - (4*2.^(1/2).*cos(t).^2.*sin(t).^2)./(sin(t).^2 + 1).^2];

pd_ddot = @(t) [    (16.*2.^(1/2).*cos(t).^3.*sin(t).^2)./(sin(t).^2 + 1).^3 - (4*2.^(1/2).*cos(t).^3)./(sin(t).^2 + 1).^2 - (2*2.^(1/2).*cos(t))./(sin(t).^2 + 1) + (12*2.^(1/2).*cos(t).*sin(t).^2)./(sin(t).^2 + 1).^2; 
                    (16*2.^(1/2).*cos(t).^3.*sin(t).^3)./(sin(t).^2 + 1).^3 - (8*2.^(1/2).*cos(t).*sin(t))./(sin(t).^2 + 1) + (12*2.^(1/2).*cos(t).*sin(t).^3)./(sin(t).^2 + 1).^2 - (12*2.^(1/2).*cos(t).^3.*sin(t))./(sin(t).^2 + 1).^2];

% Curvature
k = @(t)  ([1;0]'*pd_dot(t).*[0;1]'*pd_ddot(t) - [1;0]'*pd_ddot(t).*[0;1]'*pd_dot(t))/(sqrt(sum(pd_dot(t).*pd_dot(t),1))).^3;

% R from I to P
R = @(t) [ cos(t)  sin(t); 
          -sin(t)  cos(t)];
R_dot = @(t) [ -sin(t) cos(t);
               -cos(t) -sin(t)];
% Skew matrix
S = @(rp) [0 -rp;
           rp 0];
% R from P to I
R2 = @(t) [ cos(t)  -sin(t); 
            sin(t)  cos(t)];
R2_dot = @(t) [-sin(t) -cos(t);
                cos(t) -sin(t)];

theta = 0.2;
k_delta = 1;

delta = @(y1, u) -theta * tanh(k_delta * y1 .* u);
%delta_dot = @(y1, y1_dot, u, u_dot) -theta * k_delta * (1 - (y1_dot.*u + y1.*u_dot) .* tanh(k_delta .* y1 .* u).^2);
delta_dot = @(y1, y1_dot, u, u_dot) -k_delta * theta * (sech(k_delta * y1 .* u)).^2 .* (y1_dot .* u + y1 .* u_dot);

k1 = 0.11;
k2 = 0.22;
k3 = 0.33;

% k1 = 0.05;
% k2 = 0.5;
% k3 = 2;

%% Algorithm 

x = 3;
y = 1;
psi = pi/4;

% initial inputs
gamma = 0.5;
gamma_dot = 0;
u = 1;
x_dot = u*cos(psi);
y_dot = u*sin(psi);
psi_p_prev = 0;

% % initial inputs
% u = 1;
% r = 0.1;
% gamma_dot = 0;

% state history
state = zeros(4,length(times));
tmp = zeros(5,length(times));
aaa = zeros(7, length(times));
lyap = zeros(2,length(times));

for i = 1:length(times)
    % aux variables
    p = [x; y]; % vehicle 

    % Error computation
    psi_p = atan2([0;1]'*pd_dot(gamma), [1;0]'*pd_dot(gamma));
    if abs(psi_p_prev - psi_p) > pi
        if psi_p_prev - psi_p > 0
            psi_p = psi_p + 2*pi;
        elseif psi_p_prev - psi_p < 0
            psi_p = psi_p - 2*pi;
        end
    end
    psi_p_prev = psi_p;
    ep = R(psi_p) * (p - pd(gamma));

    s1 = ep(1);
    y1 = ep(2);

    psi_e = psi - psi_p;

    % Speed input
    u = norm(pd_dot(gamma));        % verificar 
    u_dot = pd_ddot(gamma)' * pd_dot(gamma) * gamma_dot /norm(pd_dot(gamma)); % confirmado 

    % Angular velocity input
    up = u * cos(psi_e) + k3 * s1;

    % up = norm(pd_dot(gamma)) * gamma_dot;
    gamma_dot = up / norm(pd_dot(gamma));

    %y1_dot = u * sin(psi_e) - gamma_dot * s1; % do prof why
    y1_dot = u * sin(psi_e) - s1 * k(gamma) * up;

    % Needed to lyapunov
    ep_dot = -S(k(gamma) * up) * ep + [u*cos(psi_e); u*sin(psi_e)] - [up; 0];

    psi_til = psi_e - delta(y1,u);
    
    r = k(gamma)*up + delta_dot(y1,y1_dot,u,u_dot) - k1*psi_til ...
        - k2*y1*u*(sin(psi_e)-sin(delta(y1,u)))/psi_til;

    psi_til_dot = r - k(gamma) * up - delta_dot(y1,y1_dot,u,u_dot);

    % Vehicle model
    x_dot = u*cos(psi);
    y_dot = u*sin(psi);
    psi_dot = r;
    
    % Integrate state derivatives
    x = x + x_dot*ts;
    y = y + y_dot*ts;
    psi = psi + r*ts;
    gamma = gamma + gamma_dot*ts;

    % Save to state history
    state(:,i) = [x; y; psi; gamma]; 
    tmp(:,i) = [s1; y1; y1_dot; u; u_dot];
    aaa(:, i) = [ep; ep_dot; psi_til; psi_til_dot; u_dot];

    % Lyapunov function 

    %V2 = 1/2 * ep.^2 + 1/(2*k2) * psi_til^2;
    V2 = 1/2 * ep'*ep + 1/(2*k2) * psi_til^2;
    V2_dot_a = ep' * ep_dot - 1/k2 * psi_til * psi_til_dot ;
    V2_dot_b = ep' * [u * cos(psi_e) - up; u * sin(psi_e)] + ...
        1/k2 * psi_til * (r - k(gamma)*up - delta_dot(y1,y1_dot,u,u_dot));
 
    V2_dot_final = -k3*s1^2 + y1*u*sin(delta(y1,u)) - (k1/k2)*psi_til^2;

    lyap(:,i) = [V2; V2_dot_final];

end

%% Trajectory, psi, gamma 

x = state(1,:);
y = state(2,:);
yaw = state(3,:);
gamma = state(4,:);

t = times';

% Trajetoria 
plot(x,y)
axis equal
hold on 
plot([1;0]'*pd(t), [0;1]'*pd(t), 'Color', '#A2142F','LineWidth', 1.5)
title('Trajectory')
xlabel("x")
ylabel("y")

% Yaw
figure
plot(t, yaw)
title('Vehicle heading, \psi')

% Gamma
figure
plot(t, gamma)
title('\gamma')

% clf

s1 = tmp(1,:);
y1 = tmp(2,:);
y1_dot = tmp(3,:);

figure
plot(t,y1, t,y1_dot, t(2:end),diff(y1)/ts,'--')
title('Verifing derivative of y_1')

u = tmp(4,:);
u_dot = tmp(5,:);

figure
plot(t,u,t,u_dot,t(2:end),diff(u)/ts,'--')
title('Verifing derivative of u')

%% Checking ep_dot

ep = aaa(1:2,:);
ep_dot = aaa(3:4,:);

figure 
plot(t,ep_dot(2,:), t,y1_dot, t(2:end),diff(ep(2,:))/ts,'--')
title('Verifing if y1 im using is the same one from ep')

figure 
plot(t,ep(1,:), t,ep_dot(1,:), t(2:end),diff(ep(1,:))/ts,'--')
title('Verifing derivative of s1 from ep')

figure 
plot(t,ep(2,:), t,ep_dot(2,:), t(2:end),diff(ep(2,:))/ts,'--')
title('Verifing derivative of y1 from ep')

%% Checking til and delta

psi_til = aaa(5,:);
psi_til_dot = aaa(6,:);
u_dot = aaa(7,:);

figure 
plot(t,psi_til, t,psi_til_dot, t(2:end),diff(psi_til)/ts,'--')
title('Verifing derivative of psi til')

figure 
plot(t,psi_e)
title('Verifing derivative of psi e')

deltas = delta(y1,u);
deltas_dot = delta_dot(y1,y1_dot,u,u_dot);
figure 
plot(t,deltas, t,deltas_dot, t(2:end),diff(deltas)/ts,'--')
title('Verifing derivative of delta')

%% Brincar com a Lyapunov

V2 = lyap(1,:);
V2_dot = lyap(2,:);

figure
% plot(t,V2, t,V2_dot, t(2:end),diff(V2)/ts,'--')
plot(t,V2-cumsum(V2_dot)*ts)
legend('V2', 'int v2 dot')
title('Verifing derivative of the Lyapunov function')


%% Verifying derivatives of path

% derivative pd
figure
hold on
subplot(1,2,1)
plot(t,[1 0]*pd_dot(t),t,[1 0]*pd_ddot(t),t(2:end),diff([1 0]*pd_dot(t))/ts,'.')
title('Coordinate x')
subplot(1,2,2)
plot(t,[0 1]*pd_dot(t),t,[0 1]*pd_ddot(t),t(2:end),diff([0 1]*pd_dot(t))/ts,'.')
title('Coordinate y')
legend('pd dot','pd ddot','diff pd dot', 'Location', 'southeast')

% x coordinate
figure
subplot(1,2,1)
plot(t,[1;0]'*pd(t), t,[1;0]'*pd_dot(t), t(2:end),diff([1;0]'*pd(t))./ts,'.')
legend('x','x dot','diff x')
title('Verifying derivative of x')
subplot(1,2,2)
plot(t,[1;0]'*pd_dot(t), t,[1;0]'*pd_ddot(t), t(2:end),diff([1;0]'*pd_dot(t))./ts,'.')
legend('x dot','x ddot','diff x dot')
title('Verifying double derivative of x')

% y coordinate
figure
subplot(1,2,1)
plot(t,[0;1]'*pd(t), t,[0;1]'*pd_dot(t), t(2:end),diff([0;1]'*pd(t))./ts,'.')
legend('y','y dot','diff y')
title('Verifying derivative of y')
subplot(1,2,2)
plot(t,[0;1]'*pd_dot(t), t,[0;1]'*pd_ddot(t), t(2:end),diff([0;1]'*pd_dot(t))./ts,'.')
legend('y dot','y ddot','diff y dot')
title('Verifying double derivative of y dot')

%% what is this for
% plot(t,[2;0]'*pd(t),t,[2;0]'*pd_dot(t),t(2:end),diff([2;0]'*pd(t))./ts,'.')
% plot(t,[2;0]'*pd_dot(t),t,[2;0]'*pd_ddot(t),t(2:end),diff([2;0]'*pd_dot(t))./ts,'.')




