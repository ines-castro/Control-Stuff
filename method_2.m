%% ASV control - Method 2 - Lapierre 

clear
close all
clc

%% First part 

ts = 1e-4;
times = (0:ts:50)';

% Simulation parameters
a = 5;
pd = @(t) [a*sqrt(2)*cos(t)./(sin(t).^2+1);...
            a*sqrt(2)*cos(t).*sin(t)./(sin(t).^2+1)];

% Derivatives computed symbolocally
pd_dot = @(t) [                                        - (2.^(1/2).*a.*sin(t))/(sin(t).^2 + 1) - (2*2.^(1/2).*a.*cos(t).^2.*sin(t))/(sin(t).^2 + 1).^2;
                (2.^(1/2).*a.*cos(t).^2)/(sin(t).^2 + 1) - (2.^(1/2).*a.*sin(t).^2)/(sin(t).^2 + 1) - (2.*2.^(1/2).*a.*cos(t).^2.*sin(t).^2)/(sin(t).^2 + 1).^2];

pd_ddot = @(t) [    (6*2.^(1/2).*a.*cos(t).*sin(t).^2)/(sin(t).^2 + 1).^2 - (2.^(1/2).*a.*cos(t))/(sin(t).^2 + 1) - (2*2.^(1/2).*a.*cos(t).^3)/(sin(t).^2 + 1).^2 + (8*2.^(1/2).*a.*cos(t).^3.*sin(t).^2)/(sin(t).^2 + 1).^3; 
                   (6*2.^(1/2).*a.*cos(t).*sin(t).^3)/(sin(t).^2 + 1).^2 - (6*2.^(1/2).*a.*cos(t).^3.*sin(t))/(sin(t).^2 + 1).^2 + (8.*2.^(1/2).*a.*cos(t).^3.*sin(t).^3)/(sin(t).^2 + 1).^3 - (4*2.^(1/2).*a.*cos(t).*sin(t))/(sin(t).^2 + 1)];

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

theta = 0.2;
k_delta = 1;

delta = @(y1, u) -theta * tanh(k_delta * y1 .* u);
delta_dot = @(y1, y1_dot, u, u_dot) -k_delta * theta * (sech(k_delta * y1 .* u)).^2 .* (y1_dot .* u + y1 .* u_dot);

k1 = 10;
k2 = 5;
k3 = 0.5;

%% Incorporating wind and currents 

X_udot = -21.8;                 % K g
Y_vdot = -608.1;                % K g
N_rdot = -364.5;                % K gm^2

m = 400;                        % K g

% inertia momentum around the z_c azis
Iz = 326;                       % k gm^2

M = diag([(m - X_udot) (m - Y_vdot) (Iz - N_rdot)]);

C = @(r) [0 -m*r 0; m*r 0 0; 0 0 0];

X_u = - 0.5;                    % K gs^-1
X_usquared = -7.6;              % K gm^-2 s
Y_vvc = -581.2;                 % K gm^−1
N_r = -0.26;                    % K gm^2 s^−1
N_rr = -1764.2;                 % K gm^2

D = @(u, r) diag([(X_u + X_usquared * u) (Y_vvc) (N_r + N_rr * abs(r))]);

m_u = m - X_udot;
m_v = m - Y_vdot;
m_r = Iz - N_rdot;

d_u = @(u) - X_u - (X_usquared * abs(u)); 
d_v = @(v) - Y_vvc ; 
d_r = @(r) - N_r - (N_rr * abs(r)); 

% model
surge_dot = @(tau_u, u) (1/m_u) * (tau_u - d_u(u) * u);
sway_dot = @(tau_v, v) (1/m_v) * (tau_v - d_v(v) * v);
yaw_dot = @(tau_r, r) (1/m_r) * (tau_r - d_r(r) * r);

%% Algorithm 

x = 7;
y = 4;
psi = pi/4;

% initial inputs
gamma = 0.8;
gamma0 = gamma;
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
    u = norm(pd_dot(gamma));        
    u_dot = pd_ddot(gamma)' * pd_dot(gamma) * gamma_dot /norm(pd_dot(gamma)); 

    % Angular velocity input
    up = u * cos(psi_e) + k3 * s1;

    gamma_dot = up / norm(pd_dot(gamma));

    y1_dot = u * sin(psi_e) - s1 * k(gamma) * up;

    % Needed to lyapunov
    ep_dot = -S(k(gamma) * up) * ep + [u*cos(psi_e); u*sin(psi_e)] - [up; 0];

    psi_til = psi_e - delta(y1,u);
    
    r = k(gamma)*up + delta_dot(y1,y1_dot,u,u_dot) - k1*psi_til ...
        - k2*y1*u*(sin(psi_e)-sin(delta(y1,u)))/psi_til;

    psi_til_dot = r - k(gamma) * up - delta_dot(y1,y1_dot,u,u_dot);

    %%%%%%%%%%%%%%%%%%%%%% inner loop %%%%%%%%%%%%%%%%%%%%%%
    dt2 = 0.005;
    timesin = (0:dt2:5)';
    veloc = zeros(2, length(timesin));

    u_ref = u; % works
    r_ref = 2;
    ref = [u_ref; 0; r_ref]; % valores desejados

    disp(['I am trying to get u = ', num2str(u_ref)]);
    disp(['I am trying to get r = ', num2str(r_ref)]);

    surge = 0;
    sway = 0;
    yaw = 0;
    current = [surge; sway; yaw];

    tau_u = 0;
    tau_v = 0;
    tau_r = 0;

    % ganhos dos controladores
    kp = 8;
    kd = 0.9;
    kp_2 = 0.2;
    kd_2 = 1;

    for j = 1:length(timesin)

        e = current - ref;
        e_ponto = [surge_dot(tau_u, surge); sway_dot(tau_v, sway); yaw_dot(tau_r, yaw)];
        disp(['e(3) = ', num2str(e(3))]);
        % motores
        tau_u = d_u(surge) + m_u*(-kp*e(1) - kd*e_ponto(1));
        tau_v = 0;
        tau_r = d_r(yaw) + m_r*(-kp_2*e(3) - kd_2*e_ponto(3));

        % atualizar valores 
        surge = surge + dt2 * surge_dot(tau_u, surge);
        sway = sway + dt2 * sway_dot(tau_v, sway);
        yaw = yaw + dt2 * yaw_dot(tau_r, yaw);
        %disp(['yaw = ', num2str(yaw)]);
        current = [surge; sway; yaw];

        veloc(:,j) = [surge; yaw];

    end
    disp(['Final u = ', num2str(surge)]);
    disp(['Final r = ', num2str(yaw)]);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

%% Inner loop controller

u = veloc(1,:); % surge
r = veloc(2,:); % yaw

t = timesin';
plot(t,u)

%% Trajectory, psi, gamma 

x = state(1,:);
y = state(2,:);
psi = state(3,:);
gamma = state(4,:);

t = times';

% Trajectory
plot(x,y)
axis equal
hold on
coelho = pd(gamma0);
%plot(coelho(1), coelho(2), 'Marker', "pentagram", 'MarkerFaceColor', '#A2142F','MarkerSize', 12)
plot([1;0]'*pd(t), [0;1]'*pd(t), 'Color', '#A2142F','LineWidth', 1)
title('Trajectory')
xlabel("x")
ylabel("y")

%%

% Yaw
figure
plot(t, psi)
title('Vehicle heading, \psi')

% Gamma
figure
plot(t, gamma)
title('\gamma')

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