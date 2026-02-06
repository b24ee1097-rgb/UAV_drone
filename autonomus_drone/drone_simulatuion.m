%% Quadrotor Simulation (Complete Working Code)

% Simulation time (seconds)
start_time = 0;
end_time   = 10;
dt         = 0.005;
times      = start_time:dt:end_time;
N          = numel(times);

% Physical parameters
m  = 1.0;                 % mass (kg)
g  = 9.81;                % gravity (m/s^2)
L  = 0.25;                % arm length (m)
k  = 3e-6;                % thrust coefficient
b  = 1e-7;                % drag coefficient
kd = 0.1;                 % linear drag
I  = diag([0.01 0.01 0.02]); % inertia matrix

% Initial state
x     = [0; 0; 10];       % position
xdot  = zeros(3,1);       % velocity
theta = zeros(3,1);       % roll, pitch, yaw

% Initial angular velocity disturbance (rad/s)
deviation = 2;
thetadot = 2 * deviation * rand(3,1) - deviation;

% Logging
x_hist     = zeros(3,N);
theta_hist = zeros(3,N);

%% Simulation loop
idx = 1;
for t = times
    % Controller input (motor speeds squared)
    inputs = input(t);

    % Convert Euler rates to body angular velocity
    omega = thetadot2omega(thetadot, theta);

    % Linear and angular accelerations
    a        = acceleration(inputs, theta, xdot, m, g, k, kd);
    omegadot = angular_acceleration(inputs, omega, I, L, b, k);

    % Integrate
    omega     = omega + dt * omegadot;
    thetadot = omega2thetadot(omega, theta);
    theta    = theta + dt * thetadot;

    xdot = xdot + dt * a;
    x    = x + dt * xdot;

    % Log
    x_hist(:,idx)     = x;
    theta_hist(:,idx) = theta;
    idx = idx + 1;
end

%% Plot results
figure;
subplot(2,1,1)
plot(times, x_hist)
xlabel('Time (s)')
ylabel('Position (m)')
legend('x','y','z')
grid on

subplot(2,1,2)
plot(times, rad2deg(theta_hist))
xlabel('Time (s)')
ylabel('Angle (deg)')
legend('roll','pitch','yaw')
grid on

%% ================= FUNCTIONS =================

function u = input(~)
% Constant hover input (omega^2)
u = 400 * ones(4,1);
end

function T = thrust(inputs, k)
T = [0; 0; k * sum(inputs)];
end

function tau = torques(inputs, L, b, k)
tau = [
    L * k * (inputs(1) - inputs(3));
    L * k * (inputs(2) - inputs(4));
    b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))
];
end

function a = acceleration(inputs, angles, xdot, m, g, k, kd)
gravity = [0; 0; -g];
R = rotation(angles);
T = R * thrust(inputs, k);
Fd = -kd * xdot;
a = gravity + (1/m) * T + Fd;
end

function omegadot = angular_acceleration(inputs, omega, I, L, b, k)
tau = torques(inputs, L, b, k);
omegadot = I \ (tau - cross(omega, I * omega));
end

function R = rotation(angles)
phi   = angles(1);
theta = angles(2);
psi   = angles(3);

R = [
    cos(psi)*cos(theta), ...
    cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi), ...
    cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
    sin(psi)*cos(theta), ...
    sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), ...
    sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
    -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)
];
end

function omega = thetadot2omega(thetadot, angles)
phi = angles(1);
theta = angles(2);

W = [
    1 0 -sin(theta);
    0 cos(phi) cos(theta)*sin(phi);
    0 -sin(phi) cos(theta)*cos(phi)
];
omega = W * thetadot;
end

function thetadot = omega2thetadot(omega, angles)
phi = angles(1);
theta = angles(2);

W = [
    1 0 -sin(theta);
    0 cos(phi) cos(theta)*sin(phi);
    0 -sin(phi) cos(theta)*cos(phi)
];
thetadot = W \ omega;
end
%% 3D Quadrotor Visualization

figure;
axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
view(3)
hold on
xlim([-5 5])
ylim([-5 5])
zlim([0 15])

% Quad geometry (body frame)
arm1 = [ L  0  0; -L  0  0]';
arm2 = [ 0  L  0;  0 -L  0]';

% Plot handles
h_arm1 = plot3(0,0,0,'r','LineWidth',2);
h_arm2 = plot3(0,0,0,'b','LineWidth',2);
h_motors = plot3(0,0,0,'ko','MarkerFaceColor','k');

for k = 1:10:N   % skip frames for speed
    pos = x_hist(:,k);
    ang = theta_hist(:,k);

    % Rotation matrix
    R = rotation(ang);

    % Rotate arms
    arm1_w = R * arm1 + pos;
    arm2_w = R * arm2 + pos;

    % Motor positions
    motors = [ arm1(:,1) arm1(:,2) arm2(:,1) arm2(:,2) ];
    motors_w = R * motors + pos;

    % Update plots
    set(h_arm1,'XData',arm1_w(1,:), ...
               'YData',arm1_w(2,:), ...
               'ZData',arm1_w(3,:));
    set(h_arm2,'XData',arm2_w(1,:), ...
               'YData',arm2_w(2,:), ...
               'ZData',arm2_w(3,:));
    set(h_motors,'XData',motors_w(1,:), ...
                 'YData',motors_w(2,:), ...
                 'ZData',motors_w(3,:));

    drawnow
end
%% Full 3D Mesh Quadcopter Animation

figure('Color','w');
axis equal
grid on
view(3)
xlabel('X')
ylabel('Y')
zlabel('Z')
xlim([-5 5])
ylim([-5 5])
zlim([0 15])
hold on

% ===== BODY GEOMETRY (BODY FRAME) =====
body_size = [0.3 0.2 0.1]; % x y z
[bx, by, bz] = cuboid(body_size);

% ===== ARM GEOMETRY =====
arm_len = L;
arm_w   = 0.04;
arm_h   = 0.04;

[ax1, ay1, az1] = cuboid([2*arm_len arm_w arm_h]);
[ax2, ay2, az2] = cuboid([arm_w 2*arm_len arm_h]);

% ===== ROTOR GEOMETRY =====
r = 0.12;
theta = linspace(0,2*pi,40);
rotor = [r*cos(theta); r*sin(theta); zeros(size(theta))];

% Plot handles
h_body = surf(bx,by,bz,'FaceColor',[0.2 0.2 0.2],'EdgeColor','none');
h_arm1 = surf(ax1,ay1,az1,'FaceColor',[0.8 0.1 0.1],'EdgeColor','none');
h_arm2 = surf(ax2,ay2,az2,'FaceColor',[0.1 0.1 0.8],'EdgeColor','none');

h_rotor(1) = fill3(0,0,0,'k');
h_rotor(2) = fill3(0,0,0,'k');
h_rotor(3) = fill3(0,0,0,'k');
h_rotor(4) = fill3(0,0,0,'k');

lighting gouraud
camlight headlight

% ===== ANIMATION LOOP =====
for k = 1:10:size(x_hist,2)

    pos = x_hist(:,k);
    ang = theta_hist(:,k);
    R   = rotation(ang);

    % Transform body
    [X,Y,Z] = transform_mesh(bx,by,bz,R,pos);
    set(h_body,'XData',X,'YData',Y,'ZData',Z);

    % Transform arms
    [X,Y,Z] = transform_mesh(ax1,ay1,az1,R,pos);
    set(h_arm1,'XData',X,'YData',Y,'ZData',Z);

    [X,Y,Z] = transform_mesh(ax2,ay2,az2,R,pos);
    set(h_arm2,'XData',X,'YData',Y,'ZData',Z);

    % Rotor centers (body frame)
    motors = [
        L  0  0;
       -L  0  0;
        0  L  0;
        0 -L  0
    ]';

    motors_w = R * motors + pos;

    for i = 1:4
        rot = R * rotor + motors_w(:,i);
        set(h_rotor(i),'XData',rot(1,:), ...
                       'YData',rot(2,:), ...
                       'ZData',rot(3,:));
    end

    drawnow
end
function [X,Y,Z] = cuboid(sz)
lx = sz(1)/2; ly = sz(2)/2; lz = sz(3)/2;
[X,Y,Z] = meshgrid([-lx lx],[-ly ly],[-lz lz]);
end

function [Xw,Yw,Zw] = transform_mesh(X,Y,Z,R,p)
pts = R * [X(:)'; Y(:)'; Z(:)'] + p;
Xw = reshape(pts(1,:), size(X));
Yw = reshape(pts(2,:), size(Y));
Zw = reshape(pts(3,:), size(Z));
end


