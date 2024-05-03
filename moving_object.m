% Constants and Initial Conditions
G = 6.67430e-11;  % Gravitational constant in m^3 kg^-1 s^-2
M = 5.972e24;     % Mass of Earth in kg
dt = 10;          % Time step in seconds

% Initial position and velocity for elliptical orbit
x = 6.371e6 + 400000;  % Distance from Earth's center + altitude of 400 km at perigee
y = 0;
eccentricity = 0.1;  % Eccentricity of the ellipse

% Velocity at perigee for elliptical orbit
perigee_velocity = sqrt((G * M * (1 + eccentricity)) / (x * (1 - eccentricity)));
vx = 0;
vy = perigee_velocity;  % Adjusted for elliptical orbit

% Simulation time
T = 5400;  % Total time in seconds to see full elliptical motion
steps = T / dt;

% Arrays for storing positions
X = zeros(1, steps);
Y = zeros(1, steps);

% Euler's method to update position and velocity
for i = 1:steps
    % Store positions for plotting
    X(i) = x;
    Y(i) = y;
    
    % Calculate radial distance
    r = sqrt(x^2 + y^2);
    
    % Calculate acceleration components
    ax = -G * M * x / r^3;
    ay = -G * M * y / r^3;
    
    % Update velocities
    vx = vx + ax * dt;
    vy = vy + ay * dt;
    
    % Update positions
    x = x + vx * dt;
    y = y + vy * dt;
end

% Plotting the orbit
figure;
plot(X, Y);
axis equal;
title('Elliptical Orbital Path of a Spacecraft');
xlabel('X Position (m)');
ylabel('Y Position (m)');
grid on;
