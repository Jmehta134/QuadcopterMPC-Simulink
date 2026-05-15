% --- Quadcopter Physical Parameters ---
m = 1.0;      % Mass of the drone (kg)
g = 9.81;     % Gravity (m/s^2)
Ixx = 0.01;   % Inertia around X-axis (Roll)
Iyy = 0.01;   % Inertia around Y-axis (Pitch)
Izz = 0.02;   % Inertia around Z-axis (Yaw)

% --- State Matrix A (12x12) ---
% State Vector: [x, y, z, dx, dy, dz, phi, theta, psi, p, q, r]
A = zeros(12, 12);

% Velocities are the derivatives of positions
A(1, 4) = 1; A(2, 5) = 1; A(3, 6) = 1; 

% Angles cause linear accelerations (Gravity coupling)
A(4, 8) = g; % Pitching forward causes negative X acceleration
A(5, 7) = -g;  % Rolling right causes positive Y acceleration

% Angular velocities are the derivatives of angles
A(7, 10) = 1; A(8, 11) = 1; A(9, 12) = 1;

% --- Input Matrix B (12x4) ---
% Input Vector: [U1, U2, U3, U4]
B = zeros(12, 4);

% U1 (Total Thrust) affects Z acceleration
B(6, 1) = 1 / m; 

% U2, U3, U4 (Torques) affect angular accelerations
B(10, 2) = 1 / Ixx; % Roll torque
B(11, 3) = 1 / Iyy; % Pitch torque
B(12, 4) = 1 / Izz; % Yaw torque

% 1. Define the Plant (Using your exact same A and B matrices from hover)
plant = ss(A, B, eye(12), zeros(12,4)); 

% 2. Setup the MPC Parameters
Ts = 0.05;              % How fast the controller runs (Sample Time)
PredictionHorizon = 30; % How many steps into the future it looks
ControlHorizon = 5;     % How many motor commands it plans ahead

% 3. Create the actual MPC Object
mpc_obj = mpc(plant, Ts, PredictionHorizon, ControlHorizon);

% 4. Set Motor Constraints (This is why MPC is better than LQR!)
% Let's strictly limit how much the thrust (U1) can change to simulate RPM limits
mpc_obj.MV(1).Min = -10;  
mpc_obj.MV(1).Max = 10;   

% 5. Tuning (Similar to your Q and R matrices)
% Penalize state errors heavily on the angles (states 7, 8, 9)
mpc_obj.Weights.OutputVariables = [10 10 10 5 5 5 50 50 50 1 1 1]; 
% Penalize using too much control effort
mpc_obj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];