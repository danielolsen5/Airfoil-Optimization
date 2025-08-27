% Optimization for minimizing Drag-to-Lift Ratio with Aircraft Parameters

% Define air density as a function of altitude (ISA Model)
function rho = air_density(altitude)
    % Constants
    T0 = 288.15; % Standard temperature in Kelvin at sea level
    P0 = 101325; % Standard pressure in Pascals at sea level
    R = 287.05;  % Specific gas constant for air in J/(kg*K)
    L = 0.00649; % Temperature lapse rate in K/m
    T = T0 - L * altitude; % Temperature in Kelvin at altitude
    P = P0 * (T / T0)^(9.81 / (R * L)); % Pressure in Pascals at altitude
    rho = P / (R * T); % Air density in kg/m^3
end

% Define the lift coefficient (Cl) model
function Cl = lift_coefficient(thickness, camber, AoA, M)
    % Simplified lift coefficient (Cl) estimation for an airfoil with Prandtl-Glauert correction
    Cl0 = 2 * pi * AoA * (1 - (thickness / 100)^2);
    beta = sqrt(1 - M^2);
    Cl = Cl0 / beta;
end

% Define the drag coefficient (Cd) model with induced drag and wave drag
function Cd = drag_coefficient(thickness, camber, AoA, AR, e, M)
    % Simplified drag coefficient (Cd) estimation for an airfoil
    Cd_form = 0.01 + (thickness / 100)^2 + (camber / 10)^2; % Base form drag coefficient
    Cl = lift_coefficient(thickness, camber, AoA, M); % Lift coefficient
    Cd_induced = (Cl^2) / (pi * AR * e); % Induced drag based on aspect ratio and span efficiency
    if M > 0.8 % Add wave drag for transonic speeds
        Cd_wave = 0.0025 * (M - 0.8)^2;
    else
        Cd_wave = 0;
    end
    Cd = Cd_form + Cd_induced + Cd_wave; % Total drag coefficient
end

% Define the drag-to-lift ratio objective function
function D_L_ratio = drag_to_lift_ratio(params, altitude, speed, weight, S, AR, e)
    % Optimization parameters
    thickness = params(1);
    camber = params(2);
    AoA = params(3); % Angle of attack
    
    % Compute air density based on altitude
    rho = air_density(altitude);
    
    % Compute Mach number
    a = 340.29; % Speed of sound at sea level in m/s (ISA)
    M = speed / a;
    
    % Compute lift and drag coefficients using the geometry parameters
    Cl = lift_coefficient(thickness, camber, AoA, M);
    Cd = drag_coefficient(thickness, camber, AoA, AR, e, M);
    
    % Compute the required lift coefficient to balance weight
    required_Cl = (2 * weight) / (rho * speed^2 * S);
    
    % Compute the drag-to-lift ratio
    if Cl == 0
        D_L_ratio = Inf; % Avoid division by zero
    else
        D_L_ratio = Cd / Cl;
    end
    
    % Penalize deviations from the required lift coefficient
    D_L_ratio = D_L_ratio + 10 * abs(Cl - required_Cl);
end

% Optimization procedure
function [optimized_thickness, optimized_camber, optimized_AoA, min_D_L_ratio] = optimize_airfoil(altitude, speed, weight, S, AR, e)
    % Initial guess for the parameters [thickness, camber, AoA]
    initial_guess = [12.0, 2.0, 5.0]; % 10% thickness, 2% camber, 5 degrees AoA
    
    % Set the bounds 
    lb = [5.0, 0.0, -10.0]; % Lower bounds: [min thickness, min camber, min AoA]
    ub = [20.0, 8.0, 20.0]; % Upper bounds: [max thickness, max camber, max AoA]
    
    % Define optimization options
    options = optimset('fminunc');
    options.TolX = 1e-6;
    options.TolFun = 1e-6;
    options.Display = 'iter';
    
    % Minimize the drag-to-lift ratio
    [optimal_params, min_D_L_ratio] = fminunc(@(params) drag_to_lift_ratio(params, altitude, speed, weight, S, AR, e), ...
                                               initial_guess, options);
    
    % Optimized parameters
    optimized_thickness = optimal_params(1);
    optimized_camber = optimal_params(2);
    optimized_AoA = optimal_params(3);
end

% Given variables
altitude = 10000;  % Altitude in meters
speed = 290;      % True Airspeed (TAS) in m/s
weight = 2491004.1;   % Aircraft weight in Newtons (N)
S = 325;           % Wing area in m^2
AR = 9.5;           % Wing aspect ratio (for induced drag calculation)
e = 0.85;         % Span efficiency factor

% Do the optimization
[optimized_thickness, optimized_camber, optimized_AoA, min_D_L_ratio] = optimize_airfoil(altitude, speed, weight, S, AR, e);

% Display the results
fprintf('Optimized Thickness: %.2f%%\n', optimized_thickness);
fprintf('Optimized Camber: %.2f%%\n', optimized_camber);
fprintf('Optimized Angle of Attack: %.2f degrees\n', optimized_AoA);
fprintf('Minimum Drag-to-Lift Ratio: %.4f\n', min_D_L_ratio);
