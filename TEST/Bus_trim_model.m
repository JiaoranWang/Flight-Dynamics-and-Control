%% Search for a specified operating point for the model - FlightGear_BUS_trim.
%
% This MATLAB script is the command line equivalent of the trim model
% tab in linear analysis tool with current specifications and options.
% It produces the exact same operating points as hitting the Trim button.

% MATLAB(R) file generated by MATLAB(R) 9.2 and Simulink Control Design (TM) 4.5.
%
% Generated on: 27-Mar-2020 18:44:20

%% Specify the model name
model = 'FlightGear_BUS_trim';

%% Create the operating point specification object.
opspec = operspec(model);

%% Set the constraints on the states in the model.
% - The defaults for all states are Known = false, SteadyState = true,
%   Min = -Inf, and Max = Inf.

% State (1) - FlightGear_BUS_trim/Rot dyn1/Integrator
% - Default model initial conditions are used to initialize optimization.

% State (2) - FlightGear_BUS_trim/Rot kine/Integrator
opspec.States(2).x = [0;0;1.5708];

% State (3) - FlightGear_BUS_trim/Rot kine/Integrator1
opspec.States(3).x = [0;0;1.5708];
opspec.States(3).SteadyState = [false;false;false];

% State (4) - FlightGear_BUS_trim/Rot kine/PID Controller/Filter
% - Default model initial conditions are used to initialize optimization.
opspec.States(4).SteadyState = false;

% State (5) - FlightGear_BUS_trim/Rot kine/PID Controller/Integrator
% - Default model initial conditions are used to initialize optimization.
opspec.States(5).SteadyState = false;

% State (6) - FlightGear_BUS_trim/Rot kine/PID Controller1/Filter
% - Default model initial conditions are used to initialize optimization.
opspec.States(6).SteadyState = [false;false;false];

% State (7) - FlightGear_BUS_trim/Rot kine/PID Controller1/Integrator
% - Default model initial conditions are used to initialize optimization.
opspec.States(7).SteadyState = [false;false;false];

% State (8) - FlightGear_BUS_trim/Trans Dyn1/Integrator
% - Default model initial conditions are used to initialize optimization.

% State (9) - FlightGear_BUS_trim/new tran kine sph/Integrator
opspec.States(9).x = [-2709477.1659;-4270606.022;3872800.8582];


%% Create the options
opt = findopOptions('DisplayReport','iter');

%% Perform the operating point search.
[op,opreport] = findop(model,opspec,opt);