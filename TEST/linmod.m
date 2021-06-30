model1 = 'Perfect_Lineaer';

%% Specify the analysis I/Os
% Get the analysis I/Os from the model
io = getlinio(model1);

%% Specify the operating point
% Create the operating point variable op_trim1 using model initial condition as a starting point
op = operpoint('Perfect_Lineaer');


%% Linearize the model
sys1 = linearize(model1,io,op);

model2 = 'FlightGear_BUS_aerosurface_trim';

%% Specify the analysis I/Os
% Get the analysis I/Os from the model
io = getlinio(model2);

%% Specify the operating point
% Create the operating point variable op_trim1 using model initial condition as a starting point
op = operpoint('FlightGear_BUS_aerosurface_trim');


%% Linearize the model
sys2 = linearize(model2,io,op)
bode(sys1, sys2, 'k'); 
rlocus(sys1)
step(sys1)
gram(sys1,'c')
