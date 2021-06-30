%% Define Aircraft Mass and Geometry Properties (using grams)
% mass xSize ySize zSize xLoc yLoc zLoc
% 1 2 3 4 5 6 7
Rearth=6371000; g_EN=[0; 0; 9.8];
x_E_E_BE0=[Rearth,0,0];
startTime = 0;
stopTime = inf;
stepTime = 0.01;
RatioToRealTime = .8;
deg2rad = pi/180; m = 2.0;
Vbbe_int = [10; 0; 0];
w_B_BE_ini = [1e0; 1e-2; 1e-2];
Fext_N = [10 10 10];
Mext_B = [1; 1; 1];
v_N_AE=[0;0;0];
InitQuat = [1;0;0;0];
p_E_BE_ini_deg= [37.6286; -122.393; 10];
p_E_BE_ini = p_E_BE_ini_deg .* [deg2rad; deg2rad; 1];
Theta_ini_deg = [0; 0; 90];
Theta_ini =Theta_ini_deg * deg2rad;
rudder=0;
Aileron_left=0;
Aileron_right=0;
elevator=0;

MnG = ...
[90 0.1 0.96 0.01 -0.23 0.44 0; % RightWing+Servo
90 0.1 0.96 0.01 -0.23 -0.44 0; % LeftWing+Servo
13 0.075 0.35 0.002 -0.76 0 0.16; % Elevator
72 0.065 0.035 0.015 -0.05 0 0.03; % Battery
106 0.87 0.07 0.07 -0.4 0 0; % Fuselage
27 0.05 0.03 0.005 -0.05 0 0.02; % Motor Controller
10 0.04 0.02 0.005 0.1 0 0.02; % Radio
20 0.05 0.01 0.01 -0.014 0 0; % 2 Servos
40 0.03 0.02 0.02 0.02 0 0.01; % Motor
12 0 0.26 0.025 0.05 0 0.01];% Propeller

M_total=sum(MnG(:,1))/1000;
CG_x=sum((MnG(:,1)/1000).*MnG(:,5))/0.48;
CG_y=sum((MnG(:,1)/1000).*MnG(:,6))/M_total;
CG_z=sum((MnG(:,1)/1000).*MnG(:,7))/M_total;
CG=[CG_x;CG_y;CG_z];
J0=ones(3);
J0(1,1)=sum((MnG(:,3).^2+MnG(:,4).^2).*MnG(:,1)/(12*1000)+MnG(:,1)/1000.*(MnG(:,6).^2+MnG(:,7).^2));
J0(2,2)=sum((MnG(:,2).^2+MnG(:,4).^2).*MnG(:,1)/(12*1000)+MnG(:,1)/1000.*(MnG(:,5).^2+MnG(:,7).^2));
J0(3,3)=sum((MnG(:,3).^2+MnG(:,2).^2).*MnG(:,1)/(12*1000)+MnG(:,1)/1000.*(MnG(:,6).^2+MnG(:,5).^2));
J0(1,2)=sum(MnG(:,1)/1000.*MnG(:,5).*MnG(:,6));
J0(2,1)=J0(1,2);
J0(1,3)=sum(MnG(:,1)/1000.*MnG(:,5).*MnG(:,7));
J0(3,1)=J0(1,3);
J0(2,3)=sum(MnG(:,1)/1000.*MnG(:,7).*MnG(:,6));
J0(3,2)=J0(2,3);

J_cg=ones(3);
J_cg(1,1)=sum((MnG(:,3).^2+MnG(:,4).^2).*MnG(:,1)/(12*1000)+MnG(:,1)/1000.*((MnG(:,6)-CG_y).^2+(MnG(:,7)-CG_z).^2));
J_cg(2,2)=sum((MnG(:,2).^2+MnG(:,4).^2).*MnG(:,1)/(12*1000)+MnG(:,1)/1000.*((MnG(:,5)-CG_x).^2+(MnG(:,7)-CG_z).^2));
J_cg(3,3)=sum((MnG(:,2).^2+MnG(:,3).^2).*MnG(:,1)/(12*1000)+MnG(:,1)/1000.*((MnG(:,5)-CG_x).^2+(MnG(:,6)-CG_y).^2));
J_cg(1,2)=sum(MnG(:,1)/1000.*(MnG(:,5)-CG_x).*(MnG(:,6)-CG_y));
J_cg(2,1)=J_cg(1,2);
J_cg(1,3)=sum(MnG(:,1)/1000.*(MnG(:,5)-CG_x).*(MnG(:,7)-CG_z));
J_cg(3,1)=J_cg(1,3);
J_cg(2,3)=sum(MnG(:,1)/1000.*(MnG(:,6)-CG_y).*(MnG(:,7)-CG_z));
J_cg(3,2)=J_cg(2,3);

%% Use the following for the properties of each surface:
n_s_2345=[ [0;0;-1];[0;1; 0];[0; 0; -1]; [0; 0; -1];];
CL0_2345=[0;0; 0.05; 0.05];
e_s2345 = [0.8;0.8;0.9;0.9];
i_s2345 = [0; 0; 0.05; 0.05;];
CD0_s2345 =[0.01;0.01;0.01;0.01];
CDa_s2345 = [1;1;1;1];
a0_s2345 = [0;0;0.05;0.05];
CM0_s2345 = [0;0;-0.05;-0.05];
CMa_s2345 = [0; 0; 0; 0];

%%S2 = Elevator; S3=rudder; S5=left wing; S4=right wing
%% parameters
%positions
xyz_s=[[-0.76 0 0.16];[-0.76 0 -0.09];[-0.23 0.44 0];[-0.23 -0.44 0];];
%chord_c each x_size
c_2345=[0.075;0.08;0.1;0.1];
%span_b each y_size
b_2345=[0.35;0.08;0.96;0.96];
%area_b*c
A_2345=c_2345.*b_2345;
%aspect ratio b/c
r=b_2345./c_2345;
% aerodynamic center =1/4 chord, the position of which:  xp+1/4*x_size
ac_2345=[xyz_s(1,1);xyz_s(2,1);xyz_s(3,1);xyz_s(4,1)]+1/4*c_2345;
CLa=2*pi*(r./(2+r));
%% Properties
CLa=2*pi*(r./(2+r)); V_B_BA=[ 1 1 1];V_inf=norm(V_B_BA); 
n2=[0,0,-1]; n3=[0,1,0]; n4=[0,0,-1]; n5=[0,0,-1]; n=[n2',n3',n4', n5'];

for i=1:4
a_ss=zeros(4,1);
a_ss(i)=i_s2345(i)-asin(complex(V_B_BA*n(:,i))/V_inf);
end
CL=CL0_2345+CLa.*a_ss; a=a_ss-a0_s2345;
cdl1=CL.^2; cdl2=(pi*(e_s2345.*r)); cdl=cdl1./cdl2;
CD=CD0_s2345+CDa_s2345.*(a.*a)+cdl;
CM=CM0_s2345+CMa_s2345.*a_ss;
L= CL.*A_2345*(0.5*(V_inf^2));
D=CD.*A_2345*(0.5*V_inf.^2);
CD_f=0.5;
D_fulsage=CD_f.*0.001*(0.5*V_inf.^2);
M=CM.*A_2345.*c_2345*(0.5*V_inf.^2);
for i=1:4
F_surf=zeros(3,4);
F_surf=L(i)*n(:,i)-[D(i);0;0];
end
F_fulsage=-[D_fulsage;0;0];
F_inner_total=F_surf+F_fulsage;
alp=10;
bet=3;
wind_angle=[alp, bet];
Csb=[cos(alp), 0, sin(alp); 0, 1,0; -1*sin(alp) 0 cos(alp)];
Cws=[cos(bet),sin(bet),0 ; -1*sin(bet),cos(bet),0;0, 0 ,1];
Cwb=Cws*Csb;
Cbw=inv(Cwb);
F_b=Cbw*F_inner_total;
F=Cbw*F_surf;
for i=1:4
c=cross(xyz_s(i,:),F_b(:,1));
C=[0,0,0;0,0,-1;0, 1,0];
M_s2345=zeros(3,4);
M_s2345=M(i)*C*n(:,i)+cross(xyz_s(i,:),F_b(:,1));
for j=1:3
   Minner_total=zeros(3,1);
   Minner_total=M_s2345(1,:);
end
end

% parameters
Rearth=6371000; g_EN=[0; 0; 9.81];
X_E_E_BE0=[Rearth,0,0];
startTime = 0;
stopTime = inf;
stepTime = 0.01;
RatioToRealTime = .8;
deg2rad = pi/180; m = 2.0;
x_E_E_BE0 = [Rearth;0;0];
InitQuat = [0;0;0;0];
V_int = [0; 0; 0];
ini_rates = [1e-2; 1e-0; 1e-2];
Fext_N = -g_EN * m;
Mext_B = [0; 0; 0];
x_E_E_BE0 = [Rearth;0;0];
InitQuat = [1;0;0;0];




















