% Read data relative to the European Case study


clear all

% File containing data
filename='data_problem.xlsx';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define problem data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Three dimensional system: 1 thermal equation,1 battery storage 
load activation % w

% State and input bounds for the battery
Cap_b=60;  %120;  %kwh
p_rate=10;  % Kw 
leak_battery=0; %1/2000;

xlow=[-10, -1, -10];
xup=[40,Cap_b,40];


% 6 inputs 
% 1) Electricty (Power) used by edHP, 2) cooling HP, 3) Battery discharge rate,
% 4) Battery charge rate, 5) Power bought from the grid 6) Power inject into the grid


ulow=[0, 0, 0, 0, 0, 0,   0, 0, 0, -30,    0,0,0];

% [ueH uCeH udch uch uB uS]
      
% Inputs upper bounds in kW
% [ueH uCeH udch uch uB uS]
 
% uup=[inf, inf, p_rate, p_rate, inf, inf, inf, inf, p_rate, p_rate, inf, inf];
%           
uup=[7, 7, p_rate, p_rate, 30, 30,    7, 7, 30, 0,   100,100,100];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Retrieve parameters ofa 3 room building for a lumped model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

UV=xlsread(filename,1,'B4');     % overall heat transfer coefficient, U (Average U-value) ( W/m^2 K) 
UV=UV/1000;  % convertin W in kW
A=xlsread(filename,1,'B3');     % Wall surface Area (m^2)
rhoAir=xlsread(filename,1,'B8');     % Air density  (kg/ m^3)
B_V=xlsread(filename,1,'B6');     % Building Volume (m^3)
C_air=xlsread(filename,1,'B9');     % Air heat capacity (kJ/ kg/ K)
C_air=C_air/3600;  % Converting kJ in kWh
n_ac=xlsread(filename,1,'B7');     % Air changes per hour (hr^(-1))
C_Build=xlsread(filename,1,'B10');     % Building Capacity (kj/ K)(Note kJ/C is equivalent to kj/K)
C_Build=C_Build/3600;    % Converting kJ in kWh
AFl=xlsread(filename,1,'B2');     % Floor surface Area (m^2)


%Carbon_e=xlsread(filename,5,'B2:B722')';   % winter 
 Carbon_e=xlsread(filename,5,'B9266:B9986')'; % summer

%Ir=xlsread(filename,7,'C333125:D333485')'; % Hourly data winter    
 Ir=xlsread(filename,7,'C337757:C338117')';   % Summer

%Pr_elec=xlsread(filename,4,'B2:B1442')';      % (£/kWh)   From 1/1/2018   
 Pr_elec=xlsread(filename,4,'B18530:B19970')';   % (£/kWh)   From 13/7/2018   

% External temperature
%T_ex_r=xlsread(filename,7,'B333125:B333485')';   % (^circ C)  From 1/1/2018 
 T_ex_r=xlsread(filename,7,'B337757:B338117')';     % 13/7/2018




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Carbon Emission half hourly data 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T_carb=[0:1/2:360];  % Vector of times
Price_carbon=100;   % in £/(ton CO2e)
Price_carbon= Price_carbon/1e+6; % converted in £/gr    


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hourly data of Global horizontal irradiance (W/m^2) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Converting W to kW
Ir=Ir/1000;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Exogenous Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Desired vector of times associated to the external data
T_mex=[0:1/4:360];

% Interpolation values of Carbon emission
Carbon_em=interp1(T_carb,Carbon_e,T_mex,'linear');  % (£/kWh)
% Interpolation of radiance data
Ir=interp1([0:360],Ir,T_mex,'linear');
% Interpolation of ambient Temperature
T_ex=interp1([0:360],T_ex_r,T_mex,'linear');

% Notes
%  To Convert kJ to Wh divide by 3.6
%
%  Kelvin to celsius conversion   [C] = [K]-273.15 
%
%  The COP  for an electric driven heat pump at 7 \circ celsius is assumed equal to  3  
%
%  I assume that at 22 \circ celsius COP is 4 

COP_init=3;
T_init=7;
COP_Ta=4;
Ta=22;

% coefficient of the COP for the heat pump accounting of a linear dependence with respect to 
% the external temperature
m_COP=(COP_Ta-COP_init)/(Ta-T_init);
m_cop_cool=0.7;    % COP of the cooling system


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Batteries 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dis_eff=1;     %discharge efficiency
ch_eff=0.88;   %charge efficiency



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PVs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The prediction output is in KWp/m^2 

theta1=xlsread(filename,3,'E4');
theta2=xlsread(filename,3,'E5');
theta3=xlsread(filename,3,'E6');
% Proportion of total area covered by PVs
PV_s_Ap=1;

% Proportion of total area covered by Thermal Solar
% TSo_Ap=1/2; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ancillary service
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

r=0.1;
cost_slack=5;

T_w=[0:1/60:360];

% w=-1+rand(size(T_w));


save External_data Ir T_ex T_mex w
save case_study_sim




