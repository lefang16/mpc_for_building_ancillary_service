function dx = asOpt_Dynamics_Sim(x,u,p,t,vdat)
%Supersonic Aircraft Minimum Fuel Climb Problem - Dynamics - Internal
%
% Syntax:
%          [dx] = Dynamics(x,u,p,t,vdat)	(Dynamics Only)
%          [dx,g_eq] = Dynamics(x,u,p,t,vdat)   (Dynamics and Eqaulity Path Constraints)
%          [dx,g_neq] = Dynamics(x,u,p,t,vdat)   (Dynamics and Inqaulity Path Constraints)
%          [dx,g_eq,g_neq] = Dynamics(x,u,p,t,vdat)   (Dynamics, Equality and Ineqaulity Path Constraints)
%
% Inputs:
%    x  - state vector
%    u  - input
%    p  - parameter
%    t  - time
%    vdat - structured variable containing the values of additional data used inside
%          the function%
% Output:
%    dx - time derivative of x
%    g_eq - constraint function for equality constraints
%    g_neq - constraint function for inequality constraints
%
% Copyright (C) 2019 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
% This code is published under the MIT License.
% Department of Aeronautics and Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK
% ICLOCS (Imperial College London Optimal Control) Version 2.5
% 1 Aug 2019
% iclocs@imperial.ac.uk
%
%------------- BEGIN CODE --------------

% States

x1 = x(1); 
x2 = x(2);


% Inputs
u1 = u(1);
u2 = u(2); 
ud = u(3);
uc = u(4);

v1=u(6);
v2=u(7);

% x1 - dwelling Temperature
% X2 - state of charge of LiFePO4 battery

% in extreme case w=-1 for all t 
% x3 - dwelling Temperature  
% X4 - state of charge of LiFePO4 battery 

% u1 - Heating  HP power consumption    v1
% u2 - Cooling HP power consumpyion     v2
% ud - Battery discharge rate           v3
% uc - Battery charge rate              v4
% u_B_grid - Power bought from the grid v5
% u_s_grid - Power inject into the grid v6

% v=[v1 v2 vd vc v_B v_S] are the variation signal for provision of AS.
% where Delta u= w(t) * v(t)

UV= vdat.UV;
A= vdat.A;
rhoAir= vdat.rhoAir;
B_V= vdat.B_V;
C_air= vdat.C_air;
n_ac=vdat.n_ac;
C_Build = vdat.C_Build;
T_ex=  vdat.T_ex ;
T_mex=vdat.T_mex;
T_w=vdat.T_w;
m_COP=vdat.m_COP;
COP_init=vdat.COP_init;
T_init=vdat.T_init;
Ir=vdat.Ir;
m_cop_cool=vdat.m_cop_cool;
leak_battery=vdat.leak_battery;
PV_S=vdat.PV_s_Ap*vdat.AFl;
r=vdat.r;
w=vdat.w;


% Solar Radiance
%IRd=interp1(T_mex,Ir,t,'linear');
IRd=interp1(T_mex,Ir,t,'previous');
% Ambient Temperature
Ta=interp1(T_mex,T_ex,t,'linear');
% tracking signal
w=interp1(T_w,w,t,'linear');

COPTA=m_COP*(Ta-T_init)+COP_init; 

% x1 room temperature
dx(1)=(UV*A+rhoAir* B_V*C_air*n_ac)*(Ta-x1)./C_Build+COPTA.*(u1+w.*v1)./C_Build-m_cop_cool*(u2+w.*v2)./C_Build;
% x2 battery storage
dx(2)=-leak_battery*x2+vdat.ch_eff*(uc)-(ud)./vdat.dis_eff;
dx(3)=dx(1);









