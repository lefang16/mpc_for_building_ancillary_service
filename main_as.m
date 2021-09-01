% Main script to  define and solve solve the Optimal Control Problem using
% a Receding Horizon formulation (MPC)
%

% Define the problem

clear all; %close all;

format compact;

load External_data   % read forecast of external data

% The used data have a temporal resolution of 15 minutes and consequnetly some of the parameters can be defined as a multiple of a quartar
% of an hour only at the moment.

tstep=1/4;          % Time step of simulation (a quarter of an hour). At the moment can not be decreased.
t_end=24*7;           % End of the simulation time in hours
T_ref=0;            % At the moment it can be set a multiple of 0.25 only.
Tf=24;              % prediction horizons in hours

N_seg=4*7;
% Number of discritization segment in an hour
% etc. .__.__.__. N_seg=3


x0=[20,10,20];     % Initial condition of the state.

% This is because the data have temporal resolution of 15 minutes

N_d=N_seg/4+1;  % number of discretization point in a single interval of 15 mins


% Initialise vectors
T=[];               % Initialize vector of times for the closed loop formulation
X=[];               % Initialize vector of states for the closed loop formulation
U=[];               % Initialize vector of inputs for the closed loop formulation
P_r=[];
status_ref=[];      % Initialize vector storing the status of the optimization
simtime=0;          % initialize simulation time (Since the time is not a decision variable and the prediction horizon is constant simstart starts


for simtime=0:tstep:t_end
    
    simtime
    
    options= settings_asOpt(Tf*N_seg+1);          % Get options and solver settings
    [problem,guess]=asOpt_mpc(x0,Tf,simtime);      % Fetch the problem definition
    [solution,MRHistory,~]=solveMyProblem(problem,guess,options);  % Solve optimisation problem
    
    
    if (MRHistory.status>1)||(MRHistory.status<0)
        problem.data.cost_slack=1/5*problem.data.cost_slack;
        [solution,MRHistory,~]=solveMyProblem(problem,guess,options);  % Solve optimisation problem
        if (MRHistory.status>1)||(MRHistory.status<0)
            break  % if the status of the optimisation is not acceptable interrupt the loop
        end
        
    end
    
    
    %    Ldata=OCP.data.data;
    %    Ldata=rmfield(Ldata,'Xscale');
    
    % Update closed loop
    %simulate closed loop system
    [tv,xv,uv] = simulateSolution(problem, solution, 'ode113', 1/60);
    
    U=[U;uv(1:15,:)];                   % Store input solution at the current simulation time; in case you simulate the closed loop use uv
    X=[X;xv(1:15,1:2)];         % Store state solution at the current simulation time; in case you simulate the closed loop use xv
    T=[T;simtime+tv(1:15)];     % Store  current evaluation times; in case you simulate the closed loop use tv
    x0=xv(16,:);
    
    status_ref=[status_ref;MRHistory.status];       % Store  current status of the optimization problem
    
end

% options= settings_asOpt(Tf*N_seg+1);          % Get options and solver settings
% [problem,guess]=asOpt_mpc(x0,Tf,simtime);      % Fetch the problem definition
% for simtime=0:tstep:t_end
%
%     %[problem,guess]=asOpt_mpc(x0,Tf,simtime);      % Fetch the problem definition
%
%     [solution,MRHistory,~]=solveMyProblem(problem,guess,options);  % Solve optimisation problem
%
%
%     if (MRHistory.status>1)||(MRHistory.status<0)
%         break  % if the status of the optimisation is not acceptable interrupt the loop
%     end
%     %    Ldata=OCP.data.data;
%     %    Ldata=rmfield(Ldata,'Xscale');
%
%     % Update closed loop
%     %simulate closed loop system
%     [tv,xv,uv] = simulateSolution(problem, solution, 'ode113', 1/60);
%
%     U=[U;uv(1:15,:)];                   % Store input solution at the current simulation time; in case you simulate the closed loop use uv
%     X=[X;xv(1:15,1:2)];         % Store state solution at the current simulation time; in case you simulate the closed loop use xv
%     T=[T;simtime+tv(1:15)];     % Store  current evaluation times; in case you simulate the closed loop use tv
%     x0=xv(16,:);
%
%     status_ref=[status_ref;MRHistory.status];       % Store  current status of the optimization problem
%     [problem,guess]=asOpt_mpc(x0,Tf,simtime);       % Fetch the problem definition
%     option.start='Warm';
%     guess.time=solution.T;
%     guess.states=[solution.X(2:end-1,:);solution.X(end-1,:); solution.X(end-1,:)];
%     guess.states(1,:)=x0;
%     guess.inputs=[solution.U(2:end-1,:);solution.U(end-1,:); solution.U(end-1,:)];
%
% end

U_base=U(:,1:6);
V1=U(:,7);
V2=U(:,8);
Vd=zeros(length(U),1);
Vc=zeros(length(U),1);
V_B=U(:,9);
V_S=U(:,10);
V=[V1 V2 Vd Vc V_B V_S];
U_real=U_base+w(1:length(U))'.*V;




