clear;  %clears wworkspace
clc;    %clears command window


  
%% FOR REFERENCE:
% NUMBER OF STEPS
model.N = 20;
%Where do we define the sampling time?

% z(1) = u(1) = a
% z(2) = u(2) = delta

% z(3) = x(1) = X
% z(4) = x(2) = Y
% z(5) = x(3) = v
% z(6) = x(4) = phi
% z(7) = x(5) = a_old
% z(8) = x(6) = delta_old
model.nvar = 8;

% KEEPING TRACK OF PARAMETERS:
% p(1) - Xtarget for the current time step 
% p(2) - Ytarget for the current time step
% p(3) - vtarget for the current time step

% p(4) - weight of Xtarget
% p(5) - weight of Ytarget
% p(6) - weight of Vtarget
% p(7) - weight of DeltaAcceleration
% p(8) - weight of DeltaSteer
% p(9) - weight of Acceleration
% p(10) - weight of GreatestLatError

%p(11) - street slope

model.npar = 11;

% NONLINEAR INEQUALITY CONSTRAINTS
model.nh = 0;


%% objective function
model.objective = @(z,p) (   p(4)*(z(3)-p(1))^2 + p(5)*(z(4)-p(2))^2 + p(6)*(z(5)-p(3))^2 +p(7)*(z(7)-z(1))^2 + p(8)*(z(8)-z(2))^2 +p(9)*abs(z(1)) + p(10) * abs(z(2)));
%how penalize difference between steps of steer for example?
%introducing another param that is the last output?

%% dynamics
% We use an explicit RK4 integrator here to discretize continuous dynamics:
L=2.35585;% Physical constants of the model
integrator_stepsize = 0.2;
%what is this stepsize?
continuous_dynamics = @(x,u) [x(3)*cos(x(4));  % v*cos(phi) = Xdot
                              x(3)*sin(x(4));  % v*sin(phi) = Ydot
                              u(1);          % a = vdot
                              tan(u(2))*x(3)/L;]         % tan(delta)v/L = phidot
       
%                    state   input    equation              finesse
model.eq = @(z) [RK4( z(3:6), z(1:2), continuous_dynamics, integrator_stepsize);
                    z(1);
                    z(2);]
%insert unstetige funktion für T(U)?
% Indices on LHS of dynamical constraint - for efficiency reasons, make
% sure the matrix E has structure [0 I] where I is the identity matrix.
% forces needs a model E*z_{k+1} = c(z_k)
% where z = [u, x]'
model.E = [zeros(6,2), eye(6)]; %tells you which of 6 elements of z are influenced by init condition
model.neq  = 6;   % number of equality constraints (=states) per step
%% lower and upper bounds
%            a   delta   |   X   Y    v    theta a_old delta_old  ]
model.lb = [ -10  -20*pi/180  -inf  -inf   0    -inf    -10   -25];
model.ub = [ +10  +20*pi/180  +inf  +inf   30  +inf    10    25];
%do we have to define bounds for all z elements? (theta no sense)

% %% "non-simple" inequalities hl <= h(z) <= hu
%  max_change_delta=integrator_stepsize*0.349; %0.1sec*20grad/sec
%  model.nh = 1;
%  model.ineq = @(z,p) abs(z(2)-z(8)); % = cos(theta) <= 1
%  model.hu = max_change_delta;
%  model.hl = 0;
 


%% initial condition, write elements of z affected by initial condition here  z[3] is X
%                 X  Y  v phi 
model.xinitidx = [3, 4, 5, 6];

%% generate solver
codeoptions = getOptions('arc_solver');
codeoptions.maxit = 100;
codeoptions.cleanup = false;
codeoptions.printlevel=1;
FORCES_NLP(model, codeoptions);

%% solve - anything beyond this does not need codegen!
x = linspace(0,10,30);
y = 0.2*x;
p = polyfit(x,y,7);
x1=linspace(0,20,10);    %9 is number of ref points
y1 = polyval(p,x1);
% points to track
Xt =  [x1;y1;5 5 5 5 5 5 5 5 5 5];
%Initial reference 0 0 v_now has to be part of ref vector?
%Xt[i,1] ist referenz nach einem zeitschritt oder bei t=0?
% 1. Fill problem data
problem.x0 = zeros(model.N*model.nvar,1); % initialize to zeros
%what is this for?
problem.all_parameters = [];
for i = 1:model.N
    problem.all_parameters = [problem.all_parameters; Xt(:,i)];
end

% set initial condition
problem.xinit = [0; 0; 0; 0];

%2. call solver
output = arc_solver(problem);


%% plot
Xopt = [];
Yopt = [];
vopt = [];
Fopt = [];
sopt = [];
thetaopt = [];
for i = 1:model.N
    Fopt = [Fopt, output.(sprintf('x%d',i))(1)];
    sopt = [sopt, output.(sprintf('x%d',i))(2)];
    Xopt = [Xopt, output.(sprintf('x%d',i))(3)];
    Yopt = [Yopt, output.(sprintf('x%d',i))(4)];
    vopt = [vopt, output.(sprintf('x%d',i))(5)];
   thetaopt = [thetaopt, output.(sprintf('x%d',i))(6)];
end

figure(1); clf;
plot(Xopt,Yopt);%,x1,y1);

figure(2); clf;
subplot(2,1,1); plot(Fopt); title('acceleration')
subplot(2,1,2); plot(sopt); title('steering');

figure(3);clf;
plot(vopt);