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
% p(6) - weight of Phitarget
% p(7) - weight of DeltaAcceleration
% p(8) - weight of DeltaSteer
% p(9) - weight of Acceleration
% p(10) - weight of Steer

% p(11) - street slope

% p(12) - Obstacle1 x
% p(13) - Obstacle1 y
% p(14) - Radius1
% p(15) - Obstacle2 x
% p(16) - Obstacle2 y
% p(17) - Radius2
% p(18) - Obstacle3 x
% p(19) - Obstacle3 y
% p(20) - Radius3
% p(21) - Obstacle4 x
% p(22) - Obstacle4 y
% p(23) - Radius4
% p(24) - Obstacle5 x
% p(25) - Obstacle5 y
% p(26) - Radius5

%p(27) -Weight Obstacle1
%p(28) -Weight Obstacle2
%p(29) -Weight Obstacle3
%p(30) -Weight Obstacle4
%p(31) -Weight Obstacle5

model.npar = 31;

% NONLINEAR INEQUALITY CONSTRAINTS
model.nh = 5;

L=2.35585;% Physical constants of the model
S=0.4;
%% objective function

model.objective = @(z,p) (   p(4)*(z(3)-p(1))^2 + p(5)*(z(4)-p(2))^2 + p(6)*(z(6)-p(3))^2 ...
                            +p(7)*(z(7)-z(1))^2+ p(8)*(z(8)-z(2))^2 +p(9)*(z(1))^2 + p(10)*(z(2))^2 ...
                            +p(27)* exp(-2*((z(3)-p(12)+cos(z(6))*L/2)^2 + (z(4)-p(13)+sin(z(6))*L/2)^2 -(p(14)+S)^2 )) ...
                            +p(28)* exp(-2*((z(3)-p(15)+cos(z(6))*L/2)^2 + (z(4)-p(16)+sin(z(6))*L/2)^2 -(p(17)+S)^2 )) ...
                            +p(29)* exp(-2*((z(3)-p(18)+cos(z(6))*L/2)^2 + (z(4)-p(19)+sin(z(6))*L/2)^2 -(p(20)+S)^2 )) ...
                            +p(30)* exp(-2*((z(3)-p(21)+cos(z(6))*L/2)^2 + (z(4)-p(22)+sin(z(6))*L/2)^2 -(p(23)+S)^2 )) ...
                            +p(31)* exp(-2*((z(3)-p(24)+cos(z(6))*L/2)^2 + (z(4)-p(25)+sin(z(6))*L/2)^2 -(p(26)+S)^2 )) );

%% dynamics
% We use an explicit RK4 integrator here to discretize continuous dynamics:

integrator_stepsize = 0.2;
%what is this stepsize?
continuous_dynamics = @(x,u) [x(3)*cos(x(4));  % v*cos(phi) = Xdot
                              x(3)*sin(x(4));  % v*sin(phi) = Ydot
                              u(1);          % a = vdot
                              tan(u(2)/0.75)*x(3)/L;]         % tan(delta)v/L = phidot
       
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
%            a      delta   |   X   Y    v    theta a_old delta_old  ]
model.lb = [ -10  -20*pi/180  -inf  -inf   0    -inf    -10   -25];
model.ub = [ +10  +20*pi/180  +inf  +inf   30   +inf    10    25];
%do we have to define bounds for all z elements? (theta no sense)

% %% "non-simple" inequalities hl <= h(z) <= hu
   model.nh = 0;
%   model.ineq = @(z,p) [    ( (z(3)- p(12)+L/2)^2 + (z(4)-p(13) )^2) / (p(14)^2);
%                            ( (z(3)- p(15)+L/2)^2 + (z(4)-p(16) )^2) / (p(17)^2);
%                            ( (z(3)- p(18)+L/2)^2 + (z(4)-p(19) )^2) / (p(20)^2);
%                            ( (z(3)- p(21)+L/2)^2 + (z(4)-p(22) )^2) / (p(23)^2);
%                            ( (z(3)- p(24)+L/2)^2 + (z(4)-p(25) )^2) / (p(26)^2);  ]; % = cos(theta) <= 1
%   model.hu = +inf*ones(5,1);
%   model.hl = 1*ones(5,1);
 


%% initial condition, write elements of z affected by initial condition here  z[3] is X
%                 X  Y  v phi a_old d_old
model.xinitidx = [3, 4, 5, 6, 7, 8];

%% generate solver
codeoptions = getOptions('arc_solver');
codeoptions.maxit = 700;
codeoptions.cleanup = false;
codeoptions.printlevel=1;
codeoptions.win = 0;
codeoptions.mac = 0;
codeoptions.gnu = 1;
codeoptions.platform = 'x86_64';
codeoption.optlevel=3;
FORCES_NLP(model, codeoptions);