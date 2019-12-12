close all, clear all;

%EXPERIMENTAL CASES
% case 1
L1 = 150; o1 = 30; L2 = 80; o2 = 0; L3 = 40; o3 = 0;

%COST FUNCTION PARAMETERS
%symmetric costs
c1 = 100; c2 = 100; c3 = 100;

rho = 0.1;
maxiter = 20;

%SYSTEM CALIBRATION PARAMETERS
% k11 = 0.4; k12 = 0.01; k13 = 0.01;
% k21 = 0.01; k22 = 0.4; k23 = 0.01;
% k31 = 0.01; k32 = 0.01; k33 = 0.4;

k11 = 2; k12 = 1; k13 = 1;
k21 = 1; k22 = 2; k23 = 1;
k31 = 1; k32 = 1; k33 = 2;

%VARIABLES FOR CENTRALIZED SOLUTION
K = [k11, k12, k13;
    k21, k22, k23;
    k31, k32, k33];
c = [c1;c2;c3];
L = [L1;L2;L3];
o = [o1;o2;o3];

%VARIABLES FOR STORING THE HISTORY OF THE DISTRIBUTED SOLUTION
d11 = zeros(maxiter); d12 = zeros(maxiter); d13 = zeros(maxiter);
d21 = zeros(maxiter); d22 = zeros(maxiter); d23 = zeros(maxiter);
d31 = zeros(maxiter); d32 = zeros(maxiter); d33 = zeros(maxiter);

av1 = zeros(maxiter); av2 = zeros(maxiter); av3 = zeros(maxiter);

%DISTRIBUTED NODE INITIALIZATION
%node1
node1.index = 1;
node1.d = [0;0;0];
node1.d_av = [0;0;0];
node1.y = [0;0;0];
node1.k = [k11;k12;k13];
node1.n = norm(node1.k)^2;
node1.m = node1.n-k11^2;
node1.c = [c1;0;0];
node1.o = o1;
node1.L = L1;
%node2
node2.index = 2;
node2.d = [0;0;0];
node2.d_av = [0;0;0];
node2.y = [0;0;0];
node2.k = [k21;k22;k23];
node2.n = norm(node2.k)^2;
node2.m = node2.n-k22^2;
node2.c = [0;c2;0];
node2.o = o2;
node2.L = L2;
%node3
node3.index = 3;
node3.d = [0;0;0];
node3.d_av = [0;0;0];
node3.y = [0;0;0];
node3.k = [k31;k32;k33];
node3.n = norm(node3.k)^2;
node3.m = node3.n-k33^2;
node3.c = [0;0;c3];
node3.o = o3;
node3.L = L3;

%DISTRIBUTED SOLVER
%Initial condition (iteration = 1)
d11(1) = node1.d(1); d12(1) = node1.d(2); d13(1) = node1.d(3);
d21(1) = node2.d(1); d22(1) = node2.d(2); d23(1) = node2.d(3);
d31(1) = node3.d(1); d22(1) = node3.d(2); d23(1) = node3.d(3);

av1(1) = (d11(1)+d21(1)+d31(1))/3;
av2(1) = (d12(1)+d22(1)+d32(1))/3;
av3(1) = (d13(1)+d23(1)+d33(1))/3;
%iterations
for i=2:maxiter
    %COMPUTATION OF THE PRIMAL SOLUTIONS
    % node 1
    [d1, cost1] = consensus_iterate(node1, rho);
    node1.d = d1;
    %node2
    [d2, cost2] = consensus_iterate(node2, rho);
    node2.d = d2;
    %node3
    [d3, cost3] = consensus_iterate(node3, rho);
    node3.d = d3;
    
    %COMPUTATION OF THE AVERAGE
    %node 1
    node1.d_av = (node1.d+node2.d+node3.d)/3;
    %node 2
    node2.d_av = (node1.d+node2.d+node3.d)/3;
    %node 3
    node3.d_av = (node1.d+node2.d+node3.d)/3;
    
    %COMPUTATION OF THE LAGRANGIAN UPDATES
    %node 1
    node1.y = node1.y + rho*(node1.d-node1.d_av);
    %node 2
    node2.y = node2.y + rho*(node2.d-node2.d_av);
    %node 3
    node3.y = node3.y + rho*(node3.d-node3.d_av);
    
    %SAVING DATA FOR PLOTS
    d11(i) = node1.d(1); d12(i) = node1.d(2); d13(i) = node1.d(3);
    d21(i) = node2.d(1); d22(i) = node2.d(2); d23(i) = node2.d(3);
    d31(i) = node3.d(1); d22(i) = node3.d(2); d23(i) = node3.d(3);
    
    av1(i) = (d11(1)+d21(1)+d31(1))/3;
    av2(i) = (d12(1)+d22(1)+d32(1))/3;
    av3(i) = (d13(1)+d23(1)+d33(1))/3;
end

disp('Consensus Solutions')
d = node1.d_av
l = K*d+o

%FUNCTION TO COMPUTE THE AUGMENTED LAGRANGIAN COST AT A POSSIBLE SOLUTION
%USED BY CONSENSUS_ITERATE
function cost = evaluate_cost(node,d,rho)
cost =  node.c'*d + node.y'*(d-node.d_av) + ...
    rho/2*norm(d-node.d_av)^2;
end

%FUNCTION TO CHECK SOLUTION FEASIBILITY
function check = check_feasibility(node, d)
tol = 0.001; %%tolerance for rounding errors
if (d(node.index) < 0-tol), check = 0; return; end;
if (d(node.index) > 100+tol), check = 0; return; end;
if (d'*node.k < node.L-node.o-tol), check = 0; return; end;
check = 1;
end

%FUNCTION TO COMPUTE THE PRIMAL SOLUTION
function [d, cost] = consensus_iterate(node, rho)
d_best = [-1,-1,-1]';
cost_best = 1000000; %large number
sol_unconstrained = 1;
sol_boundary_linear = 1;
sol_boundary_0 = 1;
sol_boundary_100 = 1;
sol_linear_0 = 1;
sol_linear_100 = 1;
z = rho*node.d_av - node.y - node.c;
%unconstrained minimum
d_u = (1/rho)*z;
sol_unconstrained = check_feasibility(node,d_u);
if sol_unconstrained
    %REVISE: IF UNCONSTRAINED SOLUTION EXISTS, THEN IT IS OPTIMAL
    %NO NEED TO COMPUTE THE OTHER
    cost_unconstrained = evaluate_cost(node, d_u,rho);
    if cost_unconstrained < cost_best
        d_best = d_u;
        cost_best = cost_unconstrained;
    end;
end;
%compute minimum constrained to linear boundary
d_bl = (1/rho)*z - node.k/node.n*(node.o-node.L+(1/rho)*z'*node.k);
%check feasibility of minimum constrained to linear boundary
sol_boundary_linear = check_feasibility(node, d_bl);
% compute cost and if best store new optimum
if sol_boundary_linear
    cost_boundary_linear = evaluate_cost(node, d_bl,rho);
    if cost_boundary_linear < cost_best
        d_best = d_bl;
        cost_best = cost_boundary_linear;
    end;
end;
%compute minimum constrained to 0 boundary
d_b0 = (1/rho)*z;
d_b0(node.index) = 0;
%check feasibility of minimum constrained to 0 boundary
sol_boundary_0 = check_feasibility(node, d_b0);
% compute cost and if best store new optimum
if sol_boundary_0
    cost_boundary_0 = evaluate_cost(node, d_b0,rho);
    if cost_boundary_0 < cost_best
        d_best = d_b0;
        cost_best = cost_boundary_0;
    end;
end;
%compute minimum constrained to 100 boundary
d_b1 = (1/rho)*z;
d_b1(node.index) = 100;
%check feasibility of minimum constrained to 100 boundary
sol_boundary_100 = check_feasibility(node, d_b1);
% compute cost and if best store new optimum
if sol_boundary_100
    cost_boundary_100 = evaluate_cost(node, d_b1,rho);
    if cost_boundary_100 < cost_best
        d_best = d_b1;
        cost_best = cost_boundary_100;
    end;
end;
% compute minimum constrained to linear and 0 boundary
d_l0 = (1/rho)*z - ...
    (1/node.m)*node.k*(node.o-node.L) + ...
    (1/rho/node.m)*node.k*(node.k(node.index)*z(node.index)-z'*node.k);
d_l0(node.index) = 0;
%check feasibility of minimum constrained to linear and 0 boundary
sol_linear_0 = check_feasibility(node, d_l0);
% compute cost and if best store new optimum
if sol_linear_0
    cost_linear_0 = evaluate_cost(node, d_l0,rho);
    if cost_linear_0 < cost_best
        d_best = d_l0;
        cost_best = cost_linear_0;
    end;
end;
% compute minimum constrained to linear and 100 boundary
d_l1 = (1/rho)*z - ...
    (1/node.m)*node.k*(node.o-node.L+100*node.k(node.index)) + ...
    (1/rho/node.m)*node.k*(node.k(node.index)*z(node.index)-z'*node.k);
d_l1(node.index) = 100;
%check feasibility of minimum constrained to linear and 0 boundary
sol_linear_0 = check_feasibility(node, d_l1);
% compute cost and if best store new optimum
if sol_linear_0
    cost_linear_0 = evaluate_cost(node, d_l1,rho);
    if cost_linear_0 < cost_best
        d_best = d_l1;
        cost_best = cost_linear_0;
    end;
end;
d = d_best;
cost = cost_best;
end

