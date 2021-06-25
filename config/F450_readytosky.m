% copy of https://github.com/ctu-mrs/uav_core/commit/4713ff8b0538a8688b2d875318756e966ebf06c3#diff-b13142480ad366aa29db967be0926851f6805ea0cf207cf7c73f9de33c03bd8d
% computes ka and kb from two-point measured thrust

% masses of UAV
mass = [
2.08;
2.63;
];

% thrusts needed to hover
thrust = [
0.59;
0.72;
];

n_motors = 4;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% the gravitational acceleration
g = 9.81;

% create the main matrix
A = ones(length(mass), 2);

for i=1:length(mass)
  A(i, 1) = sqrt((mass(i)*g)/n_motors);
end

% print A
A

% compute the linear coeficients
X = A\thrust;

% plot the constants
ka = X(1)
kb = X(2)