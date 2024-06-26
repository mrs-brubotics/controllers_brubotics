% copy from https://github.com/ctu-mrs/uav_core/blob/master/miscellaneous/thrust_constants/motor_thrust_curve_estimation/readytosky_2312_920kv_8045.m
% but changed propellor to Aliexpress plastic 9450 self-tightening
% propeller of https://ctu-mrs.github.io/docs/hardware/motor_tests.html
% computes ka and kb from two-point measured thrust

% masses of UAV
mass = [
0.395;
0.509;
0.634;
0.745;
0.893;
0.997;
];

% thrusts needed to hover
thrust = [
0.5;
0.6;
0.7;
0.8;
0.9;
1.0;
];

thrust

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% the gravitational acceleration
g = 9.81;

% create the main matrix
A = ones(length(mass), 3);

for i=1:length(mass)
  A(i, 1) = sqrt((mass(i)*g));
  A(i, 2) = 1.0;
  A(i, 3) = mass(i)*g*0;
end

% print A
A;

% compute the linear coeficients
X = A\thrust;

% plot the constants
ka = X(1)
kb = X(2)
kc = X(3)

% plot
fig = figure(1);
y = 0:0.01:mass(end);
x = [];
for i=1:length(y)
  x(i) = ka*sqrt(y(i)*g) + kb + kc*y(i)*g*0;
end

hold off
plot(x, y, 'linewidth', 3)
hold on
scatter(thrust, mass, 'x', 'linewidth', 3)
xlabel('throttle [-]')
ylabel('thrust [kg]')