%% Pre init.

clear;
close all;
clc;

% Add path.
addpath('Flip_DYN');

%% System parameters.

% Sample time.
dt = 0.1;
f = 0.85;
% State transition matrix.
F = [f dt;0 f];

% Control matrix.
B = [dt^2/2;dt];

% State dimension.
n = size(F,1);


%% Solve for the Defender Control gain.

% State weight.
Q = 1*eye(n);

% Control dimension.
m = size(B,2);

% Control weight.
R = 1*eye(m);

[~,K,~,~] = idare(F,B,Q,R,[],[]);

%% Parameters

% Assume the adversary control gain is zero.
W = 0;

% Control matrix of the defender.
E = B;

% Horizon length.
L = 100;

% Defender State cost.
Q_d = Q;

% Adversary State cost.
Q_a = 2*eye(n);

% Defender's takeover cost.
D = 0.5*eye(n);
% D = rand(1)*eye(n);

% Adversary's takeover cost.
A = 0.5*eye(n);
% A = rand(1)*eye(n);

% Build the FlipDyn class.
FD = FlipDyn(F,B,E,K,W,L,Q_d,Q_a,D,A);

%% Solve.

% Linear system solve.
FD.n_solve;

%% Simulate the system.

% Number of iterations for simulation.
FD.itr = 100;

% Initial state.
x0 = rand(n,1);

FD.x0 = x0;

% Simulate the system.
[traj_str,obj_str] = FD.simulate_system;

%% Plotting.
mkr = {'o','v','D','s','x','s','v','>','<','p','h'};

% Value function parameters.

% Plot.
figure();
hold on;
cnt = 1;
P_0_norm = zeros(L,1);
for i=1:L+1
    P_0_norm(i,1) = min(eig(FD.p0_f(:,:,i)));
end
y_lo = min(P_0_norm);
L_n = L-1;
plot(P_0_norm(L-L_n:L+1),mkr{cnt},'linewidth',4,...
    'DisplayName','$\lambda_{n}(\hat{P}^{0})$',...
    'MarkerSize',8,'MarkerFaceColor',[1 1 1]);
cnt = cnt + 1;
P_1_norm = zeros(L,1);
for i=1:L+1
    P_1_norm(i,1) = min(eig(FD.p1_f(:,:,i)));
end
y_h = max(P_1_norm);
L_n = L-1;
plot(P_1_norm(L-L_n:L+1),mkr{cnt},'linewidth',4,...
    'DisplayName','$\lambda_{n}(\hat{P}^{1})$',...
    'MarkerSize',8,'MarkerFaceColor',[1 1 1]);
xlabel('Horizon','interpreter','latex');
ylabel('$$\lambda_{n}(\hat{P}^{0/1})$$','interpreter','latex');
legend('location','best','interpreter','latex');
axis tight;
ax = gca;
ax.XAxis.FontSize = 20;
ax.YAxis.FontSize = 20;
ax.Legend.FontSize = 20;


%% Plotting.
mkr = {'o','v','D','s','x','s','v','>','<','p','h'};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Defender Policy | alpha = 0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot.
figure();
hold on;
% Take the mean.
if FD.itr > 1
    def_pol0_mean = mean(FD.def_pol_p0,3);
else
    def_pol0_mean = FD.def_pol_p0;
end
plot(def_pol0_mean(1,:),'-o','linewidth',2);
xlabel('Horizon');
ylabel('Defense, \alpha = 0');
axis tight;
ax = gca;
ax.XAxis.FontSize = 20;
ax.YAxis.FontSize = 20;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Defender Policy | alpha = 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot.
figure();
hold on;
% Take the mean.
if FD.itr > 1
    def_pol1_mean = mean(FD.def_pol_p1,3);
else
    def_pol1_mean = FD.def_pol_p1;
end
plot(def_pol1_mean(1,:),'-o','linewidth',2);
xlabel('Horizon');
ylabel('Defense \alpha = 1');
axis tight;
ax = gca;
ax.XAxis.FontSize = 20;
ax.YAxis.FontSize = 20;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Adversary Policy | alpha = 0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot.
figure();
hold on;
% Take the mean.
if FD.itr > 1
    adv_pol0_mean = mean(FD.adv_pol_p0,3);
else
    adv_pol0_mean = FD.adv_pol_p0;
end
plot(adv_pol0_mean(1,:),'-o','linewidth',2);
xlabel('Horizon');
ylabel('Attack, \alpha = 0');
axis tight;
ax = gca;
ax.XAxis.FontSize = 20;
ax.YAxis.FontSize = 20;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Adversary Policy | alpha = 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot.
figure();
hold on;
% Take the mean.
if FD.itr > 1
    adv_pol1_mean = mean(FD.adv_pol_p1,3);
else
    adv_pol1_mean = FD.adv_pol_p1;
end
plot(adv_pol1_mean(1,:),'-o','linewidth',2);
xlabel('Horizon');
ylabel('Attack, \alpha = 1');
axis tight;
ax = gca;
ax.XAxis.FontSize = 20;
ax.YAxis.FontSize = 20;

