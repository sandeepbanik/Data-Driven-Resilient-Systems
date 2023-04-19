%% Pre init.

clear;
close all;
clc;

% Add path.
addpath('Flip_DYN');

% Save data.
s_data = 1;

%% System parameters.

% Sample time.
dt = 0.1;

% State transition matrix.
F = 0.85;

% Control matrix.
B = dt;

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
L = 25;

% Defender's takeover cost.
D = 0.5;
% D = rand(1);

% Adversary's takeover cost.
A = 0.25;
% A = rand(1);

% Adversary state cost.
Q_a = 1.0;

% Defender state cost.
Q_d = 1.0;

% Build the FlipDyn class.
FD = FlipDyn(F,B,E,K,W,L,Q_d,Q_a,D,A);

%% Solve.

% Linear system solve.
FD.scalar_solve;

%% Simulate the system.
% 
% % Number of iterations for simulation.
% FD.itr  = 100;
% 
% % Initial state.
% x0 = rand(1);
% 
% FD.x0 = x0;
% 
% % Simulate the system.
% [traj_str,obj_str] = FD.simulate_system;

%% Plotting.
% Value function parameters.

% Plot.
figure();
hold on;
plot(FD.p0_f,'-o','linewidth',2,'DisplayName','P^{0}');
plot(FD.p1_f,'-s','linewidth',2,'DisplayName','P^{1}');
xlabel('Horizon');
ylabel('p_0^{1/0}');
legend('location','best');
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
plot(adv_pol0_mean(1,:),'-o','linewidth',2);xlabel('Horizon');
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

%% Save

if s_data == 1
    if F < 1
        p_s = FD.p0_f;
        save('p0_leq_1.mat','p_s');
        p_s = FD.p1_f;
        save('p1_leq_1.mat','p_s');
        def_p_s = FD.def_pol_p0(2,:);
        save('d_TP_leq_1.mat','def_p_s');
        adv_p_s = FD.adv_pol_p0(2,:);
        save('a_TP_leq_1.mat','adv_p_s');
    elseif F == 1
        
        p_s = FD.p0_f;
        save('p0_equal_1.mat','p_s');
        p_s = FD.p1_f;
        save('p1_equal_1.mat','p_s');
        def_p_s = FD.def_pol_p0(2,:);
        save('d_TP_equal_1.mat','def_p_s');
        adv_p_s = FD.adv_pol_p0(2,:);
        save('a_TP_equal_1.mat','adv_p_s');
    else
        p_s = FD.p0_f;
        save('p0_geq_1.mat','p_s');
        p_s = FD.p1_f;
        save('p1_geq_1.mat','p_s');
        def_p_s = FD.def_pol_p0(2,:);
        save('d_TP_geq_1.mat','def_p_s');
        adv_p_s = FD.adv_pol_p0(2,:);
        save('a_TP_geq_1.mat','adv_p_s');
    end
end

