function [traj_str,obj_str,def_pol,att_pol] = simulate_sys(cs)
%% Parameters
F = cs.F;   % System dynamics
B = cs.B;   % Control matrix
E = cs.E;   % Adversary control matrix
K = cs.K;   % Defender feedback law
W = cs.W;   % Adversary feedback law
L = cs.L;   % Time horizon
D = cs.D;   % Defender cost matrix
A = cs.A;   % Adversary cost matrix
Q = cs.Q;   % State cost matrix
% x0 - Initial state.
x0 = cs.x0;
% Parameters of the value function for alpha = 0.
P_0_str = cs.p0_f;
% Parameters of the value function for alpha = 1.
P_1_str = cs.p1_f;
mu = cs.mu;

%% LQR Simulation.

% State dimension.
n = size(F,1);

% State iteration.
itr_s = cs.itr;

% FlipDyn state. 
F_s = zeros(L+1,itr_s);

% Save the policy.
def_pol = zeros(2,L,itr_s);
att_pol = zeros(2,L,itr_s);

% Trajectory save.
traj_str = zeros(n,L+1,itr_s);

% Initialize the state.
x_init = x0;


% Objective function.
obj_str = zeros(L+1,itr_s);

% Defender dynamics.
til_D = F - B*K;

% Attacker dynamics. 
til_W = F + E*W;


for it_s = 1:itr_s
    fprintf('Iteration number %d\n',it_s);
    
    
    % State vector;
    x_str_local = zeros(n,L+1);
    x_str_local(:,1) = x_init;
    
    % Iterate over the horizon
    for time=1:L
        fprintf('Time %d\n',time);
        if time == 24
            disp('hold');
        end
        % Value function difference.
        if n > 1
            P1_temp = P_1_str(:,:,time+1);
            P0_temp = P_0_str(:,:,time+1);
        else
            P1_temp = P_1_str(:,time+1);
            P0_temp = P_0_str(:,time+1);
        end
            
        P_til = til_W'*P1_temp*til_W - til_D'*P0_temp*til_D;
        % Determine the policy.
        % Defender policy.
        def_p_m = [x_str_local(:,time)'*A*x_str_local(:,time)/(x_str_local(:,time)'*P_til*x_str_local(:,time));
            x_str_local(:,time)'*(P_til - A)*x_str_local(:,time)/(x_str_local(:,time)'*P_til*x_str_local(:,time))];
        % Attacker policy.
        att_p_m = [x_str_local(:,time)'*(P_til - D)*x_str_local(:,time)/(x_str_local(:,time)'*P_til*x_str_local(:,time));
            x_str_local(:,time)'*D*x_str_local(:,time)/(x_str_local(:,time)'*P_til*x_str_local(:,time))];
        
        % Save the policy.
        def_pol(:,time,it_s) = def_p_m;
        att_pol(:,time,it_s) = att_p_m;
        
%         if sum(def_p_m)~=1 || sum(att_p_m)~=1
% %             disp('Hold');
%         end
        %%%%%%%%%%%%%%%%%%%%%%
        % Alpha = 0.
        %%%%%%%%%%%%%%%%%%%%%%
        if F_s(time,it_s)==0
            % Defender policy.
            def_p_al0 = def_p_m;
            % Defender policy.
            att_p_al0 = att_p_m;
        else
        %%%%%%%%%%%%%%%%%%%%%%
        % Alpha = 1.
        %%%%%%%%%%%%%%%%%%%%%%
            def_p_al1 = 1 - def_p_m;
            % Defender policy.
            att_p_al1 = 1 - att_p_m;
        end
        
        % Sample the policy.
        if F_s(time,it_s)==0
            % Defender policy.
            def_pol(:,time) = def_p_al0;
            % Attacker policy.
            att_pol(:,time) = att_p_al0;
            def_p = randsample(2,1,'true',def_p_al0);
            if def_p==1
                def_p = 0;
            else
                def_p = 1;
            end
            att_p = randsample(2,1,'true',att_p_al0);
            if att_p==1
                att_p = 0;
            else
                att_p = 1;
            end
        else
            % Defender policy.
            def_pol(:,time) = def_p_al1;
            % Attacker policy.
            att_pol(:,time) = att_p_al1;
            def_p = randsample(2,1,'true',def_p_al1);
            if def_p==1
                def_p = 0;
            else
                def_p = 1;
            end
            att_p = randsample(2,1,'true',att_p_al1);
            if att_p==1
                att_p = 0;
            else
                att_p = 1;
            end
        end
        % Dynamics.
        if F_s(time,it_s)==0
            x_str_local(:,time+1) = til_D*x_str_local(:,time);
        else
            x_str_local(:,time+1) = til_W*x_str_local(:,time);
        end
        % Store objective.
        obj_str(time,it_s) = x_str_local(:,time)'*Q*x_str_local(:,time) +...
                        (1 - F_s(time,it_s))*x_str_local(:,time)'*D*x_str_local(:,time) - ...
                        mu*F_s(time,it_s)*x_str_local(:,time)'*A*x_str_local(:,time);
        % Update the FlipDyn state.
        F_s(time+1,it_s) = ((1-def_p)*(1-att_p) + def_p*att_p)*F_s(time,it_s) + (1-def_p)*(def_p + att_p);
    end
    % Save the trajectory.
    traj_str(:,:,it_s) = x_str_local;
end

end