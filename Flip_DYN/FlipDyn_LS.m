function [p0_f,p1_f,def_pol_p0,def_pol_p1,adv_pol_p0,adv_pol_p1,gamma] = FlipDyn_LS(cs)
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

%% Checking initial conditions

if size(cs.F,1) > 1 ||...
        size(cs.E,1) > 1 || size(cs.B,1) > 1
    error('System dynamics defined is not scalar');
elseif (size(cs.F,1) ~= size(cs.B,1)) ||...
        (size(cs.F,1) ~= size(cs.E,1))
    error('Dimensions of the State transition matrix, defender control, and adversary control matrix are incompatible');
elseif size(cs.K,1) ~= size(cs.W,1)
    error('Feedback control for the defender and adversary are incompatible');
end

if cs.L < 1 
    error('Horizon length has to be greater than 1');
end


%% Backward iteration.

% State dimension.
n = size(A,1);

% Value function matrix for alpha = 0.
P_0 = zeros(n,L+1);

% Boundary condition.
P_0(:,end) = Q;

% Value function matrix for alpha = 1.
P_1 = zeros(n,L+1);

% Boundary condition.
P_1(:,end) = Q + max(A,D);

% Store policy of attack and defense for each FlipDyn state.
% For alpha = 0
pol_a0_def = zeros(2,L);
pol_a0_att = zeros(2,L);
% For alpha = 1
pol_a1_def = zeros(2,L);
pol_a1_att = zeros(2,L);

% Flag for mixed policy condition not satisfied.
flag_mp_cond = 0;
stage_mp_cond = 0;

% Loop over L stages
for i=L:-1:1
    % Checking the mixed policy condition at stage i for alpha = 0.
    if (F + E*W)^2*P_1(:,i+1) >= (A - B*K)^2*P_0(:,i+1) + max(D,A) 
        % Update the value function for alpha = 0.
        P_0(:,i) = Q + D + (F - B*K)^2*P_0(:,i+1) - D*A/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
        % Store the policy for alpha = 0.
        % Defender policy.
        pol_a0_def(:,i) = [A/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
            ((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1)  - A)/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1))];
        % Adversary policy.
        pol_a0_att(:,i) = [((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1) - D)/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
            D/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1))];
    else
        % Update the mixed policy condition (not satisfied).
        flag_mp_cond = 1;
        % Record the stage at which the mixed policy condition in not
        % satisfied.
        stage_mp_cond = i;
        % Update the value function for alpha = 0.
        P_0(:,i) = Q + (F - B*K)^2*P_0(:,i+1);
        % Store the policy for alpha = 0.
        % Defender policy.
        pol_a0_def(:,i) = [1;0];
        % Adversary policy.
        pol_a0_att(:,i) = [1;0];
        
    end
    % Checking the mixed policy condition at stage i for alpha = 1.
    if (F + E*W)^2*P_1(:,i+1) >= (F - B*K)^2*P_0(:,i+1) + max(D,A)
        % Update the value function for alpha = 1.
        P_1(:,i) = Q - A + (F + E*W)^2*P_1(:,i+1) + D*A/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
        % Store the policy for alpha = 1.
        % Defender policy.
        pol_a1_def(:,i) = [((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1) - A)/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
            A/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1))];
        % Adversary policy.
        pol_a1_att(:,i) = [D/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
            ((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1) -D)/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1))];
    else
        % Update the mixed policy condition (not satisfied).
        flag_mp_cond = 1;
        % Record the stage at which the mixed policy condition in not
        % satisfied.
        stage_mp_cond = i;
        % Update the value function for alpha = 1.
        P_1(:,i) = Q + (F + E*W)^2*P_1(:,i+1);
        % Store the policy for alpha = 1.
        % Defender policy.
        pol_a1_def(:,i) = [1;0];
        % Adversary policy.
        pol_a1_att(:,i) = [1;0];
    end    
end

if flag_mp_cond == 1
    fprintf('Mixed policy condition not satisfied at stage %d',stage_mp_cond);
else
    % Return the parameter of the value function for alpha = 0.
    p0_f = P_0;
    % Return the parameter of the value function for alpha = 1.
    p1_f = P_1;
    % Return the defender and adversary policy of the value function for alpha = 0.
    def_pol_p0 = pol_a0_def;
    adv_pol_p0 = pol_a0_att;
    % Return the defender and adversary policy of the value function for alpha = 1.
    def_pol_p1 = pol_a1_def;
    adv_pol_p1 = pol_a1_att;
    % Constant mu.
    gamma = 0;
end

%% Search for the constant for which the condition would be satisfied.

if flag_mp_cond == 1
    fprintf('Searching for the constant mu which satisfies the mixed policy condition for all stages');
    
    % Upper and lower bound.
    ub_c = 0.01;
    lb_c = 0;
    
    
    % Boundary condition.
    gamma =  ub_c;
    
    % Boolean for lower bound.
    flag_lb = 0;
    % Boolean for termination.
    flag_t = 0;
    
    while flag_t == 0 || flag_lb == 0
        % Dimension of state and control.
        n = size(F,1);
        
        % Value function matrix for alpha = 0.
        P_0 = zeros(n,L+1);
        
        % Boundary condition.
        P_0(:,end) = Q;
        
        % Value function matrix for alpha = 1.
        P_1 = zeros(n,L+1);
        
        % Boundary condition.
        P_1(:,end) = Q + max(A,D)+gamma;
        
        % Store policy of attack and defense.
        pol_a0_def = zeros(2,L);
        pol_a0_att = zeros(2,L);
        pol_a1_def = zeros(2,L);
        pol_a1_att = zeros(2,L);
        
        for i=L:-1:1
            if (F + E*W)^2*P_1(:,i+1) >= (A - B*K)^2*P_0(:,i+1) + max(D,A)
                % Update the value function for alpha = 0.
                P_0(:,i) = Q + D + (F - B*K)^2*P_0(:,i+1) - D*A/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
                % Store the policy for alpha = 0.
                % Defender policy.
                pol_a0_def(:,i) = [A/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
                    ((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1)  - A)/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1))];
                % Adversary policy.
                pol_a0_att(:,i) = [((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1) - D)/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
                    D/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1))];
                if i==1
                    if flag_lb == 1
                        % Cut the upper bound.
                        ub_c = (ub_c + lb_c)/2;
                    end
                    
                    if abs(ub_c - lb_c) <= 0.001
                        flag_t = 1;
                        break
                    end
                    % Set gamma.
                    gamma = (ub_c + lb_c)/2;
                    flag_lb = 1;
                end
            else
                % Update the value function for alpha = 0.
                P_0(:,i) =  Q + D + (F - B*K)^2*P_0(:,i+1);
                % Store the policy for alpha = 0.
                % Defender policy.
                pol_a0_def(:,i) = [1;0];
                % Adversary policy.
                pol_a0_att(:,i) = [1;0];
                
                if flag_lb == 0
                    % Increment the lower bound.
                    lb_c = ub_c;
                    ub_c = ub_c*2;
                    % Set gamma.
                    gamma = ub_c;
                    break
                else
                    % Set the lower bound.
                    lb_c = gamma;
                    % Set gamma.
                    gamma = (ub_c + lb_c)/2;
                    break
                end
            end
            
            if (F + E*W)^2*P_1(:,i+1) >= (F - B*K)^2*P_0(:,i+1) + max(D,A)
                % Update the value function for alpha = 1.
                P_1(:,i) = Q - A + (F + E*W)^2*P_1(:,i+1) + D*A/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
                % Store the policy for alpha = 1.
                % Defender policy.
                pol_a1_def(:,i) = [((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1) - A)/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
                    A/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1))];
                % Adversary policy.
                pol_a1_att(:,i) = [D/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
                    ((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1) -D)/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1))];
            else
                % Update the value function for alpha = 1.
                P_1(:,i) = Q - A + (F + E*W)^2*P_1(:,i+1);
                % Store the policy for alpha = 1.
                % Defender policy.
                pol_a1_def(:,i) = [1;0];
                % Adversary policy.
                pol_a1_att(:,i) = [1;0];
            end
        end
    end
    
    % Final iteration.
    fprintf('Found the constant mu %.2f \n',gamma);
    
    % State dimension.
    n = size(A,1);
    
    % Value function matrix for alpha = 0.
    P_0 = zeros(n,L+1);
    
    % Boundary condition.
    P_0(:,end) = Q;
    
    % Value function matrix for alpha = 1.
    P_1 = zeros(n,L+1);
    
    % Boundary condition.
    P_1(:,end) = Q + max(A,D) + gamma;
    
    % Store policy of attack and defense for each FlipDyn state.
    % For alpha = 0
    pol_a0_def = zeros(2,L);
    pol_a0_att = zeros(2,L);
    % For alpha = 1
    pol_a1_def = zeros(2,L);
    pol_a1_att = zeros(2,L);
    
    
    % Loop over L stages
    for i=L:-1:1
        % Checking the mixed policy condition at stage i for alpha = 0.
        if (F + E*W)^2*P_1(:,i+1) >= (A - B*K)^2*P_0(:,i+1) + max(D,A)
            % Update the value function for alpha = 0.
            P_0(:,i) = Q + D + (F - B*K)^2*P_0(:,i+1) - D*A/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
            % Store the policy for alpha = 0.
            % Defender policy.
            pol_a0_def(:,i) = [A/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
                ((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1)  - A)/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1))];
            % Adversary policy.
            pol_a0_att(:,i) = [((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1) - D)/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
                D/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1))];
        else
            % Update the value function for alpha = 0.
            P_0(:,i) = Q + (F - B*K)^2*P_0(:,i+1);
            % Store the policy for alpha = 0.
            % Defender policy.
            pol_a0_def(:,i) = [1;0];
            % Adversary policy.
            pol_a0_att(:,i) = [1;0];
            
        end
        % Checking the mixed policy condition at stage i for alpha = 1.
        if (F + E*W)^2*P_1(:,i+1) >= (F - B*K)^2*P_0(:,i+1) + max(D,A)
            % Update the value function for alpha = 1.
            P_1(:,i) = Q - A + (F + E*W)^2*P_1(:,i+1) + D*A/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
            % Store the policy for alpha = 1.
            % Defender policy.
            pol_a1_def(:,i) = [((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1) - A)/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
                A/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1))];
            % Adversary policy.
            pol_a1_att(:,i) = [D/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1));
                ((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1) -D)/((F + E*W)^2*P_1(:,i+1) - (F-B*K)^2*P_0(:,i+1))];
        else
            % Update the value function for alpha = 1.
            P_1(:,i) = Q + (F + E*W)^2*P_1(:,i+1);
            % Store the policy for alpha = 1.
            % Defender policy.
            pol_a1_def(:,i) = [1;0];
            % Adversary policy.
            pol_a1_att(:,i) = [1;0];
        end
    end
    % Return the parameter of the value function for alpha = 0.
    p0_f = P_0;
    % Return the parameter of the value function for alpha = 1.
    p1_f = P_1;
    % Return the defender and adversary policy of the value function for alpha = 0.
    def_pol_p0 = pol_a0_def;
    adv_pol_p0 = pol_a0_att;
    % Return the defender and adversary policy of the value function for alpha = 1.
    def_pol_p1 = pol_a1_def;
    adv_pol_p1 = pol_a1_att;
end

end