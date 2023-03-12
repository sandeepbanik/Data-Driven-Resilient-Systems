function [P_0,P_1,gamma] = FlipDyn_NS(cs)
% FlipDyn_NS solves the FlipDyn game for a scalar system with
% the parameters sepcified in the FlipDyn class. 
% [P_0,P_1,gamma] = FlipDyn_NS(FD) 
% FD is the FlipDyn class with the required properties.
% P_0 - value function parameter p0, n by n matrix (alpha = 0).
% P_1 - value function parameter p0, n by n matrix (alpha = 0).
% gamma - value function parameter constant mu.
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

if (size(cs.F,1) ~= size(cs.B,1)) ||...
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
P_0 = zeros(n,n,L+1);

% Boundary condition.
P_0(:,:,end) = Q;

% Value function matrix for alpha = 1.
P_1 = zeros(n,n,L+1);

% Boundary condition.
P_1(:,:,end) = Q + max(A,D);

% Flag for mixed policy condition not satisfied.
flag_mp_cond = 0;
stage_mp_cond = 0;

% Defender dynamics.
til_D = (F - B*K);
% Adversary dynamics.
til_W = (F + E*W);

% Loop over L stages
for i=L:-1:1
    % P tilde
    P_til = til_W'*P_1(:,:,i+1)*til_W - til_D'*P_0(:,:,i+1)*til_D;
    % Necessary condition for a mixed policy.
    m_cond = eig(til_W'*P_1(:,:,i+1)*til_W - til_D'*P_0(:,:,i+1)*til_D - max(D,A));
    
    % Checking the mixed policy condition at stage i for alpha = 0.
    if all(m_cond >= 0 )
        % Update the value function for alpha = 0.
        P_0(:,:,i) = Q + D + til_D'*P_0(:,:,i+1)*til_D - (D/(P_til))*A;
        % Update the value function for alpha = 1.
        P_1(:,:,i) = Q - A + til_W'*P_1(:,:,i+1)*til_W + (D/(P_til))*A;
    else
        % Update the mixed policy condition (not satisfied).
        flag_mp_cond = 1;
        % Record the stage at which the mixed policy condition in not
        % satisfied.
        stage_mp_cond = i;
        % Update the value function for alpha = 0.
        P_0(:,:,i) = Q + til_D'*P_0(:,:,i+1)*til_D ;
        % Update the value function for alpha = 1.
        P_1(:,:,i) = Q + til_W'*P_1(:,:,i+1)*til_W;
    end
end

% Constant mu.
gamma = 0;

if flag_mp_cond == 1
    fprintf('Mixed policy condition not satisfied at stage %d\n',stage_mp_cond);
end

%% Search for the constant for which the condition would be satisfied.

if flag_mp_cond == 1
    fprintf('Searching for the constant mu which satisfies the mixed policy condition for all stages\n');
    
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
        P_0 = zeros(n,n,L+1);
        
        % Boundary condition.
        P_0(:,:,end) = Q;
        
        % Value function matrix for alpha = 1.
        P_1 = zeros(n,n,L+1);
        
        % Boundary condition.
        P_1(:,:,end) = Q + max(A,D) + gamma*eye(n);
        
        
        for i=L:-1:1
            % P tilde
            P_til = til_W'*P_1(:,:,i+1)*til_W - til_D'*P_0(:,:,i+1)*til_D;
            % Necessary condition for a mixed policy.
            m_cond = eig(til_W'*P_1(:,:,i+1)*til_W - til_D'*P_0(:,:,i+1)*til_D - max(D,A));
            
            if all(m_cond >= 0 )
                % Update the value function for alpha = 0.
                P_0(:,:,i) = Q + D + til_D'*P_0(:,:,i+1)*til_D - (D/(P_til))*A;
                % Update the value function for alpha = 1.
                P_1(:,:,i) = Q - A + til_W'*P_1(:,:,i+1)*til_W + (D/(P_til))*A;
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
                P_0(:,:,i) = Q + til_D'*P_0(:,:,i+1)*til_D ;
                % Update the value function for alpha = 1.
                P_1(:,:,i) = Q + til_W'*P_1(:,:,i+1)*til_W;
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
            
        end
    end
    
    % Final iteration.
    fprintf('Found the constant mu %.2f \n',gamma);
    % State dimension.
    n = size(A,1);
    
    % Value function matrix for alpha = 0.
    P_0 = zeros(n,n,L+1);
    
    % Boundary condition.
    P_0(:,:,end) = Q;
    
    % Value function matrix for alpha = 1.
    P_1 = zeros(n,n,L+1);
    
    % Boundary condition.
    P_1(:,:,end) = Q + max(A,D) + gamma*eye(n);
    
    % Loop over L stages
    for i=L:-1:1
        % P tilde
        P_til = til_W'*P_1(:,:,i+1)*til_W - til_D'*P_0(:,:,i+1)*til_D;
        % Necessary condition for a mixed policy.
        m_cond = eig(til_W'*P_1(:,:,i+1)*til_W - til_D'*P_0(:,:,i+1)*til_D - max(D,A));
        
        % Checking the mixed policy condition at stage i for alpha = 0.
        if all(m_cond >= 0 )
            % Update the value function for alpha = 0.
            P_0(:,:,i) = Q + D + til_D'*P_0(:,:,i+1)*til_D - (D/(P_til))*A;
            % Update the value function for alpha = 1.
            P_1(:,:,i) = Q - A + til_W'*P_1(:,:,i+1)*til_W + (D/(P_til))*A;
        else
            % Update the value function for alpha = 0.
            P_0(:,:,i) = Q + til_D'*P_0(:,:,i+1)*til_D ;
            % Update the value function for alpha = 1.
            P_1(:,:,i) = Q + til_W'*P_1(:,:,i+1)*til_W;
        end
    end
end
end