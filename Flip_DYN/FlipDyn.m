classdef FlipDyn < dynamicprops
    properties
        F % System dynamics
        B % Control matrix
        E % Adversary control matrix
        K % Defender feedback law
        W % Adversary feedback law
        D % Defender cost matrix
        A % Adversary cost matrix
        Q_d % Defender State cost matrix
        Q_a % Adversary State cost matrix
        L % Time horizon
        x0 % Initial state for simulation (Optional)
        itr % Number of iterations (for simulating the system).
    end
    methods
        function cs = FlipDyn(F,B,E,K,W,L,Q_d,Q_a,D,A,x0,itr)
            %
            % FLIPDYN class file.
            % FD = FlipDyn(F,B,E,K,W,L,Q,D,A,x0,itr) creates a class with
            % the following properties.
            % F - System dynamics
            % B - Control matrix
            % E - Adversary control matrix
            % K - Defender feedback law
            % W - Adversary feedback law
            % D - Defender cost matrix
            % A - Adversary cost matrix
            % Q - State cost matrix
            % L - Time horizon
            % x0 - Initial state for simulation (Optional)
            % itr - Number of iterations (for simulating the system).
            % The matrices Q,D,A must be positive-definite.
            % When omitted or set to [], x0 and int default to the values
            % null and 1, respectively.
            arguments
                F double
                B double 
                E double
                K double
                W double
                L double
                Q_d double
                Q_a double
                D double
                A double
                x0 {mustBeNonempty} = false;
                itr double = 1;
            end
            if nargin > 1
                cs.F = F;
                cs.B = B;
                cs.E = E;
                cs.K = K;
                cs.W = W;
                cs.L = L;
                cs.Q_d = Q_d;
                cs.Q_a = Q_a;
                cs.D = D;
                cs.A = A;
                if x0
                    cs.x0 = x0;
                end
                if itr
                    cs.itr = itr;
                else
                    cs.itr = 1;
                end
            end
        end
        % Method to solve a scalar system.
        function scalar_solve(cs)
            % SCALAR_SOLVE solves the FlipDyn game for a scalar system with
            % the parameters sepcified in the FlipDyn class. 
            % FD = FlipDyn(F,B,E,K,W,L,Q,D,A,x0,itr) 
            % FD.scalar_solve returns the following properties.
            % FD.p0_f - value function parameter p0 (alpha = 0).
            % FD.p1_f - value function parameter p0 (alpha = 0).
            % FD.def_pol_p0 - Defender policy corresponding to alpha = 0.
            % FD.def_pol_p1 - Defender policy corresponding to alpha = 1.
            % FD.adv_pol_p0 - Adversary policy corresponding to alpha = 0.
            % FD.adv_pol_p1 - Adversary policy corresponding to alpha = 1.
            % FD.mu - value function parameter constant mu.
            [p0_f,p1_f,def_pol_p0,def_pol_p1,adv_pol_p0,adv_pol_p1,mu] = FlipDyn_LS(cs);
            
            % Store the policy and parameters of the value function.
            % Parameters of the value function for alpha = 0.
            if isprop(cs,'p0_f')==0
                cs.addprop('p0_f');
            end
            cs.p0_f = p0_f;
            % Parameters of the value function for alpha = 1.
            if isprop(cs,'p1_f')==0
                cs.addprop('p1_f');
            end
            cs.p1_f = p1_f;
            % Defender policy for alpha = 0.
            if isprop(cs,'def_pol_p0')==0
                cs.addprop('def_pol_p0');
            end
            cs.def_pol_p0 = def_pol_p0;
            % Defender policy for alpha = 1.
            if isprop(cs,'def_pol_p1')==0
                cs.addprop('def_pol_p1');
            end
            cs.def_pol_p1 = def_pol_p1;
            % Adversary policy for alpha = 0.
            if isprop(cs,'adv_pol_p0')==0
                cs.addprop('adv_pol_p0');
            end
            cs.adv_pol_p0 = adv_pol_p0;
            % Adversary policy for alpha = 1.
            if isprop(cs,'adv_pol_p1')==0
                cs.addprop('adv_pol_p1');
            end
            cs.adv_pol_p1 = adv_pol_p1;
            % Mu.
            if isprop(cs,'mu')==0
                cs.addprop('mu');
            end
            cs.mu = mu;
            
        end
        % Method to solve a n dimensional system.
        function n_solve(cs)
            % n_solve solves the FlipDyn game for a n-dimensional system with
            % the parameters sepcified in the FlipDyn class. 
            % FD = FlipDyn(F,B,E,K,W,L,Q,D,A,x0,itr) 
            % FD.n_solve returns the following properties.
            % FD.p0_f - value function parameter p0, n by n matrix (alpha = 0).
            % FD.p1_f - value function parameter p0, n by n matrix (alpha = 0).
            % FD.mu - value function parameter constant mu.
            [P0,P1,mu] = FlipDyn_NS(cs);
            
            % Store the policy and parameters of the value function.
            % Parameters of the value function for alpha = 0.
            if isprop(cs,'p0_f')==0
                cs.addprop('p0_f');
                cs.p0_f = P0;
            else
                cs.p0_f = P0;
            end
            % Parameters of the value function for alpha = 1.
            if isprop(cs,'p1_f')==0
                cs.addprop('p1_f');
                cs.p1_f = P1;
            else
                cs.p1_f = P1;
            end
            % Mu.
            if isprop(cs,'mu')==0
                cs.addprop('mu');
            end
            cs.mu = mu;
            
        end
        function [traj_str,obj_str] = simulate_system(cs)
            % simulate_system simulates the FlipDyn game after solving for the value function parameters p0 and p1. 
            % FD = FlipDyn(F,B,E,K,W,L,Q,D,A,x0,itr) 
            % [traj_str,obj_str] = FD.simulate_system returns the following items.
            % traj_str - Stored trajectories (itr number of trajectories).
            % obj_str - Objective function for the corresponding trajectories.
            % FD.def_pol_p0 - Defender policy corresponding to alpha = 0.
            % FD.def_pol_p1 - Defender policy corresponding to alpha = 1.
            % FD.adv_pol_p0 - Adversary policy corresponding to alpha = 0.
            % FD.adv_pol_p1 - Adversary policy corresponding to alpha = 1.
            [traj_str,obj_str,def_pol,adv_pol] = simulate_sys(cs);
            % Defender policy for alpha = 0.
            if isprop(cs,'def_pol_p0')==0
                cs.addprop('def_pol_p0');
            end
            cs.def_pol_p0 = def_pol;
            % Defender policy for alpha = 1.
            if isprop(cs,'def_pol_p1')==0
                cs.addprop('def_pol_p1');
            end
            cs.def_pol_p1 = 1 - def_pol;
            % Adversary policy for alpha = 0.
            if isprop(cs,'adv_pol_p0')==0
                cs.addprop('adv_pol_p0');
            end
            cs.adv_pol_p0 = adv_pol;
            % Adversary policy for alpha = 1.
            if isprop(cs,'adv_pol_p1')==0
                cs.addprop('adv_pol_p1');
            end
            cs.adv_pol_p1 = 1 - adv_pol;
        end
        
    end
end