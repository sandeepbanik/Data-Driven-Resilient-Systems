classdef FlipDyn < dynamicprops
    properties
        F % System dynamics
        B % Control matrix
        E % Adversary control matrix
        K % Defender feedback law
        W % Adversary feedback law
        D % Defender cost matrix
        A % Adversary cost matrix
        Q % State cost matrix
        L % Time horizon
        x0 % Initial state for simulation (Optional)
        itr % Number of iterations (for simulating the system).
    end
    methods
        function cs = FlipDyn(F,B,E,K,W,L,x0,itr)
            arguments
                F double
                B double 
                E double
                K double
                W double
                L double
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