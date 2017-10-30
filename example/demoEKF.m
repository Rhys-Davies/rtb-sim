classdef demoEKF < handle

%% demoEKF
%
% Last Modified: October 2017
%
% Author: Rhys Davies
% 
% This is a rather basic Extended Kalman Filter SLAM class. Assumes perfect
% data association (each observation is returned with a unique identifier).
% This could almost be used as a drop in module for a certain subject.
% Hence I will not be documenting it (if I even leave
% it in the rtb-sim repo to begin with).
%
% If for some reason you wish to use to use/modify/redistribute this code,
% I have placed in under a MIT License. 
%
    
    properties
        
        mu_t
        sigma_t
        lndmrk_id
        R
        Q

    end
    
    methods
        
        function obj = demoEKF(init_pose,omega_l,omega_r,omega_d,omega_b)

            obj.mu_t = [init_pose(1);init_pose(2);init_pose(3)];
            obj.sigma_t = zeros(3,3);
            obj.R = [omega_l^2 0; 0 omega_r^2]; % Change this
            obj.Q = [omega_d^2 0; 0 omega_b^2]; % Change this            
        
        end
    
        function [state,sig] = update(obj,odo,sense)
        %% sense is [range,bearing,id]
        % odo = [delta distance, delta bearing]

            delta_d = odo(1);
            delta_theta = odo(2);       
            z = sense;


            %% Do prediction.
            mu_bar = obj.mu_t;
            mu_bar(1:3,1) = obj.update_state(obj.mu_t(1:3), delta_d, delta_theta);

            l = size(obj.lndmrk_id,2);

            Jxr = obj.get_jx(delta_d,mu_bar(3));
            Jx = [Jxr zeros(3,2*l); zeros(2*l,3) eye(2*l,2*l)]; % Jacobian matrix of F wrt the
            JxT = transpose(Jx);

            Jur = obj.get_ju(obj.mu_t(1:3));
            Ju = [Jur; zeros(2*l,2)];
            JuT = transpose(Ju);

            sigma_bar = (Jx * obj.sigma_t * JxT) + (Ju * obj.R * JuT);
    
 
            for y = 1:size(z,1)



                %% Initialize all the landmarks
                % If landmark i not seen before, initializefa
                % mu_bar = mu_bar append initlnd(z,mu_bar)
                % L = get L matrix
                % sigma_bar = [sigma_bar 0;
                %              0         L*Q*Lt]
                % k == -1 if landmark new
                % k == n if landmark seen before (n = index in landmarklist
                % which is directly related to its location in mu_t)
                
                check = find(obj.lndmrk_id == z(y,3),1); 
                
                if isempty(check)

                    new = obj.init_lnd(z(y,:),mu_bar(1:3));
                    
                    obj.lndmrk_id = [obj.lndmrk_id,z(y,3)];
                    mu_bar = [mu_bar;new];

                    [Lz, LzT] = obj.get_L(mu_bar(1:3,1),z(y,:));

                    z1 = zeros(size(sigma_bar,1),2);
                    z2 = zeros(2,size(sigma_bar,2));
                    sigma_bar = [sigma_bar, z1; z2, Lz*obj.Q*LzT];
                    
                    n = size(obj.lndmrk_id,2);

                else
                
                    n = check(1);

                end


            %% Do Estimation


                l = size(obj.lndmrk_id,2);

                a = 4 + (2*(n - 1));

                zti = obj.get_z(mu_bar(1:3,1),mu_bar(a:a+1,1)); 

                s = [z(y,1); z(y,2)] - zti;

                s(2) = wrapToPi(s(2)); %Need to make sure all angle stay between pi and -pi



                [gt, gtT] = obj.get_g(mu_bar(1:3,1),mu_bar(a:a+1,1),zti(1),n,l);

                kt = sigma_bar*gtT*((gt*sigma_bar*gtT) + obj.Q)^(-1); 

                mu_bar = mu_bar + kt*s; % Updated position         

                mu_bar(3) = wrapToPi(mu_bar(3)); % Wrap angles... again.

                sigma_bar = (eye(size(sigma_bar)) - (kt * gt)) * sigma_bar; %Update covariance matrix.


            end

            obj.mu_t = mu_bar;
            obj.sigma_t = sigma_bar;
            
            state = obj.mu_t;
            sig = obj.sigma_t;
             
        end
    end
    
    methods (Static)        
        
        function [Gt,GtT] = get_g(bot,lnd,r,n,totl)
            
            n = n - 1;
            totl = totl - 1;
            G1 = [-(lnd(1)-bot(1))/r,  -(lnd(2)-bot(2))/r, 0;
                    (lnd(2)-bot(2))/(r^2), -(lnd(1)-bot(1))/(r^2), -1];
            G2 =  [(lnd(1)-bot(1))/r,  (lnd(2)-bot(2))/r;
                    -(lnd(2)-bot(2))/(r^2), (lnd(1)-bot(1))/(r^2)];    

            Gt = [G1,zeros(2,n*2),G2,zeros(2,(totl-n)*2)];    

            GtT = transpose(Gt); 
            
        end

        %% Range and bearing from robot to landmark
        function [zti] = get_z(bot,lnd)
            
            r = sqrt((bot(1) - lnd(1))^2 + (bot(2) - lnd(2))^2);
            b = wrapToPi(wrapToPi(atan2(lnd(2) - bot(2), lnd(1) - bot(1))) - bot(3)); % Wrap to pi

            zti = [r ; b];
            
        end   

        %% Update state
        
        function [new_state] = update_state(state, delta_d, delta_th)
            
            xt = state(1) + delta_d * cos(state(3));
            yt = state(2) + delta_d * sin(state(3));
            tht = wrapToPi(state(3) + delta_th); % Wrap to pi

            new_state = [xt ; yt ; tht];
            
        end   

        %% Initialize a new landmark
        function [lnew] = init_lnd(z,mu)
            
            theta = wrapToPi(mu(3)+z(2));
            lnew = [(mu(1) + z(1) * cos(theta));
                    (mu(2) + z(1) * sin(theta))];
                
        end

        %% Change this to reflect onmi wheel robot.
        function [Lz, LzT] = get_L(mu,z)
            
            theta = wrapToPi(mu(3)+z(2));
            Lz = [cos(theta) -z(1)*sin(theta);
                  sin(theta)  z(1)*cos(theta)];   
            LzT = transpose(Lz);
            
        end

        %% And this
        function [jx] = get_jx(delta_d,theta)
            
            jx = [1 0 -delta_d*sin(theta); 
                  0 1  delta_d*cos(theta);
                  0 0  1                ];
              
        end

        %% And this
        function [ju] = get_ju(mu_t)
            
             ju = [cos(mu_t(3)) 0; %-1*delta_d*sin(mu_t(3))
                   sin(mu_t(3)) 0;  %delta_d*cos(mu_t(3))
                   0            1];

        end        
        
        
       
    end
    
end

