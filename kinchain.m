classdef kinchain
    %KINCHAIN a class for modeling open kinematic chains
    %   Provides utilities for visualization and forward kinematics.
    %   Computes symbolic forward kinematic transformation matrix to
    %   simplify future computations
    
    properties
        n %number of joints
        q %Symbolic actuator parameters
        a %DH parameter 'a'
        d %DH parameter 'd'
        alpha %DH parameter 'alpha'
        theta %DH parameter 'theta'
        As %Sequence of symbolic transformation matrices between each frame
        A %Symbolic transformation matrix between base and end-effector
    end
    
    methods
        function obj = kinchain(q,a,d,alpha,theta)
            %KINCHAIN constructor
            %   Accepts double DH parameters or symbolic DH parameters in
            %   terms of symbolic actuator parameters q
            n = size(a,1);
            obj.n = n;
            obj.a = a;
            obj.d = d;
            obj.alpha = alpha;
            obj.theta = theta;
            obj.q = q;


            a = permute(a,[3,2,1]);
            d = permute(d,[3,2,1]);
            alpha = permute(alpha,[3,2,1]);
            theta = permute(theta,[3,2,1]);
            
            obj.As=[cos(theta), -sin(theta).*cos(alpha),  sin(theta).*sin(alpha), a.*cos(theta);
                    sin(theta),  cos(theta).*cos(alpha), -cos(theta).*sin(alpha), a.*sin(theta);
                  zeros(1,1,n),              sin(alpha),              cos(alpha),             d;
                  zeros(1,1,n),            zeros(1,1,n),            zeros(1,1,n),   ones(1,1,n)];
            
            A = eye(4);
            for (i=1:n)
                A = A*obj.As(:,:,i);
            end
            obj.A = A;
        end
        
        function An1 = computePose(obj,q)
            %COMPUTEPOSE does forward kinematics of thechain
            %   Quickly computes transformation from base to end effector
            %   given explicit actuator parameters q
            An1 = double(subs(obj.A,obj.q,q));
        end
        
        function void = visualizePose(obj,q)
            %VISUALIZEPOSE visualize the pose using plot3
            
            n = obj.n;
            x = zeros(1,n+1);
            y = x;
            z = x;
            As = double(subs(obj.As,obj.q,q));
            A = eye(4);
            
            %Plot axes for the base frame
            plot3([0,.1],[0,0],[0,0],'r-',[0,0],[0,.1],[0,0],'g-',[0,0],[0,0],[0,.1],'b-','LineWidth',2.0)
            hold all
            for (i=1:n)
                j = i+1;
                A = A*As(:,:,i);
                x(i+1) = A(1,4);
                y(i+1) = A(2,4);
                z(i+1) = A(3,4);
                
                % Plot axes for each frame
                plot3([x(j),x(j)+.1*A(1,1)],[y(j),y(j)+.1*A(2,1)],[z(j),z(j)+.1*A(3,1)],'r-','LineWidth',2.0)
                plot3([x(j),x(j)+.1*A(1,2)],[y(j),y(j)+.1*A(2,2)],[z(j),z(j)+.1*A(3,2)],'g-','LineWidth',2.0)
                plot3([x(j),x(j)+.1*A(1,3)],[y(j),y(j)+.1*A(2,3)],[z(j),z(j)+.1*A(3,3)],'b-','LineWidth',2.0)
            end
            
            axis equal;
            plot3(x,y,z,'k-','LineWidth',1.0);
            hold off;
        end
    end
end

