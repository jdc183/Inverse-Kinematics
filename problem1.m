global motoman
% Symbolic variables for joint angles
q = sym('q%d', [6 1]);

% DH parameters:
a = [.145;1.15;.25;0;0;0];
d = [.540;0;0;-1.812;0;-0.1];
alpha = [3*pi/2;-pi;-pi/2;pi/2;3*pi/2;pi];
% i6 = eye(6);
% i6(3,3)=-1;
theta = q + [0;-pi/2;0;0;0;0];

R6_0 = [0.707107  0.0  0.707107;
        0.0      -1.0  0.0     ;
        0.707107  0.0 -0.707107];
qs = zeros(6,40);
poses = zeros(4,4,40);
for i = 1:40
    o6_0 = [2.0; -2.0+i*0.1; 0.2];
    motoman = kinchain(q,a,d,alpha,theta);
    qs(:,i) = ik(R6_0,o6_0);
    poses(:,:,i) = motoman.computePose(qs(:,i));
    if (mod(i,5)==0)
        motoman.visualizePose(qs(:,i));
        drawnow
    end
    hold on
end
% plot3([0,o6_0(1)],[0,o6_0(2)],[0,o6_0(3)],'p-','LineWidth',1.0);

function q = ik(R6_0,o6_0)
    global motoman
    d6 = motoman.d(6);
    o4_0 = o6_0 + d6*R6_0(:,3);
    x = o4_0(1);
    y = o4_0(2);
    
    %Simple arctangent
    q1 = atan2(y,x);

    A1_0 = double(subs(motoman.As(:,:,1),'q1',q1));
    
    % X and Y values wrt frame 1
    x = sqrt((o4_0(1)-A1_0(1,4))^2 + (o4_0(2)-A1_0(2,4))^2);
    y = o4_0(3)-A1_0(3,4);
    
    %Distance from wrist point to frame 1 origin
    r = sqrt(x^2+y^2);
    
    %Angle between r and horizontal
    beta = atan2(y,x);

    %DH parameters
    a3 = motoman.a(3);
    d4 = motoman.d(4);
    a2 = motoman.a(2);
    
    % angle between humerus and vertical = 90 - beta - law of cosines
    q2 = pi/2-beta-acos((a2^2+r^2-a3^2-d4^2)/(2*a2*r));
    
    % angle between humerus and a3 portion of the forearm link
    % 180 - fixed elbow angle - law of cosines 
    q3 = -pi-atan2(d4,a3)+acos((a2^2+a3^2+d4^2-r^2)/(2*a2*sqrt(a3^2+d4^2)));
    
    % Compute transform to wrist point
    A3_0 = A1_0*double(subs(motoman.As(:,:,2),'q2',q2)*subs(motoman.As(:,:,3),'q3',q3));
    R3_0 = A3_0(1:3,1:3);
    
    % Rotation of end effector wrt frame 3
    R6_3 = transpose(R3_0)*R6_0;
    
    r33 = R6_3(3,3);
    r23 = R6_3(2,3);
    r13 = R6_3(1,3);
    r32 = R6_3(3,2);
    r31 = R6_3(3,1);
    
    % Simple inverse trig to compute qs from R matrix
    q5 = -pi+acos(r33);
    q4 = atan2(r23,r13)-pi;
    
    % Angle between desired frame 6 and frame 6 with q6=0
    A6_0_0q6 = motoman.computePose([q1;q2;q3;q4;q5;0]);
    R6_0_0q6 = A6_0_0q6(1:3,1:3);
    
    q6 = real(acos(dot(R6_0_0q6(:,2),R6_0(:,2))));
    
    q = [q1;q2;q3;q4;q5;q6];
end