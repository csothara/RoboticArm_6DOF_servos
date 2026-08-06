clear all;
%%%LengthParameters (cm)   *Origin<=>Elbow
d1=6.5;  %UpperArm Shoulder-Elbow
d2=7;  %Forearm Elbow-Wrist
d3=6.5;  %Hand Wrist-Claw (gripping position)

%%%INPUTS COORDINATES & Angle: 
X=10;
Y=5;
A3=5*pi/7; %Angle of End Effector / Claw


R=sqrt(X^2+Y^2); % Distance (0,0) to (X,Y)

% if R > d1+d2+d3
%     error('position unreachable.');
% end


%%%INVERSE KINEMATICS : Cosinus Law, Origin to Wrist

%INTERMEDIATE COORDINATES : Wrist(Mx,My)
Mx=X-d3*sin(A3);
My=Y-d3*cos(A3);

r=sqrt(Mx^2+My^2); % Distance (0,0) to (Mx,My)

a=d1;b=d2;c=r;
A1=-(-pi/2 + atan2(My,Mx) + acos( (b^2-a^2-c^2)/(-2*a*c)) );
A2=-(-pi + acos( (Mx^2+My^2-a^2-b^2)/(-2*a*b) ) );
deg1=A1*360/(2*pi);
deg2=A2*360/(2*pi);


% Ob=acos( (b^2-a^2-c^2)/(-2*a*c))
% Oc=acos( (b^2-a^2-c^2)/(-2*a*c) )
% A1=-(-pi/2 + atan(y/x) + Ob)
% A2=-(-pi + Oc)

%Rotation Matrix
R1=[cos(A1) sin(A1) ; -sin(A1) cos(A1)];%Shoulder
R2=[cos(A2) sin(A2) ; -sin(A2) cos(A2)];%Elbow
R3=[cos(A3) sin(A3) ; -sin(A3) cos(A3)];%Wrist


%Default Vectors (Arm in default position : Ai=0)
D1=[0 ; d1]; %Shoulder-Elbow
D2=[0 ; d2]; %Elbow-Wrist
D3=[0 ; d3]; %Wrist-Claw

%Successive Transformations: Origin to EndEffector (Translations+Rotations)
T1=R1*D1;  %Elbow
T2=T1+R2*R1*D2;  %Wrist
T3 = T2 + R3 * D3;  %Claw END

%Points
plt1=[[0;0]  T1]; % [Shoulder , Elbow]
plt2=[T1 , T2]; % [Elbow , Wrist]
plt3=[T2 , T3]; % [Wrist , Claw]

%Local Space Frames (POV of different servos)
rep1=[1 0 ; 0 0 ; 0 1]'; %Shoulder
rep2=T1+R1*rep1(:,:);    %Elbow
rep3=T2+R2*R1*rep1(:,:); %Wrist


Dist1=sqrt( (plt1(1,2)-plt1(1,1))^2 + (plt1(2,2)-plt1(2,1))^2 );
Dist2=sqrt( (plt2(1,2)-plt2(1,1))^2 + (plt2(2,2)-plt2(2,1))^2 );
Dist=d3;

%plot
clf;
figure(1);

%Local Space Frames plot
plot(rep1(1,:),rep1(2,:),'red');hold on;%Shoulder
plot(rep2(1,:),rep2(2,:),'red');hold on;%Elbow
plot(rep3(1,:),rep3(2,:),'red');hold on;%Wrist

plot(plt1(1,:),plt1(2,:),'cyan','displayname',sprintf('UpperArm (Dist1=%.1f)',d1));
hold on; %UpperArm
plot(plt2(1,:),plt2(2,:),'green','displayname',sprintf('ForeArm (Dist2=%.1f)',d2));
hold on; %ForeArm
plot(plt3(1,:),plt3(2,:),'magenta','displayname',sprintf('Claw (Dist3=%.1f)',d3));
hold on; %Claw

% plot Circles, Effectors Range of Motion
t=linspace(-pi,pi,25);
C1=[d1*cos(t) ; d1*sin(t)];
C2=T1+[d2*cos(t) ; d2*sin(t)];
C3=T2+[d3*cos(t) ; d3*sin(t)];
plot(C1(1,:),C1(2,:),'yellow --','displayname',sprintf('ROM Elbow'));hold on;
plot(C2(1,:),C2(2,:),'yellow --','displayname',sprintf('ROM Wrist '));hold on;
plot(C3(1,:),C3(2,:),'yellow --','displayname',sprintf('ROM Claw'));hold on;
legend;

set(gca,'DataAspectRatio',[1,1,1])

