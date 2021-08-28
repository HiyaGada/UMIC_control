m=1;
dt=0.0001;
n=5;

%Altitude Control
A0 = [0 1 ; 0 0];
B0 = [0 ; 1];
[K0, S0, e0] = lqr(A0, B0, eye(2), 1);

for i=1:n
zd = [5; 0];
zvec = zeros(2, n);
zvec(1,1)= 5;
zvec(2,1)= 0;
uz = zeros(1,n);
uz(1,i) = -K0*(zvec(:,i) - zd);

%Find zvec at next time instant
zvec(1,i+1)= zvec(1, i) + dt*(zvec(2, i) + dt*(uz(1, i)));
end 

% U1 = m*(g-uz);
% 
% %Attitude Control
% Ix= 2;
% Iy= 2;
% Iz= 2;
% A1 = [0 1 0 0 0 0; 0 0 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0; 0 0 0 0 0 1; 0 0 0 0 0 0];
% B1 = [0 0 0; 1/Ix 0 0; 0 0 0; 0 1/Iy 0; 0 0 0; 0 0 Iz];
% 
% [K1, S1, e1] =  lqr(A1, B1, eye(6), 0.5);
% disp(K1);

%[U2; U3; U4] = -K1*angle; 