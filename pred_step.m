function [covarEst,uEst] =pred_step(uPrev,covarPrev,angVel,acc,dt)

% preallocation of matrix sizes to speed code execution (no dynamic sizing)
state_deriv=zeros(15,1);
uEst=zeros(15,1);
covarEst=zeros(15,1);
A=zeros(15,15);
F=zeros(15,15);
U=zeros(15,15);

% creating random matrix of values between 0 and 1 for Q which will be
% constant for different iterations.
rng(1);
process_variances=diag(rand(15));
Q=dt*diag(process_variances);


persistent counter
persistent rotMatrix
persistent gInvMatrix
persistent j
persistent g
persistent h
persistent n
persistent l
persistent t
% the following function runs only once to initialize the symbolic
% expressions
if isempty(counter)
    counter=0;
    syms yaw pitch roll wx wy wz ax ay az;
    w=[wx;wy;wz]; % ang velocity
    a=[ax;ay;az]; % acceleration

    syms px p_y pz  vx vy vz bax bay baz bgx bgy bgz;
    syms nvx nvy nvz ngx ngy ngz nax nay naz nbax nbay nbaz nbgx nbgy nbgz;
    bg=[bgx;bgy;bgz]; % gyroscope bias
    ba=[bax;bay;baz]; % accelerometer bias
    na=[nax;nay;naz]; % acc noise
    ng=[ngx;ngy;ngz]; % gyroscope noise

    rotMatrix=[cos(yaw)*cos(pitch) cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll) cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
        sin(yaw)*cos(pitch) sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll) sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
       -sin(pitch) cos(pitch)*sin(roll) cos(pitch)*cos(roll)];
  
    gMatrix=[1 0 -sin(pitch);0 cos(roll) sin(roll)*cos(pitch);0 -sin(roll) cos(roll)*cos(pitch)];
    gInvMatrix=inv(gMatrix);
    v=[px;p_y;pz;roll;pitch;yaw;vx;vy;vz;bgx;bgy;bgz;bax;bay;baz];
    xdot=[vx-nvx;vy-nvy;vz-nvz;gInvMatrix*(w-bg-ng);rotMatrix*(a-ba-na);
        nbgx;nbgy;nbgz;nbax;nbay;nbaz];
    noise=[nvx;nvy;nvz;ngx;ngy;ngz;nax;nay;naz;nbgx;nbgy;nbgz;nbax;nbay;nbaz];
    j=jacobian(xdot,v); % A matrix in symbolic terms
    n=jacobian(xdot,noise); % U matrix in symbolic terms
end

counter=counter+1
% getting values from previous state
px=uPrev(1,1);
p_y=uPrev(2,1);
pz=uPrev(3,1);
roll=uPrev(4,1);
pitch=uPrev(5,1);
yaw=uPrev(6,1);
bax=uPrev(10,1);
bay=uPrev(11,1);
baz=uPrev(12,1);
bgx=uPrev(13,1);
bgy=uPrev(14,1);
bgz=uPrev(15,1);
bg=[bgx;bgy;bgz];
ba=[bax;bay;baz];
vx=uPrev(4,1);
vy=uPrev(5,1);
vz=uPrev(6,1);
nvx=0;
nvy=0;
nvz=0;
ngx=0;
ngy=0;
ngz=0;
nax=0;
nay=0;
naz=0;
nbgx=0;
nbgy=0;
nbgz=0;
nbax=0;
nbay=0;
nbaz=0;

grav=[0;0;-9.81];

wx=angVel(1,:);
wy=angVel(2,:);
wz=angVel(3,:);
ax=acc(1,:);
ay=acc(2,:);
az=acc(3,:);


% the following condition initializes the matlabfunction hurdle only once
if counter==1
    g=matlabFunction(j,'Sparse',true,'Optimize',false);
    h=matlabFunction(n,'Sparse',true,'Optimize',false);
    l=matlabFunction(rotMatrix);
    t=matlabFunction(gInvMatrix);
end

A=g(ax,ay,az,bax,bay,baz,bgy,bgz,nax,nay,naz,ngy,ngz,pitch,roll,wy,wz,yaw);
U=h(pitch,roll,yaw);
R=l(pitch,roll,yaw);
G_Inv=t(pitch,roll);


% process of calculating uestimated.
% using vectorization for code optimizatio 
i=1:3;
state_deriv(i,1)=uPrev(i+6,1);

m=4:6;
state_deriv(m,1)=G_Inv(m-3,:)*(angVel(:,1)-uPrev(10:12,1));

k=7:9;
state_deriv(k,1)=grav(k-6,:)+R(k-6,:)*(acc(:,1)-uPrev(13:15,1));


uEst=uPrev+state_deriv*dt;

% calclulate components for covariance estimation
F=eye(15,15)+dt*A;
% calculate covariance estimated
covarEst=F*covarPrev*transpose(F)+U*Q*transpose(U);

end

