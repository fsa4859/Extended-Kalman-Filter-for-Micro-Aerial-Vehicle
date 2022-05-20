clear; % Clear variables
datasetNum = 9; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime] = init(datasetNum);
Z = sampledVicon(7:9,:);%all the measurements that you need for the update
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %J ust for saving state his.
prevTime=0; %last time step in real time
%write your code state_deriv(k,1)=grav(k-6,1)+R(k-6,:)*(acc(:,1)-uPrev(k+3,1));here calling the pred_step.m and upd_step.m functions
for i = 1:length(sampledTime)
    angVel=sampledData(i).omg;
    acc=sampledData(i).acc;
    dt=sampledTime(:,i)-prevTime;
    % measurement at time t (position and orientation)
    z_t=Z(:,i);
    [covarEst, uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt);
    [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst);
    savedStates(:,i)=uCurr;
    uPrev=uCurr;
    covarPrev=covar_curr;
    prevTime=sampledTime(:,i);
end

plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);