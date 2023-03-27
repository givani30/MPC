function [E,F,G,constraintSlope,constraintIntercept] = updateConstraints(eps,obstacle,detection,lanewidth,lanes)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    % Extract relevant states
    Y = eps(5); % The position of the car in the y direction
    X = eps(6); % The position of the car in the x direction

% if detection
%     slope=(obstacle.rlSafeY-Y)/(obstacle.rlSafeX-X);
%    if X<=obstacle.rlSafeX %In front of obstacle 
%        if Y>obstacle.rlSafeY 
%             cons_slope =0;
%             intercept=obstacle.rlSafeY;
%        else
%            cons_slope=tan(atan2(slope,1));
%            intercept=obstacle.rlSafeY - cons_slope*obstacle.rlSafeX;
%        end
%    elseif((X>obstacle.rlSafeX)&&(X<=obstacle.flX))
%        cons_slope=0;
%        intercept=obstacle.rlSafeY;
%    else
%        cons_slope=0;
%        intercept=-lanewidth*lanes/2;
%    end
%    [E,F,G]=baseConstraints(lanes,lanes);
%    %Update third row of F and G
%    F(3,:)=[0 0 -1 cons_slope];
%    G(3)=-intercept;
% else 
%     [E,F,G]=baseConstraints(lanewidth,lanes);
%     return
% end
%%
if detection
    if X<=obstacle.rlSafeX %voor obstakel
        Slope=(obstacle.rlSafeY-Y)/(obstacle.rlSafeX-X);
        constraintSlope=tan(atan2(Slope,1));
        constraintIntercept=obstacle.rlSafeY - constraintSlope*obstacle.rlSafeX;
    elseif (X>obstacle.rlSafeX)&&(X<=obstacle.flX) 
        constraintSlope=0;
        constraintIntercept=obstacle.rlSafeY;
    else
        constraintSlope =0;
        constraintIntercept=-lanewidth*lanes/2;
    end  
else
    constraintSlope =0;
    constraintIntercept=-lanewidth*lanes/2;
end
%return constraints
E1=[0 0 0];
    F1=[0 0 1 0];

    G1=lanewidth*lanes/2;
    % 
    E2=[0 0 0];
    F2=[0 0 -1 0];
    G2=lanewidth*lanes/2;
    %
    E3=[0 0 0];
    F3=[0 0 1 0];
    G3=-1*constraintIntercept;
    E=[E1;E2;E3];
    F=[F1;F2;F3];
    G=[G1;G2;G3];
    