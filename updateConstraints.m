function [E,F,G] = updateConstraints(eps,obstacle,detection,lanewidth,lanes)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    % Extract relevant states
    Y = eps(5); % The position of the car in the y direction
    X = eps(6); % The position of the car in the x direction

if not(detection)
    [E,F,G]=baseConstraints(lanewidth,lanes);
    return
else
   if X<obstacle.rlSafeX %In front of obstacle 
        slope=(obstacle.rlSafeY-Y)/(obstacle.rlSafeX-X);
        intercept=Y;
   else
       slope=(Y-obstacle.flSafeY)/(obstacle.flSafeX-X);
       intercept=Y;
   end
   [E,F,G]=baseConstraints(lanes,lanes);
   %Update third row of F and G
   F(3,:)=[0 0 0 0 1 -slope];
   G(3)=intercept;
end