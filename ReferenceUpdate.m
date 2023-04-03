function [refY,WeightY] = ReferenceUpdate(eps,obstacle,detection,lanewidth)
X = eps(1);
    if detection && X<=obstacle.flSafeX
        refY = 2*obstacle.rlSafeY;
        WeightY = 150;
    else
        refY=0;
        WeightY = 30;
    end
end