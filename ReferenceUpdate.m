function [refY] = ReferenceUpdate(eps,obstacle,detection)
X = eps(1);
Y = eps(2);
if detection
    if X<=obstacle.flSafeX
        refY = obstacle.rlSafeY+0.1*obstacle.rlSafeY;
    else 
        refY = 0 ;      
end