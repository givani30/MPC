function obstacle = ObstacleGeometry(obstacle)
%Obstacle:
% Front left
obstacle.flX = obstacle.X+obstacle.Length/2;
obstacle.flY = obstacle.Y+obstacle.Width/2;
% Front right
obstacle.frX = obstacle.X+obstacle.Length/2;
obstacle.frY = obstacle.Y-obstacle.Width/2;
% Rear left
obstacle.rlX = obstacle.X-obstacle.Length/2;
obstacle.rlY = obstacle.flY;
% Rear right
obstacle.rrX = obstacle.X-obstacle.Length/2;
obstacle.rrY = obstacle.frY;

%Safe zone:
% Front left
obstacle.flSafeX = obstacle.flX+obstacle.safeDistanceX; 
obstacle.flSafeY = obstacle.flY+obstacle.safeDistanceY;
% Front right
obstacle.frSafeX = obstacle.frX+obstacle.safeDistanceX;
obstacle.frSafeY = obstacle.frY-obstacle.safeDistanceY;
% Rear le
obstacle.rlSafeX = obstacle.rlX-obstacle.safeDistanceX; 
obstacle.rlSafeY = obstacle.flSafeY;
% Rear right
obstacle.rrSafeX = obstacle.X-obstacle.safeDistanceX;
obstacle.rrSafeY = obstacle.frSafeY;

end