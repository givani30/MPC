function obstacle = createObstacle()
    %Creates an obstacle
    obstacle = struct;
    obstacle.Length = 30;
    obstacle.Width = 2;
    obstacle.X = 50; %distance to obstacle 
    obstacle.Y = 0; %obstacle in middle of road
    obstacle.DetectionDistance = 100; %distance from which the obstacle can be detected
    obstacle.safeDistanceX = obstacle.Length/2; %safedistance around obstacle in X direction
    obstacle.safeDistanceY = obstacle.Width/2; %safedistance around obstacle in Y direction
    obstacle=ObstacleGeometry(obstacle);
end
