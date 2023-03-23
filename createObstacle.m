function obstacle = createObstacle(lanewidth)
    %Creates an obstacle
    obstacle = struct;
    obstacle.Length = 4;
    obstacle.Width = 2;
    obstacle.X = 50; %distance to obstacle 
    obstacle.Y = 0; %obstacle in middle of road
    obstacle.DetectionDistance = 25; %distance from which the obstacle can be detected
    obstacle.safeDistanceX = obstacle.Length; %safedistance around obstacle in X direction
    obstacle.safeDistanceY = lanewidth; %safedistance around obstacle in Y direction
    obstacle=ObstacleGeometry(obstacle);
end
