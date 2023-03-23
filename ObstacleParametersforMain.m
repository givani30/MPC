%alles wat in de main moet over de obstacle struct
obstacle = struct;
obstacte.Length = 4;
obstacle.Width = 2;
obstacle.X = 50; %distance to obstacle 
obstacle.Y = 0; %obstacle in middle of road
obstacle.DetectionDistance = 25; %distance from which the obstacle can be detected
obstacle.safeDistanceX = obstacle.Lenght; %safedistance around obstacle in X direction
obstacle.safeDistanceY = lanewidth; %safedistance around obstacle in Y direction
