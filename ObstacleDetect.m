function detected = ObstacleDetect(eps,obstacle)
X = eps(1);
Y = eps(2);
absolute_distance = sqrt(abs(Y)^2 + abs(obstacle.X-X)^2); %obsolute distance to the obstacle
distance_X = obstacle.X-X; %distance to the obstacle from center of car in X direction
distance_Y = abs(obstacle.Y - Y); %distance to the obstacle from center of car in Y direction
detected = ((absolute_distance <= obstacle.DetectionDistance)) ;
end
%Detection zone car -> in front of car 
%Express car location in car coordinates/body fixed frame
%compare location obstacle to detection zone location
%if statement true or false