For loop
	If Obstacledetected(state,obstacle)
		Calculate new  third constraint
	
	else
		Disable third constraint
	Update constraints
	Update MPC
	Caclulate control input
	Update state
	
Function True/False Obstacledetected(state, obstacle)
	%Detection range
	%
Function calculateObstacle
Function [E,F,G] UpdateConstraints(state,obstacle)