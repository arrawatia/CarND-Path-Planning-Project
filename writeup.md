# Writeup

The goal is to safely navigate around a virtual highway with other traffic. The traffic is moving close to the speed limit. 

The car has to drive close to the speed limit and pass slower traffic whenever possible. It should not hit other cars and drive inside marked road lanes. 

The car should make one complete loop around the ~7km highway. The total acceleration should stay under 10 m/s^2 and jerk should be below 10 m/s^3.


# Rubric

## Compilation

#### CRITERIA : The code compiles correctly.
 
The code compiles without any erros using `cmake` and `make` on my macbook running Mac OS 10.14.5 

 
## Valid Trajectories

The car is able to drive around 5 miles without triggering any error conditions. The video below shows one sample run.

![out.gif](out.gif)
#### CRITERIA : The car is able to drive at least 4.32 miles without incident
The video recording above shows the car drving successfully for 5 miles without any incident.

#### CRITERIA : The car drives according to the speed limit
The video shows that car tries to stay close to the speed limit whenever possible. It changes speed to react to the traffic but always speed up close to 50mph.

#### CRITERIA : Max Acceleration and Jerk are not exceeded
The video shows that the car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

#### CRITERIA : Car does not have collisions
The car does not collide with any other car on the road.

#### CRITERIA : The car stays in its lane, except for the time between changing lanes
The video shows that car stays in its lane. It only changes lane to pass slower cars. 


#### CRITERIA : The car is able to change lanes
The video shows the car changing lanes smoothly and safely.
 
# Reflection
 
The project solution has three parts. It first predicts the state of the world, decides what action to take based on the prediction and then generates sensible trajectories to execute the decided upon action.

We predict where cars will be in the future using data from sensor fusion. The sensor fusion data has telemetry for all other cars in the vicinity. The idea is to go through each car and figure out if they are going in the same direction and if so, find the lane they will be in the future. 

We figure out if there is a car in the same lane ahead of us and if it is too close (less than 30m away). We also figure out if there is a lane to the left or right that is safe to go into. Safe is defined as "no cars are predicted to be in that lane 30m ahead and 30m behind our car."

Prediction step implementation: [L256-L323](https://github.com/arrawatia/CarND-Path-Planning-Project/blob/master/src/main.cpp#L256-L323)

Once the future state is predicted, we decide on the behaviour of the car based on the following algorithm.

- If the car is too close to the car ahead of it, 
     - change lane to the left if already not in left most lane and it is safe to do so
     - if car cannot go into left lane, check if can go right and change to the right lane if it is safe to do so
     - if car cannot change lanes, slow down to avoid colliding with the car ahead

- If the car is not too close to the car ahead in its lane
     - try to speed up to stay close to the speed limit
     - try to switch to center lane if it is safe to do so.

Behavior planning implementation : [L324-L358](https://github.com/arrawatia/CarND-Path-Planning-Project/blob/master/src/main.cpp#L324-L358)


We then generate smooth, drivable, safe and comfortable trajectories to execute the desired behavior. I followed the method described in the project walkthrough. This method uses a spline library to generate smooth trajectories. The method also takes cares of boundary conditions and tries to minimize sudden changes to the trajectory. We also ensure that the vehicle accelerates slowly from a dead stop in the beginning.

The car drives around the track successfully and I stopped iterating on the code when the car was able to pass all criteria specified in the rubric. But I can see that it velocity changes are abrupt. We could use a more sophisticated controller to make these changes smoother.

Trajectory generation implementation : [L360-L512]( https://github.com/arrawatia/CarND-Path-Planning-Project/blob/master/src/main.cpp#L360-L512)

   
      
  


    

