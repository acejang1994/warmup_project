Warm Up Project 
Claire Keum, James Jang, Filippos Lymperopoulos

1. Which behaviors did you implement?
	Wall Following, Person Following, Obstacle Avoiding and drawing a contour around an object.

2.For each behavior, what strategy did you use to implement the behavior? 
	a. Wall Following
We had a function call that will calculate the angular velocity, so that when the robot’s laser scan data points (in our case, we picked 45 degrees and 135 degrees angle data) do not match, the function will use a proportional control for returning an angular velocity to control the robot.

	b. Person Following
		We wrote a function that detected the “object” - “person”. We scanned the area in front of the Neato from -45 to 45 degrees and registered the location of objects that appear in the proximity of 1 meter from the laser scanner. We translated the coordinates of this detected object into Cartesian coordinates and aggregated all signals into a “centroid” point that represented the scanned object found. We further calculated the distance from this point as well as the angle to that point and implemented proportional control to approach it in our linear and angular rates of displacement.

	c. Obstacle Avoiding
		For obstacle avoiding, the robot first went straight until it found an obstacle, which was represented by a “centroid” as mentioned on person following. When the robot found an obstacle, the linear velocity was set to zero and it started turning in z axis proportional to the angle of the obstacle. Our function found_obstacle, constantly checks if there is an obstacle in front of the robot. When the robot is 45 degrees off of the obstacle then it determines that it avoided the obstacle and goes straight.
		
	d. New behavior
          One of the videos that we watched in our first day of the class, a robot was making a contour of an object. Instead of using a bump sensor data (since using it in this case might be too simple) we wanted to incorporate the behavior that we have implemented already. As for a robot to draw a contour of an object, it should get close to the object (follow a person behavior) and then make a contour of the person. It was done by using the code for the person following and obstacle avoid, but added a line of code where it says ‘when the robot loses a signal, try to turn left to get the signal back.’ 

3.For the finite state controller, what were the states?  What did the robot do in each state? 
	We didn’t formally implement a finite state controller with its suggested code architecture. However, we combined two states into a new behavior in our system. One is person following behavior and the other one used the idea of ‘obstacle avoidance,’ which finally gave us a new behavior, drawing a contour of a person.

4.How did you combine and how did you detect when to transition between behaviors?
	We detected transitions between behaviors via certain boolean parameters that declared if an obstacle/person was found and when that value was True, we pursued with the second state.

5.How did you structure your code?
	The code was structured as follows:
General class  with __init__ method where variables and topics are published
callback() function that repeatedly collected values and invoked other helper functions
helper() functions that performed the task in question
run() method that published stuff when ros was not shutdown

6.What if any challenges did you face along the way?
	Some of the challenges that we faced was still getting used the using rospy to control the robot. One of the big problems that we had was when we the laser scan did not pick up any signal for an obstacle. When the robot did not detect any obstacles in its proximity, since we were first using class global variable, we were doing computations with the “old” obstacles, which threw us off many times. We switched to local variables to reset the obstacle location every time the robot scanned the room.

7.What would you do to improve your project if you had more time?
	Effective use of Rviz would be definitely something we could put more time on, and make the behavior more robust, by using more data points or refining some constants that we chose to control the robot. 

8.Did you learn any interesting lessons for future robotic programming projects?
	It was great to see how we could create a new topic i.e centroid, and publish it that can be visualized in Rviz. Collecting data from a robot and giving a feedback right away to the robot was very interesting and thought we could use such a feedback control loop system in various environments. 
