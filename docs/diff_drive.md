# Motion of the Differential drive

Linear velocity of reference point is the average of wheels velocities:

`linear_velocity = (left_velocity + right_velocity) / 2`

Angular velocity refers to how fast a point object revolves about a fixed origin, i.e. the time rate of change of its angular position relative to the origin
Let's assume that rotation is positive CCW, viewed from above. 
If, hypothetically, the robot is rotating CCW, then it would mean that the right wheel is moving some distance further than the left wheel. 
The difference in distances traveled becomes the arc length scribed by the right wheel (again, relative to the left), and the wheel base, or distance between the wheels, becomes the hypotenuse or radius for the rotation.

Continuing with the assumption, positive is CCW, we would wind up with an equation like:

`angular_rotation = (right_distance - left_distance) / WHEEL_BASE`

Also, `angular_velocity` is the rate of change of angular position with respect to time, so we can divide through by time to get:

`angular_velocity = (right_velocity - left_velocity) / WHEEL_BASE`

We can solve for the `right_velocity` in the `angular_velocity` equation:

`right_velocity = angular_velocity * WHEEL_BASE + left_velocity`

And then plug that into the linear velocity equation:

`linear_velocity = (left_velocity + angular_velocity * WHEEL_BASE + left_velocity) / 2`

Then we can solve for the `left_velocity`:

~~~
linear_velocity = left_velocity / 2 + left_velocity / 2 + (angular_velocity * WHEEL_BASE) / 2;

linear_velocity = left_velocity + (angular_velocity * WHEEL_BASE) / 2;
~~~

and we got: 

`left_velocity  = linear_velocity - (angular_velocity * WHEEL_BASE) / 2`

Then we can plug this back into the expression for the `right_velocity`:

~~~
right_velocity = angular_velocity * WHEEL_BASE + left_velocity

right_velocity = angular_velocity * WHEEL_BASE + linear_velocity - (angular_velocity * WHEEL_BASE) / 2

right_velocity = angular_velocity * WHEEL_BASE - (angular_velocity * WHEEL_BASE) / 2 + linear_velocity
~~~

and we got:

`right_velocity = (angular_velocity * WHEEL_BASE) / 2 + linear_velocity;`

This then gives the equations for `left_` and `right_velocities` in terms of linear velocity and angular velosity.

Now, recall the equations for the conversion from `_rpm` to `_velocity`:

~~~
left_velocity = left_rpm * (RPM_TO_RAD_PER_SECONDS * DIST_PER_RAD)

right_velocity = right_rpm * (RPM_TO_RAD_PER_SECONDS * DIST_PER_RAD)
~~~

Where:

`RPM_TO_RAD_PER_SECONDS = 2 * Pi / 60 = 0.1047`

The `DIST_PER_RAD` is just the circumference, `2 * Pi * WHEEL_RADIUS`, divided by the number of radians in a complete revolution, which is just `2 * Pi`. 
This means that `DIST_PER_RAD` is the same as the `WHEEL_RADIUS`. Convert `RPM` to `rad/s`, and then use that value with the wheel radius to get speed.

Important to note - `left_rpm` and `right_rpm` actually need to be in RPM, and `DIST_PER_RAD` and/or `WHEEL_RADIUS` needs to be in meters in order to get the output velocities in meters per second. 

And you can see that:

`left_rpm  = left_velocity / (RPM_TO_RAD_PER_S * DIST_PER_RAD)`
`right_rpm = right_velocity / (RPM_TO_RAD_PER_S * DIST_PER_RAD)`

Which, substituting in the `_velocity` terms, gives:

`left_rpm  = (linear_velocity - (angular_velocity * WHEEL_BASE) / 2) / (RPM_TO_RAD_PER_S * DIST_PER_RAD)`

`right_rpm = (linear_velocity + (angular_velocity * WHEEL_BASE) / 2) /(RPM_TO_RAD_PER_S * DIST_PER_RAD)`

# Sources
1. https://robotics.stackexchange.com/questions/18048/inverse-kinematics-for-differential-robot-knowing-linear-and-angular-velocities
