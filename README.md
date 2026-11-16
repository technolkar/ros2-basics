Exp	Setting	Result	Lesson
1	Kp = 2	smooth exponential, τ ≈ 0.5 s	P on a simple system = first-order response
2	+ disturbance 0.5	steady error 0.25 rad (predicted 0.25)	P alone leaves a steady error = d / Kp
3	+ Ki = 1	error slowly → 0, small overshoot	I removes steady error, and resetting it slows that down
4	Kp = 15	saturated ramps, jitter at target	output limits and sampling delay limit the gain
5	+ Kd = 0.3	no real improvement	D needs inertia, and turtlesim has none