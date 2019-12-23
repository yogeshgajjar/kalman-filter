# kalman_filter

Kalman filter for tracking the position of an object of unknown dynamics. 

## Concept 

Kalman filter is a Bayes filter that estimates the current state of the system based on the previous state, current motion command and current observation. The filter consists of two cycles called prediction cycle and correction or update cycle. The predict cycle estimates the new belief of the state given previous state and control command. The update cycle utilizes future belief calculated in the prediction state and utilizes the new sensor commands to correct or update the belief. This is an iterative and recursive process. 

Assumptions - 
- Distributions are gaussian 
- The system is linear i.e the motion model and the observation models are linear where the motion model is the probability of the current state given the previous state and motion command at time t and the observation model is the probability of an observation given the current state. 

## Problem 

To track the position of an object of unknown dynamics. We used a constant jerk model that assumes that the acceleration is linear. Also, the discrete time-step of delta t = 0.1, the dynamics are as follows 

Motion Model : 
`x(t+1) = Ax(t)`
`A = [1 0.1 0 0; 0 1 0.1 0; 0 0 1 0.1; 0 0 0 1]`

Observation Model : 
`z(t) = Cx(t) + v(t)`
`C = [1 0 0 0]` 

where `v(t)` is a zero-mean Gaussian Sensor Noise with variance `Q = 1.0` 

The initial belief is as follows 

`mu = [5 1 0 0 ]
`sigma = [10 0 0 0; 0 10 0 0; 0 0 10 0; 0 0 0 10]` 

The true position of the object changes as follows - 
`p(t) = sin(0.1 * t) with p(0) = 0` 

and timesteps T = 100 timesteps 
 <!-- `$z = x + y$`.

`$$a^2 + b^2 = c^2$$`

`$$\begin{vmatrix}a & b\\
c & d
\end{vmatrix}=ad-bc$$`

A = 
![filename](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%201%20%26%200.1%20%26%200%20%26%200%5C%5C%200%20%26%201%20%26%200.1%20%26%200%5C%5C%200%20%26%200%20%26%201%20%26%200.1%5C%5C%200%20%26%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D)
 -->
