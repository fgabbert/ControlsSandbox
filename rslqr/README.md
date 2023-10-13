# Robust Servomechanism Linear Quadratic Regulator (RSLQR)

The Robust Servomechanism Linear-Quadratic Regulator (RSLQR) is a popular control methodology in the aerospace industry that solves the command-tracking problem by converting it into an error-regulation problem (servomechanism) and then using linear-quadratic regulation (lqr) to optimize the gains.

You can run the `rslqrScript.m` file in this directory to generate the same plot as below if you have matlab :)

# TODO
- Add better description and derivation ^
- Add screenshots of simulink model and equate it to the matlab state-space models
- Add linear analysis section to verify robustness
- Add the iterative design process with design charts:
  - Vary $q_{ii}$ from a to b
  - At each iteration: 
    - Compute gain matrix
    - Create closed-loop system
    - Simulate and extract time-domain performance metrics
    - Evaluate frequency-domain metrics

# Example: RSLQR Lateral-Directional Controller

## Bare-Airframe
The linear time-invariant lateral-directional dynamics of a generic transport aircraft in cruise are defined as follows:

$$\dot x_{ba} = A_{ba}x_{ba} + B_{ba}u_{ba}$$ 

$$y_{ba} = C_{ba}x_{ba} + D_{ba}u_{ba}$$  

$$ where: $$

$$ x_{ba} = \begin{bmatrix}
\phi, \beta, p, r \end{bmatrix}^T $$

$$ u_{ba} = \begin{bmatrix}
\delta_{a}, \delta_{r} \end{bmatrix} $$

$$
A_{ba} = 
\begin{bmatrix}
0      &   0        & 1.0000 &       0\\
0.0487 &  -0.0829   &    0   &   -1.0000\\
0      &  -4.5460   &-1.6990 &  0.1717\\
0      &  3.3820    &-0.0654 & -0.0893\\
\end{bmatrix}
$$

$$
B_{ba} = 
\begin{bmatrix}
0       & 0\\
0       & 0.0116\\
27.2760 &  0.5758\\
0.3952  & -1.3620\\
\end{bmatrix}
$$

$$
C_{ba} = I_{(n_{y_{ba}}, n_{x_{ba}})}
$$

$$
D_{ba} = 0_{(n_{y_{ba}}, n_{u_{ba}})}
$$

## Add Actuators
Now, let's add simple first-order actuators to the plant model, so that we can account for actuator dynamics in our gain optimization. We will assume the same actuator dynamics for both the aileron and rudder actuators.

$$ 
\dfrac{\delta}{\delta_{cmd}} =  \dfrac{20.2}{s+20.2} 
$$

Connect the actuator and airframe plant models in series, extending our system dynamics as follows:

$$
\dot x_{p} = A_{p}x_{p} + B_{p}u_{p}
$$ 

$$
y_{p} = C_{p}x_{p} + D_{p}u_{p}
$$  

$$ 
where: 
$$

$$ x_{p} = \begin{bmatrix}
\phi, \beta, p, r, \delta_{a}, \delta_{r} \end{bmatrix}^T $$

$$ u_{p} = \begin{bmatrix}
\delta_{a_{cmd}}, \delta_{r_{cmd}} \end{bmatrix} $$

$$
A_{p} = 
\begin{bmatrix}
         0 &       0  &  1.0000  &       0 &        0 &        0\\
    0.0487 & -0.0829  &       0  & -1.0000 &        0 &   0.0116\\
         0 & -4.5460  & -1.6990  &  0.1717 &  27.2760 &   0.5758\\
         0 &  3.3820  & -0.0654  & -0.0893 &   0.3952 &  -1.3620\\
         0 &       0  &       0  &       0 & -20.2000 &        0\\
         0 &       0  &       0  &       0 &        0 & -20.2000\\
\end{bmatrix}
$$

$$
B_{p} = 
\begin{bmatrix}
0     &   0\\
0     &   0\\
0     &   0\\
0     &   0\\
20.2  &   0\\
0     &   20.2\\
\end{bmatrix}
$$

$$
C_{p} = 
\begin{bmatrix}
I_{(n_{y_{ba}}, n_{x_{ba}})} & 0_{(n_{y_{p}}, n_{x_{p}})}
\end{bmatrix}
$$

$$
D_{p} = 0_{(n_{y_{p}}, n_{u_{p}})}
$$

## Add Tracking Error
We now further extend our system dynamics by adding tracking error terms for the outputs that we want to track command reference values.  

In our case, we are designing both a bank-angle ($\phi$) and sideslip ($\beta$) tracker.  
Since the number of outputs that we wish to control is equal to the number of control inputs we have available to us, the problem is well-posed.  

We will have a single integrator on each tracking error term, and this state is defined as:  

$$
e_{y_I} = \dfrac{(y - y_{ref})}{s}
$$

Which means that:  

$$
\dot{e_{y_I}} = (y - y_{ref})
$$

Adding an error term for both $\phi$ and $\beta$, our state vector is further extended to:

$$ 
x_{aug} = 
\begin{bmatrix}
e_{\phi}, e_{\beta},\phi, \beta, p, r, \delta_{a}, \delta_{r}\\
\end{bmatrix}^T 
$$

And:

$$
A_{aug} = 
\begin{bmatrix}
0 & 0  &  1.0000  &        0  &        0  &        0 &         0 &        0\\
0 & 0  &       0  &   1.0000  &        0  &        0 &         0 &        0\\
0 & 0  &       0  &        0  &   1.0000  &        0 &         0 &        0\\
0 & 0  &  0.0487  &  -0.0829  &        0  &  -1.0000 &         0 &   0.0116\\
0 & 0  &       0  &  -4.5460  &  -1.6990  &   0.1717 &   27.2760 &   0.5758\\
0 & 0  &       0  &   3.3820  &  -0.0654  &  -0.0893 &    0.3952 &  -1.3620\\
0 & 0  &       0  &        0  &        0  &        0 &  -20.2000 &        0\\
0 & 0  &       0  &        0  &        0  &        0 &         0 & -20.2000\\
\end{bmatrix}
$$

$$
B_{aug} = 
\begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & 0 \\
0 & 0 \\
0 & 0 \\
0 & 0 \\
20.2 & 0 \\
0 & 20.2 \\
\end{bmatrix}
$$

## Compute Optimal Gains
Using our augmented state-space model that includes our bare-airframe, actuators, and tracking error integrators, we can compute the optimal gain matrix $K$ using the standard LQR technique using the performance index:

$$ 
J = \int_0^\infty (x^TQx + u^TRu)d\tau
$$

This is done by solving the algebraic Riccati equation (ARE) of the form:

$$ 
PA + A^TP - PBR^{-1}B^TP + Q = 0
$$

For the sake of brevity, we will simply select our state weighting matrix (Q) and our control weighting matrix (R), and utilize Matlab's `lqr()` function to compute $K$.

For $Q$, we will only assign weights to our error states, and for a first-guess will equally weight $\phi$ and $\beta$ error regulation:

$$ 
Q = diag([10, 10, 0, 0, 0, 0, 0, 0]) 
$$

For $R$, we will just equally weight $\delta_{a}$ and $\delta_{r}$

$$ 
R = eye(2,2) 
$$

Now, the optimal gain matrix is computed using `lqr()`

$$ 
[K,S,E] = lqr(A_{aug}, B_{aug}, Q, R)
$$

Which returns the gain matrix:

$$
K = 
\begin{bmatrix}
    3.1619  &  0.0464  &   1.5961 &   -0.1543  &   0.3171  &   0.0274 &    0.3628  &   0.0062\\
   -0.0464  &   3.1619  &   0.0019 &    1.2418 &    0.0258 &   -1.3627  &   0.0062  &   0.0893
\end{bmatrix}
$$ 

It is worth noting that these gains seem reasonably small in value.  

It is also worth noting that this illustrates one of the downsides to LQR: Full state feedback is required, and each state has a gain for each control input. This can result in a large number of gain tables that need to be bookept to stabilize the aircraft throughout the flight envelope.

## Verify Results
We can now verify how well our now-stabilized system performs by creating the closed-loop sys:

$$
\begin{align*}
A_{cl} &= A_{aug} - B_{aug}K \\
B_{cl} &= [-1*\text{eye}(2,2), \text{zeros}(2,6)]' \\
C_{cl} &= \text{eye}(\text{size}(A_{cl})) \\
D_{cl} &= \text{zeros}(\text{size}(B_{cl}))
\end{align*}
$$


We can now simulate the step response for a $\phi_{cmd}$ and a $\beta_{cmd}$ and plot the results.  
![closedLoopStepResponse](https://github.com/fgabbert/ControlsSandbox/assets/13810793/d3c23b96-6ab2-4f6f-9dac-376d4c64f49c)

As seen above, for a 20 degree bank angle command:
- Acceptable rise time and settling time
- Zero steady-state error due to the tracking error integrator
- Less than 0.5 degrees of $\beta$ excursion during the roll
- Smooth and relatively small actuator motion
- $\beta$ oscillations are long-period and lightly-damped, could probably improve this.
