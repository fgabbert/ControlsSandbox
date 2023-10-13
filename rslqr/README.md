# Add Robust Servomechanism Linear Quadratic Regulator (RSLQR)

{Insert brief overview on what RSLQR is and why it's useful}

## Example: RSLQR-Optimized Lateral-Directional Controller

The linear time-invariant lateral-directional dynamics of a generic transport aircraft in cruise are defined as follows:

$$\dot x = Ax + Bu$$ 

$$y = Cx + Du$$  

$$ where: $$

$$ x = \begin{bmatrix}
\phi, \beta, p, r \end{bmatrix}^T $$

$$ u = \begin{bmatrix}
\delta_{a}, \delta_{r} \end{bmatrix} $$


Now, let's add simple first-order actuators to the plant model, so that we can account for actuator dynamics in our gain optimization. We will assume the same actuator dynamics for both the aileron and rudder actuators.

$$ \dfrac{\delta}{\delta_{cmd}} =  \dfrac{20.2}{s+20.2} $$

Now, we connect the actuator and airframe plant models in series, extending our system dynamics as follows:

$$\dot x = Ax + Bu$$ 

$$y = Cx + Du$$  

$$ where: $$

$$ x = \begin{bmatrix}
\phi, \beta, p, r, \delta_{a}, \delta_{r} \end{bmatrix}^T $$

$$ u = \begin{bmatrix}
\delta_{a_{cmd}}, \delta_{r_{cmd}} \end{bmatrix} $$