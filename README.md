# 3D Pendulum Simulation

Download Executable (.exe)
**Controls:** '1'/'2'/'3' pendulum types, 'r' reset, 'p' show plots, mouse to rotate camera.

---

## Lagrangian Formulation

For a 3D spherical pendulum with coordinates $(\phi, \theta)$:
**Position vector:** $\mathbf{r} = L(\sin\phi\sin\theta, -\cos\phi, \sin\phi\cos\theta)$
**Kinetic energy:** $T = \frac{1}{2}mL^2(\dot{\phi}^2 + \sin^2\phi \cdot \dot{\theta}^2)$
**Potential energy:** $V = -mgL\cos\phi$
**Lagrangian:** $\mathcal{L} = T - V = \frac{1}{2}mL^2(\dot{\phi}^2 + \sin^2\phi \cdot \dot{\theta}^2) + mgL\cos\phi$

---

## Equations of Motion

Applying Euler-Lagrange equations $\frac{d}{dt}\frac{\partial\mathcal{L}}{\partial\dot{q}} - \frac{\partial\mathcal{L}}{\partial q} = 0$:

$$\boxed{\ddot{\phi} = \sin\phi\cos\phi \cdot \dot{\theta}^2 - \frac{g}{L}\sin\phi}$$

$$\boxed{\ddot{\theta} = -2\cot\phi \cdot \dot{\phi}\dot{\theta}}$$

---

## Coupled Systems

For multiple pendulums, coupling terms are added:

$$\ddot{\phi}_i = \sin\phi_i\cos\phi_i \cdot \dot{\theta}_i^2 - \frac{g}{L_i}\sin\phi_i + \alpha\sin(\phi_i - \phi_j)$$

$$\ddot{\theta}_i = -2\cot\phi_i \cdot \dot{\phi}_i\dot{\theta}_i + \beta\sin(\theta_i - \theta_j)$$

where $\alpha, \beta$ are coupling coefficients.

---

## Runge-Kutta 4th Order Integration

For the system $\mathbf{y}' = \mathbf{f}(t, \mathbf{y})$:

$$\mathbf{y}_{n+1} = \mathbf{y}_n + \frac{h}{6}(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4)$$

where:

$\mathbf{k}_1 = h\mathbf{f}(t_n, \mathbf{y}_n)$
$\mathbf{k}_2 = h\mathbf{f}(t_n + h/2, \mathbf{y}_n + \mathbf{k}_1/2)$
$\mathbf{k}_3 = h\mathbf{f}(t_n + h/2, \mathbf{y}_n + \mathbf{k}_2/2)$
$\mathbf{k}_4 = h\mathbf{f}(t_n + h, \mathbf{y}_n + \mathbf{k}_3)$
**Local truncation error:** $O(h^5)$
**Global error:** $O(h^4)$

---

## Chaos Theory

### Sensitive Dependence on Initial Conditions

**Lyapunov exponent:** $\lambda = \lim_{t \to \infty} \frac{1}{t} \ln\frac{|\delta\mathbf{Z}(t)|}{|\delta\mathbf{Z}(0)|}$

For chaotic systems: $\lambda > 0$ (exponential divergence of nearby trajectories)


---

### Why Pendulums Are Chaotic

* **Nonlinear coupling:** $\sin(\phi_i - \phi_j)$ terms create complex interactions
* **High-dimensional phase space:** 8D for double, 12D for triple pendulum
* **Energy exchange:** Continuous transfer between kinetic and potential energy across multiple modes

---

### Phase Space Structure

**State vector for double pendulum:** $\mathbf{Z} = (\phi_1, \dot{\phi}_1, \theta_1, \dot{\theta}_1, \phi_2, \dot{\phi}_2, \theta_2, \dot{\theta}_2)$
**Evolution:** $\frac{d\mathbf{Z}}{dt} = \mathbf{F}(\mathbf{Z})$
**Strange attractors** emerge in this high-dimensional space with fractal geometry.

---

## Conservation Laws

**Energy:** $E = \frac{1}{2}\sum_i m_i(\dot{x}_i^2 + \dot{y}_i^2 + \dot{z}_i^2) + \sum_i m_i g y_i$
**Angular momentum (y-component):** $L_y = \sum_i m_i L_i^2 \sin^2\phi_i \cdot \dot{\theta}_i$

---

## Numerical Parameters

* **Step size:** $h = 0.002$ s
* **Coupling strength:** $\alpha = 0.015-0.02$
* **Damping:** $\gamma = 0.02-0.04$
