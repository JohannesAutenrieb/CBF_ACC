# CBF_ACC

### Discription
This collection of MATLAB scripts intends to study the performance of state-constrained controllers utilizing control barrier functions in the context of adaptive cruise control.


## Problem Formulation
We consider a case in which two vehicles, modeled as point masses, are moving in a straight line. The following vehicle is equipped with an ACC and the lead vehicle drives with constant speed $v_0$.

* **Control Objective**: Cruising at a given speed $v_d$ for the following vehicle.
* **Safety Objective**: Ensure that the distance $D$ is not violating the following safety constraint:

$$\\begin{equation}
    D \geq T_h v
\\end{equation}$$

with $T_h$ being the look-ahead time.

<p align=center>
<img src="https://github.com/JohannesAutenrieb/CBF_ACC/blob/main/Images/snipping_ACC_Case.png" alt="snipping_ACC_Case" height=300px>
</p>

The dynamics of the system can be defined as follows:

$$\\begin{equation}
    \begin{bmatrix}
    \dot{x} \\
    \dot{v}\\
    \dot{D} \\
    \end{bmatrix} = 
    \begin{bmatrix}
    v \\
    - \frac{1}{m} F_r(v)\\
    v_0 - v \\
    \end{bmatrix}     +
    \begin{bmatrix}
    0\\
    \frac{1}{m}\\
    0 \\
    \end{bmatrix} F_w
\\end{equation}$$

with $x$ being the position, $m$ the mass and $v$ the speed of the controlled vehicle. The control input $u$ of the ACC is defined as the wheel force $F_w$, while the aerodyanmic drag is given as $F_r$, which is defined as:

$$\\begin{equation}
    F_r(v) = f_0 + f_1 v + f_2 v^2
\\end{equation}$$


## Concept of Control Lyapunov Functions

We consider a control affine plant of the form:

$$\\begin{equation}
\dot{x}(t) = f(x(t)) + g(x) (u(t))
\\end{equation}$$

where $x(t) \in \mathbf{R}^{n}$ is a measurable state vector and $u(t) \in \mathbf{R}^{m}$ is a control input vector. The control input is assumed to be magnitude limited by $\vert u_0 \vert$, which leads to the following closed set for the control input space

$$\\begin{equation}
\mathcal{U} = \begin{Bmatrix} u \in \mathbf{R}^{m} : - u_0 \geq u(t) \geq u_0 \end{Bmatrix}
\\end{equation}$$

For which we define the control objective of globally stabilizing the considered system to a point $x^∗ = 0$, and hence imposing $x(t) \rightarrow 0$.  This can be achieved by finding a feedback control law $k(x)$ that drives a positive-definite continuous function $V : D \in \mathbf{R}^{m} \rightarrow R \geq 0$. 

<p align=center>
<img src="https://github.com/JohannesAutenrieb/CBF_ACC/blob/main/Images/CLF_Visual.png" alt="CBF_Function_Plot" height=300px>
</p>

The system is globally stabilizable if there exists the class $\mathcal{K}_{\infty}$ functions $\alpha_1$, $\alpha_2$,  $\alpha_3$ such that

$$\\begin{equation}
    \alpha_1 (\lVert x \rVert) \leq V(x) \leq \alpha_2 (\lVert x \rVert)
\\end{equation}$$


$$\\begin{equation}
    \inf_{u \in U} \dot{V}(x, u) \leq - \alpha_3 (\lVert x \rVert)
\\end{equation}$$

where $\alpha_3(h(x)$ is often chosen to be

$$\\begin{equation}
    \alpha_3(\lVert x \rVert) = \lambda V(x)
\\end{equation}$$

with $\lambda$ being $\lambda > 0$. Since any controller that respects the above requirements can ensure stability, there is no need to explicitly construct the feedback controller 

$$\\begin{equation}
k(x) = \begin{Bmatrix} u \in \mathbf{R}^{m} : \dot{V}(x, u) = [L_f V (x) + L_g V (x)u] ≤ −\lambda V (x) \end{Bmatrix}
\\end{equation}$$

One can define the following positive definite control Lyapunov function (CLF) $V(x,u)$:

$$\\begin{equation}
    \inf_{u \in U} L_f V (x) + L_g V (x)u  \leq - \alpha_3 (\lVert x \rVert)
\\end{equation}$$  

## Concept of Control Barrier Functions

We consider again a control affine plant of the form:

$$\\begin{equation}
\dot{x}(t) = f(x(t)) + g(x) (u(t))
\\end{equation}$$

where $x(t) \in \mathbf{R}^{n}$ is a measurable state vector and $u(t) \in \mathbf{R}^{m}$ is a control input vector. The control input is assumed to be magnitude limited by $\vert u_0 \vert$, which leads to the following closed set for the control input space

$$\\begin{equation}
\mathcal{U} = \begin{Bmatrix} u \in \mathbf{R}^{m} : - u_0 \geq u(t) \geq u_0 \end{Bmatrix}
\\end{equation}$$

 A closed set $\mathcal{C} \in \mathbf{R}^n$, which we consider as a safe set, is defined in the following form:
  
  $$\\begin{equation}
      \mathcal{C} = \begin{Bmatrix} x \in \mathbf{R}^n : h(x) \geq 0 \end{Bmatrix}
  \\end{equation}$$
  
with $h: \mathbf{R}^n \times \mathbf{R}^p \to \mathbf{R}$ being a  continuously differentiable function, called  control barrier function (CBF).
  
<p align=center>
<img src="https://github.com/JohannesAutenrieb/CBF_ACC/blob/main/Images/CBF_Visual.png" alt="CBF_Function_Plot" height=300px>
</p>
  
The CBF can ensure for the presented control affine system that for any initial condition $x_0 := x(t_0) \in \mathcal{C}$, that $x(t)$ stays within $\mathcal{C}$ for any $t$, if there exists an extended class $\mathcal{K}$ functions $\alpha$ such that for all $x \in Int(\mathcal{C})$

$$\\begin{equation}
    \sup_{u \in U} [L_f B(x) + L_g B(x) u  + \alpha(h(x))] \geq 0
\\end{equation}$$

where $\alpha(h(x)$ often chosen to be

$$\\begin{equation}
    \alpha(h(x)) = \gamma h(x)
\\end{equation}$$

with $\gamma$ being $\gamma > 0$.

## Pointwise  CLF-CBF-QP Controller

By using a QP-based approach, it is possible to unify both CLF-based "performance objectives" and CBF-based "safety considerations". Using a quadratic programming formulation, the solution for $u$ can be found by solving:

$$\\begin{align}
&\min_{u \in \mathcal{U}}
\begin{aligned}[t]
  &\frac{1}{2} u^T H u + F u
\end{aligned} \\
&\text{s.t.} \notag \\
& L_f V(x) + L_g V(x)u + c_3 V(x) - \delta \leq 0, \notag\\
& L_f h(x) + L_g h(x)u - \alpha (h(x)) \leq 0, \notag
\\end{align}$$

with $\delta$ being a slack or relaxation variable that ensures a feasible QP  by relaxing the condition on stability to guarantee safety.

### Simulation Study

After each simulation run, a plot with results is given out. An example of such a plot is given here:

<p align=center>
<img src="https://github.com/JohannesAutenrieb/CBF_ACC/blob/main/Images/Example_results_acc.png" alt="Example_results_acc" height=500px>
</p>

## Dependencies


The scripts use external libraries, which need to be installed.
* [Export_fig](https://github.com/altmany/export_fig)


Further, the following MATLAB toolboxes are needed:
* [Optimization Toolbox](https://www.mathworks.com/products/optimization.html)

**The software was tested with MATLAB 2020b under Windows 11 Home.** 

## License

The contents of this repository are covered under the [MIT License](LICENSE).


## References

We kindly acknowledge the following papers, which have been the foundation of the here presented scripts:

* [[1] J. Zeng, B. Zhang, and K. Sreenath, “Safety-Critical Model Predictive Control with Discrete-Time Control
Barrier Function,” in 2021 American Control Conference (ACC), May 2021, pp. 3882–3889.](https://ieeexplore.ieee.org/document/9483029)
* [[2] A. D. Ames, X. Xu, J. W. Grizzle and P. Tabuada, "Control Barrier Function Based Quadratic Programs for Safety Critical Systems," in IEEE Transactions on Automatic Control, vol. 62, no. 8, pp. 3861-3876.](https://ieeexplore.ieee.org/document/7782377)
