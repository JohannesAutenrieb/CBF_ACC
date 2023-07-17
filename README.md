# CBF_ACC

### Discription
This collection of MATLAB scripts intends to study the performance of state-constrained controllers utilizing control barrier functions in the context of adaptive cruise control.

## Concept of Control Lyapunov Functions

We consider a control affine plant of the form:

$$\\begin{equation}
\dot{x}(t) = f(x(t)) + g(x) (u(t))
\\end{equation}$$

For which we define the control objective of globally stabilizing the considered system to a point $x^∗ = 0$, and hence imposing $x(t) \rightarrow 0$.  This can be achieved by finding a feedback control law $k(x)$ that drives a positive-definite continuous function $V : D \in \mathbf{R}^{m} \rightarrow R \geq 0$. The system globally stabilizable, if there exists the class $\mathcal{K}_{\infty}$ functions $\alpha_1$, $\alpha_2$,  $\alpha_3$ such that

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

with $\lambda$ being $\lambda > 0$. Since any controller that respects the above requirements can ensure stability, there is no need to explicitly construct the feedback controller $k(X) = \begin{Bmatrix} u \in \mathbf{R}^{m} : \dot{V}(x, u) = [L_f V (x) + L_g V (x)u] ≤ −\lambda V (x) \end{Bmatrix}$. One can define the following positive definite control Lyapunov function (CLF) $V(x,u), which satisfies:

$$\\begin{equation}
    \inf_{u \in U} L_f V (x) + L_g V (x)u  \leq - \alpha_3 (\lVert x \rVert)
\\end{equation}$$

Since a solution space for $u=k(X)$ exists, secondary performance objectives can be considered by using an optimization-based approach. Using a quadratic programming formulation, the min-norm solution for $u$ can be found by solving:

$$\\begin{align}
&\min_{u \in \mathcal{U}}
\begin{aligned}[t]
  &\|u - u_d\|_2
\end{aligned} \\
&\text{s.t.} \notag \\
& L_f V(x,\hat{\theta}) + L_g V(x,\hat{\theta}) u \leq - c_3 V(x,\hat{\theta}), \notag
\\end{align}$$
  

## Concept of Control Barrier Functions

We consider a linear plant with parametric uncertainties of the form:

$$\\begin{equation}
\dot{x}_p(t) = A_p x_p(t) + B_p \Lambda (R(u(t))(t))
\\end{equation}$$

where $x_p(t) \in \mathbf{R}^{n}$ is a measurable state vector and $u(t) \in \mathbf{R}^{m}$ is a control input vector. The matrices $A_p \in \mathbf{R}^{n \times n}$  and $\Lambda \in \mathbf{R}^{m \times m}$ are unkown and $\Lambda$ has only diagonal positive entries. The control input is assumed to be magnitude limited by $\vert u_0 \vert$, which leads to the following closed set for the control input space

$$\\begin{equation}
\mathcal{U} = \begin{Bmatrix} u \in \mathbf{R}^{m} : - u_0 \geq u(t) \geq u_0 \end{Bmatrix}
\\end{equation}$$

The objective is to determine a $u(t)$ for \eqref{LinearPlantModel} such that the plant state $x_p(t)$ tracks a desired reference $ x_d(t)$ and that for any initial condition $x_0 := x(t_0) \in S$, it is ensured that the plant state vector $x_p(t)$ stays within the safe set $S \in \mathbf{R}^n$ i.e. the control input ensures that there is a CBF with $h(x,u) \geq 0$ for $\forall t \geq 0$.

 A closed set $\mathcal{C} \in \mathbf{R}^n$, which we consider as a safe set, is defined in the following form:
  
  $$\\begin{equation}
      \mathcal{C} = \begin{Bmatrix} x \in \mathbf{R}^n : h(x) \geq 0 \end{Bmatrix}
  \\end{equation}$$
  
with $h: \mathbf{R}^n \times \mathbf{R}^p \to \mathbf{R}$ being a  continuously differentiable function, called  control barrier function (CBF).
  
<p align=center>
<img src="https://github.com/JohannesAutenrieb/1D_MPC_CBF/blob/main/Images/CBF_Function_Plot.png" alt="CBF_Function_Plot" height=300px>
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

### Pointwise Controller and CBF-QP Safety Filter

A linear controller of the following form is defined:

$$\\begin{align}
u_d = K_1 x + K_2 x_d
\\end{align}$$

The linear controller is tuned regarding the desired control performance but cannot generate safe commands by itself. Therefore the following CLF-QP safety filter is used to adapt $u_d$ such that $x(t)$ stays within $\mathcal{C}$ for any $t$.

$$\\begin{align}
&\min_{u \in \mathcal{U}}
\begin{aligned}[t]
  &\|u - u_d\|_2
\end{aligned} \\
&\text{s.t.} \notag \\
& L_f h(x) + L_g h(x)u - \gamma h(x) \leq 0, \notag
\\end{align}$$

After each simulation run, a plot with results is given out. An example of such a plot is given here:

<p align=center>
<img src="https://github.com/JohannesAutenrieb/1D_MPC_CBF/blob/main/Images/Example_Simulation_Results.png" alt="MISSION_GUI" height=1000px>
</p>

## Dependencies


The scripts use external libraries, which need to be installed.
* YAMLIP
* Export_fiq


Further, the following MATLAB toolboxes are needed:
* Optimization Toolbox

**The software was tested with MATLAB 2020b under Windows 11 Home.** 

## License

The contents of this repository are covered under the [MIT License](LICENSE).


## License

The contents of this repository are covered under the [MIT License](LICENSE).


## References

We kindly acknowledge the following papers, which have been the foundation of the here presented scripts:

* [[1] J. Zeng, B. Zhang, and K. Sreenath, “Safety-Critical Model Predictive Control with Discrete-Time Control
Barrier Function,” in 2021 American Control Conference (ACC), May 2021, pp. 3882–3889.](https://ieeexplore.ieee.org/document/9483029)
* [[2] A. D. Ames, X. Xu, J. W. Grizzle and P. Tabuada, "Control Barrier Function Based Quadratic Programs for Safety Critical Systems," in IEEE Transactions on Automatic Control, vol. 62, no. 8, pp. 3861-3876.](https://ieeexplore.ieee.org/document/7782377)
