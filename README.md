# CBF_ACC

### Discription
This collection of MATLAB scripts intends to study the performance of state-constrained controllers utilizing control barrier functions in the context of adaptive cruise control.

## Concept of Control Lyapunov Functions

We consider a linear plant with parametric uncertainties of the form:

$$\\begin{equation}
\dot{x}_p(t) = A_p x_p(t) + B_p \Lambda (R(u(t))(t))
\\end{equation}$$

       
A positive-definite continuous function $V(x,\hat{\theta})$ is a globally stabilizing CLF for \eqref{system_equation_2} if there exists the class $\mathcal{K}_{\infty}$ functions $\alpha_1$, $\alpha_2$,  $\alpha_3$ such that:

\begin{equation}
    \label{CLF_Condition_1}
    \alpha_1 (\lVert x \rVert, \hat{\theta}) \leq V(x, \hat{\theta}) \leq \alpha_2 (\lVert x \rVert, \hat{\theta})
\end{equation}


\begin{equation}
    \label{CLF_Condition_2}
    \inf_{u \in U} \dot{V}(x,\theta, u) \leq - \alpha_3 (\lVert x \rVert, \hat{\theta})
\end{equation}

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

Where $\alpha(h(x)$ often chosen to be

$$\\begin{equation}
    \alpha(h(x)) = \gamma h(x)
\\end{equation}$$

with $\gamma$ being $\gamma > 0$.

To motivate safety for systems of this form, and hence
control barrier functions, we begin by considering the famil-
iar objective of stabilizing the system. Suppose we have the
control objective of (asymptotically) stabilizing the nonlinear
control system (1) to a point x∗ = 0, i.e., driving x(t) → 0.
In a nonlinear context, this can be achieved—and, in fact,
understood—by equivalently finding a feedback control law
that drives a positive definite function, V : D ⊂ Rn → R≥0,
to zero. That is, if
∃ u = k(x) s.t.  ̇V (x, k(x)) ≤ −γ(V (x)), (2)
where
 ̇V (x, k(x)) = Lf V (x) + Lg V (x)k(x),
then the system is stabilizable to V (x∗) = 0, i.e., x∗ = 0.
Note that here γ : R≥0 → R≥0 is a class K function
defined on the entire real line for simplicity, i.e., γ maps
zero to zero, γ(0) = 0, and it is strictly monotonic: for
all r1, r2 ∈ R≥0, r1 < r2 implies that γ(r1) < γ(r2).
Thus, the process of stabilizing a nonlinear system can be
understood as finding an input that creates a one-dimensional
stable system given by the Lyapunov function:  ̇V ≤ −γ(V ),
wherein the comparison lemma (see, e. g., [33]) implies that
the full-order nonlinear system (1) is thus stable under the
control law u = k(x).
The above observations motivate the notion of a control
Lyapunov function wherein a function V is shown to stabilize
the system without the need to explicitly construct the
feedback controller u = k(x). That is, as first observed
by Sontag and Artstein [34], [35], [36], we only need a
controller to exist that results in the desired inequality on
 ̇V . Concretely, V is a control Lyapunov function (CLF) if it
is positive definite and satisfies:
inf
u∈U [Lf V (x) + Lg V (x)u] ≤ −γ(V (x)), 


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

Where $\alpha(h(x)$ often chosen to be

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
