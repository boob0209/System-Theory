# System Theory Markdown HW1
------

## **Chapter1**
### **1.1 Optimization without Constraints**

* A **scalar performance index** $L(u)$ is a function of a **control or decision vector**

$$
u \in \mathbb{R}^m
$$

The objective is to determine the value of $u$ that minimizes $L(u)$.

* Using the **Taylor series expansion** for an increment in $L$:

$$
dL = L_u^Tdu + \frac{1}{2}du^T L_{uu}du + O(3)
$$

where $O(3)$ represents terms of order three and higher.

* **Gradient**

$$
L_u \triangleq \frac{\partial L}{\partial u}
$$

* **Hessian matrix (curvature matrix)**

$$
L_{uu} \triangleq \frac{\partial^2L}{\partial u^2}
$$

----
### **Example 1.1-1. Quadractic Surfaces**

For the quadratic performance index

$$
L(u)=\frac{1}{2}u^TQu+S^Tu
$$

the critical point is obtained from

$$
L_u=Qu+S=0
$$

so

$$
u^*=-Q^{-1}S.
$$

The Hessian is

$$
L_{uu}=Q,
$$

which determines the type of critical point.

For

$$
Q=
\begin{bmatrix}
1&1\\
1&2
\end{bmatrix},
\qquad
S=
\begin{bmatrix}
0\\
1
\end{bmatrix},
$$

we obtain

$$
u^*=
\begin{bmatrix}
1\\
-1
\end{bmatrix},
\qquad
L^*=-\frac{1}{2}.
$$

Since $Q>0$, $u^*$ is a **minimum**.

![Contours and the gradient vector](./images/Fig1.1-1.png)

> The gradient is perpendicular to the contours and points in the direction of increasing $L(u)$.
### **1.2 Optimization with Equality Constraints**

* A **scalar performance index** $L(x,u)$ is a function of a **control vector** $u \in \mathbb{R}^m$ and an **auxiliary (state) vector** $x \in \mathbb{R}^n$.

    The objective is to minimize $L(x,u)$ while satisfying the **equality constraint**

$$
f(x,u)=0
$$

* Introduce the **Lagrange multiplier** $\lambda$ and define the **Hamiltonian function**

$$
H(x,u,\lambda)=L(x,u)+\lambda^T f(x,u)
$$

* **Necessary conditions** for a stationary point:

$$
H_\lambda=\frac{\partial H}{\partial \lambda}=f(x,u)=0
$$

$$
H_x=\frac{\partial H}{\partial x}
=L_x+f_x^T\lambda=0
$$

$$
H_u=\frac{\partial H}{\partial u}
=L_u+f_u^T\lambda=0
$$

These equations are used to determine $x$, $\lambda$, and $u$.

* **Sufficient condition**

  The necessary conditions only determine a stationary point.  
For a constrained minimum, the **curvature matrix with $f$ equal to zero is positive definite.
------
### **Example 1.2-1. Quadratic Surface with Linear Constraint**

* The performance index and **equality constraint** are

$$
L(x,u)=\frac{1}{2}x^2+xu+u^2+u
$$

$$
f(x,u)=x-3=0
$$

* Define the **Hamiltonian**

$$
H=L+\lambda f
=\frac{1}{2}x^2+xu+u^2+u+\lambda(x-3)
$$

* **Necessary conditions**

$$
H_\lambda=x-3=0
$$

$$
H_x=x+u+\lambda=0
$$

$$
H_u=x+2u+1=0
$$

Solving these equations gives

$$
x=3,\qquad u=-2,\qquad \lambda=-1
$$

Therefore, the constrained stationary point is

$$
(x,u)^*=(3,-2)
$$

and the constrained curvature is

$$
L_{uu}^{f}=2>0
$$

so $(3,-2)$ is a **constrained minimum**.

![Contours of L and the constraint](./images/Fig1.2-1.png)

* At the constrained minimum, the gradients of $L$ and $f$ are **parallel**, and the constraint is **tangent to a contour of $L$**.

---------

## **Chapter2**
### **2.1 Solution of the General Discrete-Time Optimization Problem**

* The nonlinear **discrete-time system** is

$$
x_{k+1}=f^k(x_k,u_k)
$$

where $x_k$ is the **state vector** and $u_k$ is the **control vector**.

* The **performance index** over the time interval $[i,N]$ is

$$
J_i=\phi(N,x_N)+\sum_{k=i}^{N-1}L^k(x_k,u_k)
$$

where $\phi(N,x_N)$ is the **terminal cost** and $L^k(x_k,u_k)$ is the **cost at each time step**.

The objective is to find the control sequence $u_k^*$ that minimizes $J_i$ while satisfying the system dynamics.

* Introduce the **Lagrange multiplier (costate)** $\lambda_{k+1}$ and define the **Hamiltonian**


$$
H^k(x_k,u_k)=L^k(x_k,u_k)+\lambda_{k+1}^T f^k(x_k,u_k)
$$

* **State equation**

$$
x_{k+1}=\frac{\partial H^k}{\partial \lambda_{k+1}}=f^k(x_k,u_k)
$$

The state equation develops **forward in time**.

* **Costate equation**

$$
\lambda_k=\frac{\partial H^k}{\partial x_k}=
\left(\frac{\partial f^k}{\partial x_k}\right)^T\lambda_{k+1}+
\frac{\partial L^k}{\partial x_k}
$$

The costate equation develops **backward in time**.

* **Stationarity condition**

$$
0=\frac{\partial H^k}{\partial u_k}=\left(\frac{\partial f^k}{\partial u_k}\right)^T\lambda_{k+1}
+\frac{\partial L^k}{\partial u_k}
$$

This condition is used to determine the **optimal control** $u_k^*$.

* **Boundary conditions**

For a **fixed initial state**,

$$
x_i=\text{given}
$$

For a **fixed final state**,

$$
x_N=\text{given}
$$

For a **free final state**,

$$
\lambda_N=\frac{\partial\phi}{\partial x_N}
$$
