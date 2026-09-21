#　1　STATIC　OPTIMIZATION
## 1.1 OPTIMIZATION WITH CONSTRAINTS

It is desired to determine a control vector $u \in R^m$ that results in a minimum cost $L(u)$.

To analyze how $u$ affects $L(u)$, we can express the variation of $L$ as:

$$
dL = L(u+du) - L(u).
$$ 

$du$ denotes a change in vector $u$. By substituting the Taylor expansion into this definition and subtracting $L(u)$ from both sides, we obtain the final expression for the variation:

$$
dL = L_u^Tdu + \frac{1}{2}du^TL_{uu}du + O(3).
$$

where:
* $L_{u}$ is the gradient vector (first-order partial derivatives) evaluated at $u$.

* $L_{uu}$ is the Hessian matrix (second-order partial derivatives) evaluated at $u$.

* $O(3)$ represents third- and higher-order terms.

For a critical point 

$$
L_u = 0.
$$ 

For a critical point to be a local minimum, it is required that

$$
dL = \frac{1}{2}du^TL_{uu}du + O(3) \gt 0, \text{for any }du
$$ 

which is guaranteed when $L_{uu} \gt 0$ (positive definite).

The properties of $L_{uu}$ determine the local geometry of \(L(u)\) around a critical point:
* Negative definite ($L_{uu} < 0 $), a local maximum
* Indefinite, a saddle point.
* Semidefinite, higher terms of the expansion must be examined to determine the type of critical point.

**Note**. The gradient is defined throughout the book as a column vector

## 1.2 OPTIMIZATION WITH EQUALITY CONSTRAINTS
Now let the cost be $L(x, u)$, a function of the control vector $u$ and an state vector $x ∈ R^n$. The optimization problem is to determine the $u$ that minimizes $L(x, u)$ and at the same time satisfies the constraint equation.
$$f(x,u)= 0.$$ For any $u$, above equation provides $n$ scalar equations that determine the $x$.

Following the previous approach, we find local minimum conditions under $f(x,u)=0$ by Taylor-expanding $dL$.

$$
dL = L_x^T dx + L_u^T du + \frac{1}{2} \begin{bmatrix} dx^T & du^T \end{bmatrix} \begin{bmatrix} L_{xx} & L_{xu} \\ L_{ux} & L_{uu} \end{bmatrix} \begin{bmatrix} dx \\ du \end{bmatrix} + O(3).
$$ 

### Necessary conditions at critical point
At a stationary point, $dL = 0$ to first-order approximation for arbitrary increments $du$ when $df = 0$. Consequently, a critical point must satisfy the following equations

$$
\begin{align}
dL &= L_x^T dx + L_u^T du = 0 \\
df &= f_x dx + f_u du = 0
\end{align}
$$ 

From $(2)$ we get

$$
dx = -f_x^{-1} f_udu
$$ 

Substituting this into $(1)$ yields

$$ 
\begin{align*}
dL &= L_x^T (-f_x^{-1} f_udu) + L_u^T du \\
&=(L_u^T - L_x^T f_x^{-1} f_u)du
\end{align*}
$$ 

Holding $f$ constant, the derivative of $L$ with respect to $u$ is therefore:

$$
\frac{\partial L}{\partial u}\bigg|_{df=0} = (L_u^T - L_x^T f_x^{-1} f_u)^T = L_u-f_u^Tf_x^{-T}L_x
$$ 

Where

$$
f_x^{-T} = (f_x^{-1})^T
$$ 

and 

$$
L_u = \frac{\partial L}{\partial u}\bigg|_{dx=0}
$$ 

Thus, for $dL = 0$ to hold to first order for arbitrary  $du$ when $df = 0$, we must have:

$$
\begin{align}
L_u - f_u^Tf_x^{-T}L_x = 0 
\end{align}
$$ 

Which is a **necessary condition** for a minimum. 

Let's explore two alternative ways to obtain $(3)$. 

####  Lagrange multiplier 
First write $(1)(2)$ as:

$$
\begin{bmatrix} dL \\ df \end{bmatrix} = \begin{bmatrix} L_{x}^T & L_{u}^T \\ f_{x} & f_{u} \end{bmatrix} \begin{bmatrix} dx \\ du \end{bmatrix}=0
$$

The linear system must yield a coherent solution. Algebraically, this system is represented by an \((n + 1) \times (n + m)\) coefficient matrix. 

A strict critical point implies a state of geometric tangency, where the gradient of $L$ becomes aligned with the gradient of the constraints $f$. For this specific geometric alignment to occur, the coefficient matrix must lose full row rank; its rank must be less than \(n + 1\). This reduction in rank dictates that the rows of the matrix are linearly dependent, which guarantees that there exist  $n$-dimensional vector $\lambda$ such that

$$
\begin{bmatrix} 1 & \lambda^T \end{bmatrix}\begin{bmatrix} L_{x}^T & L_{u}^T \\ f_{x} & f_{u} \end{bmatrix}=0
$$

From above, we get:

$$
\begin{align}
L_x^T + \lambda^Tf_x &= 0 \\
L_u^T + \lambda^Tf_u &=0
\end{align}
$$

Solving $(4)$ for $\lambda$ and  substituting in the $(5)$ condition:

$$
\begin{align*}
\lambda^T = -L_x^Tf_x^{-1} \\
L_u^T + (-L_x^Tf_x^{-1})f_u &=0
\end{align*}
$$

Again yields $(3)$, where the vector $\lambda$ is called **Lagrange multiplier**. Let $du=0$ in $(1)(2)$ and substitute $dx$:

$$
dL = L_x^T f_x^{-1}df.
$$

Which can be 

$$
\frac{\partial L}{\partial f}\bigg|_{du=0} = (L_x^T f_x^{-1})^T = -\lambda.
$$

Thus, $-\lambda$ represents the partial derivative of $L$ with respect to the constraint $f$ while holding the control $u$ constant. This captures how changes in the constraints affect the performance index when the control remains fixed.


####  Hamiltonian function

As a third method of obtaining $(3)$,is the **Hamiltonian function**

$$
\begin{align}
H(x,u,\lambda) = L(x, u) + \lambda^Tf(x, u).
\end{align}
$$

Where $λ ∈ R^n$ is an undetermined Lagrange multiplier. To determine $x$, $u$ and $λ$, resulting in a critical point, we assume that:

$$
dH = H_x^Tdx + H_u^Tdu + H_{\lambda}^Td\lambda.
$$

First that:

$$
\begin{align}
H_{\lambda} = f(x, u)=0.
\end{align}
$$

Which is the constraint relation. And so that:

$$
H|_{f=0} = L
$$

Not taking the coupling between $du$ and $dx$ into account, it is convenient to choose $\lambda$ such that:

$$
\begin{align}
H_x = L_x + f_x^T\lambda=0
\end{align}
$$

And assume $(7)(8)$ are satisfied, then:

$$
dH = dL = H_u^Tdu
$$

In this conditions $H = L$ and based on $(1)$: $dL=0$ for critical point, finnally we get:

$$
\begin{align}
H_u = 0
\end{align}
$$

In summary, necessary conditions for a minimum point of $L(x, u)$ that also satisfies the constraint $f (x, u) = 0$ are

$$
\begin{align*}
\frac{\partial H}{\partial \lambda} &= f = 0.\\
\frac{\partial H}{\partial x} &= L_x + f_x^T\lambda = 0.\\
\frac{\partial H}{\partial u} &= L_u + f_u^T\lambda = 0.
\end{align*}
$$

Introducing Lagrange multipliers transforms the problem of minimizing $L(x, u)$ under the constraint $f(x, u) = 0$ into an minimization of the Hamiltonian $H(x, u, \lambda)$ without constraints.

### Sufficient conditions at critical point
While conditions $(7)(8)(9)$ determine a stationary point, we now derive a test to guarantee that this point is a local minimum. 

$$
\begin{align}
dL &= \begin{bmatrix}L_x^T && L_u^T \end{bmatrix}\begin{bmatrix}dx \\ du \end{bmatrix} + \frac{1}{2} \begin{bmatrix} dx^T & du^T \end{bmatrix} \begin{bmatrix} L_{xx} & L_{xu} \\ L_{ux} & L_{uu} \end{bmatrix} \begin{bmatrix} dx \\ du \end{bmatrix} + O(3). \\ 
df &=  \begin{bmatrix}f_x && f_u \end{bmatrix}\begin{bmatrix}dx \\ du \end{bmatrix} + \frac{1}{2} \begin{bmatrix} dx^T & du^T \end{bmatrix} \begin{bmatrix} f_{xx} & f_{xu} \\ f_{ux} & f_{uu} \end{bmatrix} \begin{bmatrix} dx \\ du \end{bmatrix} + O(3).
\end{align}
$$ 

Recall that $(6)$, and use Hamiltonian to rewrite the equtions:

$$
\begin{align}
\begin{bmatrix}1 && \lambda^T \end{bmatrix} \begin{bmatrix}dL \\ df \end{bmatrix}=\begin{bmatrix}H_x^T && H_u^T \end{bmatrix}\begin{bmatrix}dx \\ du \end{bmatrix}+ \frac{1}{2} \begin{bmatrix} dx^T & du^T \end{bmatrix} \begin{bmatrix} H_{xx} & H_{xu} \\ H_{ux} & H_{uu} \end{bmatrix} \begin{bmatrix} dx \\ du \end{bmatrix} + O(3).
\end{align}
$$

To find sufficient conditions for a minimum, we examine the second-order term. We must first account for the dependence of $dx$ on $du$ in $(12)$. Assuming we are at a critical point where $H_x = 0$, $H_u = 0$, and $df = 0$, it follows that:

$$
dx = -f_x^{-1} f_udu + O(2)
$$

Substituting this relation into $(12)$ yields

$$
dL = \frac{1}{2}du^T \begin{bmatrix} -f_u^Tf_x^{-T} && I \end{bmatrix} \begin{bmatrix} H_{xx} & H_{xu} \\ H_{ux} & H_{uu} \end{bmatrix} \begin{bmatrix} -f_x^{-1}f_u \\ I \end{bmatrix}du + O(3).
$$

To ensure a minimum, $dL$  should be **positive for all increments $du$.** Which is guaranteed when the Hessian matrix with $df=0$ :

$$
\begin{align}
L_{uu}|_{f} &= \begin{bmatrix} -f_u^Tf_x^{-T} && I \end{bmatrix} \begin{bmatrix} H_{xx} & H_{xu} \\ H_{ux} & H_{uu} \end{bmatrix} \begin{bmatrix} -f_x^{-1}f_u \\ I \end{bmatrix} \\
&= H_{uu} - f_u^T f_x^{-T} H_{xu} - H_{ux} f_x^{-1} f_u +  - f_u^T f_x^{-T} H_{xx} f_x^{-1} f_u
\end{align}
$$

is positive definite. If the constraint $f(x, u)$ is identically zero for all $x$ and $u$, then $(14)$ reduces to $L_{uu}$. If $(14)$ is negative definite (or indefinite), the stationary point is a constrained maximum (or saddle point).
