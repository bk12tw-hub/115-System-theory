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
$$where:
* $L_{u}$ is the gradient vector (first-order partial derivatives) evaluated at $u$.

* $L_{uu}$ is the Hessian matrix (second-order partial derivatives) evaluated at $u$.

* $O(3)$ represents third- and higher-order terms.

For a critical point 
$$
L_u = 0.
$$ For a critical point to be a local minimum, it is required that
$$
dL = \frac{1}{2}du^TL_{uu}du + O(3) \gt 0, \text{for any }du
$$ which is guaranteed when $L_{uu} \gt 0$ (positive definite).

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
$$ For a point to be a critical point, following equations need to be satisfied:
$$
\begin{align}
dL &= L_x^T dx + L_u^T du = 0 \\
df &= f_x dx + f_u du = 0
\end{align}
$$ From $(2)$ we get
$$dx = -f_x^{-1} f_udu$$ Substituting this into $(1)$ yields
$$ 
\begin{align*}
dL &= L_x^T (-f_x^{-1} f_udu) + L_u^T du \\
&=(L_u^T - L_x^T f_x^{-1} f_u)du
\end{align*}
$$ Holding $f$ constant, the derivative of $L$ with respect to $u$ is therefore:
$$\frac{\partial L}{\partial u}\bigg|_{df=0} = (L_u^T - L_x^T f_x^{-1} f_u)^T = L_u-f_u^Tf_x^{-T}L_x$$ Where 
$$f_x^{-T} = (f_x^{-1})^T$$ and $$L_u = \frac{\partial L}{\partial u}\bigg|_{dx=0}$$ Thus, for $dL = 0$ to hold to first order for arbitrary  $du$ when $df = 0$, we must have:
$$L_u - f_u^Tf_x^{-T}L_x = 0 $$ Which is a **necessary condition** for a minimum. 


