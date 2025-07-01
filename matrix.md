# Computation On Serial Matrix

## Forward Kinematics

$$
^0_1T = \begin{bmatrix}
C_1 & -S_1 & 0 & 0 \\
S_1 & C_1 & 0 & 0\\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
^1_2T = \begin{bmatrix}
C_2 & 0 & S_2 & 1.3 \\
0 & 1 & 0 & 40\\
-S_2 & 0 & C_2 & 95 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

$$
^0_2T = \begin{bmatrix}
C_1C_2 & -S_1 & C_1S_2 & 1.3C_1-40S_1 \\
S_1C_2 & C_1 & S_1S_2 & 40C_1+1.3S_1\\
-S_2 & 0 & C_2 & 95 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
^2_3T = \begin{bmatrix}
C_3 & 0 & S_3 & -133.3 \\
0 & 1 & 0 & -27.5\\
-S_3 & 0 & C_3 & 0.5 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

$$
^0_3T = \begin{bmatrix}
C_1C_{23} & -S_1 & C_1S_{23} & -133.3C_1C_2+0.5C_1S_2+1.3C_1-12.5S_1 \\
S_1C_{23} & C_1 & S_1S_{23} & -133.3S_1C_2+0.5S_1S_2+1.3S_1+12.5C_1\\
-S_{23} & 0 & C_{23} & 0.5C_2+133.3S_2+95 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
^3_{ee}T = \begin{bmatrix}
1 & 0 & 0 & -126.994 \\
0 & 1 & 0 & -12.2355 \\
0 & 0 & 1 & 2.8614 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

$$
^0_{ee}T = \begin{bmatrix}
C_1C_{23} & -S_1 & C_1S_{23} & 2.8614C_1S_{23}-126.994C_1C_{23}-133.3C_1C_2+0.5C_1S_2+1.3C_1-0.2645S_1 \\
S_1C_{23} & C_1 & S_1S_{23} & 2.8614S_1S_{23}-126.994S_1C_{23}-133.3S_1C_2+0.5S_1S_2+1.3S_1+0.2645C_1 \\
-S_{23} & 0 & C_{23} & 126.994S_{23}+2.8614C_{23}+0.5C_2+133.3S_2+95 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

> **Solutions**:  
> $$
> ^0_{ee}T = \begin{bmatrix}
> C_1C_{23} & -S_1 & C_1S_{23} & C_1(2.8614S_{23}-126.994C_{23}-133.3C_2+0.5S_2+1.3)-0.2645S_1 \\
> S_1C_{23} & C_1 & S_1S_{23} & S_1(2.8614S_{23}-126.994C_{23}-133.3C_2+0.5S_2+1.3)+0.2645C_1 \\
> -S_{23} & 0 & C_{23} & 126.994S_{23}+2.8614C_{23}+0.5C_2+133.3S_2+95 \\
> 0 & 0 & 0 & 1 \\
> \end{bmatrix}
> $$

## Inverse kinematics

### Inverse Kinematics Summary
#### Derivation
The following equations solve for joint angles $\theta_1$, $\theta_2$, and $\theta_3$ given end-effector position $(x, y, z)$. Constants are precomputed as:
- $R = 2.8614^2 + 126.994^2 \approx 16135.95$
- $C = 133.3^2 + 0.5^2 = 17769.14$

**Step 1: Solve for $\theta_1$**
$$
s_1 x - c_1 y = -0.2645
$$
where $s_1 = \sin\theta_1$, $c_1 = \cos\theta_1$.  
> **Solutions**:  
> $$
> \theta_1 = atan2(x, -y) \pm \arccos(\frac{-0.2645}{\sqrt{x^2 + y^2}})
> $$
> *Requirement*: $x^2 + y^2 \geq 0.2645^2$ for real solutions.  
> *(2 potential solutions)*

**Step 2: Solve for $\theta_2$**
For each $\theta_1$, compute:  
$$
\boxed{
\begin{aligned}
U &= c_1 x + s_1 y - 1.3 \\
V &= z - 95 \\
M &= 133.3U - 0.5V \\
N &= -0.5U - 133.3V \\
L &= \frac{1}{2}( R - C - U^2 - V^2 )
\end{aligned}
}
$$
**Solve**:  
$$
M \cos\theta_2 + N \sin\theta_2 = L
$$
> **Solutions**:  
> $$
> \theta_2 = atan2(N, M) \pm \arccos(\frac{L}{\sqrt{M^2 + N^2}})
> $$
> *Requirement*: $M^2 + N^2 \geq L^2$ for real solutions.  
> *(Up to 2 solutions per $\theta_1$)*

**Step 3: Solve for $\theta_3$**
For each $(\theta_1, \theta_2)$ pair, compute:  
$$
\boxed{
\begin{aligned}
P &= U + 133.3c_2 - 0.5s_2 \\
Q &= V - 0.5c_2 - 133.3s_2 \\
s_{23} &= (2.8614P + 126.994Q) / R \\
c_{23} &= (-126.994P + 2.8614Q) / R 
\end{aligned}
}
$$
> **Solutions**: 
> $$
> \theta_3 = atan2(s_{23}, c_{23}) - \theta_2
> $$
> where $s_2 = \sin\theta_2$, $c_2 = \cos\theta_2$.  
> *(1 solution per $(\theta_1, \theta_2)$ pair)*

---
> **Note**:  
> Solution Count
> - $\theta_1$: Up to 2 solutions
> - $\theta_2$: Up to 2 solutions per $\theta_1$
> - $\theta_3$: 1 solution per $(\theta_1, \theta_2)$  
> **Total**: Up to 4 inverse kinematics solutions for a given $(x, y, z)$.
---
> **Note**:  
> 1. **Verification**: Always plug solutions back into forward kinematics to verify.
> 2. **Singularities**: 
>    - If $x^2 + y^2 < 0.2645^2$, $\theta_1$ is undefined.
>    - If $M^2 + N^2 < L^2$, $\theta_2$ is undefined for that $\theta_1$.
> 3. **Efficiency**: Precompute constants $R$ and $C$ for speed.
> 4. **Numerical Stability**: Use robust `atan2` and handle edge cases (e.g., $\sqrt{x^2 + y^2} \approx 0.2645$).
---

### Jacobian for Joint-Space Trajectory Control

The Jacobian matrix $ J $ relates joint velocities $ \dot{\theta} $ to end-effector linear/angular velocities $ \dot{x} $:

$$
\dot{x} = J(\theta) \dot{\theta}
$$

where:
- $ \dot{x} = \begin{bmatrix} v_x \\ v_y \\ v_z \end{bmatrix} $ (end-effector linear velocity)
- $ \dot{\theta} = \begin{bmatrix} \dot{\theta}_1 \\ \dot{\theta}_2 \\ \dot{\theta}_3 \end{bmatrix} $ (joint velocities)
- $ J(\theta) $ is the **geometric Jacobian** (size $ 3 \times 3 $ for position control).

**1. Deriving the Jacobian from Forward Kinematics**
Given your forward kinematics:
$$
^0_{ee}T = \begin{bmatrix}
C_1 C_{23} & -S_1 & C_1 S_{23} & x \\
S_1 C_{23} & C_1 & S_1 S_{23} & y \\
-S_{23} & 0 & C_{23} & z \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

The **position part** of the end-effector is:
$$
x = 2.8614 C_1 S_{23} - 126.994 C_1 C_{23} - 133.3 C_1 C_2 + 0.5 C_1 S_2 + 1.3 C_1 - 0.2645 S_1
$$
$$
y = 2.8614 S_1 S_{23} - 126.994 S_1 C_{23} - 133.3 S_1 C_2 + 0.5 S_1 S_2 + 1.3 S_1 + 0.2645 C_1
$$
$$
z = 126.994 S_{23} + 2.8614 C_{23} + 0.5 C_2 + 133.3 S_2 + 95
$$

The **Jacobian** is computed by taking partial derivatives:
$$
J = \begin{bmatrix}
\frac{\partial x}{\partial \theta_1} & \frac{\partial x}{\partial \theta_2} & \frac{\partial x}{\partial \theta_3} \\
\frac{\partial y}{\partial \theta_1} & \frac{\partial y}{\partial \theta_2} & \frac{\partial y}{\partial \theta_3} \\
\frac{\partial z}{\partial \theta_1} & \frac{\partial z}{\partial \theta_2} & \frac{\partial z}{\partial \theta_3}
\end{bmatrix}
$$

**2. Analytical Jacobian Components**
$$
\frac{\partial x}{\partial \theta_1} = -S_1 (2.8614 S_{23} - 126.994 C_{23} - 133.3 C_2 + 0.5 S_2 + 1.3) - 0.2645 C_1
$$

$$
\frac{\partial x}{\partial \theta_2} = C_1 (2.8614 C_{23} + 126.994 S_{23} + 133.3 S_2 + 0.5 C_2)
$$

$$
\frac{\partial x}{\partial \theta_3} = C_1 (2.8614 C_{23} + 126.994 S_{23})
$$

$$
\frac{\partial y}{\partial \theta_1} = C_1 (2.8614 S_{23} - 126.994 C_{23} - 133.3 C_2 + 0.5 S_2 + 1.3) - 0.2645 S_1
$$
$$
\frac{\partial y}{\partial \theta_2} = S_1 (2.8614 C_{23} + 126.994 S_{23} + 133.3 S_2 + 0.5 C_2)
$$

$$
\frac{\partial y}{\partial \theta_3} = S_1 (2.8614 C_{23} + 126.994 S_{23})
$$

$$
\frac{\partial z}{\partial \theta_1} = 0
$$

$$
\frac{\partial z}{\partial \theta_2} = 126.994 C_{23} - 2.8614 S_{23} - 0.5 S_2 + 133.3 C_2
$$

$$
\frac{\partial z}{\partial \theta_3} = 126.994 C_{23} - 2.8614 S_{23}
$$

**3. Final Jacobian Matrix**

> **Solutions**: 
> $$
> J = \begin{bmatrix}
> -S_1 A - 0.2645 C_1 & C_1 B & C_1 D \\
> C_1 A - 0.2645 S_1 & S_1 B & S_1 D \\
> 0 & E & F
> \end{bmatrix}
> $$
> 
> where:
> $$
> \begin{aligned}
> A &= 2.8614 S_{23} - 126.994 C_{23} - 133.3 C_2 + 0.5 S_2 + 1.3 \\
> B &= 2.8614 C_{23} + 126.994 S_{23} + 133.3 S_2 + 0.5 C_2 \\
> D &= 2.8614 C_{23} + 126.994 S_{23} \\
> E &= 126.994 C_{23} - 2.8614 S_{23} - 0.5 S_2 + 133.3 C_2 \\
> F &= 126.994 C_{23} - 2.8614 S_{23}
> \end{aligned}
> $$
>
> or
> $$
> \begin{aligned}
> A &= -E + 1.3 \\
> B &= D + 133.3 S_2 + 0.5 C_2 \\
> D &= 2.8614 C_{23} + 126.994 S_{23} \\
> E &= F - 0.5 S_2 + 133.3 C_2 \\
> F &= 126.994 C_{23} - 2.8614 S_{23}
> \end{aligned}
> $$

**4. Using the Jacobian for Trajectory Control**

**(a) Forward Velocity Kinematics**
$$
\begin{bmatrix} v_x \\ v_y \\ v_z \end{bmatrix} = J \begin{bmatrix} \dot{\theta}_1 \\ \dot{\theta}_2 \\ \dot{\theta}_3 \end{bmatrix}
$$

**(b) Inverse Velocity Kinematics (Joint Velocities)**
If $ J $ is invertible (non-singular), compute joint velocities:
$$
\dot{\theta} = J^{-1} \dot{x}
$$

**(c) Singularity Handling**
- **Singular configurations**: When $\det(J) = 0$ (e.g., $\theta_2 = 0$ or $\theta_3 = 0$).
- **Damped Least Squares (DLS)** can be used to avoid numerical instability:
  $$
  \dot{\theta} = J^T (J J^T + \lambda^2 I)^{-1} \dot{x}
  $$
  where $\lambda$ is a damping factor.

> **Note**:  
> - The **Jacobian** maps joint velocities to end-effector velocities.
> - **Analytical derivation** involves partial differentiation of FK equations.
> - **Singularities** must be handled for smooth trajectory control.
> - **Numerical methods** (DLS, pseudoinverse) improve robustness.

---
End of Document
