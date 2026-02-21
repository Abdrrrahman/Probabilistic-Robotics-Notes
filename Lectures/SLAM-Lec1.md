# Robot Mapping & SLAM — Quick Recap

---
## Core Definitions

| Term                 | Meaning                                                                                          |
| -------------------- | ------------------------------------------------------------------------------------------------ |
| **Robot**            | A device that moves through an environment with sensors, mobility, and controls.                 |
| **Map**              | Any model of the environment used for decision-making. Geometric, topological, dense, or sparse. |
| **Pose**             | Position + orientation together: $(x, y, \theta)$                                                |
| **State Estimation** | Estimating unknowns (position, landmarks) from noisy data                                        |
| **Localization**     | Estimating where the robot is — assumes a known map                                              |
| **Mapping**          | Building a map — assumes known poses                                                             |
| **SLAM**             | Both at once — no known map, no known poses                                                      |
| **Navigation**       | Robot decides where to go — relies on SLAM output                                                |

---
## The SLAM Problem

### The Chicken-or-Egg Issue

> Good localization needs a map. Good mapping needs accurate poses. They're coupled — you have to solve them together.
### Formal Setup
$$\text{Inputs: } u_{1:t} \text{ (controls/odometry)}, \quad z_{1:t} \text{ (sensor observations)}$$
$$\text{Outputs: } m \text{ (map)}, \quad x_{0:t} \text{ (robot path)}$$
> [!note] Note There's always one more pose than controls. If you have $t$ controls, you get poses $x_0, x_1, \ldots, x_t$.
### Full SLAM vs Online SLAM
**Full SLAM**: estimate the entire trajectory:
$$P(x_{0:t},, m \mid z_{1:t},, u_{1:t})$$
**Online SLAM**: estimate only the current pose:
$$P(x_t,, m \mid z_{1:t},, u_{1:t})$$
Online SLAM is obtained by marginalizing out past poses:
$$P(x_t, m \mid z_{1:t}, u_{1:t}) = \int \cdots \int P(x_{0:t}, m \mid z_{1:t}, u_{1:t}), dx_0 \cdots dx_{t-1}$$
Most real applications only need online SLAM.

---
## Why SLAM is Hard

> [!warning] Challenge 1 — Correlation Map estimates and pose estimates depend on each other. Fixing one changes the other. You can't decouple them.

> [!warning] Challenge 2 — Data Association Is this the same landmark I saw before? A wrong match makes the system _confidently wrong_ — uncertainty shrinks around a bad estimate. This is called an **inconsistent estimate**.

> [!warning] Challenge 3 — Error Accumulation Even tiny motion errors stack up over time. At small scales you can ignore it. At large scales you can't.

---
## Why Probabilistic?
Sensors and motors always have noise. Instead of saying "the robot is here," we maintain a full distribution.

The goal of SLAM is to estimate:
$${P(x_{0:t},, m \mid z_{1:t},, u_{1:t})}$$

---
## Two Fundamental Models
### Motion Model
$$P(x_t \mid x_{t-1},, u_t)$$
Given the previous pose and a control command → probability distribution over the new pose.
Used in the **prediction step**.

> [!tip] The Banana Problem Real motion uncertainty isn't Gaussian — small orientation errors curve into a banana-shaped distribution over longer distances due to the nonlinear link between angle and position.
### Observation Model
$$P(z_t \mid x_t,, m)$$
Given the current pose and the map → what sensor reading do you expect?
Used in the **correction step**.
> [!tip] Real sensors aren't Gaussian. A laser scanner can hit glass, reflect off walls, miss entirely, or return max-range garbage. The model needs to capture all of that — usually a multi-modal distribution.

---
## SLAM Flavors

| Axis           | Option A                    | Option B                         |
| -------------- | --------------------------- | -------------------------------- |
| Map type       | Volumetric (dense grid)     | Feature-based (sparse landmarks) |
| Representation | Geometric (metric)          | Topological (connections only)   |
| Correspondence | Known (perfect matching)    | Unknown (must figure it out)     |
| Environment    | Static                      | Dynamic                          |
| Uncertainty    | Small: single Gaussian      | Large: multiple hypotheses       |
| Control        | Passive (externally driven) | Active (robot explores itself)   |
| Robots         | Single                      | Multi-robot                      |

---

## Three Main Approaches
### 1. Kalman Filter (EKF)
- Assumes Gaussian noise everywhere
- Represents uncertainty as a covariance matrix $\Sigma$
- Linearizes nonlinear systems via Jacobians → errors can build up
- Solves **Online SLAM**
**Predict:** $\bar{\mu}_t = g(u_t, \mu_{t-1})$
**Update:** $K_t = \bar{\Sigma}_t H_t^T (H_t \bar{\Sigma}_t H_t^T + Q_t)^{-1}$

---
### 2. Particle Filter
- Represents the distribution with $N$ weighted samples (particles)
- Can handle **multi-modal** distributions — robot could be _here_ OR _there_
- No Gaussian assumption needed, but computationally heavier
- Solves **Online SLAM**

Each particle $i$ carries: $\langle x_t^{[i]},, w_t^{[i]} \rangle$

---
### 3. Graph-Based SLAM
- **Nodes** = robot poses / landmarks
- **Edges** = constraints from odometry or observations
- Solve Full SLAM as a least-squares optimization:

$$x^* = \arg\min_x \sum_{ij} e_{ij}(x_i, x_j)^T \Omega_{ij}, e_{ij}(x_i, x_j)$$
- Modern, scalable, handles large maps and multi-robot well
- Primarily **Full SLAM**, online variants exist

---
## Graphical Model
```
u_{t-1}      u_t
   |            |
x_{t-1} ──→ x_t ──→ ...
              ↓
             z_t
              ↑
              m
```
- $x_t$ depends on $x_{t-1}$ and $u_t$ → **motion model**
- $z_t$ depends on $x_t$ and $m$ → **observation model**
---
## Applications
- **Home:** vacuums, lawn mowers
- **Aerial:** UAV surveillance, crop monitoring, disaster response
- **Underwater:** reef and seafloor mapping
- **Underground:** mines, catacombs, archaeological sites
- **Space:** planetary rovers
- **Industry:** mobile manipulators, flexible production lines
---

_Based on Cyrill Stachniss — Robot Mapping Course (2012–2014)_