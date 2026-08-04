# Localization Architecture (AMCL)

## Particle Filter Model
Localization within a pre-built static map uses **Adaptive Monte Carlo Localization (AMCL)** based on KLD-sampling.

### Motion Model
Differential drive motion model updates particle states $(x, y, \theta)_i$ based on fused odometry increments $(\Delta x, \Delta y, \Delta \theta)$ with noise parameters $\alpha_1 \dots \alpha_4$.

### Sensor Model
Likelihood-field range finder model computes particle weights $w_i$ by matching current 2D LiDAR scans against the occupancy grid map:
$$p(z | x, m) = z_{\text{hit}} p_{\text{hit}} + z_{\text{rand}} p_{\text{rand}} + z_{\text{max}} p_{\text{max}}$$

### Resampling & Convergence
- KLD-sampling bounds the active particle count between 500 (converged) and 2000 (high uncertainty).
- Covariance matrix $\Sigma_{3 \times 3}$ is published to `/amcl_pose`.

## Failure Modes & Test Scenarios
1. **Poor Initial Pose**: Particle cloud initialized far from true location; converges within 3 spin recoveries.
2. **Kidnapped Robot**: Robot manually teleported; recovery manager detects high covariance and triggers dynamic global particle expansion.
