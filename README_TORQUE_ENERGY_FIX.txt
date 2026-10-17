Torque Energy Metric Fix

The benchmark now reports drive-wheel mechanical energy in joules instead of tau^2 integrated over time.
For each simulation step:
  wheel angular speed = (v +/- b*omega/2) / r
  mechanical power = |tau_l*omega_l + tau_r*omega_r|
  energy += mechanical power * dt

The dashboard displays three decimal places so small but non-zero energy values are not shown as 0.0.
