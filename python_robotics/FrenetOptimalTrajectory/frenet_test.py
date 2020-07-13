import frenet_optimal_trajectory
try:
    from quintic_polynomials_planner import QuinticPolynomial
    import cubic_spline_planner
except ImportError:
    raise
# way points
wx = [0.0, 10.0, 20.5, 35.0, 70.5]
wy = [0.0, -6.0, 5.0, 6.5, 0.0]
tx, ty, tyaw, tc, csp = frenet_optimal_trajectory.generate_target_course(wx, wy)

# initial state
c_speed = 10.0 / 3.6  # current speed [m/s]
c_d = 2.0  # current lateral position [m]
c_d_d = 0.0  # current lateral speed [m/s]
c_d_dd = 0.0  # current lateral acceleration [m/s]
s0 = 0.0  # current course position
path_params = {
    'K_J' : 0.1,
    'K_T' : 0.1,
    'K_D' : 1.0,
    'K_LAT' : 1.0,
    'K_LON' : 1.0,
    'DT' : 0.2,
    'TARGET_SPEED': 30.0 / 3.6
  }

area = 20.0  # animation area length [m]
path = frenet_optimal_trajectory.frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, 4.0, -7, 0, 6.94, path_params)
print(path.d)