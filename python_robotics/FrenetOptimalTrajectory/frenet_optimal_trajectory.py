"""

Frenet optimal trajectory generator

author: Atsushi Sakai (@Atsushi_twi)

Ref:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame]
(https://www.youtube.com/watch?v=Cj6tAQe7UCY)

"""

import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../QuinticPolynomialsPlanner/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../CubicSpline/")

try:
    from quintic_polynomials_planner import QuinticPolynomial
    import cubic_spline_planner
except ImportError:
    raise

SIM_LOOP = 500

show_animation = True


class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class FrenetPath:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []

# void FrenetPath::calc_lat_paths(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0, double Ti, double di, double di_d)
def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0, Ti, di, di_d, tv, path_params):
    # # Parameter
    DT = path_params['DT']  # time tick [s]
    TARGET_SPEED = path_params['TARGET_SPEED']

    # cost weights
    K_J = path_params['K_J']
    K_T = path_params['K_T']
    K_D = path_params['K_D']
    K_LAT = path_params['K_LAT']
    K_LON = path_params['K_LON']

    # Four parameters are sampled Ti, di, di_d, tv
    fp = FrenetPath()

    # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
    lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, di_d, 0.0, Ti)

    fp.t = [t for t in np.arange(0.0, Ti, DT)]
    fp.d = [lat_qp.calc_point(t) for t in fp.t]
    fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
    fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
    fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

    # Longitudinal motion planning (Velocity keeping)
    lon_qp = QuarticPolynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

    fp.s = [lon_qp.calc_point(t) for t in fp.t]
    fp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
    fp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
    fp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

    Jp = sum(np.power(fp.d_ddd, 2))  # square of jerk
    Js = sum(np.power(fp.s_ddd, 2))  # square of jerk

    # square of diff from target speed
    ds = (TARGET_SPEED - fp.s_d[-1]) ** 2
    fp.cd = K_J * Jp + K_T * Ti + K_D * fp.d[-1] ** 2
    fp.cv = K_J * Js + K_T * Ti + K_D * ds
    fp.cf = K_LAT * fp.cd + K_LON * fp.cv
    return fp


def calc_global_paths(fp, csp):
    # calc global positions
    for i in range(len(fp.s)):
        ix, iy = csp.calc_position(fp.s[i])
        if ix is None:
            break
        i_yaw = csp.calc_yaw(fp.s[i])
        di = fp.d[i]
        fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
        fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
        fp.x.append(fx)
        fp.y.append(fy)

    # calc yaw and ds
    for i in range(len(fp.x) - 1):
        dx = fp.x[i + 1] - fp.x[i]
        dy = fp.y[i + 1] - fp.y[i]
        fp.yaw.append(math.atan2(dy, dx))
        fp.ds.append(math.hypot(dx, dy))

    fp.yaw.append(fp.yaw[-1])
    fp.ds.append(fp.ds[-1])

    # calc curvature
    for i in range(len(fp.yaw) - 1):
        fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fp


def check_collision(fp, ob):
    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
             for (ix, iy) in zip(fp.x, fp.y)]

        collision = any([di <= ROBOT_RADIUS ** 2 for di in d])

        if collision:
            return False

    return True


def check_paths(fplist, ob):
    ok_ind = []
    for i, _ in enumerate(fplist):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            continue
        elif any([abs(a) > MAX_ACCEL for a in
                  fplist[i].s_dd]):  # Max accel check
            continue
        elif any([abs(c) > MAX_CURVATURE for c in
                  fplist[i].c]):  # Max curvature check
            continue
        elif not check_collision(fplist[i], ob):
            continue

        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, Ti, di, di_d, tv, path_params):
    #Initial conditions from step of env get csp(from reset), s0, c_speed,...
    path = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0, Ti, di, di_d, tv, path_params)
    path = calc_global_paths(path, csp)

    return path


def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp


def main():
    print(__file__ + " start!!")

    # way points
    wx = [0.0, 10.0, 20.5, 35.0, 70.5]
    wy = [0.0, -6.0, 5.0, 6.5, 0.0]
    # obstacle lists
    ob = np.array([[20.0, 10.0],
                   [30.0, 6.0],
                   [30.0, 8.0],
                   [35.0, 8.0],
                   [50.0, 3.0]
                   ])

    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)

    # initial state
    c_speed = 10.0 / 3.6  # current speed [m/s]
    c_d = 2.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position

    area = 20.0  # animation area length [m]

    for i in range(SIM_LOOP):
        path = frenet_optimal_planning(
            csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)
        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]

        if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(tx, ty)
            plt.plot(ob[:, 0], ob[:, 1], "xk")
            plt.plot(path.x[1:], path.y[1:], "-or")
            plt.plot(path.x[1], path.y[1], "vc")
            plt.xlim(path.x[1] - area, path.x[1] + area)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
            plt.grid(True)
            plt.pause(0.0001)

    print("Finish")
    if show_animation:  # pragma: no cover
        plt.grid(True)
        plt.pause(0.0001)
        plt.show()


if __name__ == '__main__':
    main()
