import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def testcase_trajectory(t):

    # omega is used for current drone state, while yaw and yaw_dot is used for desired state
    #         0  1  2  3   4   5   6    7    8    9     10    11    12     13     14     15       16       17       18   19
    # state: [x, y, z, x', y', z', x'', y'', z'', x''', y''', z''', x'''', y'''', z'''',omega.x, omega.y, omega.z, yaw, yaw_dot]

    w = 0.5
    t = t
    x = 5*np.sin(w*t)
    y = t
    z = t
    yaw = 0.0
    yaw_dot = 0.0
    x_dot = 5*w*np.cos(t)
    y_dot = 1.0
    z_dot = 1.0
    x_dot_dot = -5*w*np.sin(t)
    y_dot_dot = 0.0
    z_dot_dot = 0.0
    x_dot_dot_dot = -5*w*np.cos(t)
    y_dot_dot_dot = 0.0
    z_dot_dot_dot = 0.0
    x_dot_dot_dot_dot = 5*w*np.sin(t)
    y_dot_dot_dot_dot = 0
    z_dot_dot_dot_dot = 0

    return [x, y, z,
            x_dot, y_dot, z_dot,
            x_dot_dot, y_dot_dot, z_dot_dot,
            x_dot_dot_dot, y_dot_dot_dot, z_dot_dot_dot,
            x_dot_dot_dot_dot, y_dot_dot_dot_dot, z_dot_dot_dot_dot,
            0.0, 0.0, 0.0,
            yaw, yaw_dot]




def testcase_trajectory2(t):
    R = 2
    theta = t/2.0
    x = R*np.cos(theta)
    y = R*np.sin(theta)
    z = t/10 + 2
    x_dot = -R*np.sin(theta)
    y_dot = R*np.cos(theta)
    z_dot = 1/10
    x_dot_dot = -R*np.cos(theta)
    y_dot_dot = -R*np.sin(theta)
    z_dot_dot = 0
    x_dot_dot_dot = R*np.sin(theta)
    y_dot_dot_dot = -R*np.cos(theta)
    z_dot_dot_dot = 0
    x_dot_dot_dot_dot = R*np.cos(theta)
    y_dot_dot_dot_dot = R*np.sin(theta)
    z_dot_dot_dot_dot = 0
    yaw = 0
    yaw_dot = 0

    return [x, y, z,
            x_dot, y_dot, z_dot,
            x_dot_dot, y_dot_dot, z_dot_dot,
            x_dot_dot_dot, y_dot_dot_dot, z_dot_dot_dot,
            x_dot_dot_dot_dot, y_dot_dot_dot_dot, z_dot_dot_dot_dot,
            0.0, 0.0, 0.0,
            yaw, yaw_dot]


if __name__ == '__main__':

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    xs = []
    ys = []
    zs = []
    for i in range(1000):
        t = i/10.0

        # replace with your own
        state = testcase_trajectory2(t)

        x = state[0]
        y = state[1]
        z = state[2]

        xs.append(x)
        ys.append(y)
        zs.append(z)

    ax.scatter(xs, ys, zs, c='r', marker='o')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()
