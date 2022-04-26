
import matplotlib.pyplot as plt
import numpy as np
import scipy.special

def calc_bezier_path(control_points, n_points=100):
    """
    Compute bezier path (trajectory) given control points.

    :param control_points: (numpy array)
    :param n_points: (int) number of points in the trajectory
    :return: (numpy array)
    """
    traj = []
    for t in np.linspace(0, 1, n_points):
        traj.append(bezier(t, control_points))

    return np.array(traj)


def bernstein_poly(n, i, t):
    """
    Bernstein polynom.

    :param n: (int) polynom degree
    :param i: (int)
    :param t: (float)
    :return: (float)
    """
    return scipy.special.comb(n, i) * t ** i * (1 - t) ** (n - i)


def bezier(t, control_points):
    """
    Return one point on the bezier curve.

    :param t: (float) number in [0, 1]
    :param control_points: (numpy array)
    :return: (numpy array) Coordinates of the point
    """
    n = len(control_points) - 1
    return np.sum([bernstein_poly(n, i, t) * control_points[i] for i in range(n + 1)], axis=0)


def main():
    """贝塞尔曲线生成器"""

    """运行之后，将out.txt文件中的内容拷贝到程序中即可"""

    PATH_NAME = 'path_0'            # 生成路径的名称
    PATH_NUM_NAME = 'PATH_POINT_0'  # 生成路径点数量宏的名称

    PATH_POINT_NUM = 100             # 路径点的数量
    MAX_PLAN_V = 0.6                # 最高直线速度

    ACC_POINT_NUM = 20             # 路径点的数量

    control_points = np.array([     # 控制点
            [0., 0.],
            [0., 2000.], 
            [0., 3000.], 
            [1000., 4000.], 
            [2000., 4000.], 
            [4000., 4000.],
            [10000., 4000.]
      ])

    path = calc_bezier_path(control_points, n_points=PATH_POINT_NUM)

    f = open('out_contain.txt','w')

    f.write( '#define ' + 'MAX_PLAN_V' + ' '+ str(MAX_PLAN_V) + '\n')

    f.write( '#define ' + PATH_NUM_NAME + ' '+ str(PATH_POINT_NUM) + '\n')

    f.write( 'pose_t ' + PATH_NAME + '[' +PATH_NUM_NAME+ ']' + '=' + '\n')

    f.write( '{' + '\n')

    for i in range(PATH_POINT_NUM):
        point_x = format(path.T[0][i],'.3f')
        point_y = format(path.T[1][i],'.3f')
        theta = 0.0

        if i >=0 and i<ACC_POINT_NUM:
            v_scale = format(i*(1/ACC_POINT_NUM),'.2f')
        elif (PATH_POINT_NUM-i)>=0 and (PATH_POINT_NUM-i)<=ACC_POINT_NUM:
            v_scale = format((PATH_POINT_NUM-i-1)*(1/ACC_POINT_NUM),'.2f')
        else:
            v_scale = format(1.0,'.2f')

        v_plan = 'f* MAX_PLAN_V'
        f.write('    ' + '{' + str(point_x)+'f' + ' , ' + str(point_y)+'f' + ' , ' + str(theta)+'f' + ' , ' + str(v_scale) + v_plan + '}' + ','+ '\n')

    f.write( '};' + '\n')

    f.close()

    ''' 绘制曲线 '''
    fig, ax = plt.subplots()
    ax.plot(path.T[0], path.T[1], label="Bezier Path")
    ax.plot(control_points.T[0], control_points.T[1],
            '--o', label="Control Points")
    ax.legend()
    ax.axis("equal")
    ax.grid(True)
    plt.show()


if __name__ == '__main__':
    main()
