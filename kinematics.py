"""!
Implements Forward and Inverse kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""

import numpy as np
# expm is a matrix exponential function
from scipy.linalg import expm


def clamp(angle):
    """!
    @brief      Clamp angles between (-pi, pi]

    @param      angle  The angle

    @return     Clamped angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle


def FK_dh(dh_params, joint_angles, link):
    """!
    @brief      Get the 4x4 transformation matrix from link to world

                TODO: implement this function

                Calculate forward kinematics for rexarm using DH convention

                return a transformation matrix representing the pose of the desired link

                note: phi is the euler angle about the y-axis in the base frame

    @param      dh_params     The dh parameters as a 2D list each row represents a link and has the format [a, alpha, d,
                              theta]
    @param      joint_angles  The joint angles of the links
    @param      link          The link to transform from

    @return     a transformation matrix representing the pose of the desired link
    """
    pass


def get_transform_from_dh(a, alpha, d, theta):
    """!
    @brief      Gets the transformation matrix from dh parameters.

    TODO: Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transform matrix.
    """
    pass


def get_euler_angles_from_T(T):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO: Implement this function return the Euler angles from a T matrix

    @param      T     transformation matrix

    @return     The euler angles from T.
    """
    pass


def get_pose_from_T(T):
    """!
    @brief      Gets the pose from T.

                TODO: implement this function return the joint pose from a T matrix of the form (x,y,z,psi,theta,phi) where phi is
                rotation about base frame z-axis, theta is the rotation about the y-acis, and psi is the rotation about the x-axis.

    @param      T     4x4 transformation matrix

    @return     The pose from T of the form (x,y,z,psi,theta,phi).
    """
    x_pos = T[0][3]
    y_pos = T[1][3]
    z_pos = T[2][3]
    phi = 0.0
    rot = np.array([T[0][0:3], T[1][0:3], T[2][0:3]])

    theta = clamp(np.arcsin(-rot[2,0]))

    psi = clamp(np.arcsin(rot[2,1]/np.cos(theta)))

    phi = clamp(np.arcsin(rot[1,0]/np.cos(theta)))
    
    pose = np.array([x_pos,y_pos, z_pos, psi, theta, phi])
    return pose


def FK_pox(joint_angles, m_mat, s_lst):
    """!
    @brief      Get a 4-tuple (x, y, z, phi) representing the pose of the desired link

                TODO: implement this function, Calculate forward kinematics for rexarm using product of exponential
                formulation return a 4-tuple (x, y, z, phi) representing the pose of the desired link note: phi is the euler
                angle about y in the base frame

    @param      joint_angles  The joint angles
                m_mat         The M matrix
                s_lst         List of screw vectors

    @return     a 4-tuple (x, y, z, phi) representing the pose of the desired link
    """
    e1 = to_s_matrix(s_lst[0][3:6], s_lst[0][0:3], joint_angles[0])
    e2 = to_s_matrix(s_lst[1][3:6], s_lst[1][0:3], joint_angles[1])
    e3 = to_s_matrix(s_lst[2][3:6], s_lst[2][0:3], joint_angles[2])
    e4 = to_s_matrix(s_lst[3][3:6], s_lst[3][0:3], joint_angles[3])
    e5 = to_s_matrix(s_lst[4][3:6], s_lst[4][0:3], joint_angles[4])

    e12 = np.matmul(e1, e2)
    e13 = np.matmul(e12, e3)
    e14 = np.matmul(e13, e4)
    e15 = np.matmul(e14, e5)
    print('FK result = ', np.matmul(e15, m_mat))

    return np.matmul(e15, m_mat)


def to_s_matrix(w, v, th):
    """!
    @brief      Convert to matrix exponential.

    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)

    @param      w     axis of rotation as np.array x,y,z
    @param      v     top half of screw vector as np.array x,y,z
    @param      th    joint angle value

    @return     exponential screw matrix for the POX method e^([s]*theta)
    """
    sk = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    exp = expm(sk*th)

    w_vec = np.atleast_2d(w).T
    w_t = w_vec.T
    v_vec = np.atleast_2d(v).T

    wxv = np.atleast_2d(np.cross(w, v)).T
    wwtv = np.matmul(w_vec, np.matmul(w_t, v_vec*th))
    pos = np.matmul((np.identity(3) - exp), wxv) + wwtv

    res = np.block([
        [exp, pos],
        [0, 0, 0, 1]
    ])
    return res

def IK_pox(pose):
    # TODO: Throw an error if unreachable
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose as np.array (x,y,z,psi,theta,phi) to joint angles

    @param      pose        The desired pose as np.array (x,y,z,psi,theta,phi)

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """

    xi1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    xi2 = np.array([0.0, -103.91, 0.0, -1.0, 0.0, 0.0])
    xi3 = np.array([0.0, 303.91, -50.0, 1.0, 0.0, 0.0])
    xi4 = np.array([0.0, 303.91, -250.0, 1.0, 0.0, 0.0])
    xi5 = np.array([-303.91, 0.0, 0.0, 0.0, 1.0, 0.0])
    s_lst = np.array([xi1, xi2, xi3, xi4, xi5])
    m_mat = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 424.15], [0.0, 0.0, 1.0, 303.91], [0.0, 0.0, 0.0, 1]])

    gd = pose_to_T(pose)
    gi = np.linalg.inv(m_mat)
    g1 = np.matmul(gd, gi)

    x0 = m_mat[0][3]
    y0 = m_mat[1][3]
    z0 = m_mat[2][3]

    # Th1 can be solved with geometry
    th1 = np.arctan2(pose[1],pose[0]) - np.arctan2(y0,x0)
    th1 = clamp(th1)

    # e2.e3.e4.e5 = e-1.gd.gst-1 = g2
    # e2.e3.p4 = g2.p4
    # e2.e3.p4 - p2 = g2.p4 - p2
    # e2.||e3.p4 - p2|| = ||g2.p4 - p2||
    e1 = to_s_matrix(s_lst[0][3:6], s_lst[0][0:3], th1)
    e1i = np.linalg.inv(e1)
    g2 = np.matmul(e1i, g1)
    
    p4 = np.array([0.0, 250.0, 303.91, 1.0])
    p4 = np.atleast_2d(p4).T

    g2p4 = np.matmul(g2,p4)

    p2 = np.array([0.0, 0.0, 103.91, 1.0])
    p2 = np.atleast_2d(p2).T

    w = s_lst[2][3:6]
    r = np.array([0, 50, 303.91])
    p = p4[0:3].T[0]
    q = p2[0:3].T[0]
    d = np.linalg.norm(g2p4 - p2)

    # e2.||e3.p4 - p2|| = ||g2.p4 - p2||
    # Th3 can be sovled with PK subproblem 3 
    th3 = PK3(w,r,p,q,d)
    
    if(np.abs(th3[0]) < np.abs(th3[1])):
        th3 = clamp(th3[0])
    else:
        th3 = clamp(th3[1])

    e3 = to_s_matrix(s_lst[2][3:6], s_lst[2][0:3], th3)
    
    e3i = np.linalg.inv(e3)

    e3p4 = np.matmul(e3, p4)

    # e2.(e3.p4) = g2.p4
    # Th2 can be solved with PK Subproblem 1
    w = s_lst[1][3:6]
    r = p2[0:3].T[0]
    p = e3p4[0:3].T[0]
    q = g2p4[0:3].T[0]
    th2 = PK1(w,r,p,q)
    
    e2 = to_s_matrix(s_lst[1][3:6], s_lst[1][0:3], th2)
    e2i = np.linalg.inv(e2)
    
    # e4.e5 = e-3.e-2.e-1.gd.gst-1 = g3
    # e4.p5 = g3.p5
    g3 = np.matmul(e3i, np.matmul(e2i, g2))
    
    # p1 = np.array([0.0, 0.0, 0.0, 1.0])
    # p1 = np.atleast_2d(p1).T

    # g3p1 = np.matmul(g3, p1)

    p5 = np.array([0.0, 0.0, 303.91, 1.0])
    p5 = np.atleast_2d(p5).T

    g3p5 = np.matmul(g3, p5)

    # Theta 4 can be solved with PK subproblem 1
    # w1 = s_lst[3][3:6]
    # w2 = s_lst[4][3:6]
    # r = p4[0:3].T[0]
    # p = p1[0:3].T[0]
    # q = g3p1[0:3].T[0]
    # print(w1, w2, r, p, q)
    # th45 = PK2(w1, w2, r, p, q)
    # print("th45 = ", th45)

    w = s_lst[3][3:6]
    r = p4[0:3].T[0]
    p = p5[0:3].T[0]
    q = g3p5[0:3].T[0]
    th4 = PK1(w,r,p,q)

    e4 = to_s_matrix(s_lst[3][3:6], s_lst[3][0:3], th4)
    e4i = np.linalg.inv(e4)

    # e5 = e-4.g3 = g4
    # e5.p1 = g4.p1
    g4 = np.matmul(e4i, g3)

    p1 = np.array([0.0, 0.0, 0.0, 1.0])
    p1 = np.atleast_2d(p1).T

    g4p1 = np.matmul(g4, p1)

    # Theta 5 can be solved with PK subproblem 1
    w = s_lst[4][3:6]
    r = p5[0:3].T[0]
    p = p1[0:3].T[0]
    q = g4p1[0:3].T[0]
    th5 = PK1(w,r,p,q)

    # if((abs(th45[0][0]) < 2.0 and abs(th45[0][1]) < 2.0)):
    #     th4 = th45[0][0]
    #     th5 = th45[0][1]
    # else:
    #     th4 = th45[1][0]
    #     th5 = th45[1][1]

    th_mat = np.array([th1, th2, th3, th4, th5])

    return th_mat

def PK3(w, r, p, q, d):
    """!
    @brief      Solve PK subproblem 3 for possible theta values

    @param      w       Rotation Axis in form np.array x,y,z
    @param      r       point on axis in form np.array x,y,z
    @param      p       Given Initial Point in form np.array x,y,z
    @param      q       Given Reference Point in form np.array x,y,z
    @param      d       Target distance between points

    @return     th      The two theta values chosen (prioritizing keeping the elbow up)
    """
    w_hor = np.atleast_2d(w)
    w_ver = np.atleast_2d(w).T

    pmq = np.atleast_2d(p - q).T

    # define vectors u and v
    u = p - r
    u = np.atleast_2d(u).T
    v = q - r
    v = np.atleast_2d(v).T

    # define projections of u and v onto axis
    up = u - np.matmul(np.matmul(w_ver, w_hor), u)
    vp = v - np.matmul(np.matmul(w_ver, w_hor), v)
    up_norm = np.linalg.norm(up)
    vp_norm = np.linalg.norm(vp)

    dp_sq = d**2 - np.absolute(np.matmul(w_hor, pmq))[0][0]**2
    
    th0 = np.arctan2(np.matmul(w_hor, np.cross(up.T, vp.T).T)[0][0], np.matmul(up.T, vp)[0][0])
    
    theta1 = th0 + np.arccos((up_norm**2 + vp_norm**2 - dp_sq)/(2 * up_norm * vp_norm))
    theta2 = th0 - np.arccos((up_norm**2 + vp_norm**2 - dp_sq)/(2 * up_norm * vp_norm))
    
    return np.array([theta1, theta2])

def PK2(w1, w2, r, p, q):
    """!
    @brief      Solve PK subproblem 2 for possible theta values

    @param      w1      Rotation Axis 1 in form np.array x,y,z
    @param      w2      Rotation Axis 2 in form np.array x,y,z
    @param      r       Intersection Point between the 2 axes in form np.array x,y,z
    @param      P       Given Initial Point in form np.array x,y,z
    @param      q       Given Desired Point in form np.array x,y,z

    @return     th      The two theta values chosen (prioritizing keeping the elbow up)
    """
    u = p - r
    u = np.atleast_2d(u).T
    v = q - r
    v = np.atleast_2d(v).T
    print("U:",u,"V:",v)

    w1_hor = np.atleast_2d(w1)
    w1_ver = w1_hor.T
    w2_hor = np.atleast_2d(w2)
    w2_ver = w2_hor.T

    w1Tw2 = np.matmul(w1_hor, w2_ver)
    print("dot",w1Tw2)
    alpha = (np.matmul(w1Tw2 , np.matmul(w2, u)) - np.matmul(w1_hor,v)) / (w1Tw2**2 - 1)
    alpha = alpha[0][0]

    beta = (np.matmul(w1Tw2 , np.matmul(w1, v)) - np.matmul(w2_hor,u)) / (w1Tw2**2 - 1)
    beta = beta[0][0]
    gamma_sq = np.linalg.norm(u)**2 - alpha**2 - beta**2 - 2*alpha*beta*np.matmul(w1_hor,w2_ver)[0][0]
    gamma_sq = gamma_sq / (np.linalg.norm(np.cross(w1,w2))**2)
    if(gamma_sq < 0):
        gamma_sq = 0

    gamma1 = np.sqrt(gamma_sq)
    gamma2 = -np.sqrt(gamma_sq)

    z1 = alpha*w1 + beta*w2 + gamma1*np.cross(w1,w2)
    z2 = alpha*w1 + beta*w2 + gamma2*np.cross(w1,w2)

    c1 = z1 + r
    c2 = z2 + r

    theta11 = PK1(w1, r, c1, q)
    theta12 = PK1(w2, r, p, c1)

    theta21 = PK1(w1, r, c2, q)
    theta22 = PK1(w2, r, p, c2)

    return np.array([[theta11, theta12],[theta21, theta22]])

    pass

def PK1(w, r, p, q):
    """!
    @brief      Solve PK subproblem 1 for theta value

    @param      w       Rotation Axis  in form np.array x,y,z
    @param      r       Given point on rotation axis in form of np.array(x,y,z)
    @param      P       Given Initial Point in form np.array x,y,z
    @param      q       Given Desired Point in form np.array x,y,z

    @return     th      The theta values chosen
    """
    w_hor = np.atleast_2d(w)
    w_ver = np.atleast_2d(w).T

    #create vectors u and v
    u = p - r
    u = np.atleast_2d(u).T
    v = q - r
    v = np.atleast_2d(v).T

    # define projections of u and v onto axis
    up = u - np.matmul(np.matmul(w_ver, w_hor), u)
    vp = v - np.matmul(np.matmul(w_ver, w_hor), v)
   
    theta = np.arctan2(np.matmul(w_hor,np.cross(up.T, vp.T).T), np.matmul(up.T, vp))
    
    return theta[0][0]

def pose_to_T(pose):
    """!
    @brief      Compute the corresponding transformation matrix from given pose

    @param      pose    The pose of the end-effector in the form np.array (x,y,z,psi,theta,phi)

    @return     T       4x4 transformation matrix representing the pose
    """
    pos = pose[0:3]
    pos = np.atleast_2d(pos).T

    psi = pose[3]
    theta = pose[4]
    phi = pose[5]

    z = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])*phi
    y = np.array([[0, 0, 1], [0, 0, 0], [-1, 0, 0]])*theta
    x = np.array([[0, 0, 0], [0, 0, -1], [0, 1, 0]])*psi

    Rz = expm(z)
    Ry = expm(y)
    Rx = expm(x)

    R_tot = np.matmul(Rz,Ry)
    R_tot = np.matmul(R_tot,Rx)
    
    res = np.block([
        [R_tot, pos],
        [0, 0, 0, 1]
    ])
    return res

def IK_geometric(dh_params, pose):
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose as np.array x,y,z,phi to joint angles

    @param      dh_params  The dh parameters
    @param      pose       The desired pose as np.array x,y,z,phi

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """
    pass