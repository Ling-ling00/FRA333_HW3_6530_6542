# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.ธฤต_ุ6530
2.พรวลัย_6542
'''
import FRA333_HW3_6530_6542
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from math import pi

#test by compare to robotics toolbox
print('test by compare to robotics toolbox \n')
d_1 = 0.0892
a_2 = -0.425
a_3 = -0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d = d_1 , offset=pi),
        rtb.RevoluteMDH(alpha=pi/2),
        rtb.RevoluteMDH(a=a_2),
    ],tool = SE3(a_3-d_6,-d_5,d_4) @ SE3.Ry(-pi/2), name="RRR robot")

q = [0, 0, 0]

#compare forward kinematics from robotics toolbox and HW3_utils
print('compare forward kinematics')
R,P,R_e,p_e = FRA333_HW3_6530_6542.FKHW3(q)

print(f'robotics toolbox rotation matrix for q = {np.around(q, 2)}')
print(robot.fkine(q).R)
print(f'HW3_utils rotation matrix for q = {np.around(q, 2)}')
print(R_e)
print(f'diferencerial between robotics toolbox and HW3_utils is {np.linalg.norm(robot.fkine(q).R - R_e)} \n')

print(f'robotics toolbox position vector for q = {np.around(q, 2)}')
print(robot.fkine(q).t)
print(f'HW3_utils position vector for q = {np.around(q, 2)}')
print(p_e)
print(f'diferencerial between robotics toolbox and HW3_utils is {np.linalg.norm(robot.fkine(q).t - p_e)} \n')

#===========================================<ตรวจคำตอบข้อ 1>====================================================#
print('===========================================part1===========================================')

q = np.random.uniform(-pi, pi, 3)
J_toolbox = robot.jacobe(q)
J_custom = FRA333_HW3_6530_6542.endEffectorJacobianHW3(q)

print(f'robotics toolbox jacobian matrix for q = {np.around(q, 2)}')
print(J_toolbox)

print(f'HW3 jacobian matrix for q = {np.around(q, 2)}')
print(J_custom)

print(f'diferencerial between robotics toolbox and HW3 is {np.linalg.norm(J_custom - J_toolbox)} \n')
#==============================================================================================================#

#===========================================<ตรวจคำตอบข้อ 2>====================================================#
print('===========================================part2===========================================')

q = np.random.uniform(-pi, pi, 3)
flag_toolbox0 = abs(np.linalg.det(robot.jacob0(q)[:3])) < 0.001 #check from reduce jacobian ref 0 from robotics toolbox
flag_toolboxe = abs(np.linalg.det(robot.jacobe(q)[:3])) < 0.001 #check from reduce jacobian ref e from robotics toolbox
flag_custom = FRA333_HW3_6530_6542.checkSingularityHW3(q)
print(f'check singularity from reduce jacobian ref 0 from robotics toolbox for q = {np.around(q, 2)} is {flag_toolbox0}')
print(f'check singularity from reduce jacobian ref e from robotics toolbox for q = {np.around(q, 2)} is {flag_toolboxe}')
print(f'check singularity from HW3 for q = {np.around(q, 2)} is {flag_custom} \n')

q = [0,0,-0.19]
flag_toolbox0 = abs(np.linalg.det(robot.jacob0(q)[:3])) < 0.001 #check from reduce jacobian ref 0 from robotics toolbox
flag_toolboxe = abs(np.linalg.det(robot.jacobe(q)[:3])) < 0.001 #check from reduce jacobian ref e from robotics toolbox
flag_custom = FRA333_HW3_6530_6542.checkSingularityHW3(q)
print(f'check singularity from reduce jacobian ref 0 from robotics toolbox for q = {np.around(q, 2)} is {flag_toolbox0}')
print(f'check singularity from reduce jacobian ref e from robotics toolbox for q = {np.around(q, 2)} is {flag_toolboxe}')
print(f'check singularity from HW3 for q = {np.around(q, 2)} is {flag_custom} \n')
#==============================================================================================================#

#===========================================<ตรวจคำตอบข้อ 3>====================================================#
print('===========================================part3===========================================')

q = np.random.uniform(-pi, pi, 3)
w_f_m = np.random.uniform(-100, 100, 6) #w = [force, moment] for robotics toolbox 
w_m_f = [0, 0, 0, 0, 0, 0]
w_m_f[3:], w_m_f[:3] = w_f_m[:3], w_f_m[3:] #w = [moment, force] for hw3
tau_toolbox = robot.pay(w_f_m , q,frame=1) * -1 #from robotics toolbox robot.pay is -J*w -> *-1 = J*w
tau_custom = FRA333_HW3_6530_6542.computeEffortHW3(q, w_m_f)

print(f'robotics toolbox tau for q = {np.around(q, 2)} and w(moment, force) = {np.around(w_m_f,2)}')
print(tau_toolbox)
print(f'HW3 tau for q = {np.around(q, 2)} and w(moment, force) = {np.around(w_m_f, 2)}')
print(tau_custom)
print(f'diferencerial between robotics toolbox and HW3 is {np.linalg.norm(tau_custom - tau_toolbox)}')
#==============================================================================================================#