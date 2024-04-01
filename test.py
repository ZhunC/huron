import numpy as np
from DCM import dcmPlanner as planner
import matplotlib.pyplot as plt

vrpSetpoints = np.array([[0, 1, 2], [0, 2.5, 5], [0, 0, 0], [0, 5, 10]])
f = 10
COM_initial = np.array([[0], [0], [5]])

testPlanner = planner(0.5, vrpSetpoints,COM_initial)

vrpTrajectory = testPlanner.vrpTrajBuilder()
dcmSetpoints = testPlanner.dcmSetpointBuilder()
dcmTrajectory = testPlanner.dcmTrajBuilder()
comSetpoints = testPlanner.comSetPointBuilder()
comTrajectory = testPlanner.comTrajBuilder()

# VRP trajectory test

# setpointsPlot = np.transpose(vrpSetpoints)
# time_set = setpointsPlot[:,0]
# values_set = setpointsPlot[:, 1:4]
# trajPlot = np.transpose(vrpTrajectory)
# time_traj = trajPlot[:,0]
# values_traj = trajPlot[:, 1:4]

# plt.plot(time_set, values_set, 'bo')
# plt.plot(time_traj, values_traj, 'r')
# plt.show()

# DCM setpoint test

# setpointsPlot = np.transpose(vrpSetpoints)
# time_set = setpointsPlot[:,0]
# values_set = setpointsPlot[:, 1:4]
# dcmSetpointPlot = np.transpose(dcmSetpoints)
# time_dcm_Set = dcmSetpointPlot[:,0]
# values_dcm_Set = dcmSetpointPlot[:, 1:4]

# plt.plot(time_set, values_set, 'bo')
# plt.plot(time_dcm_Set, values_dcm_Set, 'ro')
# plt.show()

# DCM trajectory test

dcmSetpointPlot = np.transpose(dcmSetpoints)
time_dcm_Set = dcmSetpointPlot[:,0]
values_dcm_Set = dcmSetpointPlot[:, 1:4]
trajPlot = np.transpose(dcmTrajectory)
time_dcm_traj = trajPlot[:,0]
values_dcm_traj = trajPlot[:, 1:4]

plt.plot(time_dcm_traj, values_dcm_traj, 'r')
plt.plot(time_dcm_Set, values_dcm_Set, 'bo')
plt.show()

# COM trajectory test
comSetpointPlot = np.transpose(comSetpoints)
time_com_Set = comSetpointPlot[:,0]
values_com_Set = comSetpointPlot[:, 1:4]
trajPlot = np.transpose(comTrajectory)
time_com_traj = trajPlot[:,0]
values_com_traj = trajPlot[:, 1:4]

plt.plot(time_com_traj, values_com_traj, 'r')
plt.plot(time_com_Set, values_com_Set, 'bo')
plt.show()
