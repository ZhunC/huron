import numpy as np

"""
Implementation of a unified bipedal motion planner using the Divergent Component of Motion.
Paper: George Mesesan et al., Unified Motion Planner for Walking, Running, and Jumping Using the Three-Dimensional Diveregent Component of Motion,
IEEE Transactions on Robotics, Dec.2023

"""
class dcmPlanner():
    """
    --> Input:
        del_z : a double representing delta z in the paper.
        com_initial: a 3-by-n nparray specifiying the initial COM positions in [x; y; z]. It has to be a column vector. 
        vrpSetpoints: a 4-by-n nparray where each column vector is in format of: [t; x; y; z]. Indexed from 0-3 for rows and 0 to n-1 for columns
            interpolateFreq: an integer specifying how many interpolation points will be made between consecutive setpoints specified above
        finalDCM: a 4-by-1 nparray indicating the Final DCM endpoint, in format of [t; x; y; z]. The default value is the last VRP setpoint.
        interpolateFreq: an integer representing desired number of intermetidate points between setpoints (for all VRP, DCM and COM). Default at 10.
    
    """
    def __init__(self, del_z, vrps, com_initial, finalDCM = None, interpolateFreq = 10):
        self.del_z = del_z
        self.b = np.sqrt(del_z/9.81)
        self.vrpSetpoints = vrps
        self.f = interpolateFreq
        self.T = vrps[0, 2] - vrps[0, 1] 
        _, self.n = vrps.shape

        self.com_initial = np.zeros([4,1])
        self.com_initial[1:4, 0] = com_initial[:,0]

        if finalDCM == None:
            self.finalDCM = vrps[:, -1]   # If final DCM is not provided, it is set to equal to the VRP at the last moment
        else:
            self.finalDCM = finalDCM



    def vrpTrajBuilder(self):
        """
        Original paper Eq.10, page 4
            
        --> Output:
        vrpTrajectory: a 4-by-((f+1)n-f) nparray where each column vector is in format of: [t; x; y; z]. Number of columns is determined by the following reasoning:
            setpoint 1, f interpolations(f in short), setpoint 2, f, setpoint 3, f, ..., f, setpoint n. 
            Total number of sequence: 1 + (f+1) + (f+1) +...+ (f+1) = 1 + (f+1)(n-1) = (f+1)n - f
        """
        vrpSetpoints = self.vrpSetpoints
        n = self.n
        f = self.f
        T = self.T

        vrpTrajectory = np.zeros([4, (f+1)*n-f])
        vrpTrajectory[:,0] = vrpSetpoints[:, 0]

        for i in range(1, n):
            vrpTrajectory[:, i*(f+1)] = vrpSetpoints[:, i]
            t_init = vrpTrajectory[0, (i-1)*(f+1)]
            t_final = vrpTrajectory[0, i*(f+1)]
            vrpTrajectory[0, (i-1)*(f+1) : i*(f+1)+1] = np.linspace(t_init, t_final, f+2)
            nu_init = vrpTrajectory[1:4, (i-1)*(f+1)]
            nu_final = vrpTrajectory[1:4, i*(f+1)]
            for j in range(1, f+1):
                t = vrpTrajectory[0, (i-1)*(f+1)+j] - t_init
                vrpTrajectory[1:4, (i-1)*(f+1)+j] = (1-t/T) * nu_init + t/T * nu_final

        self.vrpTraj =  vrpTrajectory
        return vrpTrajectory

    def dcmSetpointBuilder(self):
        """
        Original paer Eq. 12, page 4

        --> Output:
        dcmSetpoints: a 4-by-n nparray where each column is the DCM setpoint at the moment of vrp setpoints. Starting at t=0 and ending with zeta_final
        """
        n = self.n
        b = self.b
        T = self.T
        zeta_final = self.finalDCM
        vrpSetpoints = self.vrpSetpoints

        dcmSetpoints = np.zeros([4, n])
        dcmSetpoints[0, :] = vrpSetpoints[0, :]
        dcmSetpoints[:, -1] = zeta_final
        frac = b/T

        for i in range(-n, -1):
            dcmSetpoints[1:4, i] = (1 - frac + frac*np.exp(-1/frac)) * vrpSetpoints[1:4, i]\
                                    + (frac - (frac+1)*np.exp(-1/frac)) * vrpSetpoints[1:4, i+1]\
                                    + np.exp(-1/frac) * dcmSetpoints[1:4, i+1]
        
        self.dcmSetpoints = dcmSetpoints
        return dcmSetpoints

    def dcmTrajBuilder(self):
        """
        Original paper Eq. 11, page 4

        --> Output:
        dcmTrajectory: a 4-by-((f+1)n-f) nparray where each column vector is in format of: [t; x; y; z].
        """
        vrpTrajectory = self.vrpTraj
        dcmSetpoints = self.dcmSetpoints
        n = self.n
        f = self.f
        T = self.T
        b = self.b

        dcmTrajectory = np.zeros([4, (f+1)*n-f])
        dcmTrajectory[:,0] = dcmSetpoints[:, 0]

        for i in range(1, n):
            dcmTrajectory[:, i*(f+1)] = dcmSetpoints[:, i]
            t_init = vrpTrajectory[0, (i-1)*(f+1)]
            t_final = vrpTrajectory[0, i*(f+1)]
            dcmTrajectory[0, (i-1)*(f+1) : i*(f+1)+1] = np.linspace(t_init, t_final, f+2)
            nu_init = vrpTrajectory[1:4, (i-1)*(f+1)]
            nu_final = vrpTrajectory[1:4, i*(f+1)]
            zeta_final = dcmTrajectory[1:4, i*(f+1)]
            for j in range(1, f+1):
                t = dcmTrajectory[0, (i-1)*(f+1)+j] - t_init
                dcmTrajectory[1:4, (i-1)*(f+1)+j] = (1 - t/T + b/T + b/t*np.exp(t/b - T/b))  * nu_init \
                + (t/T + b/T - (b/T+1)*np.exp(t/b-T/b)) * nu_final \
                + np.exp(t/b - T/b) * zeta_final                    
        self.dcmTraj =  dcmTrajectory
        return dcmTrajectory

    def comSetPointBuilder(self):
        """
        Original paper Eq. 14, page 4

        --> Output:
        comSetpoints: a 4-by-n nparray where each column is the DCM setpoint at the moment of vrp setpoints. Starting at t=0 with COM initial
        """
        n = self.n
        b = self.b
        T = self.T
        x0 = self.com_initial
        vrpSetpoints = self.vrpSetpoints
        dcmSetpoints = self.dcmSetpoints

        comSetpoints = np.zeros([4, n])
        comSetpoints[0, :] = vrpSetpoints[0, :]
        comSetpoints[:, 0] = x0[:, 0]

        for i in range(1, n):
            comSetpoints[1:4, i] = (b/T * np.exp(-T/b) * np.sinh(T/b) - np.exp(-T/b)) * vrpSetpoints[1:4, i-1] \
                                    + (1 - (b/T+1) * np.exp(-T/b) * np.sinh(T/b)) * vrpSetpoints[1:4, i] \
                                    + np.exp(-T/b) * np.sinh(T/b) * dcmSetpoints[1:4, i] \
                                    + np.exp(-T/b) * comSetpoints[1:4, i-1]
                                    
        
        self.comSetpoints = comSetpoints
        return comSetpoints

    def comTrajBuilder(self):
        """
        Original paper EQ. 13, page 4

        --> Output:
        dcmTrajectory: a 4-by-((f+1)n-f) nparray where each column vector is in format of: [t; x; y; z].
        """
        vrpTrajectory = self.vrpTraj
        dcmTrajectory = self.dcmTraj
        comSetpoints = self.comSetpoints
        n = self.n
        f = self.f
        T = self.T
        b = self.b

        comTrajectory = np.zeros([4, (f+1)*n-f])
        comTrajectory[:,0] = comSetpoints[:, 0]

        expTerm = np.exp(-T/b)
        hypbolTerm = np.sinh(T/b)

        for i in range(1, n):
            comTrajectory[:, i*(f+1)] = comSetpoints[:, i]
            t_init = vrpTrajectory[0, (i-1)*(f+1)]
            t_final = vrpTrajectory[0, i*(f+1)]
            comTrajectory[0, (i-1)*(f+1) : i*(f+1)+1] = np.linspace(t_init, t_final, f+2)
            nu_init = vrpTrajectory[1:4, (i-1)*(f+1)]
            nu_final = vrpTrajectory[1:4, i*(f+1)]
            zeta_final = dcmTrajectory[1:4, i*(f+1)]
            x0 = comTrajectory[1:4, (i-1)*(f+1)]
            for j in range(1, f+1):
                t = comTrajectory[0, (i-1)*(f+1)+j] - t_init
                comTrajectory[1:4, (i-1)*(f+1)+j] = (b/T * expTerm * hypbolTerm - expTerm)  * nu_init \
                + (1 - (b/T + 1) * expTerm * hypbolTerm) * nu_final \
                + expTerm * hypbolTerm * zeta_final \
                + expTerm * x0        
        self.comTraj =  comTrajectory
        return comTrajectory
