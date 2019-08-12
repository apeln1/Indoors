#!/usr/bin/env python
import numpy as np
import Astar
import copy
import math

m_to_cm = 100

class LocalMissionPlanner:
    def __init__(self, env_limits, tf_prefix, res, curpos, grid, curyaw):
        # Uses A* algorithm (Astar function)
        self.env_limits = np.multiply(env_limits, m_to_cm)
        self.tf_prefix = tf_prefix
        self.res = np.multiply(res, m_to_cm)
        self.curpos = np.multiply(curpos, m_to_cm)
        self.nextpos = copy.deepcopy(curpos)
        self.takeoffpos = copy.deepcopy(curpos)
        self.grid = copy.deepcopy(grid)
        self.curyaw = curyaw
        self.nextyaw = curyaw
        self.traj = []

    def TaskAssignment(self,dronepos, grid, droneyaw, traj):

        # Parameters for local mission planner
        self.curpos = np.multiply(dronepos, m_to_cm)
        traj = np.multiply(traj, m_to_cm)
        try:
            self.nextpos = [traj[0]]
        except:
            self.nextpos = dronepos
        self.curyaw = droneyaw
        self.traj = traj
        self.grid = grid

        self.MaintainSearchState()
        self.nextyaw = self.RotationAngleManager()

    def RotationAngleManager(self):
        angle = math.atan((self.nextpos[0][1] - self.curpos[0][1]) / (self.nextpos[0][0] - self.curpos[0][0]))
        if math.isnan(angle):
            angle = 0
        return angle

    def MaintainSearchState(self):

        Astar_Movement = Astar.build_trj(self.curpos, self.env_limits, self.res, self.grid, self.nextpos, self.tf_prefix)

        if Astar_Movement:
            self.traj = np.concatenate((Astar_Movement, self.traj), axis=0)
            print('Final traj')
            print(self.traj)
            print('Final traj in [m]')
            print(self.traj)
        self.traj = np.divide(self.traj, m_to_cm)

    def ReturnToHome(self):

        self.nextpos = self.takeoffpos
        Astar_Movement = Astar.build_trj(self.curpos, self.env_limits, self.res, self.grid, self.nextpos, self.tf_prefix)

        if Astar_Movement == []:
            Astar_Movement = [self.curpos[0], self.nextpos[0]]

        self.traj = np.divide(Astar_Movement, m_to_cm)

    def Land(self):
        pass

    def FindCommunication(self):
        pass

    def ExploreNewArea(self):
        pass

    def ImproveOdometry(self):
        pass


