# **************** main *******************
import matplotlib.animation as manimation
import numpy as np
import time
from EnvSim import Env
from Grid import Grid
from Drone import Drone
from Display import Display
from CentralPathPlanner import CentralPathPlanner
from WPmonitoring import WPmonitoring
from LocalMissionPlanner import LocalMissionPlanner
import copy

class DronePosGoal:
    """A simple class for holding drone position."""

    def __init__(self, pos=[], next_pos=[], trajectory=[], yaw=[]):
        self.pos = pos
        self.next_pos = next_pos
        self.trajectory = trajectory
        self.yaw = yaw

sleep_time = 0.001

num_of_agents = 3
# The number of steps in Central path planner
num_of_steps = 5
# Simulation counter
count_time_step = 0
# Each 'time_mult_path_find' iterations the Central compute trajectories
time_mult_path_find = 30
# The number of simulation iterations
num_of_iter = 1000
resolution = 10

env = Env(resolution, 2)
dict_of_drones_pos = dict()
grid = Grid(env.border_polygon_for_grid, resolution)
env_limits = grid.x_lim + grid.y_lim

# Allocations
agents = list()
drones = list()
localmissionplanner = list()
wpmonitoring = list()
drones_pos_list = list()
# Array of chosen nodes in the graph
nodesidxs_ij = list()

# Initiate all classes
for i in range(0, num_of_agents):
    drone_pos = [[-10000, -10000]]
    while (env.is_in_obs(drone_pos)) or not(env.is_in_border_polygon(drone_pos)):
        drone_pos = env.enterence + 30 * 2 * ([[0.5, 0.5]] - np.random.rand(1, 2))

    drones.append(Drone(i, drone_pos, 0, env))
    localmissionplanner.append(LocalMissionPlanner(env_limits, i, grid.res, drone_pos, grid.matrix, drones[i].current_heading))
    # agents.append(Agent(i, drone_pos, grid.res, grid.x_lim, grid.y_lim))
    drones_pos_list.append(list(drone_pos[0]))
    wpmonitoring.append(WPmonitoring(i, grid.x_lim, grid.y_lim, grid.res, drone_pos))

display = Display(env.border_polygon, env.obs_array, grid.x_lim, grid.y_lim, grid.res, grid.matrix, drones_pos_list)
centralpathplanner = CentralPathPlanner(num_of_agents, num_of_steps, grid.x_lim, grid.y_lim, grid.res, grid.matrix)

# Initiate the dictionary of all drones
for i in range(0, num_of_agents):
    dict_of_drones_pos[i] = DronePosGoal(pos=drones[i].pos, next_pos=drones[i].pos, trajectory=[], yaw=drones[i].current_heading)

movie_flag = False
if not movie_flag:
    for t in range(1, num_of_iter):
        count_time_step += 1
        if np.mod(count_time_step, time_mult_path_find) == 0 or t == 2:
            # Extract global trajectories using Central path planner
            dict_of_drones_pos, nodesidxs_ij = centralpathplanner.FindGlobalTrajectories(dict_of_drones_pos, grid.matrix)

        for i in range(0, num_of_agents):

            tof_list = drones[i].tof_sensing()
            grid.update_from_tof_sensing_list(tof_list)
            drones[i].preform_step(drones)

            # Local mission planner
            # start = time.time()
            localmissionplanner[i].TaskAssignment(dict_of_drones_pos, drones[i].pos, grid.matrix,
                                                   drones[i].current_heading, dict_of_drones_pos[i].trajectory)
            # end = time.time()
            # localtimeinterval = (end - start)*1000 # msec

            # Update parameters of drones dictionary
            dict_of_drones_pos[i].yaw = copy.deepcopy(localmissionplanner[i].nextyaw)
            dict_of_drones_pos[i].trajectory = copy.deepcopy(localmissionplanner[i].traj)

            # Chose the next WP (avoiding obstacles) and update the trajectory
            # start = time.time()
            wpmonitoring[i].FollowWP(drones[i].pos, drones[i].current_heading, grid.matrix, dict_of_drones_pos[i].trajectory, dict_of_drones_pos[i].yaw)
            # end = time.time()
            # WPtimeinterval = (end - start)*1000 # msec

            # Update parameters of drones dictionary
            dict_of_drones_pos[i].pos = copy.deepcopy(wpmonitoring[i].current_pos)
            dict_of_drones_pos[i].next_pos = copy.deepcopy(wpmonitoring[i].next_pos)
            dict_of_drones_pos[i].trajectory = copy.deepcopy(wpmonitoring[i].DroneTrj)

            drones[i].update_virtual_targets(wpmonitoring[i].next_pos, wpmonitoring[i].next_heading)
            display.plot_step(wpmonitoring[i].next_pos, grid.empty_idxs, grid.wall_idxs, drones[i].pos, i, nodesidxs_ij)

        display.fig.canvas.draw()
        time.sleep(sleep_time)

else: # Creating a movie
    FFMpegWriter = manimation.writers['ffmpeg']
    metadata = dict(title='Movie Test', artist='Matplotlib',
                    comment='Movie support!')
    writer = FFMpegWriter(fps=10, metadata=metadata)
    with writer.saving(display.fig, "writer_test.mp4", 100):

            writer.grab_frame()

# Example of euler transformation from quaternion
# from tf.transformations import euler_from_quaternion
#
# self.tfBuffer = tf2_ros.Buffer()
# self.listener = tf2_ros.TransformListener(self.tfBuffer)
#
# trans = self.tfBuffer.lookup_transform('world', tf_prefix, rospy.Time(0))
# q = (trans.transform.rotation.x,
#      trans.transform.rotation.y,
#      trans.transform.rotation.z,
#      trans.transform.rotation.w)
#
# euler = euler_from_quaternion(q, axes='sxyz')
#
# x = trans.transform.translation.x
# y = trans.transform.translation.y
# z = trans.transform.translation.z
# roll = euler[0]
# pitch = euler[1]
# yaw = euler[2]
#
# pos = [x * m_to_cm, y * m_to_cm, z * m_to_cm, roll, pitch, yaw]
