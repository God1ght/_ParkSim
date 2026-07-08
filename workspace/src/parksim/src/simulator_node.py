#!/usr/bin/env python3
import rclpy

import subprocess
import signal
import numpy as np

from pathlib import Path
import glob
import os

import traceback

import pickle

from dlp.dataset import Dataset
from dlp.visualizer import Visualizer as DlpVisualizer

from std_msgs.msg import Int16MultiArray, Bool, Float32
from parksim.msg import VehicleStateMsg
from parksim.srv import OccupancySrv
from parksim.base_node import MPClabNode, parksim_path
from parksim.pytypes import VehicleState, NodeParamTemplate

class SimulatorNodeParams(NodeParamTemplate):
    """
    template that stores all parameters needed for the node as well as default values
    """
    def __init__(self):
        self.dlp_path = parksim_path('python', 'parksim', 'priorFiles', 'data', 'DJI_0012')
        self.timer_period = 0.1
        self.random_seed = 0

        self.blocked_spots = []

        self.spawn_entering = 3
        self.spawn_exiting = 3
        self.y_bound_to_resume_spawning = 70
        self.spawn_interval_mean = 5 # (s)

        self.spots_data_path = ''
        self.agents_data_path = parksim_path('python', 'parksim', 'priorFiles', 'agents_data_0012.pickle')

        self.use_existing_agents = True
        self.background_mode = 'replay'  # replay, rule_random, or mixed

        self.spawn_qwen_ego = False
        self.qwen_ego_spawn_time = 0.5
        self.qwen_ego_spot_index = 1
        self.qwen_endpoint = ''
        self.qwen_model = 'Qwen2.5-VL-7B-Instruct'
        self.qwen_timeout = 15.0
        self.qwen_decision_period = 3.0
        self.qwen_max_candidate_spots = 8
        self.qwen_periodic_replan = False

        self.write_log = True
        self.log_path = parksim_path('vehicle_log')

class SimulatorNode(MPClabNode):
    """
    Node class for simulation
    """
    def __init__(self):
        super().__init__('simulator')
        self.get_logger().info('Initializing Simulator Node')
        namespace = self.get_namespace()

        param_template = SimulatorNodeParams()
        self.autodeclare_parameters(param_template, namespace)
        self.autoload_parameters(param_template, namespace)

        np.random.seed(self.random_seed)

        # Check whether there are unterminated vehicle processes
        all_nodes_names = [x[0] for x in self.get_node_names_and_namespaces()]
        print(all_nodes_names)
        if 'vehicle' in all_nodes_names:
            self.get_logger().error("Some vehicle nodes are not shut down cleanly. Please kill those processes first.")
            raise KeyboardInterrupt()

        # Clean up the log folder if needed
        if self.write_log:
            # yccc7: path changed
            # log_dir_path = str(Path.home()) + self.log_path
            log_dir_path = self.log_path
            if not os.path.exists(log_dir_path):
                os.mkdir(log_dir_path)
            log_files = glob.glob(log_dir_path+'/*.log')
            for f in log_files:
                os.remove(f)
            self.get_logger().info("Logs will be saved in %s. Old logs are cleared." % log_dir_path)

        # DLP
        home_path = str(Path.home())
        self.get_logger().info('Loading Dataset...')
        ds = Dataset()
        # ds.load(home_path + self.dlp_path)
        # yccc7: path changed
        ds.load(self.dlp_path)
        self.dlpvis = DlpVisualizer(ds)

        # Parking Spaces
        self.parking_spaces, self.occupied = self._gen_occupancy()
        for idx in self.blocked_spots:
            self.occupied[idx] = True

        # Agents
        self._gen_agents()

        # Spawning
        # 生成智能体，生成时间服从指数分布
        self.spawn_entering_time = list(np.random.exponential(self.spawn_interval_mean, self.spawn_entering))

        self.spawn_exiting_time = list(np.random.exponential(self.spawn_interval_mean, self.spawn_exiting))

        self.last_enter_id = None
        self.last_enter_sub = None
        self.last_enter_state = VehicleState()
        self.keep_spawn_entering = True

        self.start_time = self.get_ros_time()

        self.last_enter_time = self.start_time
        self.last_exit_time = self.start_time

        self.vehicles = []
        self.num_vehicles = 0
        self.qwen_ego_spawned = False

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Publish the simulation time
        self.sim_time_pub = self.create_publisher(Float32, '/sim_time', 10)

        # Visualizer publish this status since the button is on GUI
        self.sim_status_sub = self.create_subscription(Bool, '/sim_status', self.sim_status_cb, 10)
        self.sim_is_running = True

        self.occupancy_pub = self.create_publisher(Int16MultiArray, 'occupancy', 10)

        self.occupancy_srv = self.create_service(OccupancySrv, 'occupancy', self.occupancy_srv_callback)

        self.occupancy_cli = self.create_client(OccupancySrv, '/occupancy')

    def sim_status_cb(self, msg: Bool):
        self.sim_is_running = msg.data

    def occupancy_srv_callback(self, request, response):
        vehicle_id = request.vehicle_id
        idx = request.idx
        new_value = request.new_value

        self.occupied[idx] = new_value

        response.status = True

        self.get_logger().info("Vehicle %d changed the occupancy at %d to be %r" % (vehicle_id, idx, new_value))
        
        return response

    def _gen_occupancy(self):
        # get parking spaces
        arr = self.dlpvis.parking_spaces.to_numpy()
        # array of tuples of x-y coords of centers of spots
        parking_spaces = np.array([[round((arr[i][2] + arr[i][4]) / 2, 3), round((arr[i][3] + arr[i][9]) / 2, 3)] for i in range(len(arr))])

        scene = self.dlpvis.dataset.get('scene', self.dlpvis.dataset.list_scenes()[0])

        # figure out which parking spaces are occupied
        car_coords = [self.dlpvis.dataset.get('obstacle', o)['coords'] for o in scene['obstacles']]
        # 1D array of booleans — are the centers of any of the cars contained within this spot's boundaries?
        occupied = [any([c[0] > arr[i][2] and c[0] < arr[i][4] and c[1] < arr[i][3] and c[1] > arr[i][9] for c in car_coords]) for i in range(len(arr))]

        return parking_spaces, list(map(int, occupied))

    def _gen_agents(self):
        # home_path = str(Path.home())
        # with open(home_path + self.agents_data_path, 'rb') as f:
        #     self.agents_dict = pickle.load(f)

        # yccc7: path changed
        with open(self.agents_data_path, 'rb') as f:
            self.agents_dict = pickle.load(f)

    def add_vehicle(self, spot_index: int, agent_type: str = 'rule_based'):

        self.num_vehicles += 1

        command = [
            "ros2", "launch", "parksim", "vehicle.launch.py",
            "vehicle_id:=%d" % self.num_vehicles,
            "spot_index:=%d" % spot_index,
            "agent_type:=%s" % agent_type,
        ]
        if agent_type == 'qwen_vla':
            command.extend([
                "qwen_endpoint:=%s" % self.qwen_endpoint,
                "qwen_model:=%s" % self.qwen_model,
                "qwen_timeout:=%s" % self.qwen_timeout,
                "qwen_decision_period:=%s" % self.qwen_decision_period,
                "qwen_max_candidate_spots:=%s" % self.qwen_max_candidate_spots,
                "qwen_periodic_replan:=%s" % str(self.qwen_periodic_replan).lower(),
            ])

        self.vehicles.append(
            subprocess.Popen(command, start_new_session=True)
        )

        self.get_logger().info("A %s vehicle with id = %d is added with spot_index = %d" % (agent_type, self.num_vehicles, spot_index))
    def add_existing_vehicle(self, vehicle_id: int):

        self.num_vehicles += 1

        self.vehicles.append(
            subprocess.Popen(
                ["ros2", "launch", "parksim", "vehicle.launch.py", "vehicle_id:=%d" % vehicle_id, "spot_index:=%d" % 0],
                start_new_session=True,
            )
        )

        self.get_logger().info("An existing vehicle with id = %d is added" % vehicle_id)
    def shutdown_vehicles(self):
        for vehicle in self.vehicles:
            if vehicle.poll() is not None:
                continue

            try:
                vehicle.send_signal(signal.SIGINT)
            except ProcessLookupError:
                continue

        for vehicle in self.vehicles:
            if vehicle.poll() is not None:
                continue

            try:
                vehicle.wait(timeout=5)
                continue
            except subprocess.TimeoutExpired:
                pass

            try:
                os.killpg(vehicle.pid, signal.SIGTERM)
            except ProcessLookupError:
                continue

            try:
                vehicle.wait(timeout=2)
                continue
            except subprocess.TimeoutExpired:
                pass

            try:
                os.killpg(vehicle.pid, signal.SIGKILL)
            except ProcessLookupError:
                continue
            vehicle.wait(timeout=2)

        print("Vehicle nodes are down")

    def last_enter_cb(self, msg):
        self.unpack_msg(msg, self.last_enter_state)

        # If vehicle left entrance area, start spawning another one
        if self.last_enter_state.x.y < self.y_bound_to_resume_spawning:
            self.keep_spawn_entering = True
            self.get_logger().info("Vehicle %d left the entrance area." % self.last_enter_id)

    def try_spawn_entering(self):
        current_time = self.get_ros_time()

        if self.spawn_entering_time and current_time - self.last_enter_time > self.spawn_entering_time[0]:
            empty_spots = [i for i in range(len(self.occupied)) if not self.occupied[i]]
            chosen_spot = np.random.choice(empty_spots)
            self.add_vehicle(chosen_spot) # pick from empty spots randomly
            self.occupied[chosen_spot] = True
            self.spawn_entering_time.pop(0)

            self.last_enter_time = current_time
            self.last_enter_id = self.num_vehicles
            self.last_enter_sub = self.create_subscription(VehicleStateMsg, '/vehicle_%d/state' % self.last_enter_id, self.last_enter_cb, 10)
            self.keep_spawn_entering = False

    def try_spawn_exiting(self):
        current_time = self.get_ros_time()

        if self.spawn_exiting_time and current_time - self.last_exit_time > self.spawn_exiting_time[0]:
            departable_spots = [i for i in range(1, len(self.occupied)) if self.occupied[i] and i not in self.blocked_spots]
            if not departable_spots:
                return
            chosen_spot = int(np.random.choice(departable_spots))
            self.add_vehicle(-1 * chosen_spot)
            self.spawn_exiting_time.pop(0)

            self.last_exit_time = current_time

    def try_spawn_existing(self):
        current_time = self.get_ros_time() - self.start_time
        added_vehicles = []

        for agent in self.agents_dict:
            if self.agents_dict[agent]["init_time"] < current_time:
                self.add_existing_vehicle(agent)
                added_vehicles.append(agent)

        for added in added_vehicles:
            del self.agents_dict[added]

    def try_spawn_qwen_ego(self):
        current_time = self.get_ros_time() - self.start_time
        if self.spawn_qwen_ego and not self.qwen_ego_spawned and current_time >= self.qwen_ego_spawn_time:
            self.add_vehicle(int(self.qwen_ego_spot_index), agent_type='qwen_vla')
            self.qwen_ego_spawned = True

    def timer_callback(self):

        if self.sim_is_running:
            self.try_spawn_qwen_ego()
            mode = str(self.background_mode).lower()
            if mode not in ['replay', 'rule_random', 'mixed']:
                mode = 'replay' if self.use_existing_agents else 'rule_random'
            if mode in ['rule_random', 'mixed']:
                if self.keep_spawn_entering:
                    if self.last_enter_sub:
                        self.destroy_subscription(self.last_enter_sub)
                        self.last_enter_sub = None

                    self.try_spawn_entering()

                self.try_spawn_exiting()
            if mode in ['replay', 'mixed']:
                self.try_spawn_existing()

        # Publish current simulation time
        time_msg = Float32()
        time_msg.data = self.get_ros_time() - self.start_time
        self.sim_time_pub.publish(time_msg)

        occupancy_msg = Int16MultiArray()
        occupancy_msg.data = self.occupied
        self.occupancy_pub.publish(occupancy_msg)


def main(args=None):
    rclpy.init(args=args)

    simulator = SimulatorNode()

    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        print('Simulation is terminated')
    except:
        print('Unknown exception')
        traceback.print_exc()
    finally:
        simulator.shutdown_vehicles()
        simulator.destroy_node()
        print('Simulation stopped cleanly')

        rclpy.shutdown()

if __name__ == "__main__":
    main()
