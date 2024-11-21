#!/usr/bin/env python3

import sys
import os
import numpy as np
import heapq
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from PIL import Image, ImageOps
import yaml
import matplotlib.pyplot as plt
from copy import copy
import pandas as pd
from graphviz import Graph
from geometry_msgs.msg import Twist, PoseStamped
from math import atan2, sqrt, pow, pi
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
from geometry_msgs.msg import PointStamped


class Queue():
    def __init__(self, init_queue = []):
        self.queue = copy(init_queue)
        self.start = 0
        self.end = len(self.queue)-1

    def __len__(self):
        numel = len(self.queue)
        return numel

    def __repr__(self):
        q = self.queue
        tmpstr = ""
        for i in range(len(self.queue)):
            flag = False
            if(i == self.start):
                tmpstr += "<"
                flag = True
            if(i == self.end):
                tmpstr += ">"
                flag = True

            if(flag):
                tmpstr += '| ' + str(q[i]) + '|\n'
            else:
                tmpstr += ' | ' + str(q[i]) + '|\n'

        return tmpstr

    def __call__(self):
        return self.queue

    def initialize_queue(self,init_queue = []):
        self.queue = copy(init_queue)

    def sort(self,key=str.lower):
        self.queue = sorted(self.queue,key=key)

    def push(self,data):
        self.queue.append(data)
        self.end += 1

    def pop(self):
        p = self.queue.pop(self.start)
        self.end = len(self.queue)-1
        return p

class Map:
    def __init__(self, map_name):
        self.map_im, self.map_df, self.limits = self.__open_map(map_name)
        if self.map_im is not None:
            self.image_array = self.__get_obstacle_map(self.map_im, self.map_df)
        else:
            self.image_array = None

    def __open_map(self, map_name):
        package_share_path = get_package_share_directory('task_7')
        yaml_path = os.path.join(package_share_path, 'maps', f"{map_name}.yaml")
        try:
            with open(yaml_path, 'r') as f:
                map_df = pd.json_normalize(yaml.safe_load(f))
        except FileNotFoundError:
            print(f"File {yaml_path} not found.")
            return None, None, None

        map_image_path = os.path.join(package_share_path, 'maps', map_df.image[0])
        try:
            im = Image.open(map_image_path)
        except FileNotFoundError:
            print(f"File {map_image_path} not found.")
            return None, None, None
        size = 200, 200
        im.thumbnail(size)
        im = ImageOps.grayscale(im)
        
        xmin = map_df.origin[0][0]
        xmax = map_df.origin[0][0] + im.size[0] * map_df.resolution[0]
        ymin = map_df.origin[0][1]
        ymax = map_df.origin[0][1] + im.size[1] * map_df.resolution[0]

        return im, map_df, [xmin, xmax, ymin, ymax]

    def __get_obstacle_map(self, map_im, map_df):
        img_array = np.array(map_im)
        obstacle_thresh = map_df.occupied_thresh[0] * 255
        binary_map = np.where(img_array <= obstacle_thresh, 1, 0) 

        return binary_map

    def world_to_pixel(self, x, y):
        scale_x = 200 / 14.7 
        scale_y = 139 / 10.36  
    
       
        pixel_x = int((x + 5.3) * scale_x)  
        pixel_y = int((4.2 - y) * scale_y)  
    
        # Clamping
        # pixel_x = max(0, min(pixel_x, self.image_array.shape[1] - 1))
        # pixel_y = max(0, min(pixel_y, self.image_array.shape[0] - 1))
    
        return pixel_x, pixel_y

    def pixel_to_world(self, x, y):
        scale_x = 200 / 14.7
        scale_y = 139 / 10.36
    
        # Convert pixel coordinates back to world coordinates
        world_x = (x / scale_x) - 5.3  
        world_y = 4.2 - (y / scale_y)  
    
        return world_x, world_y




class GraphNode():
    def __init__(self,name):
        self.name = name
        self.children = []
        self.weight = []

    def __repr__(self):
        return self.name

    def add_children(self,node,w=None):
        if w == None:
            w = [1]*len(node)
        self.children.extend(node)
        self.weight.extend(w)

class Tree():
    def __init__(self,name):
        self.name = name
        self.root = 0
        self.end = 0
        self.g = {}
        self.g_visual = Graph('G')

    def __call__(self):
        for name,node in self.g.items():
            if(self.root == name):
                self.g_visual.node(name,name,color='red')
            elif(self.end == name):
                self.g_visual.node(name,name,color='blue')
            else:
                self.g_visual.node(name,name)
            for i in range(len(node.children)):
                c = node.children[i]
                w = node.weight[i]
             
                if w == 0:
                    self.g_visual.edge(name,c.name)
                else:
                    self.g_visual.edge(name,c.name,label=str(w))
        return self.g_visual

    def add_node(self, node, start = False, end = False):
        self.g[node.name] = node
        if(start):
            self.root = node.name
        elif(end):
            self.end = node.name

    def set_as_root(self,node):
        # These are exclusive conditions
        self.root = True
        self.end = False

    def set_as_end(self,node):
        # These are exclusive conditions
        self.root = False
        self.end = True


class AStar():
    def __init__(self, in_tree):
        self.in_tree = in_tree
        self.q = Queue()
        self.dist = {name: np.inf for name in in_tree.g.keys()}
        self.h = {name: 0 for name in in_tree.g.keys()}
        self.via = {name: None for name in in_tree.g.keys()}


        for name in in_tree.g.keys():
            start = tuple(map(int, name.split(',')))
            end = tuple(map(int, in_tree.end.split(',')))
            self.h[name] = np.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)


        self.dist[in_tree.root] = 0
        self.q.push((0 + self.h[in_tree.root], in_tree.root))

    def __get_f_score(self, node):
        return self.dist[node] + self.h[node]

    def solve(self, sn, en):
        while len(self.q) > 0:

            self.q.sort(key=lambda x: x[0])
            current_f_score, current_name = self.q.pop()
            current = self.in_tree.g[current_name]


            if current_name == en.name:
                return self.reconstruct_path(sn, en)

            for i in range(len(current.children)):
                neighbor = current.children[i]
                weight = current.weight[i]
                tentative_dist = self.dist[current_name] + weight

                if tentative_dist < self.dist[neighbor.name]:
                    self.via[neighbor.name] = current_name
                    self.dist[neighbor.name] = tentative_dist

                    f_score = tentative_dist + self.h[neighbor.name]
                    self.q.push((f_score, neighbor.name))

        return None

    def reconstruct_path(self, sn, en):
        path = []
        current = en.name
        total_dist = self.dist[en.name]


        while current is not None:
            path.append(current)
            current = self.via[current]

        path.reverse()
        return path, total_dist

    def reconstruct_path(self, sn, en):
        path = []
        current = en.name
        total_dist = self.dist[en.name]

        while current is not None:
            path.append(current)
            current = self.via[current]

        path.reverse()
        return path, total_dist

class MapProcessor():
    def __init__(self,name):
        self.map = Map(name)
        self.inf_map_img_array = np.zeros(self.map.image_array.shape)
        self.map_graph = Tree(name)

    def __modify_map_pixel(self,map_array,i,j,value,absolute):
        if( (i >= 0) and
            (i < map_array.shape[0]) and
            (j >= 0) and
            (j < map_array.shape[1]) ):
            if absolute:
                map_array[i][j] = value
            else:
                map_array[i][j] += value

    def __inflate_obstacle(self,kernel,map_array,i,j,absolute):
        dx = int(kernel.shape[0]//2)
        dy = int(kernel.shape[1]//2)
        if (dx == 0) and (dy == 0):
            self.__modify_map_pixel(map_array,i,j,kernel[0][0],absolute)
        else:
            for k in range(i-dx,i+dx):
                for l in range(j-dy,j+dy):
                    self.__modify_map_pixel(map_array,k,l,kernel[k-i+dx][l-j+dy],absolute)

    def inflate_map(self,kernel,absolute=True):
        # Perform an operation like dilation, such that the small wall found during the mapping process
        # are increased in size, thus forcing a safer path.
        self.inf_map_img_array = np.zeros(self.map.image_array.shape)
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.map.image_array[i][j] == 1:
                    self.__inflate_obstacle(kernel,self.inf_map_img_array,i,j,absolute)
        r = np.max(self.inf_map_img_array)-np.min(self.inf_map_img_array)
        if r == 0:
            r = 1
        self.inf_map_img_array = (self.inf_map_img_array - np.min(self.inf_map_img_array))/r

    def get_graph_from_map(self):
        # Create the nodes that will be part of the graph, considering only valid nodes or the free space
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.inf_map_img_array[i][j] == 0:
                    node = GraphNode('%d,%d'%(i,j))
                    self.map_graph.add_node(node)
        # Connect the nodes through edges
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.inf_map_img_array[i][j] == 0:
                    if (i > 0):
                        if self.inf_map_img_array[i-1][j] == 0:
                            # add an edge up
                            child_up = self.map_graph.g['%d,%d'%(i-1,j)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up],[1])
                    if (i < (self.map.image_array.shape[0] - 1)):
                        if self.inf_map_img_array[i+1][j] == 0:
                            # add an edge down
                            child_dw = self.map_graph.g['%d,%d'%(i+1,j)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw],[1])
                    if (j > 0):
                        if self.inf_map_img_array[i][j-1] == 0:
                            # add an edge to the left
                            child_lf = self.map_graph.g['%d,%d'%(i,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_lf],[1])
                    if (j < (self.map.image_array.shape[1] - 1)):
                        if self.inf_map_img_array[i][j+1] == 0:
                            # add an edge to the right
                            child_rg = self.map_graph.g['%d,%d'%(i,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_rg],[1])
                    if ((i > 0) and (j > 0)):
                        if self.inf_map_img_array[i-1][j-1] == 0:
                            # add an edge up-left
                            child_up_lf = self.map_graph.g['%d,%d'%(i-1,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up_lf],[np.sqrt(2)])
                    if ((i > 0) and (j < (self.map.image_array.shape[1] - 1))):
                        if self.inf_map_img_array[i-1][j+1] == 0:
                            # add an edge up-right
                            child_up_rg = self.map_graph.g['%d,%d'%(i-1,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up_rg],[np.sqrt(2)])
                    if ((i < (self.map.image_array.shape[0] - 1)) and (j > 0)):
                        if self.inf_map_img_array[i+1][j-1] == 0:
                            # add an edge down-left
                            child_dw_lf = self.map_graph.g['%d,%d'%(i+1,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw_lf],[np.sqrt(2)])
                    if ((i < (self.map.image_array.shape[0] - 1)) and (j < (self.map.image_array.shape[1] - 1))):
                        if self.inf_map_img_array[i+1][j+1] == 0:
                            # add an edge down-right
                            child_dw_rg = self.map_graph.g['%d,%d'%(i+1,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw_rg],[np.sqrt(2)])


        print(f"Total nodes added: {len(self.map_graph.g)}")


    def gaussian_kernel(self, size, sigma=1):
        size = int(size) // 2
        x, y = np.mgrid[-size:size+1, -size:size+1]
        normal = 1 / (2.0 * np.pi * sigma**2)
        g =  np.exp(-((x**2 + y**2) / (2.0*sigma**2))) * normal
        r = np.max(g)-np.min(g)
        sm = (g - np.min(g))*1/r
        return sm

    def rect_kernel(self, size, value):
        m = np.ones(shape=(size,size))
        return m




class Navigation(Node):
    def __init__(self, node_name='Navigation'):
        super().__init__(node_name)
        self.map_processor = MapProcessor('sync_classroom_map')
        kernel = np.ones((10, 10))  # Define a basic kernel for obstacle inflation
        self.map_processor.inflate_map(kernel)

        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()

        self.path=[]
        self.current_path_index=0
        self.previous_angle_error = 0.0
        self.last_time = self.get_clock().now()

        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        # self.create_subscription(PointStamped, '/clicked_point', self.__goal_pose_cbk, 10)
        
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.__ttbot_pose_cbk, 10)

    def __goal_pose_cbk(self, data):
        self.goal_pose = data
        self.get_logger().info(f"Received goal pose: ({self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y})")
        self.compute_path()

    def __ttbot_pose_cbk(self, data):
        self.ttbot_pose.pose = data.pose.pose
        self.follow_path()

    def compute_path(self):
        # Convert start and goal points to pixel coordinates
        start_x, start_y = self.map_processor.map.world_to_pixel(
            self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y)
        end_x, end_y = self.map_processor.map.world_to_pixel(
            self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)

        
        self.get_logger().info(f"Start pixel: ({start_x},{start_y}), End pixel: ({end_x},{end_y})")

        # Generate the graph from the inflated map
        self.map_processor.get_graph_from_map()
        start_key, end_key = f"{start_y},{start_x}", f"{end_y},{end_x}"

        # Check if start and end nodes are in the graph
        if start_key not in self.map_processor.map_graph.g:
            self.get_logger().error(f"Start node {start_key} is not in free space or graph.")
            self.plot_path_on_map(start_x, start_y, end_x, end_y, None)  # Call plot to show the issue
            return
        if end_key not in self.map_processor.map_graph.g:
            self.get_logger().error(f"End node {end_key} is not in free space or graph.")
            self.plot_path_on_map(start_x, start_y, end_x, end_y, None)  # Call plot to show the issue
            return

        # Set start and end nodes for A* search
        self.map_processor.map_graph.root = start_key
        self.map_processor.map_graph.end = end_key


        astar = AStar(self.map_processor.map_graph)
        result = astar.solve(self.map_processor.map_graph.g[start_key], self.map_processor.map_graph.g[end_key])
      

        if result:
            path, _ = result
            self.publish_path(path)
            # self.plot_path_on_map(start_x, start_y, end_x, end_y, path)
        else:
            self.get_logger().info("No path found by A* algorithm.")
            # self.plot_path_on_map(start_x, start_y, end_x, end_y, None)  # Call plot to show the issue

    def plot_path_on_map(self, start_x, start_y, end_x, end_y, path):
        # Display the inflated map as background
        plt.imshow(self.map_processor.inf_map_img_array, cmap='viridis')
        plt.colorbar()
        plt.title("Path Planning Visualization")

        # Plot start and goal points
        plt.plot(start_x, start_y, 'go', label="Start")  
        plt.plot(end_x, end_y, 'ro', label="Goal")  
        # Plot the A* path if available
        if path:
            path_x = []
            path_y = []
            for node in path:
                x, y = map(int, node.split(','))
                path_x.append(x)
                path_y.append(y)
            plt.plot(path_y, path_x, 'b-', linewidth=2, label="Planned Path")  
            plt.text(0.5, 0.5, 'No Path Found', ha='center', va='center', transform=plt.gca().transAxes, color='red')

        plt.legend()
        plt.show()

    def publish_path(self, path):
        nav_path = Path()
        nav_path.header.stamp = self.get_clock().now().to_msg()
        nav_path.header.frame_id = "map"

        for node in path:
            y, x = map(int, node.split(','))
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x, pose.pose.position.y = self.map_processor.map.pixel_to_world(x, y)
            pose.pose.orientation.w = 1.0
            nav_path.poses.append(pose)

        self.path_pub.publish(nav_path)
        self.get_logger().info("Published A* path.")
        self.current_path_index=0
        self.path=nav_path.poses
        self.follow_path()
        
    
    def follow_path(self):
        if hasattr(self, 'path') and self.current_path_index < len(self.path):
            current_goal = self.path[self.current_path_index]
            error_x = current_goal.pose.position.x - self.ttbot_pose.pose.position.x
            error_y = current_goal.pose.position.y - self.ttbot_pose.pose.position.y

            distance = sqrt(pow(error_x, 2) + pow(error_y, 2))
            if distance < 0.25:  #Threshold so that it doesnt overcprrect
                self.current_path_index += 1
                if self.current_path_index >= len(self.path):
                    self.get_logger().info("Reached the end of the path.")
                    vel_msg = Twist()
                    self.vel_pub.publish(vel_msg)  # Stop 
                    return

            # Calculating.... the angle to the goal
            angle_to_goal = atan2(error_y, error_x)
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            # Calculate PD
            angle_error = self.normalize_angle(angle_to_goal - self.get_yaw_from_quaternion(self.ttbot_pose.pose.orientation))
            d_angle_error = (angle_error - self.previous_angle_error) / dt if dt > 0 else 0
            self.previous_angle_error = angle_error

            # PD coefficients
            kp = 1.5
            kd = 0.5

            # PD control for angular velocity
            angular_velocity = kp * angle_error + kd * d_angle_error
            angular_velocity = max(-0.5, min(0.5, angular_velocity))  

            # Reduce linear speed when the angle error is large
            linear_velocity = 0.5 if abs(angle_error) < 0.1 else 0.05

            vel_msg = Twist()
            vel_msg.linear.x = linear_velocity
            vel_msg.angular.z = angular_velocity
            self.vel_pub.publish(vel_msg)

    def normalize_angle(self, angle):
        """Normalize the angle to be within -pi to pi"""
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def get_yaw_from_quaternion(self, q):
        """Convert quaternion to yaw"""
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

def main(args=None):
    rclpy.init(args=args)
    nav = Navigation()
    rclpy.spin(nav)
    nav.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main() 




