import queue
import numpy as np
import typing 
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import seaborn as sns
import pandas as pd
from dataclasses import dataclass
import math
import time
import heapq
from collections import OrderedDict
import random
import heapq
from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
import multiprocessing as mp 

# class Graph_Algorithm():

#     def SubGraph(self):
#         SubLength =5
#         SubWidth=5
#         if (self.StereoGraph()==True and self.LidarGraph()==True):
#             pass
#         elif (self.StereoGraph()==True):
#             SafetyDistance=2
#             x,y=self.GetRobotCoords()
#             direction=45
#             SafetyDistanceY=round(math.sin(math.radians(direction))*SafetyDistance)
#             SafetyDistanceX=round(math.cos(math.radians(direction))*SafetyDistance)
#             if ((direction>45 and direction < 135) or (direction>225 and direction < 315) ):
#                pass
#                 #SubGraph=self.Graph[x-5:x+5,y+SafetyDistanceY:y+SafetyDistanceY+10]
#                 #print(SubGraph,"SubGraph Y")
#                 #use checkbounds
#             if (direction%45==0):
#                 self.SubGraph=self.Graph[y+SafetyDistanceY:(y+SafetyDistanceY+5),x+SafetyDistanceX:(x+SafetyDistanceX+5)]
#             else:
                
#                 #use checkbounds
#                 """
#                 SubGraph=self.Graph[x+SafetyDistanceX:(x+SafetyDistanceX+10)- ,y-5:y+5]
#                 #print(SubGraph, "SubGraph X")
#                 print(self.Graph[x:x+5,y:+5], "Hi there")"""
#             #Listen to Ros to Upload the Obstacles

#         elif (self.LidarGraph()==False):
#             pass
#     def UpdateRobot(self):
#         pass
        
#     def checkBounds(self,nearX,farEndX,nearY, farEndY):
#         if (nearX>self.GraphlengthX or nearY>self.GraphwidthY):
#             return [None, None]
#         elif (farEndX>self.GraphlengthX):
#             farEndX=farEndX-self.GraphlengthX
#         elif (farEndY>self.GraphwidthY):
#             farEndY=farEndY-self.GraphwidthY
#         return [farEndX,farEndY]








##### TODO TODO TODO TODO TODO


##### TODO NEED TO FIX, distribution of random blocks must be lower to allow a path.
##### TODO NEED TO FIX, path must dispaper from old trajectory. 

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import time

class Graphing_Mode(Enum):
    listener_state = 0
    configure_data_state=1
    setup_graph_state=2
    update_robot_state=3
    update_pathplanning_state=4
    publisher_state=5

class TrajectoryQueueTree():
    def __init__(self):
        self.MainBranch=[]

    def add_Branch(self, CurrentBranch, BranchNode):
        Branch=[]
        CurrentBranch[BranchNode]=Branch
    def add_Node(self,CurrentBranch, DataValue):
        if CurrentBranch==None:
            self.MainBranch.append(DataValue)
            return self.MainBranch
        CurrentBranch.append(DataValue) 
        return CurrentBranch 
    def get_Main(self):
        return self.MainBranch   
    def find_Node(self,Node):
        for index, Nodlet in enumerate(self.MainBranch,0):
            if (len(Nodlet)==2):
                if Nodlet[0]==Node[0] and Nodlet[1]==Node[1]:
                    return [index, Nodlet]
    def get_Node(self,CurrentBranch,index):
        if CurrentBranch == None:
            return self.MainBranch[index]
        #index = 
    def reappend_Nodes(self,Node,NewNode):
        index=self.find_Node(Node)[0]
        if (len(self.MainBranch[index])==2):
            self.MainBranch[index]=NewNode
class Graph_Algorithm:
    def __init__(self):
        self.GraphwidthY: int = 25
        self.GraphlengthX: int = 25
        self.ScaleCourse: int=25
        self.Scale: int =1
        self.RobotCoords  = [5, 5]
        self.ChargingStationCoords  = [self.GraphlengthX - 1, self.GraphwidthY - 1]
        self.Graph = np.zeros((self.GraphwidthY, self.GraphlengthX), dtype=float)
        self.AllObstacles=[]
        self.ObstaclesListLength=0
        self.PreviousLength=0
    def SetRobotCoords(self, x, y):
        self.Graph[self.RobotCoords[1], self.RobotCoords[0]] = 0
        self.RobotCoords[0]= x
        self.RobotCoords[1]= y
        self.Graph[y, x] = 10
    def GetRobotCoords(self):
        return [self.RobotCoords[0], self.RobotCoords[1]]
    def SetChargingStation(self, x, y):
        self.ChargingStationx=x
        self.ChargingStationy=y
        self.Graph[self.ChargingStationCoords[1], self.ChargingStationCoords[0]] = 0
        self.ChargingStationCoords[0], self.ChargingStationCoords[1] = x, y
        self.Graph[y, x] = 9
    def GetChargingStationCoords(self):
         return [self.ChargingStationCoords[0], self.ChargingStationCoords[1]]

    def SetTrajectory(self):
        self.Trajectory   =TrajectoryQueueTree()
        xChargingStation: float=15
        yChargingStation: float=24
        xRobot: float = 5
        yRobot: float = 5
        self.SetRobotCoords(xRobot, yRobot)
        DistanceX: float= (xChargingStation+1-xRobot)
        DistanceY: float= (yChargingStation+1-yRobot)
        x: float = DistanceX/self.ScaleCourse
        y: float = DistanceY/self.ScaleCourse 
        countx: float=xRobot
        county: float=yRobot
        SectorX =0
        SectorY =0
        MainBranch: float=[]
        UpDown: bool
        RightLeft: bool

        if(yRobot>yChargingStation):
            UpDown=True 
        else:
            UpDown=False
        
        if(xRobot>xChargingStation):
            RightLeft=True
        
        else:
            RightLeft=False
        
        while (SectorY< self.ScaleCourse and SectorX < self.ScaleCourse):
            SectorY=int(county)
            SectorX=int(countx)
            
            if (self.Graph[SectorY,SectorX]!=10.0):
                self.Graph[SectorY,SectorX]=1
                DataValue=[SectorY,SectorX]
                self.Trajectory.add_Node(None, DataValue)
                
            if (self.ScaleCourse>=xChargingStation and self.ScaleCourse>=yChargingStation ):
                self.SetChargingStation( xChargingStation, yChargingStation)
                
            if (SectorX==self.GetChargingStationCoords()[0] and SectorY == self.GetChargingStationCoords()[1] ):
                break
            
            if (UpDown):
                countx-=x
            
            else:
                countx+=x
            
            if (RightLeft):
                county-=y
            else:
                county+=y
    def UpdateObstacles(self):
        objects=random.randint(1,24)
        Obstacles=[]
        for _ in range(objects):
            Obstacles.append([random.randint(1, 24), random.randint(1, 24)])
        for Obstacle in Obstacles:
            if Obstacle not in self.AllObstacles:
                self.AllObstacles.append([Obstacle[0],Obstacle[1]]) 
                            
    def GraphObstacles(self):
        for Obstacle in self.AllObstacles:
            if self.Graph[Obstacle[0],Obstacle[1]] not in (10, 9, 5):
                self.Graph[Obstacle[0],Obstacle[1]]=5   

    def get_graph(self):
        return self.Graph
    def dijkstra(self, start, goal):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        rows, cols = self.Graph.shape
        distances = np.full((rows, cols), np.inf)
        distances[start[0], start[1]] = 0
        pq = [(0, start)]
        previous = {tuple(start): None}

        while pq:
            current_distance, current_node = heapq.heappop(pq)

            if current_node == goal:
                path = []
                while current_node:
                    path.append(list(current_node))
                    current_node = previous[current_node]
                path.reverse()
                return path

            if current_distance > distances[current_node[0], current_node[1]]:
                continue

            for direction in directions:
                neighbor = (current_node[0] + direction[0], current_node[1] + direction[1])
                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                    if self.Graph[neighbor[0], neighbor[1]] == 5:  # Obstacle
                        continue
                    distance = current_distance + 1
                    if distance < distances[neighbor[0], neighbor[1]]:
                        distances[neighbor[0], neighbor[1]] = distance
                        previous[neighbor] = current_node
                        heapq.heappush(pq, (distance, neighbor))
        return None
    def a_star(self, start, goal):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        rows, cols = self.Graph.shape
        g_scores = {start: 0}
        f_scores = {start: self.heuristic(start, goal)}
        came_from = {}

        open_set = [(f_scores[start], start)]

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for direction in directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                    if self.Graph[neighbor[0], neighbor[1]] == 5:  # Obstacle
                        continue
                    tentative_g_score = g_scores[current] + 1  # Assuming each step has a cost of 1

                    if tentative_g_score < g_scores.get(neighbor, math.inf):
                        came_from[neighbor] = current
                        g_scores[neighbor] = tentative_g_score
                        f_scores[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_scores[neighbor], neighbor))

        return None
    def Trajectory_Adjustment(self, algorithm="dijkstra"):
        for Obstacle in self.AllObstacles:
            if Obstacle in self.Trajectory.get_Main():
                index, TrajectorycCoords = self.Trajectory.find_Node(Obstacle)
                previousTrajectorycCoords = self.Trajectory.get_Node(None, index - 1)
                nextTrajectroyCoords = self.Trajectory.get_Node(None, index + 1)

                if algorithm == "dijkstra":
                    new_path = self.dijkstra(tuple(previousTrajectorycCoords), tuple(nextTrajectroyCoords))
                elif algorithm == "a_star":
                    new_path = self.a_star(tuple(previousTrajectorycCoords), tuple(nextTrajectroyCoords))
                if new_path:
                    self.Trajectory.reappend_Nodes(Obstacle, new_path)
                    for Coords in new_path:
                        self.Graph[Coords[0], Coords[1]] = 1
                    

class GraphAlgorithmNode(Node):
    def __init__(self):
        super().__init__('graph_algorithm_subpub')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            "topic_Main_to_Graph_Algorithm",
            self.listener_callback,
            10)
        self.subscription

        self.publisher_ = self.create_publisher(Float32MultiArray, 'topic_Graph_Algorithm_to_Main', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.data_set = []
        self.subscriber_flag = False
        self.graph_flag = False
        # Initialize Graph_Algorithm
        self.graph_algorithm = Graph_Algorithm()

        # Initialize Matplotlib plot
        plt.ion()  # Turn on interactive mode
        self.fig, self.ax = plt.subplots()

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        self.data_set = msg.data
        self.subscriber_flag = True
        self.stop_publishing()

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [32.0, 32.0 * 2.0, 32.0 * 3.0]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.graph_flag= False
        # Update plot after ROS message publishing
        self.update_plot()

    def start_publishing(self):
        if self.timer is None:
            self.timer = self.create_timer(1.0, self.timer_callback)

    def stop_publishing(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
    def get_subscriber_flag(self):
        return self.subscriber_flag
    def get_init_graph(self):
        return self.graph_flag
    def set_init_graph(self):
        self.graph_flag=True
    
    def update_plot(self):
        self.graph_algorithm.SetTrajectory()
        self.graph_algorithm.UpdateObstacles()
        self.graph_algorithm.GraphObstacles
        graph = self.graph_algorithm.get_graph()
        
        self.ax.clear()
        sns.heatmap(graph, ax=self.ax,cbar=False, annot=True, cmap='viridis', vmin=0, vmax=10)
        plt.draw()
        plt.pause(0.1)  # Brief pause to ensure plot updates

def main(args=None):
    rclpy.init(args=args)
    node = GraphAlgorithmNode()
    current_mode = Graphing_Mode.listener_state
    try:
        while rclpy.ok():
            match current_mode:
                case Graphing_Mode.listener_state:  # Listener_state
                    node.stop_publishing()
                    while not node.get_subscriber_flag():
                        rclpy.spin_once(node)  # Process ROS callbacks
                        time.sleep(1)  # Delay to avoid busy waiting
                    current_mode = Graphing_Mode.configure_data_state
                    continue
                case Graphing_Mode.configure_data_state:  # Configure Data
                    if not node.get_init_graph():   
                        node.set_init_graph()
                    current_mode = Graphing_Mode.setup_graph_state
                    continue
                case Graphing_Mode.setup_graph_state:  # Setup the Graph
                    node.update_plot()
                    current_mode = Graphing_Mode.publisher_state
                case Graphing_Mode.publisher_state:
                    node.start_publishing()
                    #while True:
                    rclpy.spin_once(node)
                        #if not node.get_subscriber_flag():
                            #break
                    node.stop_publishing()
                    current_mode = Graphing_Mode.listener_state
                    continue
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        plt.close('all')  # Close the plot window

if __name__ == '__main__':
    main()