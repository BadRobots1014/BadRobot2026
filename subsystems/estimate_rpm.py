from sklearn.linear_model import LinearRegression
from commands2 import Subsystem
from hardware.impl.limelight import Limelight
from ntcore import NetworkTableInstance
import numpy as np
from math import sqrt

class EstimateRPM(Subsystem):
    def __init__(self, limelight: Limelight):
        super().__init__()
        self.limelight = limelight

        # Distances in meters need to match with RPMS by index
        self.distances = np.array([1, 3, 5, 6]).reshape(-1, 1)
        self.rpms = np.array([1, 2, 4, 5])

        self.regression = LinearRegression().fit(self.distances, self.rpms)

        self.score_distance = 9999

        self.nt_inst = NetworkTableInstance.getDefault()
        self.nt_table = self.nt_inst.getTable("RPM Estimation")

        self.distance_topic = self.nt_table.getDoubleTopic("Distance")
        self.distance_pub = self.distance_topic.publish()
        self.distance_pub.set(self.score_distance)

    def calculate_rpm(self):
        target_pose = self.limelight.target_pose_sub.get()
        target_distance = sqrt(target_pose.x**2 + target_pose.y**2)

        self.distance_pub.set(target_distance)

        return self.regression.predict(np.array([[target_distance]]))[0]