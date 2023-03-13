"""maliput_rail_car controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# from controller import Robot
from vehicle import Driver
from controller import GPS


import maliput.api
import maliput.plugin
from maliput.api import (
    Which,
)

import math
import os

def norm(vector3):
  return math.sqrt(vector3.x() * vector3.x() + vector3.y() * vector3.y() + vector3.z() * vector3.z())

def ComputeCurvature(s_lookahead, rg, inertial_position, heading):
  goal_position = ComputeGoalPoint(s_lookahead, rg, inertial_position)
  inertial_position = inertial_position.xyz()
  x = inertial_position.x()
  y = inertial_position.y()
  delta_r = -(goal_position.x() - x) * math.sin(heading) + (goal_position.y() - y) * math.cos(heading)
  difference_vector = maliput.math.Vector3(goal_position.x() - inertial_position.x(), goal_position.y() - inertial_position.y(), goal_position.z() - inertial_position.z())
  curvature = 2. * delta_r / math.pow(norm(difference_vector), 2.)
  return curvature

def ComputeGoalPoint(s_lookahead, rg, inertial_position):
  road_position_result = rg.ToRoadPosition(inertial_position)
  lane = road_position_result.road_position.lane
  s_new = road_position_result.road_position.pos.s() + s_lookahead
  if s_new > lane.length():
     s_delta = s_new - lane.length()
     lane_end = lane.GetDefaultBranch(maliput.api.Which.kFinish)
     if(lane_end is not None):
        lane = lane_end.lane
        s_new = s_delta
  return lane.ToInertialPosition(maliput.api.LanePosition(s_new, 0., 0.)).xyz()


configuration = {"osm_file" : os.getenv("MALIPUT_OSM_RESOURCE_ROOT") + "/resources/osm/circuit.osm"}
road_network = maliput.plugin.create_road_network("maliput_osm", configuration)
print("Maliput RoadNetwork has been loaded.")
rg = road_network.road_geometry()


# create the Robot/Driver instance.
driver = Driver()

# get the time step of the current world.
timestep = int(driver.getBasicTimeStep())

# Get gps device.
gps = driver.getDevice('gps')
gps.enable(timestep)

# Get inertial unit
inertial_unit = driver.getDevice('inertial unit')
inertial_unit.enable(timestep)


# Set up speed and steering angle.
driver.setCruisingSpeed(30.0) # km/h
#driver.setThrottle(1.0) # 0.0 to 1.0
driver.setSteeringAngle(0.) # -1.0 (left) to 1.0 (right)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
s_lookahead = 5.
k_gain=2.5
while driver.step() != -1:
    gps_pos = gps.getValues()
    current_yaw_angle = inertial_unit.getRollPitchYaw()[2]

    inertial_position = maliput.api.InertialPosition(gps_pos[0], gps_pos[1], gps_pos[2])
    curvature = ComputeCurvature(s_lookahead, rg, inertial_position, current_yaw_angle)

    driver.setSteeringAngle(-curvature*k_gain)
    
    # print("gps: " + gps_pos.__str__() + " | lane id: " + road_position.road_position.lane.id().string() + " | lane_pos: " + road_position.road_position.pos.srh().__str__())

    
# Enter here exit cleanup code.
