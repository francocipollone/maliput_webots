"""maliput_rail_car controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# from controller import Robot
from vehicle import Driver
from controller import GPS


import maliput.api
import maliput.plugin

import os



configuration = {"opendrive_file" : os.getenv("MALIPUT_MALIDRIVE_RESOURCE_ROOT") + "/resources/odr/LoopRoadPedestrianCrosswalk.xodr"}
road_network = maliput.plugin.create_road_network("maliput_malidrive", configuration)
print("Maliput RoadNetwork has been loaded.")
rg = road_network.road_geometry()


# create the Robot/Driver instance.
driver = Driver()

# get the time step of the current world.
timestep = int(driver.getBasicTimeStep())

# Get gps device.
gps = driver.getDevice('gps')
gps.enable(timestep)


# Set up speed and steering angle.
driver.setCruisingSpeed(20.0) # km/h
#driver.setThrottle(1.0) # 0.0 to 1.0
driver.setSteeringAngle(0.) # -1.0 (left) to 1.0 (right)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while driver.step() != -1:
    gps_pos = gps.getValues()
    road_position = rg.ToRoadPosition(maliput.api.InertialPosition(gps_pos[0], gps_pos[1], gps_pos[2]))
    # print("gps: " + gps_pos.__str__() + " | lane id: " + road_position.road_position.lane.id().string() + " | lane_pos: " + road_position.road_position.pos.srh().__str__())

# Enter here exit cleanup code.
