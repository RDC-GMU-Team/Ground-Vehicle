gpsOutput- no idea what this is used for
interpolatePoints- heper script- run when you've inserted more than 2 waypoints into raw waypoints and it will interpolate a line between them in waypoints
morning_planner- no clue
PID_deployed- old PID planner, not in use. Still functional but unreliable.
pid_speed- PID loop to regulate the speed of the UGV
planner- current RRT planner used for the competition
raw_waypoints.csv- the input for interpolatepoints
rostest- I think this was a helper script?
sensor_fusion- no clue. looks like a way to merge IMU and gps but I'm not sure if its in use
twistToCommand- taking motion planner instructions and feeds it to the arduino. Also does stuff like regulating the switching between manual and remote control.
utilities, utils, utils_legacy- all helper files for the other scripts
water_log- where water strikes are logged
waterlog- script for water strikes
waypoints- csv for waypoints fed into the planner