import time
from tqdm import tqdm
from dronekit import connect, VehicleMode

print('INFO: starting dronekit')
vehicle = connect('udpin:10.105.196.42:8000', wait_ready=True)
# vehicle = connect('udpin:10.105.196.42:14550', wait_ready=True)
vehicle.wait_ready('autopilot_version')

def g():
    vehicle.mode = VehicleMode("GUIDED")

def l():
    vehicle.mode = VehicleMode("LAND")

# while True:
#     print("Attitude: %s" % vehicle.attitude)
#     print('pitch: %s' %vehicle.attitude.pitch)
#     print("Velocity: %s" % vehicle.velocity)
#     print("GPS: %s" % vehicle.gps_0)
#     print("Flight mode currently: %s" % vehicle.mode.name)
#     print('battery level: %s' % vehicle.battery.level)
#     print('status: %s'% vehicle.system_status.state)
#     print('armed: %s'% vehicle.armed)
#     print('heading: %s' %vehicle.heading)
#     print('speed: %s'% vehicle.groundspeed)
#     print('gps fix: %s'% vehicle.gps_0.fix_type)
#     print('sattelites: %s' %vehicle.gps_0.satellites_visible)
#     print('location global lat: %s'% vehicle.location.global_frame.lat)
#     print('location global lon: %s'% vehicle.location.global_frame.lon)
#     print ('location global alt: %s' %vehicle.location.global_frame.alt)
#     print('heading: %s'% vehicle.heading)
#     print('groundspeed: %s' % vehicle.groundspeed)
#     print('velocity: %s'% vehicle.velocity[2])
#     time.sleep(1)

# Take off
print('Taking off')
takeoff_height = 1  # meter
vehicle.mode = VehicleMode("GUIDED")
vehicle.simple_takeoff(takeoff_height)
while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt>=takeoff_height*0.95:
        print("Reached target altitude")
        break
    time.sleep(0.5)
print('Take off done')

# Pitch test
try:
    print('Pitch test')
    for i in tqdm(range(100)):
        manual_control_message = vehicle.message_factory.manual_control_encode(
            0,      # target (0 indicates default target)
            100,    # x (pitch: -1000 to 1000)  - 0 is hover
            0,      # y (roll: -1000 to 1000)   - 0 is hover
            500,    # z (throttle: 0 to 1000)   - 500 is hover
            0,      # r (yaw: -1000 to 1000)    - 0 is hover
            0       # buttons (see MAVLink MANUAL_CONTROL documentation for button mapping)
        )
        vehicle.send_mavlink(manual_control_message)
        time.sleep(0.01)
except KeyboardInterrupt:
    pass

# Land
time.sleep(1)
print('Landing')
vehicle.mode = VehicleMode("LAND")
while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt==0:
        print("Landed")
        break
    time.sleep(0.5)
