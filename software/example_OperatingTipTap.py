# importing tiptap will detect if hw is available,
# if not available, a pybullet sim will be started,
# otherwise, the hardware will be made ready for use
from tiptap import (TipTap,  # tiptap interfaces
                    p,  # pybullet (equals None if on HW)
                    np)  # numpy

# When you first program the DirectServo firmware
# you'll need to calibrate it once, the calibration
# will get written into flash memory. Make sure
# to power on the robot when its joints are in what
# you want to consider to be the "zero position",
# before you calibrate. Note: The robot will not
# allow you to control it if it is not calibrated.
if False:
    TipTap.calibrate()

# whenever you power cycle TipTap, the motors
# will need to be re-told where the zero angle
# of each joint is. To do this, move the legs
# to the zero position and run TipTap.setZeroAngles()
# Note: The robot will not allow you to control it if
# it is calibrated with failsafe limits, but no zero
# position is known.
if False:
    TipTap.setZeroAngles()

# The ControlSignal is defined as = [
#     RightServo radians,
#     RightHip torque ratio,
#     RightKnee torque ratio,
#     LeftServo radians,
#     LeftHip torque ratio,
#     LeftKnee torque ratio
# ]
ControlSignal = [0. for _ in range(6)]

# You can use the TipTap.OnHardware() query to
# add in simulation specific functions
if not TipTap.OnHardware():
    line_ids = [p.addUserDebugLine([0, 0, 0], [0, 0, 0]) for _ in range(3)]

q, dq, ddq = TipTap.getStates()
dt, elapsed_time = TipTap.GetTimeStep()

# main sim/control loop
while True:

    # get a dictionary of the current IMU states
    imu_data = TipTap.getIMU()

    # feel free to do simulation specific things, using not TipTap.OnHardware()
    if not TipTap.OnHardware():
        bs = TipTap.GetBodyState()  # (only in sim)
        pos = np.array(bs["pos"])
        mat = bs["mat"]
        # draw some debugging lines (+x = red,+y = green,+z = blue)
        for bvi in range(3):
            base_vect = [bvi == 0, bvi == 1, bvi == 2]
            p.addUserDebugLine(pos, pos+mat.dot(base_vect)*0.3,
                               lineColorRGB=base_vect,
                               replaceItemUniqueId=line_ids[bvi])

    # TODO: Put your controller here, which uses "imu_data" and
    #       crafts a new "ControlSignal" list.
    #
    # Here's an example of a controller, that just works to point
    # the legs downwards
    torque_gain = -100*dt
    ControlSignal[TipTap.RightServo_num] = imu_data["roll"]
    ControlSignal[TipTap.RightHip_num] = (
        q[TipTap.RightHip_num]-imu_data["pitch"])*torque_gain
    ControlSignal[TipTap.RightKnee_num] = (
        q[TipTap.RightKnee_num] +
        q[TipTap.RightHip_num]-imu_data["pitch"])*torque_gain
    ControlSignal[TipTap.LeftServo_num] = imu_data["roll"]
    ControlSignal[TipTap.LeftHip_num] = (
        q[TipTap.LeftHip_num]-imu_data["pitch"])*torque_gain
    ControlSignal[TipTap.LeftKnee_num] = (
        q[TipTap.LeftKnee_num] +
        q[TipTap.LeftHip_num]-imu_data["pitch"])*torque_gain

    # apply controller torques and positions, and get the new joint states
    q, dq, ddq = TipTap.setStates(ControlSignal)

    # control the simulation step if not on hw, or if running on hardware
    # GetTimeStep() just returns the elapsed time info
    dt, elapsed_time = TipTap.GetTimeStep(fixedTimeStep=1./300.)
