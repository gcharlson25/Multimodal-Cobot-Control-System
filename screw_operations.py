import time
import math

import __common
__common.init_env()
import jkrc

ABS_MOVEMENT = 0
INCREMENT_MOVEMENT = 1

TCP = [0, 0, 0, 0, 0, 0]
USR = [0, 0, 0, 0, 0, 0]

IO_TOOL = 1
ON = 1
OFF = 0
GO = 1
DIRECTION = 2

HOME_JOINT = [math.radians(x) for x in [-92.059, 58.642, 135.473, -35.255, -93.233, -40.562]]
SCREW1_UNFASTEN_START_JOINT = [math.radians(x) for x in [-61.318, 84.653, 111.844, 47.054, -104.174, -19.414]]
SCREW1_FASTEN_START_JOINT = [math.radians(x) for x in [-61.318, 84.028, 109.690, 49.831, -104.174, -19.414]]
SCREW2_UNFASTEN_START_JOINT = [math.radians(x) for x in [-71.718, 83.511, 113.704, 44.405, -99.375, -28.842]]
SCREW2_FASTEN_START_JOINT = [math.radians(x) for x in [-71.718, 82.843, 111.534, 47.243, -99.375, -28.842]]
SCREW3_UNFASTEN_START_JOINT = [math.radians(x) for x in [-77.293, 83.519, 113.269, 44.143, -96.694, -33.779]]
SCREW3_FASTEN_START_JOINT = [math.radians(x) for x in [-77.293, 82.870, 111.088, 46.972, -96.694, -33.779]]
SCREW4_UNFASTEN_START_JOINT = [math.radians(x) for x in [-82.604, 84.152, 112.841, 43.500, -94.094, -38.431]]
SCREW4_FASTEN_START_JOINT = [math.radians(x) for x in [-82.604, 83.497, 110.672, 46.324, -94.094, -38.431]]
SCREW5_UNFASTEN_START_JOINT = [math.radians(x) for x in [-88.220, 85.296, 111.889, 43.076, -91.316, -43.318]]
SCREW5_FASTEN_START_JOINT = [math.radians(x) for x in [-88.220, 84.637, 109.737, 45.888, -91.316, -43.318]]

ENGAGE = [0, 0, -9, 0, 0, 0]
DISENGAGE = [0, 0, 50, 0, 0, 0]
UNFASTEN = [0, 0, 60, 0, 0, 0]
FASTEN = [0, 0, -22, 0, 0, 0]

def coordinateSetup(cobot):
    print("setting TCP")
    cobot.set_tool_data(5, TCP, "tool_screw_test")
    cobot.set_tool_id(5)

    print("setting USR") 
    cobot.set_user_frame_data(6, USR, "user_screw_test")
    cobot.set_user_frame_id(6)

def cobotSetup():
    print("\n\n\n")
    cobot = jkrc.RC("192.168.10.200")
    print("logging in")
    cobot.login()
    print("powering on")
    cobot.power_on()
    print("enabling")
    cobot.enable_robot()
    print("setting payload and centroid")
    cobot.set_payload(mass = 0.5, centroid = [0, 0, 20])
    print("setting outputs")
    cobot.set_digital_output(IO_TOOL, DIRECTION,  OFF)
    cobot.set_digital_output(IO_TOOL, GO, OFF)
    coordinateSetup(cobot)
    return cobot

def fastenScrewOperation(cobot):
    cobot.set_digital_output(IO_TOOL, DIRECTION, OFF)
    cobot.set_digital_output(IO_TOOL, GO, ON)
    time.sleep(1)
    cobot.linear_move_extend(ENGAGE, INCREMENT_MOVEMENT, True, 664.046, 1348.247, 1)
    cobot.linear_move_extend(FASTEN, INCREMENT_MOVEMENT, True, 35.000, 572.894, 1)
    cobot.set_digital_output(IO_TOOL, GO, OFF)
    cobot.linear_move_extend(DISENGAGE, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)
    cobot.set_digital_output(IO_TOOL, DIRECTION, OFF)
    cobot.set_digital_output(IO_TOOL, GO, OFF)

def unfastenScrewOperation(cobot):
    cobot.linear_move_extend(ENGAGE, INCREMENT_MOVEMENT, True, 970.854, 2288.403, 1)
    time.sleep(1)
    cobot.set_digital_output(IO_TOOL, DIRECTION, ON)
    cobot.set_digital_output(IO_TOOL, GO, ON)
    time.sleep(0.5)
    cobot.linear_move_extend(UNFASTEN, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)
    cobot.set_digital_output(IO_TOOL, GO, OFF)
    cobot.linear_move_extend(DISENGAGE, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)

def unfastenScrew1(cobot):
    print("unfastening screw one")
    cobot.joint_move(SCREW1_UNFASTEN_START_JOINT, ABS_MOVEMENT, True, 2)
    unfastenScrewOperation(cobot)
    cobot.linear_move_extend(DISENGAGE, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)

def fastenScrew1(cobot):
    print("fastening screw one")
    cobot.joint_move(SCREW1_FASTEN_START_JOINT, ABS_MOVEMENT, True, 2)
    fastenScrewOperation(cobot)
    cobot.linear_move_extend(DISENGAGE, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)

def unfastenScrew2(cobot):
    print("unfastening screw two")
    cobot.joint_move(SCREW2_UNFASTEN_START_JOINT, ABS_MOVEMENT, True, 2)
    unfastenScrewOperation(cobot)
    cobot.linear_move_extend(DISENGAGE, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)

def fastenScrew2(cobot):
    print("fastening screw two")
    cobot.joint_move(SCREW2_FASTEN_START_JOINT, ABS_MOVEMENT, True, 2)
    fastenScrewOperation(cobot)
    cobot.linear_move_extend(DISENGAGE, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)

def unfastenScrew3(cobot):
    print("unfastening screw three")
    cobot.joint_move(SCREW3_UNFASTEN_START_JOINT, ABS_MOVEMENT, True, 2)
    unfastenScrewOperation(cobot)
    cobot.linear_move_extend(DISENGAGE, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)

def fastenScrew3(cobot):
    print("fastening screw three")
    cobot.joint_move(SCREW3_FASTEN_START_JOINT, ABS_MOVEMENT, True, 2)
    fastenScrewOperation(cobot)
    cobot.linear_move_extend(DISENGAGE, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)

def unfastenScrew4(cobot):
    print("unfastening screw four")
    cobot.joint_move(SCREW4_UNFASTEN_START_JOINT, ABS_MOVEMENT, True, 2)
    unfastenScrewOperation(cobot)
    cobot.linear_move_extend(DISENGAGE, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)

def fastenScrew4(cobot):
    print("fastening screw four")
    cobot.joint_move(SCREW4_FASTEN_START_JOINT, ABS_MOVEMENT, True, 2)
    fastenScrewOperation(cobot)
    cobot.linear_move_extend(DISENGAGE, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)

def unfastenScrew5(cobot):
    print("unfastening screw five")
    cobot.joint_move(SCREW5_UNFASTEN_START_JOINT, ABS_MOVEMENT, True, 2)
    unfastenScrewOperation(cobot)
    cobot.linear_move_extend(DISENGAGE, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)

def fastenScrew5(cobot):
    print("fastening screw five")
    cobot.joint_move(SCREW5_FASTEN_START_JOINT, ABS_MOVEMENT, True, 2)
    fastenScrewOperation(cobot)
    cobot.linear_move_extend(DISENGAGE, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)

def unfastenAll(cobot):
    unfastenScrew1(cobot)
    unfastenScrew2(cobot)
    unfastenScrew3(cobot)
    unfastenScrew4(cobot)
    unfastenScrew5(cobot)

def fastenAll(cobot):
    fastenScrew1(cobot)
    fastenScrew2(cobot)
    fastenScrew3(cobot)
    fastenScrew4(cobot)
    fastenScrew5(cobot)

def unfastenAfterAlign(cobot):
    print("unfastening the screw")
    cobot.linear_move_extend([0, 0, -19, 0, 0, 0], INCREMENT_MOVEMENT, True, 970.854, 2288.403, 1)
    time.sleep(1)
    cobot.set_digital_output(IO_TOOL, DIRECTION, ON)
    cobot.set_digital_output(IO_TOOL, GO, ON)
    time.sleep(0.5)
    cobot.linear_move_extend(UNFASTEN, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)
    cobot.set_digital_output(IO_TOOL, GO, OFF)
    cobot.linear_move_extend(DISENGAGE, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)
    cobot.linear_move_extend([0, 0, -78.5, 0, 0, 0], INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)
    time.sleep(1)
    print("fastening the screw")
    cobot.set_digital_output(IO_TOOL, DIRECTION, OFF)
    cobot.set_digital_output(IO_TOOL, GO, ON)
    time.sleep(1)
    cobot.linear_move_extend(ENGAGE, INCREMENT_MOVEMENT, True, 664.046, 1348.247, 1)
    cobot.linear_move_extend(FASTEN, INCREMENT_MOVEMENT, True, 35.000, 572.894, 1)
    cobot.set_digital_output(IO_TOOL, GO, OFF)
    cobot.linear_move_extend(DISENGAGE, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)
    cobot.set_digital_output(IO_TOOL, DIRECTION, OFF)
    cobot.set_digital_output(IO_TOOL, GO, OFF)
    cobot.linear_move_extend(DISENGAGE, INCREMENT_MOVEMENT, True, 1800, 3500.000, 1)

def main():
    cobot = cobotSetup()
    cobot.joint_move(HOME_JOINT, ABS_MOVEMENT, True, 2)
    cobot.set_digital_output(IO_TOOL, DIRECTION,  OFF)
    cobot.set_digital_output(IO_TOOL, GO, OFF)
    unfastenScrew5(cobot)
    fastenScrew5(cobot)
    cobot.set_digital_output(IO_TOOL, DIRECTION,  OFF)
    cobot.set_digital_output(IO_TOOL, GO, OFF)
    cobot.joint_move(HOME_JOINT, ABS_MOVEMENT, True, 2) 
        
if __name__ == '__main__':
    main()    

