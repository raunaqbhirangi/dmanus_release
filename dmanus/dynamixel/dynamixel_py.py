import os, ctypes
import time
import numpy as np

# os.sys.path.append('../dynamixel_functions_py')                 # Path setting
import dynamixel_functions as dynamixel  # Uses Dynamixel SDK library
import click
import copy


class Dynamixel_X:
    def __init__(self):
        # Control table address
        self.ADDR_TORQUE_ENABLE = 64  # Control table addr different in Dxl model
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_PRESENT_CURRENT = 126
        self.ADDR_PRESENT_VELOCITY = 128
        self.ADDR_PRESENT_POS_VEL = 128  # Trick to get position and velocity at once
        self.ADDR_MAX_VELOCITY = 44
        self.ADDR_HARDWARE_ERROR = 70

        # control mode options
        self.ADDR_OPERATION_MODE = 11
        self.ADDR_GOAL_PWM = 100  # Lowest byte of goal torque value

        # Data Byte Length
        self.LEN_PRESENT_POSITION = 4
        self.LEN_PRESENT_VELOCITY = 4
        self.LEN_PRESENT_POS_VEL = 8
        self.LEN_GOAL_POSITION = 4
        self.LEN_GOAL_PWM = 2


class Dynamixel_MX:
    def __init__(self):
        # Control table address
        self.ADDR_TORQUE_ENABLE = (
            24  # Control table address is different in Dynamixel model
        )
        self.ADDR_GOAL_POSITION = 30
        self.ADDR_PRESENT_POSITION = 36
        self.ADDR_PRESENT_VELOCITY = 38
        self.ADDR_PRESENT_POS_VEL = 36  # Trick to get position and velocity at once
        self.ADDR_MAX_VELOCITY = 44
        self.ADDR_HARDWARE_ERROR = 70

        # torque control mode options (left over from P1)
        self.ADDR_TORQUE_CONTROL_MODE = 70
        self.ADDR_GOAL_PWM = -1  # torque mode not supported yet

        # Data Byte Length
        self.LEN_PRESENT_POSITION = 2
        self.LEN_PRESENT_VELOCITY = 2
        self.LEN_PRESENT_POS_VEL = 4
        self.LEN_GOAL_POSITION = 2
        self.LEN_GOAL_PWM = 2


DXL_NULL_TORQUE_VALUE = 0
DXL_MIN_CW_TORQUE_VALUE = 1024
DXL_MAX_CW_TORQUE_VALUE = 2047
DXL_MIN_CCW_TORQUE_VALUE = 0
DXL_MAX_CCW_TORQUE_VALUE = 1023

# Settings for MX28
POS_SCALE = 2 * np.pi / 4096  # (=.088 degrees)
VEL_SCALE = 0.229 * 2 * np.pi / 60  # (=0.229rpm)
CURR_SCALE = 2.69

DXL_X_CURRENT_MODE = 0  # Value for setting X motor to current(torque) control mode
DXL_X_POSITION_MODE = 3  # Value for setting X motor to position control mode
DXL_X_PWM_MODE = 16  # Value for setting X motor to PWM control mode. Current control doesn't quite do what you want
DXL_X_CURRENT_LIMIT = 120  # Current limit. Normal limit is 1193, but conservative
DXL_X_PWM_LIMIT = 500  # Current limit. Normal limit is 1193, but conservative


TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE = 100  # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE = 4000  # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 15  # Dynamixel moving status threshold

# ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS = 0  # Communication Success result value
# COMM_TX_FAIL                = -1001                       # Communication Tx Failed


class dxl:

    # devicename: Port name being used on your controller # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
    def __init__(
        self,
        motor_id,
        motor_type="X",
        baudrate=1000000,
        devicename="/dev/ttyUSB0",
        protocol=2,
    ):

        self.motor_id = motor_id
        self.motor_type = motor_type
        self.baudrate = baudrate
        self.devicename = devicename
        self.protocol = protocol
        self.n_motors = len(motor_id)

        if motor_type == "MX":
            self.motor = Dynamixel_MX()
        elif motor_type == "X":
            self.motor = Dynamixel_X()
        else:
            quit("Motor type not recognized")

        # default mode
        self.ctrl_mode = {key: None for key in self.motor_id}

        # Initialize PortHandler Structs
        # Set the port path and Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.port_num = dynamixel.portHandler(self.devicename.encode("utf-8"))

        # Initialize PacketHandler Structs
        dynamixel.packetHandler()

    def open_port(self):
        # Open port
        if dynamixel.openPort(self.port_num):
            print("Succeeded to open the port!")
        else:
            print("Failed to open the port %s" % self.devicename)
            os.system("sudo chmod a+rw %s" % self.devicename)
            print("Editing permissions and trying again")
            if dynamixel.openPort(self.port_num):
                print("Succeeded to open the port!")
            else:
                quit(
                    "Failed to open the port! Run following command and try again.\nsudo chmod a+rw %s"
                    % self.devicename
                )

        # Set port baudrate
        if dynamixel.setBaudRate(self.port_num, self.baudrate):
            print("Succeeded to change the baudrate!")
        else:
            quit("Failed to change the baudrate!")

        # Enable Dynamixel Torque
        self.engage_motor(self.motor_id, True)

        # Initialize Group instance

        # controls
        self.group_desPos = dynamixel.groupSyncWrite(
            self.port_num,
            self.protocol,
            self.motor.ADDR_GOAL_POSITION,
            self.motor.LEN_GOAL_POSITION,
        )
        self.group_desTor = dynamixel.groupSyncWrite(
            self.port_num,
            self.protocol,
            self.motor.ADDR_GOAL_PWM,
            self.motor.LEN_GOAL_PWM,
        )  # NOTE: not all motors support them

        # positions
        self.group_pos = dynamixel.groupBulkRead(self.port_num, self.protocol)
        for m_id in self.motor_id:
            dxl_addparam_result = ctypes.c_ubyte(
                dynamixel.groupBulkReadAddParam(
                    self.group_pos,
                    m_id,
                    self.motor.ADDR_PRESENT_POSITION,
                    self.motor.LEN_PRESENT_POSITION,
                )
            ).value
            if dxl_addparam_result != 1:
                print("[ID:%03d] groupBulkRead addparam_posfailed" % (m_id))
                quit()

        # velocities
        self.group_vel = dynamixel.groupBulkRead(self.port_num, self.protocol)
        for m_id in self.motor_id:
            dxl_addparam_result = ctypes.c_ubyte(
                dynamixel.groupBulkReadAddParam(
                    self.group_vel,
                    m_id,
                    self.motor.ADDR_PRESENT_VELOCITY,
                    self.motor.LEN_PRESENT_VELOCITY,
                )
            ).value
            if dxl_addparam_result != 1:
                print("[ID:%03d] groupBulkRead addparam_vel failed" % (m_id))
                quit()

        # positions and velocities
        self.group_pos_vel = dynamixel.groupBulkRead(self.port_num, self.protocol)
        for m_id in self.motor_id:
            dxl_addparam_result = ctypes.c_ubyte(
                dynamixel.groupBulkReadAddParam(
                    self.group_pos_vel,
                    m_id,
                    self.motor.ADDR_PRESENT_POS_VEL,
                    self.motor.LEN_PRESENT_POS_VEL,
                )
            ).value
            if dxl_addparam_result != 1:
                print("[ID:%03d] groupBulkRead addparam_posfailed" % (m_id))
                quit()

        # buffers
        self.dxl_present_position = float("nan") * np.zeros(self.n_motors)
        self.dxl_present_velocity = float("nan") * np.zeros(self.n_motors)
        self.dxl_last_position = float("nan") * np.zeros(self.n_motors)
        self.dxl_last_velocity = float("nan") * np.zeros(self.n_motors)
        print("Dynamixels successfully connected")

    # Cheak health
    def okay(self, motor_id=None):
        if motor_id is None:
            motor_id = self.motor_id.copy()
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num, self.protocol)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num, self.protocol)

        if (dxl_comm_result != COMM_SUCCESS) or (dxl_error != COMM_SUCCESS):
            # Print apprpriate error
            if dxl_comm_result != COMM_SUCCESS:
                print(
                    "\n" + str(dynamixel.getTxRxResult(self.protocol, dxl_comm_result))
                )
            if dxl_error != COMM_SUCCESS:
                print(
                    "\n"
                    + str(dynamixel.getRxPacketError(self.protocol, dxl_error))
                    + ". Error_id:"
                    + str(dxl_error)
                )

            # print hardware status
            print("Motor id(hardware status): [ ", end="")
            for m_id in motor_id:
                print(
                    "%d(%d), "
                    % (
                        m_id,
                        dynamixel.read1ByteTxRx(
                            self.port_num,
                            self.protocol,
                            m_id,
                            self.motor.ADDR_HARDWARE_ERROR,
                        ),
                    ),
                    end="",
                )
            print("]", flush=True)
            return False
        else:
            return True

    @property
    def motors_enabled(self):
        motor_id = self.motor_id.copy()
        enable_flags = []
        for dxl_id in motor_id:
            enable_flags.append(
                bool(
                    dynamixel.read1ByteTxRx(
                        self.port_num,
                        self.protocol,
                        dxl_id,
                        self.motor.ADDR_TORQUE_ENABLE,
                    )
                )
            )
        return enable_flags

    @property
    def motor_mode(self):
        return self.ctrl_mode

    # Engage/ Disengage the motors. enable = True/ False
    def engage_motor(self, motor_id, enable):
        enabled_flags = copy.deepcopy(self.motors_enabled)

        for dxl_id in motor_id:
            dxl_list_id = self.motor_id.index(dxl_id)
            if enabled_flags[dxl_list_id] == enable:
                continue
            # fault handelling
            while True:
                dynamixel.write1ByteTxRx(
                    self.port_num,
                    self.protocol,
                    dxl_id,
                    self.motor.ADDR_TORQUE_ENABLE,
                    enable,
                )
                if self.okay([dxl_id]):
                    break
                else:
                    print(
                        "dxl%d: Error with ADDR_TORQUE_ENABLE. Retrying after reboot ..."
                        % dxl_id,
                        flush=True,
                    )
                    dynamixel.reboot(self.port_num, self.protocol, dxl_id)
                    time.sleep(0.25)  # Pause for reboot

    def _set_register(self, dxl_id: int, address: int, size: int, value: int):
        if size == 1:
            dynamixel.write1ByteTxRx(
                self.port_num, self.protocol, dxl_id, address, value
            )
        elif size == 2:
            dynamixel.write2ByteTxRx(
                self.port_num, self.protocol, dxl_id, address, value
            )
        elif size == 4:
            dynamixel.write4ByteTxRx(
                self.port_num, self.protocol, dxl_id, address, value
            )

    # Enable/Disable torque control. Currently only allows all the motors
    # to be either position control or torque control. Leaves motors engaged
    # or disengaged depending on initial state.
    def torque_control(self, motor_id, enable: bool):

        motors_enabled = copy.deepcopy(self.motors_enabled)
        # Disengage all motors so register isn't locked

        for dxl_id in motor_id:
            # If control mode is same as desired, skip
            if self.ctrl_mode[dxl_id] == enable:
                continue

            # Check if motor is engaged
            dxl_list_id = self.motor_id.index(dxl_id)
            enable_flag = motors_enabled[dxl_list_id]
            if enable_flag:
                self.engage_motor([dxl_id], enable=False)
            while True:
                if isinstance(self.motor, Dynamixel_X):
                    if enable:
                        dynamixel.write1ByteTxRx(
                            self.port_num,
                            self.protocol,
                            dxl_id,
                            self.motor.ADDR_OPERATION_MODE,
                            DXL_X_PWM_MODE,
                        )
                        # dynamixel.write2ByteTxRx(
                        #     self.port_num,
                        #     self.protocol,
                        #     dxl_id,
                        #     self.motor.ADDR_GOAL_PWM,
                        #     0,
                        # )
                    else:
                        dynamixel.write1ByteTxRx(
                            self.port_num,
                            self.protocol,
                            dxl_id,
                            self.motor.ADDR_OPERATION_MODE,
                            DXL_X_POSITION_MODE,
                        )
                else:
                    dynamixel.write1ByteTxRx(
                        self.port_num,
                        self.protocol,
                        dxl_id,
                        self.motor.ADDR_TORQUE_CONTROL_MODE,
                        enable,
                    )

                if self.okay([dxl_id]):
                    break
                else:
                    print(
                        "dxl%d: Error with enabling torque control. Retrying after reboot ..."
                        % dxl_id,
                        flush=True,
                    )
                    dynamixel.reboot(self.port_num, self.protocol, dxl_id)
                    time.sleep(0.25)  # Pause for reboot

            # Toggle ctrl_mode
            self.ctrl_mode[dxl_id] = enable
            # Reengage motor if previously engaged
            if enable_flag:
                self.engage_motor([dxl_id], enable=True)

    def getIndividual_curr(self, motor_id):
        dxl_present_curr = []
        # Read present position
        for dxl_id in motor_id:
            dxl_present_curr.append(
                dynamixel.read2ByteTxRx(
                    self.port_num,
                    self.protocol,
                    dxl_id,
                    self.motor.ADDR_PRESENT_CURRENT,
                )
            )
        if not self.okay(motor_id):
            self.close(motor_id)
            quit("error getting self.motor.ADDR_PRESENT_CURRENT")

        dxl_present_curr = np.array(dxl_present_curr)

        for i in range(len(dxl_present_curr)):
            if dxl_present_curr[i] >= 1024:
                dxl_present_curr[i] = -1.0 * (dxl_present_curr[i] - 1024)

        return CURR_SCALE * dxl_present_curr

    # Returns pos in radians and velocity in radian/ sec
    def get_pos_vel_old(self, motor_id):

        dxl_present_position = []
        dxl_present_velocity = []

        # Bulkread present positions
        dynamixel.groupBulkReadTxRxPacket(self.group_pos_vel)

        # Retrieve data
        for i in range(self.n_motors):
            dxl_id = motor_id[i]
            # Get present position value
            dxl_getdata_result = ctypes.c_ubyte(
                dynamixel.groupBulkReadIsAvailable(
                    self.group_pos_vel,
                    dxl_id,
                    self.motor.ADDR_PRESENT_POS_VEL,
                    self.motor.LEN_PRESENT_POS_VEL,
                )
            ).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupBulkRead get_pos_vel failed" % (dxl_id))
                dxl_present_position.append(float("nan"))
                dxl_present_velocity.append(float("nan"))
                # quit()
            else:
                dxl_present_position.append(
                    dynamixel.groupBulkReadGetData(
                        self.group_pos_vel,
                        dxl_id,
                        self.motor.ADDR_PRESENT_POSITION,
                        self.motor.LEN_PRESENT_POSITION,
                    )
                )
                dxl_present_velocity.append(
                    dynamixel.groupBulkReadGetData(
                        self.group_pos_vel,
                        dxl_id,
                        self.motor.ADDR_PRESENT_VELOCITY,
                        self.motor.LEN_PRESENT_VELOCITY,
                    )
                )

        dxl_present_velocity = np.array(dxl_present_velocity)
        for i in range(len(dxl_present_velocity)):
            if dxl_present_velocity[i] >= 1024:
                dxl_present_velocity[i] = -1.0 * (dxl_present_velocity[i] - 1024)
        return POS_SCALE * np.array(dxl_present_position), VEL_SCALE * np.array(
            dxl_present_velocity
        )

    # Returns pos in radians and velocity in radian/ sec
    def get_pos_vel(self, motor_id):

        # Bulkread present positions
        dynamixel.groupBulkReadTxRxPacket(self.group_pos_vel)
        if not self.okay(motor_id):
            print(
                "try one more time. If not, we will spoof packets below ====================== "
            )
            # try one more time. If not, we will spoof packets below.
            dynamixel.groupBulkReadTxRxPacket(self.group_pos_vel)

        dxl_errored = []
        # Retrieve data
        for i in range(self.n_motors):
            dxl_id = motor_id[i]

            # Get present position value
            dxl_getdata_result = ctypes.c_ubyte(
                dynamixel.groupBulkReadIsAvailable(
                    self.group_pos_vel,
                    dxl_id,
                    self.motor.ADDR_PRESENT_POS_VEL,
                    self.motor.LEN_PRESENT_POS_VEL,
                )
            ).value
            if dxl_getdata_result != 1:
                # send last known values
                dxl_errored.append(dxl_id)

                self.dxl_present_position[i] = self.dxl_last_position[i].copy()
                self.dxl_present_velocity[i] = self.dxl_last_velocity[i].copy()
            else:
                self.dxl_last_position[i] = self.dxl_present_position[i].copy()
                self.dxl_last_velocity[i] = self.dxl_present_velocity[i].copy()

                dxl_present_position = dynamixel.groupBulkReadGetData(
                    self.group_pos_vel,
                    dxl_id,
                    self.motor.ADDR_PRESENT_POSITION,
                    self.motor.LEN_PRESENT_POSITION,
                )
                dxl_present_velocity = dynamixel.groupBulkReadGetData(
                    self.group_pos_vel,
                    dxl_id,
                    self.motor.ADDR_PRESENT_VELOCITY,
                    self.motor.LEN_PRESENT_VELOCITY,
                )
                if dxl_present_velocity >= 1024:
                    dxl_present_velocity = -1.0 * (dxl_present_velocity - 1024)

                self.dxl_present_position[i] = POS_SCALE * dxl_present_position
                self.dxl_present_velocity[i] = VEL_SCALE * dxl_present_velocity

        if len(dxl_errored):
            self.okay(motor_id)
            print(
                "groupBulkRead get_pos_vel failed. Sending last known values for dynamixel ids: "
                + str(dxl_errored),
                flush=True,
            )
        return self.dxl_present_position.copy(), self.dxl_present_velocity.copy()

    # Returns pos in radians
    def get_pos(self, motor_id):
        dxl_present_position = []

        # Bulkread present positions
        dynamixel.groupBulkReadTxRxPacket(self.group_pos)

        # Retrieve data
        for dxl_id in motor_id:
            # Get present position value
            dxl_getdata_result = ctypes.c_ubyte(
                dynamixel.groupBulkReadIsAvailable(
                    self.group_pos,
                    dxl_id,
                    self.motor.ADDR_PRESENT_POSITION,
                    self.motor.LEN_PRESENT_POSITION,
                )
            ).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupBulkRead pos_getdata failed" % (dxl_id))
                dxl_present_position.append(0)
                quit()
            else:
                dxl_present_position.append(
                    dynamixel.groupBulkReadGetData(
                        self.group_pos,
                        dxl_id,
                        self.motor.ADDR_PRESENT_POSITION,
                        self.motor.LEN_PRESENT_POSITION,
                    )
                )

        return POS_SCALE * np.array(dxl_present_position)

    # Returns vel in radians/sec
    def get_vel(self, motor_id):
        dxl_present_velocity = []

        dynamixel.groupBulkReadTxRxPacket(self.group_vel)
        for dxl_id in motor_id:
            # Get present velocity value
            dxl_getdata_result = ctypes.c_ubyte(
                dynamixel.groupBulkReadIsAvailable(
                    self.group_vel,
                    dxl_id,
                    self.motor.ADDR_PRESENT_VELOCITY,
                    self.motor.LEN_PRESENT_VELOCITY,
                )
            ).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupBulkRead vel_getdata failed" % (dxl_id))
                dxl_present_velocity.append(0)
                quit()
            else:
                dxl_present_velocity.append(
                    dynamixel.groupBulkReadGetData(
                        self.group_vel,
                        dxl_id,
                        self.motor.ADDR_PRESENT_VELOCITY,
                        self.motor.LEN_PRESENT_VELOCITY,
                    )
                )

        dxl_present_velocity = np.array(dxl_present_velocity)
        for i in range(len(dxl_present_velocity)):
            if dxl_present_velocity[i] >= 1024:
                dxl_present_velocity[i] = -1.0 * (dxl_present_velocity[i] - 1024)

        return VEL_SCALE * dxl_present_velocity

    # Returns pos in radians
    def getIndividual_pos(self, motor_id):
        dxl_present_position = []

        # Read present position and velocity
        for dxl_id in motor_id:
            dxl_present_position.append(
                dynamixel.read2ByteTxRx(
                    self.port_num,
                    self.protocol,
                    dxl_id,
                    self.motor.ADDR_PRESENT_POSITION,
                )
            )
        if not self.okay(motor_id):
            self.close(motor_id)
            quit("error getting self.motor.ADDR_PRESENT_POSITION")
        return POS_SCALE * np.array(dxl_present_position)

    # Returns vel in radians/sec
    def getIndividual_vel(self, motor_id):
        dxl_present_velocity = []
        # Read present position
        for dxl_id in motor_id:
            dxl_present_velocity.append(
                dynamixel.read2ByteTxRx(
                    self.port_num,
                    self.protocol,
                    dxl_id,
                    self.motor.ADDR_PRESENT_VELOCITY,
                )
            )
        if not self.okay(motor_id):
            self.close(motor_id)
            quit("error getting self.motor.ADDR_PRESENT_VELOCITY")

        dxl_present_velocity = np.array(dxl_present_velocity)

        for i in range(len(dxl_present_velocity)):
            if dxl_present_velocity[i] >= 1024:
                dxl_present_velocity[i] = -1.0 * (dxl_present_velocity[i] - 1024)

        return VEL_SCALE * dxl_present_velocity

    # Expects des_pos in radians
    def setIndividual_des_pos(self, motor_id, des_pos_inRadians):
        # if in torque mode, activate position control mode
        des_ctrl_mode = [self.ctrl_mode.get(mid) for mid in motor_id]
        if any(des_ctrl_mode):
            self.torque_control(motor_id, enable=False)
            if not self.okay(motor_id):
                self.close(motor_id)
                quit("error disabling torque control mode")

        # Write goal position
        for i in range(len(motor_id)):
            dynamixel.write4ByteTxRx(
                self.port_num,
                self.protocol,
                motor_id[i],
                self.motor.ADDR_GOAL_POSITION,
                int(des_pos_inRadians[i] / POS_SCALE),
            )
        if not self.okay(motor_id):
            self.close(motor_id)
            quit("error setting ADDR_GOAL_POSITION =====")

    # Expects des_pos in radians (0-2*pi)
    def set_des_pos(self, motor_id, des_pos_inRadians):

        des_pos_inRadians = np.clip(des_pos_inRadians, 0.0, 2 * np.pi)
        # if in torque mode, activate position control mode
        des_ctrl_mode = [self.ctrl_mode.get(mid) for mid in motor_id]
        if any(des_ctrl_mode):
            self.torque_control(motor_id, enable=False)
            if not self.okay(motor_id):
                self.close(motor_id)
                quit("error disabling torque control mode")

        # Write goal position
        for i in range(len(motor_id)):
            dxl_addparam_result = ctypes.c_ubyte(
                dynamixel.groupSyncWriteAddParam(
                    self.group_desPos,
                    motor_id[i],
                    int(des_pos_inRadians[i] / POS_SCALE),
                    self.motor.LEN_GOAL_POSITION,
                )
            ).value
            if dxl_addparam_result != 1:
                print(dxl_addparam_result)
                print("[ID:%03d] groupSyncWrite addparam failed" % (motor_id[i]))
                self.close(motor_id)
                quit()

        # Syncwrite goal position
        dynamixel.groupSyncWriteTxPacket(self.group_desPos)
        if not self.okay(motor_id):
            self.close(motor_id)
            quit("error bulk commanding desired positions")

        # Clear syncwrite parameter storage
        dynamixel.groupSyncWriteClearParam(self.group_desPos)

    # Set desired torques
    def set_des_torque(self, motor_id, des_tor):
        # If in position mode, activate torque mode
        des_ctrl_mode = [self.ctrl_mode.get(mid) for mid in motor_id]
        if not all(des_ctrl_mode):
            self.torque_control(motor_id, enable=True)
            if not self.okay(motor_id):
                self.close(motor_id)
                quit("error enabling torque control mode")

        # Write goal position
        for i in range(len(motor_id)):
            dxl_addparam_result = ctypes.c_ubyte(
                dynamixel.groupSyncWriteAddParam(
                    self.group_desTor,
                    motor_id[i],
                    int(des_tor[i]),
                    self.motor.LEN_GOAL_PWM,
                )
            ).value
            if dxl_addparam_result != 1:
                print(dxl_addparam_result)
                print("[ID:%03d] groupSyncWrite addparam failed" % (motor_id[i]))
                self.close(motor_id)
                quit()

        # Syncwrite goal position
        dynamixel.groupSyncWriteTxPacket(self.group_desTor)
        if not self.okay(motor_id):
            self.close(motor_id)
            quit("error bulk commanding desired torques")

        # Clear syncwrite parameter storage
        dynamixel.groupSyncWriteClearParam(self.group_desTor)

    # Set desired torques
    def setIndividual_des_torque(self, motor_id, des_tor):
        # If in position mode, activate torque mode
        des_ctrl_mode = [self.ctrl_mode.get(mid) for mid in motor_id]
        if not all(des_ctrl_mode):
            self.torque_control(motor_id, enable=True)
            if not self.okay(motor_id):
                self.close(motor_id)
                quit("error enabling torque control mode")

        # Write goal position
        for i in range(len(motor_id)):
            dynamixel.write2ByteTxRx(
                self.port_num,
                self.protocol,
                motor_id[i],
                self.motor.ADDR_GOAL_PWM,
                int(des_tor[i]),
            )
        if not self.okay(motor_id):
            self.close(motor_id)
            quit("error setting ADDR_GOAL_PWM")

    # Set maximum velocity
    def set_max_vel(self, motor_id, max_vel):
        for dxl_id in motor_id:
            dynamixel.write4ByteTxRx(
                self.port_num,
                self.protocol,
                dxl_id,
                self.motor.ADDR_MAX_VELOCITY,
                max_vel,
            )
            if not self.okay(motor_id):
                self.close(motor_id)
                quit("error setting self.motor.ADDR_MAX_VELOCITY")

    # Close connection
    def close(self, motor_id=None):
        if motor_id is None:
            motor_id = self.motor_id

        if self.port_num is not None:
            # Disengage Dynamixels
            self.engage_motor(motor_id, False)

            # Close port
            dynamixel.closePort(self.port_num)
            self.port_num = None  # mark as closed

        return True

    def __del__(self):
        self.close(self.motor_id)