# Motor pins
PIN_MOTOR1_LEGA = 8
PIN_MOTOR1_LEGB = 7
PIN_MOTOR2_LEGA = 5
PIN_MOTOR2_LEGB = 6

# IR sensor pins
PIN_IR_LEFT = 14  # Default GPIO Channel for Left   IR Detector
PIN_IR_MIDDLE = 15  # Default GPIO Channel for Middle IR Detector
PIN_IR_RIGHT = 18  # Default GPIO Channel for Right  IR Detector

# Magnetometer pins
PIN_MAG_LATCH = 27
PIN_MAG_ADDRESS = 4
PIN_MAG_READY = 17
PIN_MAG_PIN0 = 9
PIN_MAG_PIN1 = 10
PIN_MAG_PIN2 = 11
PIN_MAG_PIN3 = 12
PIN_MAG_PIN4 = 22
PIN_MAG_PIN5 = 23
PIN_MAG_PIN6 = 24
PIN_MAG_PIN7 = 25
MIN_0, MAX_0 = 75, 221
MIN_1, MAX_1 = 62, 214

# Ultrasound Pins
PIN_US_TRIGGER_L = 13
PIN_US_TRIGGER_M = 19
PIN_US_TRIGGER_R = 26
PIN_US_ECHO_L = 16
PIN_US_ECHO_M = 20
PIN_US_ECHO_R = 21

# Drive choices
STRAIGHT = "STRAIGHT"
VEER_L = "VEER_L"
STEER_L = "STEER_L"
VEER_R = "VEER_R"
STEER_R = "STEER_R"
TURN_L = "TURN_L"
HOOK_L = "HOOK_L"
TURN_R = "TURN_R"
HOOK_R = "HOOK_R"
SPIN_L = "SPIN_L"
SPIN_R = "SPIN_R"

# Change in position table
DX_DY_TABLE = [
    (0, 1),  # Heading 0: North
    (-1, 1),  # Heading 1: Northwest
    (-1, 0),  # Heading 2: West
    (-1, -1),  # Heading 3: Southwest
    (0, -1),  # Heading 4: South
    (1, -1),  # Heading 5: Southeast
    (1, 0),  # Heading 6: East
    (1, 1),  # Heading 7: Northeast
]

# Ultrasound wall following behaviors constants
MAX_THRESHOLD = 0.2
MIN_THRESHOLD = 0.1
NOMINIAL_DISTANCE = 0.3