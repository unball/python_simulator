from Box2D import b2Color

TARGET_FPS = 60
TIME_STEP = 0.01 / TARGET_FPS 

#TARGET_FPS = 60
ZOOM = 5.0                      
FIELD_H = (174)  # (1) Height of the screen pygame with the Box2D 
FIELD_W = (134)  # (2) Width of the screen pygame with the Box2D
CORRECTION_FATOR_METER_TO_CM = 10**2 # <==== Very important: 1 m = 1*10^2 cm
MAX_ROBOTS_ALLIES = 3
CENTER_AXIS_X = -75
CENTER_AXIS_Y = 65

# Proprerties of bodies
MASS_BALL = 0.046	# Kg
MASS_ROBOT = 0,200	# Kg (adjust later)


# --- Colors ---
ORANGE = (255,165,0)
BLUE = (0,0,255)
YELLOW = (255,255,0)
WHITE = b2Color(1, 1, 1)
OPPON_COLOR = (123, 122, 6)
BACKGROUND_COLOR = '#e2873c'
UnBall_blue  = (3, 63, 118) 
UnBall_green = (2, 128, 54) 

