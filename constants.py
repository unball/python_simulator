from Box2D import b2Color

TARGET_FPS = 60
TIME_STEP = 0.01 / TARGET_FPS 
DRAW_GRAPHICS = True

#TARGET_FPS = 60
ZOOM = 5.0                      
FIELD_H = (174)  # (1) Height of the screen pygame with the Box2D 
FIELD_W = (134)  # (2) Width of the screen pygame with the Box2D
CORRECTION_FATOR_METER_TO_CM = 10**2 # <==== Very important: 1 m = 1*10^2 cm
MAX_ROBOTS_ALLIES = 5
# CENTER_AXIS_X = -0
# CENTER_AXIS_Y = 0

# Proprerties of bodies
MASS_BALL = 0.046	# Kg
MASS_ROBOT = 5.100	# Kg (adjust later)

# --- Colors ---
BLUE = b2Color(0,0,1)
YELLOW = b2Color(1,1,0)
WHITE = b2Color(1,1,1)
