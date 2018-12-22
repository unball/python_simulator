from Box2D import b2Color

# --- Random values ---
OPPONENT = 0
ALLIE = 1

# --- General units ---
LOGO_SIZE = 150, 150
BUTTON_SEPARATOR = (50, 2)

PPM = 5             # Pixel per centimeter               
FIELD_W = (1743/2)  # (1) Width of the screen pygame with the Box2D 
FIELD_H = (1341/2)  # (2) Height of the screen pygame with the Box2D
                    # Both (1) and (2) don't interfere in the bodies inside the field

#TARGET_FPS = 60
TARGET_FPS = 60
TIME_STEP = 0.01 / TARGET_FPS # What does it mean this part ?

# --- Units (cm) ---
ROBOT_W = 7.5 
ROBOT_H = 7.5         
BALL_RADIUS = 2.135
BIG_FIELD_RADIUS = 20    
SMALL_FIELD_RADIUS = 10

# --- Colors ---
ORANGE = (255,165,0)
BLUE = (0,0,255)
YELLOW = (255,255,0)
WHITE = b2Color(1, 1, 1)
OPPON_COLOR = (123, 122, 6)
BACKGROUND_COLOR = '#e2873c'
UnBall_blue  = (3, 63, 118) 
UnBall_green = (2, 128, 54) 

