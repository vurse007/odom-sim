import pygame
import sys
import math

#init pygame
pygame.init()

#constants
WIDTH, HEIGHT = 600, 600
FIELD_COLOR = (30,30,30)
ROBOT_COLOR = (180,180,180)
NIB_COLOR = (255,100,100)
TRACKING_WHEEL_COLOR = (100,255,100)
ODOM_DOT_COLOR = (100,100,255)  # Blue color for odometry position dot
ROBOT_SIZE = 80
MOVE_SPEED = 10
TURN_SPEED = math.radians(7) #radians per frame (5 degrees)

# Field dimensions (12ft x 12ft converted to pixels)
FIELD_SIZE_INCHES = 12 * 12  # 144 inches
PIXELS_PER_INCH = WIDTH / FIELD_SIZE_INCHES  # pixels per inch conversion
INCHES_PER_PIXEL = FIELD_SIZE_INCHES / WIDTH  # inches per pixel

#create screen
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("odom sim ~ two tracker inertial setup")

#starting center position of the robot
robot_x = WIDTH // 2
robot_y = HEIGHT // 2
angle = 0.0 #in radians, 0 = facing right
angular_velocity = 0.0
TURN_ACCEL = math.radians(0.5)

#velocity stuff
vx, vy = 0,0
FRICTION = 0.9
ACCEL_FACTOR = 0.1

# Odometry parameters (in inches)
# Distance from tracking center to vertical encoder (back center)
VERTICAL_ENCODER_OFFSET = 8.0  # back encoder offset (inches)

# Tracking wheel encoder simulation - 2 encoders as specified
horizontal_encoder = 0.0    # accumulated distance in pixels (center, measures x-axis movement)
vertical_encoder = 0.0      # accumulated distance in pixels (back, measures y-axis movement)

# Previous encoder values for delta calculation
prev_horizontal_encoder = 0.0
prev_vertical_encoder = 0.0

# Odometry state
odom_x = 0.0          # position in inches
odom_y = 0.0          # position in inches
odom_theta = 0.0      # orientation from IMU (actual robot angle)

# Font for displaying information
font = pygame.font.Font(None, 24)

def pixels_to_inches(pixels):
    """Convert pixels to inches"""
    return pixels * INCHES_PER_PIXEL

def inches_to_pixels(inches):
    """Convert inches to pixels"""
    return inches * PIXELS_PER_INCH

def get_encoder_values_pixels():
    """Return current encoder values in pixels"""
    return horizontal_encoder, vertical_encoder

def simulate_encoders(old_x, old_y, old_angle, new_x, new_y, new_angle):
    """Simulate encoder readings based on robot movement"""
    global horizontal_encoder, vertical_encoder
    
    # Calculate movement in pixels
    dx_pixels = new_x - old_x
    dy_pixels = new_y - old_y
    
    # Transform global movement to robot's local frame
    cos_angle = math.cos(old_angle)
    sin_angle = math.sin(old_angle)
    
    # Robot's local coordinate system (relative to robot's orientation):
    # local_x = sideways movement (positive = right)
    # local_y = forward movement (positive = forward)
    local_x = dx_pixels * cos_angle + dy_pixels * sin_angle
    local_y = -dx_pixels * sin_angle + dy_pixels * cos_angle
    
    # Horizontal encoder (at center) measures side-to-side movement
    horizontal_encoder += local_x
    
    # Vertical encoder (at back) measures forward/backward movement
    # Also affected by rotation around the tracking center due to offset
    dtheta = new_angle - old_angle
    vertical_encoder += local_y + (VERTICAL_ENCODER_OFFSET * PIXELS_PER_INCH * dtheta)

def draw_bot(screen, x, y, angle):
    robot_surface = pygame.Surface((ROBOT_SIZE, ROBOT_SIZE), pygame.SRCALPHA)
    robot_surface.fill(ROBOT_COLOR)

    # nib (points upward as forward)
    center = (ROBOT_SIZE // 2, ROBOT_SIZE // 2)
    nib_length = 10
    nib_tip = (center[0], center[1] - nib_length)
    left = (center[0] - 4, center[1])
    right = (center[0] + 4, center[1])
    pygame.draw.polygon(robot_surface, NIB_COLOR, [nib_tip, left, right])
    
    # Draw tracking wheels as specified
    # Horizontal encoder wheel (exact center) - measures x-axis movement
    horizontal_wheel_x = center[0]
    horizontal_wheel_y = center[1]
    pygame.draw.circle(robot_surface, TRACKING_WHEEL_COLOR, (horizontal_wheel_x, horizontal_wheel_y), 3)
    
    # Vertical encoder wheel (back center) - measures y-axis movement
    vertical_wheel_x = center[0]
    vertical_wheel_y = center[1] + int(VERTICAL_ENCODER_OFFSET * PIXELS_PER_INCH / 2)
    pygame.draw.circle(robot_surface, TRACKING_WHEEL_COLOR, (vertical_wheel_x, vertical_wheel_y), 3)

    angle_degrees = math.degrees(angle)
    rotated = pygame.transform.rotate(robot_surface, -angle_degrees)
    rect = rotated.get_rect(center=(x, y))
    screen.blit(rotated, rect.topleft)

def draw_odom_position(screen, odom_x_inches, odom_y_inches):
    """Draw a blue dot representing the odometry position"""
    # Convert field coordinates to screen coordinates
    # Field center (0,0) is at screen center
    screen_center_x = WIDTH // 2
    screen_center_y = HEIGHT // 2
    
    # Convert odometry position from inches to pixels and adjust for screen coordinates
    odom_screen_x = screen_center_x + inches_to_pixels(odom_x_inches)
    odom_screen_y = screen_center_y - inches_to_pixels(odom_y_inches)  # Y-axis is flipped in screen coordinates
    
    # Draw the blue dot (only if it's within screen bounds)
    if 0 <= odom_screen_x <= WIDTH and 0 <= odom_screen_y <= HEIGHT:
        pygame.draw.circle(screen, ODOM_DOT_COLOR, (int(odom_screen_x), int(odom_screen_y)), 5)

def get_coordinates(): #coordinates are in pixels and theta is in degrees
    while True:
        try:
            coords = input("Enter new coordinates as x,y,theta (e.g. 300,200,90 - theta in degs, coords in pxls): ")
            x_str, y_str, theta_str = coords.split(",")
            new_x = float(x_str.strip())
            new_y = float(y_str.strip())
            theta = float(theta_str.strip())
            return new_x, new_y, math.radians(theta)
        except Exception:
            print("Invalid input, please enter coordinates as x,y,theta")

# ~ odom code ~ #

def odom_reset(set_x_inches=0.0, set_y_inches=0.0, set_theta=None):
    """Reset odometry to specified position"""
    global odom_x, odom_y, odom_theta
    global prev_horizontal_encoder, prev_vertical_encoder
    
    odom_x = set_x_inches
    odom_y = set_y_inches
    odom_theta = set_theta if set_theta is not None else angle
    
    # Reset previous encoder values to current encoder values
    prev_horizontal_encoder = horizontal_encoder
    prev_vertical_encoder = vertical_encoder

def odom_update(horizontal_pixels, vertical_pixels):  # Encoder values in pixels
    global prev_horizontal_encoder, prev_vertical_encoder, odom_x, odom_y, odom_theta
    
    # Step 1: Get current and previous theta from IMU
    prev_theta = odom_theta
    odom_theta = angle  # Reading from IMU (global angle variable)

    # Step 2: Calculate encoder deltas in pixels, then convert to inches
    delta_horizontal_pixels = horizontal_pixels - prev_horizontal_encoder
    delta_vertical_pixels = vertical_pixels - prev_vertical_encoder
    
    delta_horizontal_inches = pixels_to_inches(delta_horizontal_pixels)
    delta_vertical_inches = pixels_to_inches(delta_vertical_pixels)

    # Step 3: Calculate change in angle
    dtheta = odom_theta - prev_theta

    # Step 4: Apply vertical encoder offset correction
    # The vertical encoder is offset from the tracking center, so rotation affects its reading
    corrected_vertical_inches = delta_vertical_inches - (VERTICAL_ENCODER_OFFSET * dtheta)

    # Step 5: Get average theta for the movement
    avg_theta = (prev_theta + odom_theta) / 2

    # Step 6: Transform local encoder movements to global coordinates
    cos_avg = math.cos(avg_theta)
    sin_avg = math.sin(avg_theta)
    
    # Transform from robot's local frame to global frame
    # delta_horizontal_inches = local x movement (sideways)
    # corrected_vertical_inches = local y movement (forward/back) after offset correction
    # Note: Using screen coordinate system where positive Y is down
    global_dx = delta_horizontal_inches * cos_avg - corrected_vertical_inches * sin_avg
    global_dy = -(delta_horizontal_inches * sin_avg + corrected_vertical_inches * cos_avg)
    # ADD A NEGATIVE SIGN ABOVE BECAUSE PYGAME Y AXIS IS MIRRORED TO IRL, SO WE NEED TO MULTIPLY BY NEGATIVE ONE TO DISPLAY IT PROPERLY

    # Step 7: Update global position
    odom_x += global_dx
    odom_y += global_dy

    # Step 8: Update previous encoder values
    prev_horizontal_encoder = horizontal_pixels
    prev_vertical_encoder = vertical_pixels

#reset the odom for the simulation
odom_reset()

#game loop
clock = pygame.time.Clock()
running = True

while running:
    clock.tick(60) #limit to 60 fps

    # Store old position for encoder simulation
    old_x, old_y, old_angle = robot_x, robot_y, angle

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN:
                # Pause simulation and get new coordinates
                new_x, new_y, new_theta = get_coordinates()
                
                # Clamp the coordinates inside the screen bounds
                half = ROBOT_SIZE // 2
                robot_x = max(half, min(WIDTH - half, new_x))
                robot_y = max(half, min(HEIGHT - half, new_y))

                angle = new_theta
                
                # Reset odometry with proper units
                odom_reset(pixels_to_inches(new_x), pixels_to_inches(new_y), new_theta)

                # Reset velocity so it doesn't drift away immediately
                vx, vy = 0.0, 0.0
                angular_velocity = 0.0
                
            elif event.key == pygame.K_e:
                # Print encoder values in pixels
                h_pixels, v_pixels = get_encoder_values_pixels()
                print(f"Encoder values in pixels - Horizontal (X): {h_pixels:.2f}, Vertical (Y): {v_pixels:.2f}")
                print(f"Odometry position in inches: ({odom_x:.2f}, {odom_y:.2f})")
    
    # key pressing detection
    keys = pygame.key.get_pressed()

    # Calculate forward vector (angle 0 = facing up)
    dx = math.sin(angle)
    dy = -math.cos(angle)

    # Apply velocity from movement keys
    if keys[pygame.K_w]:
        vx += MOVE_SPEED * dx * ACCEL_FACTOR
        vy += MOVE_SPEED * dy * ACCEL_FACTOR

    if keys[pygame.K_s]:
        vx -= MOVE_SPEED * dx * ACCEL_FACTOR
        vy -= MOVE_SPEED * dy * ACCEL_FACTOR

    if keys[pygame.K_a]:
        angular_velocity -= TURN_ACCEL
    if keys[pygame.K_d]:
        angular_velocity += TURN_ACCEL

    # Apply velocity and friction (drift)
    robot_x += vx
    robot_y += vy

    vx *= FRICTION
    vy *= FRICTION
    angular_velocity *= FRICTION
    angle += angular_velocity

    #keep in bounds
    half = ROBOT_SIZE // 2
    robot_x = max(half, min(WIDTH - half, robot_x))
    robot_y = max(half, min(HEIGHT - half, robot_y))

    # Simulate encoders based on movement
    simulate_encoders(old_x, old_y, old_angle, robot_x, robot_y, angle)
    
    # Update odometry
    h_pixels, v_pixels = get_encoder_values_pixels()
    odom_update(h_pixels, v_pixels)
    
    #draw field
    screen.fill(FIELD_COLOR)
    
    # Draw field grid (every 2 feet = 24 inches)
    grid_spacing_inches = 24
    grid_spacing_pixels = inches_to_pixels(grid_spacing_inches)
    
    for x in range(0, WIDTH + 1, int(grid_spacing_pixels)):
        pygame.draw.line(screen, (255,255,255), (x,0), (x,HEIGHT), 1)
    for y in range(0, HEIGHT + 1, int(grid_spacing_pixels)):
        pygame.draw.line(screen, (255,255,255), (0,y), (WIDTH,y), 1)
    
    # Draw center lines
    pygame.draw.line(screen, (255,255,0), (WIDTH//2, 0), (WIDTH//2, HEIGHT), 2)
    pygame.draw.line(screen, (255,255,0), (0, HEIGHT//2), (WIDTH, HEIGHT//2), 2)
    
    # Draw robot first
    draw_bot(screen, robot_x, robot_y, angle)
    
    # Draw odometry position as blue dot ON TOP of robot
    draw_odom_position(screen, odom_x, odom_y)
    
    # Display information on screen
    encoder_text = font.render(f"Encoders (px): H={h_pixels:.1f}, V={v_pixels:.1f}", True, (255,255,255))
    odom_text = font.render(f"Odom pos (in): ({odom_x:.1f}, {odom_y:.1f})", True, (255,255,255))
    # Convert actual position to inches for display
    actual_x_inches = pixels_to_inches(robot_x - WIDTH//2)
    actual_y_inches = pixels_to_inches(HEIGHT//2 - robot_y)  # Flip Y coordinate
    actual_text = font.render(f"Actual pos (in): ({actual_x_inches:.1f}, {actual_y_inches:.1f})", True, (255,255,255))
    angle_text = font.render(f"Angle: {math.degrees(angle):.1f}°", True, (255,255,255))
    
    screen.blit(encoder_text, (10, 10))
    screen.blit(odom_text, (10, 30))
    screen.blit(actual_text, (10, 50))
    screen.blit(angle_text, (10, 70))
    
    # Display controls
    controls_text = font.render("WASD: Move, Enter: Set pos, E: Print encoders", True, (255,255,255))
    screen.blit(controls_text, (10, HEIGHT - 30))

    pygame.display.flip()

#quit pygame
pygame.quit()
sys.exit()