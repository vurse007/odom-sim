import pygame
import sys
import math

# Init pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 600, 600
FIELD_COLOR = (30, 30, 30)
ROBOT_COLOR = (180, 180, 180)
GHOST_COLOR = (180, 180, 180, 100)  # Semi-transparent
NIB_COLOR = (255, 100, 100)
GHOST_NIB_COLOR = (100, 255, 100)
TRACKING_WHEEL_COLOR = (100, 255, 100)
ODOM_DOT_COLOR = (100, 100, 255)
ROBOT_SIZE = 80
TURN_SPEED = math.radians(7)

# Field dimensions (12ft x 12ft converted to pixels)
FIELD_SIZE_INCHES = 12 * 12  # 144 inches
PIXELS_PER_INCH = WIDTH / FIELD_SIZE_INCHES
INCHES_PER_PIXEL = FIELD_SIZE_INCHES / WIDTH

# Create screen
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Boomerang Controller - Robot Simulator")

# Robot state
robot_x = WIDTH // 2
robot_y = HEIGHT // 2
angle = 0.0  # In radians, 0 = facing up
robot_vx, robot_vy = 0.0, 0.0
robot_angular_velocity = 0.0

# Ghost robot state
ghost_x = WIDTH // 2 + 100
ghost_y = HEIGHT // 2
ghost_angle = 0.0
ghost_mode = True  # Start in ghost control mode

# Odometry parameters (in inches)
HORIZONTAL_ENCODER_OFFSET = -8.0
VERTICAL_ENCODER_OFFSET = 0.0

# Tracking wheel encoder simulation
horizontal_encoder = 0.0
vertical_encoder = 0.0
prev_horizontal_encoder = 0.0
prev_vertical_encoder = 0.0

# Odometry state
odom_x = 0.0
odom_y = 0.0
odom_theta = 0.0

# Boomerang controller parameters
target_x = None
target_y = None
target_theta = None
autonomous_mode = False

# Boomerang controller gains
LINEAR_KP = 0.06
ANGULAR_KP = 0.15
CARROT_DISTANCE = 20.0  # Distance ahead to look (in inches)
GLEAD_GAIN = 0.7  # How much to bias toward target angle (0 = pure pursuit, 1 = always face target angle)
MIN_SPEED = 0.2
MAX_SPEED = 6.0
MAX_ANGULAR_SPEED = 0.12
ARRIVAL_THRESHOLD = 1.5  # inches
ANGLE_THRESHOLD = math.radians(3)  # radians

# Font for displaying information
font = pygame.font.Font(None, 24)


def pixels_to_inches(pixels):
    """Convert pixels to inches"""
    return pixels * INCHES_PER_PIXEL


def inches_to_pixels(inches):
    """Convert inches to pixels"""
    return inches * PIXELS_PER_INCH


def simulate_encoders(old_x, old_y, old_angle, new_x, new_y, new_angle):
    """Simulate encoder readings based on robot movement"""
    global horizontal_encoder, vertical_encoder
    
    dx_pixels = new_x - old_x
    dy_pixels = new_y - old_y
    
    cos_angle = math.cos(old_angle)
    sin_angle = math.sin(old_angle)
    
    local_x = dx_pixels * cos_angle + dy_pixels * sin_angle
    local_y = -dx_pixels * sin_angle + dy_pixels * cos_angle
    
    dtheta = new_angle - old_angle
    
    horizontal_encoder += local_x - (HORIZONTAL_ENCODER_OFFSET * PIXELS_PER_INCH * dtheta)
    vertical_encoder += local_y + (VERTICAL_ENCODER_OFFSET * PIXELS_PER_INCH * dtheta)


def odom_reset(set_x_inches=0.0, set_y_inches=0.0, set_theta=None):
    """Reset odometry to specified position"""
    global odom_x, odom_y, odom_theta
    global prev_horizontal_encoder, prev_vertical_encoder
    
    odom_x = set_x_inches
    odom_y = set_y_inches
    odom_theta = set_theta if set_theta is not None else angle
    
    prev_horizontal_encoder = horizontal_encoder
    prev_vertical_encoder = vertical_encoder


def odom_update(horizontal_pixels, vertical_pixels):
    """Update odometry based on encoder values"""
    global prev_horizontal_encoder, prev_vertical_encoder, odom_x, odom_y, odom_theta
    
    prev_theta = odom_theta
    odom_theta = angle
    
    delta_horizontal_pixels = horizontal_pixels - prev_horizontal_encoder
    delta_vertical_pixels = vertical_pixels - prev_vertical_encoder
    
    delta_horizontal_inches = pixels_to_inches(delta_horizontal_pixels)
    delta_vertical_inches = pixels_to_inches(delta_vertical_pixels)
    
    dtheta = odom_theta - prev_theta
    
    corrected_horizontal_inches = delta_horizontal_inches + (HORIZONTAL_ENCODER_OFFSET * dtheta)
    corrected_vertical_inches = delta_vertical_inches - (VERTICAL_ENCODER_OFFSET * dtheta)
    
    avg_theta = (prev_theta + odom_theta) / 2
    
    cos_avg = math.cos(avg_theta)
    sin_avg = math.sin(avg_theta)
    
    global_dx = corrected_horizontal_inches * cos_avg - corrected_vertical_inches * sin_avg
    global_dy = -(corrected_horizontal_inches * sin_avg + corrected_vertical_inches * cos_avg)
    
    odom_x += global_dx
    odom_y += global_dy
    
    prev_horizontal_encoder = horizontal_pixels
    prev_vertical_encoder = vertical_pixels


def boomerang_controller(robot_x_in, robot_y_in, robot_theta, target_x_in, target_y_in, target_theta):
    """
    Pure g-lead boomerang controller that drives robot to target pose
    G-lead blends between following the path and orienting toward the target angle
    Returns: (linear_velocity, angular_velocity) in inches/frame and radians/frame
    """
    # Calculate distance to target
    dx = target_x_in - robot_x_in
    dy = target_y_in - robot_y_in
    distance = math.sqrt(dx * dx + dy * dy)
    
    # Calculate angle error to target orientation
    angle_error_to_target = angle_difference(target_theta, robot_theta)
    
    # Check if we've arrived - both position AND angle must be good
    if distance < ARRIVAL_THRESHOLD and abs(angle_error_to_target) < ANGLE_THRESHOLD:
        return 0.0, 0.0  # Arrived!
    
    # Calculate the approach point - a point behind the target in the opposite direction of target heading
    approach_distance = 12.0  # How far back from target to aim for (inches)
    approach_x = target_x_in - approach_distance * math.sin(target_theta)
    approach_y = target_y_in - approach_distance * math.cos(target_theta)
    
    # Calculate vector to approach point
    dx_approach = approach_x - robot_x_in
    dy_approach = approach_y - robot_y_in
    distance_to_approach = math.sqrt(dx_approach * dx_approach + dy_approach * dy_approach)
    
    # Calculate carrot point toward the approach point
    carrot_distance = min(CARROT_DISTANCE, distance_to_approach)
    if distance_to_approach > 0.1:  # Avoid division by zero
        carrot_x = robot_x_in + (dx_approach / distance_to_approach) * carrot_distance
        carrot_y = robot_y_in + (dy_approach / distance_to_approach) * carrot_distance
    else:
        carrot_x = robot_x_in
        carrot_y = robot_y_in
    
    # Calculate angle to carrot point (pure pursuit component)
    angle_to_carrot = math.atan2(carrot_x - robot_x_in, carrot_y - robot_y_in)
    
    # G-lead: blend between carrot angle and target angle based on distance
    # As we get closer, bias more toward the target angle
    distance_factor = min(1.0, distance / 40.0)  # Normalize distance (closer = smaller factor)
    glead_factor = GLEAD_GAIN * (1.0 - distance_factor)  # More g-lead when closer
    
    # Blend the angles using g-lead
    target_angle_to_follow = angle_to_carrot + glead_factor * angle_difference(target_theta, angle_to_carrot)
    
    # Calculate angle error
    angle_error = angle_difference(target_angle_to_follow, robot_theta)
    
    # Calculate linear velocity with better damping
    # Reduce speed more aggressively based on both angle error and distance
    angle_penalty = max(0.2, 1.0 - (abs(angle_error) / math.pi) * 1.5)
    
    # Smooth distance-based speed scaling with more aggressive slowdown
    if distance < 8.0:
        distance_factor_speed = distance / 8.0
    else:
        distance_factor_speed = 1.0
    
    linear_vel = LINEAR_KP * distance * angle_penalty * distance_factor_speed
    
    # Dynamic speed limits based on how close we are
    if distance > ARRIVAL_THRESHOLD * 2:
        linear_vel = max(MIN_SPEED, min(MAX_SPEED, linear_vel))
    else:
        # Very close - tighter speed control
        max_close_speed = min(MAX_SPEED, distance * 0.8)  # Speed proportional to distance
        if abs(angle_error_to_target) > ANGLE_THRESHOLD:
            linear_vel = max(0.15, min(max_close_speed, linear_vel))
        else:
            linear_vel = min(max_close_speed, linear_vel)
    
    # Calculate angular velocity with damping when very close
    angular_vel = ANGULAR_KP * angle_error
    
    # Reduce angular velocity when very close to prevent overshoot
    if distance < 3.0:
        angular_vel *= 0.7
    
    angular_vel = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angular_vel))
    
    return linear_vel, angular_vel


def angle_difference(target_angle, current_angle):
    """Calculate the shortest angle difference between two angles"""
    diff = target_angle - current_angle
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff


def draw_bot(screen, x, y, angle, color, nib_color):
    """Draw a robot at the specified position"""
    robot_surface = pygame.Surface((ROBOT_SIZE, ROBOT_SIZE), pygame.SRCALPHA)
    robot_surface.fill(color)
    
    # Draw nib
    center = (ROBOT_SIZE // 2, ROBOT_SIZE // 2)
    nib_length = 10
    nib_tip = (center[0], center[1] - nib_length)
    left = (center[0] - 4, center[1])
    right = (center[0] + 4, center[1])
    pygame.draw.polygon(robot_surface, nib_color, [nib_tip, left, right])
    
    angle_degrees = math.degrees(angle)
    rotated = pygame.transform.rotate(robot_surface, -angle_degrees)
    rect = rotated.get_rect(center=(x, y))
    screen.blit(rotated, rect.topleft)


def draw_odom_position(screen, odom_x_inches, odom_y_inches):
    """Draw a blue dot representing the odometry position"""
    screen_center_x = WIDTH // 2
    screen_center_y = HEIGHT // 2
    
    odom_screen_x = screen_center_x + inches_to_pixels(odom_x_inches)
    odom_screen_y = screen_center_y - inches_to_pixels(odom_y_inches)
    
    if 0 <= odom_screen_x <= WIDTH and 0 <= odom_screen_y <= HEIGHT:
        pygame.draw.circle(screen, ODOM_DOT_COLOR, (int(odom_screen_x), int(odom_screen_y)), 5)


# Reset the odometry
odom_reset()

# Game loop
clock = pygame.time.Clock()
running = True

while running:
    dt = clock.tick(60) / 1000.0  # Delta time in seconds
    
    old_x, old_y, old_angle = robot_x, robot_y, angle
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN:
                if ghost_mode:
                    # Set target from ghost position and start autonomous mode
                    target_x = pixels_to_inches(ghost_x - WIDTH // 2)
                    target_y = pixels_to_inches(HEIGHT // 2 - ghost_y)
                    target_theta = ghost_angle
                    autonomous_mode = True
                    ghost_mode = False
                    print(f"Target set: ({target_x:.1f}, {target_y:.1f}) @ {math.degrees(target_theta):.1f}°")
            elif event.key == pygame.K_ESCAPE:
                # Cancel autonomous mode and return to ghost mode
                autonomous_mode = False
                ghost_mode = True
                target_x = None
                target_y = None
                target_theta = None
    
    keys = pygame.key.get_pressed()
    
    if ghost_mode:
        # Control ghost robot
        ghost_speed = 5.0
        ghost_turn_speed = math.radians(3)
        
        if keys[pygame.K_w]:
            ghost_x += ghost_speed * math.sin(ghost_angle)
            ghost_y -= ghost_speed * math.cos(ghost_angle)
        if keys[pygame.K_s]:
            ghost_x -= ghost_speed * math.sin(ghost_angle)
            ghost_y += ghost_speed * math.cos(ghost_angle)
        if keys[pygame.K_a]:
            ghost_angle -= ghost_turn_speed
        if keys[pygame.K_d]:
            ghost_angle += ghost_turn_speed
        
        # Keep ghost in bounds
        half = ROBOT_SIZE // 2
        ghost_x = max(half, min(WIDTH - half, ghost_x))
        ghost_y = max(half, min(HEIGHT - half, ghost_y))
    
    elif autonomous_mode:
        # Run boomerang controller
        robot_x_in = pixels_to_inches(robot_x - WIDTH // 2)
        robot_y_in = pixels_to_inches(HEIGHT // 2 - robot_y)
        
        linear_vel, angular_vel = boomerang_controller(
            robot_x_in, robot_y_in, angle,
            target_x, target_y, target_theta
        )
        
        # Check if arrived
        if linear_vel == 0.0 and angular_vel == 0.0:
            print("Arrived at target!")
            autonomous_mode = False
            ghost_mode = True
        
        # Apply velocities (convert from inches to pixels)
        robot_x += linear_vel * PIXELS_PER_INCH * math.sin(angle)
        robot_y -= linear_vel * PIXELS_PER_INCH * math.cos(angle)
        angle += angular_vel
        
        # Keep in bounds
        half = ROBOT_SIZE // 2
        robot_x = max(half, min(WIDTH - half, robot_x))
        robot_y = max(half, min(HEIGHT - half, robot_y))
    
    # Simulate encoders and update odometry
    simulate_encoders(old_x, old_y, old_angle, robot_x, robot_y, angle)
    odom_update(horizontal_encoder, vertical_encoder)
    
    # Draw field
    screen.fill(FIELD_COLOR)
    
    # Draw grid
    grid_spacing_inches = 24
    grid_spacing_pixels = inches_to_pixels(grid_spacing_inches)
    
    for x in range(0, WIDTH + 1, int(grid_spacing_pixels)):
        pygame.draw.line(screen, (255, 255, 255), (x, 0), (x, HEIGHT), 1)
    for y in range(0, HEIGHT + 1, int(grid_spacing_pixels)):
        pygame.draw.line(screen, (255, 255, 255), (0, y), (WIDTH, y), 1)
    
    # Draw center lines
    pygame.draw.line(screen, (255, 255, 0), (WIDTH // 2, 0), (WIDTH // 2, HEIGHT), 2)
    pygame.draw.line(screen, (255, 255, 0), (0, HEIGHT // 2), (WIDTH, HEIGHT // 2), 2)
    
    # Draw target position if set
    if target_x is not None:
        target_screen_x = WIDTH // 2 + inches_to_pixels(target_x)
        target_screen_y = HEIGHT // 2 - inches_to_pixels(target_y)
        pygame.draw.circle(screen, (255, 0, 255), (int(target_screen_x), int(target_screen_y)), 8, 2)
        # Draw target angle line
        line_length = 30
        end_x = target_screen_x + line_length * math.sin(target_theta)
        end_y = target_screen_y - line_length * math.cos(target_theta)
        pygame.draw.line(screen, (255, 0, 255), (target_screen_x, target_screen_y), (end_x, end_y), 2)
    
    # Draw robot
    draw_bot(screen, robot_x, robot_y, angle, ROBOT_COLOR, NIB_COLOR)
    
    # Draw ghost if in ghost mode
    if ghost_mode:
        draw_bot(screen, ghost_x, ghost_y, ghost_angle, GHOST_COLOR, GHOST_NIB_COLOR)
    
    # Draw odometry position
    draw_odom_position(screen, odom_x, odom_y)
    
    # Display information
    actual_x_inches = pixels_to_inches(robot_x - WIDTH // 2)
    actual_y_inches = pixels_to_inches(HEIGHT // 2 - robot_y)
    
    mode_text = "GHOST MODE - Position ghost and press ENTER" if ghost_mode else "AUTONOMOUS MODE - Press ESC to cancel"
    mode_surface = font.render(mode_text, True, (255, 255, 100))
    screen.blit(mode_surface, (10, 10))
    
    odom_text = font.render(f"Odom pos (in): ({odom_x:.1f}, {odom_y:.1f})", True, (255, 255, 255))
    actual_text = font.render(f"Actual pos (in): ({actual_x_inches:.1f}, {actual_y_inches:.1f})", True, (255, 255, 255))
    angle_text = font.render(f"Angle: {math.degrees(angle):.1f}°", True, (255, 255, 255))
    
    screen.blit(odom_text, (10, 35))
    screen.blit(actual_text, (10, 55))
    screen.blit(angle_text, (10, 75))
    
    if target_x is not None:
        target_text = font.render(f"Target: ({target_x:.1f}, {target_y:.1f}) @ {math.degrees(target_theta):.1f}°", True, (255, 0, 255))
        screen.blit(target_text, (10, 95))
    
    # Display controls
    controls_text = font.render("WASD: Move/Control, ENTER: Confirm, ESC: Cancel", True, (255, 255, 255))
    screen.blit(controls_text, (10, HEIGHT - 30))
    
    pygame.display.flip()

pygame.quit()
sys.exit()