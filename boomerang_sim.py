import pygame
import sys
import math

# Import odometry module
from odom_sim import (
    Odometry,
    wrap_to_pi,
    to_deg,
    to_rad,
    pixels_to_inches,
    inches_to_pixels,
    draw_grid,
    draw_robot,
    draw_odom_dot,
    WIDTH,
    HEIGHT,
    PIXELS_PER_INCH,
    INCHES_PER_PIXEL,
)

# ==========================================================================
#  PYGAME INIT & CONSTANTS
# ==========================================================================

pygame.init()

FIELD_COLOR = (30, 30, 30)
ROBOT_COLOR = (180, 180, 180)
GHOST_COLOR = (180, 180, 180, 100)
NIB_COLOR = (255, 100, 100)
GHOST_NIB_COLOR = (100, 255, 100)
ODOM_DOT_COLOR = (100, 100, 255)
TARGET_COLOR = (255, 0, 255)
ROBOT_SIZE = 80

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Boomerang Controller Simulator")
font = pygame.font.Font(None, 24)

# ==========================================================================
#  ROBOT STATE
# ==========================================================================

robot_x = WIDTH // 2
robot_y = HEIGHT // 2
angle = 0.0

# Ghost robot state
ghost_x = WIDTH // 2 + 100
ghost_y = HEIGHT // 2
ghost_angle = 0.0
ghost_mode = True

# Target state
target_x = None
target_y = None
target_theta = None
autonomous_mode = False

# Create odometry instance
odom = Odometry()
odom.reset(angle)

# ==========================================================================
#  BOOMERANG CONTROLLER PARAMETERS
# ==========================================================================

LINEAR_KP = 0.06
ANGULAR_KP = 0.15
CARROT_DISTANCE = 20.0
GLEAD_GAIN = 0.7
MIN_SPEED = 0.2
MAX_SPEED = 6.0
MAX_ANGULAR_SPEED = 0.12
ARRIVAL_THRESHOLD = 1.5
ANGLE_THRESHOLD = math.radians(3)

# ==========================================================================
#  BOOMERANG CONTROLLER
# ==========================================================================

def angle_difference(target_angle, current_angle):
    """Calculate the shortest angle difference between two angles"""
    diff = target_angle - current_angle
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff


def boomerang_controller(robot_x_in, robot_y_in, robot_theta, target_x_in, target_y_in, target_theta):
    """
    Pure g-lead boomerang controller that drives robot to target pose.
    Returns: (linear_velocity, angular_velocity) in inches/frame and radians/frame
    """
    # Calculate distance to target
    dx = target_x_in - robot_x_in
    dy = target_y_in - robot_y_in
    distance = math.sqrt(dx * dx + dy * dy)
    
    # Calculate angle error to target orientation
    angle_error_to_target = angle_difference(target_theta, robot_theta)
    
    # Check if we've arrived
    if distance < ARRIVAL_THRESHOLD and abs(angle_error_to_target) < ANGLE_THRESHOLD:
        return 0.0, 0.0
    
    # Calculate the approach point
    approach_distance = 12.0
    approach_x = target_x_in - approach_distance * math.sin(target_theta)
    approach_y = target_y_in - approach_distance * math.cos(target_theta)
    
    # Calculate vector to approach point
    dx_approach = approach_x - robot_x_in
    dy_approach = approach_y - robot_y_in
    distance_to_approach = math.sqrt(dx_approach * dx_approach + dy_approach * dy_approach)
    
    # Calculate carrot point
    carrot_distance = min(CARROT_DISTANCE, distance_to_approach)
    if distance_to_approach > 0.1:
        carrot_x = robot_x_in + (dx_approach / distance_to_approach) * carrot_distance
        carrot_y = robot_y_in + (dy_approach / distance_to_approach) * carrot_distance
    else:
        carrot_x = robot_x_in
        carrot_y = robot_y_in
    
    # Calculate angle to carrot point
    angle_to_carrot = math.atan2(carrot_x - robot_x_in, carrot_y - robot_y_in)
    
    # G-lead blending
    distance_factor = min(1.0, distance / 40.0)
    glead_factor = GLEAD_GAIN * (1.0 - distance_factor)
    
    target_angle_to_follow = angle_to_carrot + glead_factor * angle_difference(target_theta, angle_to_carrot)
    
    # Calculate angle error
    angle_error = angle_difference(target_angle_to_follow, robot_theta)
    
    # Calculate linear velocity
    angle_penalty = max(0.2, 1.0 - (abs(angle_error) / math.pi) * 1.5)
    
    if distance < 8.0:
        distance_factor_speed = distance / 8.0
    else:
        distance_factor_speed = 1.0
    
    linear_vel = LINEAR_KP * distance * angle_penalty * distance_factor_speed
    
    # Speed limits
    if distance > ARRIVAL_THRESHOLD * 2:
        linear_vel = max(MIN_SPEED, min(MAX_SPEED, linear_vel))
    else:
        max_close_speed = min(MAX_SPEED, distance * 0.8)
        if abs(angle_error_to_target) > ANGLE_THRESHOLD:
            linear_vel = max(0.15, min(max_close_speed, linear_vel))
        else:
            linear_vel = min(max_close_speed, linear_vel)
    
    # Calculate angular velocity
    angular_vel = ANGULAR_KP * angle_error
    
    if distance < 3.0:
        angular_vel *= 0.7
    
    angular_vel = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angular_vel))
    
    return linear_vel, angular_vel

# ==========================================================================
#  DRAWING HELPERS
# ==========================================================================

def draw_target(surface, target_x_in, target_y_in, target_theta):
    """Draw target position and orientation"""
    target_screen_x = WIDTH // 2 + inches_to_pixels(target_x_in)
    target_screen_y = HEIGHT // 2 - inches_to_pixels(target_y_in)
    
    # Draw target circle
    pygame.draw.circle(surface, TARGET_COLOR, (int(target_screen_x), int(target_screen_y)), 8, 2)
    
    # Draw target angle line
    line_length = 30
    end_x = target_screen_x + line_length * math.sin(target_theta)
    end_y = target_screen_y - line_length * math.cos(target_theta)
    pygame.draw.line(surface, TARGET_COLOR, (target_screen_x, target_screen_y), (end_x, end_y), 2)

# ==========================================================================
#  MAIN LOOP
# ==========================================================================

clock = pygame.time.Clock()
running = True

while running:
    clock.tick(60)
    
    old_x, old_y, old_angle = robot_x, robot_y, angle
    
    # --- EVENT HANDLING ---
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN:
                if ghost_mode:
                    # Set target from ghost position
                    target_x = pixels_to_inches(ghost_x - WIDTH // 2)
                    target_y = pixels_to_inches(HEIGHT // 2 - ghost_y)
                    target_theta = ghost_angle
                    autonomous_mode = True
                    ghost_mode = False
                    print(f"Target set: ({target_x:.1f}, {target_y:.1f}) @ {to_deg(target_theta):.1f}°")
            
            elif event.key == pygame.K_ESCAPE:
                # Cancel autonomous mode
                autonomous_mode = False
                ghost_mode = True
                target_x = None
                target_y = None
                target_theta = None
            
            elif event.key == pygame.K_r:
                # Reset everything
                robot_x = WIDTH // 2
                robot_y = HEIGHT // 2
                angle = 0.0
                ghost_x = WIDTH // 2 + 100
                ghost_y = HEIGHT // 2
                ghost_angle = 0.0
                ghost_mode = True
                autonomous_mode = False
                target_x = None
                target_y = None
                target_theta = None
                odom.reset(angle)
            
            elif event.key == pygame.K_e:
                print(f"--- DEBUG ---")
                print(f"Odom (in): X={odom.cpp_x:.2f}, Y={odom.cpp_y:.2f}")
                print(f"Angle: {to_deg(angle):.1f}°")
    
    keys = pygame.key.get_pressed()
    
    # --- GHOST MODE ---
    if ghost_mode:
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
    
    # --- AUTONOMOUS MODE ---
    elif autonomous_mode:
        # Get robot position in inches
        robot_x_in = pixels_to_inches(robot_x - WIDTH // 2)
        robot_y_in = pixels_to_inches(HEIGHT // 2 - robot_y)
        
        # Run boomerang controller
        linear_vel, angular_vel = boomerang_controller(
            robot_x_in, robot_y_in, angle,
            target_x, target_y, target_theta
        )
        
        # Check if arrived
        if linear_vel == 0.0 and angular_vel == 0.0:
            print("Arrived at target!")
            autonomous_mode = False
            ghost_mode = True
        
        # Apply velocities
        robot_x += linear_vel * PIXELS_PER_INCH * math.sin(angle)
        robot_y -= linear_vel * PIXELS_PER_INCH * math.cos(angle)
        angle += angular_vel
        
        # Keep in bounds
        half = ROBOT_SIZE // 2
        robot_x = max(half, min(WIDTH - half, robot_x))
        robot_y = max(half, min(HEIGHT - half, robot_y))
    
    # --- ODOMETRY UPDATE ---
    move_x = robot_x - old_x
    move_y = robot_y - old_y
    move_theta = angle - old_angle
    
    odom.simulate_encoders(move_x, move_y, move_theta, old_angle)
    odom.update(to_deg(angle))
    
    # Get pygame coordinates for drawing
    odom_offset_x, odom_offset_y = odom.get_pygame_position()
    
    # --- DRAWING ---
    screen.fill(FIELD_COLOR)
    draw_grid(screen, WIDTH, HEIGHT)
    
    # Draw target if set
    if target_x is not None:
        draw_target(screen, target_x, target_y, target_theta)
    
    # Draw robot
    draw_robot(screen, robot_x, robot_y, angle, ROBOT_SIZE, ROBOT_COLOR, NIB_COLOR)
    
    # Draw ghost if in ghost mode
    if ghost_mode:
        draw_robot(screen, ghost_x, ghost_y, ghost_angle, ROBOT_SIZE, GHOST_COLOR, GHOST_NIB_COLOR)
    
    # Draw odometry position
    draw_odom_dot(screen, odom_offset_x, odom_offset_y, WIDTH, HEIGHT, ODOM_DOT_COLOR)
    
    # --- UI TEXT ---
    actual_x_inches = pixels_to_inches(robot_x - WIDTH // 2)
    actual_y_inches = pixels_to_inches(HEIGHT // 2 - robot_y)
    
    mode_text = "GHOST MODE - Position ghost and press ENTER" if ghost_mode else "AUTONOMOUS MODE - Press ESC to cancel"
    mode_surface = font.render(mode_text, True, (255, 255, 100))
    screen.blit(mode_surface, (10, 10))
    
    texts = [
        f"Odom pos (in): ({odom.cpp_x:.1f}, {odom.cpp_y:.1f})",
        f"Actual pos (in): ({actual_x_inches:.1f}, {actual_y_inches:.1f})",
        f"Angle: {to_deg(angle):.1f}°",
    ]
    
    for i, text in enumerate(texts):
        surface = font.render(text, True, (255, 255, 255))
        screen.blit(surface, (10, 35 + i * 20))
    
    if target_x is not None:
        target_text = font.render(f"Target: ({target_x:.1f}, {target_y:.1f}) @ {to_deg(target_theta):.1f}°", True, TARGET_COLOR)
        screen.blit(target_text, (10, 95))
    
    controls = "WASD: Move | ENTER: Confirm | ESC: Cancel | R: Reset | E: Debug"
    screen.blit(font.render(controls, True, (200, 200, 200)), (10, HEIGHT - 25))
    
    pygame.display.flip()

pygame.quit()
sys.exit()