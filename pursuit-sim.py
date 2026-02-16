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

# Field dimensions (12ft x 12ft converted to pixels)
FIELD_SIZE_INCHES = 12 * 12  # 144 inches
PIXELS_PER_INCH = WIDTH / FIELD_SIZE_INCHES
INCHES_PER_PIXEL = FIELD_SIZE_INCHES / WIDTH

# Create screen
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Pure Pursuit Controller - Robot Simulator")

# Robot state
robot_x = WIDTH // 2
robot_y = HEIGHT // 2
angle = 0.0  # In radians, 0 = facing up (North), CW positive
robot_vx, robot_vy = 0.0, 0.0
robot_angular_velocity = 0.0

# "Ghost" (used as UI helper)
ghost_x = WIDTH // 2 + 100
ghost_y = HEIGHT // 2
ghost_angle = 0.0
ghost_mode = True  # Start in waypoint placement mode

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

# Target / autonomous
target_x = None
target_y = None
target_theta = None
autonomous_mode = False

# Font for displaying information
font = pygame.font.Font(None, 24)

# ============================
# PURE PURSUIT PARAMETERS
# ============================

BASE_LOOKAHEAD = 18.0
MIN_LOOKAHEAD = 6.0
MAX_LOOKAHEAD = 30.0

HEADING_KP = 0.15
HEADING_BLEND_DIST = 18.0
HEADING_BLEND_POWER = 3.0

PATH_COMPLETION_DIST = 1.5
FINAL_HEADING_TOLERANCE = 3.0
SETTLE_COUNT_TARGET = 15

MAX_SPEED = 1.0
MAX_ANGULAR_SPEED = 0.12
MIN_SPEED = 0.2

# Path: list of waypoints in inches + heading radians (IMU frame)
path = []
current_segment_idx = 0
settle_count = 0
current_lookahead_pt = None
current_lookahead_radius = BASE_LOOKAHEAD

# Waypoint placement heading (rotate with Q/E)
place_heading = 0.0

# Track if we've entered final approach (prevents re-entering pursuit after passing target)
in_final_approach = False

# ============================
# Helper functions
# ============================

def pixels_to_inches(pixels):
    return pixels * INCHES_PER_PIXEL

def inches_to_pixels(inches):
    return inches * PIXELS_PER_INCH

def wrap_to_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def angle_difference(target_angle, current_angle):
    return wrap_to_pi(target_angle - current_angle)

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
    global odom_x, odom_y, odom_theta
    global prev_horizontal_encoder, prev_vertical_encoder

    odom_x = set_x_inches
    odom_y = set_y_inches
    odom_theta = set_theta if set_theta is not None else angle

    prev_horizontal_encoder = horizontal_encoder
    prev_vertical_encoder = vertical_encoder

def odom_update(horizontal_pixels, vertical_pixels):
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

def draw_bot(screen, x, y, ang, color, nib_color):
    """Draw a robot at the specified position"""
    robot_surface = pygame.Surface((ROBOT_SIZE, ROBOT_SIZE), pygame.SRCALPHA)
    robot_surface.fill(color)

    center = (ROBOT_SIZE // 2, ROBOT_SIZE // 2)
    nib_length = 10
    nib_tip = (center[0], center[1] - nib_length)
    left = (center[0] - 4, center[1])
    right = (center[0] + 4, center[1])
    pygame.draw.polygon(robot_surface, nib_color, [nib_tip, left, right])

    angle_degrees = math.degrees(ang)
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

def screen_to_field_inches(mx, my):
    """Mouse screen -> field inches (robot frame: center is 0,0; +x east, +y north)"""
    x_in = pixels_to_inches(mx - WIDTH // 2)
    y_in = pixels_to_inches(HEIGHT // 2 - my)
    return x_in, y_in

def field_inches_to_screen(x_in, y_in):
    sx = WIDTH // 2 + inches_to_pixels(x_in)
    sy = HEIGHT // 2 - inches_to_pixels(y_in)
    return sx, sy

# ============================
# Pure pursuit geometry
# ============================

def line_circle_intersection(start, end, robot_pos, lookahead):
    sx, sy = start[0], start[1]
    ex, ey = end[0], end[1]
    rx, ry = robot_pos[0], robot_pos[1]

    dx = ex - sx
    dy = ey - sy

    fx = sx - rx
    fy = sy - ry

    a = dx * dx + dy * dy
    if a < 1e-9:
        return False, None, None

    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - lookahead * lookahead

    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return False, None, None

    disc = math.sqrt(disc)
    t1 = (-b - disc) / (2.0 * a)
    t2 = (-b + disc) / (2.0 * a)

    t = None
    if 0.0 <= t2 <= 1.0:
        t = t2
    elif 0.0 <= t1 <= 1.0:
        t = t1

    if t is None:
        return False, None, None

    ix = sx + t * dx
    iy = sy + t * dy
    return True, (ix, iy), t

def find_lookahead_point(robot_pos, lookahead):
    """Search segments from current idx; pick furthest valid intersection."""
    global current_segment_idx

    best = None
    best_seg = current_segment_idx
    best_t = -1.0

    for i in range(current_segment_idx, len(path) - 1):
        start = path[i]
        end = path[i + 1]
        found, pt, t = line_circle_intersection(start, end, robot_pos, lookahead)
        if found:
            if (best is None) or (i > best_seg) or (i == best_seg and t > best_t):
                best = pt
                best_seg = i
                best_t = t

    if best is not None:
        current_segment_idx = best_seg
        if best_seg < len(path) - 1:
            curr_wp = path[best_seg]
            next_wp = path[best_seg + 1]
            dist_curr = math.hypot(robot_pos[0] - curr_wp[0], robot_pos[1] - curr_wp[1])
            dist_next = math.hypot(robot_pos[0] - next_wp[0], robot_pos[1] - next_wp[1])
            if dist_next < dist_curr:
                current_segment_idx = min(best_seg + 1, len(path) - 2)
        return best

    return (path[-1][0], path[-1][1])

def calculate_pursuit_curvature(robot_pos, lookahead_point, lookahead):
    dx = lookahead_point[0] - robot_pos[0]
    dy = lookahead_point[1] - robot_pos[1]

    angle_to_point = math.atan2(dx, dy)
    alpha = wrap_to_pi(angle_to_point - robot_pos[2])

    return (2.0 * math.sin(alpha)) / max(lookahead, 1e-6)

def pure_pursuit_step():
    """Returns (linear_vel_in_per_frame, angular_vel_rad_per_frame)."""
    global settle_count, current_lookahead_pt, current_lookahead_radius, in_final_approach

    rx = pixels_to_inches(robot_x - WIDTH // 2)
    ry = pixels_to_inches(HEIGHT // 2 - robot_y)
    robot_pos = (rx, ry, angle)

    final_x, final_y, final_theta = path[-1]
    dxF = final_x - rx
    dyF = final_y - ry
    dist_to_final = math.hypot(dxF, dyF)
    
    heading_error = wrap_to_pi(final_theta - angle)

    # ---------- Check for arrival ----------
    if dist_to_final < PATH_COMPLETION_DIST and abs(math.degrees(heading_error)) < FINAL_HEADING_TOLERANCE:
        settle_count += 1
        if settle_count >= SETTLE_COUNT_TARGET:
            return 0.0, 0.0
    else:
        settle_count = max(0, settle_count - 1)  # Decay settle count gradually

    # ---------- Final approach mode ----------
    # Once we enter final approach, stay in it (no more pursuit)
    TERMINAL_DIST = 10.0
    
    if dist_to_final < TERMINAL_DIST or in_final_approach:
        in_final_approach = True
        
        current_lookahead_radius = BASE_LOOKAHEAD
        current_lookahead_pt = (final_x, final_y)
        
        # Calculate if we're facing toward or away from target
        angle_to_target = math.atan2(dxF, dyF)
        angle_error_to_pos = wrap_to_pi(angle_to_target - angle)
        
        # Check if we're facing roughly toward the target (within ~90 degrees)
        facing_target = abs(angle_error_to_pos) < math.pi / 2
        
        # Very close - just rotate to final heading
        if dist_to_final < PATH_COMPLETION_DIST * 2:
            omega = HEADING_KP * heading_error
            omega = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, omega))
            
            # Tiny forward motion only if facing target and not there yet
            v = 0.0
            if facing_target and dist_to_final > PATH_COMPLETION_DIST * 0.5:
                v = min(0.5, dist_to_final * 0.2)
            
            return v, omega
        
        # If we overshot (facing away from target), we need to back up or turn around
        if not facing_target:
            # Turn toward target first, minimal forward motion
            omega = HEADING_KP * angle_error_to_pos
            omega = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, omega))
            
            # Could optionally drive backward here, but turning is usually better
            return 0.0, omega
        
        # Normal final approach - facing target, drive toward it
        # Blend between position-seeking and final heading based on distance
        blend = max(0.0, min(1.0, 1.0 - (dist_to_final / TERMINAL_DIST)))
        blended_angle_error = (1.0 - blend) * angle_error_to_pos + blend * heading_error
        
        omega = HEADING_KP * blended_angle_error
        omega = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, omega))
        
        # Speed proportional to distance, with minimum
        v = min(MAX_SPEED * 0.5, dist_to_final * 0.4)
        v = max(MIN_SPEED * 0.5, v)
        
        # Reduce speed if we need to turn a lot
        turn_penalty = max(0.3, 1.0 - abs(blended_angle_error) / math.pi)
        v *= turn_penalty
        
        return v, omega

    # ---------- Normal pursuit ----------
    settle_count = 0
    in_final_approach = False
    
    lookahead = max(MIN_LOOKAHEAD, min(MAX_LOOKAHEAD, BASE_LOOKAHEAD))
    current_lookahead_radius = lookahead

    lookahead_pt = find_lookahead_point(robot_pos, lookahead)
    current_lookahead_pt = lookahead_pt

    curvature = calculate_pursuit_curvature(robot_pos, lookahead_pt, lookahead)

    # Speed control
    v = MAX_SPEED
    
    # Slow down based on curvature
    curvature_factor = max(0.3, 1.0 - abs(curvature) * 5.0)
    v *= curvature_factor
    
    # Slow down approaching end
    if dist_to_final < 20.0:
        v *= max(0.3, dist_to_final / 20.0)
    
    v = max(MIN_SPEED, min(MAX_SPEED, v))

    # Angular velocity from curvature
    omega = curvature * v
    
    # Add heading blend toward final heading as we approach
    if dist_to_final < HEADING_BLEND_DIST:
        blend = 1.0 - math.exp(
            -HEADING_BLEND_POWER *
            (HEADING_BLEND_DIST - dist_to_final) /
            HEADING_BLEND_DIST
        )
        blend = max(0.0, min(0.5, blend))
        omega += blend * HEADING_KP * heading_error
    
    omega = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, omega))

    return v, omega

# Reset odometry
odom_reset()

# Game loop
clock = pygame.time.Clock()
running = True

while running:
    dt = clock.tick(60) / 1000.0

    old_x, old_y, old_angle = robot_x, robot_y, angle

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.MOUSEBUTTONDOWN and ghost_mode:
            if event.button == 1:
                mx, my = pygame.mouse.get_pos()
                x_in, y_in = screen_to_field_inches(mx, my)
                path.append((x_in, y_in, place_heading))

            if event.button == 3 and len(path) > 0:
                mx, my = pygame.mouse.get_pos()
                x_in, y_in = screen_to_field_inches(mx, my)
                lx, ly, _ = path[-1]
                dx = x_in - lx
                dy = y_in - ly
                place_heading = math.atan2(dx, dy)

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN:
                if ghost_mode and len(path) >= 2:
                    autonomous_mode = True
                    ghost_mode = False
                    current_segment_idx = 0
                    settle_count = 0
                    in_final_approach = False  # Reset final approach flag

                    target_x, target_y, target_theta = path[-1]
                    print(f"Path started with {len(path)} waypoints. Final: ({target_x:.1f},{target_y:.1f}) @ {math.degrees(target_theta):.1f}°")

            elif event.key == pygame.K_ESCAPE:
                autonomous_mode = False
                ghost_mode = True
                current_segment_idx = 0
                settle_count = 0
                current_lookahead_pt = None
                in_final_approach = False  # Reset final approach flag

            elif event.key == pygame.K_BACKSPACE and ghost_mode:
                if len(path) > 0:
                    path.pop()

            elif event.key == pygame.K_c and ghost_mode:
                path.clear()

    keys = pygame.key.get_pressed()

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

        if keys[pygame.K_q]:
            place_heading -= math.radians(2.5)
        if keys[pygame.K_e]:
            place_heading += math.radians(2.5)
        place_heading = wrap_to_pi(place_heading)

        half = ROBOT_SIZE // 2
        ghost_x = max(half, min(WIDTH - half, ghost_x))
        ghost_y = max(half, min(HEIGHT - half, ghost_y))

    elif autonomous_mode:
        v, omega = pure_pursuit_step()

        if v == 0.0 and omega == 0.0:
            print("Arrived at final target!")
            autonomous_mode = False
            ghost_mode = True
            in_final_approach = False

        robot_x += v * PIXELS_PER_INCH * math.sin(angle)
        robot_y -= v * PIXELS_PER_INCH * math.cos(angle)
        angle += omega
        
        # Wrap the angle to prevent unbounded growth
        angle = wrap_to_pi(angle)

        half = ROBOT_SIZE // 2
        robot_x = max(half, min(WIDTH - half, robot_x))
        robot_y = max(half, min(HEIGHT - half, robot_y))

    # Simulate encoders + odom
    simulate_encoders(old_x, old_y, old_angle, robot_x, robot_y, angle)
    odom_update(horizontal_encoder, vertical_encoder)

    # =========================================================
    # DRAW FIELD
    # =========================================================
    screen.fill(FIELD_COLOR)

    # Grid
    grid_spacing_inches = 24
    grid_spacing_pixels = inches_to_pixels(grid_spacing_inches)

    for x in range(0, WIDTH + 1, int(grid_spacing_pixels)):
        pygame.draw.line(screen, (255, 255, 255), (x, 0), (x, HEIGHT), 1)
    for y in range(0, HEIGHT + 1, int(grid_spacing_pixels)):
        pygame.draw.line(screen, (255, 255, 255), (0, y), (WIDTH, y), 1)

    # Center lines
    pygame.draw.line(screen, (255, 255, 0), (WIDTH // 2, 0), (WIDTH // 2, HEIGHT), 2)
    pygame.draw.line(screen, (255, 255, 0), (0, HEIGHT // 2), (WIDTH, HEIGHT // 2), 2)

    # Draw path polyline + waypoints + headings
    if len(path) >= 1:
        for i in range(len(path)):
            sx, sy = field_inches_to_screen(path[i][0], path[i][1])
            pygame.draw.circle(screen, (255, 0, 255), (int(sx), int(sy)), 6, 2)

            tick_len = 18
            ex = sx + tick_len * math.sin(path[i][2])
            ey = sy - tick_len * math.cos(path[i][2])
            pygame.draw.line(screen, (255, 0, 255), (sx, sy), (ex, ey), 2)

            if i < len(path) - 1:
                nx, ny = field_inches_to_screen(path[i + 1][0], path[i + 1][1])
                pygame.draw.line(screen, (255, 0, 255), (sx, sy), (nx, ny), 2)

    # Draw preview waypoint at mouse in ghost mode
    if ghost_mode:
        mx, my = pygame.mouse.get_pos()
        pygame.draw.circle(screen, (255, 0, 255), (mx, my), 6, 1)
        ex = mx + 18 * math.sin(place_heading)
        ey = my - 18 * math.cos(place_heading)
        pygame.draw.line(screen, (255, 0, 255), (mx, my), (ex, ey), 2)

    # Draw target position if set
    if target_x is not None:
        tx, ty = field_inches_to_screen(target_x, target_y)
        pygame.draw.circle(screen, (255, 0, 255), (int(tx), int(ty)), 8, 2)
        line_length = 30
        end_x = tx + line_length * math.sin(target_theta)
        end_y = ty - line_length * math.cos(target_theta)
        pygame.draw.line(screen, (255, 0, 255), (tx, ty), (end_x, end_y), 2)

    # Draw lookahead circle + point
    if autonomous_mode and len(path) >= 2:
        pygame.draw.circle(
            screen,
            (0, 255, 255),
            (int(robot_x), int(robot_y)),
            int(inches_to_pixels(current_lookahead_radius)),
            1
        )

        if current_lookahead_pt is not None:
            lx, ly = field_inches_to_screen(current_lookahead_pt[0], current_lookahead_pt[1])
            pygame.draw.circle(screen, (0, 255, 0), (int(lx), int(ly)), 6)

    # Draw robot
    draw_bot(screen, robot_x, robot_y, angle, ROBOT_COLOR, NIB_COLOR)

    # Draw odometry dot
    draw_odom_position(screen, odom_x, odom_y)

    # Display information
    actual_x_inches = pixels_to_inches(robot_x - WIDTH // 2)
    actual_y_inches = pixels_to_inches(HEIGHT // 2 - robot_y)

    if ghost_mode:
        mode_text = "Click to add waypoints | ENTER to run | C clear | BKSP undo"
    else:
        mode_text = "AUTONOMOUS MODE - Press ESC to cancel"
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
    
    # Show final approach status
    if autonomous_mode and in_final_approach:
        approach_text = font.render("FINAL APPROACH", True, (0, 255, 255))
        screen.blit(approach_text, (10, 115))

    controls_text = font.render("WASD: Move/Control | Q/E: Rotate waypoint heading | ENTER: Run", True, (255, 255, 255))
    screen.blit(controls_text, (10, HEIGHT - 30))

    pygame.display.flip()

pygame.quit()
sys.exit()