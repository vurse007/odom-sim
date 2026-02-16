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
NIB_COLOR = (255, 100, 100)
ODOM_DOT_COLOR = (100, 100, 255)
PATH_COLOR = (255, 0, 255)
LOOKAHEAD_COLOR = (0, 255, 255)
LOOKAHEAD_PT_COLOR = (0, 255, 0)
ROBOT_SIZE = 80

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Pure Pursuit Controller Simulator")
font = pygame.font.Font(None, 24)

# ==========================================================================
#  ROBOT STATE
# ==========================================================================

robot_x = WIDTH // 2
robot_y = HEIGHT // 2
angle = 0.0  # In radians, 0 = facing up (North), CW positive

# Mode state
placement_mode = True  # True = placing waypoints, False = autonomous

# Target state
target_x = None
target_y = None
target_theta = None
autonomous_mode = False

# Create odometry instance
odom = Odometry()
odom.reset(angle)

# ==========================================================================
#  PURE PURSUIT PARAMETERS
# ==========================================================================

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

# Path state
path = []  # List of waypoints: (x_inches, y_inches, heading_radians)
current_segment_idx = 0
settle_count = 0
current_lookahead_pt = None
current_lookahead_radius = BASE_LOOKAHEAD

# Waypoint placement heading (rotate with Q/E)
place_heading = 0.0

# Track if we've entered final approach
in_final_approach = False

# ==========================================================================
#  HELPER FUNCTIONS
# ==========================================================================

def angle_difference(target_angle, current_angle):
    """Calculate shortest angle difference"""
    return wrap_to_pi(target_angle - current_angle)


def screen_to_field_inches(mx, my):
    """Convert mouse screen coordinates to field inches"""
    x_in = pixels_to_inches(mx - WIDTH // 2)
    y_in = pixels_to_inches(HEIGHT // 2 - my)
    return x_in, y_in


def field_inches_to_screen(x_in, y_in):
    """Convert field inches to screen coordinates"""
    sx = WIDTH // 2 + inches_to_pixels(x_in)
    sy = HEIGHT // 2 - inches_to_pixels(y_in)
    return sx, sy

# ==========================================================================
#  PURE PURSUIT GEOMETRY
# ==========================================================================

def line_circle_intersection(start, end, robot_pos, lookahead):
    """Find intersection of line segment with circle centered at robot"""
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
    """Search segments from current idx; pick furthest valid intersection"""
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
    """Calculate curvature to reach lookahead point"""
    dx = lookahead_point[0] - robot_pos[0]
    dy = lookahead_point[1] - robot_pos[1]

    angle_to_point = math.atan2(dx, dy)
    alpha = wrap_to_pi(angle_to_point - robot_pos[2])

    return (2.0 * math.sin(alpha)) / max(lookahead, 1e-6)

# ==========================================================================
#  PURE PURSUIT CONTROLLER
# ==========================================================================

def pure_pursuit_step():
    """
    Run one step of pure pursuit.
    Returns (linear_vel_in_per_frame, angular_vel_rad_per_frame)
    """
    global settle_count, current_lookahead_pt, current_lookahead_radius, in_final_approach

    # Get robot position in field inches
    rx = pixels_to_inches(robot_x - WIDTH // 2)
    ry = pixels_to_inches(HEIGHT // 2 - robot_y)
    robot_pos = (rx, ry, angle)

    # Get final target
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
        settle_count = max(0, settle_count - 1)

    # ---------- Final approach mode ----------
    TERMINAL_DIST = 10.0

    if dist_to_final < TERMINAL_DIST or in_final_approach:
        in_final_approach = True

        current_lookahead_radius = BASE_LOOKAHEAD
        current_lookahead_pt = (final_x, final_y)

        # Calculate if we're facing toward or away from target
        angle_to_target = math.atan2(dxF, dyF)
        angle_error_to_pos = wrap_to_pi(angle_to_target - angle)

        facing_target = abs(angle_error_to_pos) < math.pi / 2

        # Very close - just rotate to final heading
        if dist_to_final < PATH_COMPLETION_DIST * 2:
            omega = HEADING_KP * heading_error
            omega = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, omega))

            v = 0.0
            if facing_target and dist_to_final > PATH_COMPLETION_DIST * 0.5:
                v = min(0.5, dist_to_final * 0.2)

            return v, omega

        # If we overshot, turn toward target
        if not facing_target:
            omega = HEADING_KP * angle_error_to_pos
            omega = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, omega))
            return 0.0, omega

        # Normal final approach
        blend = max(0.0, min(1.0, 1.0 - (dist_to_final / TERMINAL_DIST)))
        blended_angle_error = (1.0 - blend) * angle_error_to_pos + blend * heading_error

        omega = HEADING_KP * blended_angle_error
        omega = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, omega))

        v = min(MAX_SPEED * 0.5, dist_to_final * 0.4)
        v = max(MIN_SPEED * 0.5, v)

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

    curvature_factor = max(0.3, 1.0 - abs(curvature) * 5.0)
    v *= curvature_factor

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

# ==========================================================================
#  DRAWING HELPERS
# ==========================================================================

def draw_path(surface):
    """Draw the path with waypoints and headings"""
    if len(path) < 1:
        return

    for i in range(len(path)):
        sx, sy = field_inches_to_screen(path[i][0], path[i][1])

        # Draw waypoint circle
        pygame.draw.circle(surface, PATH_COLOR, (int(sx), int(sy)), 6, 2)

        # Draw heading tick
        tick_len = 18
        ex = sx + tick_len * math.sin(path[i][2])
        ey = sy - tick_len * math.cos(path[i][2])
        pygame.draw.line(surface, PATH_COLOR, (sx, sy), (ex, ey), 2)

        # Draw line to next waypoint
        if i < len(path) - 1:
            nx, ny = field_inches_to_screen(path[i + 1][0], path[i + 1][1])
            pygame.draw.line(surface, PATH_COLOR, (sx, sy), (nx, ny), 2)


def draw_waypoint_preview(surface, mx, my, heading):
    """Draw preview waypoint at mouse position"""
    pygame.draw.circle(surface, PATH_COLOR, (mx, my), 6, 1)
    ex = mx + 18 * math.sin(heading)
    ey = my - 18 * math.cos(heading)
    pygame.draw.line(surface, PATH_COLOR, (mx, my), (ex, ey), 2)


def draw_lookahead(surface, robot_x, robot_y, radius, lookahead_pt):
    """Draw lookahead circle and point"""
    # Draw lookahead circle
    pygame.draw.circle(
        surface,
        LOOKAHEAD_COLOR,
        (int(robot_x), int(robot_y)),
        int(inches_to_pixels(radius)),
        1
    )

    # Draw lookahead point
    if lookahead_pt is not None:
        lx, ly = field_inches_to_screen(lookahead_pt[0], lookahead_pt[1])
        pygame.draw.circle(surface, LOOKAHEAD_PT_COLOR, (int(lx), int(ly)), 6)


def draw_target_marker(surface, target_x, target_y, target_theta):
    """Draw target position and orientation"""
    tx, ty = field_inches_to_screen(target_x, target_y)
    pygame.draw.circle(surface, PATH_COLOR, (int(tx), int(ty)), 8, 2)

    line_length = 30
    end_x = tx + line_length * math.sin(target_theta)
    end_y = ty - line_length * math.cos(target_theta)
    pygame.draw.line(surface, PATH_COLOR, (tx, ty), (end_x, end_y), 2)

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

        elif event.type == pygame.MOUSEBUTTONDOWN and placement_mode:
            if event.button == 1:  # Left click - add waypoint
                mx, my = pygame.mouse.get_pos()
                x_in, y_in = screen_to_field_inches(mx, my)
                path.append((x_in, y_in, place_heading))

            if event.button == 3 and len(path) > 0:  # Right click - set heading
                mx, my = pygame.mouse.get_pos()
                x_in, y_in = screen_to_field_inches(mx, my)
                lx, ly, _ = path[-1]
                dx = x_in - lx
                dy = y_in - ly
                place_heading = math.atan2(dx, dy)

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN:
                if placement_mode and len(path) >= 2:
                    autonomous_mode = True
                    placement_mode = False
                    current_segment_idx = 0
                    settle_count = 0
                    in_final_approach = False

                    target_x, target_y, target_theta = path[-1]
                    print(f"Path started with {len(path)} waypoints. Final: ({target_x:.1f},{target_y:.1f}) @ {to_deg(target_theta):.1f}°")

            elif event.key == pygame.K_ESCAPE:
                autonomous_mode = False
                placement_mode = True
                current_segment_idx = 0
                settle_count = 0
                current_lookahead_pt = None
                in_final_approach = False

            elif event.key == pygame.K_BACKSPACE and placement_mode:
                if len(path) > 0:
                    path.pop()

            elif event.key == pygame.K_c and placement_mode:
                path.clear()

            elif event.key == pygame.K_r:
                # Reset everything
                robot_x = WIDTH // 2
                robot_y = HEIGHT // 2
                angle = 0.0
                placement_mode = True
                autonomous_mode = False
                path.clear()
                current_segment_idx = 0
                settle_count = 0
                current_lookahead_pt = None
                in_final_approach = False
                target_x = None
                target_y = None
                target_theta = None
                place_heading = 0.0
                odom.reset(angle)

            elif event.key == pygame.K_e:
                print(f"--- DEBUG ---")
                print(f"Odom (in): X={odom.cpp_x:.2f}, Y={odom.cpp_y:.2f}")
                print(f"Angle: {to_deg(angle):.1f}°")

    keys = pygame.key.get_pressed()

    # --- PLACEMENT MODE (Waypoint Placement) ---
    if placement_mode:
        # Rotate waypoint heading with Q/E
        if keys[pygame.K_q]:
            place_heading -= math.radians(2.5)
        if keys[pygame.K_e]:
            place_heading += math.radians(2.5)
        place_heading = wrap_to_pi(place_heading)

    # --- AUTONOMOUS MODE ---
    elif autonomous_mode:
        v, omega = pure_pursuit_step()

        if v == 0.0 and omega == 0.0:
            print("Arrived at final target!")
            autonomous_mode = False
            placement_mode = True
            in_final_approach = False

        robot_x += v * PIXELS_PER_INCH * math.sin(angle)
        robot_y -= v * PIXELS_PER_INCH * math.cos(angle)
        angle += omega
        angle = wrap_to_pi(angle)

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

    # Draw path
    draw_path(screen)

    # Draw preview waypoint at mouse in placement mode
    if placement_mode:
        mx, my = pygame.mouse.get_pos()
        draw_waypoint_preview(screen, mx, my, place_heading)

    # Draw target marker if set
    if target_x is not None:
        draw_target_marker(screen, target_x, target_y, target_theta)

    # Draw lookahead visualization
    if autonomous_mode and len(path) >= 2:
        draw_lookahead(screen, robot_x, robot_y, current_lookahead_radius, current_lookahead_pt)

    # Draw robot
    draw_robot(screen, robot_x, robot_y, angle, ROBOT_SIZE, ROBOT_COLOR, NIB_COLOR)

    # Draw odometry position
    draw_odom_dot(screen, odom_offset_x, odom_offset_y, WIDTH, HEIGHT, ODOM_DOT_COLOR)

    # --- UI TEXT ---
    actual_x_inches = pixels_to_inches(robot_x - WIDTH // 2)
    actual_y_inches = pixels_to_inches(HEIGHT // 2 - robot_y)

    if placement_mode:
        mode_text = "Click to add waypoints | ENTER to run | C clear | BKSP undo | R reset"
    else:
        mode_text = "AUTONOMOUS MODE - Press ESC to cancel"
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
        target_text = font.render(f"Target: ({target_x:.1f}, {target_y:.1f}) @ {to_deg(target_theta):.1f}°", True, PATH_COLOR)
        screen.blit(target_text, (10, 95))

    # Show final approach status
    if autonomous_mode and in_final_approach:
        approach_text = font.render("FINAL APPROACH", True, LOOKAHEAD_COLOR)
        screen.blit(approach_text, (10, 115))

    # Show current waypoint heading in placement mode
    if placement_mode:
        heading_text = font.render(f"Waypoint heading: {to_deg(place_heading):.1f}° (Q/E to rotate)", True, (200, 200, 200))
        screen.blit(heading_text, (10, HEIGHT - 50))

    controls_text = font.render("Left-click: Add waypoint | Right-click: Aim last waypoint", True, (200, 200, 200))
    screen.blit(controls_text, (10, HEIGHT - 25))

    pygame.display.flip()

pygame.quit()
sys.exit()