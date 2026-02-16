import pygame
import sys
import math

# ==========================================================================
#  HELPER FUNCTIONS (Matching C++)
# ==========================================================================

def wrap_to_pi(angle):
    """Matches util::wrap_to_pi"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def to_deg(rad):
    return rad * 180.0 / math.pi


def to_rad(deg):
    return deg * math.pi / 180.0


# ==========================================================================
#  CONSTANTS
# ==========================================================================

WIDTH, HEIGHT = 600, 600
FIELD_SIZE_INCHES = 12 * 12  # 144 inches
PIXELS_PER_INCH = WIDTH / FIELD_SIZE_INCHES
INCHES_PER_PIXEL = FIELD_SIZE_INCHES / WIDTH

# Encoder offsets (matching C++)
H_OFFSET = 0.0
V_OFFSET = 0.0


# ==========================================================================
#  UNIT CONVERSION
# ==========================================================================

def pixels_to_inches(p):
    return p * INCHES_PER_PIXEL


def inches_to_pixels(i):
    return i * PIXELS_PER_INCH


# ==========================================================================
#  ODOMETRY CLASS (Encapsulates all odometry state and logic)
# ==========================================================================

class Odometry:
    """
    Odometry class that matches the C++ implementation.
    Tracks robot position using encoder data and IMU heading.
    """
    
    def __init__(self):
        # Encoder totals (accumulated distance in pixels)
        self.h_enc_total = 0.0  # Horizontal pod (measures sideways)
        self.v_enc_total = 0.0  # Vertical pod (measures forward)
        
        # Previous values for delta calculation
        self.prev_h_enc = 0.0
        self.prev_v_enc = 0.0
        
        # C++ odometry state
        self.cpp_x = 0.0       # C++ X = East (+X on field, right)
        self.cpp_y = 0.0       # C++ Y = North (+Y on field, up)
        self.cpp_theta = 0.0   # C++ Theta (Radians, 0 = North)
        self.imu_heading_offset = 0.0
    
    def reset(self, angle=0.0):
        """Reset odometry to origin with given angle as reference"""
        self.cpp_x = 0.0
        self.cpp_y = 0.0
        self.cpp_theta = 0.0
        
        self.h_enc_total = 0.0
        self.v_enc_total = 0.0
        self.prev_h_enc = 0.0
        self.prev_v_enc = 0.0
        
        self.imu_heading_offset = to_deg(angle)
    
    def simulate_encoders(self, dx_pix, dy_pix, d_angle_rad, current_angle):
        """
        Calculates what the encoders WOULD read based on movement.
        
        Args:
            dx_pix: Change in x position (pixels)
            dy_pix: Change in y position (pixels)
            d_angle_rad: Change in angle (radians)
            current_angle: Current robot angle (radians)
        """
        # Robot's forward direction in pygame coordinates
        forward_x = math.sin(current_angle)
        forward_y = -math.cos(current_angle)
        
        # Robot's right direction in pygame coordinates
        right_x = math.cos(current_angle)
        right_y = math.sin(current_angle)
        
        # Project global movement onto robot's local axes
        local_x = dx_pix * right_x + dy_pix * right_y      # Sideways
        local_y = dx_pix * forward_x + dy_pix * forward_y  # Forward
        
        # Accumulate encoder readings
        self.h_enc_total += local_x - (H_OFFSET * PIXELS_PER_INCH * d_angle_rad)
        self.v_enc_total += local_y + (V_OFFSET * PIXELS_PER_INCH * d_angle_rad)
    
    def update(self, current_imu_deg):
        """
        Run the C++ odometry logic.
        This is a direct port of the C++ update() function.
        
        Args:
            current_imu_deg: Current IMU heading in degrees
        """
        # Calculate encoder deltas in inches
        delta_h_inches = pixels_to_inches(self.h_enc_total - self.prev_h_enc)
        delta_v_inches = pixels_to_inches(self.v_enc_total - self.prev_v_enc)
        
        # Update previous encoder values
        self.prev_h_enc = self.h_enc_total
        self.prev_v_enc = self.v_enc_total
        
        # Calculate deltas (matching C++ variable names)
        dS = delta_h_inches  # Sideways (Horizontal Pod)
        dF = delta_v_inches  # Forward (Vertical Pod)
        
        # Convert IMU heading to radians
        heading_rad = to_rad(current_imu_deg - self.imu_heading_offset)
        self.cpp_theta = wrap_to_pi(heading_rad)
        
        # Global Frame Math (THE CORE C++ LOGIC)
        # C++: double global_dx =  dS * cosH + dF * sinH;  // East
        # C++: double global_dy =  dF * cosH - dS * sinH;  // North
        cosH = math.cos(self.cpp_theta)
        sinH = math.sin(self.cpp_theta)
        
        global_dx = dS * cosH + dF * sinH  # East/West movement
        global_dy = dF * cosH - dS * sinH  # North/South movement
        
        # Integrate position
        self.cpp_x += global_dx  # X = East
        self.cpp_y += global_dy  # Y = North
    
    def get_position_inches(self):
        """Get current position in inches (C++ convention: X=East, Y=North)"""
        return self.cpp_x, self.cpp_y, self.cpp_theta
    
    def get_pygame_position(self):
        """
        Get position translated to pygame screen coordinates.
        Returns offset from screen center in pixels.
        """
        # C++ X (East)  -> Pygame +X (Right)
        # C++ Y (North) -> Pygame -Y (Up)
        screen_offset_x = inches_to_pixels(self.cpp_x)
        screen_offset_y = -inches_to_pixels(self.cpp_y)
        return screen_offset_x, screen_offset_y


# ==========================================================================
#  DRAWING FUNCTIONS
# ==========================================================================

def draw_grid(surface, width, height):
    """Draw the field grid"""
    grid_spacing_pixels = inches_to_pixels(24)
    
    for x in range(0, width + 1, int(grid_spacing_pixels)):
        pygame.draw.line(surface, (60, 60, 60), (x, 0), (x, height), 1)
    for y in range(0, height + 1, int(grid_spacing_pixels)):
        pygame.draw.line(surface, (60, 60, 60), (0, y), (width, y), 1)
    
    # Center lines (yellow)
    pygame.draw.line(surface, (255, 255, 0), (width // 2, 0), (width // 2, height), 2)
    pygame.draw.line(surface, (255, 255, 0), (0, height // 2), (width, height // 2), 2)


def draw_robot(surface, x, y, angle, size, body_color, nib_color):
    """Draw a robot at the specified position"""
    robot_surface = pygame.Surface((size, size), pygame.SRCALPHA)
    robot_surface.fill(body_color)
    
    # Nib (direction indicator)
    center = (size // 2, size // 2)
    nib_length = 15
    nib_tip = (center[0], center[1] - nib_length)
    left = (center[0] - 6, center[1])
    right = (center[0] + 6, center[1])
    pygame.draw.polygon(robot_surface, nib_color, [nib_tip, left, right])
    
    # Rotate and draw
    angle_degrees = math.degrees(angle)
    rotated = pygame.transform.rotate(robot_surface, -angle_degrees)
    rect = rotated.get_rect(center=(x, y))
    surface.blit(rotated, rect.topleft)


def draw_odom_dot(surface, odom_x, odom_y, width, height, color=(100, 100, 255)):
    """Draw a dot representing the odometry position"""
    screen_x = width // 2 + odom_x
    screen_y = height // 2 + odom_y
    
    if 0 <= screen_x <= width and 0 <= screen_y <= height:
        pygame.draw.circle(surface, color, (int(screen_x), int(screen_y)), 6)


# ==========================================================================
#  STANDALONE SIMULATION (Only runs when executed directly)
# ==========================================================================

if __name__ == "__main__":
    pygame.init()
    
    # Constants
    FIELD_COLOR = (30, 30, 30)
    ROBOT_COLOR = (180, 180, 180)
    NIB_COLOR = (255, 100, 100)
    ODOM_DOT_COLOR = (100, 100, 255)
    ROBOT_SIZE = 80
    MOVE_SPEED = 10
    TURN_ACCEL = math.radians(0.5)
    FRICTION = 0.9
    ACCEL_FACTOR = 0.1
    
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Odom Sim (C++ Logic)")
    font = pygame.font.Font(None, 24)
    
    # Robot state
    robot_x = WIDTH // 2
    robot_y = HEIGHT // 2
    angle = 0.0
    angular_velocity = 0.0
    vx, vy = 0, 0
    
    # Create odometry instance
    odom = Odometry()
    odom.reset(angle)
    
    clock = pygame.time.Clock()
    running = True
    
    while running:
        clock.tick(60)
        
        old_x, old_y, old_angle = robot_x, robot_y, angle
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    robot_x = WIDTH // 2
                    robot_y = HEIGHT // 2
                    angle = 0.0
                    vx, vy, angular_velocity = 0, 0, 0
                    odom.reset(angle)
                elif event.key == pygame.K_e:
                    print(f"--- DEBUG ---")
                    print(f"C++ Odom (in): X={odom.cpp_x:.2f}, Y={odom.cpp_y:.2f}")
                    print(f"Angle: {to_deg(angle):.1f}°")
        
        # Physics
        keys = pygame.key.get_pressed()
        
        forward_x = math.sin(angle)
        forward_y = -math.cos(angle)
        
        if keys[pygame.K_w]:
            vx += MOVE_SPEED * forward_x * ACCEL_FACTOR
            vy += MOVE_SPEED * forward_y * ACCEL_FACTOR
        if keys[pygame.K_s]:
            vx -= MOVE_SPEED * forward_x * ACCEL_FACTOR
            vy -= MOVE_SPEED * forward_y * ACCEL_FACTOR
        if keys[pygame.K_a]:
            angular_velocity -= TURN_ACCEL
        if keys[pygame.K_d]:
            angular_velocity += TURN_ACCEL
        
        robot_x += vx
        robot_y += vy
        angle += angular_velocity
        
        vx *= FRICTION
        vy *= FRICTION
        angular_velocity *= FRICTION
        
        # Keep robot in bounds
        half = ROBOT_SIZE // 2
        robot_x = max(half, min(WIDTH - half, robot_x))
        robot_y = max(half, min(HEIGHT - half, robot_y))
        
        # Update odometry
        move_x = robot_x - old_x
        move_y = robot_y - old_y
        move_theta = angle - old_angle
        
        odom.simulate_encoders(move_x, move_y, move_theta, old_angle)
        odom.update(to_deg(angle))
        
        # Get pygame coordinates for drawing
        odom_offset_x, odom_offset_y = odom.get_pygame_position()
        
        # Drawing
        screen.fill(FIELD_COLOR)
        draw_grid(screen, WIDTH, HEIGHT)
        draw_robot(screen, robot_x, robot_y, angle, ROBOT_SIZE, ROBOT_COLOR, NIB_COLOR)
        draw_odom_dot(screen, odom_offset_x, odom_offset_y, WIDTH, HEIGHT, ODOM_DOT_COLOR)
        
        # UI
        actual_x_inches = pixels_to_inches(robot_x - WIDTH // 2)
        actual_y_inches = pixels_to_inches(HEIGHT // 2 - robot_y)
        
        texts = [
            f"C++ Odom (in): X={odom.cpp_x:.1f} (East), Y={odom.cpp_y:.1f} (North)",
            f"Actual (in):   X={actual_x_inches:.1f}, Y={actual_y_inches:.1f}",
            f"Angle: {to_deg(angle):.1f}°",
        ]
        
        for i, text in enumerate(texts):
            surface = font.render(text, True, (255, 255, 255))
            screen.blit(surface, (10, 10 + i * 20))
        
        controls = "WASD: Move | R: Reset | E: Debug"
        screen.blit(font.render(controls, True, (200, 200, 200)), (10, HEIGHT - 25))
        
        pygame.display.flip()
    
    pygame.quit()
    sys.exit()