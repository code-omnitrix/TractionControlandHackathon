import pygame
import numpy as np
import csv
import time
import importlib
import importlib.util
import sys
import math
from physics import BicyclePhysics
from practice_track import PRACTICE_TRACK, TIME_LIMIT

# Screen dimensions
SCREEN_WIDTH, SCREEN_HEIGHT = 1200, 700
BG_COLOR = (240, 240, 240)
TRACK_COLOR = (100, 100, 100)
BIKE_COLOR = (0, 0, 200)
TEXT_COLOR = (0, 0, 0)
ALERT_COLOR = (200, 0, 0)
WARNING_COLOR = (255, 165, 0)
SUCCESS_COLOR = (0, 150, 0)
DASHBOARD_BG = (255, 255, 255, 220)
DASHBOARD_BORDER = (100, 100, 100, 255)
LABEL_COLOR = (50, 50, 50)

class EcoGearSimulator:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Eco-Gear Challenge Simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('Arial', 24, bold=True)
        self.small_font = pygame.font.SysFont('Arial', 18)
        self.monospace_font = pygame.font.SysFont('Courier New', 22, bold=True)
        
        # Load controller
        self.controller_path = "controller_template.py"
        self.load_controller()
        
        # Initialize physics
        self.bike = BicyclePhysics(PRACTICE_TRACK, TIME_LIMIT)
        
        # Precompute track profile
        self.track_profile = self.generate_elevation_profile()
        
        # Logging
        self.log_file = f"simulation_log_{int(time.time())}.csv"
        self.setup_logger()
        self.logging_enabled = True
        
        # Simulation state
        self.paused = False
        self.show_help = False
        
    def load_controller(self):
        """Dynamically load user's controller"""
        try:
            spec = importlib.util.spec_from_file_location("controller", self.controller_path)
            self.controller = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.controller)
            print(f"Loaded controller: {self.controller_path}")
        except Exception as e:
            print(f"Error loading controller: {e}")
            print("Using default controller template")
            self.controller_path = "controller_template.py"
            spec = importlib.util.spec_from_file_location("controller", self.controller_path)
            self.controller = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.controller)
    
    def generate_elevation_profile(self):
        """Precompute elevation points for rendering"""
        profile = []
        elevation = 0.0
        last_x = 0.0
        
        for seg in self.bike.track_segments:
            start, end, slope, _ = seg
            segment_length = end - start
            
            # Fill in points between segments
            if start > last_x:
                for x in np.linspace(last_x, start, int((start - last_x) * 10) + 1):
                    profile.append((x, elevation))
            
            # Process current segment
            for x in np.linspace(start, end, int(segment_length * 10) + 1):
                dx = x - last_x
                elevation += slope * dx
                profile.append((x, elevation))
                last_x = x
        
        return profile
    
    def setup_logger(self):
        """Initialize CSV logger"""
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'x', 'v', 'segment_idx', 'slope', 'mu', 
                'gear_ratio', 'energy_step', 'total_energy', 'slip_event',
                'F_drive', 'F_gravity', 'F_drag', 'F_roll'
            ])
    
    def log_step(self, state):
        """Log current state to CSV"""
        if not self.logging_enabled or self.bike.completed or self.bike.failed:
            return
        
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                time.time(),
                state['x'],
                state['v'],
                state['segment_idx'],
                state['slope'],
                state['mu'],
                state['gear_ratio'],
                state['energy_step'],
                state['total_energy'],
                state['slip_event'],
                state['F_drive'],
                state['F_gravity'],
                state['F_drag'],
                state['F_roll']
            ])
    
    def draw_track(self):
        """Render elevation profile with friction coloring"""
        if not self.track_profile:
            return
        
        # View parameters
        scale = 1000 / self.bike.total_length
        offset_x = SCREEN_WIDTH // 2 - self.bike.x * scale
        ground_y = 500
        
        # Draw ground line
        start_screen_x = offset_x
        end_screen_x = self.bike.total_length * scale + offset_x
        pygame.draw.line(self.screen, (0, 0, 0), (start_screen_x, ground_y), (end_screen_x, ground_y), 2)
        
        # Draw track segments
        for i in range(1, len(self.track_profile)):
            x1, y1 = self.track_profile[i-1]
            x2, y2 = self.track_profile[i]
            
            # Screen coordinates
            screen_x1 = x1 * scale + offset_x
            screen_x2 = x2 * scale + offset_x
            screen_y1 = ground_y - y1 * 50
            screen_y2 = ground_y - y2 * 50
            
            # Only draw if visible
            if screen_x2 < 0 or screen_x1 > SCREEN_WIDTH:
                continue
            
            # Get segment properties for coloring
            _, (_, _, _, mu) = self.bike.get_current_segment(x1)
            
            # Color by friction (blue=high mu, red=low mu)
            color_intensity = min(1.0, max(0.0, mu))
            color = (
                int(255 * (1 - color_intensity)),  # Red decreases with mu
                int(255 * color_intensity),        # Green increases with mu
                0
            )
            
            pygame.draw.line(self.screen, color, (screen_x1, screen_y1), (screen_x2, screen_y2), 4)
        
        # Draw segment boundaries
        for seg in self.bike.track_segments:
            start, end, _, _ = seg
            screen_x = start * scale + offset_x
            if 0 <= screen_x <= SCREEN_WIDTH:
                pygame.draw.line(self.screen, (0, 0, 0), (screen_x, ground_y-20), (screen_x, ground_y+20), 2)
        
        # Draw finish line
        finish_x = self.bike.total_length * scale + offset_x
        if 0 <= finish_x <= SCREEN_WIDTH:
            pygame.draw.line(self.screen, (0, 0, 0), (finish_x, ground_y-30), (finish_x, ground_y+30), 4)
    
    def draw_bike(self):
        """Draw bicycle sprite at current position with proper rotation based on slope"""
        ground_y = 500
        bike_x = SCREEN_WIDTH // 2
        bike_y = ground_y - self.bike.elevation * 50 - 15  # Offset for bike height
        
        # Calculate rotation angle based on visual aspect ratio
        scale = 1000 / self.bike.total_length
        vertical_scale = 50
        visual_slope = self.bike.current_slope * (vertical_scale / scale)
        
        # Limit rotation to a reasonable range (-60 to 60 degrees)
        max_rotation = 60
        angle = math.degrees(math.atan(visual_slope))
        angle = max(-max_rotation, min(max_rotation, angle))
        
        # Create bike surface
        bike_surface = pygame.Surface((40, 40), pygame.SRCALPHA)
        cx, cy = 20, 20  # Center of bike surface
        
        # Draw bike components
        pygame.draw.rect(bike_surface, BIKE_COLOR, (cx-10, cy-10, 20, 20))  # Body
        pygame.draw.circle(bike_surface, (50, 50, 50), (cx-8, cy+15), 6)    # Front wheel
        pygame.draw.circle(bike_surface, (50, 50, 50), (cx+8, cy+15), 6)    # Rear wheel
        pygame.draw.circle(bike_surface, (255, 200, 0), (cx, cy-15), 8)     # Rider
        
        # Rotate bike surface
        rotated_bike = pygame.transform.rotate(bike_surface, angle)
        rotated_rect = rotated_bike.get_rect()
        rotated_rect.center = (bike_x, bike_y)
        
        # Draw rotated bike
        self.screen.blit(rotated_bike, rotated_rect)
    
    def draw_dashboard(self):
        """Render real-time metrics with improved aesthetics and proper text wrapping"""
        state = self.bike.get_state()
        # Calculate progress-based score (increases as vehicle progresses)
        progress = min(1.0, state['x'] / self.bike.total_length)
        score = progress * 1000000 - state['total_energy']
        # score = max(0, score)  # REMOVED: Allow negative scores to show energy penalty impact
        
        # Status indicators
        status_color = SUCCESS_COLOR if state['completed'] else (
            ALERT_COLOR if state['failed'] else TEXT_COLOR
        )
        
        # Draw dashboard background
        dashboard_width = 320
        dashboard_height = 440  # Increased height to accommodate all text
        pygame.draw.rect(self.screen, DASHBOARD_BG, (10, 10, dashboard_width, dashboard_height), border_radius=15)
        pygame.draw.rect(self.screen, DASHBOARD_BORDER, (10, 10, dashboard_width, dashboard_height), 2, border_radius=15)
        
        # Score display - now increases with progress
        score_color = (0, 150, 0) if score > 0 else (200, 0, 0)
        score_text = self.font.render(f"SCORE: {score:,.0f}", True, score_color)
        self.screen.blit(score_text, (20, 25))
        
        # Energy display
        energy_color = WARNING_COLOR if state['total_energy'] > 50000 else TEXT_COLOR
        energy_text = self.monospace_font.render(f"ENERGY: {state['total_energy']:,.0f} J", True, energy_color)
        self.screen.blit(energy_text, (20, 60))
        
        # Time display
        time_color = ALERT_COLOR if state['time_limit_exceeded'] else TEXT_COLOR
        time_text = self.monospace_font.render(
            f"TIME: {state['total_time']:.1f}s / {TIME_LIMIT:.1f}s", 
            True, 
            time_color
        )
        self.screen.blit(time_text, (20, 90))
        
        # Gear display
        gear_text = self.monospace_font.render(f"GEAR: {state['gear_ratio']:.2f}", True, TEXT_COLOR)
        self.screen.blit(gear_text, (20, 120))
        
        # Slip counter
        slip_color = ALERT_COLOR if state['slip_count'] > 50 else TEXT_COLOR
        slip_text = self.monospace_font.render(f"SLIP COUNT: {state['slip_count']}", True, slip_color)
        self.screen.blit(slip_text, (20, 150))
        
        # Position and velocity
        pos_text = self.monospace_font.render(f"POSITION: {state['x']:.1f}m", True, TEXT_COLOR)
        vel_text = self.monospace_font.render(f"VELOCITY: {state['v']:.2f} m/s", True, TEXT_COLOR)
        self.screen.blit(pos_text, (20, 180))
        self.screen.blit(vel_text, (20, 210))
        
        # Track progress bar
        progress_width = 280
        progress_bar = pygame.Rect(20, 245, progress_width, 20)
        pygame.draw.rect(self.screen, (200, 200, 200), progress_bar)
        pygame.draw.rect(self.screen, (0, 150, 0), (20, 245, progress_width * progress, 20))
        pygame.draw.rect(self.screen, (0, 0, 0), progress_bar, 1)
        
        # Position label
        pos_label = self.small_font.render(f"Progress: {progress*100:.1f}%", True, TEXT_COLOR)
        self.screen.blit(pos_label, (150, 248))
        
        # Next segment warning
        next_seg = self.bike.get_next_segment()
        if next_seg:
            next_text = self.small_font.render(
                f"NEXT: {next_seg[0]:.0f}-{next_seg[1]:.0f}m | Slope={next_seg[2]:.2f} | μ={next_seg[3]:.2f}", 
                True, 
                WARNING_COLOR
            )
            # Check if text is too long and wrap if necessary
            if next_text.get_width() > dashboard_width - 40:
                # Split into two lines if too long
                part1 = self.small_font.render(
                    f"NEXT: {next_seg[0]:.0f}-{next_seg[1]:.0f}m | Slope={next_seg[2]:.2f}", 
                    True, 
                    WARNING_COLOR
                )
                part2 = self.small_font.render(
                    f"μ={next_seg[3]:.2f}", 
                    True, 
                    WARNING_COLOR
                )
                self.screen.blit(part1, (20, 275))
                self.screen.blit(part2, (20, 295))
            else:
                self.screen.blit(next_text, (20, 275))
        else:
            # If no next segment, show finish line info
            finish_text = self.small_font.render("FINISH LINE AHEAD!", True, SUCCESS_COLOR)
            self.screen.blit(finish_text, (20, 275))
        
        # Status message - handle long messages
        if state['completed']:
            status_msg = "FINISHED! Press R to restart"
            status_color = SUCCESS_COLOR
        elif state['failed']:
            reason = "TIME LIMIT EXCEEDED" if state['time_limit_exceeded'] else "TOO MANY SLIPS"
            status_msg = f"FAILED: {reason}. Press R to restart"
            status_color = ALERT_COLOR
        else:
            status_msg = "SPACE: Pause | R: Restart | L: Toggle Logging | H: Help"
            status_color = TEXT_COLOR
        
        # Wrap status message if too long
        status_lines = []
        words = status_msg.split()
        current_line = ""
        
        for word in words:
            test_line = current_line + word + " "
            if self.font.size(test_line)[0] < dashboard_width - 40:
                current_line = test_line
            else:
                status_lines.append(current_line)
                current_line = word + " "
        status_lines.append(current_line)
        
        # Draw status lines
        y_pos = 305
        for line in status_lines:
            status_text = self.font.render(line.strip(), True, status_color)
            self.screen.blit(status_text, (20, y_pos))
            y_pos += 28
        
        # Logging status
        log_status = "LOGGING: ON" if self.logging_enabled else "LOGGING: OFF"
        log_color = (0, 150, 0) if self.logging_enabled else (150, 150, 150)
        log_text = self.small_font.render(f"{log_status} (Press L to toggle)", True, log_color)
        
        # Place logging text below status messages
        self.screen.blit(log_text, (20, y_pos + 10))
    
    def draw_help(self):
        """Draw help overlay"""
        if not self.show_help:
            return
        
        # Semi-transparent background
        overlay = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
        overlay.fill((200, 200, 200, 200))
        self.screen.blit(overlay, (0, 0))
        
        # Help text
        help_lines = [
            "ECO-GEAR CHALLENGE HELP",
            "",
            "Objective: Minimize energy consumption while completing the track within the time limit",
            "",
            "Controls:",
            "  SPACE - Pause/resume simulation",
            "  R     - Reset simulation",
            "  L     - Toggle data logging",
            "  H     - Show/hide this help screen",
            "",
            "Physics Notes:",
            "  - Coasting (G=0) consumes ZERO energy",
            "  - Energy is consumed only when applying torque (G>0)",
            "  - Wheel slip occurs when drive force exceeds friction limit",
            "  - Excessive slip wastes massive amounts of energy",
            "",
            "Strategy Tips:",
            "  - Coast on downhills to save energy",
            "  - Use low gears (high ratio) for steep climbs to avoid slip",
            "  - Use high gears (low ratio) on flats for efficiency",
            "  - Monitor upcoming terrain using track_info",
            "",
            "Press H to close this help screen"
        ]
        
        y_pos = 100
        for line in help_lines:
            text = self.font.render(line, True, (0, 0, 0))
            self.screen.blit(text, (100, y_pos))
            y_pos += 35
    
    def handle_events(self):
        """Process user input"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_r:
                    self.bike.reset()
                    self.paused = False
                elif event.key == pygame.K_l:
                    self.logging_enabled = not self.logging_enabled
                elif event.key == pygame.K_h:
                    self.show_help = not self.show_help
                elif event.key == pygame.K_c:
                    # Reload controller (for development)
                    self.load_controller()
        
        return True
    
    def run(self):
        """Main simulation loop"""
        running = True
        
        while running:
            # Handle events
            running = self.handle_events()
            if not running:
                break
            
            # Physics update
            if not self.paused and not (self.bike.completed or self.bike.failed):
                # Prepare track_info for controller
                track_info = {
                    'segments': self.bike.track_segments,
                    'next_segment': self.bike.get_next_segment(),
                    'finish_line': self.bike.total_length
                }
                
                # Get gear ratio from controller
                try:
                    gear_ratio = self.controller.get_gear_ratio(
                        self.bike.x,
                        self.bike.v,
                        self.bike.current_slope,
                        self.bike.current_mu,
                        track_info
                    )
                    # Validate and clamp gear ratio
                    gear_ratio = float(gear_ratio)
                    gear_ratio = max(0.0, min(5.0, gear_ratio))
                except Exception as e:
                    print(f"Controller error: {e}")
                    gear_ratio = 1.0  # Safe default
                
                # Update physics
                state = self.bike.update(gear_ratio, 0.01)
                if state:
                    self.log_step(state)
            
            # Rendering
            self.screen.fill(BG_COLOR)
            self.draw_track()
            self.draw_bike()
            self.draw_dashboard()
            self.draw_help()
            
            pygame.display.flip()
            self.clock.tick(100)  # 100 FPS = 0.01s timestep
        
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    simulator = EcoGearSimulator()
    simulator.run()