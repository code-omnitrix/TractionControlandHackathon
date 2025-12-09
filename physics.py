import numpy as np

class BicyclePhysics:
    def __init__(self, track_segments, time_limit):
        # Constants
        self.m = 70.0       # kg (rider + bike)
        self.g = 9.81       # m/s^2
        self.R = 0.35       # m (wheel radius)
        self.I = 0.3        # kg*m^2 (Moment of Inertia for one wheel)
        self.T_input = 100  # Nm (fixed input torque)
        self.C_rr = 0.01    # Rolling resistance coefficient
        self.rho = 1.2      # kg/m^3 (air density)
        self.C_d = 0.9      # Drag coefficient
        self.A = 0.5        # m^2 (frontal area)
        
        # Track setup
        self.track_segments = track_segments
        self.total_length = max(seg[1] for seg in track_segments)
        
        # State variables
        self.reset()
        self.time_limit = time_limit
        
    def reset(self):
        self.x = 0.0          # Position (m)
        self.v = 0.0          # Velocity (m/s)
        self.omega = 0.0      # Wheel angular velocity (rad/s)
        self.elevation = 0.0  # Current elevation (m)
        self.total_time = 0.0
        self.total_energy = 0.0
        self.slip_count = 0
        self.completed = False
        self.failed = False
        self.time_limit_exceeded = False
        self.last_segment_idx = -1
        self.current_slope = 0.0
        self.current_mu = 0.8
        self.gear_ratio = 1.0
        self.slip_event = False
        self.forces = {
            'drive': 0.0,
            'gravity': 0.0,
            'drag': 0.0,
            'roll': 0.0,
            'front_dynamic_drag': 0.0
        }
        
    def get_current_segment(self, x):
        """Find segment containing position x"""
        for i, (start, end, slope, mu) in enumerate(self.track_segments):
            if start <= x < end:
                return i, (start, end, slope, mu)
        return len(self.track_segments)-1, self.track_segments[-1]
    
    def get_next_segment(self):
        """Get next segment info if available"""
        current_idx, _ = self.get_current_segment(self.x)
        if current_idx < len(self.track_segments) - 1:
            return self.track_segments[current_idx + 1]
        return None
    
    def update_elevation(self, dx):
        """Update elevation based on slope"""
        segment_idx, (start, end, slope, mu) = self.get_current_segment(self.x)
        if segment_idx != self.last_segment_idx:
            self.last_segment_idx = segment_idx
            self.current_slope = slope
            self.current_mu = mu
            
        self.elevation += slope * dx

    def get_tire_force(self, slip_ratio, normal_force, mu):
        """
        Calculate tire traction force using a simplified Magic Formula.
        Fx = D * sin(C * arctan(B * SR))
        """
        D = mu * normal_force
        B = 10.0
        C = 1.9
        return D * np.sin(C * np.arctan(B * slip_ratio))
    
    def calculate_forces(self, gear_ratio):
        """Calculate all forces acting on the bike"""
        # Normal force
        theta = np.arctan(self.current_slope)
        N = self.m * self.g * np.cos(theta)
        
        # Calculate Slip Ratio
        # SR = (omega * R - v) / |v| (with epsilon)
        v_safe = max(abs(self.v), 0.1) # Avoid div by zero
        slip_ratio = (self.omega * self.R - self.v) / v_safe
        
        # Calculate Rear Traction Fx_rear
        F_x_rear = self.get_tire_force(slip_ratio, N, self.current_mu)
        
        # Calculate Rear Wheel Dynamics
        # I * alpha = tau - Fx * R
        tau = self.T_input * gear_ratio if gear_ratio > 0 else 0.0
        alpha_rear = (tau - F_x_rear * self.R) / self.I
        
        # Forces on Body
        F_gravity = -self.m * self.g * np.sin(theta)
        F_roll = -self.C_rr * N * np.sign(self.v + 1e-5)
        F_drag = -0.5 * self.rho * self.C_d * self.A * self.v**2 * np.sign(self.v + 1e-5)
        
        # F_net_static (excluding front wheel dynamic drag)
        F_net_static = F_x_rear + F_gravity + F_roll + F_drag
        
        # Effective Mass for linear acceleration
        # F_net_static = (m + I/R^2) * a
        m_eff = self.m + (self.I / self.R**2)
        a_bike = F_net_static / m_eff
        
        # Back calculate F_x_front (Front wheel drag due to spin up)
        # I * alpha_front = F_x_front * R
        # alpha_front = a_bike / R (assuming no slip)
        # F_x_front = I * a_bike / R^2
        F_x_front = (self.I * a_bike) / (self.R**2)
        
        # True Net Force on Body (m * a)
        F_net = F_net_static - F_x_front
        
        # Check for slip event (threshold based)
        self.slip_event = abs(slip_ratio) > 0.15 # Threshold for "excessive slip"
        if self.slip_event and gear_ratio > 0:
             self.slip_count += 1
        
        self.forces = {
            'drive': F_x_rear,
            'gravity': F_gravity,
            'drag': F_drag,
            'roll': F_roll,
            'front_dynamic_drag': F_x_front
        }
        
        return F_net, F_x_rear, alpha_rear
    
    def update(self, gear_ratio, dt):
        """Update physics state"""
        if self.completed or self.failed:
            return
        
        self.gear_ratio = gear_ratio
        self.total_time += dt
        
        # Check time limit
        if self.total_time > self.time_limit:
            self.time_limit_exceeded = True
            self.failed = True
            return
        
        # Get current segment properties
        _, (_, _, self.current_slope, self.current_mu) = self.get_current_segment(self.x)
        
        # Calculate forces
        F_net, F_drive, alpha_rear = self.calculate_forces(gear_ratio)
        
        # Energy consumption (only when driving)
        energy_step = 0.0
        if gear_ratio > 0:
            # Power = Torque * Omega
            power = (self.T_input * gear_ratio) * self.omega
            # Ensure power is non-negative (engine doesn't absorb energy)
            power = max(0.0, power)
            energy_step = power * dt
            self.total_energy += energy_step
        
        # Update velocity and position
        a = F_net / self.m
        self.v += a * dt
        
        # Update wheel angular velocity
        self.omega += alpha_rear * dt
        
        # Prevent negative velocity/omega (simple constraint for this simulation)
        if self.v < 0:
            self.v = 0
        if self.omega < 0:
            self.omega = 0
        
        dx = self.v * dt
        self.x += dx
        self.update_elevation(dx)
        
        # Check completion
        if self.x >= self.total_length:
            self.completed = True
            self.x = self.total_length
        
        # Check for stalling on uphill
        if self.current_slope > 0 and self.v < 0.01 and F_net < 0:
            self.failed = True
        else:
            self.failed = False
        
        # Check slip limit (REMOVED: Slip is now a natural part of the physics model)
        # if self.slip_count > 5:
        #    self.failed = True
        
        return {
            'x': self.x,
            'v': self.v,
            'elevation': self.elevation,
            'total_time': self.total_time,
            'total_energy': self.total_energy,
            'slip_count': self.slip_count,
            'completed': self.completed,
            'failed': self.failed,
            'time_limit_exceeded': self.time_limit_exceeded,
            'gear_ratio': self.gear_ratio,
            'slip_event': self.slip_event,
            'energy_step': energy_step,
            'segment_idx': self.last_segment_idx,
            'slope': self.current_slope,
            'mu': self.current_mu,
            'forces': self.forces,
            'F_drive': self.forces['drive'],
            'F_gravity': self.forces['gravity'],
            'F_drag': self.forces['drag'],
            'F_roll': self.forces['roll']
        }
    
    def get_state(self):
        """Get current state for logging"""
        return {
            'x': self.x,
            'v': self.v,
            'total_time': self.total_time,
            'total_energy': self.total_energy,
            'slip_count': self.slip_count,
            'completed': self.completed,
            'failed': self.failed,
            'time_limit_exceeded': self.time_limit_exceeded,
            'gear_ratio': self.gear_ratio,
            'slip_event': self.slip_event,
            'segment_idx': self.last_segment_idx,
            'slope': self.current_slope,
            'mu': self.current_mu,
            'F_drive': self.forces['drive'],
            'F_gravity': self.forces['gravity'],
            'F_drag': self.forces['drag'],
            'F_roll': self.forces['roll']
        }