def get_gear_ratio(x, v, slope, mu, track_info):
    """
    Dynamic gear shifting strategy for the Eco-Gear Challenge
    
    Args:
        x (float): Current position in meters
        v (float): Current velocity in m/s
        slope (float): Current segment slope (fraction, e.g., 0.3 = 30% grade)
        mu (float): Current friction coefficient
        track_info (dict): Dictionary containing track information with keys:
            - 'segments': List of all track segments [(start, end, slope, mu), ...]
            - 'next_segment': Next segment info (start, end, slope, mu) or None
            - 'finish_line': Position of finish line in meters
    
    Returns:
        float: Gear ratio in range [0.0, 5.0] 
               (0.0 = coasting, 0.1-5.0 = active gear ratios)
    """
    # Basic strategy: Coast on downhills, use appropriate gears elsewhere
    if slope < -0.05:  # Downhill section
        return 0.0  # Coasting (zero energy consumption)
    
    # Check upcoming terrain if available
    next_seg = track_info.get('next_segment')
    if next_seg:
        next_start, next_end, next_slope, next_mu = next_seg
        
        # Prepare for steep uphill
        if next_slope > 0.4 and x > next_start - 10:  # 10m before steep climb
            return 4.5  # Low gear for maximum torque
        
        # Prepare for icy section (low friction)
        if next_mu < 0.4 and x > next_start - 5:  # 5m before low-friction zone
            return min(2.0, max(0.5, 1.0 / (next_mu + 0.1)))  # Conservative gear
    
    # Current segment strategy
    if slope > 0.3:  # Steep uphill
        return 4.0   # Low gear for climbing
    elif slope > 0.1:  # Moderate uphill
        return 2.5
    elif v > 10.0:   # High speed on flat/downhill
        return 0.8   # High gear for efficiency
    else:
        return 1.5   # Default gear for moderate terrain