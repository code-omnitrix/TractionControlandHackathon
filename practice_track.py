# Practice track configuration
PRACTICE_TRACK = [
    (0, 50, 0.0, 0.8),    # 0-50m: Flat, high friction (asphalt)
    (50, 100, 0.1, 0.6),  # 50-100m: Mild uphill, medium friction
    (100, 150, -0.1, 0.4) # 100-150m: Gentle downhill, low friction (wet surface)
]

# Time limit (150% of theoretical minimum time)
TIME_LIMIT = 35.0  # seconds