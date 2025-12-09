# Eco-Gear Challenge Simulator

A Python-based simulation environment for developing energy-efficient gear shifting strategies.

## Setup Instructions

### Prerequisites
- Python 3.8 or higher
- pip (Python package installer)

### Installation

1. Navigate to the project directory:
   ```bash
   cd TractionControlandHackathon
   ```

2. Install the required dependencies:
   ```bash
   pip install pygame numpy
   ```

## Usage

To start the simulator, run the main script:

```bash
python main.py
```

## How to Develop Your Strategy

1. Open `controller_template.py` in your text editor.
2. Modify the `get_gear_ratio` function to implement your logic.
3. Your goal is to minimize energy consumption while completing the track within the time limit.
4. The function receives the current state (position, velocity, slope, friction) and track information.
5. Return a gear ratio between `0.0` (coasting) and `5.0`.

## Simulator Controls

- **SPACE**: Pause / Resume simulation
- **R**: Restart simulation
- **L**: Toggle data logging (saves to CSV)
- **H**: Show / Hide help screen
- **C**: Reload controller file (useful for testing changes without restarting the app)
