import numpy as np
from collections import deque

G = 9.81 

NUM_PENDULUMS = 2 
NUM_STATE_VARS_PER_PENDULUM = 4 # phi, omega_phi, theta, omega_theta

# Pendulum variables:
# phi: polar angle (angle from negative Y-axis, 0 = straight down)
# omega_phi: angular velocity of phi
# theta: azimuthal angle (rotation around Y-axis, like longitude)
# omega_theta: angular velocity of theta

L = [1.0, 1.0]
M = [1.0, 1.0]
INITIAL_STATE = [np.pi / 4, 0.0, np.pi / 4, 0.5, np.pi / 4, 0.0, np.pi / 4, 0.0]

# RK4 integration step size
RK4_DT = 0.002

# Window Config
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
TITLE = "3D Pendulum Simulation"

# Display and Trace Settings
SHOW_TRACE = True                                   # Show/hide the trace of the last mass
TRACE_ALPHA = 0.4                                   # Trace transparency (value between 0.0 and 1.0)
DISPLAY_UPDATE_INTERVAL = 1/60.0                    # Target frame rate for display updates

# Global variables
currentState = None
lastGlutTime = 0.0                                  # Last time recorded by GLUT for delta_time calculation
lastDisplayTime = 0.0                               # Last time display was updated, for frame rate control
simulationTime = 0.0                                # Accumulated simulation time

tracePoints = deque(maxlen=10000)                   # List of (x, y, z) tuples for the last mass's trace
energyData = deque(maxlen=20000)                    # Stores [time, K.E., P.E., Total E.]
angularMomentumData = deque(maxlen=20000)           # Stores [time, X-Angular Momentum, Y-Angular Momentum, Z-Angular Momentum]

# 3D Camera Config
cameraAzimuth = 45.0
cameraElevation = 30.0
cameraDistance = sum(L) * 2.0
mouseDown = False
lastMouseX, lastMouseY = 0, 0