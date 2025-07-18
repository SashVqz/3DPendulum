import numpy as np
import matplotlib.pyplot as plt
import config

def integrateRk4(derivsFunc, currentT, currentStateArray, dt, *args):
    """
    Performs one step of Runge-Kutta 4th order integration.
    """
    k1 = dt * derivsFunc(currentT, currentStateArray, *args)
    k2 = dt * derivsFunc(currentT + 0.5 * dt, currentStateArray + 0.5 * k1, *args)
    k3 = dt * derivsFunc(currentT + 0.5 * dt, currentStateArray + 0.5 * k2, *args)
    k4 = dt * derivsFunc(currentT + dt, currentStateArray + k3, *args)

    nextStateArray = currentStateArray + (k1 + 2*k2 + 2*k3 + k4) / 6.0

    return nextStateArray

def calculateCartesianPositions(state, lengths):
    """
    Calculates the Cartesian (x, y, z) positions of all pendulum masses.
    Assumes the pivot is at (0,0,0) and the pendulums hang downwards along the -Y axis initially.
    """
    positions = [(0.0, 0.0, 0.0)] # The first point is the origin (pivot)
    currentX, currentY, currentZ = 0.0, 0.0, 0.0 # Current pivot position

    for i in range(config.NUM_PENDULUMS):
        # Each pendulum has 4 state variables: phi, omega_phi, theta, omega_theta
        # So, the index for phi is i*NUM_STATE_VARS_PER_PENDULUM (i*4)
        # And the index for theta is i*NUM_STATE_VARS_PER_PENDULUM + 2 (i*4 + 2)
        phi = state[i * config.NUM_STATE_VARS_PER_PENDULUM]
        theta = state[i * config.NUM_STATE_VARS_PER_PENDULUM + 2]
        length = lengths[i]

        # Calculate the displacement of this rod in 3D using spherical coordinates
        # Assuming origin is (0,0,0) and pendulum hangs downwards along -Y axis.
        # X = L * sin(phi) * sin(theta)  (theta is azimuthal rotation around Y)
        # Y = -L * cos(phi)              (phi is angle from -Y axis)
        # Z = L * sin(phi) * cos(theta)

        dx = length * np.sin(phi) * np.sin(theta)
        dy = -length * np.cos(phi)
        dz = length * np.sin(phi) * np.cos(theta)

        # Update the mass position and the pivot for the next rod
        currentX += dx
        currentY += dy
        currentZ += dz # Now includes Z component
        positions.append((currentX, currentY, currentZ)) # Add the current mass's position

    return positions

def calculateEnergies(state, lengths, masses, g):
    kineticEnergy = 0.0
    potentialEnergy = 0.0
    positions = calculateCartesianPositions(state, lengths)

    for i in range(config.NUM_PENDULUMS):
        mass = masses[i]
        current_mass_x, current_mass_y, current_mass_z = positions[i + 1]

        phi = state[i * config.NUM_STATE_VARS_PER_PENDULUM]
        phi_dot = state[i * config.NUM_STATE_VARS_PER_PENDULUM + 1]
        theta = state[i * config.NUM_STATE_VARS_PER_PENDULUM + 2]
        theta_dot = state[i * config.NUM_STATE_VARS_PER_PENDULUM + 3]
        length = lengths[i]

        # For spherical coordinates (r, phi, theta) with r=L (constant length)
        # x = L sin(phi) sin(theta)
        # y = -L cos(phi)
        # z = L sin(phi) cos(theta)

        # vx = d(x)/dt = L * (phi_dot*cos(phi)*sin(theta) + theta_dot*sin(phi)*cos(theta))
        # vy = d(y)/dt = L * (phi_dot*sin(phi))
        # vz = d(z)/dt = L * (phi_dot*cos(phi)*cos(theta) - theta_dot*sin(phi)*sin(theta))

        vx_segment = length * (phi_dot * np.cos(phi) * np.sin(theta) + theta_dot * np.sin(phi) * np.cos(theta))
        vy_segment = length * (phi_dot * np.sin(phi))
        vz_segment = length * (phi_dot * np.cos(phi) * np.cos(theta) - theta_dot * np.sin(phi) * np.sin(theta))

        v_squared = vx_segment**2 + vy_segment**2 + vz_segment**2
        kineticEnergy += 0.5 * mass * v_squared

        # Potential energy: m * g * h
        height = current_mass_y # y axis
        potentialEnergy += mass * g * height

    totalEnergy = kineticEnergy + potentialEnergy
    return kineticEnergy, potentialEnergy, totalEnergy

def calculateAngularMomentum(state, lengths, masses):
    angularMomentumX = 0.0
    angularMomentumY = 0.0
    angularMomentumZ = 0.0

    positions = calculateCartesianPositions(state, lengths)

    for i in range(config.NUM_PENDULUMS):
        mass = masses[i]
        x, y, z = positions[i + 1] 

        phi = state[i * config.NUM_STATE_VARS_PER_PENDULUM]
        phi_dot = state[i * config.NUM_STATE_VARS_PER_PENDULUM + 1]
        theta = state[i * config.NUM_STATE_VARS_PER_PENDULUM + 2]
        theta_dot = state[i * config.NUM_STATE_VARS_PER_PENDULUM + 3]
        length = lengths[i]

        vx_segment = length * (phi_dot * np.cos(phi) * np.sin(theta) + theta_dot * np.sin(phi) * np.cos(theta))
        vy_segment = length * (phi_dot * np.sin(phi))
        vz_segment = length * (phi_dot * np.cos(phi) * np.cos(theta) - theta_dot * np.sin(phi) * np.sin(theta))

        # Angular momentum components L = r x mv
        # L_x = m * (y*vz - z*vy)
        # L_y = m * (z*vx - x*vz)
        # L_z = m * (x*vy - y*vx)

        angularMomentumX += mass * (y * vz_segment - z * vy_segment)
        angularMomentumY += mass * (z * vx_segment - x * vz_segment)
        angularMomentumZ += mass * (x * vy_segment - y * vx_segment)

    return angularMomentumX, angularMomentumY, angularMomentumZ

def showAnalysisPlots():
    if not config.energyData:
        print("No analysis data to show. Run the simulation first.")
        return

    times = [d[0] for d in config.energyData]
    kineticE = [d[1] for d in config.energyData]
    potentialE = [d[2] for d in config.energyData]
    totalE = [d[3] for d in config.energyData]

    plt.style.use('dark_background')
    plt.rcParams.update({
        'axes.facecolor': "#000000",
        'figure.facecolor': '#000000',
        'text.color': 'white',
        'axes.labelcolor': 'white',
        'xtick.color': 'white',
        'ytick.color': 'white',
        'grid.color': '#444444',
        'legend.facecolor': '#000000',
        'legend.edgecolor': '#444444',
        'axes.edgecolor': '#444444'
    })

    fig, axes = plt.subplots(1, 2, figsize=(10, 5))

    # Energy Conservation Plot
    axes[0].plot(times, kineticE, label='Kinetic Energy', color='#61AFEF') # Blue
    axes[0].plot(times, potentialE, label='Potential Energy', color='#98C379') # Green
    axes[0].plot(times, totalE, label='Total Energy', color='#E06C75', linewidth=2) # Thicker Red
    axes[0].set_title('Energy Conservation (Approx.)')
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Energy (J)')
    axes[0].legend()
    axes[0].grid(True, linestyle='--', alpha=0.6)

    # Angular Momentum Plot
    if config.angularMomentumData:
        angMomX = [d[1] for d in config.angularMomentumData]
        angMomY = [d[2] for d in config.angularMomentumData]
        angMomZ = [d[3] for d in config.angularMomentumData]

        axes[1].plot(times, angMomX, label='Angular Momentum X', color='#C678DD') # Purple
        axes[1].plot(times, angMomY, label='Angular Momentum Y', color='#56B6C2') # Teal
        axes[1].plot(times, angMomZ, label='Angular Momentum Z', color='#E6C07B') # Orange

        axes[1].set_title('System Angular Momentum (Approx.)')
        axes[1].set_xlabel('Time (s)')
        axes[1].set_ylabel('Angular Momentum (kg·m²/s)')
        axes[1].legend()
        axes[1].grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout()
    plt.show()