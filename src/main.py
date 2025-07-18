import numpy as np
from OpenGL.GLUT import *
import sys

import config
import pendulum
import simulationUtils
import openglDisplay

def resetSimulation():
    config.NUM_PENDULUMS = len(config.L)
    config.NUM_STATE_VARS_PER_PENDULUM = 4 # hphi, omega_phi, theta, omega_theta
    expected_initial_state_len = config.NUM_PENDULUMS * config.NUM_STATE_VARS_PER_PENDULUM

    if len(config.INITIAL_STATE) != expected_initial_state_len:
        if config.NUM_PENDULUMS == 1:
            config.INITIAL_STATE[:] = [np.pi / 4, 0.0, np.pi / 4, 0.5]
        
        elif config.NUM_PENDULUMS == 2:
            config.INITIAL_STATE[:] = [np.pi / 4, 0.0, np.pi / 4, 0.5, np.pi / 4, 0.0, np.pi / 4, 0.0]
        
        elif config.NUM_PENDULUMS == 3:
            config.INITIAL_STATE[:] = [np.pi / 4, 0.0, np.pi / 4, 0.5, np.pi / 4, 0.0, np.pi / 4, 0.0, np.pi / 4, 0.0, np.pi / 4, 0.0]
        
        else:
            config.INITIAL_STATE[:] = [np.pi / 4, 0.0, np.pi / 4, 0.0] 
    
    config.currentState = np.array(config.INITIAL_STATE, dtype=float)
    config.simulationTime = 0.0
    config.tracePoints.clear()
    config.energyData.clear()
    config.angularMomentumData.clear()

    try:
        config.lastGlutTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0
    except:
        config.lastGlutTime = 0.0 

    config.lastDisplayTime = config.lastGlutTime 
    print("Simulation reset. Initial state:", config.currentState.tolist())
    glutPostRedisplay()

def keyboard(key, x, y):
    keyDecoded = key.decode("utf-8")

    if key == b'\x1b':
        sys.exit(0)

    elif keyDecoded == 'r':
        resetSimulation()

    elif keyDecoded == 't':
        config.SHOW_TRACE = not config.SHOW_TRACE
        print(f"Trace: {'Enabled' if config.SHOW_TRACE else 'Disabled'}")

    elif keyDecoded == 'p':
        simulationUtils.showAnalysisPlots()

    elif keyDecoded == '1':
        print("Switching to Single 3D Pendulum simulation.")
        config.L[:] = [2.5]
        config.M[:] = [5.0]

        # phi, omega_phi, theta, omega_theta
        config.INITIAL_STATE[:] = [np.pi / 4, 0.0, np.pi / 4, 0.5]
        config.NUM_STATE_VARS_PER_PENDULUM = 4
        resetSimulation()

    elif keyDecoded == '2':
        print("Switching to Double 3D Pendulum simulation.")
        config.L[:] = [1.0, 1.0]
        config.M[:] = [1.0, 1.0]

        # phi1, omega_phi1, theta1, omega_theta1, phi2, omega_phi2, theta2, omega_theta2
        config.INITIAL_STATE[:] = [np.pi / 4, 0.0, np.pi / 4, 0.5, np.pi / 4, 0.0, np.pi / 4, 0.0]
        config.NUM_STATE_VARS_PER_PENDULUM = 4
        resetSimulation()

    elif keyDecoded == '3':
        print("Switching to Triple 3D Pendulum simulation.")
        config.L[:] = [1.0, 1.0, 1.0]
        config.M[:] = [3.0, 3.0, 1.0]

        # phi1, omega_phi1, theta1, omega_theta1, phi2, omega_phi2, theta2, omega_theta2, phi3, omega_phi3, theta3, omega_theta3
        config.INITIAL_STATE[:] = [np.pi / 4, 0.0, np.pi / 4, 0.5, np.pi / 4, 0.0, np.pi / 4, 0.0, np.pi / 4, 0.0, np.pi / 4, 0.0]
        config.NUM_STATE_VARS_PER_PENDULUM = 4
        resetSimulation()

    glutPostRedisplay()

def mouse(button, state, x, y):
    if button == GLUT_LEFT_BUTTON:
        if state == GLUT_DOWN:
            config.mouseDown = True
            config.lastMouseX, config.lastMouseY = x, y
        elif state == GLUT_UP:
            config.mouseDown = False
    
    elif button == 3: # zoom in
        config.cameraDistance = max(0.1, config.cameraDistance - 0.2)
        glutPostRedisplay()

    elif button == 4: # zoom out
        config.cameraDistance += 0.2
        glutPostRedisplay()


def motion(x, y):
    if config.mouseDown:
        dx = x - config.lastMouseX
        dy = y - config.lastMouseY
        config.cameraAzimuth = (config.cameraAzimuth - dx * 0.5) % 360
        config.cameraElevation = max(-90, min(90, config.cameraElevation + dy * 0.5))
        config.lastMouseX, config.lastMouseY = x, y
        glutPostRedisplay()

def idle():
    currentGlutTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0
    elapsedSinceLastIdle = currentGlutTime - config.lastGlutTime
    config.lastGlutTime = currentGlutTime

    if elapsedSinceLastIdle < 1e-6:
        if (currentGlutTime - config.lastDisplayTime) >= config.DISPLAY_UPDATE_INTERVAL:
            glutPostRedisplay()
            config.lastDisplayTime = currentGlutTime
        return

    numRk4Steps = max(1, int(elapsedSinceLastIdle / config.RK4_DT))

    try:
        pendulumDerivsFunc = pendulum.getPendulumDerivsFunc(config.NUM_PENDULUMS)
    except ValueError as e:
        print(f"Error: {e}. Please select 1, 2 or 3 pendulums.")
        return

    for _ in range(numRk4Steps):
        if config.NUM_PENDULUMS == 1:
            config.currentState = simulationUtils.integrateRk4(
                pendulumDerivsFunc, config.simulationTime, config.currentState,
                config.RK4_DT, config.L[0], config.M[0], config.G
            )
        else:
            config.currentState = simulationUtils.integrateRk4(
                pendulumDerivsFunc, config.simulationTime, config.currentState,
                config.RK4_DT, config.L, config.M, config.G
            )

        config.simulationTime += config.RK4_DT

        kineticE, potentialE, totalE = simulationUtils.calculateEnergies(config.currentState, config.L, config.M, config.G)
        config.energyData.append([config.simulationTime, kineticE, potentialE, totalE])
        angMomX, angMomY, angMomZ = simulationUtils.calculateAngularMomentum(config.currentState, config.L, config.M)
        config.angularMomentumData.append([config.simulationTime, angMomX, angMomY, angMomZ])

        lastPendulumPositions = simulationUtils.calculateCartesianPositions(config.currentState, config.L)
        if config.NUM_PENDULUMS > 0:
            config.tracePoints.append(lastPendulumPositions[config.NUM_PENDULUMS])

    if (currentGlutTime - config.lastDisplayTime) >= config.DISPLAY_UPDATE_INTERVAL:
        glutPostRedisplay()
        config.lastDisplayTime = currentGlutTime

def main():
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
    glutInitWindowSize(config.WINDOW_WIDTH, config.WINDOW_HEIGHT)
    glutCreateWindow(config.TITLE.encode('utf-8'))

    openglDisplay.initGl(config.WINDOW_WIDTH, config.WINDOW_HEIGHT)

    glutDisplayFunc(openglDisplay.display)
    glutReshapeFunc(openglDisplay.reshape)
    glutKeyboardFunc(keyboard)
    glutMouseFunc(mouse)
    glutMotionFunc(motion)
    glutIdleFunc(idle)

    print("Controls:")
    print("  'q' or 'ESC': Exit the program")
    print("  'r': Reset simulation to initial conditions")
    print("  't': Toggle visibility of the last pendulum's trace")
    print("  'p': Show analysis plots (Energy and Angular Momentum)")
    print("  '1': Switch to Single 3D Pendulum simulation")
    print("  '2': Switch to Double 3D Pendulum simulation (Default)")
    print("  '3': Switch to Triple 3D Pendulum simulation")
    print("  Drag Mouse (Left Click): Rotate camera.")
    print("  Mouse Scroll Wheel: Zoom in/out.")

    resetSimulation()
    glutMainLoop()

if __name__ == "__main__":
    main()