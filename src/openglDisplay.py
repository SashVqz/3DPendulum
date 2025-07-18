from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np
import config
import simulationUtils

def initGl(width, height):
    glClearColor(0.1, 0.1, 0.1, 1.0)               
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glEnable(GL_LINE_SMOOTH)
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
    glEnable(GL_POINT_SMOOTH)
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()

    reshape(width, height)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()

    # Set up camera 
    eyeX = config.cameraDistance * np.cos(np.radians(config.cameraElevation)) * np.sin(np.radians(config.cameraAzimuth))
    eyeY = config.cameraDistance * np.sin(np.radians(config.cameraElevation))
    eyeZ = config.cameraDistance * np.cos(np.radians(config.cameraElevation)) * np.cos(np.radians(config.cameraAzimuth))
    gluLookAt(eyeX, eyeY, eyeZ, 0, 0, 0, 0, 1, 0)

    drawAxes()
    pendulumPositions = simulationUtils.calculateCartesianPositions(config.currentState, config.L)

    glPointSize(10)
    glColor3f(1.0, 1.0, 1.0) 
    glBegin(GL_POINTS)
    glVertex3f(0.0, 0.0, 0.0)
    glEnd()

    for i in range(config.NUM_PENDULUMS):
        parentPos = pendulumPositions[i]  
        childPos = pendulumPositions[i+1] 

        if i == config.NUM_PENDULUMS - 1: glColor3f(1.0, 0.5, 0.5)   # Light Red for last mass
        else: glColor3f(0.4, 0.7, 1.0)                               # Light Blue for other masses

        glLineWidth(4)
        glBegin(GL_LINES)
        glVertex3f(parentPos[0], parentPos[1], parentPos[2])
        glVertex3f(childPos[0], childPos[1], childPos[2])
        glEnd()

        glPointSize(20.0)
        glBegin(GL_POINTS)
        glVertex3f(childPos[0], childPos[1], childPos[2])
        glEnd()

    if config.SHOW_TRACE and len(config.tracePoints) > 1:
        glColor4f(0.8, 0.8, 0.2, config.TRACE_ALPHA)                 # Yellowish trace
        glLineWidth(2)
        glBegin(GL_LINE_STRIP)

        for pointX, pointY, pointZ in config.tracePoints: glVertex3f(pointX, pointY, pointZ)
        
        glEnd()

    glutSwapBuffers()

def drawAxes():
    axisLength = sum(config.L) * 0.75
    glLineWidth(2)

    # X-axis (Red)
    glColor3f(1.0, 0.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(axisLength, 0.0, 0.0)
    glEnd()
    glPointSize(5)
    glBegin(GL_POINTS)
    glVertex3f(axisLength + 0.1, 0.0, 0.0)
    glEnd()

    glColor3f(1.0, 0.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(-axisLength, 0.0, 0.0)
    glEnd()
    glPointSize(5)
    glBegin(GL_POINTS)
    glVertex3f(-axisLength - 0.1, 0.0, 0.0)
    glEnd()

    # Y-axis (Green)
    glColor3f(0.0, 1.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, axisLength , 0.0)
    glEnd()
    glPointSize(5)
    glBegin(GL_POINTS)
    glVertex3f(0.0, axisLength + 0.1, 0.0)
    glEnd()

    glColor3f(0.0, 1.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, -axisLength , 0.0)
    glEnd()
    glPointSize(5)
    glBegin(GL_POINTS)
    glVertex3f(0.0, -axisLength - 0.1, 0.0)
    glEnd()

    # Z-axis (Blue)
    glColor3f(0.0, 0.0, 1.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 0.0, axisLength)
    glEnd()
    glPointSize(5)
    glBegin(GL_POINTS)
    glVertex3f(0.0, 0.0, axisLength + 0.1)
    glEnd()

    glColor3f(0.0, 0.0, 1.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 0.0, -axisLength)
    glEnd()
    glPointSize(5)
    glBegin(GL_POINTS)
    glVertex3f(0.0, 0.0, -axisLength - 0.1)
    glEnd()

def reshape(width, height):
    if height == 0: height = 1

    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()

    aspectRatio = float(width) / float(height)
    gluPerspective(45, aspectRatio, 0.1, config.cameraDistance * 3) # fovY, aspectRatio, zNear, zFar

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()