import numpy as np
import warnings

np.seterr(over='warn', invalid='warn')
def safe_divide(numerator, denominator, fallback=0.0):
    if abs(denominator) < 1e-10:
        return fallback
    try:
        result = numerator / denominator
        
        if np.isnan(result) or np.isinf(result):
            return fallback
        
        return result
    
    except (OverflowError, FloatingPointError):
        return fallback

def safe_power(base, exponent, max_val=1e10):
    try:
        if abs(base) > 1e5: 
            base = np.sign(base) * 1e5

        result = base ** exponent

        if np.isnan(result) or np.isinf(result) or abs(result) > max_val:
            return np.sign(result) * max_val if not np.isnan(result) else 0.0
        
        return result
    
    except (OverflowError, FloatingPointError):
        return max_val if base > 0 else -max_val

def clamp_state(state, max_angular_velocity=50.0):
    state = np.array(state)
    for i in range(1, len(state), 2):
        if abs(state[i]) > max_angular_velocity:
            state[i] = np.sign(state[i]) * max_angular_velocity
    
    return state

def validate_initial_conditions(state, max_angle=np.pi, max_angular_velocity=10.0):
    state = np.array(state)
    
    for i in range(0, len(state), 2):
        if abs(state[i]) > max_angle:
            state[i] = np.sign(state[i]) * max_angle
    
    for i in range(1, len(state), 2):
        if abs(state[i]) > max_angular_velocity:
            state[i] = np.sign(state[i]) * max_angular_velocity
    
    return state

def get_safe_initial_conditions(num_pendulums, amplitude=0.5, max_velocity=2.0):
    np.random.seed(42)    
    state = []

    for i in range(num_pendulums):
        phi = amplitude * np.random.uniform(-1, 1)
        if abs(phi) < 0.1:
            phi = 0.1 * np.sign(phi) if phi != 0 else 0.1
        
        omega_phi = max_velocity * np.random.uniform(-0.5, 0.5)  
        theta = 2 * np.pi * np.random.uniform(-0.5, 0.5)
        omega_theta = max_velocity * np.random.uniform(-0.5, 0.5)
        state.extend([phi, omega_phi, theta, omega_theta])
    
    return np.array(state)

def single3DPendulumDerivs(t, state, length, mass, g):
    state = clamp_state(state)
    phi, omega_phi, theta, omega_theta = state
    
    damping = 0.02
    damping_term = -damping * omega_phi

    sin_phi = np.sin(phi)
    cos_phi = np.cos(phi)
    
    if abs(sin_phi) < 1e-6:
        sin_phi = 1e-6 * np.sign(sin_phi) if sin_phi != 0 else 1e-6
    
    # Aceleración angular en phi
    # Término centrífugo + término gravitacional + amortiguamiento
    centrifugal_term = safe_power(omega_theta, 2) * sin_phi * cos_phi
    gravitational_term = -(g / length) * sin_phi
    
    d_omega_phi_dt = centrifugal_term + gravitational_term + damping_term
    
    # Aceleración angular en theta
    # Término de Coriolis + amortiguamiento
    coriolis_term = safe_divide(-2 * omega_phi * omega_theta, np.tan(phi), 0.0)
    
    d_omega_theta_dt = coriolis_term - damping * omega_theta
    
    derivatives = np.array([omega_phi, d_omega_phi_dt, omega_theta, d_omega_theta_dt])
    derivatives = np.nan_to_num(derivatives, nan=0.0, posinf=1e10, neginf=-1e10)    
    return derivatives

def double3DPendulumDerivs(t, state, lengths, masses, g):
    state = clamp_state(state)
    phi1, omega_phi1, theta1, omega_theta1, phi2, omega_phi2, theta2, omega_theta2 = state
    
    L1, L2 = lengths
    m1, m2 = masses
    
    # Parámetros de amortiguamiento y acoplamiento
    damping = 0.03
    coupling_strength = 0.02
    
    base_state1 = [phi1, omega_phi1, theta1, omega_theta1]
    base_state2 = [phi2, omega_phi2, theta2, omega_theta2]
    
    base_derivs1 = single3DPendulumDerivs(t, base_state1, L1, m1, g)
    base_derivs2 = single3DPendulumDerivs(t, base_state2, L2, m2, g)
    
    accel_phi1 = base_derivs1[1]
    accel_theta1 = base_derivs1[3]
    accel_phi2 = base_derivs2[1]
    accel_theta2 = base_derivs2[3]    
    
    mass_ratio12 = m2 / (m1 + m2)
    mass_ratio21 = m1 / (m1 + m2) 
    delta_phi = phi1 - phi2
    delta_theta = theta1 - theta2
    
    coupling_phi1 = coupling_strength * mass_ratio12 * np.sin(delta_phi) * (1 + 0.1 * np.cos(delta_theta))
    coupling_phi2 = -coupling_strength * mass_ratio21 * np.sin(delta_phi) * (1 + 0.1 * np.cos(delta_theta))
    coupling_theta1 = coupling_strength * mass_ratio12 * np.sin(delta_theta) * (1 + 0.1 * np.cos(delta_phi))
    coupling_theta2 = -coupling_strength * mass_ratio21 * np.sin(delta_theta) * (1 + 0.1 * np.cos(delta_phi))
    
    accel_phi1 += coupling_phi1
    accel_phi2 += coupling_phi2
    accel_theta1 += coupling_theta1
    accel_theta2 += coupling_theta2
    
    accel_phi1 -= damping * omega_phi1
    accel_theta1 -= damping * omega_theta1
    accel_phi2 -= damping * omega_phi2
    accel_theta2 -= damping * omega_theta2
    
    derivatives = np.array([
        omega_phi1, accel_phi1, omega_theta1, accel_theta1,
        omega_phi2, accel_phi2, omega_theta2, accel_theta2
    ])
    derivatives = np.nan_to_num(derivatives, nan=0.0, posinf=1e10, neginf=-1e10)
    
    return derivatives

def triple3DPendulumDerivs(t, state, lengths, masses, g):
    state = clamp_state(state)
    
    phi1, omega_phi1, theta1, omega_theta1, \
    phi2, omega_phi2, theta2, omega_theta2, \
    phi3, omega_phi3, theta3, omega_theta3 = state
    
    L1, L2, L3 = lengths
    m1, m2, m3 = masses
    
    damping = 0.04
    coupling_strength = 0.015
    
    base_state1 = [phi1, omega_phi1, theta1, omega_theta1]
    base_state2 = [phi2, omega_phi2, theta2, omega_theta2]
    base_state3 = [phi3, omega_phi3, theta3, omega_theta3]
    
    base_derivs1 = single3DPendulumDerivs(t, base_state1, L1, m1, g)
    base_derivs2 = single3DPendulumDerivs(t, base_state2, L2, m2, g)
    base_derivs3 = single3DPendulumDerivs(t, base_state3, L3, m3, g)
    
    accel_phi1 = base_derivs1[1]
    accel_theta1 = base_derivs1[3]
    accel_phi2 = base_derivs2[1]
    accel_theta2 = base_derivs2[3]
    accel_phi3 = base_derivs3[1]
    accel_theta3 = base_derivs3[3]
    
    mass_ratio12 = m2 / (m1 + m2)
    mass_ratio21 = m1 / (m1 + m2)
    delta_phi12 = phi1 - phi2
    delta_theta12 = theta1 - theta2
    
    coupling_phi1_2 = coupling_strength * mass_ratio12 * np.sin(delta_phi12)
    coupling_phi2_1 = -coupling_strength * mass_ratio21 * np.sin(delta_phi12)
    coupling_theta1_2 = coupling_strength * mass_ratio12 * np.sin(delta_theta12)
    coupling_theta2_1 = -coupling_strength * mass_ratio21 * np.sin(delta_theta12)
    
    mass_ratio23 = m3 / (m2 + m3)
    mass_ratio32 = m2 / (m2 + m3)
    delta_phi23 = phi2 - phi3
    delta_theta23 = theta2 - theta3
    
    coupling_phi2_3 = coupling_strength * mass_ratio23 * np.sin(delta_phi23)
    coupling_phi3_2 = -coupling_strength * mass_ratio32 * np.sin(delta_phi23)
    coupling_theta2_3 = coupling_strength * mass_ratio23 * np.sin(delta_theta23)
    coupling_theta3_2 = -coupling_strength * mass_ratio32 * np.sin(delta_theta23)
    
    mass_ratio13 = m3 / (m1 + m2 + m3)
    mass_ratio31 = m1 / (m1 + m2 + m3)
    delta_phi13 = phi1 - phi3
    delta_theta13 = theta1 - theta3
    
    coupling_phi1_3 = 0.5 * coupling_strength * mass_ratio13 * np.sin(delta_phi13)
    coupling_phi3_1 = -0.5 * coupling_strength * mass_ratio31 * np.sin(delta_phi13)
    coupling_theta1_3 = 0.5 * coupling_strength * mass_ratio13 * np.sin(delta_theta13)
    coupling_theta3_1 = -0.5 * coupling_strength * mass_ratio31 * np.sin(delta_theta13)
    
    accel_phi1 += coupling_phi1_2 + coupling_phi1_3
    accel_theta1 += coupling_theta1_2 + coupling_theta1_3    
    accel_phi2 += coupling_phi2_1 + coupling_phi2_3
    accel_theta2 += coupling_theta2_1 + coupling_theta2_3
    accel_phi3 += coupling_phi3_2 + coupling_phi3_1
    accel_theta3 += coupling_theta3_2 + coupling_theta3_1
    
    accel_phi1 -= damping * omega_phi1
    accel_theta1 -= damping * omega_theta1
    accel_phi2 -= damping * omega_phi2
    accel_theta2 -= damping * omega_theta2
    accel_phi3 -= damping * omega_phi3
    accel_theta3 -= damping * omega_theta3
    
    derivatives = np.array([
        omega_phi1, accel_phi1, omega_theta1, accel_theta1,
        omega_phi2, accel_phi2, omega_theta2, accel_theta2,
        omega_phi3, accel_phi3, omega_theta3, accel_theta3
    ])
    derivatives = np.nan_to_num(derivatives, nan=0.0, posinf=1e10, neginf=-1e10)
    
    return derivatives

def getPendulumDerivsFunc(numPendulums):
    if numPendulums == 1: return single3DPendulumDerivs
    elif numPendulums == 2: return double3DPendulumDerivs
    elif numPendulums == 3: return triple3DPendulumDerivs
    else: raise ValueError("error, format : {}".format(numPendulums))
