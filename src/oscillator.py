import numpy as np
from scipy import integrate
from numba import jit


# ODE constants
y0 = [0.2, 0, -0.2, 0, 0.2, 0, -0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0]     # Initial amplitudes
t0 = 0                                                              # Initial time
alpha = 5                                                           # Convergence speed factor
eps1 = 0.03                                                         # TC coupling strength
eps2 = 0.5                                                          # CTr Coupling strength

# ODE default parameters
CPG_period = 4                                                      # Period (seconds)
CPG_freq = 2 * np.pi / CPG_period                                   # Frequency (rad)
TC_Left_amp = 0.5                                                   # TC-joint amplitude (Left side)
TC_Right_amp = 0.5                                                  # TC-joint amplitude (Right side)
CTr_amplitude = np.square(0.85)                                     # CTr-joint amplitude
TC_phase = 180                                                      # TC-joint phase angle(deg)
CTr_phase = -90                                                     # CTr-joint phase angle(deg)

# No-variable parameters
theta1 = np.deg2rad(TC_phase)


# Hopf ODEs definition function
@jit(nopython=True)
def hopf(t, y, mu1, mu2, mu3, w, theta2):
    sin_t1 = np.sin(theta1)
    cos_t1 = np.cos(theta1)
    sin_t2 = np.sin(theta2)
    cos_t2 = np.cos(theta2)

   
    # Oscillators couplings
    tc_to_1 = y[0] * cos_t1 - y[1] * sin_t1
    tc_to_2 = y[2] * cos_t1 - y[3] * sin_t1
    tc_to_3 = y[4] * cos_t1 - y[5] * sin_t1
    tc_to_4 = y[6] * cos_t1 - y[7] * sin_t1

    ctr_to_1 = y[0] * cos_t2 - y[1] * sin_t2
    ctr_to_2 = y[2] * cos_t2 - y[3] * sin_t2
    ctr_to_3 = y[4] * cos_t2 - y[5] * sin_t2
    ctr_to_4 = y[6] * cos_t2 - y[7] * sin_t2

    # Oscillator_1 - Left_side - Front_leg & Hind_leg - TC_joint
    r1 = mu1 - y[0] ** 2 - y[1] ** 2
    dydt0 = alpha * y[0] * r1 + w * y[1] + eps1 * (tc_to_2 + tc_to_4)
    dydt1 = alpha * y[1] * r1 - w * y[0]

    # Oscillator_2 - Left_side - Middle_leg - TC_joint
    r2 = mu1 - y[2] ** 2 - y[3] ** 2
    dydt2 = alpha * y[2] * r2 + w * y[3] + eps1 * (tc_to_1 + tc_to_3)
    dydt3 = alpha * y[3] * r2 - w * y[2]

    # Oscillator_3 - Right_side - Front_leg & Hind_leg - TC_joint
    r3 = mu2 - y[4] ** 2 - y[5] ** 2
    dydt4 = alpha * y[4] * r3 + w * y[5] + eps1 * (tc_to_4 + tc_to_2)
    dydt5 = alpha * y[5] * r3 - w * y[4]

    # Oscillator_4 - Right_side - Middle_leg - TC_joint
    r4 = mu2 - y[6] ** 2 - y[7] ** 2
    dydt6 = alpha * y[6] * r4 + w * y[7] + eps1 * (tc_to_3 + tc_to_4)
    dydt7 = alpha * y[7] * r4 - w * y[6]
# --------------------------------------------------------------------------------------
    # Oscillator_5 - Left_side  - Front_leg & Hind_leg - CTr_joint
    r5 = mu3 - y[8] ** 2 - y[9] ** 2
    dydt8 = 0.85*alpha * y[8] * r5 + w * y[9] + eps2 * ctr_to_1
    dydt9 = 0.85*alpha * y[9] * r5 - w * y[8]

    # Oscillator_6 - Left_side  - Middle_leg - CTr_joint
    r6 = mu3 - y[10] ** 2 - y[11] ** 2
    dydt10 = 0.85*alpha * y[10] * r6 + w * y[11] + eps2 * ctr_to_2
    dydt11 = 0.85*alpha * y[11] * r6 - w * y[10]

    # Oscillator_7 - Right_side  - Front_leg & Hind_leg - CTr_joint
    r7 = mu3 - y[12] ** 2 - y[13] ** 2
    dydt12 = 0.85*alpha * y[12] * r7 + w * y[13] + eps2 * ctr_to_3
    dydt13 = 0.85*alpha * y[13] * r7 - w * y[12]

    # Oscillator_8 - Right_side  - Middle_leg - CTr_joint
    r8 = mu3 - y[14] ** 2 - y[15] ** 2
    dydt14 = 0.85*alpha * y[14] * r8 + w * y[15] + eps2 * ctr_to_4
    dydt15 = 0.85*alpha * y[15] * r8 - w * y[14]

    return np.array([dydt0, dydt1, dydt2, dydt3, dydt4, dydt5, dydt6, dydt7, dydt8, dydt9, dydt10, dydt11, dydt12, dydt13, dydt14, dydt15])


def wrapper(t, y, mu1, mu2, mu3, w, theta2):
    return hopf(t, y, mu1, mu2, mu3, w, theta2)



class ODE():
    def __init__(self):
        self.fun = integrate.ode(wrapper).set_integrator('vode', method='adams')
        # Raw results of hopf ODE integration
        self.raw = np.array(y0)
    
    def hopf_parameters(self, tc_left_amp, tc_right_amp, ctr_amp, cpg_freq, theta2):
        tc_left_amp = np.clip(tc_left_amp, 0.0, 0.9)
        tc_right_amp = np.clip(tc_right_amp, 0.0, 0.9)
        ctr_amp = np.clip(ctr_amp, 0.0, 0.85)
        self.fun.set_f_params(np.square(tc_left_amp), np.square(tc_right_amp), np.square(ctr_amp), cpg_freq, np.deg2rad(theta2))
        return
        
    # Set initial conditions
    def hopf_initial(self, init_amp, init_time):
        self.fun.set_initial_value(init_amp, init_time)
        return
        
    # Hopf numerical integration (compiled to C by jitcode)
    def hopf_integrate(self, t):
        y = self.fun.integrate(t)  # get one more value, add it to the array
        #self.raw = np.array([y[0], y[1], y[2], y[3], y[4], y[5], y[6], y[7], 0, 0, 0, 0, 0, 0, 0, 0])
        self.raw = y
        return y[1], y[3], y[1], y[5], y[7], y[5], y[9], y[11], y[9], y[13], y[15], y[13]
        
    def hopf_restart(self):
        self.fun = integrate.ode(wrapper).set_integrator('vode', method='adams')
        
        
# Jitcode ODE definition and integration method selection
myODE = ODE()

# Set initial conditions and default parameters
myODE.hopf_initial(y0, t0)
myODE.hopf_parameters(TC_Left_amp, TC_Right_amp, CTr_amplitude, CPG_freq, CTr_phase)
