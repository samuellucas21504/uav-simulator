import numpy as np

class PidController:
    def __init__(self):
        self._int_err = np.zeros(3)

    def control(self, state, ref, dt):
        WEIGHT = 0.2
        KP = WEIGHT # Proporcional
        KI = WEIGHT / 10 # Integrativo
        KD = WEIGHT / 4 # Derivativo

        Kp = np.diag([KP, KP, KP*1.2])
        Ki = np.diag([KI, KI, KI*1.2])
        Kd = np.diag([KD, KD, KD*1.2])

        roll, pitch, yaw = state[9], state[10], state[11]
        p, q, r = state[3], state[4], state[5]

        e     = np.array([ref[0] - roll,  ref[1] - pitch,  ref[2] - yaw])
        dedt  = -np.array([p, q, r]) # derivada = –vel. angulares
        self._int_err += e * dt
        self._int_err = np.clip(self._int_err, -0.5, 0.5)

        torque = Kp @ e + Ki @ self._int_err + Kd @ dedt

        return float(torque[0]), float(torque[1]), float(torque[2])
