import numpy as np

class PidController:
    # attitude pid
    def control(self, state, ref, dt):
        WEIGHT = 0.2
        KP = WEIGHT
        KI = WEIGHT / 10
        KD = WEIGHT / 4

        Kp = np.diag([KP, KP, KP * 2])
        Ki = np.diag([KI, KI, KI * 2])
        Kd = np.diag([KD, KD, KD * 2])

        _int_err = np.zeros(3)

        phi, theta, psi = state[9], state[10], state[11]
        p, q, r = state[3], state[4], state[5]

        e     = np.array([ref[0] - phi,  ref[1] - theta,  ref[2] - psi])
        dedt  = -np.array([p, q, r])
        _int_err += e * dt
        _int_err = np.clip(_int_err, -0.5, 0.5)

        torque = Kp @ e + Ki @ _int_err + Kd @ dedt

        return float(torque[0]), float(torque[1]), float(torque[2])
