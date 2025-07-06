import numpy as np
import support_files_drone as sfd

class MpcController:
    def __init__(self):
        self.support = sfd.SupportFilesDrone()
        c = self.support.constants

        self.Ts  = c['Ts']
        self.hz  = c['hz']
        self.ct  = c['ct']; self.cq = c['cq']; self.l = c['l']

        self.U2 = self.U3 = self.U4 = 0.0

        self.k = 0

    def control(self, state, ref, dt):
        """
        state : vetor de 12 estados
        ref   : tupla (phi_ref, theta_ref, psi_ref)   – apenas a próxima amostra
        dt    : Ts (=0.1 s). Mantido para ficar compatível com o PID
        retorna: (U2, U3, U4) – torques [N·m] calculados via MPC
        """

        Ad, Bd, Cd, Dd, *_, phi, phi_dot, theta, theta_dot, psi, psi_dot = \
            self.support.LPV_cont_discrete(state, omega_total=0)

        x_aug_t = np.array([[phi, phi_dot, theta, theta_dot, psi, psi_dot,
                             self.U2, self.U3, self.U4]]).T

        r = np.tile(ref, self.hz)

        Hdb, Fdbt, *_ = self.support.mpc_simplification(Ad, Bd, Cd, Dd, self.hz)

        ft = np.concatenate((x_aug_t.flatten(), r)) @ Fdbt
        du = -np.linalg.inv(Hdb) @ ft.reshape(-1, 1)

        self.U2 += du[0, 0]
        self.U3 += du[1, 0]
        self.U4 += du[2, 0]

        return float(self.U2), float(self.U3), float(self.U4)
