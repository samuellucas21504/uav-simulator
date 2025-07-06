import numpy as np
import support_files_drone as sfd

class MpcController:
    def __init__(self, U2, U3, U4):
        self.support = sfd.SupportFilesDrone()
        c = self.support.constants

        self.Ts  = c['Ts']
        self.hz  = c['hz']
        self.ct  = c['ct']; self.cq = c['cq']; self.l = c['l']

        self.U2, self.U3, self.U4 = U2, U3, U4

        self.k = 0

    def control(self, states, omega_total, refSignals, lpv_cont_discrete):
        controlled_states=self.support.constants['controlled_states'] # number of outputs
        hz = self.support.constants['hz']  # horizon period
        k = 0  # for reading reference signals

        # Generate the discrete state space matrices
        Ad, Bd, Cd, Dd, x_dot, y_dot, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot = lpv_cont_discrete
        # Generate the augmented current state and the reference vector
        x_aug_t = np.transpose(
            [np.concatenate(([phi, phi_dot, theta, theta_dot, psi, psi_dot], [self.U2, self.U3, self.U4]), axis=0)])
        # Ts=0.1 s
        # From the refSignals vector, only extract the reference values from your [current sample (NOW) + Ts] to [NOW+horizon period (hz)]
        # Example: t_now is 3 seconds, hz = 15 samples, so from refSignals vectors, you move the elements to vector r:
        # r=[Phi_ref_3.1, Theta_ref_3.1, Psi_ref_3.1, Phi_ref_3.2, ... , Phi_ref_4.5, Theta_ref_4.5, Psi_ref_4.5]
        # With each loop, it all shifts by 0.1 second because Ts=0.1 s
        k = k + controlled_states
        if k + controlled_states * hz <= len(refSignals):
            r = refSignals[k:k + controlled_states * hz]
        else:
            r = refSignals[k:len(refSignals)]
            hz = hz - 1

        # Generate the compact simplification matrices for the cost function
        Hdb, Fdbt, Cdb, Adc = self.support.mpc_simplification(Ad, Bd, Cd, Dd, hz)
        ft = np.matmul(np.concatenate((np.transpose(x_aug_t)[0][0:len(x_aug_t)], r), axis=0), Fdbt)

        du = -np.matmul(np.linalg.inv(Hdb), np.transpose([ft]))

        # Update the real inputs
        self.U2 = self.U2 + du[0][0]
        self.U3 = self.U3 + du[1][0]
        self.U4 = self.U4 + du[2][0]

        return float(self.U2), float(self.U3), float(self.U4), velocityXYZ_total
