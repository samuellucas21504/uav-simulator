# drone_sim/viz/animation.py
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
import support_files_drone as sfd

class DroneAnimator:
    def __init__(
            self,
            x_ref: np.ndarray,
            x_dot_ref,
            y_ref: np.ndarray,
            y_dot_ref,
            z_ref: np.ndarray,
            z_dot_ref,
            t,
            extension: float = 0.0,
    ) -> None:
        support=sfd.SupportFilesDrone()
        constants=support.constants

        Ts=constants['Ts']
        sub_loop=constants['sub_loop']

        self.x_ref, self.x_dot_ref, self.y_ref, self.y_dot_ref, self.z_ref, self.z_dot_ref = x_ref, x_dot_ref, y_ref, y_dot_ref, z_ref, z_dot_ref

        self.t_angles=np.arange(0,t[-1]+Ts,Ts)
        self.t_ani=np.arange(0,t[-1]+Ts/sub_loop,Ts/sub_loop)

        self.extension = extension

        # ---------- layout ----------
        self.fig = plt.figure(figsize=(16, 9), dpi=120, facecolor=(0.8, 0.8, 0.8))
        self.gs = gridspec.GridSpec(4, 3)

        self._init_3d_axes()
        self._init_side_plots()
        self.anim = None  # será criado em play()

    def play(self, states_ani: np.ndarray, u_ani: np.ndarray, interval: int = 20):
        self.states = states_ani
        self.u_hist = u_ani

        self.anim = animation.FuncAnimation(
            self.fig,
            self._update,
            frames=len(states_ani),
            interval=interval,
            repeat=True,
            blit=False,
        )
        plt.show()


    def _init_3d_axes(self):
        ax0 = self.fig.add_subplot(self.gs[0:3, 0:2], projection="3d",
                                   facecolor=(0.9, 0.9, 0.9))
        ax0.set_title("© Mark Misin Engineering", fontsize=15)

        max_ref = max(self.x_ref.max(), self.y_ref.max())
        min_ref = min(self.x_ref.min(), self.y_ref.min())
        ax0.set_xlim(min_ref, max_ref)
        ax0.set_ylim(min_ref, max_ref)
        ax0.set_zlim(0, self.z_ref.max())

        ax0.set_xlabel("X [m]")
        ax0.set_ylabel("Y [m]")
        ax0.set_zlabel("Z [m]")

        self.ref_line, = ax0.plot(self.x_ref, self.y_ref, self.z_ref,
                                  "b", lw=1, label="reference")
        self.traj_line, = ax0.plot([], [], [], "r", lw=1, label="trajectory")
        self.body_x, = ax0.plot([], [], [], "r", lw=5, label="drone_x")
        self.body_y, = ax0.plot([], [], [], "g", lw=5, label="drone_y")
        ax0.legend(loc="upper left")

        self.ax_main = ax0
        self.length_x = max_ref * 0.15
        self.length_y = max_ref * 0.15

    def _init_side_plots(self):
        ax_phi = self.fig.add_subplot(self.gs[3, 0], facecolor=(0.9, 0.9, 0.9))
        ax_theta = self.fig.add_subplot(self.gs[3, 1], facecolor=(0.9, 0.9, 0.9))
        self.body_phi, = ax_phi.plot([], [], "--g", lw=2,
                                     label="drone_y (+: Z-up,Y-right,φ-CCW)")
        self.body_theta, = ax_theta.plot([], [], "--r", lw=2,
                                         label="drone_x (+: Z-up,X-left,θ-CCW)")
        ax_phi.set_xlim(-self.length_y * 0.9, self.length_y * 0.9)
        ax_theta.set_xlim(self.length_x * 0.9, -self.length_x * 0.9)
        for ax in (ax_phi, ax_theta):
            ax.set_ylim(-self.length_x * 0.011, self.length_x * 0.011)
            ax.grid(True); ax.legend(fontsize="small")

        ax_u = [self.fig.add_subplot(self.gs[i, 2], facecolor=(0.9, 0.9, 0.9))
                for i in range(4)]
        labels = ["Thrust (U1) [N]", "Roll (U2) [Nm]",
                  "Pitch (U3) [Nm]", "Yaw (U4) [Nm]"]
        self.u_lines = []
        for ax, lab in zip(ax_u, labels):
            (line,) = ax.plot([], [], "b", lw=1, label=lab)
            ax.set_xlim(0, self.t_ani[-1])
            ax.grid(True); ax.legend(fontsize="small")
            self.u_lines.append(line)
        ax_u[-1].set_xlabel("t-time [s]", fontsize=15)

    def _update(self, num: int):
        x, y, z, phi, theta, psi = self.states[num]

        R_x = np.array([[1, 0, 0],
                        [0, np.cos(phi), -np.sin(phi)],
                        [0, np.sin(phi),  np.cos(phi)]])
        R_y = np.array([[ np.cos(theta), 0, np.sin(theta)],
                        [0, 1, 0],
                        [-np.sin(theta), 0, np.cos(theta)]])
        R_z = np.array([[np.cos(psi), -np.sin(psi), 0],
                        [np.sin(psi),  np.cos(psi), 0],
                        [0, 0, 1]])
        R = R_z @ R_y @ R_x

        e_x = np.array([[self.length_x + self.extension], [0], [0]])
        e_y = np.array([[0], [self.length_y + self.extension], [0]])
        bx = R @ e_x; bx_neg = R @ -e_x
        by = R @ e_y; by_neg = R @ -e_y

        self.body_x.set_data([x + bx_neg[0, 0], x + bx[0, 0]],
                             [y + bx_neg[1, 0], y + bx[1, 0]])
        self.body_x.set_3d_properties([z + bx_neg[2, 0], z + bx[2, 0]])
        self.body_y.set_data([x + by_neg[0, 0], x + by[0, 0]],
                             [y + by_neg[1, 0], y + by[1, 0]])
        self.body_y.set_3d_properties([z + by_neg[2, 0], z + by[2, 0]])

        # -------------- trilha 3-D --------------
        self.traj_line.set_data(self.states[:num, 0], self.states[:num, 1])
        self.traj_line.set_3d_properties(self.states[:num, 2])

        self.body_phi.set_data([-self.length_y * 0.9 * 0.9,
                                self.length_y * 0.9 * 0.9],
                               [by_neg[2, 0], by[2, 0]])
        self.body_theta.set_data([self.length_x * 0.9 * 0.9,
                                  -self.length_x * 0.9 * 0.9],
                                 [bx[2, 0], bx_neg[2, 0]])

        for k, ln in enumerate(self.u_lines):
            ln.set_data(self.t_ani[:num], self.u_hist[:num, k])

        return (*self.u_lines, self.body_phi, self.body_theta,
                self.body_x, self.body_y, self.traj_line)

    def plot_summary(
            self,
            t,
            innerDyn_length,
            ref_angles_total,
            UTotal,
            omegas_bundle,
            velocityXYZ_total,
            statesTotal_ani,
    ):
        X_ref, X_dot_ref, Y_ref, Y_dot_ref, Z_ref, Z_dot_ref = self.x_ref, self.x_dot_ref, self.y_ref, self.y_dot_ref, self.z_ref, self.z_dot_ref
        t_ani = self.t_ani
        t_angles = self.t_angles

        statesTotal_x=statesTotal_ani[:,0]
        statesTotal_y=statesTotal_ani[:,1]
        statesTotal_z=statesTotal_ani[:,2]
        statesTotal_phi=statesTotal_ani[:,3]
        statesTotal_theta=statesTotal_ani[:,4]
        statesTotal_psi=statesTotal_ani[:,5]

        ax=plt.axes(projection='3d')
        ax.plot(X_ref,Y_ref,Z_ref,'b',label='reference')
        ax.plot(statesTotal_x,statesTotal_y,statesTotal_z,'r',label='trajectory')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        copyright=ax.text(0,max(Y_ref),max(Z_ref)*1.20,'© Mark Misin Engineering',size=15)
        ax.legend()
        plt.show()

        # Position and velocity plots
        plt.subplot(2,1,1)
        plt.plot(t,X_ref,'b',linewidth=1,label='X_ref')
        plt.plot(t_ani,statesTotal_x,'r',linewidth=1,label='X')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('X [m]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='center right',fontsize='small')

        plt.subplot(2,1,2)
        plt.plot(t,X_dot_ref,'b',linewidth=1,label='X_dot_ref')
        plt.plot(t,velocityXYZ_total[0:len(velocityXYZ_total):innerDyn_length,0],'r',linewidth=1,label='X_dot')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('X_dot [m/s]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='center right',fontsize='small')
        plt.show()

        plt.subplot(2,1,1)
        plt.plot(t,Y_ref,'b',linewidth=1,label='Y_ref')
        plt.plot(t_ani,statesTotal_y,'r',linewidth=1,label='Y')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('Y [m]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='center right',fontsize='small')

        plt.subplot(2,1,2)
        plt.plot(t,Y_dot_ref,'b',linewidth=1,label='Y_dot_ref')
        plt.plot(t,velocityXYZ_total[0:len(velocityXYZ_total):innerDyn_length,1],'r',linewidth=1,label='Y_dot')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('Y_dot [m/s]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='center right',fontsize='small')
        plt.show()


        plt.subplot(2,1,1)
        plt.plot(t,Z_ref,'b',linewidth=1,label='Z_ref')
        plt.plot(t_ani,statesTotal_z,'r',linewidth=1,label='Z')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('Z [m]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='center right',fontsize='small')

        plt.subplot(2,1,2)
        plt.plot(t,Z_dot_ref,'b',linewidth=1,label='Z_dot_ref')
        plt.plot(t,velocityXYZ_total[0:len(velocityXYZ_total):innerDyn_length,2],'r',linewidth=1,label='Z_dot')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('Z_dot [m/s]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='center right',fontsize='small')
        plt.show()


        # Orientation plots
        plt.subplot(3,1,1)
        plt.plot(t_angles,ref_angles_total[:,0],'b',linewidth=1,label='Phi_ref')
        plt.plot(t_ani,statesTotal_phi,'r',linewidth=1,label='Phi')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('Phi [rad]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='lower right',fontsize='small')

        plt.subplot(3,1,2)
        plt.plot(t_angles,ref_angles_total[:,1],'b',linewidth=1,label='Theta_ref')
        plt.plot(t_ani,statesTotal_theta,'r',linewidth=1,label='Theta')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('Theta [rad]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='lower right',fontsize='small')

        plt.subplot(3,1,3)
        plt.plot(t_angles,ref_angles_total[:,2],'b',linewidth=1,label='Psi_ref')
        plt.plot(t_ani,statesTotal_psi,'r',linewidth=1,label='Psi')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('Psi [rad]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='lower right',fontsize='small')
        plt.show()


        # Control input plots
        plt.subplot(4,2,1)
        plt.plot(t_angles,UTotal[0:len(UTotal),0],'b',linewidth=1,label='U1')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('U1 [N]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='upper right',fontsize='small')

        plt.subplot(4,2,3)
        plt.plot(t_angles,UTotal[0:len(UTotal),1],'b',linewidth=1,label='U2')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('U2 [Nm]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='upper right',fontsize='small')

        plt.subplot(4,2,5)
        plt.plot(t_angles,UTotal[0:len(UTotal),2],'b',linewidth=1,label='U3')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('U3 [Nm]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='upper right',fontsize='small')

        plt.subplot(4,2,7)
        plt.plot(t_angles,UTotal[0:len(UTotal),3],'b',linewidth=1,label='U4')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('U4 [Nm]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='upper right',fontsize='small')

        plt.subplot(4,2,2)
        plt.plot(t_angles,omegas_bundle[0:len(omegas_bundle),0],'b',linewidth=1,label='omega 1')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('omega 1 [rad/s]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='upper right',fontsize='small')

        plt.subplot(4,2,4)
        plt.plot(t_angles,omegas_bundle[0:len(omegas_bundle),1],'b',linewidth=1,label='omega 2')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('omega 2 [rad/s]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='upper right',fontsize='small')

        plt.subplot(4,2,6)
        plt.plot(t_angles,omegas_bundle[0:len(omegas_bundle),2],'b',linewidth=1,label='omega 3')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('omega 3 [rad/s]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='upper right',fontsize='small')

        plt.subplot(4,2,8)
        plt.plot(t_angles,omegas_bundle[0:len(omegas_bundle),3],'b',linewidth=1,label='omega 4')
        plt.xlabel('t-time [s]',fontsize=15)
        plt.ylabel('omega 4 [rad/s]',fontsize=15)
        plt.grid(True)
        plt.legend(loc='upper right',fontsize='small')
        plt.show()
