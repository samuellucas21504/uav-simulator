'''
LICENSE AGREEMENT

In relation to this Python file:

1. Copyright of this Python file is owned by the author: Mark Misin
2. This Python code can be freely used and distributed
3. The copyright label in this Python file such as

copyright=ax_main.text(x,y,'© Mark Misin Engineering',size=z)
that indicate that the Copyright is owned by Mark Misin MUST NOT be removed.

WARRANTY DISCLAIMER!

This Python file comes with absolutely NO WARRANTY! In no event can the author
of this Python file be held responsible for whatever happens in relation to this Python file.
For example, if there is a bug in the code and because of that a project, invention,
or anything else it was used for fails - the author is NOT RESPONSIBLE!

'''
import platform

from controllers.mpc_controller import MpcController
from controllers.pid_controller import PidController
from view.DroneAnimator import DroneAnimator

print("Python " + platform.python_version())
import numpy as np
print("Numpy " + np.__version__)
import matplotlib
print("Matplotlib " + matplotlib.__version__)
import support_files_drone as sfd

matplotlib.use("MacOSX")

class DroneLPV:
    def simulate(self, controller):
        # Create an object for the support functions.
        support=sfd.SupportFilesDrone()
        constants=support.constants

        # Load the constant values needed in the main file
        Ts=constants['Ts']
        controlled_states=constants['controlled_states'] # number of outputs
        innerDyn_length=constants['innerDyn_length'] # number of inner control loop iterations

        # Generate the refence signals
        t=np.arange(0,100+Ts*innerDyn_length,Ts*innerDyn_length) # time from 0 to 100 seconds, sample time (Ts=0.4 second)
        X_ref,X_dot_ref,X_dot_dot_ref,Y_ref,Y_dot_ref,Y_dot_dot_ref,Z_ref,Z_dot_ref,Z_dot_dot_ref,psi_ref=support.trajectory_generator(t)
        plotl=len(t) # Number of outer control loop iterations

        # Load the initial state vector
        ut=0
        vt=0
        wt=0
        pt=0
        qt=0
        rt=0
        xt=0
        yt=-1
        zt=0
        phit=0
        thetat=0
        psit=psi_ref[0]

        states=np.array([ut,vt,wt,pt,qt,rt,xt,yt,zt,phit,thetat,psit])
        statesTotal=[states] # It will keep track of all your states during the entire manoeuvre
        # statesTotal2=[states]
        statesTotal_ani=[states[6:len(states)]]
        # Assume that first Phi_ref, Theta_ref, Psi_ref are equal to the first phit, thetat, psit
        ref_angles_total=np.array([[phit,thetat,psit]])

        velocityXYZ_total=np.array([[0,0,0]])

        # Initial drone propeller states
        omega1=110*np.pi/3 # rad/s at t=-Ts s (Ts seconds before NOW)
        omega2=90*np.pi/3 # rad/s at t=-Ts s (Ts seconds before NOW)
        omega3=110*np.pi/3 # rad/s at t=-Ts s (Ts seconds before NOW)
        omega4=90*np.pi/3 # rad/s at t=-Ts s (Ts seconds before NOW)
        omega_total=omega1-omega2+omega3-omega4

        ct=constants['ct']
        cq=constants['cq']
        l=constants['l']

        U1=ct*(omega1**2+omega2**2+omega3**2+omega4**2) # Input at t = -Ts s
        U2=ct*l*(omega2**2-omega4**2) # Input at t = -Ts s
        U3=ct*l*(omega3**2-omega1**2) # Input at t = -Ts s
        U4=cq*(-omega1**2+omega2**2-omega3**2+omega4**2) # Input at t = -Ts s
        UTotal=np.array([[U1,U2,U3,U4]]) # 4 inputs
        omegas_bundle=np.array([[omega1,omega2,omega3,omega4]])
        UTotal_ani=UTotal

        ########## Start the global controller #################################

        for i_global in range(0,plotl-1):
            # Implement the position controller (state feedback linearization)
            phi_ref, theta_ref, U1=support.pos_controller(X_ref[i_global+1],X_dot_ref[i_global+1],X_dot_dot_ref[i_global+1],Y_ref[i_global+1],Y_dot_ref[i_global+1],Y_dot_dot_ref[i_global+1],Z_ref[i_global+1],Z_dot_ref[i_global+1],Z_dot_dot_ref[i_global+1],psi_ref[i_global+1],states)
            Phi_ref=np.transpose([phi_ref*np.ones(innerDyn_length+1)])
            Theta_ref=np.transpose([theta_ref*np.ones(innerDyn_length+1)])

            # Make Psi_ref increase continuosly in a linear fashion per outer loop
            Psi_ref=np.transpose([np.zeros(innerDyn_length+1)])
            for yaw_step in range(0, innerDyn_length+1):
                Psi_ref[yaw_step]=psi_ref[i_global]+(psi_ref[i_global+1]-psi_ref[i_global])/(Ts*innerDyn_length)*Ts*yaw_step

            temp_angles=np.concatenate((Phi_ref[1:len(Phi_ref)],Theta_ref[1:len(Theta_ref)],Psi_ref[1:len(Psi_ref)]),axis=1)
            ref_angles_total=np.concatenate((ref_angles_total,temp_angles),axis=0)
            # Create a reference vector
            refSignals=np.zeros(len(Phi_ref)*controlled_states)

            # Build up the reference signal vector:
            # refSignal = [Phi_ref_0, Theta_ref_0, Psi_ref_0, Phi_ref_1, Theta_ref_2, Psi_ref_2, ... etc.]
            k=0
            for i in range(0,len(refSignals),controlled_states):
                refSignals[i]=Phi_ref[k][0]
                refSignals[i+1]=Theta_ref[k][0]
                refSignals[i+2]=Psi_ref[k][0]
                k=k+1

            # Initiate the controller - simulation loops
            # statesTotal2=np.concatenate((statesTotal2,[states]),axis=0)
            for i in range(0,innerDyn_length):
                lpv_cont_discrete = support.LPV_cont_discrete( states, omega_total )
                Ad, Bd, Cd, Dd, x_dot, y_dot, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot = lpv_cont_discrete
                x_dot = np.transpose([x_dot])
                y_dot = np.transpose([y_dot])
                z_dot = np.transpose([z_dot])
                temp_velocityXYZ = np.concatenate(([[x_dot], [y_dot], [z_dot]]), axis=1)
                velocityXYZ_total = np.concatenate((velocityXYZ_total, temp_velocityXYZ), axis=0)

                if controller == 'pid':
                    U2, U3, U4 = PidController().control(
                        states,
                        refSignals,
                        Ts
                    )
                elif controller == 'mpc':
                    U2, U3, U4 = MpcController(U2, U3, U4).control(states, omega_total, refSignals, lpv_cont_discrete)
                else:
                    raise NotImplementedError('Controlador não implementado')

                # Keep track of your inputs
                UTotal=np.concatenate((UTotal,np.array([[U1,U2,U3,U4]])),axis=0)
                # print(UTotal)

                # Compute the new omegas based on the new U-s
                U1C=U1/ct
                U2C=U2/(ct*l)
                U3C=U3/(ct*l)
                U4C=U4/cq

                UC_vector=np.zeros((4,1))
                UC_vector[0,0]=U1C
                UC_vector[1,0]=U2C
                UC_vector[2,0]=U3C
                UC_vector[3,0]=U4C

                omega_Matrix=np.zeros((4,4))
                omega_Matrix[0,0]=1
                omega_Matrix[0,1]=1
                omega_Matrix[0,2]=1
                omega_Matrix[0,3]=1
                omega_Matrix[1,1]=1
                omega_Matrix[1,3]=-1
                omega_Matrix[2,0]=-1
                omega_Matrix[2,2]=1
                omega_Matrix[3,0]=-1
                omega_Matrix[3,1]=1
                omega_Matrix[3,2]=-1
                omega_Matrix[3,3]=1

                omega_Matrix_inverse=np.linalg.inv(omega_Matrix)
                omegas_vector=np.matmul(omega_Matrix_inverse,UC_vector)

                omega1P2=omegas_vector[0,0]
                omega2P2=omegas_vector[1,0]
                omega3P2=omegas_vector[2,0]
                omega4P2=omegas_vector[3,0]

                if omega1P2<=0 or omega2P2<=0 or omega3P2<=0 or omega4P2<=0:
                    print("You can't take a square root of a negative number")
                    print("The problem might be that the trajectory is too chaotic or it might have large discontinuous jumps")
                    print("Try to make a smoother trajectory without discontinuous jumps")
                    print("Other possible causes might be values for variables such as Ts, hz, innerDyn_length, px, py, pz")
                    print("If problems occur, please download the files again, use the default settings and try to change values one by one.")
                    exit()
                else:
                    omega1=np.sqrt(omega1P2)
                    omega2=np.sqrt(omega2P2)
                    omega3=np.sqrt(omega3P2)
                    omega4=np.sqrt(omega4P2)

                omegas_bundle=np.concatenate((omegas_bundle,np.array([[omega1,omega2,omega3,omega4]])),axis=0)

                # Compute the new total omega
                omega_total=omega1-omega2+omega3-omega4
                # Compute new states in the open loop system (interval: Ts/10)
                states,states_ani,U_ani=support.open_loop_new_states(states,omega_total,U1,U2,U3,U4)
                # print(states)
                # print(statesTotal)
                statesTotal=np.concatenate((statesTotal,[states]),axis=0)
                # print(statesTotal)
                statesTotal_ani=np.concatenate((statesTotal_ani,states_ani),axis=0)
                UTotal_ani=np.concatenate((UTotal_ani,U_ani),axis=0)

        # === Animação ===

        anim = DroneAnimator(
            X_ref,
            X_dot_ref,
            Y_ref,
            Y_dot_ref,
            Z_ref,
            Z_dot_ref,
            t
        )

        anim.play(statesTotal_ani, UTotal_ani, interval=5)
        anim.plot_summary(
            t=t,
            innerDyn_length=innerDyn_length,
            ref_angles_total=ref_angles_total,
            UTotal=UTotal,
            omegas_bundle=omegas_bundle,
            velocityXYZ_total=velocityXYZ_total,
            statesTotal_ani=statesTotal_ani,
        )
