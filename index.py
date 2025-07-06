from MAIN_LPV_MPC_drone import DroneLPV

from controllers.pid_controller import PidController
from controllers.mpc_controller import MpcController

if __name__ == "__main__":
    lpv = DroneLPV()
    lpv.simulate(PidController())