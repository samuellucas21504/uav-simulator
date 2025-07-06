from MAIN_LPV_MPC_drone import DroneLPV
from utils.mpl_backend import safe_set_backend
safe_set_backend()

if __name__ == "__main__":
    lpv = DroneLPV()
    lpv.simulate('pid')