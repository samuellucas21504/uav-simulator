import sys

import matplotlib


def safe_set_backend():
    if sys.platform == "darwin":
        current = matplotlib.get_backend()
        if current != "MacOSX":
            try:
                matplotlib.use("MacOSX")
                print("[mpl] backend trocado para MacOSX")
            except Exception as err:
                print(f"[mpl] 'MacOSX' indisponível, mantendo '{current}': {err}")
    else:
        pass