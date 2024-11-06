import numpy as np

def wrapToPi(angle):
    # type: (float) -> float
    """
    Converte l'angolo di input nell'intervallo [-pi, pi].
    Parametri:
        angle (float): L'angolo in radianti da convertire.
    Ritorna:
        float: L'angolo convertito nell'intervallo [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi