import numpy as np

def clarke_transform(iA: float, iB: float, iC: float) -> tuple[float, float]:
    ialpha = iA
    ibeta = (iA + 2.0 * iB) / np.sqrt(3.0)
    return ialpha, ibeta

def clarke_transform_array(iABC: np.ndarray) -> np.ndarray:
    if iABC.ndim == 1:
        iA, iB, iC = iABC[0], iABC[1], iABC[2]
        ialpha, ibeta = clarke_transform(iA, iB, iC)
        return np.array([ialpha, ibeta])
    else:
        # batch transformation
        iA = iABC[:, 0]
        iB = iABC[:, 1]
        iC = iABC[:, 2]
        ialpha = iA
        ibeta = (iA + 2.0 * iB) / np.sqrt(3.0)
        return np.column_stack([ialpha, ibeta])
    
def park_transform(ialpha: float, ibeta: float, theta: float) -> tuple[float, float]:
    cosT = np.cos(theta)
    sinT = np.sin(theta)
    iD = ialpha * cosT + ibeta * sinT
    iQ = -ialpha * sinT + ibeta * cosT
    return iD, iQ

def park_transform_array(iab: np.ndarray, theta: float) -> np.ndarray:
    cosT = np.cos(theta)
    sinT = np.sin(theta)
    
    if iab.ndim == 1:
        ialpha, ibeta = iab[0], iab[1]
        iD = ialpha * cosT + ibeta * sinT
        iQ = -ialpha * sinT + ibeta * cosT
        return np.array([iD, iQ])
    else:
        ialpha = iab[:, 0]
        ibeta = iab[:, 1]
        iD = ialpha * cosT + ibeta * sinT
        iQ = -ialpha * sinT + ibeta * cosT
        return np.column_stack([iD, iQ])

def inverse_park(uD: float, uQ: float, theta: float) -> tuple[float, float]:
    cosT = np.cos(theta)
    sinT = np.sin(theta)
    ualpha = uD * cosT - uQ * sinT
    ubeta = uD * sinT + uQ * cosT
    return ualpha, ubeta

def inverse_park_array(udq: np.ndarray, theta: float) -> np.ndarray:
    cosT = np.cos(theta)
    sinT = np.sin(theta)
    
    if udq.ndim == 1:
        uD, uQ = udq[0], udq[1]
        ualpha = uD * cosT - uQ * sinT
        ubeta = uD * sinT + uQ * cosT
        return np.array([ualpha, ubeta])
    else:
        uD = udq[:, 0]
        uQ = udq[:, 1]
        ualpha = uD * cosT - uQ * sinT
        ubeta = uD * sinT + uQ * cosT
        return np.column_stack([ualpha, ubeta])
    
def abc_to_dq(iA: float, iB: float, iC: float, theta: float) -> tuple[float, float]:
    ialpha, ibeta = clarke_transform(iA, iB, iC)
    iD, iQ = park_transform(ialpha, ibeta, theta)
    return iD, iQ

def dq_to_abc(uD: float, uQ: float, theta: float) -> tuple[float, float, float]:
    ualpha, ubeta = inverse_park(uD, uQ, theta)
    uA = ualpha
    uB = -ualpha / 2.0 + ubeta * np.sqrt(3.0) / 2.0
    uC = -ualpha / 2.0 - ubeta * np.sqrt(3.0) / 2.0
    return uA, uB, uC