"""tests for coordinate transformations."""

import pytest
import numpy as np
from acmzzz.core.transforms import (
    clarke_transform,
    clarke_transform_array,
    park_transform,
    park_transform_array,
    inverse_park,
    inverse_park_array,
    abc_to_dq,
    dq_to_abc,
)

def test_clarke_transform_balanced():
    """test clarke transform with balanced three-phase."""
    # balanced currents : 1A, -0.5A, -0.5A
    ialpha, ibeta = clarke_transform(1.0, -0.5, -0.5)

    # iA + iB + iC = 0 means sum is balanced
    # ialpha should equal iA = 1.0
    assert pytest.approx(ialpha, rel=1e-6) == 1.0
    # ibeta = (iA + 2*iB) / sqrt(3) = (1 - 1) / sqrt(3) = 0
    assert pytest.approx(ibeta, rel=1e-6) == 0.0

def test_clarke_transform_phase_sequence():
    """test clarke transform with 60A amplitude, 120 phase sequence."""
    # typical three-phase with 60A amplitude
    iA = 60.0 * np.cos(0)
    iB = 60.0 * np.cos(-2 * np.pi / 3)  # -120
    iC = 60.0 * np.cos(2 * np.pi / 3)   # 120
    
    ialpha, ibeta = clarke_transform(iA, iB, iC)

    # result should be vector of amplitude 60
    magnitude = np.sqrt(ialpha**2 + ibeta**2)
    assert pytest.approx(magnitude, rel=1e-6) == 60.0

def test_clarke_transform_array():
    """test clarke transform with array input."""
    iABC = np.array([1.0, -0.5, -0.5])
    iab = clarke_transform_array(iABC)

    assert iab.shape == (2,)
    assert pytest.approx(iab[0], rel=1e-6) == 1.0
    assert pytest.approx(iab[1], rel=1e-6) == 0.0

def test_park_transform_zero_angle():
    """test park transform with zero angle (dq aligns with ab)."""
    iD, iQ = park_transform(1.0, 0.0, theta=0.0)
    
    # with theta = 0: cos(0)=1, sin(0)=0
    # iD = ia * 1 + ib * 0 = 1.0
    # iQ = -ia * 0 + ib * 1 = 0.0
    assert pytest.approx(iD, rel=1e-6) == 1.0
    assert pytest.approx(iQ, rel=1e-6) == 0.0

def test_park_transform_90_degree():
    """test park transform with 90 degree."""
    iD, iQ = park_transform(1.0, 0.0, theta=np.pi / 2)
    
    # with theta=pi/2: cos=0, sin=1
    # iD = ia * 0 + ib * 1 = 0.0
    # iQ = -ia * 1 + ib * 0 = -1.0
    assert pytest.approx(iD, rel=1e-6) == 0.0
    assert pytest.approx(iQ, rel=1e-6) == -1.0

def test_park_transform_array():
    """test park transform with array input."""
    iab = np.array([1.0, 0.0])
    idq = park_transform_array(iab, theta=0.0)

    assert idq.shape == (2,)
    assert pytest.approx(idq[0], rel=1e-6) == 1.0
    assert pytest.approx(idq[1], rel=1e-6) == 0.0

def test_inverse_park():
    """test inverse park transform."""
    ualpha, ubeta = inverse_park(100.0, 50.0, theta=0.0)
    
    # with theta=0: cos=1, sin=0
    # ua = uD * 1 - uQ * 0 = 100
    # ub = uD * 0 + uQ * 1 = 50
    assert pytest.approx(ualpha, rel=1e-6) == 100.0
    assert pytest.approx(ubeta, rel=1e-6) == 50.0

def test_inverse_park_90_degree():
    """test inverse park transform with 90 angle."""
    ualpa, ubeta = inverse_park(100.0, 50.0, theta=np.pi / 2)

    assert pytest.approx(ualpa, rel=1e-6) == -50.0
    assert pytest.approx(ubeta, rel=1e-6) == 100.0

def test_inverse_park_array():
    """test inverse park transform with array input."""
    udq = np.array([100.0, 50.0])
    uab = inverse_park_array(udq, theta=0.0)
    
    assert uab.shape == (2,)
    assert pytest.approx(uab[0], rel=1e-6) == 100.0
    assert pytest.approx(uab[1], rel=1e-6) == 50.0

def test_park_inverse_roundtrip():
    """test park and inverse park are inverses."""
    ialpha = 50.0
    ibeta = 30.0
    theta = 0.8     # arbitrary angle

    # forward transform
    iD, iQ = park_transform(ialpha, ibeta, theta)

    # backward transform
    ualpha, ubeta = inverse_park(iD, iQ, theta)

    # should recover original values
    assert pytest.approx(ualpha, rel=1e-6) == ialpha
    assert pytest.approx(ubeta, rel=1e-6) == ibeta

def test_clarke_park_roundtrip():
    """test clarke + park + inverse park + inverse clarke roundtrip."""
    iA = 10.0
    iB = -5.0
    iC = -5.0
    theta = 0.5

    # forward: ABC -> dq
    iD, iQ = abc_to_dq(iA, iB, iC, theta)

    # backward: dq -> ABC
    uA, uB, uC = dq_to_abc(iD, iQ, theta)

    # should recover original values (approximately due to amplitude invariant)
    assert pytest.approx(uA, rel=1e-6) == iA
    assert pytest.approx(uB, rel=1e-6) == iB
    assert pytest.approx(uC, rel=1e-6) == iC

def test_abc_to_dq():
    """test combined ABC to dq transform."""
    iA, iB, iC = 1.0, -0.5, -0.5
    theta = 0.0

    iD, iQ = abc_to_dq(iA, iB, iC, theta)

    # with theta=0, dq aligns with ab, and clarke gives (1, 0)
    assert pytest.approx(iD, rel=1e-6) == 1.0
    assert pytest.approx(iQ, rel=1e-6) == 0.0

def test_dq_to_abc_sum():
    """test that dq_to_abc produces balanced three-phase (sum = 0)."""
    uD, uQ = 100.0, 50.0
    theta = 0.5

    uA, uB, uC = dq_to_abc(uD, uQ, theta)

    # balanced three-phase should sum to zero
    assert pytest.approx(uA + uB + uC, rel=1e-6) == 0.0

def test_magnitude_preservation():
    """test that transforms preserve vector magnitude."""
    # three-phase with known amplitude
    amplitude = 100.0
    phase = np.pi / 4

    iA = amplitude * np.cos(phase)
    iB = amplitude * np.cos(phase - 2 * np.pi / 3)
    iC = amplitude * np.cos(phase + 2 * np.pi / 3)

    # clarke transform
    ialpha, ibeta = clarke_transform(iA, iB, iC)
    mag_ab = np.sqrt(ialpha**2 + ibeta**2)

    # park transform (any theta)
    theta = 1.234
    iD, iQ = park_transform(ialpha, ibeta, theta)
    mag_dq = np.sqrt(iD**2 + iQ**2)

    # all magnitudes should be equal to original amplitude
    assert pytest.approx(mag_ab, rel=1e-6) == amplitude
    assert pytest.approx(mag_dq, rel=1e-6) == amplitude
                         
