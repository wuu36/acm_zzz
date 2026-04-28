"""test for machineparams class."""

import pytest
from acmzzz.core.machine_params import MachineParams


def test_default_params():
    """test default machineparams initialization."""
    params = MachineParams()
    assert params.npp == 4
    assert params.R == 1.1
    assert params.Ld == 5e-3
    assert params.Lq == 6e-3
    assert params.KE == 0.095
    assert params.Rreq == 0.0
    assert params.Js == 0.0006168

def test_custom_params():
    """test custom machineparams initialization."""
    params = MachineParams(
        npp=21,
        R=0.1222,
        Ld=0.0011,
        Lq=0.00125,
        KE=0.127,
        Js=0.203,
    )
    assert params.npp == 21
    assert params.R == 0.1222
    assert params.Ld == 0.0011
    assert params.Lq == 0.00125
    assert params.KE == 0.127
    assert params.Js == 0.203

def test_inverse_properties():
    """test inverse property calculations."""
    params = MachineParams(npp=4, Js=0.0006168)
    assert params.npp_inv == 0.25
    assert pytest.approx(params.Js_inv, rel=1e-6) == 1.0 / 0.0006168

def test_motor_type_detection():
    """test pmsm/im detetion.""" 
    # pmsm case
    pmsm_params = MachineParams(KE=0.095, Rreq=0.0)
    assert pmsm_params.is_pmsm is True
    assert pmsm_params.is_induction_motor is False
    
    # induction motor case
    im_params = MachineParams(KE=0.0, Rreq=1.5)
    assert im_params.is_pmsm is False
    assert im_params.is_induction_motor is True

def test_validation_negative_npp():
    """test validation fails for negative pole pairs."""
    with pytest.raises(ValueError, match="npp"):
        MachineParams(npp=-1)

def test_validation_negative_resistance():
    """test validation fails for negative resistance."""
    with pytest.raises(ValueError, match="R"):
        MachineParams(R=-1.0)

def test_validation_negative_inductance():
    """test validation fails for zero/negative inductance."""
    with pytest.raises(ValueError, match="Ld"):
        MachineParams(Ld=0.0)

    with pytest.raises(ValueError, match="Lq"):
        MachineParams(Lq=-1e-3)

def test_validation_negative_inertia():
    """test validation fails for zero/negative inertia"""
    with pytest.raises(ValueError, match="Js"):
        MachineParams(Js=0.0)

def test_fixture_params(motor_config):
    """test creating machineparams from fixture."""
    params = MachineParams(**motor_config)
    assert params.npp == motor_config['npp']
    assert params.R == motor_config['R']
    assert params.Js == motor_config['Js']
    
    