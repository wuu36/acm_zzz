"""Test basic imports for acmsimpy package"""

import pytest

def test_import_acmzzz():
    """test that acmzzz package can be imported."""
    import acmzzz
    assert acmzzz is not None

def test_acmzzz_version():
    """test that acmzzz has a version attribute."""
    import acmzzz
    assert hasattr(acmzzz, '__version__')
    assert acmzzz.__version__ == "0.1.0"

def test_import_numpy():
    """test that numpy is available."""
    import numpy as np
    assert np.__version__ is not None

def test_import_matplotlib():
    """test that matplotlib is available."""
    import matplotlib
    assert matplotlib.__version__ is not None

def test_motor_config_fixture(motor_config):
    """test that motor_config fixture works."""
    assert motor_config['npp'] == 4
    assert motor_config['R'] == 1.1
    assert 'Js' in motor_config

def test_controller_config_fixture(controller_config):
    """test that controller_config fixture works."""
    assert controller_config['CL_TS'] == 1e-4
    assert controller_config['VL_TS'] == 5e-4