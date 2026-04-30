"""pytest configuration and fixtures for acmzzz test."""

import pytest


@pytest.fixture
def motor_config():
    """default motor configuration for testing."""
    return {
        'npp': 4,
        'R': 1.1,
        'Ld': 5e-3,
        'Lq': 6e-3,
        'KE': 0.095,        # 永磁体磁链 [Wb]
        'Js': 0.0006168,    # 转动惯量 [kg.m^2]
    }

@pytest.fixture
def controller_config():
    """default controller configureation for testing."""
    return {
        'CL_TS': 1e-4,      # 电流环采样周期 [s]
        'VL_TS': 5e-4,      # 速度环采样周期 [s]
        'VL_ceiling': 5,    # 速度环降采样倍数
    }

@pytest.fixture
def foc_config():
    """default FOC configuration for testing."""
    return {
        'CL_TS': 1e-4,
        'VL_TS': 5e-4,
        'VL_ceiling': 5,
        'speed_Kp': 0.038,
        'speed_Ki': 30.5,
        'speed_limit': 10.0,
        'current_Kp': 6.4,
        'current_Ki': 1523.2,
        'current_limit': 300.0,
    }