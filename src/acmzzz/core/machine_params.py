from dataclasses import dataclass

@dataclass
class MachineParams:
    # name plate data
    npp: int = 4        # number of pole pairs
    IN: float = 3.0     # Rated current [A]
    
    # electrical parameters
    R: float = 1.1      # stator resistance [Ohm]
    Ld: float = 5e-3    # d-axis inductance [H]
    Lq: float = 6e-3    # q-axis inductance [H]
    KE: float = 0.095   # PM flux linkage [Wb] (PMSM), or 0 for IM
    Rreq: float = 0.0   # Rotor resistance [Ohm] (IM), or 0 for PMSM
    
    # mechanical parameters
    Js: float = 0.0006168   # total inertia [kg.m^2]
    
    def __post_init__(self):
        """validate parameters after initialization."""
        if self.npp <= 0:
            raise ValueError("npp must be positive")
        if self.R < 0:
            raise ValueError("R must be non-negative")
        if self.Ld <= 0 or self.Lq <= 0:
            raise ValueError("Ld and Lq must be positive")
        if self.Js <= 0:
            raise ValueError("Js must be positive")
        

    @property
    def npp_inv(self) -> float:
        """inverse of pole pairs (1/npp)."""
        return 1.0 / self.npp
    
    @property
    def Js_inv(self) -> float:
        """inverse of inertia (1/Js)."""
        return 1.0/ self.Js
    
    @property
    def is_pmsm(self) -> bool:
        """check if motor is PMSM(has permanent magnet flux)"""
        return self.KE > 0
    
    @property
    def is_induction_motor(self) -> bool:
        """check if motor is induction motor (has rotor resistance)."""
        return self.Rreq > 0