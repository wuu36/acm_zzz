
from dataclasses import dataclass

@dataclass
class PIRegulator:
    # Gains
    Kp: float
    Ki: float
    OutLimit: float

    # State variables (initialized to zero)
    Ref: float = 0.0
    Fbk: float = 0.0
    Err: float = 0.0
    ErrPrev: float = 0.0
    Out: float = 0.0
    OutPrev: float = 0.0

    def execute(self):
        # calculate error
        self.Err = self.Ref - self.Fbk

        # incremental PI calculation
        self.Out = self.OutPrev + self.Kp * (self.Err - self.ErrPrev) + self.Ki * self.Err

        # output limiting (anti-windup)
        if self.Out > self.OutLimit:
            self.Out = self.OutLimit
        elif self.Out < -self.OutLimit:
            self.Out = -self.OutLimit

        # store for next iteration
        self.ErrPrev = self.Err
        self.OutPrev = self.Out
    
    def reset(self):
        self.Err = 0.0
        self.ErrPrev = 0.0
        self.Out = 0.0
        self.OutPrev = 0.0
    
    def set_reference(self, ref: float):
        self.Ref = ref
    
    def set_feedback(self, fbk: float):
        self.Fbk = fbk
    
    def get_output(self) -> float:
        return self.Out

    def get_error(self) -> float:
        return self.Err
    
@dataclass
class PIDRegulator:
    # gains
    Kp: float
    Ki: float
    Kd: float
    tau: float  # derivative filter time constant

    # limits
    OutLimit: float
    IntLimit: float

    # sampling period
    T: float

    # state variables
    Ref: float = 0.0
    Fbk: float = 0.0
    Err: float = 0.0
    Out: float = 0.0

    # internal states
    integrator: float = 0.0
    differentiator: float = 0.0
    prev_error: float = 0.0
    prev_measurement: float = 0.0

    def execute(self):
        # Error
        self.Err = self.Ref - self.Fbk

        # proportional term
        P_term = self.Kp * self.Err

        # integral term with anti-windup
        self.integrator += self.Ki * self.Err * self.T

        # limit integrator (anti-windup)
        if self.integrator > self.IntLimit:
            self.integrator = self.IntLimit
        elif self.integrator < -self.IntLimit:
            self.integrator = -self.IntLimit

        I_term = self.integrator

        alpha = self.T / (2.0 * self.tau + self.T)
        
        D_term = self.Kd * (self.prev_measurement - self.Fbk) / self.T
        self.differentiator = alpha * self.differentiator + (1 - alpha) * D_term
        
        # total output
        self.Out = P_term + I_term + self.differentiator

        # output limiting
        if self.Out > self.OutLimit:
            self.Out = self.OutLimit
        elif self.Out < -self.OutLimit:
            self.Out = -self.OutLimit
            
        # store for next iteration
        self.prev_error = self.Err
        self.prev_measurement = self.Fbk

    def reset(self):
        self.Err = 0.0
        self.Out = 0.0
        self.integrator = 0.0
        self.differentiator = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0

def incremental_pi(reg: PIRegulator):
    """Execute incremental PI on a regulator (standalone function).

    This is a functional version of PIRegulator.execute(),
    matching the original code's numba-compatible pattern.

    Args:
        reg: PIRegulator instance to execute
    """
    reg.Err = reg.Ref - reg.Fbk
    reg.Out = reg.OutPrev + reg.Kp * (reg.Err - reg.ErrPrev) + reg.Ki + reg.Err
    
    # output limiting
    if reg.Out > reg.OutLimit:
        reg.Out = reg.OutLimit
    elif reg.Out < -reg.OutLimit:
        reg.Out = -reg.OutLimit

    # store previous values
    reg.ErrPrev = reg.Err
    reg.OutPrev = reg.Out