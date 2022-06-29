from cmath import nan
from typing import Tuple


class KalmanFilter1D():
    _q = nan # Process noise
    
    _z = nan # Current measurement value
    _r = nan # Measurement uncertainty
    
    _x_prev = nan # Prediction of current state from previous estimate
    _p_prev = nan # Prediction estimate uncertainty
    _k = nan # Current Kalman gain
    _x_n = nan # Current state estimate
    _p_n = nan # Current estimate uncertainty
    _x_next = nan # Next state estimate
    _p_next = nan # Next estimate uncertainty

    def __init__(self, ix, ip, q) -> None:
        self._q = q

        self._x_n = ix
        self._p_n = ip
        self.predict_next_state()

    
    # Kalman Filter Equations

    def _calc_kalman_gain(self):
        self._k = self._p_prev / (self._p_prev + self._r)


    def _calc_state_estimate(self, z):
        self._calc_kalman_gain()
        self._x_n = self._x_prev + self._k * (z-self._x_prev)


    def _calc_state_uncertainty(self):
        self._p_n = (1-self._k) * self._p_prev


    def _calc_next_state_estimate(self):
        self._x_next = self._x_n


    def _calc_next_state_uncertainty(self):
        self._p_next = self._p_n


    # Filter function wrappers

    def estimate_current_state(self, z, r) -> Tuple:
        self._x_prev = self._x_next
        self._p_prev = self._p_next
        self._r = r
        self._calc_state_estimate(z)
        self._calc_state_uncertainty()
        return (self._x_n, self._p_n)
    

    def predict_next_state(self) -> Tuple:
        self._calc_next_state_estimate()
        self._calc_next_state_uncertainty()
        return (self._x_next, self._p_next)


    # Getters and setters

    def get_current_estimate(self):
        return self._x_n


    def get_current_uncertainty(self):
        return self._p_n


    def get_next_estimate(self):
        return self._x_next


    def get_next_uncertainty(self):
        return self._p_next + self._q


    def set_process_noise(self, q):
        self._q = q
    