"""
This is a Python PID module heavily inspired by the Arduino PID Library
written by Brett Beauregard.
http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

Modified by Viraj Bangari to allow simulated time steps.
"""

__author__ = "Stian Sandve"
__license__ = "GPLv3"
__version__ = "1.1.1"

import time
from enum import Enum


class Direction(Enum):
    direct = 1
    reverse = 2


class Mode(Enum):
    automatic = 1
    manual = 2


class PID(object):

    def __init__(self, kp, ki, kd, setpoint, sample_time):
        """
        The parameters specified here are those for for which we can't set up
        reliable defaults, so we need to have the user set them.
        :param kp: Proportional Tuning Parameter
        :param ki: Integral Tuning Parameter
        :param kd: Derivative Tuning Parameter
        :param set_point: The value that we want the process to be.
        :param controller_direction:
        """

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.i_term = 0

        self.out_max = 0
        self.out_min = 0

        self.last_input = 0
        self.output = 0
        self.input = 0

        self.set_point = setpoint
        self.mode = Mode.automatic

        self.sample_time = sample_time
        self.controller_direction = Direction.direct
        self.set_tunings(self.kp, self.ki, self.kd)

        self.last_time = 0

    def compute(self, input, current_time):
        """
        This, as they say, is where the magic happens. This function should
        be called every time "void loop()" executes. The function will decide
        for itself whether a new PID Output needs to be computed.
        :param input: Input value for the PID controller.
        :return: Returns true when the output is computed,
        false when nothing has been done.
        """

        if self.mode is Mode.manual:
            return 0, False

        error = self.set_point - input

        self.i_term += (self.ki * error)
        delta_input = input - self.last_input
        self.output = self.kp * error + self.i_term - self.kd * delta_input
        self.last_input = input
        self.last_time = current_time
        return self.output

    def set_tunings(self, kp, ki, kd):
        """
        This function allows the controller's dynamic performance to be
        adjusted. It's called automatically from the constructor,
        but tunings can also be adjusted on the fly during normal operation.
        :param kp: Proportional Tuning Parameter
        :param ki: Integral Tuning Parameter
        :param kd: Derivative Tuning Parameter
        """

        sample_time_in_sec = self.sample_time

        self.kp = kp
        self.ki = ki * sample_time_in_sec
        self.kd = kd / sample_time_in_sec

    def set_mode(self, mode):
        """
        Allows the controller Mode to be set to manual (0) or Automatic
        (non-zero) when the transition from manual to auto occurs,
        the controller is automatically initialized.
        :param mode: The mode of the PID controller.
        Can be either manual or automatic.
        """

        if self.mode is Mode.manual and mode is Mode.automatic:
            self.initialize()

        self.mode = mode

    def initialize(self):
        """
        Does all the things that need to happen to ensure a smooth transfer
        from manual to automatic mode.
        """

        self.i_term = self.output
        self.last_input = self.input
        if self.i_term > self.out_max:
            self.i_term = self.out_max
        elif self.i_term < self.out_min:
            self.i_term = self.out_min
