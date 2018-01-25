"""This module is a collection of functions for working with various
motor inputs.

Written for GIT LIT, the Georgia Institute of Technology team in the
University Student Launch Intitiative (USLI) 2017-2018 competition.

Will Wills
Email: whwthree@gmail.com
Cellphone #: (678) 313-8206

Original Date:
Nov 19 2017
Last changed:
Nov 19 2017
"""
from easy_gpio import outpins, up, ups, down, downs, get, toggle, step, states
from RPi.GPIO import cleanup

class StepperMotor:

    def __init__(self, max_steps=50, min_steps=0, current_steps=0,
                 pins=(31, 33, 35, 37), delay=0.001):
        self.max_steps = max_steps
        self.min_steps = min_steps
        self.pins = pins
        outpins(pins)
        downs(pins)
        self.rest_pin, self.sleep_pin, self.step_pin, self.dir_pin = pins
        self.ready_pins = (self.rest_pin, self.sleep_pin)
        self.current_steps = current_steps
        self.delay = delay

    @staticmethod
    def cleanup():
        cleanup()

    def ready(self):
        ups(self.ready_pins)

    def shut_down(self):
        downs(self.ready_pins)

    def step(self, num_steps):
        if (not get(self.dir_pin)
                and self.min_steps <= self.current_steps - num_steps):
            # decrease actuation
            step(self.step_pin, num_steps, self.delay)
            self.current_steps -= num_steps
        elif (get(self.dir_pin)
                and self.current_steps + num_steps <= self.max_steps):
            # increase actuation
            step(self.step_pin, num_steps, self.delay)
            self.current_steps += num_steps
        else:
            raise ValueError('Operation would exceed step limits.')

    def deploy(self, num_steps):
        up(self.dir_pin)
        self.step(num_steps)

    def retract(self, num_steps):
        down(self.dir_pin)
        self.step(num_steps)

