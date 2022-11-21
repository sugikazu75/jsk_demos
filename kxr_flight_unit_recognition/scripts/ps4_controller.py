#!/usr/bin/env python

class PS4ControllerButton:
    def __init__(self):
        self.state = 0
        self.prev_state = 0

    def __call__(self, state):
        self.update(state)

    def update(self, state):
        self.prev_state = self.state
        self.state = state

    def wasPressed(self):
        if self.prev_state == 0 and self.state != 0:
            return True
        else:
            return False

    def wasReleased(self):
        if self.prev_state != 0 and self.state == 0:
            return True
        else:
            return False

    def isPressed(self):
        if self.state != 0:
            return True
        else:
            return False

class PS4Controller:
    def __init__(self):
        self.button_square = PS4ControllerButton()
        self.button_cross = PS4ControllerButton()
        self.button_circle = PS4ControllerButton()
        self.button_triangle = PS4ControllerButton()
        self.button_l1 = PS4ControllerButton()
        self.button_r1 = PS4ControllerButton()
        self.button_l2 = PS4ControllerButton()
        self.button_r2 = PS4ControllerButton()
        self.button_share = PS4ControllerButton()
        self.button_options = PS4ControllerButton()
        self.button_l3 = PS4ControllerButton()
        self.button_r3 = PS4ControllerButton()
        self.button_ps4 = PS4ControllerButton()
        self.button_tpad = PS4ControllerButton()

        self.axis_l_h = PS4ControllerButton()
        self.axis_l_v = PS4ControllerButton()
        self.axis_r_h = PS4ControllerButton()
        self.axis_l2 = PS4ControllerButton()
        self.axis_r2 = PS4ControllerButton()
        self.axis_r_v = PS4ControllerButton()

        self.button_up = PS4ControllerButton()
        self.button_down = PS4ControllerButton()
        self.button_left = PS4ControllerButton()
        self.button_right = PS4ControllerButton()

