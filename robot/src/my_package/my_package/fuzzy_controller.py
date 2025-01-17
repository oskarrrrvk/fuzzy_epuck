#!/usr/bin/env python

from skfuzzy import control as ctrl
import skfuzzy as fuzz
import numpy as np

class Fuzzy:
    def __init__(self):
        univers_spd=np.arange(-0.03,0.11,0.01)
        univers_dist=np.arange(0.095,0.71,0.096)

        self.current_spd = ctrl.Antecedent(univers_spd,"current_speed")
        self.dist = ctrl.Antecedent(univers_dist,"distance")
        self.ajusted_spd = ctrl.Consequent(univers_spd,"ajusted_speed")

        self.dist["close"] = fuzz.trapmf(self.dist.universe,[0.0,0.0,0.4,0.5])
        self.dist["far"] = fuzz.trapmf(self.dist.universe,[0.3,0.5,0.7,0.7])

        self.current_spd["low"] = fuzz.trapmf(self.current_spd.universe,[0.0,0.0,0.03,0.05])
        self.current_spd["fast"] = fuzz.trapmf(self.current_spd.universe,[0.04,0.07,0.1,0.1])

        self.ajusted_spd["low"] = fuzz.trapmf(self.ajusted_spd.universe,[-0.03,-0.03,0.03,0.05])
        self.ajusted_spd["fast"] = fuzz.trapmf(self.ajusted_spd.universe,[0.04,0.07,0.1,0.1])

        self.rul1 = ctrl.Rule((self.dist["close"] & self.current_spd["fast"]) | (self.dist["close"] & self.current_spd["low"]), self.ajusted_spd["low"])
        self.rul2 = ctrl.Rule((self.dist["far"] & self.current_spd["low"]) | (self.dist["far"] & self.current_spd["fast"]), self.ajusted_spd["fast"])
        
        self.control = ctrl.ControlSystem([self.rul1,self.rul2])
        self.simulation = ctrl.ControlSystemSimulation(self.control)

    def calculate_speed(self,distance, current_speed):
        self.simulation.input["distance"] = distance
        self.simulation.input["current_speed"] = current_speed
        self.simulation.compute()
        return self.simulation.output["ajusted_speed"]
