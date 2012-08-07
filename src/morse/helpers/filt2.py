import math

class Filt2:
   """ filtre du second ordre Yann """
   def __init__(self, wn = 1.0, ksi = 1.0):
      self.wn = wn
      self.ksi = ksi
      self.x = [0.0, 0.0, 0.0, 0.0 ]
      self.xn = [0.0, 0.0 ]

   def setParam(self,wn , ksi):
      self.wn = wn
      self.ksi = ksi

   def init():
      self.x = [0.0, 0.0, 0.0, 0.0 ]

   def simulate(self,u, dt):
      self.xn[0] = self.x[0] + (self.x[3] + self.x[1])*dt/2.0
      self.xn[1] = self.xn[1] + (self.x[2] + self.wn * self.wn * (u - self.x[0]) - 2.0 * self.ksi * self.wn * self.x[1]) * dt/2.0

      self.x[2] = self.wn * self.wn * (u - self.x[0]) - 2.0 * self.ksi * self.wn * self.x[1]
      self.x[3] = self.x[1]
      self.x[0] = self.xn[0]
      self.x[1] = self.xn[1]

