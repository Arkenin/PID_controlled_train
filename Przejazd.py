# -*- coding: utf-8 -*-
"""
Created on Wed Feb 24 14:31:37 2021

@author: arkni
"""
from PID import PID


#%% Klasa pociągu. Pociąg z klasą

class Train():
    
    def __init__(self):
        self.mass = 1210000
        self.axles_no = 8
        self.cars_no = 11
        self.a = 0
        self.aBefore = 0
        self.v = 30
        self.s = 0
        self.dt = 0.1
        self.F = 0
        self.Fmot = 0
        self.FmotBefore = 0
        self.Fdyn = 0
        self.Fstat = 0
        self.timeCurrent = 0
        self.timeBefore = 0
        self.pid_output = 0 #from 0 to 100. about 50 is no braking neither no speeding up

        
        
    def __str__(self):
        '''zaprezentowanie parametrów pociagu'''
        return "s: {:.3f} v:{:.3f} a:{:.3f} dt:{:.3f}".format(self.s, self.v, self.a, self.dt)

        
    def iteration(self, timeCurrent):
        self.timeCurrent = timeCurrent
        if self.timeCurrent <= self.timeBefore:
            print("Nothing happend")
            return 0
        self.dt = self.timeCurrent - self.timeBefore
        self.control()
        self.force_sum()
        self.acceleration()
        self.speed_change()
        self.distance_change()
        
        

        
        
            
        self.timeBefore = self.timeCurrent
        return 1
        
    
    def resist_dyn(self):
        self.Fdyn = ((6.5+(1.5*(self.v*0.36)))*self.mass/1000)+(150*self.axles_no) + 10*(2.7+self.cars_no)*((self.v*0.36)**2)
        return self.Fdyn
        pass
    
    def resist_stat(self):
        return 0
        pass
    
    def force_sum(self):
        self.opory = self.resist_dyn() + self.resist_stat()
        if self.opory > self.Fmot and self.Fmot > 0:
            self.opory = self.Fmot
        self.F = self.Fmot - self.opory
        return self.F
    
    def acceleration(self):
        self.aBefore = self.a
        self.a = self.F / self.mass
        return self.a
    
    def speed_change(self):
        self.v = self.v + self.a * self.dt
        if self.v < 0:
            self.v = 0
        return self.v
    
    def distance_change(self):
        self.s = self.s +(self.v*self.dt)+((self.a*(self.dt)**2)/2)
        return self.s
        pass
    
    def control(self):
        #900kN maks moc
        self.FmotBefore = self.Fmot
        if self.pid_output > 100:
            self.pid_output = 100
        elif self.pid_output < -100:
            self.pid_output = -100
        
        
        self.Fmot = self.pid_output * 9000 # 9000N/100 * (-100,100)%
        while self.Fmot - self.FmotBefore > 70000:
            self.Fmot -= 70000
        while self.Fmot - self.FmotBefore < -70000:
            self.Fmot += 70000
        
    def jerk_calc(self):
        return (self.a - self.aBefore)/self.dt
        
        
    
#%% Obliczenia

reg = ""
emu = Train()
print(emu)
print("Opór dynamiczny: {:.2f}".format(emu.resist_dyn()))
tv = []
sv = []
vv = []

pid = PID(P = 50, I=0.0, D=0.0, current_time=0)
pid_poz = PID(P = .0005, I=0.0, D=0.0, current_time=0)

pid.SetPoint=20.0
pid.setSampleTime(0.1)
pid.setWindup(20.0)

next_stop = 10000.0
pid_poz.SetPoint = next_stop
pid_poz.setSampleTime(0.1)
pid_poz.setWindup(20.0)

for i in range(10000):
    t = i/10
    if i > 800:
        pid.SetPoint=60.0
        
    pid.update(emu.v, current_time=t)
    pid_poz.update(emu.s, current_time=t)

    if next_stop - emu.s > 3000 or next_stop + 4000 < emu.s :
        emu.pid_output = pid.output - 50
        reg = "spe"
    else:
        emu.pid_output = pid_poz.output - 50
        reg = "pos"
    
    
    emu.iteration(t)
    tv.append(t)
    sv.append(emu.s)
    vv.append(emu.v)

    
    print(emu, "t: {}, out: {:.2f} sil: {:.1f}, {}".format(t, emu.pid_output, emu.Fmot/1000,reg))
    
    if emu.jerk_calc()>0.5:
        print(emu, "t: {}, j: {:.2f} sil: {:.1f}, {}".format(t, emu.jerk_calc(), emu.Fmot/1000, reg))
    

import matplotlib
import matplotlib.pyplot as plt

# Data for plotting


fig, ax = plt.subplots()
ax.plot(tv, vv)

ax.set(xlabel='time (s)', ylabel='distance (m)',
       title='Przejazd teoretyczny')
ax.grid()

plt.show()








