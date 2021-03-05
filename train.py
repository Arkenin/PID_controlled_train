# -*- coding: utf-8 -*-
"""
Created on Wed Mar  3 15:36:05 2021

@author: arkni
Train class

"""

class Train():
    
    def __init__(self):
        self.mass = 1210000
        self.axles_no = 8
        self.cars_no = 11
        self.a = 0
        self.aBefore = 0
        self.v = 10
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
            # print("Nothing happend")
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
    
    def resist_stat(self, profile = 0):
        '''resistance from uphill,downhill'''
        return 9.81*profile*(self.mass)
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
            
        if False: #Turning off controll
            self.Fmot = 0
        
    def jerk_calc(self):
        return (self.a - self.aBefore)/self.dt
        
        
#%% Calculations
def train_ride(vect_ster, save = False, stats = []):
        
    optimu = Train()

    # print("Opór dynamiczny: {:.2f}".format(optimu.resist_dyn()))
    if save:
        tv = []
        sv = []
        vv = []
    
    for i in range(100):
        t = i/10
        
            
        optimu.pid_output = vect_ster[i]
        optimu.iteration(t)
        if save:
            tv.append(t)
            sv.append(optimu.s)
            vv.append(optimu.v)
    if save:
        stats.append(tv)
        stats.append(sv)
        stats.append(vv)
        
    return (optimu.v - 0)**2 + ((optimu.s - 100)/10)**2

#%%Minimize
import numpy as np
from scipy.optimize import minimize

vect_ster = np.zeros(100)
res = minimize(train_ride, vect_ster, method='Nelder-Mead', options={'xatol': 1e-8, 'disp': True})

#%% After res

stats = []
train_ride(res.x, save = True, stats=stats)
tv, sv, vv = stats


#%% Plotting
import matplotlib
import matplotlib.pyplot as plt

# Data for plotting
fig = plt.figure(figsize=(12, 6))

ax = fig.add_subplot(121)
ax2 = fig.add_subplot(122)


ax.plot(tv, vv)

ax2.plot(tv, sv)
ax2.plot(tv, vv)
#ax2.set_xlim([330, 500])
#ax2.set_ylim([9800, 10500])



#fig.plot(tv, zv)



#plt.set(xlabel='time (s)', ylabel='distance (m)',
#      title='Przejazd teoretyczny')
#ax.grid()

plt.show()
