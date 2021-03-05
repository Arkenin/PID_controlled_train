# -*- coding: utf-8 -*-
"""
Created on Wed Feb 24 14:31:37 2021

@author: arkni
"""
from PID import PID
import math


#%% Klasa pociągu. Pociąg z klasą

class Train():
    
    def __init__(self):
        self.mass = 1210000
        self.axles_no = 8
        self.cars_no = 11
        self.a = 0
        self.aBefore = 0
        self.v = 0
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
        
        
    
#%% Obliczenia

reg = ""
emu = Train()
print(emu)
print("Opór dynamiczny: {:.2f}".format(emu.resist_dyn()))
tv = []
sv = []
vv = []
zv = []
rv = []

pid = PID(P = 40, I=0.2, D=0.0, current_time=0)
pid_poz = PID(P = 8.0, I=2.0, D=100.0, current_time=0)
pid_acc = PID(P = 25, I=100.00, D=200, current_time=0)


pid.SetPoint=30.0
pid.setSampleTime(0.1)
pid.setWindup(20.0)

next_stop = 10000.0

pid_poz.setSampleTime(0.1)
pid_poz.setWindup(10.0)
pid_poz.setPoint = 0

pid_acc.setSampleTime(0.1)
pid_acc.setWindup(50.0)
# predTime = 150
predAcc = -0.6
predActive = 0
ile = 0
for i in range(4500):
    t = i/10
    if i > 800:
        pass
        #pid.SetPoint=30.0
        
    pid.update(emu.v, current_time=t)

    pid_acc.update(emu.a, current_time=t)
    # predCalc = emu.s + emu.v*predTime + 0.5*predAcc*predTime**2
    predCalc = emu.s + (emu.v**2 - 2*predAcc*0)/(-predAcc*2)
    
    # predTime = (-emu.v + (emu.v**2 + 2*predAcc*1000)**0.5)/predAcc
    
    if predCalc > next_stop - 3 and (predActive == 0):
        ile+=(emu.v**2 - 2*predAcc*0)/(predAcc*2)
        appPredCalc = predCalc
        predTime = -emu.v/predAcc
        predActive = 1
        predVel = emu.v
        predDt = emu.dt
        predSets = []
        predDis = emu.s
        predIter = 0
        predReal = []
        
        for j in range(round(predTime/emu.dt)):
            predDis += predVel*predDt + 0.5*predAcc*predDt**2
            predVel += predAcc*predDt
            predSets.append(predDis)
        predLen = j+1
    if not predActive == 1 :
        reg_pos = 0
        emu.pid_output = pid.output
        reg = "spe"
        zv.append(pid.SetPoint)
        s_set = 0
    else:

        # pid_poz.SetPoint = next_stop
        # if reg_pos == 0:
        #     dt = emu.dt
        #     czas = 70
        #     reg_pos = 1
        #     t_start = t
        #     v = emu.v
        #     s_set = emu.s
        #     a = -0.6
        # s_set += v*dt + 0.5*a*dt
        # v += a*dt
        # if v < 0:
        #     v = 0
        if predIter < predLen:
            pid_poz.SetPoint = predSets[predIter]
            predIter += 1
            pid_poz.update(emu.s, current_time=t)
            
            emu.pid_output = pid_poz.output
            reg = "pos"
            zv.append(pid_poz.SetPoint)
            predReal.append(emu.s)
        else:
            predActive = 2
    
    
    emu.iteration(t)
    tv.append(t)
    sv.append(emu.s)
    vv.append(emu.v)
    rv.append(s_set)
    

    if i%10 == 0:
        pass
        # print(emu, "t: {}, out: {:.2f} sil: {:.1f}, {}, Cal:{:.1f}".format(t, emu.pid_output, emu.Fmot/1000,reg,predCalc))
    
    if emu.jerk_calc()>0.5:
        print(emu, "t: {}, j: {:.2f} sil: {:.1f}, {}".format(t, emu.jerk_calc(), emu.Fmot/1000, reg))
    
#%% Plotting
import matplotlib
import matplotlib.pyplot as plt

# Data for plotting
fig = plt.figure(figsize=(12, 6))

ax = fig.add_subplot(121)
ax2 = fig.add_subplot(122)


ax.plot(tv, vv)

ax2.plot(predSets)
ax2.plot(predReal)

# ax2.set_xlim([330, 500])
# ax2.set_ylim([9800, 10500])



#fig.plot(tv, zv)



#plt.set(xlabel='time (s)', ylabel='distance (m)',
#      title='Przejazd teoretyczny')
#ax.grid()

plt.show()








