# -*- coding: utf-8 -*-
"""
Created on Wed Feb 24 14:31:37 2021

@author: arkni
"""
from PID import PID
import math
from data_loco import profile, powerLookup, curves, limits, stops

emuPowerLookup = [900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 900, 882, 860, 840, 820, 802, 784, 767, 751, 735, 720, 706, 692, 678, 666, 653, 641, 630, 619, 608, 598, 588, 578, 569, 560, 551, 543, 535, 527, 519, 511, 504, 497, 490, 483, 477, 470, 464, 458, 452, 447, 441, 436, 430, 425, 420, 415, 410, 406, 401, 396, 392, 388, 383, 379, 375, 371, 368, 364, 360, 356, 353, 349, 346, 343, 339, 336, 333, 330, 327, 324, 321, 318, 315, 312, 309, 307, 304, 302, 299, 296, 294, 292, 289, 287, 285, 282, 280, 278, 276, 273, 271, 269, 267, 265, 263, 261, 259, 258, 256, 254, 252, 250, 248, 247, 245, 243, 242, 240, 238, 237, 235, 234, 232, 231, 229, 228, 226, 225, 223, 222, 221, 219, 218, 216, 215, 214, 213, 211, 210, 209, 208]

#%% Klasa pociągu. Pociąg z klasą

class Train():
    
    


    def __init__(self):
        self.mass = 1210000
        self.axles_no = 8
        self.cars_no = 11
        self.a = 0
        self.aBefore = 0
        self.v = 10
        self.s = 0
        self.t = 0
        self.dt = 0.1
        self.F = 0
        self.Fmot = 0
        self.FmotBefore = 0
        self.Fdyn = 0
        self.Fstat = 0
        self.timeCurrent = 0
        self.timeBefore = 0
        self.pid_output = 0 #from 0 to 100. about 50 is no braking neither no speeding up
        
        self.powerLookup = []
        self.profileActual = 0
        
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
        self.check_env()
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
        return 9.8105 * self.profileActual * (self.mass)
        pass
    
    def check_env(self):
        #Checking profile
        if emu.s > self.profileNext[0]:
            self.profileStart = self.profileNext[0]
            self.profileActual = self.profileNext[1]
            try:
                self.profileNext = self.profile.pop(0)
            except:
                self.profileActual = 0
        #Checking curve
        pass
        
    def set_power_lookup(self, powerLookup):
        self.powerLookup = powerLookup
        
    def set_profile(self, profile):
        self.profile = profile
        try:
            tmp = self.profile.pop(0)
            self.profileStart = tmp[0]
            self.profileActual = tmp[1]

            self.profileNext = self.profile.pop(0)
        except:
            self.profileActual = 0
            
        
    def set_curves(self, curves):
        self.curves = curves
        try:
            curves_actual = self.curves.pop(0)
            self.curvesStart = curves_actual[0]
            self.curvesStop = curves_actual[1]
            self.curvesR = curves_actual[2]
        except:
            self.curves_actual = 0
                
                
        
    
    
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
        self.Fmot = min(self.powerLookup[round(emu.v*3.6)]*1000, self.Fmot)
        if False: #Turning off controll
            self.Fmot = 0
        
    def jerk_calc(self):
        return (self.a - self.aBefore)/self.dt
        
#%% Regulator do pociągu
class TrainController:
    
    def __init__(self, emu):
        self.predActive = 0
        self.next_stop = -1
        self.actualLimit = 0
        
        
        self.pid = PID(P = 50, I=0.4, D=1.0, current_time=0)
        self.pid.SetPoint = 30.0
        self.pid.setSampleTime(0.1)
        self.pid.setWindup(20.0)
        
        
        self.pid_poz = PID(P = 40.0, I=3.0, D=200.0, current_time=0)
        self.pid_poz.setSampleTime(0.1)
        self.pid_poz.setWindup(10.0)
        self.pid_poz.setPoint = 0
        
        self.predIter = 0
        self.predLen = 0
        
        #Data collection:
        self.tv = []
        self.sv = []
        self.vv = []
        self.zv = []
        self.predReal = []
        self.predSets = []
    
    def predict(self, emu, predAcc = -0.6):
            
        self.predAcc = predAcc
        self.predCalc = emu.s + (emu.v**2 - 2*predAcc*0)/(-predAcc*2)
            
        if self.predCalc > self.next_stop - 3 and (self.predActive == 0):
    
            self.predActive = 1
            self.predIter = 0
            self.predVel = emu.v
            self.predTime = -emu.v/self.predAcc
            self.predDis = emu.s
            self.predSets = []
            self.predReal = []


            
            for j in range(round(self.predTime/emu.dt)):
                self.predDis += self.predVel*emu.dt + 0.5*self.predAcc*emu.dt**2
                self.predVel += self.predAcc*emu.dt
                self.predSets.append(self.predDis)
            self.predLen = j+1
            
    def control(self, emu):
        self.predict(emu)
        if  self.predActive == 0:
            if emu.s > self.nextLimit[0]:
                self.pid.SetPoint = self.nextLimit[1]
                try:
                    self.nextLimit = self.limits.pop(0)
                except:
                    print("No limits for you")
                    self.nextLimit[0] = 99999999
                    pass
                
            self.pid.update(emu.v, current_time=emu.timeCurrent)
            emu.pid_output = self.pid.output 
            
        if not self.predActive == 1:

            self.zv.append(self.pid.SetPoint)

        
            if self.predIter < self.predLen:
                self.pid_poz.SetPoint = self.predSets[self.predIter]
                self.predIter += 1
                self.pid_poz.update(emu.s, current_time=emu.timeCurrent)
                
                emu.pid_output = self.pid_poz.output
                self.zv.append(self.pid_poz.SetPoint)
                self.predReal.append(emu.s)
        else:
            try:
                self.next_stop = self.stops.pop(0)
            except:
                self.next_stop = 999999999
            self.predActive = 0
            
        self.tv.append(emu.timeCurrent)
        self.sv.append(emu.s)
        self.vv.append(emu.v)
    def set_stops(self, stops):
        self.stops = stops
        self.next_stop = self.stops.pop(0)
    def set_limits(self, limits):
        self.limits = limits
        self.actualLimit = self.limits.pop(0)
        self.pid.SetPoint = self.actualLimit[1]
        self.nextLimit = self.limits.pop(0)
    

        
        
        
#%% Obliczenia

reg = ""

emu = Train()
emu.set_power_lookup(emuPowerLookup)
emu.set_curves(curves)
emu.set_profile(profile)

emuCtrl = TrainController(emu)
emuCtrl.set_stops([10000,17000,25000])
emuCtrl.set_limits([[0,30],[2000,40],[8000,20],[15000,30],[27000,30]])



    
print(emu)
print("Opór dynamiczny: {:.2f}".format(emu.resist_dyn()))
# tv = []
# sv = []
# vv = []
# zv = []
# rv = []




# pid = PID(P = 40, I=0.2, D=0.0, current_time=0)
# pid.SetPoint=30.0
# pid.setSampleTime(0.1)
# pid.setWindup(20.0)


# pid_poz = PID(P = 8.0, I=2.0, D=100.0, current_time=0)
# pid_poz.setSampleTime(0.1)
# pid_poz.setWindup(10.0)
# pid_poz.setPoint = 0





for i in range(130000):
    t = i/10

        
    # pid.update(emu.v, current_time=t)

    # predCalc = emu.s + (emu.v**2 - 2*predAcc*0)/(-predAcc*2)
        
    # if predCalc > next_stop - 3 and (predActive == 0):

    #     predTime = -emu.v/predAcc
    #     predActive = 1
    #     predVel = emu.v
    #     predDt = emu.dt
    #     predSets = []
    #     predDis = emu.s
    #     predIter = 0
    #     predReal = []
        
    #     for j in range(round(predTime/emu.dt)):
    #         predDis += predVel*predDt + 0.5*predAcc*predDt**2
    #         predVel += predAcc*predDt
    #         predSets.append(predDis)
    #     predLen = j+1
    # if not predActive == 1 :
    #     reg_pos = 0
    #     emu.pid_output = pid.output
    #     reg = "spe"
    #     zv.append(pid.SetPoint)
    #     s_set = 0
    # else:

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
    #     if predIter < predLen:
    #         pid_poz.SetPoint = predSets[predIter]
    #         predIter += 1
    #         pid_poz.update(emu.s, current_time=t)
            
    #         emu.pid_output = pid_poz.output
    #         reg = "pos"
    #         zv.append(pid_poz.SetPoint)
    #         predReal.append(emu.s)
    #     else:
    #         predActive = 2
    
    emuCtrl.control(emu)
    emu.iteration(t)
    # tv.append(t)
    # sv.append(emu.s)
    # vv.append(emu.v)

    

    if i%1000 == 0 and i:
        print(emu, "t: {}, out: {:.2f} sil: {:.1f}, {}, Cal:{:.1f}, S:{}, Prof:{}".format(t, emu.pid_output, emu.Fmot/1000,reg,emuCtrl.predCalc,emuCtrl.predActive,emu.profileActual))
    
    if emu.jerk_calc()>0.6:
        print(emu, "t: {}, j: {:.2f} sil: {:.1f}, {}".format(t, emu.jerk_calc(), emu.Fmot/1000, reg))
    
#%% Plotting
import matplotlib
import matplotlib.pyplot as plt

# Data for plotting
fig = plt.figure(figsize=(12, 6), dpi=300)

ax = fig.add_subplot(121)
ax2 = fig.add_subplot(122)


ax.plot(emuCtrl.tv, emuCtrl.vv)

ax2.plot(emuCtrl.predSets,linewidth=3)
ax2.plot(emuCtrl.predReal,'--',linewidth=2)

# ax.set_xlim([1000, 1500])
# ax.set_ylim([29, 31])

# ax2.set_xlim([1000, 1500])
# ax2.set_ylim([24000, 25500])



#fig.plot(tv, zv)

ax.set(xlabel='time (s)', ylabel='velocity (m/s)',
      title='Train velocity')

ax2.set(xlabel='samples (-)', ylabel='distance (m)',
      title='Last braking')
ax.grid()
ax2.grid()
ax2.legend(["Trajectory","Measurement"])

plt.show()








