#!/usr/bin/env pybricks-micropython
# from typing_extensions import TypeVarTuple
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.nxtdevices import LightSensor, TemperatureSensor
from pybricks.iodevices import Ev3devSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import I2CDevice
import math

#constants
#setup
ev3 = EV3Brick()
time = StopWatch()
wheelTrack = 145 #to be measured
wheelDiameter = 33 #to be measured

#motors
leftmotor = Motor(Port.C, positive_direction=Direction.CLOCKWISE)
rightmotor = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
bot = DriveBase(leftmotor, rightmotor, wheelDiameter, wheelTrack)
clawmotor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
sortmotor = Motor(Port.D, positive_direction=Direction.CLOCKWISE)

#sensors
leftcolour = Ev3devSensor(Port.S1)
rightcolour = Ev3devSensor(Port.S2)
ToF = I2CDevice(Port.S4, 0x02 >> 1)
lsidesound = UltrasonicSensor(Port.S3)
# sidecolour = Ev3devSensor(Port.S4)
# frontcolour = Ev3devSensor(Port.S3)
lcol = 0
rcol = 0
fcol = 0
scol = 0

#colours
lblackthres = 3
rblackthres = 3
lwhitethres = 90
rwhitethres = 90
lmid = 51
rmid = 53
lmin = [26, 26, 24, 63] #18 
lmax = [190, 188, 170, 255] #120
rmin = [19, 18, 12, 36]
rmax = [114, 107, 97, 253] #124
lrange = (164, 162, 146) #58/100
rrange = (95, 92, 85) #76/100
DOUBLEBLACK = 'DB'
DOUBLEWHITE = 'DW'
LEFTBLACK = 'LB'
RIGHTBLACK = 'RB'


#program
#linetrack
class pid():
    def __init__(self, kconf):
        self.iSum = 0
        self.prevE = None
        self.kconf = kconf
    
    def e(self, error):
        ep = error*self.kconf[0]
        ed = 0 if self.prevE == None else self.kconf[2]*(error-self.prevE)
        self.prevE = error
        if abs(error) < 0.2: self.iSum += error
        ei = self.kconf[1] * self.iSum
        return min(100, max(-100, ep + ei + ed))

#functions
def calibrate(lcol, rcol):
    global lmin, rmin, lrange, rrange
    lcol = leftcolour.read("RGB")
    rcol = rightcolour.read("RGB")
    clcol = min(1, max(0, float(((lcol[1] - lmin[1])/100)/(lrange[1]/100))))
    crcol = min(1, max(0, float(((rcol[1] - rmin[1])/100)/(rrange[1]/100))))
    return clcol, crcol

knorm = pid((95, 0, 0)) #best k values: 1.38 - 1.55, try 2 next session

def linetrack(lcole, rcole, kconf):
    # lcole, rcole = calibrate(lcol, rcol) #unsure
    # print(200*(lcole - rcole))
    # print(100-abs(lcole-rcole))
    # lcol = (leftcolour.reflection()/lmax)*100
    # rcol = (rightcolour.reflection()/rmax)*100
    # print(lcole - rcole)
    bot.drive((1-abs(lcole - rcole))*60, kconf.e(lcole - rcole)) #or isit abs(error)?

def linetrackForDist(lcol, rcol, kconf, distance):
    # lcol = leftcolour.read("RGB")
    # rcol = rightcolour.read("RGB")
    # lcole = calibrate(lcol, rcol)[0]
    # rcole = calibrate(lcol, rcol)[1]
    # bot.stop()
    # wait(50)
    # bot.reset()
    dist = bot.distance() + distance
    # initialTime = time.time()
    # decelerateDur = max_speed/acceleration_gain
    # decelerateStartDist = dist - 0.5 * max_speed * decelerateDur 
    # decelerateCount = 0
    # print("start:", decelerateStartDist)
    while bot.distance() < dist:
        lcol = leftcolour.read("RGB")
        rcol = rightcolour.read("RGB")
        lcole = calibrate(lcol, rcol)[0]
        rcole = calibrate(lcol, rcol)[1]
        bot.drive((1-abs(lcole - rcole))*60, kconf.e(lcole - rcole))

def get_ms_dist(dist):
    data = dist.read(0x42, length = 2)
    return (0xFF & data[0]) + ((0xFF & data[1]) << 8)


def ToFdetect():
    error = get_ms_dist(dist) - 20
    error = min(errror, 100)
    bot.drive(1-abs(error)*60, 1.2*(error))

# def linetrackForJunction(speed, kconf, ignore = None):
#     speed = min(speed, 690)
#     while True:
#         lcol = (leftcolour.read("NORM")/lmax)*100
#         rcol = (rightcolour.read("NORM")/rmax)*100
#         if not ignore:
#             if checkDouble(lcol, rcol) != DOUBLEWHITE:
#                 # bot.stop()
#                 # wait(50)
#                 # bot.straight(-1)
#                 break  
#             else:
#                 bot.drive(speed, kconf.e(lcol-rcol))
#         else:
#             if ignore == "left":
#                 if checkDouble(lcol, rcol) == DOUBLEBLACK or checkDouble(lcol, rcol) == RIGHTBLACK:
#                     # bot.stop()
#                     # wait(50)
#                     # bot.straight(-1)
#                     break  
#                 else:
#                     bot.drive(speed, kconf.e(lcol-rcol))
#             elif ignore == "right":
#                 if checkDouble(lcol, rcol) == DOUBLEBLACK or checkDouble(lcol, rcol) == LEFTBLACK:
#                     # bot.stop()
#                     # wait(50)
#                     # bot.straight(-1)
#                     break  
#                 else:
#                     bot.drive(speed, kconf.e(lcol-rcol))
#     bot.straight(0)

#movement
toDeg = lambda d: d/wheelDiameter/pi*360 #cm #angle to 
def oneturn(motor, angle, dir = "back"):
    bot.stop()
    degrees = abs(degreeDist((wheelTrack - 5)*(degreeToRadian(angle))))
    if dir == "back":
        if motor == "left":
            if angle < 0:
                leftmotor.run_angle(-1000, degrees)
                leftmotor.stop()
        elif motor == "right":
            if angle < 0:
                rightmotor.run_angle(-1000, degrees)
                rightmotor.stop()
    else:
        if motor == "left":
            if angle > 0:
                leftmotor.run_angle(1000, degrees)
                leftmotor.stop()
        elif motor == "right":
            if angle > 0:
                rightmotor.run_angle(1000, degrees)
                rightmotor.stop()
#colour detection
idealWhite = [35, 41, 57] #recalibrate
idealLeftGreen = [50, 57, 100] #recalibrate
idealRightGreen = [35, 41, 85] #recalibrate
# idealYellow = [[255, 199, 104], [255, 200, 110], [255, 173, 30], [255, 175, 26], [255, 187, 78], [255, 187, 81]]
# idealBlue = [[180, 190, 255], [158, 177, 255], [57, 114, 255], [130, 180, 255], [139, 165, 255], [148, 164, 255]]

is_col = lambda src, tgt: all(c[0] <= x <= c[1] for x, c in zip(src, tgt))

# def detectColour(side):
#     global leftcolour
#     global rightcolour
#     if side == "both":
#         rcol1 = rightcolour.read("NORM")
#         lcol1 = leftcolour.read("NORM")
#     rcol2 = list(rcol1)
#     lcol2 = list(lcol1)
#     rcol2 = rcol2[:3]
#     lcol2 = lcol2[:3]
#     print(rcol2)
#     # print(rcol, lcol)
#     idealW = idealWhite
#     # idealY = idealYellow
#     idealLG = idealLeftGreen
#     idealRG = idealRightGreen
#     # idealB = idealBlue
#     wDiff = 0
#     # yDiff = 0
#     gDiffR = 0
#     gDiffL = 0
#     # bDiff = 0
#     nDiff = 0
#     for i in range(3):
#         wDiff += pow((rcol2[i] - idealW[i]), 2)
#         print(wDiff)
#     # for i in range(3):
#     #     yDiff += pow((rcol2[i] - idealY[i]), 2)
#     for i in range(3):
#         gDiffR += pow((rcol2[i] - idealRG[i]), 2)
#     for i in range(3):
#         gDiffL += pow((lcol2[i] - idealLG[i]), 2)
#     # for i in range(3):
#     #     bDiff += pow((rcol2[i] - idealB[i]), 2)
#     for i in range(3):
#         nDiff += pow((rcol2[i]), 2)
#     # if (yDiff < gDiffR) and (yDiff < bDiff) and (yDiff < nDiff):
#     #     rightcolour = "yellow"
#     if (gDiffR < wDiff) and (gDiffR < gDiffL) and (gDiffR < nDiff): #and (gDiffR < bDiff)
#         rightcolour = "green"
#     # elif (yDiff < gDiffR) and (yDiff < bDiff) and (yDiff < nDiff):
#     #     rightcolour = "yellow"
#     elif (wDiff < gDiffR) and (wDiff < gDiffL) and (wDiff < nDiff): # and (wDiff < gDiffL):
#         rightcolour = "white"
#         leftcolour = "white"
#     elif (gDiffL < gDiffR) and (gDiffL < nDiff) and (gDiffL < wDiff):
#         leftcolour = "green"
#     # elif (bDiff < nDiff):
#     #     rightcolour = "blue"
#     else: 
#         rightcolour = None
#         leftcolour = None
#     return (rightcolour, leftcolour)
#     # elif side == "right":
#     #     fcol = frontcolour.read("NORM")


idealLG = ((11, 15), (20, 32), (20, 33)) #recalibrate
idealRG = ((17, 23), (35, 50), (20, 33)) #recalibrate
'''
def junctionCheck(lcol, rcol): #iteration 1
    global lmin
    global rmin
    # lcol = leftcolour.read("NORM")
    # rcol = rightcolour.read("NORM")
    # print("lcol: " + str(lcol) + "rcol:  " + str(rcol))
    # print("lg: " + str(lg) + "rg:  " + str(rg))    
    if lcol[1] < lmin[1] and rcol[1] < rmin[1]:
        bot.straight(-30)
        wait(1000)
        # lcol = leftcolour.read("NORM")
        # rcol = rightcolour.read("NORM")
        if is_col(rcol, idealRG):
            print("rg")
            bot.straight(50)
            wait(100)
            bot.turn(60) #maybe it doesnt need perfect 90?
        if is_col(lcol, idealLG):
            print("lg")
            bot.straight(50)
            wait(100)
            bot.turn(-60) #maybe it doesnt need perfect 90?
        if is_col(lcol, idealLG) and is_col(rcol, idealRG): 
            print("dg")
            bot.turn(360)
        else:   
            linetrack(lcol, rcol, knorm)
    else:
        linetrack(lcol, rcol, knorm)
'''
def junctionCheckEz(lcol, rcol): #iteration 2
    lcol = leftcolour.read("RGB")
    rcol = rightcolour.read("RGB")
    if calibrate(lcol, rcol)[0] < 0.5 and calibrate(lcol, rcol)[1] < 0.5: #figure out correct no. later
        bot.straight(-20)
        wait(1000)
        lcol1 = leftcolour.read("COLOR")
        rcol1 = rightcolour.read("COLOR")
        print (lcol1, rcol1)
        if lcol1 == 4: #figure out correct no. later
            print("lg")
            bot.straight(50)
            wait(100)
            bot.turn(-60) #maybe it doesnt need perfect 90?
        if rcol1 == 4: #figure out correct no. later
            print("rg")
            bot.straight(50)
            wait(100)
            bot.turn(60) #maybe it doesnt need perfect 90?
        if lcol1 == 4 and rcol1 == 4:
            print("dg")
            bot.turn(360)
        else:   
            linetrack(lcol, rcol, knorm)
    else:
        linetrack(lcol, rcol, knorm)


    #     lcol1 = lcollist[-10:]
    #     rcol1 = rcollist[-10:]
    #     idealRW = [94, 95, 123, 255]
    #     idealLW = [142, 150, 179, 255]
    #     idealRG = [33, 51, 29, 109]
    #     idealLG = [88, 89, 113, 255]
    #     wrDiff = 0
    #     wlDiff = 0
    #     rgDiff = 0
    #     lgDiff = 0
    #     nDiff = 0
    #     for i in range(4):
    #         wrDiff += pow((rcol1[i] - idealRW[i]), 2) #try extracting the indiv x, y, z if it doesn't work
    #         print(wrDiff)
    #     for i in range(4):
    #         wlDiff += pow((lcol1[i] - idealLW[i]), 2) #try extracting the indiv x, y, z if it doesn't work
    #         print(wlDiff)
    #     for i in range(4):
    #         rgDiff += pow((rcol1[i] - idealRG[i]), 2)
    #         print(rgDiff)
    #     for i in range(4):
    #         lgDiff += pow((lcol1[i] - idealLG[i]), 2)
    #         print(lgDiff)
    #     for i in range(4):
    #         nDiff += pow((rcol1[i]), 2)
    #     if (rgDiff < wrDiff) and (rgDiff < nDiff): #and (gDiffR < bDiff), and (rgDiff < lgDiff)
    #         rightcolour = "green"
    #     elif (wrDiff < rgDiff) and (wlDiff < lgDiff) and (wrDiff < nDiff) and (wlDiff < nDiff): # and (wDiff < gDiffL):
    #         rightcolour = "white"
    #         leftcolour = "white"
    #     elif (glDiff < nDiff) and (glDiff < wlDiff): #and (gDiffL < gDiffR)
    #         leftcolour = "green"
    #     else: 
    #         rightcolour = None
    #         leftcolour = None

    # if lcol[1] < lmin[1] and rcol[1] < rmin[1]:    
    #     if rightcolour == "green" and leftcolour == None:
    #         print("rg")
    #         bot.straight(50)
    #         wait(100)
    #         bot.turn(60) #maybe it doesnt need perfect 90?
    #     elif leftcolour == "green" and rightcolour == None:
    #         print("lg")
    #         bot.straight(50)
    #         wait(100)
    #         bot.turn(-60) #maybe it doesnt need perfect 90?
    #     elif rightcolour == "green" and leftcolour == None:
    #         print("dg")
    #         bot.turn(360)
    #     else:
    #         linetrack(lcol, rcol, knorm)

        # if calibrate(lcol, rcol)[1] < 0.6:
        #     linetrack(lcole, rcole, knorm)
        # break
        # if run == 5:
        #     print("done")
        #     while calibrate(lcol, rcol)[1] > 0.6:
        #         rightmotor.run(250)
        #         leftmotor.run(150)
        #         break
        #     linetrack(lcole, rcole, knorm)

#leftobstacleDodge()

# def obstacleDodge():
#     global run
#     if run <=4:
#      tank.on_for_seconds(-20,-20,2.5)
#      tank.off
#      log('Obstacle Detected')
#      sound.beep()
#      tank.on_for_degrees(-100,sturn,350) #tank.on_for_degrees(-100,sturn,turnAvoid) 
#      detects = 0
#      while run <=3:
#         dist = sidesound.distance_centimeters
#         tank.on(0,sslow)
#         if 1 < dist < 12:
#             tank.off()
#             sound.beep()
#             run+=1
#             detects += 1
#             tank.on_for_degrees(0,sslow,450) #sslow #distAvoid
#             tank.on_for_degrees(100,sturn,turnAvoid) #turnAvoid
#         lcol = leftColor.hls
#         rcol = rightColor.hls
#         if run ==4: # detects > 2 and (lcol[1] < lblackThres or rcol[1] < rblackThres): 
#             log('Finished')
#             # tank.on_for_degrees(-50,30,550)
#             tank.on_for_seconds(30,30,0.1)
#             tank.on_for_degrees(-100,sturn,540)
#             linetrack(lcol,rcol,snorm,knorm)
#             # tank.off()
#             # log('Finished')
#             # tank.on_for_degrees(-100,sturn,toDeg(9))
#             break


        

    # detects = 0
    #  while run <=3:
    #     dist = lsidesound.distance()
    #     tank.on(0,sslow)
    #     if 1 < dist < 12:
    #         tank.off()
    #         sound.beep()
    #         run+=1
    #         detects += 1
    #         tank.on_for_degrees(0,sslow,450) #sslow #distAvoid
    #         tank.on_for_degrees(100,sturn,turnAvoid) #turnAvoid
    #     lcol = leftColor.hls
    #     rcol = rightColor.hls
    #     if run ==4: # detects > 2 and (lcol[1] < lblackThres or rcol[1] < rblackThres): 
    #         log('Finished')
    #         # tank.on_for_degrees(-50,30,550)
    #         tank.on_for_seconds(30,30,0.1)
    #         tank.on_for_degrees(-100,sturn,540)
    #         linetrack(lcol,rcol,snorm,knorm)
    #         # tank.off()
    #         # log('Finished')
    #         # tank.on_for_degrees(-100,sturn,toDeg(9))
    #         break
     #  oneturn("right", 70, dir = "front") #tank.on_for_degrees(-100,sturn,turnAvoid = 350) 
    #  detects = 0
    #  while run <=3:
    #     dist = lsidesound.distance()
    #     leftmotor.run(15)
    #     rightmotor.run(50)
    #     if 30 < dist < 90:
    #         bot.stop()
    #         print("y")
    #         run += 1
    #         detects += 1
    #         rightmotor.run_angle(20, 450)
    #         leftmotor.run_angle(0, 450)
    #         rightmotor.run_angle(50, 350)
    #         leftmotor.run_angle(20, 350)
    #         # tank.on_for_degrees(0,sslow,450) #sslow #distAvoid
    #         # tank.on_for_degrees(100,sturn,turnAvoid) #turnAvoid
    #     lcol = leftcolour.rgb
    #     rcol = rightcolour.rgb
    #     if run == 4: # detects > 2 and (lcol[1] < lblackThres or rcol[1] < rblackThres): 
    #         print('Finished')
    #         # tank.on_for_degrees(-50,30,550)
    #         while calibrate(lcol, rcol)[1] > 0.5:
    #             leftmotor.run(20)
    #             rightmotor.run(50)
    #         # tank.on_for_seconds(30,30,0.1)
    #         # tank.on_for_degrees(-100,sturn,540)
    #         linetrack(lcol, rcol, knorm)
    #         # tank.off()
    #         # log('Finished')
    #         # tank.on_for_degrees(-100,sturn,toDeg(9))
    #         break


def ballCollect():
    clawmotor.run_angle(440, 150)
    clawmotor.run_angle(240, 120)
    wait(500)
    clawmotor.run_target(250, 0)

def cubeCollect(side): #s = r(theta)
    clawmotor.run_angle(410, 100)
    clawmotor.run_angle(220, 120)
    wait(500)
    clawmotor.run_target(370, 0)
    if side == "left":
        while calibrate(lcol,rcol)[0] < 0.5:
            rightmotor.run(60)
            leftmotor.run(20)
            break
        linetrack(lcol, rcol, knorm)
    if side == "right":
        while calibrate(lcol,rcol)[1] < 0.5:
            rightmotor.run(20)
            leftmotor.run(60)
            break
        linetrack(lcol, rcol, knorm)


# def sorting(ball):
#     if ball = "black":
#         sortmotor.run_angle(300, 120)
#         sortmotor.run_angle(-300, 120)
#     if ball = "silver":
#         sortmotor.run_angle(300, -120)
#         sortmotor.run_angle(300, 120)

#run
# ev3.speaker.beep()
# ballCollect()

while True:
    print("2")
    lcol = leftcolour.read("RGB")
    rcol = rightcolour.read("RGB")
    lcole, rcole = calibrate(lcol, rcol)
    bot.straight(20)
    # dist = lsidesound.distance()
    # print(dist)
    leftobstacleDodge()
    # print(lcol, rcol)
    # print(calibrate(lcol, rcol))
    # print(colourdists(lcol, rcol)[0])

    # junctionCheckFinal(lcole, rcole)
    # linetrack(lcole, rcole, knorm)
    # ballCollect()

'''
#PROGRAM
while True:
    lcol = leftcolour.rgb()
    rcol = rightcolour.rgb()
    linetrack(lcol, rcol, knorm)
    junctionCheck(lcol, rcol)    

'''

'''
#old 2021 code
toDeg = lambda d: d/wheelDiameter/pi*360 #cm

#movement turning (affected by CG)
turn90 = toDeg(10.5)
turnneg90 = toDeg(-10.5)
distAvoid = toDeg(600.545)
turnAvoid = toDeg(650.712) #4
distJunc = toDeg(2.5)
turnJuncAlign = toDeg(0)
distUTurn = toDeg(-5.236)

#pid
knorm = pid((1.4,0,0.0002)) #1.5 or 1.4?
knormscale = pid((170,0,0.02)) #1.5 or 1.4?
kslow = pid((0.7,0,0.0001))
ksonic = pid((10000000,0.0,-0.0005))
kcomp = pid((1.5,0,0))
kturn = pid((0.5,0,0))

#movement turn
distAvoid = toDeg(6.545)
turnAvoid = toDeg(4.712) 

#speed
snorm = 30
ssweep = 40
sslow = 15
sturn = 20
ssonic = 10
sclaw = (-100, 30, 10, 10)

#colour tuning
# lt: blueCube = ((0.6, 0.7),(9, 13),(-1.1, -0.6)) 
# lt: whiteBall = ((0.6, 0.7),(35.0, 57.0),(-0.2, -0.1))
# lt: orangeBall = ((0.6, 0.7), (20,2 4), (-0.4, -0.2))
blueCube = ((0.55, 0.65), (7, 14), (-1.2, -0.9)) 
whiteBall = ((0.6, 0.71), (15.0, 45.0), (-0.25, -0.15))
orangeBall = ((0.6, 0.71), (28, 35), (-0.45, -0.26))
lblackThres = 23.0
rblackThres = 24.5
lwhiteThres = 252.5 
rwhiteThres = 220.0
# green = ((0.2, 0.8), (40, 100), (-0.5, -0.1))


lgreen = ((0.2, 0.4), (88,100), (-0.5,-0.1))
rgreen = ((0.3, 0.5), (80, 100), (-0.35, -0.15)) 

#cn change to (50,90) if needed

lred = ((0.01, 0.04), (90, 110), (-0.9, -0.7)) #to be calibrated
rred = ((0.9, 1.1), (70,90), (-0.75, -0.6)) #to be calibrated
lyellow = ((0.1, 0.3), (130, 140), (0.7, -0.5)) #to be calibrated
ryellow = ((0.1, 0.3), (130, 160), (-0.5, -0.3)) #to be calibrated
lblue = ((0.55, 0.7), (95, 115), (-0.6, -0.4)) #to be calibrated
rblue = ((0.58, 0.7), (115, 125), (-0.75, -0.5)) #to be calibrated
floorColor = (0.0, 5.0, 1.0)
# blueStrip = ((0.3,0.7),(0.3,0.6),(0.8,1.1))
# silver = ((0,0.6),(130,255),(-0.5,0))
# black = ((),(),()) #to be calibrated
# white = ((),(),()) #to be calibrated
# blackBall = ((0.5,0.8),(0,7),(-1,-0.2)) 
blackbox = ((0.6,0.7),(2.5,3.0),(-2.0,-1.5)) #to be calibrated

lcollist0 = []
rcollist0 = []
lcollist1 = []
rcollist1 = []
lcollist2 = []
rcollist2 = []

lcolmax1 = 237.0
lcolmin1 = 20.0
rcolmax1 = 219.0
rcolmin1 = 8.0
lcolmax0 = 0.994553
lcolmin0 = 0.0655270
rcolmax0 = 0.997916
rcolmin0 = 0.243243
lcolmax2 = -0.0121065
lcolmin2 = -0.746192
rcolmax2 = -0.0866141
rcolmin2 = -0.793103


lcolper = []
rcolper = []
lgreenscale = ((0.08357, 0.82846), (0.18723, 0.72768), (0.098924, 0.97811))
rgreenscale = ((-0.088518, 0.82863), (0.14014, 0.77030), (0.08590, 0.97656))
#lgreenscale = ((0.20623, 0.40623), (0.286727, 0.486727), (0.27216, 0.47216))
#rgreenscale = ((0.04816, 0.24816), (0.18978, 0.38978), (0.52738, 0.72738))

lblackscale = ((0.07216,0.94844),(0.041189,0.21441),(0.09892,1.00630))
rblackscale = ((0.39459, 0.89885), (0.038004, 0.21163), (0.09625, 0.96144))
lwhitescale = ((0.149481, 0.84536), (0.80549, 1.03890), (0.77193, 1.0321))
rwhitescale = ((0.69999, 0.80935), (0.74109, 1.0118), (0.63937, 0.89521))
lredscale = ((-0.072737, -0.061537), (0.35926, 0.39359), (-0.44235, -0.37156))
rredscale = ((1.6363, 1.6648), (0.25178, 0.28028), (-0.21434, -0.095524))
lbluescale = ((0.69072, 0.71413), (0.25171, 0.34096), (0.039618, 0.17406))
rbluescale = ((0.78826, 0.80858), (0.27790, 0.35866), (-0.097637, 0.020864))
lyellowscale =((0.10222, 0.12559), (0.43935, 0.55148), (0.0024608, 0.26269))
ryellowscale =((-0.53907, -0.49084), (0.38004, 0.47980), (0.59944, 0.81330))
#green
lgreenlist = ((0.23931,0.64743),(81.5,169.0),(-0.45945,-0.030716)) #key in the max & min value of the green
rgreenlist = ((0.32407,0.55092),(38.5,108.0),(-0.333333,-0.13812))
#black
lblacklist = ((0.125,0.83333),(186.0,35.0),(-0.45945,-0.016949))
rblacklist = ((0.61666, 0.69298),(17.0,32.5),(-0.54285,-0.21951))
#white
lwhitelist = ((0.1875,0.75),(186.0,237.0),(-0.13126,-0.0043478))
rwhitelist = ((0.61868,0.65954),(165.0,222.0),(-0.30726,-0.14932))
#red
lredlist = ((0.0078740,0.016927),(88.5,96.0),(-0.72340,-0.68888))
rredlist = ((0.96851,0.97916),(62.0,68.0),(-0.72357,-0.66917))
#blue
lbluelist = ((0.625,0.64393),(65.0,84.5),(-0.48837,-0.42281))
rbluelist = ((0.65166,0.65925),(67.5,84.5),(-0.66666,-0.60810))
#yellow
lyellowlist = ((0.14930,0.16819),(106.0,130.5),(-0.50649,-0.37959))
ryellowlist = ((0.15573,0.17375),(89.0,110.0),(-0.32673,-0.19230))



minlcol = (0.23,26.5,-0.17) #colour when black
minrcol = (0.65,23.5,-0.22) #colour when black
#minhcol = (,,) #colour when black
maxlcol = (0.65,191,-0.05) #colour when white
maxrcol = (0.64,168,-0.29) #colour when white
maxhcol = (0.7,2.5,-2.0) #colour when white
rangelcol = (0.42,164.5,0.12)
rangercol = (0.01,144.5,0.51)
#rangehcol = (,,) 

# greenleft = () #to be calibrated 
# greenright = ()
# greenscale = (,,)


# lcolmax = maxlcol[1]
# lcolmin = minlcol[1]
# rcolmax = maxrcol[1]
# rcolmin = minrcol[1]

#enums
DOUBLEBLACK = 'DB'
DOUBLEWHITE = 'DW'
DOUBLENONE = 'DN'
DOUBLESILVER = 'DS'
DOUBLERED = 'DR'
DOUBLEYELLOW = 'DY'
DOUBLEBLUE = 'DBLUE'
DOUBLEGREEN = 'DG'


#sensors
leftColor = ColorSensor(INPUT_4)
rightColor = ColorSensor(INPUT_3)
hornColor = ColorSensor(INPUT_2)
leftColor.mode = ColorSensor.MODE_RGB_RAW
rightColor.mode = ColorSensor.MODE_RGB_RAW
hornColor.mode = ColorSensor.MODE_REF_RAW
sidesound = UltrasonicSensor(INPUT_1)
sidesound.mode = UltrasonicSensor.MODE_US_DIST_CM
#hornsound = UltrasonicSensor(C3)
#hornsound.mode = UltrasonicSensor.MODE_US_DIST_CM
sound = Sound()

#move
tank = MoveSteering(OUTPUT_A,OUTPUT_B)
#tank.set_args(speed_i=tank.motors[OUTPUT_A].speed_i*1.5)
claw = Motor(OUTPUT_C)
sorter = Motor(OUTPUT_D)
claw.position = 0
claw.polarity = "inversed"
#gate = Motor(OUTPUT_D)


sclaw = (100,-30,-10,-10)

numDetectBall = 3

#functions
is_col = lambda src,tgt: all(c[0] <= x <=c[1] for x,c in zip(src,tgt))

def rescale(lcol,rcol):
    global lcolper
    global rcolper
    global lgreenscale
    global rgreenscale
    global lblackscale
    global rblackscale
    global lwhitescale 
    global rwhitescale 
    global lredscale
    global rredscale
    global lbluescale
    global rbluescale
    global lyellowscale
    global ryellowscale
    lcolper = ((lcol[0]-lcolmin0)/(lcolmax0-lcolmin0),(lcol[1]-lcolmin1)/(lcolmax1-lcolmin1),(lcol[2]-lcolmin2)/(lcolmax2-lcolmin2))
    rcolper = ((rcol[0]-rcolmin0)/(rcolmax0-rcolmin0),(rcol[1]-rcolmin1)/(rcolmax1-rcolmin1),(rcol[2]-rcolmin2)/(rcolmax2-rcolmin2))
    
    
    lgreenscale = (((lgreenlist[0][0]-lcolmin0)/(lcolmax0-lcolmin0),(lgreenlist[0][1]-lcolmin0)/(lcolmax0-lcolmin0)),((lgreenlist[1][0]-lcolmin1)/(lcolmax1-lcolmin1),(lgreenlist[1][1]-lcolmin1)/(lcolmax1-lcolmin1)),((lgreenlist[2][0]-lcolmin2)/(lcolmax2-lcolmin2),(lgreenlist[2][1]-lcolmin2)/(lcolmax2-lcolmin2)))
    rgreenscale = (((rgreenlist[0][0]-rcolmin0)/(rcolmax0-rcolmin0),(rgreenlist[0][1]-rcolmin0)/(rcolmax0-rcolmin0)),((rgreenlist[1][0]-rcolmin1)/(rcolmax1-rcolmin1),(rgreenlist[1][1]-rcolmin1)/(rcolmax1-rcolmin1)),((rgreenlist[2][0]-rcolmin2)/(lcolmax2-lcolmin2),(rgreenlist[2][1]-rcolmin2)/(rcolmax2-rcolmin2)))
    lblackscale = (((lblacklist[0][0]-lcolmin0)/(lcolmax0-lcolmin0),(lblacklist[0][1]-lcolmin0)/(lcolmax0-lcolmin0)),((lblacklist[1][0]-lcolmin1)/(lcolmax1-lcolmin1),(lblacklist[1][1]-lcolmin1)/(lcolmax1-lcolmin1)),((lblacklist[2][0]-lcolmin2)/(lcolmax2-lcolmin2),(lblacklist[2][1]-lcolmin2)/(lcolmax2-lcolmin2)))
    rblackscale = (((rblacklist[0][0]-rcolmin0)/(rcolmax0-rcolmin0),(rblacklist[0][1]-rcolmin0)/(rcolmax0-rcolmin0)),((rblacklist[1][0]-rcolmin1)/(rcolmax1-rcolmin1),(rblacklist[1][1]-rcolmin1)/(rcolmax1-rcolmin1)),((rblacklist[2][0]-rcolmin2)/(lcolmax2-lcolmin2),(rblacklist[2][1]-rcolmin2)/(rcolmax2-rcolmin2)))
    lwhitescale = (((lwhitelist[0][0]-lcolmin0)/(lcolmax0-lcolmin0),(lwhitelist[0][1]-lcolmin0)/(lcolmax0-lcolmin0)),((lwhitelist[1][0]-lcolmin1)/(lcolmax1-lcolmin1),(lwhitelist[1][1]-lcolmin1)/(lcolmax1-lcolmin1)),((lwhitelist[2][0]-lcolmin2)/(lcolmax2-lcolmin2),(lwhitelist[2][1]-lcolmin2)/(lcolmax2-lcolmin2)))
    rwhitescale = (((rwhitelist[0][0]-rcolmin0)/(rcolmax0-rcolmin0),(rwhitelist[0][1]-rcolmin0)/(rcolmax0-rcolmin0)),((rwhitelist[1][0]-rcolmin1)/(rcolmax1-rcolmin1),(rwhitelist[1][1]-rcolmin1)/(rcolmax1-rcolmin1)),((rwhitelist[2][0]-rcolmin2)/(lcolmax2-lcolmin2),(rwhitelist[2][1]-rcolmin2)/(rcolmax2-rcolmin2)))
    lredscale = (((lredlist[0][0]-lcolmin0)/(lcolmax0-lcolmin0),(lredlist[0][1]-lcolmin0)/(lcolmax0-lcolmin0)),((lredlist[1][0]-lcolmin1)/(lcolmax1-lcolmin1),(lredlist[1][1]-lcolmin1)/(lcolmax1-lcolmin1)),((lredlist[2][0]-lcolmin2)/(lcolmax2-lcolmin2),(lredlist[2][1]-lcolmin2)/(lcolmax2-lcolmin2)))
    rredscale = (((rredlist[0][0]-rcolmin0)/(rcolmax0-rcolmin0),(rredlist[0][1]-rcolmin0)/(rcolmax0-rcolmin0)),((rredlist[1][0]-rcolmin1)/(rcolmax1-rcolmin1),(rredlist[1][1]-rcolmin1)/(rcolmax1-rcolmin1)),((rredlist[2][0]-rcolmin2)/(lcolmax2-lcolmin2),(rredlist[2][1]-rcolmin2)/(rcolmax2-rcolmin2)))
    lbluescale = (((lbluelist[0][0]-lcolmin0)/(lcolmax0-lcolmin0),(lbluelist[0][1]-lcolmin0)/(lcolmax0-lcolmin0)),((lbluelist[1][0]-lcolmin1)/(lcolmax1-lcolmin1),(lbluelist[1][1]-lcolmin1)/(lcolmax1-lcolmin1)),((lbluelist[2][0]-lcolmin2)/(lcolmax2-lcolmin2),(lbluelist[2][1]-lcolmin2)/(lcolmax2-lcolmin2)))
    rbluescale = (((rbluelist[0][0]-rcolmin0)/(rcolmax0-rcolmin0),(rbluelist[0][1]-rcolmin0)/(rcolmax0-rcolmin0)),((rbluelist[1][0]-rcolmin1)/(rcolmax1-rcolmin1),(rbluelist[1][1]-rcolmin1)/(rcolmax1-rcolmin1)),((rbluelist[2][0]-rcolmin2)/(lcolmax2-lcolmin2),(rbluelist[2][1]-rcolmin2)/(rcolmax2-rcolmin2)))
    #lyellowscale = (((lyellowlist[0][0]-lcolmin0)/(lcolmax0-lcolmin0),(lyellowlist[0][1]-lcolmin0)/(lcolmax0-lcolmin0)),((lyellowlist[1][0]-lcolmin1)/(lcolmax1-lcolmin1),(lyellowlist[1][1]-lcolmin1)/(lcolmax1-lcolmin1)),((lyellowlist[2][0]-lcolmin2)/(lcolmax2-lcolmin2),(lyellowlist[2][1]-lcolmin2)/(lcolmax2-lcolmin2)))
    ryellowscale = (((ryellowlist[0][0]-rcolmin0)/(rcolmax0-rcolmin0),(ryellowlist[0][1]-rcolmin0)/(rcolmax0-rcolmin0)),((ryellowlist[1][0]-rcolmin1)/(rcolmax1-rcolmin1),(ryellowlist[1][1]-rcolmin1)/(rcolmax1-rcolmin1)),((ryellowlist[2][0]-rcolmin2)/(lcolmax2-lcolmin2),(ryellowlist[2][1]-rcolmin2)/(rcolmax2-rcolmin2)))
    
    return lcolper,rcolper

def log(msg):
    print(msg)
    print(msg, file=stderr)

def linetrack(lcol,rcol,speed,kconf): 
    tank.on(kconf.e(lcol[1]-rcol[1]), speed)

def linetrackper(lcolper,rcolper,speed,kconf): 
    tank.on(kconf.e(lcolper[1]-rcolper[1]), speed)
    #log("left speed:"+str(kconf.e(lcol[1]-rcol[1]))+"right speed:"+str(speed))

def uturn(speed,lcol,rcol):
    log("Turn U")
    # tank.on_for_degrees(0,speed,distUTurn)
    # tank.on(-100,speed)
    tank.on_for_seconds(-100,speed,4)
    # if lcol[1] > lwhiteThres and rcol[1] > rwhiteThres:
    #     linetrack(lcol,rcol,snorm,knorm)

    #while lcol[1] > lblackThres and rcol[1] > rblackThres: pass
    #while leftColor.rgb[1] > blackThres: pass

def leftlineturn(speed):
    tank.on_for_seconds(speed,-100,0.5)

def rightlineturn(speed):
    tank.on_for_seconds(-100,speed,0.5)
    # log("Turn "+("left" if steer < 0 else "right"))
    # tank.on_for_degrees(0,speed,distJunc)
    # tank.on(steer,speed)
    # # while rcol[1] > rblackThres if steer > 0 else lcol[1] > lblackThres: pass
    
    # if steer > 0:
    #     while rightColor.hls[1] > rblackThres:
    #         pass
    # else:
    #     while leftColor.hls[1] > lblackThres:
    #         pass

    # tank.on_for_degrees(steer,speed,turnJuncAlign)

def checkDouble(lcol,rcol):
    if lcol[1] < lblackThres and rcol[1] < rblackThres: return DOUBLEBLACK
    if lcol[1] > lwhiteThres and rcol[1] > rwhiteThres: return DOUBLEWHITE
    # if is_col(lcolper,lblackscale) and is_col(rcolper,rblackscale): 
    #     print("doubleblack")
    #     return DOUBLEBLACK
    # if is_col(lcolper,lwhitescale) and is_col(rcolper,rwhitescale):
    #     return DOUBLEWHITE
    # if is_col(lcol,lred) and is_col(rcol,rred): return DOUBLERED
    # if is_col(lcol,lyellow) and is_col(rcol,ryellow): return DOUBLEYELLOW
    # if is_col(lcol,lgreen) and is_col(rcol,rgreen): return DOUBLEGREEN 
    # if is_col(lcol,lblue) and is_col(rcol,rblue): return DOUBLEBLUE
  

#try disabling green junc detection for set time after junc turn
junctionFlag = False
#lcolWhite = False
#rcolBlack = False
squares = 0
def junctionCheck(lcol,rcol):
    global junctionFlag
    global squares
    if checkDouble(lcol,rcol) == DOUBLEBLACK: junctionFlag = True
    if checkDouble(lcol,rcol) == DOUBLEWHITE: junctionFlag = False
    if not junctionFlag:
        lg = is_col(lcol,lgreen)
        rg = is_col(rcol,rgreen)
        if lg or rg:
            if checkDouble(lcol,rcol) != DOUBLEBLACK and checkDouble(lcol,rcol) != DOUBLEWHITE:
                lcol = leftColor.hls
                rcol = rightColor.hls
                # linetrack(lcol,rcol,sslow,kslow) #may need to remove
                if is_col(lcol,lgreen): lg = True
                if is_col(rcol,rgreen): rg = True
                print(lcol)
                print(lg)
                print(rcol)
                print(rg)
                if lg and rg: 
                    print("doublegreen detected")
                    uturn(sturn,lcol,rcol)
                    linetrack(lcol,rcol,snorm,knorm)
                # if lg and squares >= 4:
                #     sound.beep()
                #     print("left green detected")
                #     # tank.on_for_seconds(snorm,snorm,0.2)
                #     # leftlineturn(snorm)
                #     tank.on_for_degrees(-50,30,300) #maybe it doesnt need perfect 90?
                #     squares +=1
                #     break
                if rg: # or lg 
                    sound.beep()
                    print("right green detected")
                    tank.on_for_seconds(0,-30,0.2)
                    if checkDouble(lcol,rcol) != DOUBLEBLACK:
                     tank.on_for_seconds(0,30,0.3)
                     tank.on_for_degrees(50,30, 280) #350)
                     squares += 1 
                    else:
                     linetrack(lcol,rcol,snorm,knorm)
                # elif rg  and squares == 1: #or lg
                #     sound.beep()
                #     print("right green detected")
                #     tank.on_for_degrees(50,30,480) 
                #     squares +=1 
                # elif rg and squares >=2: 
                #     # tank.on_for_seconds(snorm,snorm,0.3)
                #     # rightlineturn(snorm)
                #     sound.beep()
                #     print("right green detected")
                #     tank.on_for_degrees(50,30,350) #maybe it doesnt need perfect 90?
                #     squares +=1
                #     break
                # elif lg or rg and squares == 4:
                #     sound.beep()
                #     print("right green detected")
                #     tank.on_for_degrees(50,30,350) #maybe it doesnt need perfect 90?
                #     squares +=1
                #     break
                elif lg: # or rg and squares == 5
                    sound.beep()
                    print("left green detected")
                    tank.on_for_degrees(-50,30,300) #maybe it doesnt need perfect 90?
                    squares +=1
                # elif lg or rg and squares == 6:
                #     sound.beep()
                #     print("left green detected")
                #     tank.on_for_degrees(-50,30,300) #maybe it doesnt need perfect 90?
                #     squares +=1
                #     break
                    
               
squares = 0
def junctionCheck(lcol,rcol):
    # if checkDouble(lcolper,rcolper) == DOUBLEWHITE: 
    global squares
    lcol = leftColor.hls
    rcol = rightColor.hls
    lg = is_col(lcol,lgreen)
    rg = is_col(rcol,rgreen)
    print("lcol: "+str(lcol)+"rcol:  "+str(rcol))
    print("lg: "+str(lg)+"rg:  "+str(rg))
            
    if lg or rg and squares == 1:
       print("right green detected")
       tank.on_for_degrees(50,30,300) #maybe it doesnt need perfect 90?
       squares +=1 
    if lg or rg and squares == 5:
        print("right green detected")
        tank.on_for_degrees(50,30,300) #maybe it doesnt need perfect 90?
        squares +=1 
    if checkDouble(lcol,rcol) == DOUBLEBLACK:
        print("double black detected")
        tank.on_for_degrees(-20,-20,120)
        tank.off()
        print("stop")
        # tank.off()
        # tank.off()
        # tank.off()
        # tank.off()                
        
        if lg and rg and squares >= 8: 
            print("doublegreen detected")
            uturn(sturn,lcol)
        elif lg or rg and squares == 0:
            sound.beep()
            print("right green detected")
            tank.on_for_degrees(50,30,300) #maybe it doesnt need perfect 90?
            squares +=1
            # break
            # lineturn(-100,sturn,rcol,lcol)   
        elif lg or rg and squares == 2:
            sound.beep()
            print("right green detected")
            tank.on_for_degrees(50,30,300) #maybe it doesnt need perfect 90?
            squares +=1
            # break
        elif lg or rg and squares == 4:
            sound.beep()
            print("left green detected")
            tank.on_for_degrees(-50,30,300) #maybe it doesnt need perfect 90?
            squares +=1
            # break
            # lineturn(100,sturn,rcol,lcol)
        else: 
            tank.on_for_seconds(0,30,2)

cube = 0
ballDetFlag = 0
def cubeCollect(hcol):
    global ballDetFlag
    global cube
    #log(hcol)
    ob = is_col(hcol,orangeBall)
    wb = is_col(hcol,whiteBall)
    bc = is_col(hcol,blueCube)
    if ob or wb or bc:
        tank.off()
        log("Lifting...")
        while Motor.STATE_STALLED in claw.state: claw.on(80)
        claw.on_for_seconds(30,30)
        sound.beep()

        log("Releasing...")
        while Motor.STATE_STALLED in claw.state: claw.on(-20)
        claw.on_for_seconds(-20,1)
        sound.beep()
        log("Sorting...")
        
        if ob:
            log("orange")
            sorter.on_for_degrees(sclaw[3],100)
            sorter.on_for_degrees(sclaw[3],-100)
        elif wb:
            log("white")
            sorter.on_for_degrees(sclaw[3],-100)
            sorter.on_for_degrees(sclaw[3],100)
        elif bc:
            log("blue")
            sorter.on_for_degrees(sclaw[3],100)
            sorter.on_for_degrees(sclaw[3],-100) 
        ballDetFlag += 1
        cube += 1
        if cube == 1:
            sound.beep()
            tank.on_for_seconds(0,30,8)
            linetrack(lcol,rcol,snorm,knorm)
            cube+=1


def ballCollect(hcol):
    global ballDetFlag
    #log(hcol)
    ob = is_col(hcol,orangeBall)
    wb = is_col(hcol,whiteBall)
    bc = is_col(hcol,blueCube)
    if ob or wb or bc:
        # tank.off()
        log("Lifting...")
        while Motor.STATE_STALLED in claw.state: claw.on(80)
        claw.on_for_seconds(30,30)
        sound.beep()
        tank.off()
        log("Releasing...")
        while Motor.STATE_STALLED in claw.state: claw.on(-20)
        claw.on_for_seconds(-20,1)
        sound.beep()
        log("Sorting...")
        
        if ob:
            log("orange")
            sorter.on_for_degrees(sclaw[3],100)
            sorter.on_for_degrees(sclaw[3],-100)
        elif wb:
            log("white")
            sorter.on_for_degrees(sclaw[3],-100)
            sorter.on_for_degrees(sclaw[3],100)
        elif bc:
            log("blue")
            sorter.on_for_degrees(sclaw[3],100)
            sorter.on_for_degrees(sclaw[3],-100) 
        ballDetFlag += 1

run = 0
def obstacleDodge():
    global run
    if run <=4:
     tank.on_for_seconds(-20,-20,2.5)
     tank.off
     log('Obstacle Detected')
     sound.beep()
     tank.on_for_degrees(-100,sturn,350) #tank.on_for_degrees(-100,sturn,turnAvoid) 
     detects = 0
     while run <=3:
        dist = sidesound.distance_centimeters
        tank.on(0,sslow)
        if 1 < dist < 12:
            tank.off()
            sound.beep()
            run+=1
            detects += 1
            tank.on_for_degrees(0,sslow,450) #sslow #distAvoid
            tank.on_for_degrees(100,sturn,turnAvoid) #turnAvoid
        lcol = leftColor.hls
        rcol = rightColor.hls
        if run ==4: # detects > 2 and (lcol[1] < lblackThres or rcol[1] < rblackThres): 
            log('Finished')
            # tank.on_for_degrees(-50,30,550)
            tank.on_for_seconds(30,30,0.1)
            tank.on_for_degrees(-100,sturn,540)
            linetrack(lcol,rcol,snorm,knorm)
            # tank.off()
            # log('Finished')
            # tank.on_for_degrees(-100,sturn,toDeg(9))
            break

zoneDetects = 0
def zoneDetection(lcol,rcol):
    if is_col(lcol,lred) and is_col(rcol,rred):
        sound.beep()
        print("zone detected")
        zoneCollect()
        # global zoneDetects
        # zoneDetects += 1
    # if zoneDetects > 5: 
    #     zoneCollect()
        

boundaryFlag = False
detectGreen = 0
notexit = True
def boundaryCheck(lcol,rcol):
    global boundaryFlag
    global exitFlag
    global detectGreen

    if is_col(lcol,lyellow) or is_col(rcol,ryellow):
        boundaryFlag = True
    if is_col(lcol,lred) or is_col(rcol,rred):
        boundaryFlag = True
    if is_col(lcol,lblue) or is_col(rcol,rblue):
        boundaryFlag = True
    if lcol[1] < lblackThres or rcol[1] < rblackThres:  
        boundaryFlag = True
    if is_col(lcol,lgreen) or is_col(rcol,rgreen):   
        boundaryFlag = True

    # if checkDouble(lcol,rcol) != DOUBLEWHITE: 
    #     boundaryFlag = True
    # if checkDouble(lcol,rcol) == DOUBLEGREEN: 
    #     boundaryFlag = True
    #     notexit = False
    

    

def align():
    tank.on_for_degrees(0,snorm,toDeg(-10))
    tank.on_for_degrees(0,snorm,toDeg(5))

def walltrack(val,tgt,speed,kconf): 
    tank.on(kconf.e(val-tgt),speed)

def gyrotrack(val,tgt,speed,kconf):
    e = tgt - val
    tank.on(kconf.e(-e),speed)


def gyroturn2(tgt,speed,kconf):
    while True:
        e = tgt - compass.value()
        if e > 180: e = e - 360
        elif e < -180: e = e + 360
        if e == 0: 
            tank.off()
            break
        tank.on(-100 if e<0 else 100,max(3,kconf.e(abs(e))))
        
def gyroturn(ang,kconf,ini=None):
    ini = compass.angle if ini == None else ini
    tgt = ini - ang
    while True:
        e = compass.angle - tgt
        #log((e,tgt,compass.angle))
        if abs(e) == 0: 
            tank.off()
            return tgt
        tank.on(-100 if e<0 else 100,max(3,kconf.e(abs(e))))

ballCount = 0
dropCount = 0
successDeposit = False
depositFlag = 0
def zoneCollect():
    global ballCount
    global dropCount
    global successDeposit
    global boundaryFlag
    global depositFlag
    
    tank.on_for_seconds(0,50,1)
    tank.on_for_degrees(-50,30,730)

    print("align upon entry")
    
    #spiral calculations
    total_turns = 10
    back_dist = 3
    spiral_decrement = 10
    turns = 0
    # successDeposit = False
    
    while turns <= total_turns:
        lcol = leftColor.hls
        rcol = rightColor.hls
        hcol = hornColor.hls
        tank.on(0,30) #can change the speed
        ballCollect(hcol)

        
        if ballDetFlag >= 1 and is_col(hcol,blackbox):
            print("evacuation point found")
            tank.on_for_seconds(0,-50,1)
            tank.on_for_degrees(0,sturn,distUTurn)
            tank.on(-100,sturn)
            tank.on_for_seconds(0,-30,1)

            # gate.on_for_degrees(10,135) # speed, degrees to be calibrated
            # gate.on_for_seconds(0,2) # rest for 2 seconds
            # gate.on_for_degrees(10,-135) 
            depositFlag += 1
            if depositFlag >= 3:
                successDeposit = True
        elif is_col(hcol,blackbox):
            tank.on_for_seconds(0,-50,1)
            tank.on_for_degrees(50,30,780)
        

        boundaryCheck(lcol,rcol)
        if boundaryFlag:
            if turns == 0:
                sound.beep()
                print("odd turn")
                print(turns)
                # log((back_dist,turns))
                tank.on_for_seconds(0,-30,2.5)
                #tank.on_for_degrees(0,ssweep,toDeg(-back_dist)) # to be verified how to turn 
                tank.on_for_degrees(50,30,780) #turn 90
                tank.on_for_seconds(0,50,0.7)
                tank.on_for_degrees(50,30,780)
                turns += 1
                boundaryFlag = False
                sound.beep()

        if boundaryFlag:
            if turns == 1 or turns == 3 or turns == 5 or turns == 7 or turns == 9:
                sound.beep()
                print("even turn")
                print(turns)
                tank.on_for_seconds(0,-30,2.5)
                tank.on_for_degrees(-50,30,780)
                tank.on_for_seconds(0,50,0.7)
                tank.on_for_degrees(-50,30,780)
                turns += 1
                boundaryFlag = False
                sound.beep()

        if boundaryFlag:
            if turns == 2 or turns == 4 or turns == 6 or turns == 8 or turns == 10:
                sound.beep()
                print("even turn")
                print(turns)
                tank.on_for_seconds(0,-30,2.5)
                tank.on_for_degrees(50,30,780)
                tank.on_for_seconds(0,50,0.7)
                tank.on_for_degrees(50,30,780)
                turns += 1
                boundaryFlag = False
                sound.beep()

        if successDeposit:
            break
    
    
    while successDeposit:
        if is_col(lcol,lyellow) or is_col(rcol,ryellow):
            tank.on_for_seconds(0,20,0.5)
            tank.on_for_degrees(-50,30,780)
            tank.on_for_seconds(0,20,0.5)
            break
        linetrack(lcol,rcol,snorm,knorm)

        tank.on(0,50)
        boundaryCheck(lcol,rcol)
        if boundaryFlag:
            tank.on(0,100)
            if hcol > floorColor[1]:
                if is_col(hcol,blueCube):
                    ballCollect(hcol)
                else:
                    obstacleDodge()
            else:
                junctionCheck(lcol,rcol)
                linetrack(lcol,rcol,snorm,knorm)
                
        else:
            tank.on(0,-30)
            tank.on_for_degrees(0,snorm,turnneg90)

            # tank.on_for_degrees(0,snorm,distJunc)
            # tank.on(-100,snorm)
            # while rightColor.hls > rblackThres if steer > 0 else leftColor.hls[1] > lblackThres: pass
            # tank.on_for_degrees(-100,snorm,turnJuncAlign)

            linetrack(lcol, rcol, snorm, knorm)

            if is_col(hcol,green):
                notexit = False

            # if is_col(lcol,lgreen) and is_col(rcol,rgreen):
            #     notexit = False
    
           
#time == input(end time)  

#use time to determine distance travelled and to be travelled, to be calibrated and testd
def zoneSpiral():
    if #time < <
    #travel < original time at same speed
    #tank.on(100)
    #time shorter side 
    #tank.on(100)
    #time longer side
    #tank.on(100)
    #time shorter side
    
    #if ssnd detect box
    if obstacleDodge #time when short side, break, must turn left then turn right, turn left
    #travel time shorter (with box)
    if obstacleDodge #time when long side break, must turn left then turn right, turn left
    #travel time shorter (with box)


#use color sensor to detect the box (deposit zone) - make it go straight until reach black line and track 
#until one of the corners
#code for detecting obstacles + ball + calibrate black box ultrasound
#use ultrasonic to detect where the box is to avoid it and continue spiralling 
#after all 3 have been deposited, use ultrasonic again to avoid the box and go back to the black line
#havent figure out how to travel when drop ball (make bot go in front a bit then turn left?)

'''
'''
while False: #ball collect mechanism
    print("5 Feb start")
    hcol = hornColor.rgb
    #log(hcol)

    ballCollect(hcol)
while True: #line tracking mechanism
    lcol = leftColor.rgb
    rcol = rightColor.rgb
    hcol = hornColor.rgb
    ssnd = sidesound.distance_centimeters
    #hsnd = hornsound.distance_centimeters
    log("Left Color: "+str(lcol)+", Right Color: "+str(rcol)+", Front Color: "+str(hcol))
    #continue
    
    if hcol > floorColor[1]:
        if is_col(hcol,blue):
            ballcollect()
        else:
            obstacleDodge()
    else:
        linetrack(lcol,rcol,snorm,knorm)
    
    junctionCheck(lcol,rcol)
    zoneDetection(lcol,rcol,hcol)
    
    #if 1 < hsnd < 14: obstacleDodge()
    #else: linetrack(lcol,rcol,snorm,knorm)
    #if checkDouble(lcol,rcol) == DOUBLEWHITE: zoneDetection(hsnd)
    #junctionCheck(lcol,rcol)
'''
'''
    #if ball collected,
    dropcount = 0
    tank.on()
    if DOUBLEBLACK:
        tank.on(-100)
        linetrack

    if ssnd #box:
        break
        #ball drop (????????/)
        dropcount += 1
    
     linetrack(kconf.e(lcol[1]-rcol[1]), speed) #not sure!
        if lineturn(steer,speed):
         tank.on(-100)
         turnno += 1
         if turnno == 1
        #start time

         if turnno == 2:
             #end time
             break
             tank.on(-100)
    zoneCollect()

for dropcount < 3:
    zoneTrack
    zoneSpiral
    ballCollect #help
    linetrack(kconf.e(lcol[1]-rcol[1]), speed)
    if is_col(rcol,yellow) and is_col(lcol,yellow):
        turn90
        if checkDouble(lcol,rcol) == DOUBLEBLACK:
            linetrack(kconf.e(lcol[1]-rcol[1]), speed)
        else:
            turn180
            linetrack(kconf.e(lcol[1]-rcol[1]), speed)






# def FindMax():
#     #linetrack(lcol, rcol, sslow, knorm)

#     lcollist1.append(lcol[1])
#     rcollist1.append(rcol[1])
#     lcollist0.append(lcol[0])
#     rcollist0.append(rcol[0])
#     lcollist2.append(lcol[2])
#     rcollist2.append(rcol[2])

#     if len(lcollist1) >=5:
#         global lcolscale 
#         global rcolscale 

#         lcolmax1 = max(lcollist1)
#         lcolmin1 = min(lcollist1)
#         rcolmax1 = max(rcollist1)
#         rcolmin1 = min(rcollist1)
#         lcolmax0 = max(lcollist0)
#         lcolmin0 = min(lcollist0)
#         rcolmax0 = max(rcollist0)
#         rcolmin0 = min(rcollist0)
#         lcolmax2 = max(lcollist2)
#         lcolmin2 = min(lcollist2)
#         rcolmax2 = max(rcollist2)
#         rcolmin2 = min(rcollist2)

#         print("lcolmax1:  "+str(lcolmax1))
#         print("lcolmin1:  "+str(lcolmin1))
#         print("rcolmax1:  "+str(rcolmax1))
#         print("rcolmin1:  "+str(rcolmin1))
#         print("lcolmax0:  "+str(lcolmax0))
#         print("lcolmin0:  "+str(lcolmin0))
#         print("rcolmax0:  "+str(rcolmax0))
#         print("rcolmin0:  "+str(rcolmin0))
#         print("lcolmax2:  "+str(lcolmax2))
#         print("lcolmin2:  "+str(lcolmin2))
#         print("rcolmax2:  "+str(rcolmax2))
#         print("rcolmin2:  "+str(rcolmin2))



print("hello!")
while True: #line tracking mechanism
    lcol = leftColor.hls
    rcol = rightColor.hls
    hcol = hornColor.hls
    cube = 0
    red = 0
    if hcol[1] > floorColor[1]:
            if is_col(hcol,blueCube):
             cubeCollect(hcol)
            elif cube > 1 and squares >= 4:
             obstacleDodge()
    elif is_col(rcol,rred) or is_col(lcol,lred):
        zoneCollect()
        red += 1
    elif is_col(rcol,rred) and is_col(lcol,lred) and red >= 1:
        tank.off()
    else:
        linetrack(lcol,rcol,snorm,knorm)
        junctionCheck(lcol,rcol)
    
    
    # tank.on(0,30)
    # ballCollect(hcol)
    # linetrack(lcol,rcol,snorm,knorm)
    
    
    # ballCollect(hcol)
    # junctionCheck(lcol,rcol)
    # linetrack(lcol,rcol,snorm,knorm) 
    
    # print(hcol)
    # if hcol[1] > floorColor[1]:
    #     if is_col(hcol,blueCube):
    #          ballCollect(hcol)
    #     else:
    #         obstacleDodge()
    # else:
    #     linetrack(lcol,rcol,snorm,knorm)
    # linetrack(lcol,rcol,snorm,knorm)
    # junctionCheck(lcol,rcol)
    # linetrack(lcol,rcol,snorm,knorm) 
    # print("hcol:" +str(hcol))
    # if hcol[1] > floorColor[1]:
    #     if is_col(hcol,blueCube):
    #         ballCollect(hcol)
    #     else:
    #         obstacleDodge()
    # else:
    #     linetrack(lcol,rcol,snorm,knorm)
    
    # tank.on(0,30)
    # if is_col(hcol,blackbox):
    #         print("evacuation point found")
    #         tank.on_for_seconds(0,-50,1)
    #         tank.on_for_degrees(30,-50,1560)
    #         tank.on_for_seconds(30,-50,1.5)
    # topp = 0
    # if topp == 0 and is_col(lcol,lblue) or is_col(rcol,rblue):
    #         tank.on_for_seconds(0,-20,0.5)
    #         tank.on_for_degrees(50,30,760)
    #         tank.on_for_seconds(0,-20,0.5)
    #         topp =+1
    # else:
    #     junctionCheck(lcol,rcol)
    #     linetrack(lcol,rcol,snorm,knorm) 
    
    
    # linetrack(lcol,rcol,snorm,knorm)
    # obstacleDodge()
    # zoneDetection(lcol,rcol)
    # print(hcol)
    # junctionCheck(lcol,rcol)
    # linetrack(lcol,rcol,30,knorm)
    # zoneDetection(lcol,rcol)
    # tank.on(0,20)
    # ballCollect(hcol)
    # linetrack(lcol,rcol,snorm,knorm)
    # print("hblue: "+str(hcol))
    # print("rblue: "+str(rcol))
    # linetrack(lcol,rcol,snorm,knorm)
    # zoneDetection(lcol,rcol)
    # tank.on_for_degrees(0,ssweep,toDeg(180))

    # tank.on(30,50)
    # tank.on_for_degrees(-50,30,750)

    # junctionCheck(lcol,rcol)
    # linetrack(lcol,rcol,snorm,knorm)  

        
    # junctionCheck(lcolper,rcolper)
    # print("leftcolor: " + str(lcolper))
    # print("rightcolor: " + str(rcolper))
    # print("lcolper:   "+str(lcolper)+"rcolper:  "+str(rcolper))
    
    # junctionCheck(lcolper,rcolper)
    # linetrackper(lcolper,rcolper,snorm,knormscale)
   
    # if is_col(rcolper,rgreenscale):
    #     print("right green")
    #     sound.beep()
   
    if is_col(lcolper,lblackscale):
        
        print("left black")

    if is_col(rcolper,rblackscale):
    #     print("right black")

    
    #junctionCheck(lcolper,rcolper)
    
    # linetrackper(lcolper,rcolper,snorm,knormscale)
    # if is_col(rcolper,rgreenscale): 
    #     print("right green detected")
    #     sound.beep()


    # junctionCheck(lcolper,rcolper)
    # linetrackper(lcolper,rcolper,snorm,knormscale)
    
    
    # if checkDouble(lcolper,rcolper) == DOUBLEWHITE: 
    # # junctionFlag = False
    #   print ("double white")

    

    # lg = is_col(lcolper,lgreenscale)
    # rg = is_col(rcolper,rgreenscale)
    # if lg or rg:
    #     while checkDouble(lcolper,rcolper) != DOUBLEBLACK:
    #         linetrackper(lcolper,rcolper,sslow,kslow)
    #         if is_col(lcolper,lgreenscale): lg = True
    #         if is_col(rcolper,rgreenscale): rg = True

    #     if lg and rg: 
    #         print("doublegreen detected")
    #         uturn(sturn,lcol)
    #     elif lg:
    #         print("left green detected")
    #         lineturn(-100,sturn)   
    #     elif rg:
    #         print("right green detected")
    #         lineturn(100,sturn)

    # junctionCheck(lcolper,rcolper)
    # linetrackper(lcolper,rcolper,snorm,knormscale)

    
    
    

    # if is_col(rcolper,rgreenscale): print("right green detected")
    
    # rescale(lcol,rcol)
    # junctionCheck(lcolper,rcolper)
    # linetrackper(lcolper,rcolper,snorm,knormscale)

    # rescale(lcol,rcol)
    # print("lyellowscale: "+str(lyellowscale))
    # print("ryellowscale: "+str(ryellowscale))
   
'''
