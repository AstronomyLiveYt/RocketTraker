from tkinter import *
from tkinter import filedialog
from tkinter import ttk
import tkinter as tk
import ephem
import math
import os
import socket
import cv2
import numpy as np
import pandas as pd
import sys
import time
import datetime
import re
import json
import serial
import io
import random
import threading
import win32com.client
import imutils
import pygame
import traceback
import launchlistdownloader
import flightclubdownloader
from PIL import Image as PILImage, ImageTk
from urllib.request import urlopen
import rocketplanetarium

# Serialize OpenCV use across threads to avoid PyEval_RestoreThread: NULL tstate crash
_cv2_lock = threading.Lock()

class trackSettings:
    
    objectfollow = False
    telescopetype = 'LX200'
    mounttype = 'AltAz'
    tracking = False
    boxSize = 50
    mousecoords = (320,240)
    mapmousecoords = (0,0)
    degorhours = 'Degrees'
    mainviewX = 320
    mainviewY = 240
    crosshairX = 320
    crosshairY = 240
    maglimit = 5
    setcenter = False
    imagescale = 1.0
    trajFile = ''
    fileSelected = False
    Lat = 0.0
    Lon = 0.0
    joytracking = False
    trackingtype = 'Features'
    minbright = 50
    clickpixel = 0
    maxpixel = 255
    flip = 'NoFlip'
    foundtarget = False
    runningsimulation = False
    runninglaunch = False
    feedingdata = False
    rotate = 0
    objectverticalpixels = 0
    objecthorizontalpixels = 0
    previousrecord = 0
    joystickIP = ""
    joystickconnected = False
    buttonpushed = False
    calibratestart = False
    calspeed = 0.1
    gomanual = False
    holdratebutton = 0
    starttrackbutton = 1
    gomanualbutton = 7
    interruptbutton = 4
    focuserbutton1 = 8
    focuserbutton2 = 9
    spiralbutton = 10
    lastspiralbutton = 0
    focuserCOM = 10
    focuserconnected = False
    trackingmode = 'Regular'
    horizonaltitude = 0.0
    spiralRadius = 0.2
    spiralSearch = False
    predictionLock = False
    predictionlockbutton = 11
    lastpredictionLockbutton = 0
    pressure = 0.0
    temperature = 30
    launchtime = datetime.datetime.utcnow()
    aggression = 1.0
    screenshrink = 0.40
    ioptronnormalmoderesponses = ['5035','0035','0052','0033','0034','0050','0051']
    ioptronspecialmoderesponses = ['9035','8035','8052','8033','8034','8050','8051']
    ASCOMFocuser = False
    haltcompatible = False
    focuserabsolute = False
    maxfocuserstep = 1
    absolutefocuser = False
    alignedpoints = []
    renderStarMap = False
    syncTarget = ''
    syncDistance = 9999999999
    syncRA = 0
    syncDec = 0
    slewTarget = ''
    slewDistance = 0
    slewRA = 0
    slewDec = 0
    goforSlew = False
    slewCompleted = False
    cancelLaunch = False
    
class KalmanFilter:

    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

    def Estimate(self, coordX, coordY):
        ''' This function estimates the position of the object'''
        with _cv2_lock:
            measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
            self.kf.correct(measured)
            predicted = self.kf.predict()
            return predicted

class videotrak:
    
    def get_x_y(img, roibox, imageroi):
        with _cv2_lock:
            return videotrak._get_x_y_impl(img, roibox, imageroi)

    def _get_x_y_impl(img, roibox, imageroi):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if trackSettings.trackingtype == 'Features':
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(2,2))
            img = clahe.apply(img)
        #remember how big the total image and ROI are 
        origheight, origwidth = img.shape[:2]
        roiheight, roiwidth = imageroi.shape[:2]
        #Set up the end of the maximum search time
        searchend = time.time() + 0.2
        finalroidiff = float('inf')
        difflowered = False
        keepgoing = True
        #pull the latest searchx and searchy coordinates from the last known position
        searchx1 = roibox[0][0]
        searchy1 = roibox[0][1]
        if trackSettings.trackingtype == 'Features':
            while keepgoing is True:
                for ycheck in range((searchy1-15),(searchy1+15)):
                    if time.time() > searchend:
                        break
                    for xcheck in range((searchx1-15),(searchx1+15)):
                        #check and make sure the new position of the region of interest won't put us over the border of the window, correct back to the edge of border if it would, otherwise take the new roi coordinates
                        if xcheck < 0: 
                            xcheck = 0
                        elif xcheck > (origwidth - roiwidth):
                            xcheck = (origwidth - roiwidth)
                        if ycheck < 0: 
                            ycheck = 0
                        elif ycheck > (origheight - roiheight):
                            ycheck = (origheight - roiheight)
            #set up the roi to search within the original image
                        imagecomp = img[ycheck:int(ycheck+roiheight),xcheck:int(xcheck+roiwidth)]
            #subtract the reference roi from the search area and get the difference of the arrays
                        imagecompforsub = imagecomp.astype(np.int8)
                        imageroiforsub = imageroi.astype(np.int8)
                        imagediff = imagecompforsub - imageroiforsub
                        imagediff = np.absolute(imagediff)
                        imagediff = np.sum(imagediff)
                        imagediff = (imagediff/(np.sum(imageroi)))*100
            #if we dropped to a new minimum, save the new minimum diff and save the x and y coordinates we're at.  Set diff lowered flag to true
                        if imagediff < finalroidiff:
                            finalroidiff = imagediff
                            searchx2 = xcheck
                            searchy2 = ycheck
                            difflowered = True
            #check if we ran out of time
                        if time.time() > searchend:
                            break   
            #back on the keep going loop, check if the diff lowered in the last search run.  If not, we found a local minimum and don't need to keep going.  If we did, start a new search around the new location
                if difflowered is True:
                    keepgoing = True
                    difflowered = False
                else:
                    keepgoing = False
                if time.time() > searchend:
                    print('outtatime')
                    break   
            #print(finalroidiff)
            #figure out if the difference from roi is low enough to be acceptable
            if finalroidiff < 35:
                # Avoid cv2.waitKey(1) here - not needed with Tkinter and can cause
                # PyEval_RestoreThread: NULL tstate when used with other threads.
                searchx1last = searchx2
                searchy1last = searchy2
                learnimg = img[searchy1last:(searchy1last+roiheight),searchx1last:(searchx1last+roiwidth)]
                imageroi = (imageroi * 0.9) + (learnimg * 0.1)
                roibox = [(searchx1last,searchy1last), ((searchx1last+roiwidth),(searchy1last+roiheight))]
                trackSettings.foundtarget = True
            else:
                #print("Didn't find it, keep looking at last known coordinates.")
                searchx1last = roibox[0][0]
                searchy1last = roibox[0][1]
                if trackSettings.objectfollow is True and trackSettings.joytracking is True:
                    searchy1last = int(trackSettings.objectverticalpixels-(roiwidth/2))
                    searchx1last = int(trackSettings.objecthorizontalpixels-(roiwidth/2))
                    roibox = [(searchx1last,searchy1last), ((searchx1last+roiwidth),(searchy1last+roiheight))]
                trackSettings.foundtarget = False
        if trackSettings.trackingtype == 'Bright':
            blurred = cv2.GaussianBlur(img, (5, 5), 0)
            blurred = blurred[searchy1:int(searchy1+roiheight),searchx1:int(searchx1+roiwidth)]
            thresh = cv2.threshold(blurred, float(trackSettings.minbright), 255, cv2.THRESH_BINARY)[1]
            cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
            #cnts = cnts[0]
            cX = []
            cY = []
            #for c in cnts:
            M = cv2.moments(cnts[0])
            try:
                cX.append(int(M["m10"] / M["m00"]))
                cY.append(int(M["m01"] / M["m00"]))
                framestabilized = True
                trackSettings.foundtarget = True
            except:
                print('unable to track this frame')
                trackSettings.foundtarget = False
            if len(cX) > 0:
                cXdiff = (roiwidth/2) - cX[0]
                cYdiff = (roiheight/2) - cY[0]
                searchx1 = int(searchx1 -cXdiff)
                searchy1 = int(searchy1 -cYdiff)
                if searchx1 < 0:
                    searchx1 = 0
                if searchy1 < 0:
                    searchy1 = 0
            roibox = [(searchx1,searchy1), ((searchx1+roiwidth),(searchy1+roiheight))]
            imageroi = thresh.copy()
        return(roibox, imageroi)
    

class buttons:       

    def equatorial_to_horizon(self, dec, ra, lat, lst):
        hour = lst - ra
        if math.degrees(hour) < 0:
            hour = math.radians(math.degrees(hour) + 360)
        elif math.degrees(hour) > 360:
            hour = math.radians(math.degrees(hour) - 360)
        alt = math.asin(math.sin(dec)*math.sin(lat)+math.cos(dec)*math.cos(lat)*math.cos(hour))
        az = math.acos((math.sin(dec)-math.sin(lat)*math.sin(alt))/(math.cos(lat)*math.cos(alt)))
        if math.sin(hour)>0:
            az = (360 - (az * 180/math.pi))*(math.pi/180)
        az = math.radians(math.degrees(az))
        if math.degrees(az) > 360:
            az = math.radians(math.degrees(az) - 360)
        alt = math.degrees(alt)
        az = math.degrees(az)
        return(alt, az)
    
    def calc_obliquity(self, T):
        #T = number of julian centuries since 2000 Jan 1.5
        epsilon = 23.43929167 - 0.013004167 * T - 0.000000167 * T**2 + 0.000000502778 * T**3
        return(epsilon)

    def calc_nutation(self, current_time):
        #Do a test
        #B1900time = datetime.datetime(1900,1,1,00,00,00)
        #testtime = datetime.datetime(1988, 9, 1, 0,0,0)
        #time_difference = (testtime - B1900time).total_seconds()
        #T = (time_difference / (365.25 * 24 * 3600))/100
        #A =  100.002136 * T
        #L =  279.6967 + 360.0 * (A - int(A))
        #B =  5.372617 * T
        #Omega =  259.1833 - 360.0 * (B - int(B))
        #ecllonDelta = (-17.2*math.sin(math.radians(Omega)) - 1.3*math.sin(math.radians(2*L)))
        #ecloblDelta = (9.2*math.cos(math.radians(Omega)) + 0.5*math.cos(math.radians(2*L)))
        #print(ecllonDelta,ecloblDelta)
        #T = number of julian centuries since 1900 Jan 0.5
        B1900time = datetime.datetime(1900,1,1,00,00,00)
        time_difference = (current_time - B1900time).total_seconds()
        T = (time_difference / (365.25 * 24 * 3600))/100
        A =  100.002136 * T
        L =  279.6967 + 360.0 * (A - int(A))
        B =  5.372617 * T
        Omega =  259.1833 - 360.0 * (B - int(B))
        ecllonDelta = (-17.2*math.sin(math.radians(Omega)) - 1.3*math.sin(math.radians(2*L)))/3600
        ecloblDelta = (9.2*math.cos(math.radians(Omega)) + 0.5*math.cos(math.radians(2*L)))/3600
        return(ecllonDelta,ecloblDelta)
        

    def correct_aberration(self, ra, dec, current_time):
        #This corrects for both stellar aberration AND nutation using the function above, be advised if you want to use this function elsewhere
        J2000time = datetime.datetime(2000, 1, 1, 12, 0, 0)
        time_difference = (current_time - J2000time).total_seconds()
        J2000T = (time_difference / (365.25 * 24 * 3600))/100
        obliquity = self.calc_obliquity(J2000T)
        ecllonDelta,ecloblDelta = self.calc_nutation(current_time)
        obliquity +=ecloblDelta
        sun = ephem.Sun(current_time)
        sunra = sun.g_ra
        sundec = sun.g_dec
        suneq = ephem.Equatorial(sunra, sundec, epoch=current_time)
        sunlon = ephem.Ecliptic(suneq).lon
        deltaRA = (-20.5*((math.cos(math.radians(ra))*math.cos((sunlon))*math.cos(math.radians(obliquity))+math.sin(math.radians(ra))*math.sin((sunlon)))/math.cos(math.radians(dec))))/3600
        deltaDec = (-20.5*(math.cos((sunlon))*math.cos(math.radians(obliquity))*(math.tan(math.radians(obliquity))*math.cos(math.radians(dec))-math.sin(math.radians(ra))*math.sin(math.radians(dec)))+math.cos(math.radians(ra))*math.sin(math.radians(dec))*math.sin((sunlon))))/3600
        deltaRAsec = deltaRA*3600
        deltaDecsec = deltaDec*3600
        correctedRA = ra + deltaRA
        correctedDec = dec + deltaDec
        correctedStar = ephem.Equatorial(math.radians(correctedRA), math.radians(correctedDec), epoch=current_time)
        correctedStarEcl = ephem.Ecliptic(correctedStar, epoch=current_time)
        nutdeltaRa = (math.cos(math.radians(obliquity))+math.sin(math.radians(obliquity))*math.sin(math.radians(correctedRA))*math.tan(math.radians(correctedDec)))*ecllonDelta-math.cos(math.radians(correctedRA))*math.tan(math.radians(correctedDec))*ecloblDelta
        nutdeltaDec = math.cos(math.radians(correctedRA))*math.sin(math.radians(obliquity))*ecllonDelta+math.sin(math.radians(correctedRA))*ecloblDelta
        fullycorrectedRA = correctedRA + nutdeltaRa
        fullycorrectedDec = correctedDec + nutdeltaDec
        return(fullycorrectedRA, fullycorrectedDec)
    
    def __init__(self, master):
        self.master = master
        trackSettings.screen_width = root.winfo_screenwidth()
        trackSettings.screen_height = root.winfo_screenheight()
        self.starmapsize = round(trackSettings.screen_height*0.6)
        print(trackSettings.screen_width)
        print(trackSettings.screen_height)
        pygame.init()
        self.joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
        
        self.alt = 0.0
        self.az = 0.0
        self.altratelast = 0.0
        self.azratelast = 0.0
        
        #Prepare to check command rate of telescope
        self.tlastcommand = datetime.datetime.utcnow()
        
        #Prepare for remote connection
        self.HEADERSIZE = 10
        self.remotejoystick = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.full_msg = ''
        self.new_msg = True
        
        self.collect_images = False
        self._after_id = None
        self.topframe = Frame(master)
        master.winfo_toplevel().title("RocketTraker")
        self.topframe.pack(side=TOP)
        self.textframe = Frame(master)
        self.textframe.pack(side=BOTTOM)
        self.bottomframe = Frame(master)
        self.bottomframe.pack(side=BOTTOM)
        self.menu = Menu(master)
        master.config(menu=self.menu)
        
        master.bind("<Up>", self.goup)
        master.bind("<Left>", self.goleft)
        master.bind("<Down>", self.godown)
        master.bind("<Right>", self.goright)

        master.bind("<w>", self.chup)
        master.bind("<a>", self.chleft)
        master.bind("<s>", self.chdown)
        master.bind("<d>", self.chright)

        self.labelLat = Label(self.bottomframe, text='Latitude (N+)')
        self.labelLat.grid(row=5, column = 0)
        self.entryLat = Entry(self.bottomframe)
        #self.entryLat = Entry(self.bottomframe, show='*')
        self.entryLat.grid(row = 5, column = 1)
        self.labelLon = Label(self.bottomframe, text='Longitude (E+)')
        self.labelLon.grid(row=6, column = 0)
        self.entryLon = Entry(self.bottomframe)
        #self.entryLon = Entry(self.bottomframe, show='*')
        self.entryLon.grid(row = 6, column = 1)
        self.joyxrev = IntVar()
        self.joyyrev = IntVar()
        self.logaltaz = IntVar()
        self.logtelpos = IntVar()
        self.recordvideo = IntVar()
        self.usecountdown = IntVar()
        #self.labelBright = Label(self.bottomframe, text='Minimum Brightness')
        #self.labelBright.grid(row=8, column = 0)
        #self.entryBright = Entry(self.bottomframe)
        #self.entryBright.grid(row = 8, column = 1)
        
        #Load Bright Star Catalogue
        self.starcat = []
        #Catalog list will be RA, Dec, Name, Magnitude, Spectral Classification
        try:
            STAR_FOLDER = os.path.dirname(os.path.abspath(__file__))
            constellationsfile = os.path.join(STAR_FOLDER, 'constellationlines.csv')
            self.constellationdf = pd.read_csv(constellationsfile)
            starfile = os.path.join(STAR_FOLDER, 'bsc5.dat')
            J2000time = datetime.datetime(2000, 1, 1, 12, 0, 0)  # January 1st, 2000 at 12:00 UTC
            current_date = datetime.datetime.utcnow()  # Get the current date and time in UTC

            # Calculate the difference in seconds
            time_difference = (current_date - J2000time).total_seconds()

            # Convert the time difference from seconds to years (considering an average year length)
            fractional_year = time_difference / (365.25 * 24 * 3600)
            with open(starfile) as f:
                lines = [line.rstrip('\n') for line in f]
                for idx, line in enumerate(lines):
                    try:
                        rahour = (line[75:77])
                        ramin = (line[77:79])
                        rasec = (line[79:83])
                        decdeg = (line[83:86])
                        decmin = (line[86:88])
                        decsec = (line[88:90])
                        spectral = line[129:130]
                        mag = line[102:107]
                        raprop = line[148:154]
                        decprop = line[154:160]
                        name = line[7:24]
                        rahour = float(rahour)
                        ramin = float(ramin)
                        rasec = float(rasec)
                        decdeg = float(decdeg)
                        decmin = float(decmin)
                        decsec = float(decsec)
                        totalraprop = float(raprop)*fractional_year
                        totaldecprop = float(decprop)*fractional_year
                        raj2000 = ((((rasec/60)+ramin)/60)+rahour)*15
                        if decdeg > 0:
                            decj2000 = ((((decsec/60)+decmin)/60)+decdeg)
                        else:
                            decj2000 = (decdeg-(((decsec/60)+decmin)/60))
                        raproped = raj2000+(totalraprop/3600)
                        decproped = decj2000+(totaldecprop/3600)
                        starj2000 = ephem.Equatorial(math.radians(raproped), math.radians(decproped), epoch=ephem.J2000)
                        starEOD = ephem.Equatorial(starj2000, epoch=current_date)
                        #correct for nutation and aberration
                        correctedRA, correctedDec = self.correct_aberration(math.degrees(starEOD.ra), math.degrees(starEOD.dec), current_date)
                        correctedStar = ephem.Equatorial(math.radians(correctedRA), math.radians(correctedDec), epoch=current_date)
                        self.starcat.append((math.degrees(correctedStar.ra), math.degrees(correctedStar.dec),name,mag,spectral))
                    except:
                        pass
        except Exception as starerror:
            print('Unable to load bright star catalog')
            print(starerror)
        #current_date = datetime.datetime.utcnow()  # Get the current date and time in UTC
        #star = ephem.Equatorial(math.radians(self.starcat[7543][0]),math.radians(self.starcat[7543][1]),epoch=current_date)
        #print(self.starcat[7543][2],star.ra,star.dec)
        
        try:
            config = open('rocketconfig.txt', 'r')
            clines = [line.rstrip('\n') for line in config]
            trackSettings.telescopetype = str(clines[0])
            trackSettings.mainviewX = int(clines[3])
            trackSettings.mainviewY = int(clines[4])
            trackSettings.imagescale = float(clines[5])
            trackSettings.Lat = float(clines[6])
            trackSettings.Lon = float(clines[7])
            trackSettings.trackingtype = str(clines[8])
            trackSettings.minbright = float(clines[9])
            trackSettings.flip = str(clines[10])
            trackSettings.mounttype = str(clines[11])
            trackSettings.rotate = int(clines[12])
            trackSettings.joystickIP = str(clines[13])
            trackSettings.calspeed = float(clines[14])
            trackSettings.crosshairX = int(clines[15])
            trackSettings.crosshairY = int(clines[16])
            trackSettings.holdratebutton = int(clines[17])
            trackSettings.starttrackbutton = int(clines[18])
            trackSettings.gomanualbutton = int(clines[19])
            trackSettings.interruptbutton = int(clines[20])
            trackSettings.focuserbutton1 = int(clines[21])
            trackSettings.focuserbutton2 = int(clines[22])
            trackSettings.focuserCOM = int(clines[23])
            trackSettings.trackingmode = str(clines[24])
            trackSettings.horizonaltitude = float(clines[25])
            trackSettings.spiralRadius = float(clines[26])
            trackSettings.temperature = float(clines[27])
            trackSettings.pressure = float(clines[28])
            trackSettings.spiralbutton = int(clines[29])
            trackSettings.predictionlockbutton = int(clines[30])
            trackSettings.aggression = float(clines[31])
            trackSettings.screenshrink = float(clines[32])
            config.close()
        except Exception as e:
            print(e)
            print('Config file not present or corrupted.')
        
        try:
            self.pointingerrorDF = pd.read_csv('telpoints.csv')
        except Exception as e:
            print(e)
        
        try:
            #geolocation = geocoder.ip('me')
            #self.entryLat.insert(0, geolocation.latlng[0])
            #self.entryLon.insert(0, geolocation.latlng[1])
            self.entryLat.insert(0, trackSettings.Lat)
            self.entryLon.insert(0, trackSettings.Lon)
        except:
            self.entryLat.insert(0, trackSettings.Lat)
            self.entryLon.insert(0, trackSettings.Lon)
        try:
            launchlistdownloader.get_launches()
        except:
            pass
        
        
        
        launches_df = pd.read_csv('launchids.csv', sep=';', encoding="Latin-1")
        self.LAUNCHES = launches_df['name'].tolist()
        self.NET = launches_df['net'].tolist()
        self.LAUNCHIDS = launches_df['id'].tolist()
        self.droplist = StringVar(root)
        try:
            self.droplist.set(self.LAUNCHES[0])
        except:
            self.droplist.set('blank')
        
        self.numberofstages = [1]
        self.droplist2 = StringVar(root)
        try:
            self.droplist2.set(self.numberofstages[0])
        except:
            self.droplist2.set('blank')
        
        self.countdowntext = StringVar('')
        
        #self.entryBright.insert(0, trackSettings.minbright)
        self.startButton = Button(self.bottomframe, text='Start Camera', command=self.set_img_collect)
        self.startButton.grid(row=1, column = 0)
        self.startButton2 = Button(self.bottomframe, text='Camera Calibration', command=self.start_calibration)
        self.startButton2.grid(row=4, column = 0)
        self.startButton3 = Button(self.bottomframe, text='Set Center Point', command=self.set_center)
        self.startButton3.grid(row=4, column = 1)
        self.startButton4 = Button(self.bottomframe, text='Start Joystick Tracking', command=self.start_joy_track)
        self.startButton4.grid(row=8, column = 1)
        self.slewabortButton = Button(self.bottomframe, text='Abort Slew', command=self.abortSlew)
        self.slewabortButton.grid(row=7, column = 0)
        self.startButton5 = Button(self.bottomframe, text='Connect Scope', command=self.set_tracking)
        self.startButton5.grid(row=1, column = 1)
        self.startButtonCross = Button(self.bottomframe, text='Reset Crosshair', command=self.set_crosshair)
        self.startButtonCross.grid(row=7, column = 1)
        try:
            self.dropDown1 = OptionMenu(self.bottomframe, self.droplist, *self.LAUNCHES, command=self.LaunchSelect)
            self.dropDown1.grid(row=8, column = 0)
        except:
            pass

        self.stageLabel = Label(self.bottomframe, text='Countdown Timer')
        self.stageLabel.grid(row = 6, column = 2)
        self.stageLabel = Label(self.bottomframe, textvariable=self.countdowntext)
        self.stageLabel.grid(row = 7, column = 2)
        self.simulateButton = Button(self.bottomframe, text='Simulate Launch', command=self.start_launch_simulation)
        self.simulateButton.grid(row=8, column = 2)
        self.launchButton = Button(self.bottomframe, text='Arm Launch Tracking', command=self.arm_launch_tracking)
        self.launchButton.grid(row=8, column = 3)
        
        self.StartAltLabel = Label(self.bottomframe, text='Slew to Horizon Altitude')
        self.StartAltLabel.grid(row = 4, column=2)
        self.entryStartAlt = Entry(self.bottomframe)
        self.entryStartAlt.grid(row=4,column=3)
        self.StartAltButton = Button(self.bottomframe, text='Update Horizon Altitude', command=self.update_horizon_alt)
        self.StartAltButton.grid(row=5, column = 3)     
        self.entryStartAlt.insert(0, trackSettings.horizonaltitude)
        
        self.StartSpiralButton = Button(self.bottomframe, text='Start Spiral Search', command=self.start_spiral_search)
        self.StartSpiralButton.grid(row=5, column = 2)     
        
        self.NETLabel = Label(self.bottomframe, text='T0 Time')
        self.NETLabel.grid(row = 6, column = 3)
        self.entryNET = Entry(self.bottomframe)
        self.entryNET.grid(row = 7, column = 3)  
        
        self.reversexaxis = Checkbutton(self.bottomframe, text="Reverse Joystick X Axis", variable=self.joyxrev).grid(row=10, column = 1, sticky=W)
        self.reverseyaxis = Checkbutton(self.bottomframe, text="Reverse Joystick Y Axis", variable=self.joyyrev).grid(row=9, column = 1, sticky=W)
        self.logobservations = Checkbutton(self.bottomframe, text="Log Target Alt/Az", variable=self.logaltaz).grid(row=9, column = 2, sticky=W)
        self.logtelescopepos = Checkbutton(self.bottomframe, text="Log Telescope Alt/Az", variable=self.logtelpos).grid(row=9, column = 3, sticky=W)
        self.recordv = Checkbutton(self.bottomframe, text="Record Video", variable=self.recordvideo).grid(row=10, column = 2, sticky=W)
        self.countdownclockchoice = Checkbutton(self.bottomframe, text="Use Countdown Clock", variable=self.usecountdown).grid(row=10, column = 3, sticky=W)
        self.HoldLabel = Label(self.bottomframe, text='Hold Rate OFF')
        self.HoldLabel.grid(row = 3, column = 3)
        self.ComLabel = Label(self.bottomframe, text='COM Port')
        self.ComLabel.grid(row = 2, column = 0)
        self.entryCom = Entry(self.bottomframe)
        self.entryCom.grid(row = 2, column = 1)
        
        self.IPLabel = Label(self.bottomframe, text='Remote Joystick IP Address')
        self.IPLabel.grid(row = 2, column = 2)
        self.entryIP = Entry(self.bottomframe)
        self.entryIP.grid(row = 2, column = 3)
        self.entryIP.insert(0, trackSettings.joystickIP)
        
        self.remotejoyButton = Button(self.bottomframe, text='Connect Remote Joystick', command=self.connect_remote_joystick)
        self.remotejoyButton.grid(row = 3, column = 2)
        
        self.textbox = Text(self.textframe, height=4, width=100)
        self.textbox.grid(row=1, column=0)
        
        self.AggressionScaleLabel = Label(self.bottomframe, text=str('Aggression: '+str(round(trackSettings.aggression,4))))
        self.AggressionScaleLabel.grid(row = 9, column = 0)
        self.aggression_scale = ttk.Scale(self.bottomframe, from_=0, to=2, orient="horizontal", command=self.aggression_changed, length=200)
        self.aggression_scale.set(trackSettings.aggression)
        self.aggression_scale.grid(row=10, column=0, columnspan=1, padx=7, pady=7)
        
        try:
            self.entryCom.insert(0, clines[1])
        except:
            self.entryCom.insert(0, 0)
            
        self.CameraLabel = Label(self.bottomframe, text='Camera Number')
        self.CameraLabel.grid(row = 3, column = 0)
        self.entryCam = Entry(self.bottomframe)
        self.entryCam.grid(row = 3, column = 1)
        self.CalspeedLabel = Label(self.bottomframe, text='Calibration Speed \n(degree/second)')
        self.CalspeedLabel.grid(row = 1, column = 2)
        self.entryCal = Entry(self.bottomframe)
        self.entryCal.grid(row = 1, column = 3)
        try:
            self.entryCam.insert(0, clines[2])
        except:
            self.entryCam.insert(0, 0)
        self.entryCal.insert(0,trackSettings.calspeed)
        
        self.fileMenu = Menu(self.menu)
        self.menu.add_cascade(label='File', menu=self.fileMenu)
        self.fileMenu.add_command(label='Select Trajectory File...', command=self.filePicker)
        self.fileMenu.add_separator()
        self.fileMenu.add_command(label='Exit and Save Configuration', command=self.exitProg)
        
        self.telescopeMenu = Menu(self.menu)
        self.menu.add_cascade(label='Telescope Type', menu=self.telescopeMenu)
        self.telescopeMenu.add_command(label='LX200 Classic Alt/Az', command=self.setLX200AltAz)
        self.telescopeMenu.add_command(label='LX200 Classic Equatorial', command=self.setLX200Eq)
        self.telescopeMenu.add_command(label='ASCOM Alt/Az', command=self.setASCOMAltAz)
        self.telescopeMenu.add_command(label='ASCOM Equatorial', command=self.setASCOMEq)
        self.telescopeMenu.add_command(label='LX200 Autostar Alt/Az', command=self.setAutostarAltAz)
        
        self.trackingMenu = Menu(self.menu)
        self.menu.add_cascade(label='Tracking Type', menu=self.trackingMenu)
        self.trackingMenu.add_command(label='Feature Tracking', command=self.setFeatureTrack)
        self.trackingMenu.add_command(label='Brightness Tracking', command=self.setBrightTrack)
        
        self.modeMenu = Menu(self.menu)
        self.menu.add_cascade(label='Launch Tracking Mode', menu=self.modeMenu)
        self.modeMenu.add_command(label='Regular Predictive', command=self.setRegularMode)
        self.modeMenu.add_command(label='Adaptive Predictive', command=self.setAdaptiveMode)
        self.modeMenu.add_command(label='Slew to Horizon and Wait', command=self.setHorizonMode)
        
        self.imageMenu = Menu(self.menu)
        self.menu.add_cascade(label='Image Orientation', menu=self.imageMenu)
        self.imageMenu.add_command(label='Normal Orientation', command=self.setNoFlip)
        self.imageMenu.add_command(label='Vertical Flip', command=self.setVerticalFlip)
        self.imageMenu.add_command(label='Horizontal Flip', command=self.setHorizontalFlip)
        self.imageMenu.add_command(label='Vertical and Horizontal Flip', command=self.setVerticalHorizontalFlip)
        self.imageMenu.add_command(label='Rotate Image 0 Degrees', command=self.set0Rotate)
        self.imageMenu.add_command(label='Rotate Image 90 Degrees', command=self.setPos90Rotate)
        self.imageMenu.add_command(label='Rotate Image -90 Degrees', command=self.setNeg90Rotate)
        self.imageMenu.add_command(label='Rotate Image 180 Degrees', command=self.set180Rotate)
        
        self.mapMenu = Menu(self.menu)
        self.menu.add_cascade(label='Map', menu=self.mapMenu)
        self.mapMenu.add_command(label='All Sky Map', command=self.starMap)
        self.mapMenu.add_command(label='Pointing Error List', command=self.correctionList)
        
        self.optionsMenu = Menu(self.menu)
        self.menu.add_cascade(label='Options', menu=self.optionsMenu)
        self.optionsMenu.add_command(label='Configure Joystick Bindings', command=self.rebindJoystick)
        self.optionsMenu.add_command(label='Configure Spiral Search', command=self.configSpiral)
        self.optionsMenu.add_command(label="Configure Focuser", command=self.configFocuser)
        self.optionsMenu.add_command(label="Refraction", command=self.configRefraction)        
        self.optionsMenu.add_command(label="Window Scaling", command=self.configShrinkfactor) 
        
        #self.optionsMenu = Menu(self.menu)
        #self.menu.add_cascade(label='Joystick', menu=self.optionsMenu)

        
        #self.focuserMenu = Menu(self.menu)
        #self.menu.add_cascade(label='Focuser', menu=self.focuserMenu)
        
        
        #Initialize T0 time in entry box
        indexnumber = self.LAUNCHES.index(self.droplist.get())
        self.entryNET.delete(0, 'end')
        self.entryNET.insert(0, self.NET[indexnumber]) 
    
    def configShrinkfactor(self):
        self.shrinkwindow= Toplevel(root)
        self.shrinkwindow.geometry("350x100")
        self.shrinkwindow.title("Window Scaling Configuration")
        self.shrinkLabel = Label(self.shrinkwindow, text=str('Image Based Window Scaling Factor: '+str(round(trackSettings.screenshrink,4))),font=('Arial 13'))
        self.shrinkLabel.grid(row = 0, column = 0)
        self.shrink_scale = ttk.Scale(self.shrinkwindow, from_=0.01, to=1, orient="horizontal", command=self.shrink_changed, length=200)
        self.shrink_scale.set(trackSettings.screenshrink)
        self.shrink_scale.grid(row=1, column=0, columnspan=1, padx=7, pady=7)
    
    def shrink_changed(self, event):
        trackSettings.screenshrink = self.shrink_scale.get()
        self.shrinkLabel.config(text=str('Image Based Window Scaling Factor: '+str(round(trackSettings.screenshrink,4))))
    
    def aggression_changed(self, event):
        trackSettings.aggression = self.aggression_scale.get()
        self.AggressionScaleLabel.config(text=str('Aggression: '+str(round(trackSettings.aggression,4))))
    
    def configRefraction(self):
        self.refractionwindow= Toplevel(root)
        self.refractionwindow.geometry("550x100")
        self.refractionwindow.title("Atmospheric Refraction Correction for Predictive Tracking")
        self.pressureLabel = Label(self.refractionwindow, text='Atmospheric Pressure in millibars (set to 0 to disable refraction): ',font=('Arial 13'))
        self.pressureLabel.grid(row = 0, column = 0)
        self.entryPressure = Entry(self.refractionwindow)
        self.entryPressure.grid(row = 0, column = 1)
        self.entryPressure.insert(0, trackSettings.pressure)
        self.temperatureLabel = Label(self.refractionwindow, text='Temperature in Celsius: ',font=('Arial 13'))
        self.temperatureLabel.grid(row = 1, column = 0)
        self.entryTemperature = Entry(self.refractionwindow)
        self.entryTemperature.grid(row = 1, column = 1)
        self.entryTemperature.insert(0, trackSettings.temperature)
        self.refractionapplyButton = Button(self.refractionwindow, text=str('Apply'),command = self.refractionApply,font=('Arial 13'))
        self.refractionapplyButton.grid(row = 2, column = 0)
    
    def refractionApply(self):
        trackSettings.temperature = float(self.entryTemperature.get())
        trackSettings.pressure = float(self.entryPressure.get())
    
    def configSpiral(self):
        self.spiralwindow= Toplevel(root)
        self.spiralwindow.geometry("550x100")
        self.spiralwindow.title("Spiral Search Configuration")
        self.spiralLabel = Label(self.spiralwindow, text='Spiral Radius (set equal to FOV diameter in degrees): ',font=('Arial 13'))
        self.spiralLabel.grid(row = 0, column = 0)
        self.entrySpiral = Entry(self.spiralwindow)
        self.entrySpiral.grid(row = 0, column = 1)
        self.entrySpiral.insert(0, trackSettings.spiralRadius)
        self.spiralapplyButton = Button(self.spiralwindow, text=str('Apply'),command = self.spiralApply,font=('Arial 13'))
        self.spiralapplyButton.grid(row = 1, column = 0)
    
    def spiralApply(self):
        trackSettings.spiralRadius = float(self.entrySpiral.get())
        
    def update_horizon_alt(self):
        try:
            trackSettings.horizonaltitude = float(self.entryStartAlt.get())
            self.horizonalt = -999
        except Exception as E:
            print(E)
    
    def setRegularMode(self):
        trackSettings.trackingmode = 'Regular'
        
    def setAdaptiveMode(self):
        trackSettings.trackingmode = 'Adaptive'
        
    def setHorizonMode(self):
        trackSettings.trackingmode = 'Horizon'

    def stepApply(self):
        trackSettings.maxfocuserstep = float(self.entryFocuserStep.get())
        
    def configFocuser(self):
        self.focuserwindow= Toplevel(root)
        self.focuserwindow.geometry("500x300")
        self.focuserwindow.title("Focuser Configuration")
        self.commandsLabel = Label(self.focuserwindow, text="Astro's Focuser Serial Port: ",font=('Arial 10'))
        self.commandsLabel.grid(row = 3, column = 0)
        self.entryFocuserStep = Entry(self.focuserwindow)
        self.entryFocuserStep.grid(row = 0, column = 1)
        self.entryFocuserStep.insert(0, trackSettings.maxfocuserstep)
        self.stepLabel = Label(self.focuserwindow, text="ASCOM Focuser Step Size: ",font=('Arial 10'))
        self.stepLabel.grid(row = 0, column = 0)
        self.entryFocuserCom = Entry(self.focuserwindow)
        self.entryFocuserCom.grid(row = 3, column = 1)
        self.stepapplyButton = Button(self.focuserwindow, text=str('Apply Focuser Step'),command = self.stepApply,font=('Arial 13'))
        self.stepapplyButton.grid(row = 0, column = 2)
        self.separator = ttk.Separator(self.focuserwindow, orient='horizontal')
        self.separator.grid(row=2,columnspan=3,ipadx=0,ipady=10)
        self.entryFocuserCom.insert(0, trackSettings.focuserCOM)
        if trackSettings.focuserconnected is False:
            self.focuserconnectButton = Button(self.focuserwindow, text=str("Connect Astro's Focuser"),command = self.connectFocuser,font=('Arial 8'))
        if trackSettings.focuserconnected is True:
            self.focuserconnectButton = Button(self.focuserwindow, text=str('Disconnect'),command = self.connectFocuser,font=('Arial 10'))
        self.focuserconnectButton.grid(row = 3, column = 2)
        if trackSettings.ASCOMFocuser is False:
            self.ASCOMFocuserconnectButton = Button(self.focuserwindow, text=str('Connect ASCOM Focuser'),command = self.connectASCOMFocuser,font=('Arial 12'))
        if trackSettings.ASCOMFocuser is True:
            self.ASCOMFocuserconnectButton = Button(self.focuserwindow, text=str('Disconnect ASCOM Focuser'),command = self.connectASCOMFocuser,font=('Arial 12'))
        self.ASCOMFocuserconnectButton.grid(row = 1, column = 0)
        
    
    def connectASCOMFocuser(self):
        if trackSettings.ASCOMFocuser is True:
            self.ASCOMFocuser.Connected = False
            self.ASCOMFocuserconnectButton.configure(text='Connect ASCOM Focuser')
            trackSettings.ASCOMFocuser = False
        else:
            self.fx = win32com.client.Dispatch("ASCOM.Utilities.Chooser")
            self.fx.DeviceType = 'Focuser'
            driverName=self.fx.Choose("None")
            self.ASCOMFocuser=win32com.client.Dispatch(driverName)
            self.ASCOMFocuser.Connected = True
            self.ASCOMFocuserconnectButton.configure(text='Disconnect ASCOM Focuser')
            trackSettings.ASCOMFocuser = True
            try:
                trackSettings.absolutefocuser = self.ASCOMFocuser.Absolute
            except Exception as e:
                print(e)
            try:
                self.ASCOMFocuser.Halt()
                trackSettings.haltcompatible = True 
            except Exception as e:
                print(e)
                trackSettings.haltcompatible = False
            try:
                trackSettings.focuserabsolute = self.ASCOMFocuser.Absolute
            except Exception as e:
                print(e)
            try:
                trackSettings.maxfocuserstep = self.ASCOMFocuser.MaxIncrement
            except Exception as e:
                print(e)
                trackSettings.maxfocuserstep = self.ASCOMFocuser.MaxStep
            self.entryFocuserStep.delete(0, tk.END)
            self.entryFocuserStep.insert(0, trackSettings.maxfocuserstep)

    def connectFocuser(self):
        if trackSettings.focuserconnected is False:
            trackSettings.focuserCOM = self.entryFocuserCom.get()
            self.focusercomport = str('COM'+str(self.entryFocuserCom.get()))
            self.focuserSerial = serial.Serial(self.focusercomport, baudrate=9600, timeout=500)
            time.sleep(3)
            #self.focuserSerial.write(str.encode(':U#'))
            trackSettings.focuserconnected = True
            self.focuserconnectButton.configure(text='Disconnect')
        else:
            self.focuserSerial.close()
            time.sleep(1)
            trackSettings.focuserconnected = False
            self.focuserconnectButton.configure(text='Connect')
            
    def rebindJoystick(self):
        self.joywindow= Toplevel(root)
        self.joywindow.geometry("300x300")
        self.joywindow.title("Joystick Configuration")
        self.commandsLabel = Label(self.joywindow, text='Command',font=('Arial 14 bold'))
        self.commandsLabel.grid(row = 0, column = 0)
        self.bindingsLabel = Label(self.joywindow, text='Binding',font=('Arial 14 bold'))
        self.bindingsLabel.grid(row = 0, column = 1)
        self.pushthebuttonLabel = Label(self.joywindow, text='Manually Start Launch Tracking: ')
        self.pushthebuttonLabel.grid(row = 1, column = 0)
        self.pushbuttonConfig = Button(self.joywindow, text=str('Joystick Button '+str(trackSettings.starttrackbutton)),command = self.bindpushbutton)
        self.pushbuttonConfig.grid(row = 1, column = 1)
        self.holdratebuttonLabel = Label(self.joywindow, text='Hold Current Tracking Rate: ')
        self.holdratebuttonLabel.grid(row = 2, column = 0)
        self.holdratebuttonConfig = Button(self.joywindow, text=str('Joystick Button '+str(trackSettings.holdratebutton)),command = self.bindholdratebutton)
        self.holdratebuttonConfig.grid(row = 2, column = 1)
        self.manualbuttonLabel = Label(self.joywindow, text='Revert to Full Manual Tracking: ')
        self.manualbuttonLabel.grid(row = 3, column = 0)
        self.manualbuttonConfig = Button(self.joywindow, text=str('Joystick Button '+str(trackSettings.gomanualbutton)),command = self.bindmanualbutton)
        self.manualbuttonConfig.grid(row = 3, column = 1)
        self.focuseroutLabel = Label(self.joywindow, text='Focuser Out: ')
        self.focuseroutLabel.grid(row = 4, column = 0)
        self.focuseroutConfig = Button(self.joywindow, text=str('Joystick Button '+str(trackSettings.focuserbutton1)),command = self.bindfocuserbutton1)
        self.focuseroutConfig.grid(row = 4, column = 1)
        self.focuserinLabel = Label(self.joywindow, text='Focuser In: ')
        self.focuserinLabel.grid(row = 5, column = 0)
        self.focuserinConfig = Button(self.joywindow, text=str('Joystick Button '+str(trackSettings.focuserbutton2)),command = self.bindfocuserbutton2)
        self.focuserinConfig.grid(row = 5, column = 1)
        self.spiralbuttonlabel = Label(self.joywindow, text='Spiral Search: ')
        self.spiralbuttonlabel.grid(row = 6, column = 0)
        self.spiralbuttonConfig = Button(self.joywindow, text=str('Joystick Button '+str(trackSettings.spiralbutton)),command = self.bindspiralbutton)
        self.spiralbuttonConfig.grid(row = 6, column = 1)
        self.predictionbuttonlabel = Label(self.joywindow, text='Prediction Lock: ')
        self.predictionbuttonlabel.grid(row = 7, column = 0)
        self.predictionbuttonConfig = Button(self.joywindow, text=str('Joystick Button '+str(trackSettings.predictionlockbutton)),command = self.bindpredictionbutton)
        self.predictionbuttonConfig.grid(row = 7, column = 1)
        
    def bindfocuserbutton1(self):
        if len(self.joysticks) == 0:
            print('Connect Joystick And Restart Program!')
            self.textbox.insert(END, 'Connect Joystick And Restart Program!\n')
            self.textbox.see('end')
        else:
            self.joysticks[0].init()
            buttoncount = self.joysticks[0].get_numbuttons()
            self.pushabuttonwindow= Toplevel(self.joywindow)
            self.pushabuttonwindow.geometry("250x150")
            self.pushabuttonwindow.title("Listening")
            self.pushabuttonLabel = Label(self.pushabuttonwindow, text='Push a Joystick Button')
            self.pushabuttonLabel.grid(row = 0, column = 0)
            buttonconfigured = False
            while buttonconfigured is False:
                pygame.event.pump()
                for b in range(0,buttoncount):
                    p = self.joysticks[0].get_button(b)
                    if p > 0:
                        print(b)
                        trackSettings.focuserbutton1 = b
                        self.focuseroutConfig.configure(text=str('Joystick Button '+str(trackSettings.focuserbutton1)))
                        buttonconfigured = True
                        break
                time.sleep(0.1)
            self.pushabuttonwindow.destroy()

    def bindfocuserbutton2(self):
        if len(self.joysticks) == 0:
            print('Connect Joystick And Restart Program!')
            self.textbox.insert(END, 'Connect Joystick And Restart Program!\n')
            self.textbox.see('end')
        else:
            self.joysticks[0].init()
            buttoncount = self.joysticks[0].get_numbuttons()
            self.pushabuttonwindow= Toplevel(self.joywindow)
            self.pushabuttonwindow.geometry("250x150")
            self.pushabuttonwindow.title("Listening")
            self.pushabuttonLabel = Label(self.pushabuttonwindow, text='Push a Joystick Button')
            self.pushabuttonLabel.grid(row = 0, column = 0)
            buttonconfigured = False
            while buttonconfigured is False:
                pygame.event.pump()
                for b in range(0,buttoncount):
                    p = self.joysticks[0].get_button(b)
                    if p > 0:
                        print(b)
                        trackSettings.focuserbutton2 = b
                        self.focuserinConfig.configure(text=str('Joystick Button '+str(trackSettings.focuserbutton2)))
                        buttonconfigured = True
                        break
                time.sleep(0.1)
            self.pushabuttonwindow.destroy()
            
    def bindpushbutton(self):
        if len(self.joysticks) == 0:
            print('Connect Joystick And Restart Program!')
            self.textbox.insert(END, 'Connect Joystick And Restart Program!\n')
            self.textbox.see('end')
        else:
            self.joysticks[0].init()
            buttoncount = self.joysticks[0].get_numbuttons()
            self.pushabuttonwindow= Toplevel(self.joywindow)
            self.pushabuttonwindow.geometry("250x150")
            self.pushabuttonwindow.title("Listening")
            self.pushabuttonLabel = Label(self.pushabuttonwindow, text='Push a Joystick Button')
            self.pushabuttonLabel.grid(row = 0, column = 0)
            buttonconfigured = False
            while buttonconfigured is False:
                pygame.event.pump()
                for b in range(0,buttoncount):
                    p = self.joysticks[0].get_button(b)
                    if p > 0:
                        print(b)
                        trackSettings.starttrackbutton = b
                        self.pushbuttonConfig.configure(text=str('Joystick Button '+str(trackSettings.starttrackbutton)))
                        buttonconfigured = True
                        break
                time.sleep(0.1)
            self.pushabuttonwindow.destroy()

    def bindholdratebutton(self):
        if len(self.joysticks) == 0:
            print('Connect Joystick And Restart Program!')
            self.textbox.insert(END, 'Connect Joystick And Restart Program!\n')
            self.textbox.see('end')
        else:
            self.joysticks[0].init()
            buttoncount = self.joysticks[0].get_numbuttons()
            self.pushabuttonwindow= Toplevel(self.joywindow)
            self.pushabuttonwindow.geometry("250x150")
            self.pushabuttonwindow.title("Listening")
            self.pushabuttonLabel = Label(self.pushabuttonwindow, text='Push a Joystick Button')
            self.pushabuttonLabel.grid(row = 0, column = 0)
            buttonconfigured = False
            while buttonconfigured is False:
                pygame.event.pump()
                for b in range(0,buttoncount):
                    p = self.joysticks[0].get_button(b)
                    if p > 0:
                        print(b)
                        trackSettings.holdratebutton = b
                        self.holdratebuttonConfig.configure(text=str('Joystick Button '+str(trackSettings.holdratebutton)))
                        buttonconfigured = True
                        break
                time.sleep(0.1)
            self.pushabuttonwindow.destroy()
            
    def bindmanualbutton(self):
        if len(self.joysticks) == 0:
            print('Connect Joystick And Restart Program!')
            self.textbox.insert(END, 'Connect Joystick And Restart Program!\n')
            self.textbox.see('end')
        else:
            self.joysticks[0].init()
            buttoncount = self.joysticks[0].get_numbuttons()
            self.pushabuttonwindow= Toplevel(self.joywindow)
            self.pushabuttonwindow.geometry("250x150")
            self.pushabuttonwindow.title("Listening")
            self.pushabuttonLabel = Label(self.pushabuttonwindow, text='Push a Joystick Button')
            self.pushabuttonLabel.grid(row = 0, column = 0)
            buttonconfigured = False
            while buttonconfigured is False:
                pygame.event.pump()
                for b in range(0,buttoncount):
                    p = self.joysticks[0].get_button(b)
                    if p > 0:
                        print(b)
                        trackSettings.gomanualbutton = b
                        self.manualbuttonConfig.configure(text=str('Joystick Button '+str(trackSettings.gomanualbutton)))
                        buttonconfigured = True
                        break
                time.sleep(0.1)
            self.pushabuttonwindow.destroy()

    def bindspiralbutton(self):
        if len(self.joysticks) == 0:
            print('Connect Joystick And Restart Program!')
            self.textbox.insert(END, 'Connect Joystick And Restart Program!\n')
            self.textbox.see('end')
        else:
            self.joysticks[0].init()
            buttoncount = self.joysticks[0].get_numbuttons()
            self.pushabuttonwindow= Toplevel(self.joywindow)
            self.pushabuttonwindow.geometry("250x150")
            self.pushabuttonwindow.title("Listening")
            self.pushabuttonLabel = Label(self.pushabuttonwindow, text='Push a Joystick Button')
            self.pushabuttonLabel.grid(row = 0, column = 0)
            buttonconfigured = False
            while buttonconfigured is False:
                pygame.event.pump()
                for b in range(0,buttoncount):
                    p = self.joysticks[0].get_button(b)
                    if p > 0:
                        print(b)
                        trackSettings.spiralbutton = b
                        self.spiralbuttonConfig.configure(text=str('Joystick Button '+str(trackSettings.spiralbutton)))
                        buttonconfigured = True
                        break
                time.sleep(0.1)
            self.pushabuttonwindow.destroy()
            
    def bindpredictionbutton(self):
        if len(self.joysticks) == 0:
            print('Connect Joystick And Restart Program!')
            self.textbox.insert(END, 'Connect Joystick And Restart Program!\n')
            self.textbox.see('end')
        else:
            self.joysticks[0].init()
            buttoncount = self.joysticks[0].get_numbuttons()
            self.pushabuttonwindow= Toplevel(self.joywindow)
            self.pushabuttonwindow.geometry("250x150")
            self.pushabuttonwindow.title("Listening")
            self.pushabuttonLabel = Label(self.pushabuttonwindow, text='Push a Joystick Button')
            self.pushabuttonLabel.grid(row = 0, column = 0)
            buttonconfigured = False
            while buttonconfigured is False:
                pygame.event.pump()
                for b in range(0,buttoncount):
                    p = self.joysticks[0].get_button(b)
                    if p > 0:
                        print(b)
                        trackSettings.predictionlockbutton = b
                        self.predictionbuttonConfig.configure(text=str('Joystick Button '+str(trackSettings.predictionlockbutton)))
                        buttonconfigured = True
                        break
                time.sleep(0.1)
            self.pushabuttonwindow.destroy()

    def set_crosshair(self):
        trackSettings.crosshairX = trackSettings.mainviewX
        trackSettings.crosshairY = trackSettings.mainviewY
        
    def atm_refraction(self, altitude):
        if altitude>15:
            refraction = 0.00452*trackSettings.pressure*math.tan(math.radians((90-altitude)/(273+trackSettings.temperature)))
        else:
            refraction = (trackSettings.pressure*(0.1594+(0.0196*altitude)+(0.00002*altitude**2)))/((273+trackSettings.temperature)*(1+(0.505*altitude)+(0.0845*altitude**2)))
        return(refraction)
        
    def filePicker(self):
        trackSettings.trajFile = filedialog.askopenfilename(initialdir = ".",title = "Select Trajectory file",filetypes = (("csv files","*.csv"),("all files","*.*")))
        trackSettings.fileSelected = True
        print(trackSettings.trajFile)
        self.textbox.insert(END, str(str(trackSettings.trajFile)+'\n'))
        self.textbox.see('end')
        constnow = datetime.datetime.utcnow()
        #Get the T0 time to project the path at launch time
        t0 = self.entryNET.get()
        t0 = datetime.datetime.strptime(t0, '%Y-%m-%dT%H:%M:%SZ')
        launchobserver = ephem.Observer()
        launchobserver.lat = (str(self.entryLat.get()))
        launchobserver.lon = (str(self.entryLon.get()))
        launchobserver.date = t0
        launchobserver.pressure = trackSettings.pressure
        launchobserver.temp = trackSettings.temperature
        launchobserver.epoch = launchobserver.date
        mtelalt = -10
        mtelaz = 0
        rendersize = self.starmapsize
        rocketpath = []
        df = pd.read_csv(trackSettings.trajFile, sep=',', encoding="utf-8")
        startaltfound = False
        for index, row in df.iterrows():
            #Find the row that it rises on horizon and slew there
            rowalt = float(row['elevationDegs'])
            refraction = self.atm_refraction(rowalt)
            rowalt = rowalt+refraction
            if rowalt > trackSettings.horizonaltitude:
                startaltfound = True
            if startaltfound is True:                
                thisrowalt = float(row['elevationDegs'])
                horizonaz = float(row['azimuthDegs'])
                rocketpath.append((thisrowalt,horizonaz))
        planetariumimage, staraltazlist = rocketplanetarium.render_sky(self.starcat,self.constellationdf,constnow,launchobserver,mtelalt,mtelaz,rocketpath,trackSettings.alignedpoints,rendersize,trackSettings.maglimit)
        cv2.imshow('Rocket Trajectory at Liftoff',planetariumimage)
        cv2.waitKey(1)
    
    def errorWeightedAverage(self, ref_tel_alt, ref_tel_az):
        if len(self.pointingerrorDF) > 0:
            alt_weights = []
            az_weights = []
            alt_errors = []
            az_errors = []
            for index, row in self.pointingerrorDF.iterrows():
                sep = ephem.separation((math.radians(row['tel az']),math.radians(row['tel alt'])),(math.radians(ref_tel_az),math.radians(ref_tel_alt)))
                sep = math.degrees(sep)
                #Use the square of the separation for weights
                alt_weight = 1 / (sep + 1e-6)**2
                az_weight = 1 / (sep + 1e-6)**2
                alt_weights.append(alt_weight)
                az_weights.append(az_weight)
            for index, row in self.pointingerrorDF.iterrows():
                thisalterror = row['alt sep']*(alt_weights[index]/sum(alt_weights))
                alt_errors.append(thisalterror)
                thisazerror = row['az sep']*(az_weights[index]/sum(az_weights))
                az_errors.append(thisazerror)
            weighted_avg_alt_sep = sum(alt_errors)
            weighted_avg_az_sep = sum(az_errors)
        else:
            weighted_avg_alt_sep = 0.0
            weighted_avg_az_sep = 0.0
            
        # Return results
        return(weighted_avg_alt_sep, weighted_avg_az_sep)
    
    def syncNow(self):
        if self.tel.Connected is True:
            telalt = self.tel.Altitude
            telaz = self.tel.Azimuth
            constnow = datetime.datetime.utcnow()
            launchobserver = ephem.Observer()
            launchobserver.lat = (str(self.entryLat.get()))
            launchobserver.lon = (str(self.entryLon.get()))
            launchobserver.date = constnow
            launchobserver.pressure = trackSettings.pressure
            launchobserver.temp = trackSettings.temperature
            launchobserver.epoch = launchobserver.date
            starDec = math.radians(trackSettings.syncDec)
            starRA = math.radians(trackSettings.syncRA)
            staralt, staraz = self.equatorial_to_horizon(starDec, starRA, launchobserver.lat, launchobserver.sidereal_time())
            refraction = self.atm_refraction(staralt)
            staralt = staralt+refraction
            altdelta = staralt - telalt
            azdelta = staraz - telaz
            errordict = {'tel alt':telalt,'tel az':telaz,'alt sep':altdelta,'az sep':azdelta}
            self.pointingerrorDF = pd.concat([self.pointingerrorDF, pd.DataFrame([errordict])], ignore_index=True, sort=False)
            print(self.pointingerrorDF)
            self.pointingerrorDF.to_csv('telpoints.csv',index=False)
    
    def boresightSyncPopUp(self):
        self.syncwindow= Toplevel(root)
        self.syncwindow.geometry('200x100')
        self.syncwindow.title("Sync On Star?")
        trackSettings.syncTarget = ''
        trackSettings.syncDistance = 0
        constnow = datetime.datetime.utcnow()
        launchobserver = ephem.Observer()
        launchobserver.lat = (str(self.entryLat.get()))
        launchobserver.lon = (str(self.entryLon.get()))
        launchobserver.date = constnow
        launchobserver.pressure = trackSettings.pressure
        launchobserver.temp = trackSettings.temperature
        launchobserver.epoch = launchobserver.date
        if self.tel.Connected is True:
            #mtelra = self.tel.RightAscension*15
            #mteldec = self.tel.Declination
            telalt = self.tel.Altitude
            telaz = self.tel.Azimuth
            ref_tel_alt = telalt
            ref_tel_az = telaz
            weighted_avg_alt_sep, weighted_avg_az_sep = self.errorWeightedAverage(ref_tel_alt, ref_tel_az)
            telaz = ref_tel_az + weighted_avg_az_sep
            telalt = ref_tel_alt + weighted_avg_alt_sep
            print(telalt, telaz)
            lowestsep = 180
            for star in self.starcat:
                staralt, staraz = self.equatorial_to_horizon(math.radians(star[1]), math.radians(star[0]), launchobserver.lat, launchobserver.sidereal_time())
                refraction = self.atm_refraction(staralt)
                staralt = staralt+refraction
                currentsep = ephem.separation((math.radians(telaz),math.radians(telalt)),(math.radians(staraz),math.radians(staralt)))
                if math.degrees(currentsep)<lowestsep:
                    lowestsep = math.degrees(currentsep)
                    trackSettings.syncTarget = star[2]
                    trackSettings.syncDistance = round((lowestsep*3600),1)
                    trackSettings.syncRA = star[0]
                    trackSettings.syncDec = star[1]
        else:
            trackSettings.syncTarget='Nothing Because You Need To Connect Telescope First'
        self.synctargetLabel = Label(self.syncwindow, text=str('Sync Target: '+str(trackSettings.syncTarget)))
        self.synctargetLabel.grid(row = 0, column = 0)
        self.syncsepLabel = Label(self.syncwindow, text=str('Sync Distance Arcseconds: '+str(trackSettings.syncDistance)))
        self.syncsepLabel.grid(row = 1, column = 0)
        self.syncButton = Button(self.syncwindow, text=str('Sync!'),command = self.syncNow)
        self.syncButton.grid(row = 2, column = 0)
        
    
    def render_starMap(self):
        while trackSettings.renderStarMap is True:
            constnow = datetime.datetime.utcnow()
            launchobserver = ephem.Observer()
            launchobserver.lat = (str(self.entryLat.get()))
            launchobserver.lon = (str(self.entryLon.get()))
            launchobserver.date = constnow
            launchobserver.pressure = trackSettings.pressure
            launchobserver.temp = trackSettings.temperature
            launchobserver.epoch = launchobserver.date
            rendersize = self.starmapsize
            #Remember you need a rocketpath list even if it's empty
            rocketpath = []
            if trackSettings.fileSelected is True:
                t0 = self.entryNET.get()
                t0 = datetime.datetime.strptime(t0, '%Y-%m-%dT%H:%M:%SZ')
                #launchobserver.date = t0
                #launchobserver.epoch = launchobserver.date
                df = pd.read_csv(trackSettings.trajFile, sep=',', encoding="utf-8")
                startaltfound = False
                for index, row in df.iterrows():
                    #Find the row that it rises on horizon and slew there
                    rowalt = float(row['elevationDegs'])
                    refraction = self.atm_refraction(rowalt)
                    rowalt = rowalt+refraction
                    if rowalt > trackSettings.horizonaltitude:
                        startaltfound = True
                    if startaltfound is True:                
                        thisrowalt = float(row['elevationDegs'])
                        horizonaz = float(row['azimuthDegs'])
                        rocketpath.append((thisrowalt,horizonaz))
            mtelalt = -10
            mtelaz = 0
            try:
                if self.tel.Connected is True:
                    mtelaz = self.tel.Azimuth
                    mtelalt = self.tel.Altitude
                    #Lines to test show the weighted average error
                    #weighted_avg_alt_sep, weighted_avg_az_sep = self.errorWeightedAverage(mtelalt, mtelaz)
                    #currentaz = mtelaz - weighted_avg_az_sep
                    #currentalt = mtelalt - weighted_avg_alt_sep
                    #print(currentalt, currentaz, weighted_avg_alt_sep, weighted_avg_az_sep)
            except:
                pass
            try:
                mag = float(self.entrymaglimit.get())
                trackSettings.maglimit = mag
            except:
                pass
            planetariumimage,self.staraltazlist = rocketplanetarium.render_sky(self.starcat,self.constellationdf,constnow,launchobserver,mtelalt,mtelaz,rocketpath,trackSettings.alignedpoints,rendersize, trackSettings.maglimit)
            b,g,r = cv2.split(planetariumimage)
            tkimg = cv2.merge((r,g,b))
            tkimg = PILImage.fromarray(tkimg)
            tkimg = ImageTk.PhotoImage(image=tkimg)
            displayimg = Label(self.starmaptopframe, bg="black")
            displayimg.config(image=tkimg)
            displayimg.img = tkimg
            displayimg.grid(row = 0, column = 0)
            displayimg.bind("<Button-1>", self.map_left_click)
            displayimg.bind("<Motion>", self.map_mouse_position)
            time.sleep(1)          

    def map_mouse_position(self, event):
        trackSettings.mapmousecoords = (event.x, event.y)

    def map_left_click(self, event):
        #if trackSettings.setcenter is True:
        rendersize = self.starmapsize
        halfrendersize = rendersize/2
        mapX = trackSettings.mapmousecoords[0]
        mapY = trackSettings.mapmousecoords[1]
        centerX = halfrendersize
        centerY = halfrendersize
        zenithpix = math.sqrt((centerX - mapX)**2+(centerY - mapY)**2)
        if zenithpix == 0:
            alt = 90
            az = 0
        else:
            zenithdeg = zenithpix/(halfrendersize/90)
            alt = 90-zenithdeg
            azX = mapX - halfrendersize
            azY = halfrendersize - mapY
            az = math.degrees(math.atan2(azY,azX))-90
            if az < 0:
                az+=360
        print(alt, az)
        lowestsep = 180
        name = ''
        mag = 20
        staralt = 0
        staraz = 0
        if len(self.staraltazlist)>0:
            for star in self.staraltazlist:
                originalalt = star[0]
                originalaz = star[1]
                if star[1] < 0:
                    thisaz=star[1]+360
                else:
                    thisaz=star[1]
                currentsep = ephem.separation((math.radians(thisaz),math.radians(star[0])),(math.radians(az),math.radians(alt)))
                if math.degrees(currentsep)<lowestsep:
                    lowestorigalt = originalalt
                    lowestorigaz = originalaz
                    lowestsep = math.degrees(currentsep)
                    name = star[2]
                    mag = star[3]
                    staralt = star[0]
                    staraz = thisaz
            for star in self.starcat:
                if star[2] == name and star[3] == mag:
                    RAstar = star[0]
                    Decstar = star[1]
            #Found the star that was clicked on, now bring up options window!
            print(name, lowestsep, staralt, staraz)
            self.starSlewPopUp(name, mag, RAstar, Decstar)
        #x1 = int((zenith*math.cos(az1))*(halfrendersize/90)+halfrendersize)
        #y1 = int((zenith*math.sin(az1))*(halfrendersize/90)+halfrendersize)
    
    def starSlewPopUp(self, starname, starmag, starRA, starDec):
        #starname = starname[:-4]
        constnow = datetime.datetime.utcnow()
        launchobserver = ephem.Observer()
        launchobserver.lat = (str(self.entryLat.get()))
        launchobserver.lon = (str(self.entryLon.get()))
        launchobserver.date = constnow
        launchobserver.pressure = trackSettings.pressure
        launchobserver.temp = trackSettings.temperature
        launchobserver.epoch = launchobserver.date
        starDecrad = math.radians(starDec)
        starRArad = math.radians(starRA)
        staralt, staraz = self.equatorial_to_horizon(starDecrad, starRArad, launchobserver.lat, launchobserver.sidereal_time())
        staralt = round(staralt, 2)
        staraz = round(staraz, 2)
        self.slewstarwindow= Toplevel(root)
        self.slewstarwindow.geometry('200x220')
        self.slewstarwindow.title("Slew to Star?")
        trackSettings.slewTarget = ''
        trackSettings.slewDistance = 0
        try:
            mtelra = self.tel.RightAscension*15
            mteldec = self.tel.Declination
            #print(mtelra, mteldec)
            currentsep = ephem.separation((math.radians(mtelra),math.radians(mteldec)),(math.radians(starRA),math.radians(starDec)))
            currentsep = math.degrees(currentsep)
            trackSettings.syncDistance = round((currentsep*3600),1)
            currentsep = round(currentsep,2)
            trackSettings.slewRA = starRA
            trackSettings.slewDec = starDec
            trackSettings.syncRA = starRA
            trackSettings.syncDec = starDec
            self.slewtargetLabel = Label(self.slewstarwindow, text=str('Slew Target: '+str(starname)))
            self.slewtargetLabel.grid(row = 0, column = 0)
            self.slewaltLabel = Label(self.slewstarwindow, text=str('Target Altitude: '+str(staralt)))
            self.slewaltLabel.grid(row = 1, column = 0)
            self.slewazLabel = Label(self.slewstarwindow, text=str('Target Azimuth: '+str(staraz)))
            self.slewazLabel.grid(row=2, column =0)
            self.slewsepLabel = Label(self.slewstarwindow, text=str('Target Magnitude: '+str(starmag)))
            self.slewsepLabel.grid(row = 3, column = 0)
            self.slewButton = Button(self.slewstarwindow, text=str('Slew!'),command = self.slewtoStar)
            self.slewButton.grid(row = 4, column = 0)
            self.abortslewButton = Button(self.slewstarwindow, text=str('Abort Slew!'),command = self.abortSlew)
            self.abortslewButton.grid(row = 5, column = 0)
            self.slewsepLabel = Label(self.slewstarwindow, text=str('Slew Distance (degrees): '+str(currentsep)))
            self.slewsepLabel.grid(row = 6, column = 0)
            self.clicksyncsepLabel = Label(self.slewstarwindow, text=str('Sync Distance (arcseconds): '+str(trackSettings.syncDistance)))
            self.clicksyncsepLabel.grid(row = 7, column = 0)
            self.clicksyncButton = Button(self.slewstarwindow, text=str('Sync on '+str(starname)),command = self.syncNow)
            self.clicksyncButton.grid(row = 8, column = 0)
        except:
            trackSettings.slewTarget='Unable To Slew Without'
            targetline2 = 'A Connected ASCOM Telescope'
            self.slewtargetLabel = Label(self.slewstarwindow, text=str(trackSettings.slewTarget))
            self.slewtargetLabel.grid(row = 0, column = 0)
            self.slewaltLabel = Label(self.slewstarwindow, text=str(targetline2))
            self.slewaltLabel.grid(row = 1, column = 0)
    
    def ASCOMSlew(self):
        trackSettings.slewCompleted = False
        if trackSettings.goforSlew is True:
            trackSettings.slewCompleted = False
            self.tel.SlewToCoordinates((math.degrees(self.raslew)/15),math.degrees(self.decslew))
            trackSettings.goforSlew = False
            trackSettings.slewCompleted = True
            time.sleep(1)
            trackSettings.slewCompleted = False
    
    def abortSlew(self):
        if self.tel.Connected is True:
            self.tel.AbortSlew()
            trackSettings.slewCompleted = True
            time.sleep(1)
            trackSettings.slewCompleted = False
    
    def slewtoStar(self):
        if self.tel.Connected is True:
            constnow = datetime.datetime.utcnow()
            launchobserver = ephem.Observer()
            launchobserver.lat = (str(self.entryLat.get()))
            launchobserver.lon = (str(self.entryLon.get()))
            launchobserver.date = constnow
            launchobserver.pressure = trackSettings.pressure
            launchobserver.temp = trackSettings.temperature
            launchobserver.epoch = launchobserver.date
            starDec = math.radians(trackSettings.slewDec)
            starRA = math.radians(trackSettings.slewRA)
            staralt, staraz = self.equatorial_to_horizon(starDec, starRA, launchobserver.lat, launchobserver.sidereal_time())
            ref_tel_alt = staralt
            ref_tel_az = staraz
            weighted_avg_alt_sep, weighted_avg_az_sep = self.errorWeightedAverage(ref_tel_alt, ref_tel_az)
            #We're correcting the star alt/az, not the telescope, so subtract the delta
            correctedaz = ref_tel_az - weighted_avg_az_sep
            correctedalt = ref_tel_alt - weighted_avg_alt_sep
            refraction = self.atm_refraction(correctedalt)
            correctedalt += refraction
            launchobserver.pressure = 0.0
            raslew, decslew = launchobserver.radec_of(math.radians(correctedaz), math.radians(correctedalt))
            print(weighted_avg_alt_sep, weighted_avg_az_sep, staralt, staraz, correctedalt, correctedaz)
            self.raslew = raslew
            self.decslew = decslew
            trackSettings.goforSlew = True
            self.ASCOMSlewThread = threading.Thread(target=self.ASCOMSlew, daemon=True)
            self.ASCOMSlewThread.start()
            #self.tel.SlewToCoordinates((math.degrees(raslew)/15),math.degrees(decslew))
            
        else:
            print('You Need To Connect Telescope First')
            
    
    def setStarMapRender(self):
        if trackSettings.renderStarMap is False:
            trackSettings.renderStarMap = True
            self.renderMapButton.configure(text='Pause All Sky View')
            self.renderMapThread = threading.Thread(target=self.render_starMap, daemon=True)
            self.renderMapThread.start()
        else:
            self.renderMapButton.configure(text='Render All Sky View')
            trackSettings.renderStarMap = False
    
    def starMap(self):
        self.starmapwindow= Toplevel(root)
        mapwindowsize = int(int(self.starmapsize)+100)
        self.starmapwindow.geometry(str(str(mapwindowsize)+"x"+str(mapwindowsize)))
        self.starmapwindow.title("Planetarium Window")
        self.starmaptopframe = Frame(self.starmapwindow)
        self.starmaptopframe.pack(side=TOP)
        self.starmapbottomframe = Frame(self.starmapwindow)
        self.starmapbottomframe.pack(side=BOTTOM)
        self.renderMapButton = Button(self.starmapbottomframe, text='Render All Sky View', command=self.setStarMapRender)
        self.renderMapButton.grid(row=0, column = 1)
        self.maglimitlabel = Label(self.starmapbottomframe, text='Magnitude Limit')
        self.maglimitlabel.grid(row = 1, column = 0)
        self.entrymaglimit = Entry(self.starmapbottomframe)
        self.entrymaglimit.grid(row = 1, column = 1)
        self.entrymaglimit.insert(0, trackSettings.maglimit)
        self.boresightsyncButton = Button(self.starmapbottomframe, text='Sync on Nearest Star', command=self.boresightSyncPopUp)
        self.boresightsyncButton.grid(row=2, column = 1)

    def correctionList(self):
        self.correctionListWindow = Toplevel(root)
        self.correctionListWindow.geometry("800x400")
        self.correctionListWindow.title("Pointing Correction List")
        # Create a frame to hold the treeview and the delete buttons
        main_frame = tk.Frame(self.correctionListWindow)
        main_frame.pack(fill=tk.BOTH, expand=True)    
        # Create a treeview widget
        self.tree = ttk.Treeview(main_frame, columns=("alt sep", "az sep", "tel alt", "tel az"), show='headings', height=15)
        self.tree.heading("alt sep", text="Altitude Error")
        self.tree.heading("az sep", text="Azimuth Error")
        self.tree.heading("tel alt", text="Telescope Altitude")
        self.tree.heading("tel az", text="Telescope Azimuth")
        self.tree.grid(row=0, column=0, sticky='nsew')

        # Configure treeview column sizes
        self.tree.column("alt sep", width=150)
        self.tree.column("az sep", width=150)
        self.tree.column("tel alt", width=150)
        self.tree.column("tel az", width=150)

        # Add scrollbars
        scrollbar = ttk.Scrollbar(main_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscroll=scrollbar.set)
        scrollbar.grid(row=0, column=1, sticky='ns')

        # Frame for delete buttons
        self.button_frame = tk.Frame(main_frame)
        self.button_frame.grid(row=0, column=2, sticky='ns')

        # Populate the treeview with rows from the dataframe
        self.tree_data = {}
        for i, row in self.pointingerrorDF.iterrows():
            self.insert_row(i, row)

        # Make rows expandable
        main_frame.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
    
    def insert_row(self, index, row):
        #Inserts a row into the treeview with a delete button.

        tree_id = self.tree.insert('', 'end', values=(row['alt sep'], row['az sep'], row['tel alt'], row['tel az']))
        self.tree_data[tree_id] = index

        # Add delete button
        delete_button = tk.Button(self.button_frame, text="Delete", command=lambda tid=tree_id: self.delete_row(tid))
        delete_button.grid(row=len(self.tree_data) - 1, column=0, sticky='ew')

    def delete_row(self, tree_id):
        #Deletes the selected row from the treeview and updates the dataframe.
        index = self.tree_data[tree_id]
        self.pointingerrorDF = self.pointingerrorDF.drop(index).reset_index(drop=True)
        self.pointingerrorDF.to_csv('telpoints.csv',index=False)
        self.refresh_tree()

    def refresh_tree(self):
        #Refreshes the treeview and delete buttons after a row is deleted.
        # Clear the treeview
        for item in self.tree.get_children():
            self.tree.delete(item)

        # Clear the button frame
        for widget in self.button_frame.winfo_children():
            widget.destroy()

        # Reset tree_data and repopulate
        self.tree_data = {}
        for i, row in self.pointingerrorDF.iterrows():
            self.insert_row(i, row)
        
    def LaunchSelect(self, event):
        indexnumber = self.LAUNCHES.index(self.droplist.get())
        self.entryNET.delete(0, 'end')
        self.entryNET.insert(0, self.NET[indexnumber]) 
    
    def connect_remote_joystick(self):
        if trackSettings.joystickconnected is False:
            try:
                self.remotejoystick = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.host = socket.gethostbyname(self.entryIP.get())
                self.remotejoystick.connect((self.host, 3933))
                trackSettings.joystickconnected = True
                self.get_remote_joy_data = threading.Thread(target=self.get_joy, daemon=True)
                self.get_remote_joy_data.start()
                self.remotejoyButton.configure(text="Remote Joystick Connected")
            except Exception as e:
                print(e)
        else:
            trackSettings.joystickconnected = False
            self.remotejoystick.close()
            self.new_msg = True
            self.full_msg = ""
            self.remotejoyButton.configure(text="Connect Remote Joystick")
            
            
    def get_joy(self):
        while trackSettings.joystickconnected is True:
            try:
                self.msg = self.remotejoystick.recv(100)
                if self.new_msg:
                    #print("new msg len:",msg[:HEADERSIZE])
                    self.msglen = int(self.msg[:self.HEADERSIZE])
                    self.new_msg = False

                #print(f"full message length: {msglen}")
                self.full_msg += self.msg.decode("utf-8")
                if len(self.full_msg)-self.HEADERSIZE == self.msglen:
                    #print(self.full_msg)
                    #print("full msg recvd")
                    self.joymessage = str(self.full_msg[self.HEADERSIZE:]).split(' ')
                    self.remotejoy0 = float(self.joymessage[0])
                    self.remotejoy1 = float(self.joymessage[1])
                    self.remotethrottle = float(self.joymessage[2])
                    self.remotejoybutton1 = int(self.joymessage[3])
                    self.remotejoybutton2 = int(self.joymessage[4])
                    self.remotejoybutton3 = int(self.joymessage[5])
                    self.remotejoybutton4 = int(self.joymessage[6])
                    self.remotejoybutton5 = int(self.joymessage[7])
                    self.new_msg = True
                    self.full_msg = ""
                    
                if len(self.full_msg)-self.HEADERSIZE > self.msglen:
                    #print(self.full_msg)
                    #print("OVERFULL msg received")
                    #print(len(full_msg),msglen,full_msg)
                    self.nextmsg = str(self.full_msg[self.HEADERSIZE:]).split('#')[-1]
                    self.currentmsg = str(self.full_msg[self.HEADERSIZE:]).split('#')[0]
                    self.joymessage = str(self.currentmsg).split(' ')
                    #print(joymessage)
                    self.remotejoy0 = float(self.joymessage[0])
                    self.remotejoy1 = float(self.joymessage[1])
                    self.remotethrottle = float(self.joymessage[2])
                    self.remotejoybutton1 = int(self.joymessage[3])
                    self.remotejoybutton2 = int(self.joymessage[4])
                    self.remotejoybutton3 = int(self.joymessage[5])
                    self.remotejoybutton4 = int(self.joymessage[6])
                    self.remotejoybutton5 = int(self.joymessage[7])
                    #print(self.remotejoy0,self.remotejoy1,self.remotethrottle)
                    #new_msg = True
                    self.full_msg = self.nextmsg
                time.sleep(0.01)
            except Exception as e:
                print(e)
                self.remotejoystick.close()
                self.remotejoystick = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.host = socket.gethostbyname(self.entryIP.get())
                self.remotejoystick.connect((self.host, 3933))
                trackSettings.joystickconnected = True
                self.get_remote_joy_data = threading.Thread(target=self.get_joy, daemon=True)
                self.get_remote_joy_data.start()
                
    def chup(self, event):
        trackSettings.crosshairY -= 1
    
    def chdown(self, event):
        trackSettings.crosshairY += 1
        
    def chleft(self, event):
        trackSettings.crosshairX -= 1
    
    def chright(self, event):
        trackSettings.crosshairX +=1
    
    def setNoFlip(self):
        trackSettings.flip = 'NoFlip'
    
    def setVerticalFlip(self):
        trackSettings.flip = 'VerticalFlip'    
    
    def setHorizontalFlip(self):
        trackSettings.flip = 'HorizontalFlip'
        
    def setVerticalHorizontalFlip(self):
        trackSettings.flip = 'VerticalHorizontalFlip'
        
    def set0Rotate(self):
        trackSettings.rotate = 0
    
    def setPos90Rotate(self):
        trackSettings.rotate = 90
        
    def setNeg90Rotate(self):
        trackSettings.rotate = -90
        
    def set180Rotate(self):
        trackSettings.rotate = 180
        
    def exitProg(self):
        if getattr(self, '_after_id', None) is not None:
            try:
                self.master.after_cancel(self._after_id)
            except Exception:
                pass
            self._after_id = None
        config = open('rocketconfig.txt','w')
        config.write(str(trackSettings.telescopetype)+'\n')
        config.write(str(self.entryCom.get()) + '\n')
        config.write(str(self.entryCam.get()) + '\n')
        config.write(str(trackSettings.mainviewX) + '\n')
        config.write(str(trackSettings.mainviewY) + '\n')
        config.write(str(trackSettings.imagescale) + '\n')
        config.write(str(self.entryLat.get())+'\n')
        config.write(str(self.entryLon.get())+'\n')
        config.write(str(trackSettings.trackingtype) + '\n')
        config.write(str(trackSettings.minbright)+'\n')
        config.write(str(trackSettings.flip)+'\n')
        config.write(str(trackSettings.mounttype)+'\n')
        config.write(str(trackSettings.rotate)+'\n')
        config.write(str(self.entryIP.get())+'\n')
        config.write(str(self.entryCal.get())+'\n')
        config.write(str(trackSettings.crosshairX) + '\n')
        config.write(str(trackSettings.crosshairY) + '\n')
        config.write(str(trackSettings.holdratebutton) + '\n')
        config.write(str(trackSettings.starttrackbutton) + '\n')
        config.write(str(trackSettings.gomanualbutton) + '\n')
        config.write(str(trackSettings.interruptbutton) + '\n')
        config.write(str(trackSettings.focuserbutton1) + '\n')
        config.write(str(trackSettings.focuserbutton2) + '\n')
        config.write(str(trackSettings.focuserCOM) + '\n')
        config.write(str(trackSettings.trackingmode) + '\n')
        config.write(str(trackSettings.horizonaltitude) + '\n')
        config.write(str(trackSettings.spiralRadius) + '\n')
        config.write(str(trackSettings.temperature) + '\n')
        config.write(str(trackSettings.pressure) + '\n')
        config.write(str(trackSettings.spiralbutton) + '\n')
        config.write(str(trackSettings.predictionlockbutton) + '\n')
        config.write(str(trackSettings.aggression) + '\n')
        config.write(str(trackSettings.screenshrink) + '\n')
        config.close()
        if getattr(self, 'cap', None) is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        cv2.destroyAllWindows()
        if getattr(self, 'recordvideo', None) and self.recordvideo.get() == 1:
            self.out.release()
        self.master.quit()
        self.master.destroy()
        sys.exit(0)
    
    def start_spiral_search(self):
        if trackSettings.spiralSearch is False:
            self.currentspiraldegree = 0
            self.currentspiralradius = trackSettings.spiralRadius/3
            trackSettings.spiralSearch = True
            trackSettings.predictionLock = False
            self.StartSpiralButton.configure(text='Stop Spiral Search')
        elif trackSettings.spiralSearch is True:
            trackSettings.spiralSearch = False
            self.StartSpiralButton.configure(text='Start Spiral Search')
            
    def start_prediction_lock(self):
        if trackSettings.predictionLock is False:
            trackSettings.spiralSearch = False
            self.StartSpiralButton.configure(text='Prediction Lock ON')
            self.lastpredictiontime = datetime.datetime.utcnow()
            self.azdifflist = []
            self.altdifflist = []
            trackSettings.predictionLock = True
        elif trackSettings.predictionLock is True:
            trackSettings.predictionLock = False
            self.StartSpiralButton.configure(text='Start Spiral Search')
    
    def start_launch_simulation(self):
        if trackSettings.runningsimulation is False and trackSettings.runninglaunch is False:
            trackSettings.runningsimulation = True
            if len(self.joysticks) == 0 and trackSettings.joystickconnected is False:
                print('Connect Joystick And Restart Program!')
                self.textbox.insert(END, 'Connect Joystick And Restart Program!\n')
                self.textbox.see('end')
                trackSettings.runningsimulation = False
            else:
                if trackSettings.joystickconnected is False:
                    self.joysticks[0].init()
                self.simcalc = threading.Thread(target=self.run_sim, daemon=True)
                self.simcalc.start()
                self.simthread = threading.Thread(target=self.simulate_launch, daemon=True)
                self.simulateButton.configure(text='Stop Simulation')
                self.simthread.start()
            
        else:
            trackSettings.runningsimulation = False
            trackSettings.feedingdata = False
            self.simulateButton.configure(text='Simulate Launch')

    def arm_launch_tracking(self):
        if trackSettings.runningsimulation is False and trackSettings.runninglaunch is False and trackSettings.fileSelected is True:
            trackSettings.cancelLaunch = False
            trackSettings.runninglaunch = True
            if len(self.joysticks) == 0 and trackSettings.joystickconnected is False:
                print('Connect Joystick And Restart Program!')
                self.textbox.insert(END, 'Connect Joystick And Restart Program!\n')
                self.textbox.see('end')
                trackSettings.runninglaunch = False
            else:
                if trackSettings.joystickconnected is False:
                    self.joysticks[0].init()
                self.simcalc = threading.Thread(target=self.simulate_launch, daemon=True)
                self.simcalc.start()
                self.launchthread = threading.Thread(target=self.run_launch, daemon=True)
                self.launchButton.configure(text='Disarm Launch Tracking')
                self.launchthread.start()
        elif trackSettings.fileSelected is False:
            print('Load the launch trajectory first!')
            self.textbox.insert(END, 'Load the launch trajectory first!\n')
            self.textbox.see('end')
            
        else:
            trackSettings.runninglaunch = False
            trackSettings.buttonpushed = False
            trackSettings.feedingdata = False
            trackSettings.cancelLaunch = True
            self.launchButton.configure(text='Arm Launch Tracking')
        
    def run_launch(self):
        self.t0 = self.entryNET.get()
        self.t0 = datetime.datetime.strptime(self.t0, '%Y-%m-%dT%H:%M:%SZ')
        currenttime = datetime.datetime.utcnow()
        countdown = (self.t0 - currenttime).total_seconds()
        self.horizon_rise_time = None
        # Calculate horizon rise time before countdown so it can be displayed during countdown
        if trackSettings.trackingmode == 'Horizon' and trackSettings.fileSelected:
            try:
                df = pd.read_csv(trackSettings.trajFile, sep=',', encoding="utf-8")
                for index, row in df.iterrows():
                    rowalt = float(row['elevationDegs'])
                    refraction = self.atm_refraction(rowalt)
                    rowalt = rowalt + refraction
                    if rowalt > trackSettings.horizonaltitude:
                        waittime = row['time']
                        # Use t0 as launch time for countdown display
                        self.horizon_rise_time = self.t0 + datetime.timedelta(seconds=waittime)
                        break
            except Exception as e:
                self.horizon_rise_time = None
        while trackSettings.buttonpushed is False and trackSettings.runninglaunch is True:
            currenttime = datetime.datetime.utcnow()
            countdown = (self.t0 - currenttime).total_seconds()
            hours = countdown // 3600
            minutes = (countdown % 3600) // 60
            seconds = countdown % 60
            # Before launch always show t- (time to launch). "Rising in" only shows after t=0 in the Horizon waiting loop.
            self.countdowntext.set(str('t- '+str(math.trunc(hours))+' hours '+str(math.trunc(minutes))+' minutes '+str(math.trunc(seconds))+' seconds'))

            time.sleep(0.01)
            self.altrateout = 0.0 
            self.azrateout = 0.0
            if self.usecountdown.get() == 1 and countdown <=0:
                #Disable joystick tracking if it was still running
                if trackSettings.joytracking is True:
                    trackSettings.joytracking = False
                    self.startButton4.configure(text='Start Joystick Tracking')
                trackSettings.buttonpushed = True
            try:
                if self.remotejoybutton2 > 0 and self.usecountdown.get() == 0:
                    #Disable joystick tracking if it was still running
                    if trackSettings.joytracking is True:
                        trackSettings.joytracking = False
                        self.startButton4.configure(text='Start Joystick Tracking')
                    trackSettings.buttonpushed = True
            except:
                pass
            if trackSettings.joystickconnected is False:
                pygame.event.pump()
                if self.joysticks[0].get_button(trackSettings.starttrackbutton) > 0 and self.usecountdown.get() == 0:
                    #Disable joystick tracking if it was still running
                    if trackSettings.joytracking is True:
                        trackSettings.joytracking = False
                        self.startButton4.configure(text='Start Joystick Tracking')
                    trackSettings.buttonpushed = True
            else:
                try:
                    if self.remotejoybutton2 > 0 and self.usecountdown.get() == 0:
                        #Disable joystick tracking if it was still running
                        if trackSettings.joytracking is True:
                            trackSettings.joytracking = False
                            self.startButton4.configure(text='Start Joystick Tracking')
                        trackSettings.buttonpushed = True
                except:
                    pass
        #Use button push time UNLESS you're using the computer clock, then get the entered t0 time instead
        if self.usecountdown.get() == 0:
            initialtime = datetime.datetime.utcnow()
        else:
            #We're using the computer clock, so check t0 time one more time from text entry box and use that, but if it's been corrupted use the original t0 time
            try:
                finalt0 = self.entryNET.get()
                finalt0 = datetime.datetime.strptime(self.t0, '%Y-%m-%dT%H:%M:%SZ')
            except:
                finalt0 = self.t0
            initialtime = finalt0
        trackSettings.launchtime = initialtime
        endoffile = False
        timedeltalast = 0
        ref_tel_alt = self.tel.Altitude
        ref_tel_az = self.tel.Azimuth
        weighted_avg_alt_sep, weighted_avg_az_sep = self.errorWeightedAverage(ref_tel_alt, ref_tel_az)
        self.lastaz = ref_tel_az + weighted_avg_az_sep
        self.lastalt = ref_tel_alt + weighted_avg_alt_sep
        ####Do this if we're set to wait at the horizon mode###
        if trackSettings.trackingmode == 'Horizon':
            startgoingtime = initialtime  # default if no horizon crossing is found
            if trackSettings.fileSelected is True:
                df = pd.read_csv(trackSettings.trajFile, sep=',', encoding="utf-8")
                altlist1 = []
                azlist1 = []
                timelist = []
                ####################NEED TO LOCATE WHERE IT WILL HIT THE HORIZON AND WAIT THERE!
                self.horizonalt = -999
                horizonaz = -999
                waittime = 0
                for index, row in df.iterrows():
                    timelist.append(row['time'])
                    #Find the row that it rises on horizon and slew there
                    rowalt = float(row['elevationDegs'])
                    refraction = self.atm_refraction(rowalt)
                    rowalt = rowalt+refraction
                    if rowalt > trackSettings.horizonaltitude and self.horizonalt < -990 and trackSettings.cancelLaunch is False:
                        waittime = row['time']
                        startgoingtime = initialtime + datetime.timedelta(seconds=waittime)
                        # Set horizon_rise_time for display purposes
                        self.horizon_rise_time = initialtime + datetime.timedelta(seconds=waittime)
                        thisrowalt = float(row['elevationDegs'])
                        refraction = self.atm_refraction(thisrowalt)
                        self.horizonalt = thisrowalt+refraction
                        horizonaz = row['azimuthDegs']
                        #Slew to pad location
                        launchobserver = ephem.Observer()
                        launchobserver.lat = (str(self.entryLat.get()))
                        launchobserver.lon = (str(self.entryLon.get()))
                        launchobserver.date = datetime.datetime.utcnow()
                        launchobserver.pressure = 0
                        launchobserver.epoch = launchobserver.date
                        raslew, decslew = launchobserver.radec_of(math.radians(horizonaz), math.radians(self.horizonalt))
                        self.textbox.insert(END, str('Slew Alt: ' + str(round(self.horizonalt,2)) + ' Slew Az: ' + str(round(horizonaz,2)) +' Slew RA: ' + str(round((math.degrees(raslew)/15),2))+' hrs Slew Dec: ' + str(round(math.degrees(decslew),2))+'\n'))
                        self.textbox.see('end')
                        self.raslew = raslew
                        self.decslew = decslew
                        trackSettings.goforSlew = True
                        self.ASCOMSlewThread = threading.Thread(target=self.ASCOMSlew, daemon=True)
                        self.ASCOMSlewThread.start()
                        #self.tel.SlewToCoordinates((math.degrees(raslew)/15),math.degrees(decslew))
                        while trackSettings.slewCompleted is False:
                            time.sleep(0.01)
                        self.tel.MoveAxis(0, 0)
                        self.tel.MoveAxis(1, 0)
                    thisrowalt = float(row['elevationDegs'])
                    refraction = self.atm_refraction(thisrowalt)
                    thisrowalt = thisrowalt+refraction
                    altlist1.append(thisrowalt)
                    azlist1.append(row['azimuthDegs'])
                altlast = altlist1[0]
                azlast = azlist1[0]
            else:
                endoffile = True
            while trackSettings.runninglaunch is True and endoffile is False:
                time.sleep(0.001)
                r, c = df.shape
                currenttime = datetime.datetime.utcnow()
                if currenttime > startgoingtime:
                    try:
                        launchobserver.date = datetime.datetime.utcnow()
                        timedelta = (currenttime - initialtime).total_seconds()
                        onesecondlater = ((currenttime - initialtime).total_seconds()+1)
                        twosecondslater = onesecondlater+1
                        hours = timedelta // 3600
                        minutes = (timedelta % 3600) // 60
                        seconds = timedelta % 60
                        self.countdowntext.set(str('t+ '+str(math.trunc(hours))+' hours '+str(math.trunc(minutes))+' minutes '+str(math.trunc(seconds))+' seconds'))
                        timedelta2 = timedelta - timedeltalast
                        timedeltalast = timedelta
                        df_sort = df.iloc[(df['time']-timedelta).abs().argsort()[:2]]
                        df_sort = df_sort.sort_values(by=['time'])
                        #Find the row exactly 1 second later to be able to compute degrees per second
                        df_sort2 = df.iloc[(df['time']-onesecondlater).abs().argsort()[:2]]
                        df_sort2 = df_sort2.sort_values(by=['time'])
                        df_sort3 = df.iloc[(df['time']-twosecondslater).abs().argsort()[:2]]
                        df_sort3 = df_sort3.sort_values(by=['time'])
                        altlist1 = []
                        azlist1 = []
                        timelist = []
                        #Convert absolute coordinates to relative rates
                        index = 0
                        predictedalt = float(df_sort.iloc[index].loc['elevationDegs'])
                        refraction = self.atm_refraction(predictedalt)
                        predictedalt = predictedalt+refraction
                        predictedaz = float(df_sort.iloc[index].loc['azimuthDegs'])
                        nextpredictedalt = float(df_sort2.iloc[index].loc['elevationDegs'])
                        nextrefraction = self.atm_refraction(nextpredictedalt)
                        nextpredictedalt = nextpredictedalt+nextrefraction
                        thirdpredictedalt = float(df_sort3.iloc[index].loc['elevationDegs'])
                        thirdrefraction = self.atm_refraction(thirdpredictedalt)
                        thirdpredictedalt = thirdpredictedalt+thirdrefraction
                        nextpredictedaz = float(df_sort2.iloc[index].loc['azimuthDegs'])
                        thirdpredictedaz = float(df_sort3.iloc[index].loc['azimuthDegs'])
                        currentpredicttime = float(df_sort.iloc[index].loc['time'])
                        nextpredicttime = float(df_sort2.iloc[index].loc['time'])
                        thirdpredicttime = float(df_sort3.iloc[index].loc['time'])
                        #Account for possible unstable increments in time to get degrees per second
                        nexttimediff = nextpredicttime - currentpredicttime
                        thirdtimediff = thirdpredicttime - nextpredicttime
                        altrate = (nextpredictedalt - predictedalt)/nexttimediff
                        nextaltrate = (thirdpredictedalt - nextpredictedalt)/thirdtimediff
                        #Now I'm using df_sort2 as index+1 because df_sort2 actually finds the closest value to 1 second later, the next row of df_sort isn't NECESSARILY one second later
                        headingeast = ((df_sort2.iloc[index].loc['azimuthDegs'] + 360)-df_sort.iloc[index].loc['azimuthDegs'])/nexttimediff
                        headingwest = (df_sort2.iloc[index].loc['azimuthDegs'] - (df_sort.iloc[index].loc['azimuthDegs']+360))/nexttimediff
                        nextheadingeast = ((df_sort3.iloc[index].loc['azimuthDegs'] + 360)-df_sort2.iloc[index].loc['azimuthDegs'])/thirdtimediff
                        nextheadingwest = (df_sort3.iloc[index].loc['azimuthDegs'] - (df_sort2.iloc[index].loc['azimuthDegs']+360))/thirdtimediff
                        azrate = (df_sort2.iloc[index].loc['azimuthDegs'] - df_sort.iloc[index].loc['azimuthDegs'])/nexttimediff
                        nextazrate = (thirdpredictedaz - nextpredictedaz)/thirdtimediff
                        if abs(nextheadingeast) < abs(nextazrate):
                            nextazrate = nextheadingeast
                        elif abs(nextheadingwest) < abs(nextazrate):
                            nextazrate = nextheadingwest      
                        if abs(headingeast) < abs(azrate):
                            azrate = headingeast
                        elif abs(headingwest) < abs(azrate):
                            azrate = headingwest      
                        #Interpolate current alt and az rates!
                        altrate = altrate + (timedelta-currentpredicttime)*((nextaltrate-altrate)/(nextpredicttime-currentpredicttime))
                        azrate = azrate + (timedelta-currentpredicttime)*((nextazrate-azrate)/(nextpredicttime-currentpredicttime))
                        #Check if we're doing a spiral search and update the spiral vector if so#############################################
                        if trackSettings.joystickconnected is False:
                            joyspiral = self.joysticks[0].get_button(trackSettings.spiralbutton)
                        else:
                        #PLACEHOLDER UNTIL I UPDATE THE REMOTE JOY SERVER TO HAVE A SPIRAL BUTTON
                            joyspiral = 0
                        if joyspiral == 1 and trackSettings.lastspiralbutton == 0:
                            self.start_spiral_search()
                            trackSettings.lastspiralbutton = 1
                        elif joyspiral == 0 and trackSettings.lastspiralbutton == 1:
                            trackSettings.lastspiralbutton = 0
                        if trackSettings.spiralSearch is True:
                            circlecircumference = 2*math.pi*self.currentspiralradius
                            amounttorotate = (((trackSettings.spiralRadius/circlecircumference)*360)*timedelta2)
                            #amounttoextend = (trackSettings.spiralRadius/(360/amounttorotate))*timedelta2
                            self.currentspiraldegree += amounttorotate
                            if self.currentspiraldegree > 360:
                                self.currentspiraldegree -= 360
                                self.currentspiralradius += trackSettings.spiralRadius
                            altpoint = self.currentspiralradius * math.sin(math.radians(self.currentspiraldegree))
                            azpoint = (self.currentspiralradius * math.cos(math.radians(self.currentspiraldegree)))*(1/math.cos(math.radians(predictedalt)))
                            altpoint = predictedalt + altpoint
                            azpoint = predictedaz + azpoint
                            self.spiralalt, self.spiralaz = altpoint, azpoint
                            #print(predictedalt, predictedaz, altpoint, azpoint)
                            #print(self.currentspiraldegree, self.currentspiralradius)
                        #These should be the absolute coordinates but just store relative rates for now
                        alt = altrate
                        az = azrate
                        #Check to see if joystick button was pushed to switch over to manual tracking
                        pygame.event.pump()
                        if trackSettings.joystickconnected is False:
                            if self.joysticks[0].get_button(trackSettings.gomanualbutton) > 0:
                                timedelta = r+1
                        elif self.remotejoybutton3 > 0:
                            timedelta = r+1
                        #check to see if we reached the end of the interpolation, remember to account for extra 20 seconds before launch)
                        if (timedelta+2) > (r):
                            endoffile = True
                            trackSettings.runninglaunch = False
                            trackSettings.buttonpushed = False
                            trackSettings.feedingdata = False
                            self.launchButton.configure(text='Arm Launch Tracking')
                            trackSettings.joytracking = True
                            self.trackthread = threading.Thread(target=self.track, daemon=True)
                            self.startButton4.configure(text='Stop Joystick Tracking')
                            self.trackthread.start()
                        if math.isinf(altrate) or math.isinf(azrate):
                            pass
                        else:
                            #print(timedelta, alt, az, altrate, azrate)
                            self.timedeltaout, self.altout, self.azout, self.altrateout, self.azrateout, self.predictalt, self.predictaz, self.nextpredictedalt, self.nextpredictedaz, self.currentpredicttime, self.nextpredicttime, self.thirdpredictedalt, self.thirdpredictedaz, self.thirdpredicttime = timedelta, alt, az, altrate, azrate, predictedalt, predictedaz, nextpredictedalt, nextpredictedaz, currentpredicttime, nextpredicttime, thirdpredictedalt, thirdpredictedaz, thirdpredicttime
                            trackSettings.feedingdata = True
                    except:
                        pass
                else:
                    # Use horizon_rise_time if available for accurate display, otherwise fall back to startgoingtime
                    if self.horizon_rise_time is not None:
                        timedelta = (self.horizon_rise_time - currenttime).total_seconds()
                    else:
                        timedelta = (startgoingtime - currenttime).total_seconds()
                    if timedelta > 0:
                        hours = timedelta // 3600
                        minutes = (timedelta % 3600) // 60
                        seconds = timedelta % 60
                        self.countdowntext.set(str('Rising in: '+str(math.trunc(hours))+' hours '+str(math.trunc(minutes))+' minutes '+str(math.trunc(seconds))+' seconds'))
                    else:
                        # Past rise time, should have transitioned to tracking - show 0 or transition message
                        self.countdowntext.set(str('Rising in: 0 hours 0 minutes 0 seconds'))
                    if trackSettings.joystickconnected is False:
                        pygame.event.pump()
                        if self.joysticks[0].get_button(trackSettings.starttrackbutton) > 0:
                             startgoingtime = currenttime
                    else:
                        if self.remotejoybutton2 > 0:
                            startgoingtime = currenttime
                    if self.horizonalt < -990:
                        #Slew to new waiting spot and get new startgoing time
                        df = pd.read_csv(trackSettings.trajFile, sep=',', encoding="utf-8")
                        altlist1 = []
                        azlist1 = []
                        timelist = []
                        horizonaz = -999
                        waittime = 0
                        for index, row in df.iterrows():
                            timelist.append(row['time'])
                            #Find the row that it rises on horizon and slew there
                            rowalt = float(row['elevationDegs'])
                            refraction = self.atm_refraction(rowalt)
                            rowalt = rowalt+refraction
                            if rowalt > trackSettings.horizonaltitude and self.horizonalt < -990:
                                waittime = row['time']
                                startgoingtime = initialtime + datetime.timedelta(seconds=waittime)
                                thisrowalt = float(row['elevationDegs'])
                                refraction = self.atm_refraction(thisrowalt)
                                self.horizonalt = thisrowalt+refraction
                                horizonaz = row['azimuthDegs']
                                #Slew to pad location
                                launchobserver = ephem.Observer()
                                launchobserver.lat = (str(self.entryLat.get()))
                                launchobserver.lon = (str(self.entryLon.get()))
                                launchobserver.date = datetime.datetime.utcnow()
                                launchobserver.pressure = 0
                                launchobserver.epoch = launchobserver.date
                                raslew, decslew = launchobserver.radec_of(math.radians(horizonaz), math.radians(self.horizonalt))
                                self.raslew = raslew
                                self.decslew = decslew
                                trackSettings.goforSlew = True
                                self.ASCOMSlewThread = threading.Thread(target=self.ASCOMSlew, daemon=True)
                                self.ASCOMSlewThread.start()
                                #self.tel.SlewToCoordinates((math.degrees(raslew)/15),math.degrees(decslew))
                                while trackSettings.slewCompleted is False:
                                    time.sleep(0.01)
                                self.tel.MoveAxis(0, 0)
                                self.tel.MoveAxis(1, 0)
                            thisrowalt = float(row['elevationDegs'])
                            refraction = self.atm_refraction(thisrowalt)
                            thisrowalt = thisrowalt+refraction
                            altlist1.append(thisrowalt)
                            azlist1.append(row['azimuthDegs'])
                        altlast = altlist1[0]
                        azlast = azlist1[0]
                        
        ###Do this if we're set to adaptive tracking mode###
        elif trackSettings.trackingmode == 'Adaptive':
            if trackSettings.fileSelected is True:
                df = pd.read_csv(trackSettings.trajFile, sep=',', encoding="utf-8")
                altlist1 = []
                azlist1 = []
                timelist = []
                altratelist = []
                azratelist = []
                r, c = df.shape
                for index, row in df.iterrows():
                    timelist.append(row['time'])
                    thisrowalt = float(row['elevationDegs'])
                    refraction = self.atm_refraction(thisrowalt)
                    thisrowalt = thisrowalt+refraction
                    altlist1.append(thisrowalt)
                    azlist1.append(row['azimuthDegs'])
                    #Pre-Calculate rates
                    if (index+2)<r:
                        predictedalt = float(df.iloc[index].loc['elevationDegs'])
                        refraction = self.atm_refraction(predictedalt)
                        predictedalt = predictedalt+refraction
                        predictedaz = float(df.iloc[index].loc['azimuthDegs'])
                        nextpredictedalt = float(df.iloc[index+1].loc['elevationDegs'])
                        nextrefraction = self.atm_refraction(nextpredictedalt)
                        nextpredictedalt = nextpredictedalt+nextrefraction
                        altrate = nextpredictedalt - predictedalt
                        headingeast = ((df.iloc[index+1].loc['azimuthDegs'] + 360)-df.iloc[index].loc['azimuthDegs'])
                        headingwest = (df.iloc[index+1].loc['azimuthDegs'] - (df.iloc[index].loc['azimuthDegs']+360))
                        azrate = df.iloc[index+1].loc['azimuthDegs'] - df.iloc[index].loc['azimuthDegs']
                        if abs(headingeast) < abs(azrate):
                            azrate = headingeast
                        elif abs(headingwest) < abs(azrate):
                            azrate = headingwest  
                        altratelist.append(altrate)
                        azratelist.append(azrate)
                altratelist.append(0.0)
                azratelist.append(0.0)
                altratelist.append(0.0)
                azratelist.append(0.0)
                df['azrate'] = azratelist
                df['altrate'] = altratelist
                altlast = altlist1[0]
                azlast = azlist1[0]
                #Sync to pad location
                ref_tel_alt = self.tel.Altitude
                ref_tel_az = self.tel.Azimuth
                weighted_avg_alt_sep, weighted_avg_az_sep = self.errorWeightedAverage(ref_tel_alt, ref_tel_az)
                azlast = azlast - weighted_avg_az_sep
                altlast = altlast - weighted_avg_alt_sep
                launchobserver = ephem.Observer()
                launchobserver.lat = (str(self.entryLat.get()))
                launchobserver.lon = (str(self.entryLon.get()))
                launchobserver.date = datetime.datetime.utcnow()
                launchobserver.pressure = 0
                launchobserver.epoch = launchobserver.date
                rasync, decsync = launchobserver.radec_of(math.radians(azlast), math.radians(altlast))
                self.tel.SyncToCoordinates((math.degrees(rasync)/15), math.degrees(decsync))
                pad = ephem.FixedBody()
                pad._ra = rasync
                pad._dec = decsync
                pad._epoch = launchobserver.date
                pad.compute(launchobserver)
                padalt = pad.alt
                padaz = pad.az
                print(altlast, azlast, rasync, decsync, padalt, padaz)
            else:
                endoffile = True
            currentindex = 0.0
            lasttime = datetime.datetime.utcnow()
            while trackSettings.runninglaunch is True and endoffile is False:
                time.sleep(0.001)
                r, c = df.shape
                timedelta = (currenttime - initialtime).total_seconds()
                currenttime = datetime.datetime.utcnow()
                timedelta2 = (currenttime - lasttime).total_seconds()
                lasttime = datetime.datetime.utcnow()
                currentindex = currentindex + timedelta2
                hours = timedelta // 3600
                minutes = (timedelta % 3600) // 60
                seconds = timedelta % 60
                self.countdowntext.set(str('t+ '+str(math.trunc(hours))+' hours '+str(math.trunc(minutes))+' minutes '+str(math.trunc(seconds))+' seconds'))
                #Find minimum difference between current position and time to df
                mindiff = 99999999999999999
                mindiffindex = 0
                ref_tel_alt = self.tel.Altitude
                ref_tel_az = self.tel.Azimuth
                weighted_avg_alt_sep, weighted_avg_az_sep = self.errorWeightedAverage(ref_tel_alt, ref_tel_az)
                currentaz = ref_tel_az + weighted_avg_az_sep
                currentalt = ref_tel_alt + weighted_avg_alt_sep
                try:
                    for index, row in df.iterrows():
                        thisrowalt = float(row['elevationDegs'])
                        refraction = self.atm_refraction(thisrowalt)
                        thisrowalt = thisrowalt+refraction
                        currentdiff = math.sqrt(abs(thisrowalt-currentalt)**2+abs(row['azimuthDegs']-currentaz)**2)+(abs(row['time']-currentindex))
                        if currentdiff < mindiff:
                            mindiff = currentdiff
                            mindiffindex = index
                    #we're going to adjust a moving variable representing where we are in the playback based on scope current position
                    indexdiff = float(mindiffindex) - currentindex
                    currentindex = currentindex+(indexdiff/5)
                    outindex = round(currentindex)
                    altrate = altratelist[outindex]
                    azrate = azratelist[outindex]
                    #df_sort = df.iloc[(df['time']-timedelta).abs().argsort()[:2]]
                    #df_sort = df_sort.sort_values(by=['time'])
                    #altlist1 = []
                    #azlist1 = []
                    #timelist = []
                    #Convert absolute coordinates to relative rates
                    #index = 0
                    #altrate = df_sort.iloc[index+1].loc['elevationDegs'] - df_sort.iloc[index].loc['elevationDegs']
                    #headingeast = ((df_sort.iloc[index+1].loc['azimuthDegs'] + 360)-df_sort.iloc[index].loc['azimuthDegs'])
                    #headingwest = (df_sort.iloc[index+1].loc['azimuthDegs'] - (df_sort.iloc[index].loc['azimuthDegs']+360))
                    #azrate = df_sort.iloc[index+1].loc['azimuthDegs'] - df_sort.iloc[index].loc['azimuthDegs']
                    #if abs(headingeast) < abs(azrate):
                    #    azrate = headingeast
                    #elif abs(headingwest) < abs(azrate):
                    #    azrate = headingwest      
                    #These should be the absolute coordinates but just store relative rates for now
                    alt = altrate
                    az = azrate
                    #Check to see if joystick button was pushed to switch over to manual tracking
                    pygame.event.pump()
                    if self.joysticks[0].get_button(trackSettings.gomanualbutton) > 0:
                        currentindex = r+999999999999999999999999999999
                    #check to see if we reached the end of the interpolation, remember to account for extra 20 seconds before launch)
                    if currentindex > (r):
                        endoffile = True
                        trackSettings.runninglaunch = False
                        trackSettings.buttonpushed = False
                        trackSettings.feedingdata = False
                        self.launchButton.configure(text='Arm Launch Tracking')
                        trackSettings.joytracking = True
                        self.trackthread = threading.Thread(target=self.track, daemon=True)
                        self.startButton4.configure(text='Stop Joystick Tracking')
                        self.trackthread.start()
                    if math.isinf(altrate) or math.isinf(azrate):
                        pass
                    else:
                        #print(timedelta, alt, az, altrate, azrate)
                        self.timedeltaout, self.altout, self.azout, self.altrateout, self.azrateout = timedelta, alt, az, altrate, azrate
                        trackSettings.feedingdata = True
                except Exception as exce:
                    print(exce)
                    pass
        ##Do this if we're in regular predictive tracking##
        elif trackSettings.trackingmode == 'Regular':
            if trackSettings.fileSelected is True:
                df = pd.read_csv(trackSettings.trajFile, sep=',', encoding="utf-8")
                altlist1 = []
                azlist1 = []
                timelist = []
                for index, row in df.iterrows():
                    timelist.append(row['time'])
                    thisrowalt = float(row['elevationDegs'])
                    refraction = self.atm_refraction(thisrowalt)
                    thisrowalt = thisrowalt+refraction
                    altlist1.append(thisrowalt)
                    azlist1.append(row['azimuthDegs'])
                altlast = altlist1[0]
                azlast = azlist1[0]
                #Sync to pad location
                ref_tel_alt = self.tel.Altitude
                ref_tel_az = self.tel.Azimuth
                weighted_avg_alt_sep, weighted_avg_az_sep = self.errorWeightedAverage(ref_tel_alt, ref_tel_az)
                azlast = azlast - weighted_avg_az_sep
                altlast = altlast - weighted_avg_alt_sep
                
                launchobserver = ephem.Observer()
                launchobserver.lat = (str(self.entryLat.get()))
                launchobserver.lon = (str(self.entryLon.get()))
                launchobserver.date = datetime.datetime.utcnow()
                launchobserver.pressure = 0
                launchobserver.epoch = launchobserver.date
                rasync, decsync = launchobserver.radec_of(math.radians(azlast), math.radians(altlast))
                self.tel.SyncToCoordinates((math.degrees(rasync)/15), math.degrees(decsync))
                
                launchobserver = ephem.Observer()
                launchobserver.lat = (str(self.entryLat.get()))
                launchobserver.lon = (str(self.entryLon.get()))
                launchobserver.date = datetime.datetime.utcnow()
                launchobserver.pressure = trackSettings.pressure
                launchobserver.epoch = launchobserver.date
            else:
                endoffile = True
            while trackSettings.runninglaunch is True and endoffile is False:
                time.sleep(0.001)
                r, c = df.shape
                currenttime = datetime.datetime.utcnow()
                launchobserver.date = datetime.datetime.utcnow()
                timedelta = (currenttime - initialtime).total_seconds()
                onesecondlater = ((currenttime - initialtime).total_seconds()+1)
                twosecondslater = onesecondlater+1
                hours = timedelta // 3600
                minutes = (timedelta % 3600) // 60
                seconds = timedelta % 60
                self.countdowntext.set(str('t+ '+str(math.trunc(hours))+' hours '+str(math.trunc(minutes))+' minutes '+str(math.trunc(seconds))+' seconds'))
                timedelta2 = timedelta - timedeltalast
                timedeltalast = timedelta
                df_sort = df.iloc[(df['time']-timedelta).abs().argsort()[:2]]
                df_sort = df_sort.sort_values(by=['time'])
                #Find the row exactly 1 second later to be able to compute degrees per second
                df_sort2 = df.iloc[(df['time']-onesecondlater).abs().argsort()[:2]]
                df_sort2 = df_sort2.sort_values(by=['time'])
                df_sort3 = df.iloc[(df['time']-twosecondslater).abs().argsort()[:2]]
                df_sort3 = df_sort3.sort_values(by=['time'])
                altlist1 = []
                azlist1 = []
                timelist = []
                #Convert absolute coordinates to relative rates
                index = 0
                predictedalt = float(df_sort.iloc[index].loc['elevationDegs'])
                refraction = self.atm_refraction(predictedalt)
                predictedalt = predictedalt+refraction
                predictedaz = float(df_sort.iloc[index].loc['azimuthDegs'])
                nextpredictedalt = float(df_sort2.iloc[index].loc['elevationDegs'])
                nextrefraction = self.atm_refraction(nextpredictedalt)
                nextpredictedalt = nextpredictedalt+nextrefraction
                thirdpredictedalt = float(df_sort3.iloc[index].loc['elevationDegs'])
                thirdrefraction = self.atm_refraction(thirdpredictedalt)
                thirdpredictedalt = thirdpredictedalt+thirdrefraction
                nextpredictedaz = float(df_sort2.iloc[index].loc['azimuthDegs'])
                thirdpredictedaz = float(df_sort3.iloc[index].loc['azimuthDegs'])
                currentpredicttime = float(df_sort.iloc[index].loc['time'])
                nextpredicttime = float(df_sort2.iloc[index].loc['time'])
                thirdpredicttime = float(df_sort3.iloc[index].loc['time'])
                #Account for possible unstable increments in time to get degrees per second
                nexttimediff = nextpredicttime - currentpredicttime
                thirdtimediff = thirdpredicttime - nextpredicttime
                altrate = (nextpredictedalt - predictedalt)/nexttimediff
                nextaltrate = (thirdpredictedalt - nextpredictedalt)/thirdtimediff
                #Now I'm using df_sort2 as index+1 because df_sort2 actually finds the closest value to 1 second later, the next row of df_sort isn't NECESSARILY one second later
                headingeast = ((df_sort2.iloc[index].loc['azimuthDegs'] + 360)-df_sort.iloc[index].loc['azimuthDegs'])/nexttimediff
                headingwest = (df_sort2.iloc[index].loc['azimuthDegs'] - (df_sort.iloc[index].loc['azimuthDegs']+360))/nexttimediff
                nextheadingeast = ((df_sort3.iloc[index].loc['azimuthDegs'] + 360)-df_sort2.iloc[index].loc['azimuthDegs'])/thirdtimediff
                nextheadingwest = (df_sort3.iloc[index].loc['azimuthDegs'] - (df_sort2.iloc[index].loc['azimuthDegs']+360))/thirdtimediff
                azrate = (df_sort2.iloc[index].loc['azimuthDegs'] - df_sort.iloc[index].loc['azimuthDegs'])/nexttimediff
                nextazrate = (thirdpredictedaz - nextpredictedaz)/thirdtimediff
                if abs(nextheadingeast) < abs(nextazrate):
                    nextazrate = nextheadingeast
                elif abs(nextheadingwest) < abs(nextazrate):
                    nextazrate = nextheadingwest      
                if abs(headingeast) < abs(azrate):
                    azrate = headingeast
                elif abs(headingwest) < abs(azrate):
                    azrate = headingwest      
                #Interpolate current alt and az rates!
                altrate = altrate + (timedelta-currentpredicttime)*((nextaltrate-altrate)/(nextpredicttime-currentpredicttime))
                azrate = azrate + (timedelta-currentpredicttime)*((nextazrate-azrate)/(nextpredicttime-currentpredicttime))                
                #These should be the absolute coordinates but just store relative rates for now
                alt = altrate
                az = azrate
                #Check to see if joystick button was pushed to switch over to manual tracking
                pygame.event.pump()
                if self.joysticks[0].get_button(trackSettings.gomanualbutton) > 0:
                    timedelta = r+1
                #check to see if we reached the end of the interpolation, remember to account for extra 20 seconds before launch)
                if timedelta > (r):
                    endoffile = True
                    trackSettings.runninglaunch = False
                    trackSettings.buttonpushed = False
                    trackSettings.feedingdata = False
                    self.launchButton.configure(text='Arm Launch Tracking')
                    trackSettings.joytracking = True
                    self.trackthread = threading.Thread(target=self.track, daemon=True)
                    self.startButton4.configure(text='Stop Joystick Tracking')
                    self.trackthread.start()
                if math.isinf(altrate) or math.isinf(azrate):
                    pass
                else:
                    #print(timedelta, alt, az, altrate, azrate)
                    #self.timedeltaout, self.altout, self.azout, self.altrateout, self.azrateout = timedelta, alt, az, altrate, azrate
                    self.timedeltaout, self.altout, self.azout, self.altrateout, self.azrateout, self.predictalt, self.predictaz, self.nextpredictedalt, self.nextpredictedaz, self.currentpredicttime, self.nextpredicttime, self.thirdpredictedalt, self.thirdpredictedaz, self.thirdpredicttime = timedelta, alt, az, altrate, azrate, predictedalt, predictedaz, nextpredictedalt, nextpredictedaz, currentpredicttime, nextpredicttime, thirdpredictedalt, thirdpredictedaz, thirdpredicttime
                    trackSettings.feedingdata = True
        trackSettings.feedingdata = False
                
    def run_sim(self):
        initialtime = datetime.datetime.now()
        endoffile = False
        if trackSettings.joytracking is True:
            trackSettings.joytracking = False
            self.startButton4.configure(text='Start Joystick Tracking')
        timedeltalast = 0
        if trackSettings.fileSelected is True:
            df = pd.read_csv(trackSettings.trajFile, sep=',', encoding="utf-8")
            altlist1 = []
            azlist1 = []
            timelist = []
            for index, row in df.iterrows():
                timelist.append(row['time'])
                thisrowalt = float(row['elevationDegs'])
                refraction = self.atm_refraction(thisrowalt)
                thisrowalt = thisrowalt+refraction
                altlist1.append(thisrowalt)
                azlist1.append(row['azimuthDegs'])
            altlast = altlist1[0]
            azlast = azlist1[0]
        else:
            endoffile = True
        while trackSettings.runningsimulation is True and endoffile is False:
            time.sleep(0.001)
            r, c = df.shape
            currenttime = datetime.datetime.now()
            timedelta = (currenttime - initialtime).total_seconds()
            timedelta2 = timedelta - timedeltalast
            timedeltalast = timedelta
            df_sort = df.iloc[(df['time']-timedelta).abs().argsort()[:2]]
            df_sort = df_sort.sort_values(by=['time'])
            altlist1 = []
            azlist1 = []
            timelist = []
            #Convert absolute coordinates to relative rates
            index = 0
            predictedalt = float(df_sort.iloc[index].loc['elevationDegs'])
            refraction = self.atm_refraction(predictedalt)
            predictedalt = predictedalt+refraction
            predictedaz = df_sort.iloc[index].loc['azimuthDegs']
            nextpredictedalt = float(df_sort.iloc[index+1].loc['elevationDegs'])
            nextrefraction = self.atm_refraction(nextpredictedalt)
            nextpredictedalt = nextpredictedalt+nextrefraction
            altrate = nextpredictedalt - predictedalt
            headingeast = ((df_sort.iloc[index+1].loc['azimuthDegs'] + 360)-df_sort.iloc[index].loc['azimuthDegs'])
            headingwest = (df_sort.iloc[index+1].loc['azimuthDegs'] - (df_sort.iloc[index].loc['azimuthDegs']+360))
            azrate = df_sort.iloc[index+1].loc['azimuthDegs'] - df_sort.iloc[index].loc['azimuthDegs']
            if abs(headingeast) < abs(azrate):
                azrate = headingeast
            elif abs(headingwest) < abs(azrate):
                azrate = headingwest      
            #These should be the absolute coordinates but just store relative rates for now
            alt = altrate
            az = azrate
            #Check to see if joystick button was pushed to switch over to manual tracking
            pygame.event.pump()
            if trackSettings.joystickconnected is False:
                if self.joysticks[0].get_button(trackSettings.gomanualbutton) > 0:
                    timedelta = r+1
            elif self.remotejoybutton3 > 0:
                timedelta = r+1
            #check to see if we reached the end of the interpolation, remember to account for extra 20 seconds before launch)
            if timedelta > (r):
                endoffile = True
                trackSettings.runningsimulation = False
                trackSettings.feedingdata = False
                self.simulateButton.configure(text='Simulate Launch')
                trackSettings.joytracking = True
                self.trackthread = threading.Thread(target=self.track, daemon=True)
                self.startButton4.configure(text='Stop Joystick Tracking')
                self.trackthread.start()
            if math.isinf(altrate) or math.isinf(azrate):
                pass
            else:
                #print(timedelta, alt, az, altrate, azrate)
                self.timedeltaout, self.altout, self.azout, self.altrateout, self.azrateout, self.predictalt, self.predictaz = timedelta, alt, az, altrate, azrate, predictedalt, predictedaz
                trackSettings.feedingdata = True
        trackSettings.feedingdata = False
    
    def simulate_launch(self): 
        holdrate = False
        lastbutton = 0
        lastbutton2 = 0
        altratelast = 0
        azratelast = 0
        altcorrect = 0
        azcorrect = 0
        azrate = 0
        altrate = 0
        altcorrectrunning = 0
        azcorrectrunning = 0
        deccorrect = 0
        racorrect = 0
        self.diffazlast = 0
        self.diffaltlast = 0
        self.diffralast = 0
        self.diffdeclast = 0
        self.lasttotaldiff = 0.0
        i = 0
        iterate = 0
        objectazrate = 0
        objectaltrate = 0
        commandedaltratelast = 0
        commandedazratelast = 0
        firstslew = True
        launchobserver = ephem.Observer()
        launchobserver.lat = (str(self.entryLat.get()))
        launchobserver.lon = (str(self.entryLon.get()))
        launchobserver.date = datetime.datetime.utcnow()
        launchobserver.pressure = 0
        launchobserver.elevation = 0
        launchobserver.epoch = launchobserver.date
        while trackSettings.feedingdata is False:
            time.sleep(0.001)
        while trackSettings.joytracking is False and trackSettings.feedingdata is True:
            time.sleep(0.001)
            if trackSettings.objectfollow is False:
                self.dlast = self.dnow
                pygame.event.pump()
                i = i + 1
                if trackSettings.telescopetype == 'Autostar':
                    d = datetime.datetime.utcnow()
                    if trackSettings.mounttype == 'AltAz':
                        if trackSettings.joystickconnected is False:
                            joy0 = self.joysticks[0].get_axis(0)
                            joy1 = self.joysticks[0].get_axis(1)
                            joy2 = self.joysticks[0].get_axis(2)
                            throttle = ((joy2*-1.0)+1.0)/2.0
                            #joybutton2 = self.joysticks[0].get_button(4)
                            #if joybutton2 == 1 and lastbutton2 == 0:
                            #    self.ser.write(str.encode(':Q#'))
                            #    lastbutton2 = 1
                            #if joybutton2 == 0 and lastbutton2 == 1:
                            #    lastbutton2 = 0
                            joybutton = self.joysticks[0].get_button(trackSettings.holdratebutton)
                            #Do focuser commands
                            joyfocus1 = self.joysticks[0].get_button(trackSettings.focuserbutton1)
                            joyfocus2 = self.joysticks[0].get_button(trackSettings.focuserbutton2)
                            if trackSettings.focuserconnected is True:
                                try:
                                    if joyfocus1 == 1:
                                        self.focuserSerial.write(b'button1\n')
                                    elif joyfocus2 == 1:
                                        self.focuserSerial.write(b'button2\n')
                                    else:
                                        self.focuserSerial.write(b'stopbutton\n')
                                except Exception as e:
                                    print(e)
                            elif trackSettings.ASCOMFocuser is True:
                                try:
                                    if joyfocus1 == 1:
                                        if trackSettings.focuserabsolute is True:
                                            currentpos = self.ASCOMFocuser.Position
                                            newpos = currentpos + trackSettings.maxfocuserstep
                                            self.ASCOMFocuser.Move(newpos)
                                        else:
                                            self.ASCOMFocuser.Move(int(trackSettings.maxfocuserstep))
                                    elif joyfocus2 == 1:
                                        if trackSettings.focuserabsolute is True:
                                            currentpos = self.ASCOMFocuser.Position
                                            newpos = currentpos - trackSettings.maxfocuserstep
                                            self.ASCOMFocuser.Move(newpos)
                                        else:
                                            self.ASCOMFocuser.Move(int(-1*trackSettings.maxfocuserstep))
                                    elif trackSettings.haltcompatible is True:
                                        try:
                                            self.ASCOMFocuser.Halt()
                                        except:
                                            pass
                                except Exception as e:
                                    print(e)
                        else:
                            joy0 = self.remotejoy0
                            joy1 = self.remotejoy1
                            throttle = self.remotethrottle
                            joybutton2 = self.remotejoybutton2
                            joybutton = self.remotejoybutton1
                            joybutton3 = self.remotejoybutton3
                            joybutton4 = self.remotejoybutton4
                            joybutton5 = self.remotejoybutton5
                            try:
                                if trackSettings.focuserconnected is True:
                                    if joybutton4 == 1:
                                        self.focuserSerial.write(b'button1\n')
                                    elif joybutton5 == 1:
                                        self.focuserSerial.write(b'button2\n')
                                    else:
                                        self.focuserSerial.write(b'stopbutton\n')
                                elif trackSettings.ASCOMFocuser is True:
                                    try:
                                        if joybutton4 == 1:
                                            if trackSettings.focuserabsolute is True:
                                                currentpos = self.ASCOMFocuser.Position
                                                newpos = currentpos + trackSettings.maxfocuserstep
                                                self.ASCOMFocuser.Move(newpos)
                                            else:
                                                self.ASCOMFocuser.Move(int(trackSettings.maxfocuserstep))
                                        elif joybutton5 == 1:
                                            if trackSettings.focuserabsolute is True:
                                                currentpos = self.ASCOMFocuser.Position
                                                newpos = currentpos - trackSettings.maxfocuserstep
                                                self.ASCOMFocuser.Move(newpos)
                                            else:
                                                self.ASCOMFocuser.Move(int(-1*trackSettings.maxfocuserstep))
                                        elif trackSettings.haltcompatible is True:
                                            try:
                                                self.ASCOMFocuser.Halt()
                                            except:
                                                pass
                                    except Exception as e:
                                        print(e)
                            except Exception as e:
                                print(e)
                        if joybutton == 1 and lastbutton == 0:
                            if holdrate is False:
                                holdrate = True
                                self.HoldLabel.config(text='Hold Rate ON')
                            else:
                                holdrate = False
                                self.HoldLabel.config(text='Hold Rate OFF')
                            lastbutton = 1
                        if joybutton == 0 and lastbutton == 1:
                            lastbutton = 0
                        azrate = (joy0*throttle)*self.axis0rate + float(self.azrateout)
                        altrate = (joy1*throttle*-1)*self.axis0rate + float(self.altrateout)
                        if self.joyxrev.get() == 1:
                            azrate = azrate*-1
                        if self.joyyrev.get() == 1:
                            altrate = altrate*-1
                        #if azrate > 0.0:
                        #    azrate = azrate + (0.001*random.randrange(0, 5, 1)) 
                        #if altrate > 0.0:
                        #    altrate = altrate + (0.001*random.randrange(0, 5, 1)) 
                        #if azrate < 0.0:
                        #    azrate = azrate - (0.001*random.randrange(0, 5, 1))
                        #if altrate < 0.0:
                        #    altrate = altrate - (0.001*random.randrange(0, 5, 1))
                        if azrate > self.axis0rate:
                            azrate = self.axis0rate
                        if azrate < (-1*self.axis0rate):
                            azrate = (-1*self.axis0rate)
                        if altrate > self.axis1rate:
                            altrate = self.axis1rate
                        if altrate < (-1*self.axis1rate):
                            altrate = (-1*self.axis1rate)
                        if throttle < 0.001:
                            azrate = 0.0 + float(self.azrateout)
                            altrate = 0.0 + float(self.altrateout)
                        self.textbox.insert(END, str('Az Rate: ' + str(azrate) + ' Alt Rate: ' + str(altrate) + '\n'))
                        self.textbox.see('end')
                        #if holdrate is False:
                        self.ser.write(str.encode(str(':RA'+str(azrate)+'#')))
                        self.ser.write(str.encode(str(':RE'+str(altrate)+'#')))
                        self.ser.write(str.encode(':Me#'))
                        self.ser.write(str.encode(':Mn#'))
                        azratelast = azrate
                        altratelast = altrate
                        #else:
                        #    self.tel.MoveAxis(0, azratelast)
                        #    self.tel.MoveAxis(1, altratelast)
                        altcorrect = 0
                        azcorrect = 0
                if trackSettings.telescopetype == 'ASCOM':
                    d = datetime.datetime.utcnow()
                    if trackSettings.mounttype == 'AltAz':
                        if trackSettings.joystickconnected is False:
                            joy0 = self.joysticks[0].get_axis(0)
                            joy1 = self.joysticks[0].get_axis(1)
                            joy2 = self.joysticks[0].get_axis(2)
                            throttle = ((joy2*-1.0)+1.0)/2.0
                            #joybutton2 = self.joysticks[0].get_button(4)
                            #if joybutton2 == 1 and lastbutton2 == 0:
                            #    self.tel.MoveAxis(0, 0.0)
                            #    self.tel.MoveAxis(1, 0.0)
                            #    lastbutton2 = 1
                            #if joybutton2 == 0 and lastbutton2 == 1:
                            #    lastbutton2 = 0
                            joybutton = self.joysticks[0].get_button(trackSettings.holdratebutton)
                            #Do focuser commands
                            joyfocus1 = self.joysticks[0].get_button(trackSettings.focuserbutton1)
                            joyfocus2 = self.joysticks[0].get_button(trackSettings.focuserbutton2)
                            if trackSettings.focuserconnected is True:
                                try:
                                    if joyfocus1 == 1:
                                        self.focuserSerial.write(b'button1\n')
                                    elif joyfocus2 == 1:
                                        self.focuserSerial.write(b'button2\n')
                                    else:
                                        self.focuserSerial.write(b'stopbutton\n')
                                except Exception as e:
                                    print(e)
                            elif trackSettings.ASCOMFocuser is True:
                                try:
                                    if joyfocus1 == 1:
                                        if trackSettings.focuserabsolute is True:
                                            currentpos = self.ASCOMFocuser.Position
                                            newpos = currentpos + trackSettings.maxfocuserstep
                                            self.ASCOMFocuser.Move(newpos)
                                        else:
                                            self.ASCOMFocuser.Move(int(trackSettings.maxfocuserstep))
                                    elif joyfocus2 == 1:
                                        if trackSettings.focuserabsolute is True:
                                            currentpos = self.ASCOMFocuser.Position
                                            newpos = currentpos - trackSettings.maxfocuserstep
                                            self.ASCOMFocuser.Move(newpos)
                                        else:
                                            self.ASCOMFocuser.Move(int(-1*trackSettings.maxfocuserstep))
                                    elif trackSettings.haltcompatible is True:
                                        try:
                                            self.ASCOMFocuser.Halt()
                                        except:
                                            pass
                                except Exception as e:
                                    print(e)
                        else:
                            joy0 = self.remotejoy0
                            joy1 = self.remotejoy1
                            throttle = self.remotethrottle
                            joybutton2 = self.remotejoybutton2
                            joybutton = self.remotejoybutton1
                            joybutton3 = self.remotejoybutton3
                            joybutton4 = self.remotejoybutton4
                            joybutton5 = self.remotejoybutton5
                            try:
                                if trackSettings.focuserconnected is True:
                                    if joybutton4 == 1:
                                        self.focuserSerial.write(b'button1\n')
                                    elif joybutton5 == 1:
                                        self.focuserSerial.write(b'button2\n')
                                    else:
                                        self.focuserSerial.write(b'stopbutton\n')
                                elif trackSettings.ASCOMFocuser is True:
                                    try:
                                        if joybutton4 == 1:
                                            if trackSettings.focuserabsolute is True:
                                                currentpos = self.ASCOMFocuser.Position
                                                newpos = currentpos + trackSettings.maxfocuserstep
                                                self.ASCOMFocuser.Move(newpos)
                                            else:
                                                self.ASCOMFocuser.Move(int(trackSettings.maxfocuserstep))
                                        elif joybutton5 == 1:
                                            if trackSettings.focuserabsolute is True:
                                                currentpos = self.ASCOMFocuser.Position
                                                newpos = currentpos - trackSettings.maxfocuserstep
                                                self.ASCOMFocuser.Move(newpos)
                                            else:
                                                self.ASCOMFocuser.Move(int(-1*trackSettings.maxfocuserstep))
                                        elif trackSettings.haltcompatible is True:
                                            try:
                                                self.ASCOMFocuser.Halt()
                                            except:
                                                pass
                                    except Exception as e:
                                        print(e)
                            except Exception as e:
                                print(e)
                        if joybutton == 1 and lastbutton == 0:
                            if holdrate is False:
                                holdrate = True
                                self.HoldLabel.config(text='Hold Rate ON')
                            else:
                                holdrate = False
                                self.HoldLabel.config(text='Hold Rate OFF')
                            lastbutton = 1
                        if joybutton == 0 and lastbutton == 1:
                            lastbutton = 0
                        #Calculate the az and alt rates differently if we need to include a spiral search or not
                        #Check joystick first to see if we're commanded to do spiral search or alternatively lock onto prediction
                        if trackSettings.joystickconnected is False:
                            joyspiral = self.joysticks[0].get_button(trackSettings.spiralbutton)
                        else:
                        #PLACEHOLDER UNTIL I UPDATE THE REMOTE JOY SERVER TO HAVE A SPIRAL BUTTON
                            joyspiral = 0
                        if joyspiral == 1 and trackSettings.lastspiralbutton == 0:
                            self.start_spiral_search()
                            trackSettings.lastspiralbutton = 1
                        elif joyspiral == 0 and trackSettings.lastspiralbutton == 1:
                            trackSettings.lastspiralbutton = 0
                                                
                        if trackSettings.joystickconnected is False:
                            joyprediction = self.joysticks[0].get_button(trackSettings.predictionlockbutton)
                        else:
                        #PLACEHOLDER UNTIL I UPDATE THE REMOTE JOY SERVER TO HAVE A PREDICTION LOCK BUTTON
                            joyprediction = 0
                        if joyprediction == 1 and trackSettings.lastpredictionLockbutton == 0:
                            self.start_prediction_lock()
                            trackSettings.lastpredictionLockbutton = 1
                        elif joyprediction == 0 and trackSettings.lastpredictionLockbutton == 1:
                            trackSettings.lastpredictionLockbutton = 0

                        if trackSettings.spiralSearch is False:
                            if trackSettings.predictionLock is False:
                                azrate = (joy0*throttle)*self.axis0rate + float(self.azrateout) + (0.000001*random.randrange(0, 5, 1))
                                altrate = (joy1*throttle*-1)*self.axis0rate + float(self.altrateout) + (0.000001*random.randrange(0, 5, 1))
                            elif trackSettings.predictionLock is True:
                                #Get time interval - initialize the "last time" using the "start_prediction_lock" function above
                                currentpredictiontime = datetime.datetime.utcnow()
                                ref_tel_alt = self.tel.Altitude
                                ref_tel_az = self.tel.Azimuth
                                weighted_avg_alt_sep, weighted_avg_az_sep = self.errorWeightedAverage(ref_tel_alt, ref_tel_az)
                                currentaz = ref_tel_az + weighted_avg_az_sep
                                currentalt = ref_tel_alt + weighted_avg_alt_sep
                                #Check to make sure these aren't stale coordinates from the telescope
                                telaltmovement = abs(currentalt - self.lastalt)
                                telazmovement = abs(currentaz - self.lastaz)
                                staletelcoords = False
                                #This PROBABLY still works, but need to test it to make sure I'm not mistaking random noise for non-stale coordinates
                                if telaltmovement + telazmovement < 0.0001:
                                    #It's stale so don't make corrections, just use the current interpolated alt/az rate - UNLESS PREDICTION LOCK IS ON! If prediction lock is on, using the current interpolated rate will quickly override prediction corrections, so don't issue a new command if that's the case - check for that at the time you send the command
                                    azrate = float(self.azrateout)
                                    altrate = float(self.altrateout)
                                    staletelcoords = True
                                #Interpolate current predicted position if the telescope coordinates weren't stale
                                if staletelcoords is False:
                                    #Only save the current alt/az when the coordinates aren't stale to compare later
                                    self.lastalt = currentalt
                                    self.lastaz = currentaz
                                    predictiontplus = (currentpredictiontime - trackSettings.launchtime).total_seconds()  # the point in time between current_time and initial time
                                    if self.nextpredicttime-self.currentpredicttime > 0:
                                        interpolatedalt = self.predictalt + (predictiontplus-self.currentpredicttime)*((self.nextpredictedalt-self.predictalt)/(self.nextpredicttime-self.currentpredicttime))
                                        #We NEED to handle crossing the north meridian!
                                        #check if we're crossing north meridian and correct if so
                                        headingwestaz = self.nextpredictedaz - (self.predictaz+360)
                                        headingeastaz = (self.nextpredictedaz+360) - self.predictaz
                                        diffinaz = self.nextpredictedaz - self.predictaz
                                        if abs(headingwestaz) < abs(diffinaz):
                                            self.predictaz+=360
                                        if abs(headingeastaz) < abs(diffinaz):
                                            self.nextpredictedaz+=360
                                        interpolatedaz = self.predictaz + (predictiontplus-self.currentpredicttime)*((self.nextpredictedaz-self.predictaz)/(self.nextpredicttime-self.currentpredicttime))
                                        predictaltdiff = interpolatedalt - currentalt
                                        predictazdiff = interpolatedaz - currentaz
                                        if predictazdiff < -180:
                                            predictazdiff += 360
                                        elif predictazdiff > 180:
                                            predictazdiff -= 360
                                        azrate = (predictazdiff*trackSettings.aggression) + float(self.azrateout)
                                        altrate = (predictaltdiff*trackSettings.aggression) + float(self.altrateout)
                                    else:
                                        continue
                                #self.azdifflist.append(predictazdiff)
                                #self.altdifflist.append(predictaltdiff)
                                #if len(self.azdifflist) > 2:
                                #    meanazdiff = sum(self.azdifflist)/len(self.azdifflist)
                                #    meanaltdiff = sum(self.altdifflist)/len(self.altdifflist)
                                #    self.azdifflist = []
                                #    self.altdifflist = []
                                #    azrate = ((predictazdiff*trackSettings.aggression)/predictiontimedelta) + float(self.azrateout) + (0.000001*random.randrange(0, 5, 1))
                                #    altrate = ((predictaltdiff*trackSettings.aggression)/predictiontimedelta) + float(self.altrateout) + (0.000001*random.randrange(0, 5, 1))
                                #else:
                                #    azrate = float(self.azrateout) + (0.000001*random.randrange(0, 5, 1))
                                #    altrate = float(self.altrateout) + (0.000001*random.randrange(0, 5, 1))
                        else:
                            launchobserver.date = datetime.datetime.utcnow()
                            ref_tel_alt = self.tel.Altitude
                            ref_tel_az = self.tel.Azimuth
                            weighted_avg_alt_sep, weighted_avg_az_sep = self.errorWeightedAverage(ref_tel_alt, ref_tel_az)
                            currentaz = ref_tel_az + weighted_avg_az_sep
                            currentalt = ref_tel_alt + weighted_avg_alt_sep
                            currentaltdegrees = currentalt
                            currentazdegrees = currentaz
                            altspiralrate = self.spiralalt - currentaltdegrees
                            azspiralrate = self.spiralaz - currentazdegrees
                            #print(altspiralrate, azspiralrate)
                            azrate = float(self.azrateout) + float(azspiralrate*1.5) + (0.000001*random.randrange(0, 5, 1))
                            altrate = float(self.altrateout) + float(altspiralrate*1.5) + (0.000001*random.randrange(0, 5, 1))
                            targetname = 'Predicted Position'
                            futurealt = -20
                            futureaz = 0
                            secondstorender = 1
                            #azrate = (joy0*throttle)*self.axis0rate + float(self.azrateout) + float(azspiralrate*2) + (0.000001*random.randrange(0, 5, 1))
                            #altrate = (joy1*throttle*-1)*self.axis0rate + float(self.altrateout) + float(altspiralrate*2) + (0.000001*random.randrange(0, 5, 1))
                        if self.joyxrev.get() == 1:
                            azrate = azrate*-1
                        if self.joyyrev.get() == 1:
                            altrate = altrate*-1
                        #if azrate > 0.0:
                        #    azrate = azrate + (0.001*random.randrange(0, 5, 1)) 
                        #if altrate > 0.0:
                        #    altrate = altrate + (0.001*random.randrange(0, 5, 1)) 
                        #if azrate < 0.0:
                        #    azrate = azrate - (0.001*random.randrange(0, 5, 1))
                        #if altrate < 0.0:
                        #    altrate = altrate - (0.001*random.randrange(0, 5, 1))
                        if azrate > self.axis0rate:
                            azrate = self.axis0rate
                        if azrate < (-1*self.axis0rate):
                            azrate = (-1*self.axis0rate)
                        if altrate > self.axis1rate:
                            altrate = self.axis1rate
                        if altrate < (-1*self.axis1rate):
                            altrate = (-1*self.axis1rate)
                        if throttle < 0.001 and trackSettings.spiralSearch is False and trackSettings.predictionLock is False:
                            azrate = 0.0 + float(self.azrateout) + (0.000001*random.randrange(0, 5, 1))
                            altrate = 0.0 + float(self.altrateout) + (0.000001*random.randrange(0, 5, 1))
                        try:                            
                            if trackSettings.spiralSearch is False:
                                if trackSettings.predictionLock is False:
                                    self.textbox.insert(END, str('Az Rate: ' + str(azrate) + ' Alt Rate: ' + str(altrate) +'\n'))
                                    self.textbox.see('end')
                                elif trackSettings.predictionLock is True:
                                    #self.textbox.insert(END, str('Az Diff: '+str(round(predictazdiff,2))+' Alt Diff: '+str(round(predictaltdiff,2))+' Az Rate: ' + str(round(azrate,2)) + ' Alt Rate: ' + str(round(altrate,2)) +' Current Az: ' + str(round(currentaz,2)) + ' Current Alt: ' + str(round(currentalt,2)) +'\n'))
                                    if staletelcoords is False:
                                        self.textbox.insert(END, str('Az Diff: '+str(round(predictazdiff,4))+' Alt Diff: '+str(round(predictaltdiff,4))+' Current Az: ' + str(round(currentaz,4)) + ' Current Alt: ' + str(round(currentalt,4)) +'\n'))
                                        self.textbox.see('end')
                                    #else:
                                    #    self.textbox.insert(END, str('Az Rate: ' + str(azrate) + ' Alt Rate: ' + str(altrate) +'\n'))
                                    #    self.textbox.see('end')
                            else:
                                altspiralrate, azspiralrate
                                self.textbox.insert(END, str('Spiral Az Diff: ' + str(round(azspiralrate,4)) + ' Spiral Alt Diff: ' + str(round(altspiralrate,4)) +' Spiral Radius: '+str(round(self.currentspiralradius,4))+'\n'))
                                self.textbox.see('end')
                        except Exception as poop:
                            print(poop)
                            pass
                        #if holdrate is False:
                        #Check command rate and issue command if enough time has passed
                        currentcommandtime = datetime.datetime.utcnow()
                        commandtimediff = (currentcommandtime - self.tlastcommand).total_seconds()
                        if commandtimediff > 0.1:
                            if abs(abs(commandedazratelast) - abs(azrate)) > 0.00001:
                                #Watch out for stale coordinates plus prediction lock
                                if trackSettings.predictionLock is True and staletelcoords is True:
                                    pass
                                else:
                                    #If we sent a command, update the last commanded time
                                    self.tlastcommand = datetime.datetime.utcnow()
                                    self.tel.MoveAxis(0, azrate)
                                    commandedazratelast = azrate
                            if abs(abs(commandedaltratelast) - abs(altrate)) > 0.00001:
                                #Watch out for stale coordinates plus prediction lock
                                if trackSettings.predictionLock is True and staletelcoords is True:
                                    pass
                                else:
                                    #If we sent a command, update the last commanded time
                                    self.tlastcommand = datetime.datetime.utcnow()
                                    self.tel.MoveAxis(1, altrate)
                                    commandedaltratelast = altrate
                            azratelast = azrate
                            altratelast = altrate
                        #else:
                        #    self.tel.MoveAxis(0, azratelast)
                        #    self.tel.MoveAxis(1, altratelast)
                        altcorrect = 0
                        azcorrect = 0
                if trackSettings.telescopetype == 'LX200':
                    d = datetime.datetime.utcnow()
                    if trackSettings.mounttype == 'AltAz':
                        if firstslew is True:
                            firstslew = False
                            if self.altout < 0:
                                altforrad = 0.01
                            else:
                                altforrad = self.altout
                            azforrad = self.azout
                            launchobserver = ephem.Observer()
                            launchobserver.lat = (str(self.entryLat.get()))
                            launchobserver.lon = (str(self.entryLon.get()))
                            launchobserver.date = datetime.datetime.utcnow()
                            
                            LSTnow = ephem.degrees(launchobserver.sidereal_time())
                            lpdec = math.degrees(math.asin(math.sin(math.radians(altforrad))*math.sin(math.radians(float(self.entryLat.get())))+math.cos(math.radians(altforrad))*math.cos(math.radians(float(self.entryLat.get())))*math.cos(math.radians(azforrad))))
                            lpH = math.degrees(math.acos((math.sin(math.radians(altforrad))-math.sin(math.radians(float(self.entryLat.get())))*math.sin(math.radians(lpdec)))/(math.cos(math.radians(float(self.entryLat.get())))*math.cos(math.radians(lpdec)))))
                            #lpH = math.degrees(math.atan2((-1*math.cos(math.radians(altforrad))*math.cos(math.radians(float(self.entryLat.get())))*math.sin(math.radians(azforrad))),(math.sin(math.radians(altforrad))-math.sin(math.radians(float(self.entryLat.get())))*math.sin(lpdec))))
                            if math.sin(math.radians(azforrad)) > 0:
                                lpH = lpH - 360
                            lpH = lpH * -1
                            lpra = float(LSTnow) - lpH
                            if lpra > 360:
                                lpra = lpra - 360
                            lpra = math.radians(lpra)
                            lpdec = math.radians(lpdec)
                            self.raddec = lpdec
                            self.radra = lpra
                            self.rad_to_sexagesimal_ra()
                            targetcoordra = str(':Sr ' + str(self.ra_h)+'*'+str(self.ra_m)+':'+str(int(self.ra_s))+'#')
                            targetcoorddec = str(':Sd ' + str(self.dec_d)+'*'+str(self.dec_m)+':'+str(int(self.dec_s))+'#')
                            print(targetcoordra, targetcoorddec, lpH, str(launchobserver.sidereal_time()), ephem.degrees(launchobserver.sidereal_time()), altforrad, azforrad)
                            self.ser.write(str.encode(targetcoordra))
                            self.ser.write(str.encode(targetcoorddec))
                            self.ser.write(str.encode(':CM#'))
                            self.resp = self.ser.read()
                            self.resp = self.resp.decode("utf-8", errors="ignore")
                            try:
                                while self.resp[-1] != '#':
                                    self.resp += self.ser.read().decode("utf-8", errors="ignore")
                            except:
                                print('Unable to read line')
                                self.textbox.insert(END, 'Unable to read line.\n')
                                self.textbox.see('end')
                            print(self.resp)
                        joy0 = self.joysticks[0].get_axis(0)
                        joy1 = self.joysticks[0].get_axis(1)
                        joy2 = self.joysticks[0].get_axis(2)
                        throttle = ((joy2*-1.0)+1.0)/2.0
                        azrate += ((joy0*throttle)*self.axis0rate)*0.01 
                        altrate += ((joy1*throttle*-1)*self.axis0rate)*0.01
                        if self.joyxrev.get() == 1:
                            azrate = azrate*-1
                        if self.joyyrev.get() == 1:
                            altrate = altrate*-1                  
                        #joybutton2 = self.joysticks[0].get_button(4)
                        #if joybutton2 == 1 and lastbutton2 == 0:
                        #    azrate = 0
                        #    altrate = 0
                        #    lastbutton2 = 1
                        #if joybutton2 == 0 and lastbutton2 == 1:
                        #    lastbutton2 = 0
                        #sataz = self.azout + 180 + azrate
                        sataz = self.azout + azrate
                        if sataz > 360:
                            sataz = sataz - 360
                        sataz = math.radians(sataz)
                        self.radaz = sataz
                        altforrad = self.altout + altrate
                        self.radalt = math.radians(altforrad)
                        if i > 30:
                            try:
                                self.LX200_alt_degrees()
                            except:
                                pass
                            self.LX200_alt_degrees()
                            currentalt = math.radians(self.telalt)
                            self.LX200_az_degrees()
                            currentaz = math.radians(self.telaz)
                            altdiff = self.radalt - currentalt
                            azdiff = self.radaz - currentaz
                            altcorrect = altcorrect + (altdiff)
                            azcorrect = azcorrect + (azdiff)
                            totaldiff = math.sqrt(altdiff**2 + azdiff**2)
                            i = 0
                            print(math.degrees(totaldiff))
                            self.textbox.insert(END, str('Distance from target: ' + str(totaldiff) + '\n'))
                            self.textbox.see('end')
                            self.lasttotaldiff = totaldiff
                        
                        self.radaz = self.radaz + azcorrect
                        self.radalt = self.radalt + altcorrect
                        altforrad = math.degrees(self.radalt)
                        if altforrad < 0:
                            altforrad = 0
                            self.radalt = math.radians(altforrad)
                        
                        self.rad_to_sexagesimal_alt()
                        targetcoordaz = str(':Sz ' + str(self.az_d)+'*'+str(self.az_m)+':'+str(int(self.az_s))+'#')
                        targetcoordalt = str(':Sa ' + str(self.alt_d)+'*'+str(self.alt_m)+':'+str(int(self.alt_s))+'#')
                        self.ser.write(str.encode(targetcoordaz))
                        self.ser.write(str.encode(targetcoordalt))
                        self.ser.write(str.encode(':MA#'))
            if trackSettings.objectfollow is True:   
                #Do focuser commands even if we're following with the viewfinder
                if trackSettings.joystickconnected is False:
                    pygame.event.pump()
                    joyfocus1 = self.joysticks[0].get_button(trackSettings.focuserbutton1)
                    joyfocus2 = self.joysticks[0].get_button(trackSettings.focuserbutton2)
                    if trackSettings.focuserconnected is True:
                        try:
                            if joyfocus1 == 1:
                                self.focuserSerial.write(b'button1\n')
                            elif joyfocus2 == 1:
                                self.focuserSerial.write(b'button2\n')
                            else:
                                self.focuserSerial.write(b'stopbutton\n')
                        except Exception as e:
                            print(e)
                    elif trackSettings.ASCOMFocuser is True:
                        try:
                            if joyfocus1 == 1:
                                if trackSettings.focuserabsolute is True:
                                    currentpos = self.ASCOMFocuser.Position
                                    newpos = currentpos + trackSettings.maxfocuserstep
                                    self.ASCOMFocuser.Move(newpos)
                                else:
                                    self.ASCOMFocuser.Move(int(trackSettings.maxfocuserstep))
                            elif joyfocus2 == 1:
                                if trackSettings.focuserabsolute is True:
                                    currentpos = self.ASCOMFocuser.Position
                                    newpos = currentpos - trackSettings.maxfocuserstep
                                    self.ASCOMFocuser.Move(newpos)
                                else:
                                    self.ASCOMFocuser.Move(int(-1*trackSettings.maxfocuserstep))
                            elif trackSettings.haltcompatible is True:
                                try:
                                    self.ASCOMFocuser.Halt()
                                except:
                                    pass
                        except Exception as e:
                            print(e)
                else:
                    joy0 = self.remotejoy0
                    joy1 = self.remotejoy1
                    throttle = self.remotethrottle
                    joybutton2 = self.remotejoybutton2
                    joybutton = self.remotejoybutton1
                    joybutton3 = self.remotejoybutton3
                    joybutton4 = self.remotejoybutton4
                    joybutton5 = self.remotejoybutton5
                    try:
                        if trackSettings.focuserconnected is True:
                            if joybutton4 == 1:
                                self.focuserSerial.write(b'button1\n')
                            elif joybutton5 == 1:
                                self.focuserSerial.write(b'button2\n')
                            else:
                                self.focuserSerial.write(b'stopbutton\n')
                        elif trackSettings.ASCOMFocuser is True:
                            try:
                                if joybutton4 == 1:
                                    if trackSettings.focuserabsolute is True:
                                        currentpos = self.ASCOMFocuser.Position
                                        newpos = currentpos + trackSettings.maxfocuserstep
                                        self.ASCOMFocuser.Move(newpos)
                                    else:
                                        self.ASCOMFocuser.Move(int(trackSettings.maxfocuserstep))
                                elif joybutton5 == 1:
                                    if trackSettings.focuserabsolute is True:
                                        currentpos = self.ASCOMFocuser.Position
                                        newpos = currentpos - trackSettings.maxfocuserstep
                                        self.ASCOMFocuser.Move(newpos)
                                    else:
                                        self.ASCOMFocuser.Move(int(-1*trackSettings.maxfocuserstep))
                                elif trackSettings.haltcompatible is True:
                                    try:
                                        self.ASCOMFocuser.Halt()
                                    except:
                                        pass
                            except Exception as e:
                                print(e)
                    except Exception as e:
                        print(e)
                if trackSettings.telescopetype == 'Autostar':
                    time.sleep(0.01)
                    d = datetime.datetime.utcnow()
                    if trackSettings.mounttype == 'AltAz':
                        self.LX200_alt_degrees()
                        self.LX200_alt_degrees()
                        self.LX200_az_degrees()
                        currentalt = math.radians(self.telalt)
                        currentaz = math.radians(self.telaz)
                        currentaltdegrees = math.degrees(currentalt)
                        currentazdegrees = math.degrees(currentaz)
                        #if self.logtelpos.get() == 1:
                        #    log = open('ufologtelpos.txt','a')
                        #    if math.degrees(currentaz) < 0:
                        #        reportaz = math.degrees(currentaz) + 360
                        #    elif math.degrees(currentaz) > 360:
                        #        reportaz = math.degrees(currentaz) - 360
                        #    else:
                        #        reportaz = math.degrees(currentaz)
                        #    log.write(str(self.dnow)+','+str(math.degrees(currentalt))+','+str(reportaz)+'\n')
                        #    log.close()
                        if self.dnow > self.dlast:
                            elapsedtime = self.dnow-self.dlast
                            if trackSettings.foundtarget is True:
                                objectvertical = -1 * ((self.targetY - trackSettings.mainviewY) * trackSettings.imagescale)
                                objecthorizontal = (self.targetX - trackSettings.mainviewX) * trackSettings.imagescale
                                objectangle = math.degrees(math.atan2(objectvertical, objecthorizontal)) - 90
                                objectangle2 = math.degrees(math.atan2(objectvertical, objecthorizontal))
                                objectdistance = math.sqrt((objecthorizontal**2) + (objectvertical**2) - 2 * (objectvertical * objecthorizontal * math.cos(math.radians(objectangle))))
                                trackSettings.objectverticalpixels = self.targetY
                                trackSettings.objecthorizontalpixels = self.targetX
                            else:
                                self.alt = self.alt+self.altratelast*elapsedtime.total_seconds()
                                self.az = self.az+self.azratelast*elapsedtime.total_seconds()
                                diffalt = (self.alt) - currentaltdegrees
                                diffaz = (self.az) - currentazdegrees
                                trackSettings.objectverticalpixels = int((trackSettings.mainviewY-(diffalt)/trackSettings.imagescale))
                                trackSettings.objecthorizontalpixels = int((trackSettings.mainviewX+(diffaz)/trackSettings.imagescale))
                                print (' ', self.alt, self.az, currentaltdegrees, currentazdegrees, trackSettings.objectverticalpixels, trackSettings.objecthorizontalpixels)
                                objectvertical = -1 * ((trackSettings.objectverticalpixels - trackSettings.mainviewY) * trackSettings.imagescale)
                                objecthorizontal = (trackSettings.objecthorizontalpixels - trackSettings.mainviewX) * trackSettings.imagescale
                                objectangle = math.degrees(math.atan2(objectvertical, objecthorizontal)) - 90
                                objectangle2 = math.degrees(math.atan2(objectvertical, objecthorizontal))
                                objectdistance = math.sqrt((objecthorizontal**2) + (objectvertical**2) - 2 * (objectvertical * objecthorizontal * math.cos(math.radians(objectangle))))
                            #try:
                            if trackSettings.foundtarget is True:
                                try:
                                    objectalt = 90 - math.degrees(math.acos(math.cos(math.radians(objectdistance)) * math.cos(math.radians(90 - currentaltdegrees)) + math.sin(math.radians(objectdistance)) * math.sin(math.radians(90 - currentaltdegrees)) * math.cos(math.radians(objectangle))))
                                    diffinaz = math.degrees(math.acos((math.cos(math.radians(objectdistance)) - math.cos(math.radians(90 - currentaltdegrees)) * math.cos(math.radians(90 - objectalt))) / (math.sin(math.radians(90 - currentaltdegrees)) * math.sin(math.radians(90 - objectalt)))))
                                except:
                                    pass
                                if math.fabs(objectangle2) > 90:
                                    diffinaz = -1 * diffinaz
                                try:
                                    objectaz = diffinaz + currentazdegrees
                                except:
                                    objectaz = currentazdegrees
                                if firstslew is True:
                                    firstslew = False
                                    for i in range (0,10):
                                        self.predictedCoords = self.kfObj.Estimate(objectaz, objectalt)
                                        objectaz2 = float(self.predictedCoords[0])
                                        objectalt2 = float(self.predictedCoords[1])
                                        #print(objectaz2, objectalt2, objectaz, objectalt)
                                elif abs(self.az2 - objectaz) > 90:
                                    for i in range (0,10):
                                        self.predictedCoords = self.kfObj.Estimate(objectaz, objectalt)
                                        objectaz2 = float(self.predictedCoords[0])
                                        objectalt2 = float(self.predictedCoords[1])
                                self.predictedCoords = self.kfObj.Estimate(objectaz, objectalt)
                                objectaz = float(self.predictedCoords[0])
                                objectalt = float(self.predictedCoords[1])
                                #check if we're crossing north meridian and correct if so
                                headingwestaz = objectaz - (currentazdegrees+360)
                                headingeastaz = (objectaz+360) - currentazdegrees
                                diffinaz = objectaz - currentazdegrees
                                if abs(headingwestaz) < abs(diffinaz):
                                    diffinaz = headingwestaz
                                if abs(headingeastaz) < abs(diffinaz):
                                    diffinaz = headingeastaz
                                altdiff = math.degrees(math.radians(objectalt) - currentalt)
                                azdiff = diffinaz
                                totaldiff = math.sqrt(altdiff**2 + azdiff**2)
                                self.alt2 = objectalt
                                self.az2 = objectaz
                                if self.pretrack is True:
                                    self.az = self.az2
                                    self.alt = self.alt2
                                azrate = (self.az2 - self.az)/elapsedtime.total_seconds()
                                headingwestaz = (self.az - (self.az2+360))/elapsedtime.total_seconds()
                                headingeastaz = ((self.az+360) - self.az2)/elapsedtime.total_seconds()
                                if abs(headingwestaz) < abs(azrate):
                                    azrate = headingwestaz
                                if abs(headingeastaz) < abs(azrate):
                                    azrate = headingeastaz
                                altrate = (self.alt2 - self.alt)/elapsedtime.total_seconds()
                                #if math.fabs(self.diffazlast) < math.fabs(azdiff):
                                if self.pretrack is True:
                                    self.azratelast = azrate
                                    azaccel = 0
                                if trackSettings.foundtarget is True:
                                    azaccel = azrate - self.azratelast
                                    self.azratelast = azrate
                                azrate = azrate + (azdiff*trackSettings.aggression) 
                                #if math.fabs(self.diffaltlast) < math.fabs(altdiff):
                                if self.pretrack is True:
                                    self.altratelast = altrate
                                    altaccel = 0
                                #we're done with setup for the first pass through the loop so dump pretrack
                                    self.pretrack = False
                                if trackSettings.foundtarget is True:
                                    altaccel = altrate - self.altratelast
                                    self.altratelast = altrate
                                altrate = altrate + (altdiff*trackSettings.aggression)

                        if azrate > self.axis0rate:
                            azrate = self.axis0rate
                        if azrate < (-1*self.axis0rate):
                            azrate = (-1*self.axis0rate)
                        if altrate > self.axis1rate:
                            altrate = self.axis1rate
                        if altrate < (-1*self.axis1rate):
                            altrate = (-1*self.axis1rate)
                        altrate = round(altrate, 4)
                        azrate = round(azrate, 4)
                        if throttle < 0.001:
                            self.ser.write(str.encode(':Q#'))
                        else:
                            self.textbox.insert(END, str(' Altrate: ' + str(altrate) + ' Azrate: ' + str(azrate) + '\n'))
                            self.textbox.see('end')
                            self.ser.write(str.encode(str(':RA'+str(azrate)+'#')))
                            self.ser.write(str.encode(str(':RE'+str(altrate)+'#')))
                            self.ser.write(str.encode(':Me#'))
                            self.ser.write(str.encode(':Mn#'))
                        time.sleep(0.1)
                        if trackSettings.foundtarget is True:
                            self.az = self.az2
                            self.alt = self.alt2
                if trackSettings.telescopetype == 'ASCOM':
                    time.sleep(0.01)
                    d = datetime.datetime.utcnow()
                    if trackSettings.mounttype == 'AltAz':
                        ref_tel_alt = self.tel.Altitude
                        ref_tel_az = self.tel.Azimuth
                        weighted_avg_alt_sep, weighted_avg_az_sep = self.errorWeightedAverage(ref_tel_alt, ref_tel_az)
                        currentaz = ref_tel_az + weighted_avg_az_sep
                        currentalt = ref_tel_alt + weighted_avg_alt_sep
                        currentaltdegrees = currentalt
                        currentazdegrees = currentaz
                        if self.dnow > self.dlast:
                            elapsedtime = self.dnow-self.dlast
                            currentalt = math.radians(currentalt)
                            currentaz = math.radians(currentaz)
                            if trackSettings.foundtarget is True:
                                objectvertical = -1 * ((self.targetY - trackSettings.mainviewY) * trackSettings.imagescale)
                                objecthorizontal = (self.targetX - trackSettings.mainviewX) * trackSettings.imagescale
                                objectangle = math.degrees(math.atan2(objectvertical, objecthorizontal)) - 90
                                objectangle2 = math.degrees(math.atan2(objectvertical, objecthorizontal))
                                objectdistance = math.sqrt((objecthorizontal**2) + (objectvertical**2) - 2 * (objectvertical * objecthorizontal * math.cos(math.radians(objectangle))))
                                trackSettings.objectverticalpixels = self.targetY
                                trackSettings.objecthorizontalpixels = self.targetX
                            else:
                                self.alt = self.alt+self.altratelast*elapsedtime.total_seconds()
                                self.az = self.az+self.azratelast*elapsedtime.total_seconds()
                                diffalt = (self.alt) - currentaltdegrees
                                diffaz = (self.az) - currentazdegrees
                                trackSettings.objectverticalpixels = int((trackSettings.mainviewY-(diffalt)/trackSettings.imagescale))
                                trackSettings.objecthorizontalpixels = int((trackSettings.mainviewX+(diffaz)/trackSettings.imagescale))
                                print (' ', self.alt, self.az, currentaltdegrees, currentazdegrees, trackSettings.objectverticalpixels, trackSettings.objecthorizontalpixels)
                                objectvertical = -1 * ((trackSettings.objectverticalpixels - trackSettings.mainviewY) * trackSettings.imagescale)
                                objecthorizontal = (trackSettings.objecthorizontalpixels - trackSettings.mainviewX) * trackSettings.imagescale
                                objectangle = math.degrees(math.atan2(objectvertical, objecthorizontal)) - 90
                                objectangle2 = math.degrees(math.atan2(objectvertical, objecthorizontal))
                                objectdistance = math.sqrt((objecthorizontal**2) + (objectvertical**2) - 2 * (objectvertical * objecthorizontal * math.cos(math.radians(objectangle))))
                            #try:
                            if trackSettings.foundtarget is True:
                                try:
                                    objectalt = 90 - math.degrees(math.acos(math.cos(math.radians(objectdistance)) * math.cos(math.radians(90 - currentaltdegrees)) + math.sin(math.radians(objectdistance)) * math.sin(math.radians(90 - currentaltdegrees)) * math.cos(math.radians(objectangle))))
                                    diffinaz = math.degrees(math.acos((math.cos(math.radians(objectdistance)) - math.cos(math.radians(90 - currentaltdegrees)) * math.cos(math.radians(90 - objectalt))) / (math.sin(math.radians(90 - currentaltdegrees)) * math.sin(math.radians(90 - objectalt)))))
                                except:
                                    pass
                                if math.fabs(objectangle2) > 90:
                                    diffinaz = -1 * diffinaz
                                try:
                                    objectaz = diffinaz + currentazdegrees
                                except:
                                    objectaz = currentazdegrees
                                #Check if this is the first time we've locked onto a target and initialize the karman filter so it doesn't flip out at first.
                                if firstslew is True:
                                    firstslew = False
                                    for i in range (0,10):
                                        self.predictedCoords = self.kfObj.Estimate(objectaz, objectalt)
                                        objectaz2 = float(self.predictedCoords[0])
                                        objectalt2 = float(self.predictedCoords[1])
                                        #print(objectaz2, objectalt2, objectaz, objectalt)
                                elif abs(self.az2 - objectaz) > 90:
                                    for i in range (0,10):
                                        self.predictedCoords = self.kfObj.Estimate(objectaz, objectalt)
                                        objectaz2 = float(self.predictedCoords[0])
                                        objectalt2 = float(self.predictedCoords[1])
                                self.predictedCoords = self.kfObj.Estimate(objectaz, objectalt)
                                objectaz = float(self.predictedCoords[0])
                                objectalt = float(self.predictedCoords[1])
                                #check if we're crossing north meridian and correct if so
                                headingwestaz = objectaz - (currentazdegrees+360)
                                headingeastaz = (objectaz+360) - currentazdegrees
                                diffinaz = objectaz - currentazdegrees
                                if abs(headingwestaz) < abs(diffinaz):
                                    diffinaz = headingwestaz
                                if abs(headingeastaz) < abs(diffinaz):
                                    diffinaz = headingeastaz
                                altdiff = math.degrees(math.radians(objectalt) - currentalt)
                                azdiff = diffinaz
                                totaldiff = math.sqrt(altdiff**2 + azdiff**2)
                                self.alt2 = objectalt
                                self.az2 = objectaz
                                if self.pretrack is True:
                                    self.az = self.az2
                                    self.alt = self.alt2
                                azrate = (self.az2 - self.az)/elapsedtime.total_seconds()
                                headingwestaz = (self.az - (self.az2+360))/elapsedtime.total_seconds()
                                headingeastaz = ((self.az+360) - self.az2)/elapsedtime.total_seconds()
                                if abs(headingwestaz) < abs(azrate):
                                    azrate = headingwestaz
                                if abs(headingeastaz) < abs(azrate):
                                    azrate = headingeastaz
                                altrate = (self.alt2 - self.alt)/elapsedtime.total_seconds()
                                #if math.fabs(self.diffazlast) < math.fabs(azdiff):
                                if self.pretrack is True:
                                    self.azratelast = azrate
                                    azaccel = 0
                                if trackSettings.foundtarget is True:
                                    azaccel = azrate - self.azratelast
                                    self.azratelast = azrate
                                azrate =  float(self.azrateout) + (azdiff*trackSettings.aggression) 
                                #if math.fabs(self.diffaltlast) < math.fabs(altdiff):
                                if self.pretrack is True:
                                    self.altratelast = altrate
                                    altaccel = 0
                                #we're done with setup for the first pass through the loop so dump pretrack
                                    self.pretrack = False
                                if trackSettings.foundtarget is True:
                                    altaccel = altrate - self.altratelast
                                    self.altratelast = altrate
                                altrate =  float(self.altrateout) + (altdiff*trackSettings.aggression)
                                if azrate > self.axis0rate:
                                    azrate = self.axis0rate
                                if azrate < (-1*self.axis0rate):
                                    azrate = (-1*self.axis0rate)
                                if altrate > self.axis1rate:
                                    altrate = self.axis1rate
                                if altrate < (-1*self.axis1rate):
                                    altrate = (-1*self.axis1rate)
                                #print('azdiff, altdiff, objaz, objalt, currentaz, currentalt', azdiff, altdiff, self.az2, self.alt2, currentazdegrees, currentaltdegrees, end='\r')
                                self.textbox.insert(END, str('Delta Az: ' + str(azdiff) + ' Delta Alt: ' + str(altdiff) + '\n'))
                                self.textbox.see('end')
                            else:
                                azrate =  float(self.azrateout) + (0.000001*random.randrange(0, 5, 1))
                                altrate =  float(self.altrateout) + (0.000001*random.randrange(0, 5, 1))
                                if azrate > self.axis0rate:
                                    azrate = self.axis0rate
                                if azrate < (-1*self.axis0rate):
                                    azrate = (-1*self.axis0rate)
                                if altrate > self.axis1rate:
                                    altrate = self.axis1rate
                                if altrate < (-1*self.axis1rate):
                                    altrate = (-1*self.axis1rate)
                            #Check command rate and issue command if enough time has passed
                            currentcommandtime = datetime.datetime.utcnow()
                            commandtimediff = (currentcommandtime - self.tlastcommand).total_seconds()
                            if commandtimediff > 0.1:
                                #If we sent a command, update the last commanded time
                                self.tlastcommand = datetime.datetime.utcnow()
                                if abs(abs(commandedazratelast) - abs(azrate)) > 0.001:
                                    self.tel.MoveAxis(0, azrate)
                                    commandedazratelast = azrate
                                if abs(abs(commandedaltratelast) - abs(altrate)) > 0.001:
                                    self.tel.MoveAxis(1, altrate)
                                    commandedaltratelast = altrate
                                azratelast = azrate
                                altratelast = altrate
                            if trackSettings.foundtarget is True:
                                self.az = self.az2
                                self.alt = self.alt2
                            #except:
                            #    print('Failed to do the math.')
        
    def start_joy_track(self):
        if trackSettings.joytracking is False:
            trackSettings.joytracking = True
        else:
            trackSettings.joytracking = False
            self.startButton4.configure(text='Start Joystick Tracking')
        if trackSettings.tracking is False:
            print('Connect the Scope First!')
            self.textbox.insert(END, 'Connect the Scope First!\n')
            self.textbox.see('end')
        if self.collect_images is False:
            print('Start Camera First!')
            self.textbox.insert(END, 'Start Camera First!\n')
            self.textbox.see('end')
        if len(self.joysticks) == 0 and trackSettings.joystickconnected is False:
            print('Connect Joystick And Restart Program!')
            self.textbox.insert(END, 'Connect Joystick And Restart Program!\n')
            self.textbox.see('end')
            trackSettings.joytracking = False
            self.startButton4.configure(text='Start Joystick Tracking')
        elif len(self.joysticks) > 0 and trackSettings.joystickconnected is False:
            self.joysticks[0].init()
        if trackSettings.tracking is True and self.collect_images is True and trackSettings.joytracking is True:
            self.trackthread = threading.Thread(target=self.track, daemon=True)
            self.startButton4.configure(text='Stop Joystick Tracking')
            self.trackthread.start()
        
    def track(self):
        holdrate = False
        lastbutton = 0
        lastbutton2 = 0
        altratelast = 0
        azratelast = 0
        altcorrect = 0
        azcorrect = 0
        altcorrectrunning = 0
        azcorrectrunning = 0
        deccorrect = 0
        racorrect = 0
        self.diffazlast = 0
        self.diffaltlast = 0
        self.diffralast = 0
        self.diffdeclast = 0
        self.lasttotaldiff = 0.0
        i = 0
        iterate = 0
        objectazrate = 0
        objectaltrate = 0
        commandedaltratelast = 0
        commandedazratelast = 0
        firstslew = True
        rateheld = False
        while trackSettings.joytracking is True and trackSettings.runningsimulation is False and trackSettings.runninglaunch is False:
            if trackSettings.objectfollow is False:
                self.dlast = self.dnow
                pygame.event.pump()
                i = i + 1
                if trackSettings.telescopetype == 'ASCOM':
                    d = datetime.datetime.utcnow()
                    if trackSettings.mounttype == 'AltAz':
                        if trackSettings.joystickconnected is False:
                            joy0 = self.joysticks[0].get_axis(0)
                            joy1 = self.joysticks[0].get_axis(1)
                            joy2 = self.joysticks[0].get_axis(2)
                            throttle = ((joy2*-1.0)+1.0)/2.0
                            #joybutton2 = self.joysticks[0].get_button(4)
                            #if joybutton2 == 1 and lastbutton2 == 0:
                            #    self.tel.MoveAxis(0, 0.0)
                            #    self.tel.MoveAxis(1, 0.0)
                            #    lastbutton2 = 1
                            #if joybutton2 == 0 and lastbutton2 == 1:
                            #    lastbutton2 = 0
                            joybutton = self.joysticks[0].get_button(trackSettings.holdratebutton)
                            joyfocus1 = self.joysticks[0].get_button(trackSettings.focuserbutton1)
                            joyfocus2 = self.joysticks[0].get_button(trackSettings.focuserbutton2)
                            if trackSettings.focuserconnected is True:
                                try:
                                    if joyfocus1 == 1:
                                        self.focuserSerial.write(b'button1\n')
                                    elif joyfocus2 == 1:
                                        self.focuserSerial.write(b'button2\n')
                                    else:
                                        self.focuserSerial.write(b'stopbutton\n')
                                except Exception as e:
                                    print(e)
                            elif trackSettings.ASCOMFocuser is True:
                                try:
                                    if joyfocus1 == 1:
                                        if trackSettings.focuserabsolute is True:
                                            currentpos = self.ASCOMFocuser.Position
                                            newpos = currentpos + trackSettings.maxfocuserstep
                                            self.ASCOMFocuser.Move(newpos)
                                        else:
                                            self.ASCOMFocuser.Move(int(trackSettings.maxfocuserstep))
                                    elif joyfocus2 == 1:
                                        if trackSettings.focuserabsolute is True:
                                            currentpos = self.ASCOMFocuser.Position
                                            newpos = currentpos - trackSettings.maxfocuserstep
                                            self.ASCOMFocuser.Move(newpos)
                                        else:
                                            self.ASCOMFocuser.Move(int(-1*trackSettings.maxfocuserstep))
                                    elif trackSettings.haltcompatible is True:
                                        try:
                                            self.ASCOMFocuser.Halt()
                                        except:
                                            pass
                                except Exception as e:
                                    print(e)
                        else:
                            joy0 = self.remotejoy0
                            joy1 = self.remotejoy1
                            throttle = self.remotethrottle
                            joybutton2 = self.remotejoybutton2
                            joybutton = self.remotejoybutton1
                            joybutton3 = self.remotejoybutton3
                            joybutton4 = self.remotejoybutton4
                            joybutton5 = self.remotejoybutton5
                            try:
                                if trackSettings.focuserconnected is True:
                                    if joybutton4 == 1:
                                        self.focuserSerial.write(b'button1\n')
                                    elif joybutton5 == 1:
                                        self.focuserSerial.write(b'button2\n')
                                    else:
                                        self.focuserSerial.write(b'stopbutton\n')
                                elif trackSettings.ASCOMFocuser is True:
                                    try:
                                        if joybutton4 == 1:
                                            if trackSettings.focuserabsolute is True:
                                                currentpos = self.ASCOMFocuser.Position
                                                newpos = currentpos + trackSettings.maxfocuserstep
                                                self.ASCOMFocuser.Move(newpos)
                                            else:
                                                self.ASCOMFocuser.Move(int(trackSettings.maxfocuserstep))
                                        elif joybutton5 == 1:
                                            if trackSettings.focuserabsolute is True:
                                                currentpos = self.ASCOMFocuser.Position
                                                newpos = currentpos - trackSettings.maxfocuserstep
                                                self.ASCOMFocuser.Move(newpos)
                                            else:
                                                self.ASCOMFocuser.Move(int(-1*trackSettings.maxfocuserstep))
                                        elif trackSettings.haltcompatible is True:
                                            try:
                                                self.ASCOMFocuser.Halt()
                                            except:
                                                pass
                                    except Exception as e:
                                        print(e)
                            except Exception as e:
                                print(e)
                            #print(self.remotejoy0,self.remotejoy1,self.remotethrottle)
                        #if joybutton2 == 1 and lastbutton2 == 0:
                        #    self.tel.MoveAxis(0, 0.0)
                        #    self.tel.MoveAxis(1, 0.0)
                        #    lastbutton2 = 1
                        #if joybutton2 == 0 and lastbutton2 == 1:
                        #    lastbutton2 = 0
                        if joybutton == 1 and lastbutton == 0:
                            if holdrate is False:
                                holdrate = True
                                self.HoldLabel.config(text='Hold Rate ON')
                            else:
                                holdrate = False
                                self.HoldLabel.config(text='Hold Rate OFF')
                            lastbutton = 1
                        if joybutton == 0 and lastbutton == 1:
                            lastbutton = 0
                        azrate = (joy0*throttle)*self.axis0rate
                        altrate = (joy1*throttle*-1)*self.axis0rate
                        if self.joyxrev.get() == 1:
                            azrate = azrate*-1
                        if self.joyyrev.get() == 1:
                            altrate = altrate*-1
                        self.textbox.insert(END, str('Az Rate: ' + str(azrate) + ' Alt Rate: ' + str(altrate) + '\n'))
                        self.textbox.see('end')
                        if azrate > 0.0:
                            azrate = azrate + (0.001*random.randrange(0, 5, 1))
                        if altrate > 0.0:
                            altrate = altrate + (0.001*random.randrange(0, 5, 1))
                        if azrate > self.axis0rate:
                            azrate = self.axis0rate
                        if azrate < (-1*self.axis0rate):
                            azrate = (-1*self.axis0rate)
                        if altrate > self.axis1rate:
                            altrate = self.axis1rate
                        if altrate < (-1*self.axis1rate):
                            altrate = (-1*self.axis1rate)
                        if throttle < 0.001:
                            azrate = 0.0
                            altrate = 0.0
                        if holdrate is False:
                            #Check command rate and issue command if enough time has passed
                            currentcommandtime = datetime.datetime.utcnow()
                            commandtimediff = (currentcommandtime - self.tlastcommand).total_seconds()
                            if commandtimediff > 0.1:
                                #If we sent a command, update the last commanded time
                                self.tlastcommand = datetime.datetime.utcnow()
                                if abs(abs(commandedazratelast) - abs(azrate)) > 0.001:
                                    self.tel.MoveAxis(0, azrate)
                                    commandedazratelast = azrate
                                if abs(abs(commandedaltratelast) - abs(altrate)) > 0.001:
                                    self.tel.MoveAxis(1, altrate)
                                    commandedaltratelast = altrate
                                azratelast = azrate
                                altratelast = altrate
                            rateheld = False
                        else:
                            if rateheld is False:
                                #Check command rate and issue command if enough time has passed
                                currentcommandtime = datetime.datetime.utcnow()
                                commandtimediff = (currentcommandtime - self.tlastcommand).total_seconds()
                                if commandtimediff > 0.1:
                                    #If we sent a command, update the last commanded time
                                    self.tlastcommand = datetime.datetime.utcnow()
                                    self.tel.MoveAxis(0, azratelast)
                                    self.tel.MoveAxis(1, altratelast)
                            rateheld = True
                        altcorrect = 0
                        azcorrect = 0
                    if trackSettings.mounttype == 'Eq':
                        if trackSettings.joystickconnected is False:
                            joy0 = self.joysticks[0].get_axis(0)
                            joy1 = self.joysticks[0].get_axis(1)
                            joy2 = self.joysticks[0].get_axis(2)
                            throttle = ((joy2*-1.0)+1.0)/2.0
                            #joybutton2 = self.joysticks[0].get_button(4)
                            #if joybutton2 == 1 and lastbutton2 == 0:
                            #    self.tel.MoveAxis(0, 0.0)
                            #    self.tel.MoveAxis(1, 0.0)
                            #    lastbutton2 = 1
                            #if joybutton2 == 0 and lastbutton2 == 1:
                            #    lastbutton2 = 0
                            joybutton = self.joysticks[0].get_button(trackSettings.holdratebutton)
                        else:
                            joy0 = self.remotejoy0
                            joy1 = self.remotejoy1
                            throttle = self.remotethrottle
                            joybutton2 = self.remotejoybutton2
                            joybutton = self.remotejoybutton1
                            joybutton3 = self.remotejoybutton3
                            joybutton4 = self.remotejoybutton4
                            joybutton5 = self.remotejoybutton5
                        #if joybutton2 == 1 and lastbutton2 == 0:
                        #   self.tel.MoveAxis(0, 0.0)
                        #    self.tel.MoveAxis(1, 0.0)
                        #    lastbutton2 = 1
                        #if joybutton2 == 0 and lastbutton2 == 1:
                        #    lastbutton2 = 0
                        joybutton = self.joysticks[0].get_button(trackSettings.holdratebutton)
                        if joybutton == 1 and lastbutton == 0:
                            if holdrate is False:
                                holdrate = True
                                self.HoldLabel.config(text='Hold Rate ON')
                            else:
                                holdrate = False
                                self.HoldLabel.config(text='Hold Rate OFF')
                            lastbutton = 1
                        if joybutton == 0 and lastbutton == 1:
                            lastbutton = 0
                        azrate = (joy0*throttle)*self.axis0rate
                        altrate = (joy1*throttle*-1)*self.axis0rate
                        if self.joyxrev.get() == 1:
                            azrate = azrate*-1
                        if self.joyyrev.get() == 1:
                            altrate = altrate*-1
                        self.textbox.insert(END, str('Az Rate: ' + str(azrate) + 'Alt Rate: ' + str(altrate) + '\n'))
                        self.textbox.see('end')
                        if azrate > self.axis0rate:
                            azrate = self.axis0rate
                        if azrate < (-1*self.axis0rate):
                            azrate = (-1*self.axis0rate)
                        if altrate > self.axis1rate:
                            altrate = self.axis1rate
                        if altrate < (-1*self.axis1rate):
                            altrate = (-1*self.axis1rate)
                        azrate = azrate + (0.001*random.randrange(0, 5, 1))
                        altrate = altrate + (0.001*random.randrange(0, 5, 1))
                        if holdrate is False:
                            #Check command rate and issue command if enough time has passed
                            currentcommandtime = datetime.datetime.utcnow()
                            commandtimediff = (currentcommandtime - self.tlastcommand).total_seconds()
                            if commandtimediff > 0.1:
                                #If we sent a command, update the last commanded time
                                self.tlastcommand = datetime.datetime.utcnow()
                                if abs(abs(commandedazratelast) - abs(azrate)) > 0.001:
                                    self.tel.MoveAxis(0, azrate)
                                    commandedazratelast = azrate
                                if abs(abs(commandedaltratelast) - abs(altrate)) > 0.001:
                                    self.tel.MoveAxis(1, altrate)
                                    commandedaltratelast = altrate
                                azratelast = azrate
                                altratelast = altrate
                            rateheld = False
                        else:
                            if rateheld is False:
                                #Check command rate and issue command if enough time has passed
                                currentcommandtime = datetime.datetime.utcnow()
                                commandtimediff = (currentcommandtime - self.tlastcommand).total_seconds()
                                if commandtimediff > 0.1:
                                    #If we sent a command, update the last commanded time
                                    self.tlastcommand = datetime.datetime.utcnow()
                                    self.tel.MoveAxis(0, azratelast)
                                    self.tel.MoveAxis(1, altratelast)
                            rateheld = True
                        altcorrect = 0
                        azcorrect = 0
                    time.sleep(0.001)
                if trackSettings.telescopetype == 'LX200':
                    if trackSettings.mounttype == 'AltAz':
                        altcorrectrunning = 0
                        azcorrectrunning = 0
                        if trackSettings.joystickconnected is False:
                            joy0 = self.joysticks[0].get_axis(0)
                            joy1 = self.joysticks[0].get_axis(1)
                            joy2 = self.joysticks[0].get_axis(2)
                            throttle = ((joy2*-1.0)+1.0)/2.0
                            #joybutton2 = self.joysticks[0].get_button(4)
                            #if joybutton2 == 1 and lastbutton2 == 0:
                            #    self.tel.MoveAxis(0, 0.0)
                            #    self.tel.MoveAxis(1, 0.0)
                            #    lastbutton2 = 1
                            #if joybutton2 == 0 and lastbutton2 == 1:
                            #    lastbutton2 = 0
                            joybutton = self.joysticks[0].get_button(trackSettings.holdratebutton)
                        else:
                            joy0 = self.remotejoy0
                            joy1 = self.remotejoy1
                            throttle = self.remotethrottle
                            joybutton2 = self.remotejoybutton2
                            joybutton = self.remotejoybutton1
                            joybutton3 = self.remotejoybutton3
                            joybutton4 = self.remotejoybutton4
                            joybutton5 = self.remotejoybutton5
                        if joybutton == 1 and lastbutton == 0:
                            if holdrate is False:
                                holdrate = True
                                self.HoldLabel.config(text='Hold Rate ON')
                            else:
                                holdrate = False
                                self.HoldLabel.config(text='Hold Rate OFF')
                            lastbutton = 1
                        if joybutton == 0 and lastbutton == 1:
                            lastbutton = 0
                        if holdrate is False:
                            throttle = ((joy2*-1.0)+1.0)/2.0
                            azrate = (joy0*throttle)*self.axis0rate
                            altrate = (joy1*throttle*-1)*self.axis0rate
                            altratelast = altrate
                            azratelast = azrate
                        else:
                            azrate = azratelast
                            altrate = altratelast
                        if self.joyxrev.get() == 1:
                            azrate = azrate*-1
                        if self.joyyrev.get() == 1:
                            altrate = altrate*-1
                        #self.textbox.insert(END, str('Az Rate: ' + str(azrate) + ' Alt Rate: ' + str(altrate) + '\n'))
                        #self.textbox.see('end')
                        if azrate > self.axis0rate:
                            azrate = self.axis0rate
                        if azrate < (-1*self.axis0rate):
                            azrate = (-1*self.axis0rate)
                        if altrate > self.axis1rate:
                            altrate = self.axis1rate
                        if altrate < (-1*self.axis1rate):
                            altrate = (-1*self.axis1rate)
                        altrate = round(altrate, 3)
                        azrate = round(azrate, 3)
                        self.LX200_alt_degrees()
                        self.LX200_alt_degrees()
                        self.LX200_az_degrees()
                        currentalt = math.radians(self.telalt)
                        currentaz = math.radians(self.telaz)
                        if self.logtelpos.get() == 1:
                            log = open('ufologtelpos.txt','a')
                            if math.degrees(currentaz) < 0:
                                reportaz = math.degrees(currentaz) + 360
                            elif math.degrees(currentaz) > 360:
                                reportaz = math.degrees(currentaz) - 360
                            else:
                                reportaz = math.degrees(currentaz)
                            log.write(str(self.dnow)+','+str(math.degrees(currentalt))+','+str(reportaz)+'\n')
                            log.close()
                        objectazlast = currentaz
                        objectaltlast = currentalt
                        self.radaz = currentaz + math.radians(azrate)
                        if math.degrees(self.radaz) > 360:
                            self.radaz = self.radaz - math.radians(360)
                        elif math.degrees(self.radaz) < 0:
                            self.radaz = self.radaz + math.radians(360)
                        self.radalt = currentalt + math.radians(altrate)
                        self.rad_to_sexagesimal_alt()
                        targetcoordaz = str(':Sz ' + str(self.az_d)+'*'+str(self.az_m)+':'+str(int(self.az_s))+'#')
                        targetcoordalt = str(':Sa ' + str(self.alt_d)+'*'+str(self.alt_m)+':'+str(int(self.alt_s))+'#')
                        if throttle < 0.001:
                            self.ser.write(str.encode(':Q#'))
                        else:
                            self.textbox.insert(END, str('Az: ' + str(math.degrees(self.radaz)) + ' Alt: ' + str(math.degrees(self.radalt)) + ' Altrate: ' + str(altrate) + ' Azrate: ' + str(azrate) + '\n'))
                            self.textbox.see('end')
                            self.ser.write(str.encode(targetcoordaz))
                            self.ser.write(str.encode(targetcoordalt))
                            self.ser.write(str.encode(':MA#'))
                        time.sleep(0.01)
                    if trackSettings.mounttype == 'Eq':
                        satra = self.sat.ra
                        self.radra = self.sat.ra
                        self.raddec = self.sat.dec
                        
                        self.LX200_dec_degrees()
                        self.LX200_dec_degrees()
                        currentdec = math.radians(self.teldec)
                        self.LX200_ra_degrees()
                        currentra = math.radians(self.telra)
                        decdiff = self.raddec - currentdec
                        radiff = self.radra - currentra
                        totaldiff = math.sqrt(decdiff**2 + radiff**2)
                        i = 0
                        print(math.degrees(totaldiff))
                        self.textbox.insert(END, str('Distance from target: ' + str(totaldiff) + '\n'))
                        self.textbox.see('end')
                        if self.lasttotaldiff < totaldiff:
                            deccorrect = deccorrect + (decdiff)
                            racorrect = racorrect + (radiff)
                        self.lasttotaldiff = totaldiff
                        
                        self.radra = self.radra + racorrect
                        self.raddec = self.raddec + deccorrect
                        
                        self.rad_to_sexagesimal_ra()
                        targetcoordra = str(':Sr ' + str(self.ra_h)+'*'+str(self.ra_m)+':'+str(int(self.ra_s))+'#')
                        targetcoorddec = str(':Sd ' + str(self.dec_d)+'*'+str(self.dec_m)+':'+str(int(self.dec_s))+'#')
                        self.ser.write(str.encode(targetcoordra))
                        self.ser.write(str.encode(targetcoorddec))
                        self.ser.write(str.encode(':MS#'))
                        print(targetcoordra, targetcoorddec)
                if trackSettings.telescopetype == 'Autostar':
                    if trackSettings.mounttype == 'AltAz':
                        altcorrectrunning = 0
                        azcorrectrunning = 0
                        if trackSettings.joystickconnected is False:
                            joy0 = self.joysticks[0].get_axis(0)
                            joy1 = self.joysticks[0].get_axis(1)
                            joy2 = self.joysticks[0].get_axis(2)
                            throttle = ((joy2*-1.0)+1.0)/2.0
                            #joybutton2 = self.joysticks[0].get_button(4)
                            #if joybutton2 == 1 and lastbutton2 == 0:
                            #    self.tel.MoveAxis(0, 0.0)
                            #    self.tel.MoveAxis(1, 0.0)
                            #    lastbutton2 = 1
                            #if joybutton2 == 0 and lastbutton2 == 1:
                            #    lastbutton2 = 0
                            joybutton = self.joysticks[0].get_button(trackSettings.holdratebutton)
                        else:
                            joy0 = self.remotejoy0
                            joy1 = self.remotejoy1
                            throttle = self.remotethrottle
                            joybutton2 = self.remotejoybutton2
                            joybutton = self.remotejoybutton1
                            joybutton3 = self.remotejoybutton3
                            joybutton4 = self.remotejoybutton4
                            joybutton5 = self.remotejoybutton5
                        if joybutton == 1 and lastbutton == 0:
                            if holdrate is False:
                                holdrate = True
                                self.HoldLabel.config(text='Hold Rate ON')
                            else:
                                holdrate = False
                                self.HoldLabel.config(text='Hold Rate OFF')
                            lastbutton = 1
                        if joybutton == 0 and lastbutton == 1:
                            lastbutton = 0
                        if holdrate is False:
                            throttle = ((joy2*-1.0)+1.0)/2.0
                            azrate = (joy0*throttle)*self.axis0rate
                            altrate = (joy1*throttle*-1)*self.axis0rate
                            altratelast = altrate
                            azratelast = azrate
                        else:
                            azrate = azratelast
                            altrate = altratelast
                        if self.joyxrev.get() == 1:
                            azrate = azrate*-1
                        if self.joyyrev.get() == 1:
                            altrate = altrate*-1
                        #self.textbox.insert(END, str('Az Rate: ' + str(azrate) + ' Alt Rate: ' + str(altrate) + '\n'))
                        #self.textbox.see('end')
                        if azrate > self.axis0rate:
                            azrate = self.axis0rate
                        if azrate < (-1*self.axis0rate):
                            azrate = (-1*self.axis0rate)
                        if altrate > self.axis1rate:
                            altrate = self.axis1rate
                        if altrate < (-1*self.axis1rate):
                            altrate = (-1*self.axis1rate)
                        altrate = round(altrate, 2)
                        azrate = round(azrate, 2)
                        #self.LX200_alt_degrees()
                        #self.LX200_alt_degrees()
                        #self.LX200_az_degrees()
                        #currentalt = math.radians(self.telalt)
                        #currentaz = math.radians(self.telaz)
                        #if self.logtelpos.get() == 1:
                        #    log = open('ufologtelpos.txt','a')
                        #    if math.degrees(currentaz) < 0:
                        #        reportaz = math.degrees(currentaz) + 360
                        #    elif math.degrees(currentaz) > 360:
                        #        reportaz = math.degrees(currentaz) - 360
                        #    else:
                        #        reportaz = math.degrees(currentaz)
                        #    log.write(str(self.dnow)+','+str(math.degrees(currentalt))+','+str(reportaz)+'\n')
                        #    log.close()
                        if throttle < 0.001:
                            self.ser.write(str.encode(':Q#'))
                        else:
                            self.textbox.insert(END, str(' Altrate: ' + str(altrate) + ' Azrate: ' + str(azrate) + '\n'))
                            self.textbox.see('end')
                            self.ser.write(str.encode(str(':RA'+str(azrate)+'#')))
                            self.ser.write(str.encode(str(':RE'+str(altrate)+'#')))
                            self.ser.write(str.encode(':Me#'))
                            self.ser.write(str.encode(':Mn#'))
                        time.sleep(0.1)
                                    
            if trackSettings.objectfollow is True:
                #Do focuser commands even if we're following with the viewfinder
                if trackSettings.joystickconnected is False:
                    pygame.event.pump()
                    joyfocus1 = self.joysticks[0].get_button(trackSettings.focuserbutton1)
                    joyfocus2 = self.joysticks[0].get_button(trackSettings.focuserbutton2)
                    if trackSettings.focuserconnected is True:
                        try:
                            if joyfocus1 == 1:
                                self.focuserSerial.write(b'button1\n')
                            elif joyfocus2 == 1:
                                self.focuserSerial.write(b'button2\n')
                            else:
                                self.focuserSerial.write(b'stopbutton\n')
                        except Exception as e:
                            print(e)
                    elif trackSettings.ASCOMFocuser is True:
                        try:
                            if joyfocus1 == 1:
                                if trackSettings.focuserabsolute is True:
                                    currentpos = self.ASCOMFocuser.Position
                                    newpos = currentpos + trackSettings.maxfocuserstep
                                    self.ASCOMFocuser.Move(newpos)
                                else:
                                    self.ASCOMFocuser.Move(int(trackSettings.maxfocuserstep))
                            elif joyfocus2 == 1:
                                if trackSettings.focuserabsolute is True:
                                    currentpos = self.ASCOMFocuser.Position
                                    newpos = currentpos - trackSettings.maxfocuserstep
                                    self.ASCOMFocuser.Move(newpos)
                                else:
                                    self.ASCOMFocuser.Move(int(-1*trackSettings.maxfocuserstep))
                            elif trackSettings.haltcompatible is True:
                                try:
                                    self.ASCOMFocuser.Halt()
                                except:
                                    pass
                        except Exception as e:
                            print(e)
                else:
                    joy0 = self.remotejoy0
                    joy1 = self.remotejoy1
                    throttle = self.remotethrottle
                    joybutton2 = self.remotejoybutton2
                    joybutton = self.remotejoybutton1
                    joybutton3 = self.remotejoybutton3
                    joybutton4 = self.remotejoybutton4
                    joybutton5 = self.remotejoybutton5
                    try:
                        if trackSettings.focuserconnected is True:
                            if joybutton4 == 1:
                                self.focuserSerial.write(b'button1\n')
                            elif joybutton5 == 1:
                                self.focuserSerial.write(b'button2\n')
                            else:
                                self.focuserSerial.write(b'stopbutton\n')
                        elif trackSettings.ASCOMFocuser is True:
                            try:
                                if joybutton4 == 1:
                                    if trackSettings.focuserabsolute is True:
                                        currentpos = self.ASCOMFocuser.Position
                                        newpos = currentpos + trackSettings.maxfocuserstep
                                        self.ASCOMFocuser.Move(newpos)
                                    else:
                                        self.ASCOMFocuser.Move(int(trackSettings.maxfocuserstep))
                                elif joybutton5 == 1:
                                    if trackSettings.focuserabsolute is True:
                                        currentpos = self.ASCOMFocuser.Position
                                        newpos = currentpos - trackSettings.maxfocuserstep
                                        self.ASCOMFocuser.Move(newpos)
                                    else:
                                        self.ASCOMFocuser.Move(int(-1*trackSettings.maxfocuserstep))
                                elif trackSettings.haltcompatible is True:
                                    try:
                                        self.ASCOMFocuser.Halt()
                                    except:
                                        pass
                            except Exception as e:
                                print(e)
                    except Exception as e:
                        print(e)
                if trackSettings.telescopetype == 'Autostar':
                    time.sleep(0.01)
                    d = datetime.datetime.utcnow()
                    if trackSettings.mounttype == 'AltAz':
                        self.LX200_alt_degrees()
                        self.LX200_alt_degrees()
                        self.LX200_az_degrees()
                        currentalt = math.radians(self.telalt)
                        currentaz = math.radians(self.telaz)
                        currentaltdegrees = math.degrees(currentalt)
                        currentazdegrees = math.degrees(currentaz)
                        #if self.logtelpos.get() == 1:
                        #    log = open('ufologtelpos.txt','a')
                        #    if math.degrees(currentaz) < 0:
                        #        reportaz = math.degrees(currentaz) + 360
                        #    elif math.degrees(currentaz) > 360:
                        #        reportaz = math.degrees(currentaz) - 360
                        #    else:
                        #        reportaz = math.degrees(currentaz)
                        #    log.write(str(self.dnow)+','+str(math.degrees(currentalt))+','+str(reportaz)+'\n')
                        #    log.close()
                        if self.dnow > self.dlast:
                            elapsedtime = self.dnow-self.dlast
                            if trackSettings.foundtarget is True:
                                objectvertical = -1 * ((self.targetY - trackSettings.mainviewY) * trackSettings.imagescale)
                                objecthorizontal = (self.targetX - trackSettings.mainviewX) * trackSettings.imagescale
                                objectangle = math.degrees(math.atan2(objectvertical, objecthorizontal)) - 90
                                objectangle2 = math.degrees(math.atan2(objectvertical, objecthorizontal))
                                objectdistance = math.sqrt((objecthorizontal**2) + (objectvertical**2) - 2 * (objectvertical * objecthorizontal * math.cos(math.radians(objectangle))))
                                trackSettings.objectverticalpixels = self.targetY
                                trackSettings.objecthorizontalpixels = self.targetX
                            else:
                                self.alt = self.alt+self.altratelast*elapsedtime.total_seconds()
                                self.az = self.az+self.azratelast*elapsedtime.total_seconds()
                                diffalt = (self.alt) - currentaltdegrees
                                diffaz = (self.az) - currentazdegrees
                                trackSettings.objectverticalpixels = int((trackSettings.mainviewY-(diffalt)/trackSettings.imagescale))
                                trackSettings.objecthorizontalpixels = int((trackSettings.mainviewX+(diffaz)/trackSettings.imagescale))
                                print (' ', self.alt, self.az, currentaltdegrees, currentazdegrees, trackSettings.objectverticalpixels, trackSettings.objecthorizontalpixels)
                                objectvertical = -1 * ((trackSettings.objectverticalpixels - trackSettings.mainviewY) * trackSettings.imagescale)
                                objecthorizontal = (trackSettings.objecthorizontalpixels - trackSettings.mainviewX) * trackSettings.imagescale
                                objectangle = math.degrees(math.atan2(objectvertical, objecthorizontal)) - 90
                                objectangle2 = math.degrees(math.atan2(objectvertical, objecthorizontal))
                                objectdistance = math.sqrt((objecthorizontal**2) + (objectvertical**2) - 2 * (objectvertical * objecthorizontal * math.cos(math.radians(objectangle))))
                            #try:
                            if trackSettings.foundtarget is True:
                                try:
                                    objectalt = 90 - math.degrees(math.acos(math.cos(math.radians(objectdistance)) * math.cos(math.radians(90 - currentaltdegrees)) + math.sin(math.radians(objectdistance)) * math.sin(math.radians(90 - currentaltdegrees)) * math.cos(math.radians(objectangle))))
                                    diffinaz = math.degrees(math.acos((math.cos(math.radians(objectdistance)) - math.cos(math.radians(90 - currentaltdegrees)) * math.cos(math.radians(90 - objectalt))) / (math.sin(math.radians(90 - currentaltdegrees)) * math.sin(math.radians(90 - objectalt)))))
                                except:
                                    pass
                                if math.fabs(objectangle2) > 90:
                                    diffinaz = -1 * diffinaz
                                try:
                                    objectaz = diffinaz + currentazdegrees
                                except:
                                    objectaz = currentazdegrees
                                if firstslew is True:
                                    firstslew = False
                                    for i in range (0,10):
                                        self.predictedCoords = self.kfObj.Estimate(objectaz, objectalt)
                                        objectaz2 = float(self.predictedCoords[0])
                                        objectalt2 = float(self.predictedCoords[1])
                                        #print(objectaz2, objectalt2, objectaz, objectalt)
                                elif abs(self.az2 - objectaz) > 90:
                                    for i in range (0,10):
                                        self.predictedCoords = self.kfObj.Estimate(objectaz, objectalt)
                                        objectaz2 = float(self.predictedCoords[0])
                                        objectalt2 = float(self.predictedCoords[1])                                    
                                self.predictedCoords = self.kfObj.Estimate(objectaz, objectalt)
                                objectaz = float(self.predictedCoords[0])
                                objectalt = float(self.predictedCoords[1])
                                #check if we're crossing north meridian and correct if so
                                headingwestaz = objectaz - (currentazdegrees+360)
                                headingeastaz = (objectaz+360) - currentazdegrees
                                diffinaz = objectaz - currentazdegrees
                                if abs(headingwestaz) < abs(diffinaz):
                                    diffinaz = headingwestaz
                                if abs(headingeastaz) < abs(diffinaz):
                                    diffinaz = headingeastaz
                                altdiff = math.degrees(math.radians(objectalt) - currentalt)
                                azdiff = diffinaz
                                totaldiff = math.sqrt(altdiff**2 + azdiff**2)
                                self.alt2 = objectalt
                                self.az2 = objectaz
                                if self.pretrack is True:
                                    self.az = self.az2
                                    self.alt = self.alt2
                                azrate = (self.az2 - self.az)/elapsedtime.total_seconds()
                                headingwestaz = (self.az - (self.az2+360))/elapsedtime.total_seconds()
                                headingeastaz = ((self.az+360) - self.az2)/elapsedtime.total_seconds()
                                if abs(headingwestaz) < abs(azrate):
                                    azrate = headingwestaz
                                if abs(headingeastaz) < abs(azrate):
                                    azrate = headingeastaz
                                altrate = (self.alt2 - self.alt)/elapsedtime.total_seconds()
                                #if math.fabs(self.diffazlast) < math.fabs(azdiff):
                                if self.pretrack is True:
                                    self.azratelast = azrate
                                    azaccel = 0
                                if trackSettings.foundtarget is True:
                                    azaccel = azrate - self.azratelast
                                    self.azratelast = azrate
                                azrate = azrate + (azdiff) 
                                #if math.fabs(self.diffaltlast) < math.fabs(altdiff):
                                if self.pretrack is True:
                                    self.altratelast = altrate
                                    altaccel = 0
                                #we're done with setup for the first pass through the loop so dump pretrack
                                    self.pretrack = False
                                if trackSettings.foundtarget is True:
                                    altaccel = altrate - self.altratelast
                                    self.altratelast = altrate
                                altrate = altrate + (altdiff)

                        if azrate > self.axis0rate:
                            azrate = self.axis0rate
                        if azrate < (-1*self.axis0rate):
                            azrate = (-1*self.axis0rate)
                        if altrate > self.axis1rate:
                            altrate = self.axis1rate
                        if altrate < (-1*self.axis1rate):
                            altrate = (-1*self.axis1rate)
                        altrate = round(altrate, 4)
                        azrate = round(azrate, 4)
                        if throttle < 0.001:
                            self.ser.write(str.encode(':Q#'))
                        else:
                            self.textbox.insert(END, str(' Altrate: ' + str(altrate) + ' Azrate: ' + str(azrate) + '\n'))
                            self.textbox.see('end')
                            self.ser.write(str.encode(str(':RA'+str(azrate)+'#')))
                            self.ser.write(str.encode(str(':RE'+str(altrate)+'#')))
                            self.ser.write(str.encode(':Me#'))
                            self.ser.write(str.encode(':Mn#'))
                        time.sleep(0.1)
                        if trackSettings.foundtarget is True:
                            self.az = self.az2
                            self.alt = self.alt2
                if trackSettings.telescopetype == 'ASCOM':
                    time.sleep(0.01)
                    d = datetime.datetime.utcnow()
                    if trackSettings.mounttype == 'AltAz':
                        ref_tel_alt = self.tel.Altitude
                        ref_tel_az = self.tel.Azimuth
                        weighted_avg_alt_sep, weighted_avg_az_sep = self.errorWeightedAverage(ref_tel_alt, ref_tel_az)
                        currentaz = ref_tel_az + weighted_avg_az_sep
                        currentalt = ref_tel_alt + weighted_avg_alt_sep
                        currentaltdegrees = currentalt
                        currentazdegrees = currentaz
                        if self.dnow > self.dlast:
                            elapsedtime = self.dnow-self.dlast
                            currentalt = math.radians(currentalt)
                            currentaz = math.radians(currentaz)
                            if trackSettings.foundtarget is True:
                                objectvertical = -1 * ((self.targetY - trackSettings.mainviewY) * trackSettings.imagescale)
                                objecthorizontal = (self.targetX - trackSettings.mainviewX) * trackSettings.imagescale
                                objectangle = math.degrees(math.atan2(objectvertical, objecthorizontal)) - 90
                                objectangle2 = math.degrees(math.atan2(objectvertical, objecthorizontal))
                                objectdistance = math.sqrt((objecthorizontal**2) + (objectvertical**2) - 2 * (objectvertical * objecthorizontal * math.cos(math.radians(objectangle))))
                                trackSettings.objectverticalpixels = self.targetY
                                trackSettings.objecthorizontalpixels = self.targetX
                            else:
                                self.alt = self.alt+self.altratelast*elapsedtime.total_seconds()
                                self.az = self.az+self.azratelast*elapsedtime.total_seconds()
                                diffalt = (self.alt) - currentaltdegrees
                                diffaz = (self.az) - currentazdegrees
                                trackSettings.objectverticalpixels = int((trackSettings.mainviewY-(diffalt)/trackSettings.imagescale))
                                trackSettings.objecthorizontalpixels = int((trackSettings.mainviewX+(diffaz)/trackSettings.imagescale))
                                print (' ', self.alt, self.az, currentaltdegrees, currentazdegrees, trackSettings.objectverticalpixels, trackSettings.objecthorizontalpixels)
                                objectvertical = -1 * ((trackSettings.objectverticalpixels - trackSettings.mainviewY) * trackSettings.imagescale)
                                objecthorizontal = (trackSettings.objecthorizontalpixels - trackSettings.mainviewX) * trackSettings.imagescale
                                objectangle = math.degrees(math.atan2(objectvertical, objecthorizontal)) - 90
                                objectangle2 = math.degrees(math.atan2(objectvertical, objecthorizontal))
                                objectdistance = math.sqrt((objecthorizontal**2) + (objectvertical**2) - 2 * (objectvertical * objecthorizontal * math.cos(math.radians(objectangle))))
                            #try:
                            if trackSettings.foundtarget is True:
                                try:
                                    objectalt = 90 - math.degrees(math.acos(math.cos(math.radians(objectdistance)) * math.cos(math.radians(90 - currentaltdegrees)) + math.sin(math.radians(objectdistance)) * math.sin(math.radians(90 - currentaltdegrees)) * math.cos(math.radians(objectangle))))
                                    diffinaz = math.degrees(math.acos((math.cos(math.radians(objectdistance)) - math.cos(math.radians(90 - currentaltdegrees)) * math.cos(math.radians(90 - objectalt))) / (math.sin(math.radians(90 - currentaltdegrees)) * math.sin(math.radians(90 - objectalt)))))
                                except:
                                    pass
                                if math.fabs(objectangle2) > 90:
                                    diffinaz = -1 * diffinaz
                                try:
                                    objectaz = diffinaz + currentazdegrees
                                except:
                                    objectaz = currentazdegrees
                                if firstslew is True:
                                    firstslew = False
                                    for i in range (0,10):
                                        self.predictedCoords = self.kfObj.Estimate(objectaz, objectalt)
                                        objectaz2 = float(self.predictedCoords[0])
                                        objectalt2 = float(self.predictedCoords[1])
                                        #print(objectaz2, objectalt2, objectaz, objectalt)
                                elif abs(self.az2 - objectaz) > 90:
                                    for i in range (0,10):
                                        self.predictedCoords = self.kfObj.Estimate(objectaz, objectalt)
                                        objectaz2 = float(self.predictedCoords[0])
                                        objectalt2 = float(self.predictedCoords[1])
                                self.predictedCoords = self.kfObj.Estimate(objectaz, objectalt)
                                objectaz = float(self.predictedCoords[0])
                                objectalt = float(self.predictedCoords[1])
                                #check if we're crossing north meridian and correct if so
                                headingwestaz = objectaz - (currentazdegrees+360)
                                headingeastaz = (objectaz+360) - currentazdegrees
                                diffinaz = objectaz - currentazdegrees
                                if abs(headingwestaz) < abs(diffinaz):
                                    diffinaz = headingwestaz
                                if abs(headingeastaz) < abs(diffinaz):
                                    diffinaz = headingeastaz
                                altdiff = math.degrees(math.radians(objectalt) - currentalt)
                                azdiff = diffinaz
                                totaldiff = math.sqrt(altdiff**2 + azdiff**2)
                                self.alt2 = objectalt
                                self.az2 = objectaz
                                if self.pretrack is True:
                                    self.az = self.az2
                                    self.alt = self.alt2
                                azrate = (self.az2 - self.az)/elapsedtime.total_seconds()
                                headingwestaz = (self.az - (self.az2+360))/elapsedtime.total_seconds()
                                headingeastaz = ((self.az+360) - self.az2)/elapsedtime.total_seconds()
                                if abs(headingwestaz) < abs(azrate):
                                    azrate = headingwestaz
                                if abs(headingeastaz) < abs(azrate):
                                    azrate = headingeastaz
                                altrate = (self.alt2 - self.alt)/elapsedtime.total_seconds()
                                #if math.fabs(self.diffazlast) < math.fabs(azdiff):
                                if self.pretrack is True:
                                    self.azratelast = azrate
                                    azaccel = 0
                                if trackSettings.foundtarget is True:
                                    azaccel = azrate - self.azratelast
                                    self.azratelast = azrate
                                azrate = azrate + (azdiff*trackSettings.aggression) 
                                #if math.fabs(self.diffaltlast) < math.fabs(altdiff):
                                if self.pretrack is True:
                                    self.altratelast = altrate
                                    altaccel = 0
                                #we're done with setup for the first pass through the loop so dump pretrack
                                    self.pretrack = False
                                if trackSettings.foundtarget is True:
                                    altaccel = altrate - self.altratelast
                                    self.altratelast = altrate
                                altrate = altrate + (altdiff*trackSettings.aggression)
                                if azrate > self.axis0rate:
                                    azrate = self.axis0rate
                                if azrate < (-1*self.axis0rate):
                                    azrate = (-1*self.axis0rate)
                                if altrate > self.axis1rate:
                                    altrate = self.axis1rate
                                if altrate < (-1*self.axis1rate):
                                    altrate = (-1*self.axis1rate)
                                #print('azdiff, altdiff, objaz, objalt, currentaz, currentalt', azdiff, altdiff, self.az2, self.alt2, currentazdegrees, currentaltdegrees, end='\r')
                                self.textbox.insert(END, str('Delta Az: ' + str(azdiff) + ' Delta Alt: ' + str(altdiff) + '\n'))
                                self.textbox.see('end')
                            else:
                                azrate = self.azratelast + diffaz
                                altrate = self.altratelast + diffalt
                                if azrate > self.axis0rate:
                                    azrate = self.axis0rate
                                if azrate < (-1*self.axis0rate):
                                    azrate = (-1*self.axis0rate)
                                if altrate > self.axis1rate:
                                    altrate = self.axis1rate
                                if altrate < (-1*self.axis1rate):
                                    altrate = (-1*self.axis1rate)
                            #Check command rate and issue command if enough time has passed
                            currentcommandtime = datetime.datetime.utcnow()
                            commandtimediff = (currentcommandtime - self.tlastcommand).total_seconds()
                            if commandtimediff > 0.1:
                                #If we sent a command, update the last commanded time
                                self.tlastcommand = datetime.datetime.utcnow()
                                if abs(abs(commandedazratelast) - abs(azrate)) > 0.0001:
                                    self.tel.MoveAxis(0, azrate)
                                    commandedazratelast = azrate
                                if abs(abs(commandedaltratelast) - abs(altrate)) > 0.0001:
                                    self.tel.MoveAxis(1, altrate)
                                    commandedaltratelast = altrate
                                azratelast = azrate
                                altratelast = altrate
                            if trackSettings.foundtarget is True:
                                self.az = self.az2
                                self.alt = self.alt2
                            #except:
                            #    print('Failed to do the math.')
                    if trackSettings.mounttype == 'Eq':
                        self.raddec = self.sat.dec
                        self.radra = self.sat.ra
                        currentra = float(self.tel.RightAscension)*15
                        currentdec = self.tel.Declination
                        currentdecdegrees = currentdec
                        if self.dnow > self.dlast:
                            currentdec = math.radians(currentdec)
                            currentra = math.radians(currentra)
                            objectvertical = -1 * ((self.targetY - trackSettings.mainviewY) * trackSettings.imagescale)
                            objecthorizontal = (self.targetX - trackSettings.mainviewX) * trackSettings.imagescale
                            objectangle = math.degrees(math.atan2(objectvertical, objecthorizontal)) - 90
                            objectangle2 = math.degrees(math.atan2(objectvertical, objecthorizontal))
                            objectdistance = math.sqrt((objecthorizontal**2) + (objectvertical**2) - 2 * (objectvertical * objecthorizontal * math.cos(math.radians(objectangle))))
                            try:
                                objectdec = 90 - math.degrees(math.acos(math.cos(math.radians(objectdistance)) * math.cos(math.radians(90 - currentdecdegrees)) + math.sin(math.radians(objectdistance)) * math.sin(math.radians(90 - currentdecdegrees)) * math.cos(math.radians(objectangle))))
                                diffinra = math.degrees(math.acos((math.cos(math.radians(objectdistance)) - math.cos(math.radians(90 - currentdecdegrees)) * math.cos(math.radians(90 - objectdec))) / (math.sin(math.radians(90 - currentdecdegrees)) * math.sin(math.radians(90 - objectdec)))))
                                if math.fabs(objectangle2) > 90:
                                    diffinra = -1 * diffinra
                                decdiff = math.radians(objectdec) - currentdec
                                radiff = math.radians(diffinra)
                                totaldiff = math.sqrt(decdiff**2 + radiff**2)
                                self.observer.date = (d + datetime.timedelta(seconds=1))
                                self.sat.compute(self.observer)
                                self.raddec2 = self.sat.dec
                                self.radra2 = self.sat.ra
                                rarate = (math.degrees(self.radra2 - self.radra))
                                decrate = math.degrees(self.raddec2 - self.raddec)
                                #if math.fabs(self.diffralast) < math.fabs(radiff):
                                rarate = rarate + math.degrees(radiff)*trackSettings.aggression
                                #if math.fabs(self.diffdeclast) < math.fabs(decdiff):
                                decrate = decrate + math.degrees(decdiff)*trackSettings.aggression
                                if rarate > self.axis0rate:
                                    rarate = self.axis0rate
                                if rarate < (-1*self.axis0rate):
                                    rarate = (-1*self.axis0rate)
                                if decrate > self.axis1rate:
                                    decrate = self.axis1rate
                                if decrate < (-1*self.axis1rate):
                                    decrate = (-1*self.axis1rate)
                                #Check command rate and issue command if enough time has passed
                                currentcommandtime = datetime.datetime.utcnow()
                                commandtimediff = (currentcommandtime - self.tlastcommand).total_seconds()
                                if commandtimediff > 0.1:
                                    #If we sent a command, update the last commanded time
                                    self.tlastcommand = datetime.datetime.utcnow()
                                    self.tel.MoveAxis(0, rarate)
                                    self.tel.MoveAxis(1, decrate)
                                self.diffralast = radiff
                                self.diffdeclast = decdiff
                            except:
                                print('Failed to do the math.')
                if trackSettings.telescopetype == 'LX200':
                    if trackSettings.mounttype == 'AltAz':
                        #check if it's time to correct and that we have a newer frame than last time
                        try:
                            if self.dnow > self.dlast:
                                iterate += 1
                                self.LX200_alt_degrees()
                                self.LX200_alt_degrees()
                                currentalt = math.radians(self.telalt)
                                self.LX200_az_degrees()
                                currentaz = math.radians(self.telaz)
                                objectvertical = -1 * ((self.targetY - trackSettings.mainviewY) * trackSettings.imagescale)
                                objecthorizontal = (self.targetX - trackSettings.mainviewX) * trackSettings.imagescale
                                objectangle = math.degrees(math.atan2(objectvertical, objecthorizontal)) - 90
                                objectangle2 = math.degrees(math.atan2(objectvertical, objecthorizontal))
                                objectdistance = math.sqrt((objecthorizontal**2) + (objectvertical**2) - 2 * (objectvertical * objecthorizontal * math.cos(math.radians(objectangle))))
                                objectalt = 90 - math.degrees(math.acos(math.cos(math.radians(objectdistance)) * math.cos(math.radians(90 - self.telalt)) + math.sin(math.radians(objectdistance)) * math.sin(math.radians(90 - self.telalt)) * math.cos(math.radians(objectangle))))
                                try:
                                    diffinaz = math.degrees(math.acos((math.cos(math.radians(objectdistance)) - math.cos(math.radians(90 - self.telalt)) * math.cos(math.radians(90 - objectalt))) / (math.sin(math.radians(90 - self.telalt)) * math.sin(math.radians(90 - objectalt)))))
                                except:
                                    diffinaz = 0.0
                                objectalt = math.radians(objectalt)
                                diffinaz = math.radians(diffinaz)
                                #if objectangle < 0 and objectangle > -180:
                                #    objectaz = currentaz + diffinaz
                                #else:
                                #    objectaz = currentaz - diffinaz
                                if math.fabs(objectangle2) > 90:
                                    diffinaz = -1 * diffinaz
                                objectaz = currentaz + diffinaz
                                altdiff = objectalt - currentalt
                                totaldiff = math.sqrt(altdiff**2 + diffinaz**2)
                                #if total dist to target is increasing since last frame we need to correct!
                                if trackSettings.foundtarget is True:
                                    if self.logaltaz.get() == 1:
                                        log = open('ufologaltaz.txt','a')
                                        if math.degrees(objectaz) < 0:
                                            reportaz = math.degrees(objectaz) + 360
                                        elif math.degrees(objectaz) > 360:
                                            reportaz = math.degrees(objectaz) - 360
                                        else:
                                            reportaz = math.degrees(objectaz)
                                        log.write(str(self.dnow)+','+str(math.degrees(objectalt))+','+str(reportaz)+'\n')
                                        log.close()
                                    if iterate > 10 and iterate < 15:
                                        altcorrectrunning = altcorrectrunning + (altdiff)
                                        azcorrectrunning = azcorrectrunning + (diffinaz)
                                    elif iterate == 15:
                                        altcorrect = altcorrectrunning/4
                                        azcorrect = azcorrectrunning/4
                                if iterate > 15:
                                    altcorrectrunning = altcorrect*4
                                    azcorrectrunning = azcorrect*4
                                    iterate = 0
                                print(math.degrees(totaldiff))
                                self.lasttotaldiff = totaldiff
                                try:
                                    objectaltrate = (objectalt - objectaltlast)/(self.dnow-self.dlast).total_seconds()
                                    objectazrate = (objectaz - objectazlast)/(self.dnow-self.dlast).total_seconds()
                                except:
                                    objectaltrate = 0
                                    objectazrate = 0
                                objectazlast = objectaz
                                objectaltlast = objectalt
                                self.textbox.insert(END, str('Altitude: ' + str(math.degrees(currentalt)) + ' Azimuth: ' + str(math.degrees(currentaz)) + '\n'))
                                self.textbox.see('end')
                                #print(math.degrees(altcorrect), math.degrees(azcorrect), math.degrees(altdiff), math.degrees(azdiff), math.degrees(currentalt), math.degrees(currentaz))
                            self.dlast = self.dnow
                            dcalculate = datetime.datetime.now()
                            elapsedtime = (dcalculate - self.dnow).total_seconds()
                            self.radaz = (objectaz+objectazrate*elapsedtime) + azcorrect
                            if math.degrees(self.radaz)>360:
                                self.radaz = self.radaz - math.radians(360)
                            elif math.degrees(self.radaz)<0:
                                self.radaz = self.radaz + math.radians(360)
                            self.radalt = (objectalt+objectaltrate*elapsedtime) + altcorrect
                            #self.textbox.insert(END, str('Commanded az: ' + str(math.degrees(self.radaz)) + ' Commanded alt: ' + str(math.degrees(self.radalt)) + '\n'))
                            #self.textbox.see('end')
                            self.rad_to_sexagesimal_alt()
                            targetcoordaz = str(':Sz ' + str(self.az_d)+'*'+str(self.az_m)+':'+str(int(self.az_s))+'#')
                            targetcoordalt = str(':Sa ' + str(self.alt_d)+'*'+str(self.alt_m)+':'+str(int(self.alt_s))+'#')
                            self.ser.write(str.encode(targetcoordaz))
                            self.ser.write(str.encode(targetcoordalt))
                            self.ser.write(str.encode(':MA#'))
                        except Exception:
                            traceback.print_exc()
                            pass
                    if trackSettings.mounttype == 'EQ':
                        self.raddec = self.sat.dec
                        self.radra = self.sat.ra 
                        #check if it's time to correct and that we have a newer frame than last time
                        if self.dnow > self.dlast:
                            self.LX200_dec_degrees()
                            self.LX200_dec_degrees()
                            currentdec = math.radians(self.teldec)
                            self.LX200_ra_degrees()
                            currentra = math.radians(self.telra)
                            objectvertical = -1 * ((self.targetY - trackSettings.mainviewY) * trackSettings.imagescale)
                            objecthorizontal = (self.targetX - trackSettings.mainviewX) * trackSettings.imagescale
                            objectangle = math.degrees(math.atan2(objectvertical, objecthorizontal)) - 90
                            objectangle2 = math.degrees(math.atan2(objectvertical, objecthorizontal))
                            objectdistance = math.sqrt((objecthorizontal**2) + (objectvertical**2) - 2 * (objectvertical * objecthorizontal * math.cos(math.radians(objectangle))))
                            try:
                                objectalt = 90 - math.degrees(math.acos(math.cos(math.radians(objectdistance)) * math.cos(math.radians(90 - self.teldec)) + math.sin(math.radians(objectdistance)) * math.sin(math.radians(90 - self.teldec)) * math.cos(math.radians(objectangle))))
                                diffinaz = math.degrees(math.acos((math.cos(math.radians(objectdistance)) - math.cos(math.radians(90 - self.teldec)) * math.cos(math.radians(90 - objectalt))) / (math.sin(math.radians(90 - self.teldec)) * math.sin(math.radians(90 - objectalt)))))
                                if math.fabs(objectangle2) > 90:
                                    diffinra = -1 * diffinra
                                decdiff = math.radians(objectdec) - currentdec
                                radiff = math.radians(diffinra)
                                totaldiff = math.sqrt(decdiff**2 + radiff**2)
                                #if total dist to target is increasing since last frame we need to correct!
                                if self.lasttotaldiff < totaldiff:
                                    deccorrect = deccorrect + (decdiff)
                                    racorrect = racorrect + (radiff)
                                print(math.degrees(totaldiff))
                                self.textbox.insert(END, str('Distance from Target: ' + str(totaldiff) + '\n'))
                                self.textbox.see('end')
                                self.lasttotaldiff = totaldiff
                            except:
                                print('Failed to do the math.')
                            #print(math.degrees(altcorrect), math.degrees(azcorrect), math.degrees(altdiff), math.degrees(azdiff), math.degrees(currentalt), math.degrees(currentaz))
                        self.dlast = self.dnow
                        self.radra = self.radra + racorrect
                        self.raddec = self.raddec + deccorrect
                        self.rad_to_sexagesimal_ra()
                        targetcoordra = str(':Sr ' + str(self.ra_h)+'*'+str(self.ra_m)+':'+str(int(self.ra_s))+'#')
                        targetcoorddec = str(':Sd ' + str(self.dec_d)+'*'+str(self.dec_m)+':'+str(int(self.dec_s))+'#')
                        self.ser.write(str.encode(targetcoordra))
                        self.ser.write(str.encode(targetcoorddec))
                        self.ser.write(str.encode(':MS#'))
            time.sleep(0.005)
        #stop moving the telescope if the user is on ASCOM and requested stop tracking.
        if trackSettings.telescopetype == 'ASCOM' and trackSettings.joytracking is False:
            self.tel.AbortSlew()
    
    def set_center(self):
        trackSettings.setcenter = True
    
    def setLX200AltAz(self):
        trackSettings.telescopetype = 'LX200'
        trackSettings.mounttype = 'AltAz'
        
    def setLX200Eq(self):
        trackSettings.telescopetype = 'LX200'
        trackSettings.mounttype = 'Eq'
    
    def setFeatureTrack(self):
        trackSettings.trackingtype = 'Features'
    
    def setBrightTrack(self):
        trackSettings.trackingtype = 'Bright'    
        
    def setASCOMAltAz(self):
        trackSettings.telescopetype = 'ASCOM'
        trackSettings.mounttype = 'AltAz'
    
    def setASCOMEq(self):
        trackSettings.telescopetype = 'ASCOM'
        trackSettings.mounttype = 'Eq'
    
    def setAutostarAltAz(self):
        trackSettings.telescopetype = 'Autostar'
        trackSettings.mounttype = 'AltAz'
    
    def set_img_collect(self):
        if self.collect_images is False:
            self.collect_images = True
            print('Starting Camera.')
            self.textbox.insert(END, 'Starting Camera.\n')
            self.textbox.see('end')
            self.cap = cv2.VideoCapture(int(self.entryCam.get()))
            self.displayimg = Label(self.topframe, bg="black")
            self.startButton.configure(text='Stop Camera')
            imagethread = threading.Thread(target=self.prepare_img_for_tkinter, daemon=True)
            imagethread.start()
        else:
            self.cap.release()
            self.collect_images = False
            self.startButton.configure(text='Start Camera')
    
    def read_to_hash(self):
        self.resp = self.ser.read()
        self.resp = self.resp.decode("utf-8", errors="ignore")
        try:
            while self.resp[-1] != '#':
                self.resp += self.ser.read().decode("utf-8", errors="ignore")
        except:
            print('Unable to read line')
            self.textbox.insert(END, 'Unable to read line.\n')
            self.textbox.see('end')
        #print(self.resp)
        if trackSettings.degorhours == 'Degrees':
            self.deg = int(self.resp[0:3])
            self.min = int(self.resp[3:5])
            self.sec = int(self.resp[6:8])
            self.respdegrees = ((((self.sec/60)+self.min)/60)+self.deg)
            #print(self.resp, self.respdegrees)
        if trackSettings.degorhours == 'Hours':
            self.hr = int(self.resp[0:2])
            self.min = int(self.resp[3:5])
            self.sec = int(self.resp[6:8])
            self.resphours = ((((self.sec/60)+self.min)/60)+self.hr)*15
        return
    
    def read_all_bytes(self):
        bytesToRead = self.ser.inWaiting()
        while bytesToRead == 0:
            bytesToRead = self.ser.inWaiting()
        self.resp = self.ser.read()
        self.resp = self.resp.decode("utf-8", errors="ignore")
        bytesToRead = self.ser.inWaiting()
        try:
            while bytesToRead > 0:
                self.resp += self.ser.read().decode("utf-8", errors="ignore")
                bytesToRead = self.ser.inWaiting()
        except:
            print('Unable to read line')
            self.textbox.insert(END, 'Unable to read line.\n')
            self.textbox.see('end')
        responsestr = self.resp
        return(responsestr)
    
    def set_tracking(self):
        if trackSettings.tracking is False:
            trackSettings.tracking = True
            print('Connecting to Scope.')
            self.textbox.insert(END, 'Connecting to Scope.\n')
            self.textbox.see('end')
            if trackSettings.telescopetype == 'ioptron':
                try:
                    self.axis0rate = 8
                    self.axis1rate = 6
                    self.comport = str('COM'+str(self.entryCom.get()))
                    self.ser = serial.Serial(self.comport, baudrate=115200, timeout=1, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, xonxoff=False, rtscts=False)
                    #Put the mount in special mode if it wasn't already
                    self.ser.write(str.encode(':MountInfo#'))
                    responsestr = self.read_all_bytes()
                    print(responsestr)
                    if responsestr not in trackSettings.ioptronspecialmoderesponses:
                        self.ser.write(str.encode(':ZZZ#'))
                    self.serialconnected = True
                    self.startButton5.configure(text='Disconnect Scope')
                except:
                    print('Failed to connect on ' + self.comport)
                    self.textbox.insert(END, str('Failed to connect on ' + str(self.comport) + '\n'))
                    self.textbox.see('end')
                    trackSettings.tracking = False
                    return
            if trackSettings.telescopetype == 'LX200':
                try:
                    self.axis0rate = 16
                    self.axis1rate = 16
                    self.comport = str('COM'+str(self.entryCom.get()))
                    self.ser = serial.Serial(self.comport, baudrate=9600, timeout=1, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, xonxoff=False, rtscts=False)
                    self.ser.write(str.encode(':U#'))
                    self.serialconnected = True
                    self.startButton5.configure(text='Disconnect Scope')
                except:
                    print('Failed to connect on ' + self.comport)
                    self.textbox.insert(END, str('Failed to connect on ' + str(self.comport) + '\n'))
                    self.textbox.see('end')
                    trackSettings.tracking = False
                    return
            elif trackSettings.telescopetype == 'Autostar':
                try:
                    self.axis0rate = 16
                    self.axis1rate = 16
                    self.comport = str('COM'+str(self.entryCom.get()))
                    self.ser = serial.Serial(self.comport, baudrate=9600, timeout=1, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, xonxoff=False, rtscts=False)
                    self.ser.write(str.encode(':U#'))
                    self.serialconnected = True
                    self.startButton5.configure(text='Disconnect Scope')
                except Exception as e:
                    print(e)
                    print('Failed to connect on ' + self.comport)
                    self.textbox.insert(END, str('Failed to connect on ' + str(self.comport) + '\n'))
                    self.textbox.see('end')
                    trackSettings.tracking = False
                    return
            elif trackSettings.telescopetype == 'ASCOM':
                self.x = win32com.client.Dispatch("ASCOM.Utilities.Chooser")
                self.x.DeviceType = 'Telescope'
                driverName=self.x.Choose("None")
                self.tel=win32com.client.Dispatch(driverName)
                if self.tel.Connected:
                    print("Telescope was already connected")
                    self.textbox.insert(END, str('Telescope was already connected.\n'))
                    self.textbox.see('end')
                    #self.startButton5.configure(text='Disconnect Scope')
                    axis = self.tel.CanMoveAxis(0)
                    axis2 = self.tel.CanMoveAxis(1)
                    if axis is False or axis2 is False:
                        print('This scope cannot use the MoveAxis method, aborting.')
                        self.textbox.insert(END, str('This scope cannot use the MoveAxis method, aborting.\n'))
                        self.textbox.see('end')
                        self.tel.Connected = False
                    else:
                        self.axis0rate = float(self.tel.AxisRates(0).Item(1).Maximum)
                        self.axis1rate = float(self.tel.AxisRates(1).Item(1).Maximum)
                        print(self.axis0rate)
                        print(self.axis1rate)
                        self.textbox.insert(END, str('Axis 0 max rate: '+str(self.axis0rate)+' Axis 1 max rate: '+ str(self.axis1rate)+'\n'))
                        self.textbox.see('end')
                        self.startButton5.configure(text='Disconnect Scope')
                else:
                    self.tel.Connected = True
                    if self.tel.Connected:
                        print("Connected to telescope now")
                        self.textbox.insert(END, str('Connected to telescope now.\n'))
                        self.textbox.see('end')
                        axis = self.tel.CanMoveAxis(0)
                        axis2 = self.tel.CanMoveAxis(1)
                        if axis is False or axis2 is False:
                            print('This scope cannot use the MoveAxis method, aborting.')
                            self.textbox.insert(END, str('This scope cannot use the MoveAxis method, aborting.\n'))
                            self.textbox.see('end')
                            self.tel.Connected = False
                        else:
                            self.axis0rate = float(self.tel.AxisRates(0).Item(1).Maximum)
                            self.axis1rate = float(self.tel.AxisRates(1).Item(1).Maximum)
                            print(self.axis0rate)
                            print(self.axis1rate)
                            self.textbox.insert(END, str('Axis 0 max rate: '+str(self.axis0rate)+' Axis 1 max rate: '+ str(self.axis1rate)+'\n'))
                            self.textbox.see('end')
                            self.startButton5.configure(text='Disconnect Scope')
                    else:
                        print("Unable to connect to telescope, expect exception")
                        self.textbox.insert(END, str('Unable to connect to telescope, expect exception.\n'))
                        self.textbox.see('end')
        else:
            print('Disconnecting the Scope.')
            self.textbox.insert(END, str('Disconnecting the scope.\n'))
            self.textbox.see('end')
            if trackSettings.telescopetype == 'LX200' and self.serialconnected is True:
                self.ser.write(str.encode(':Q#'))
                self.ser.write(str.encode(':U#'))
                self.ser.close()
                self.serialconnected = False
            elif trackSettings.telescopetype == 'ioptron' and self.serialconnected is True:
                #Set back to normal mode before disconnecting
                self.ser.write(str.encode(':MountInfo#'))
                responsestr = self.read_all_bytes()
                print(responsestr)
                if responsestr in trackSettings.ioptronspecialmoderesponses:
                    self.ser.write(str.encode(':ZZZ#'))
                self.ser.close()
                self.serialconnected = False
            elif trackSettings.telescopetype == 'ASCOM':
                self.tel.AbortSlew()
                self.tel.Connected = False
            trackSettings.tracking = False
            self.startButton5.configure(text='Connect Scope')
    
    def rad_to_sexagesimal_alt(self):
        self.azdeg = math.degrees(self.radaz)
        self.altdeg = math.degrees(self.radalt)
        self.az_d = math.trunc((self.azdeg))
        self.az_m = math.trunc((((self.azdeg)) - self.az_d)*60)
        self.az_s = (((((self.azdeg)) - self.az_d)*60) - self.az_m)*60
        
        self.alt_d = math.trunc(self.altdeg)
        self.alt_m = math.trunc((abs(self.altdeg) - abs(self.alt_d))*60)
        self.alt_s = (((abs(self.altdeg) - abs(self.alt_d))*60) - abs(self.alt_m))*60
    
    def rad_to_sexagesimal_ra(self):
        self.rahour = math.degrees(self.radra)/15
        self.decdeg = math.degrees(self.raddec)
        self.ra_h = math.trunc((self.rahour))
        self.ra_m = math.trunc((((self.rahour)) - self.ra_h)*60)
        self.ra_s = (((((self.rahour)) - self.ra_h)*60) - self.ra_m)*60
        
        self.dec_d = math.trunc(self.decdeg)
        self.dec_m = math.trunc((abs(self.decdeg) - abs(self.dec_d))*60)
        self.dec_s = (((abs(self.decdeg) - abs(self.dec_d))*60) - abs(self.dec_m))*60
    
    def start_calibration(self):
        calibthread = threading.Thread(target=self.set_calibration, daemon=True)
        calibthread.start()
    
    def set_calibration(self):
        if trackSettings.calibratestart is False:
            trackSettings.calibratestart = True
        else:
            if trackSettings.telescopetype == 'ASCOM':
                self.tel.MoveAxis(1, 0.0)
                self.tel.AbortSlew()
            trackSettings.calibratestart = False
        if trackSettings.tracking is False:
            print('Connect the Scope First!')
            self.textbox.insert(END, str('Connect the Scope First!\n'))
            self.textbox.see('end')
        if self.collect_images is False:
            print('Start Camera First!')
            self.textbox.insert(END, str('Start Camera First!\n'))
            self.textbox.see('end')
        if trackSettings.objectfollow is False:
            print('Pick a stationary calibration object first!')
            self.textbox.insert(END, str('Pick a stationary target first!\n'))
            self.textbox.see('end')
        speedset = True
        if trackSettings.telescopetype == 'ASCOM':
            try:
                trackSettings.calspeed = float(self.entryCal.get())
                if trackSettings.calspeed > self.axis0rate:
                    trackSettings.calspeed = self.axis0rate
                speedset = True
                if trackSettings.calspeed < 0:
                    speedset = False
                    print('Calibration speed needs to be a positive number!')
                    self.textbox.insert(END, str('Calibration speed needs to be a positive number!'))
            except:
                speedset = False
                print('Calibration speed needs to be a positive number!')
                self.textbox.insert(END, str('Calibration speed needs to be a positive number!'))
                self.textbox.see('end')
        if trackSettings.tracking is True and self.collect_images is True and trackSettings.objectfollow is True and trackSettings.calibratestart is True and speedset is True:
            if trackSettings.telescopetype == 'ASCOM':
                if trackSettings.mounttype == 'AltAz':
                    ref_tel_alt = self.tel.Altitude
                    ref_tel_az = self.tel.Azimuth
                    weighted_avg_alt_sep, weighted_avg_az_sep = self.errorWeightedAverage(ref_tel_alt, ref_tel_az)
                    currentaz = ref_tel_az + weighted_avg_az_sep
                    currentalt = ref_tel_alt + weighted_avg_alt_sep
                    self.X1 = math.radians(currentaz)
                    self.Y1 = math.radians(currentalt)
                    startx = self.targetX
                    starty = self.targetY
                    if starty < (self.height/2):
                        distmoved = 0
                        self.tel.MoveAxis(1, trackSettings.calspeed)
                        while distmoved < 100 and trackSettings.calibratestart is True:
                            #self.tel.MoveAxis(1, trackSettings.calspeed)
                            currentx = self.targetX
                            currenty = self.targetY
                            distmoved = math.sqrt((startx-currentx)**2+(starty-currenty)**2)
                            time.sleep(0.01)
                        self.tel.MoveAxis(1, 0.0)
                        self.tel.AbortSlew()
                        ref_tel_alt = self.tel.Altitude
                        ref_tel_az = self.tel.Azimuth
                        weighted_avg_alt_sep, weighted_avg_az_sep = self.errorWeightedAverage(ref_tel_alt, ref_tel_az)
                        currentaz = ref_tel_az + weighted_avg_az_sep
                        currentalt = ref_tel_alt + weighted_avg_alt_sep
                        self.X2 = math.radians(currentaz)
                        self.Y2 = math.radians(currentalt)
                        self.separation_between_coordinates()
                        self.imagescale = self.separation/distmoved
                        print(self.imagescale, ' degrees per pixel.')
                        self.textbox.insert(END, str('Image scale: '+str(self.imagescale)+' degrees per pixel.\n'))
                        self.textbox.see('end')
                    else:
                        distmoved = 0
                        self.tel.MoveAxis(1, (-1*trackSettings.calspeed))
                        while distmoved < 100 and trackSettings.calibratestart is True:
                            #self.tel.MoveAxis(1, (-1*trackSettings.calspeed))
                            currentx = self.targetX
                            currenty = self.targetY
                            distmoved = math.sqrt((startx-currentx)**2+(starty-currenty)**2)
                            time.sleep(0.01)
                        self.tel.MoveAxis(1, 0.0)
                        self.tel.AbortSlew()
                        ref_tel_alt = self.tel.Altitude
                        ref_tel_az = self.tel.Azimuth
                        weighted_avg_alt_sep, weighted_avg_az_sep = self.errorWeightedAverage(ref_tel_alt, ref_tel_az)
                        currentaz = ref_tel_az + weighted_avg_az_sep
                        currentalt = ref_tel_alt + weighted_avg_alt_sep
                        self.X2 = math.radians(currentaz)
                        self.Y2 = math.radians(currentalt)
                        self.separation_between_coordinates()
                        self.imagescale = self.separation/distmoved
                        print(self.imagescale, ' degrees per pixel.')
                        self.textbox.insert(END, str('Image scale: '+str(self.imagescale)+' degrees per pixel.\n'))
                        self.textbox.see('end')
                if trackSettings.mounttype == 'Eq':
                    self.X1 = math.radians(float(self.tel.RightAscension)*15)
                    self.Y1 = math.radians(float(self.tel.Declination))
                    startx = self.targetX
                    starty = self.targetY
                    if starty < (self.height/2):
                        distmoved = 0
                        self.tel.MoveAxis(1, trackSettings.calspeed)
                        while distmoved < 100 and trackSettings.calibratestart is True:
                            #self.tel.MoveAxis(1, trackSettings.calspeed)
                            currentx = self.targetX
                            currenty = self.targetY
                            distmoved = math.sqrt((startx-currentx)**2+(starty-currenty)**2)
                            time.sleep(0.01)
                        self.tel.MoveAxis(1, 0.0)
                        self.tel.AbortSlew()
                        self.X2 = math.radians(self.tel.RightAscension*15)
                        self.Y2 = math.radians(self.tel.Declination)
                        self.separation_between_coordinates()
                        #print('x1 ', self.X1, 'y1 ', self.Y1, 'separation ', self.separation, 'distance moved ', distmoved)
                        self.imagescale = self.separation/distmoved
                        print(self.imagescale, ' degrees per pixel.')
                        self.textbox.insert(END, str('Image scale: '+str(self.imagescale)+' degrees per pixel.\n'))
                        self.textbox.see('end')
                    else:
                        distmoved = 0
                        self.tel.MoveAxis(1, (-1*trackSettings.calspeed))
                        while distmoved < 100 and trackSettings.calibratestart is True:
                            #self.tel.MoveAxis(1, (-1*trackSettings.calspeed))
                            currentx = self.targetX
                            currenty = self.targetY
                            distmoved = math.sqrt((startx-currentx)**2+(starty-currenty)**2)
                            time.sleep(0.01)
                        self.tel.MoveAxis(1, 0.0)
                        self.tel.AbortSlew()
                        self.X2 = math.radians(self.tel.RightAscension*15)
                        self.Y2 = math.radians(self.tel.Declination)
                        self.separation_between_coordinates()
                        #print('x1 ', self.X1, 'y1 ', self.Y1, 'separation ', self.separation, 'distance moved ', distmoved)
                        self.imagescale = self.separation/distmoved
                        print(self.imagescale, ' degrees per pixel.')
                        self.textbox.insert(END, str('Image scale: '+str(self.imagescale)+' degrees per pixel.\n'))
                        self.textbox.see('end')
                trackSettings.imagescale = self.imagescale            
            if trackSettings.telescopetype == 'Autostar':
                self.LX200_az_degrees()
                self.X1 = math.radians(self.respdegrees)
                self.LX200_alt_degrees()
                self.Y1 = math.radians(self.respdegrees)
                startx = self.targetX
                starty = self.targetY
                
                if starty < (self.height/2):
                    distmoved = 0
                    self.ser.write(str.encode(':RC#'))
                    while distmoved < 100:
                        self.ser.write(str.encode(':Mn#'))
                        currentx = self.targetX
                        currenty = self.targetY
                        distmoved = math.sqrt((startx-currentx)**2+(starty-currenty)**2)
                        time.sleep(0.01)
                    self.ser.write(str.encode(':Qn#'))
                    self.LX200_az_degrees()
                    self.X2 = math.radians(self.respdegrees)
                    self.LX200_alt_degrees()
                    self.Y2 = math.radians(self.respdegrees)
                    self.separation_between_coordinates()
                    self.imagescale = self.separation/distmoved
                    print(self.imagescale, ' degrees per pixel.')
                    self.textbox.insert(END, str('Image scale: '+str(self.imagescale)+' degrees per pixel.\n'))
                    self.textbox.see('end')
                else:
                    distmoved = 0
                    self.ser.write(str.encode(':RC#'))
                    while distmoved < 100:
                        self.ser.write(str.encode(':Ms#'))
                        currentx = self.targetX
                        currenty = self.targetY
                        distmoved = math.sqrt((startx-currentx)**2+(starty-currenty)**2)
                        time.sleep(0.01)
                    self.ser.write(str.encode(':Qs#'))
                    self.LX200_az_degrees()
                    self.X2 = math.radians(self.respdegrees)
                    self.LX200_alt_degrees()
                    self.Y2 = math.radians(self.respdegrees)
                    self.separation_between_coordinates()
                    self.imagescale = self.separation/distmoved
                    print(self.imagescale, ' degrees per pixel.')
                    self.textbox.insert(END, str('Image scale: '+str(self.imagescale)+' degrees per pixel.\n'))
                    self.textbox.see('end')
                trackSettings.imagescale = self.imagescale
            if trackSettings.telescopetype == 'LX200':
                self.LX200_az_degrees()
                self.X1 = math.radians(self.respdegrees)
                self.LX200_alt_degrees()
                self.Y1 = math.radians(self.respdegrees)
                startx = self.targetX
                starty = self.targetY
                
                if starty < (self.height/2):
                    distmoved = 0
                    self.ser.write(str.encode(':RC#'))
                    while distmoved < 100:
                        self.ser.write(str.encode(':Mn#'))
                        currentx = self.targetX
                        currenty = self.targetY
                        distmoved = math.sqrt((startx-currentx)**2+(starty-currenty)**2)
                        time.sleep(0.01)
                    self.ser.write(str.encode(':Qn#'))
                    self.LX200_az_degrees()
                    self.X2 = math.radians(self.respdegrees)
                    self.LX200_alt_degrees()
                    self.Y2 = math.radians(self.respdegrees)
                    self.separation_between_coordinates()
                    self.imagescale = self.separation/distmoved
                    print(self.imagescale, ' degrees per pixel.')
                    self.textbox.insert(END, str('Image scale: '+str(self.imagescale)+' degrees per pixel.\n'))
                    self.textbox.see('end')
                else:
                    distmoved = 0
                    self.ser.write(str.encode(':RC#'))
                    while distmoved < 100:
                        self.ser.write(str.encode(':Ms#'))
                        currentx = self.targetX
                        currenty = self.targetY
                        distmoved = math.sqrt((startx-currentx)**2+(starty-currenty)**2)
                        time.sleep(0.01)
                    self.ser.write(str.encode(':Qs#'))
                    self.LX200_az_degrees()
                    self.X2 = math.radians(self.respdegrees)
                    self.LX200_alt_degrees()
                    self.Y2 = math.radians(self.respdegrees)
                    self.separation_between_coordinates()
                    self.imagescale = self.separation/distmoved
                    print(self.imagescale, ' degrees per pixel.')
                    self.textbox.insert(END, str('Image scale: '+str(self.imagescale)+' degrees per pixel.\n'))
                    self.textbox.see('end')
                trackSettings.imagescale = self.imagescale
            trackSettings.calibratestart = False    
    
    def separation_between_coordinates(self):
        self.separation = math.degrees(math.acos(math.sin(self.Y1)*math.sin(self.Y2) + math.cos(self.Y1)*math.cos(self.Y2)*math.cos(self.X1-self.X2)))


    def LX200_alt_degrees(self):
        self.ser.write(str.encode(':GA#'))
        bytesToRead = self.ser.inWaiting()
        while bytesToRead == 0:
            bytesToRead = self.ser.inWaiting()
        #print('Receiving Altitude')
        trackSettings.degorhours = 'Degrees'
        self.read_to_hash()
        self.telalt = self.respdegrees
    
    def LX200_dec_degrees(self):
        self.ser.write(str.encode(':GD#'))
        bytesToRead = self.ser.inWaiting()
        while bytesToRead == 0:
            bytesToRead = self.ser.inWaiting()
        #print('Receiving Altitude')
        trackSettings.degorhours = 'Degrees'
        self.read_to_hash()
        self.teldec = self.respdegrees

    def LX200_az_degrees(self):
        self.ser.write(str.encode(':GZ#'))
        bytesToRead = self.ser.inWaiting()
        while bytesToRead == 0:
            bytesToRead = self.ser.inWaiting()
        #print('Receiving Azimuth')
        trackSettings.degorhours = 'Degrees'
        self.read_to_hash()
        self.telaz = self.respdegrees

    def LX200_ra_degrees(self):
        self.ser.write(str.encode(':GR#'))
        bytesToRead = self.ser.inWaiting()
        while bytesToRead == 0:
            bytesToRead = self.ser.inWaiting()
        #print('Receiving Righ Ascension')
        trackSettings.degorhours = 'Hours'
        self.read_to_hash()
        self.telra = float(self.resphours)*15
        #print(self.resphours)
    
    def _on_mousewheel(self, event):
        trackSettings.boxSize = trackSettings.boxSize + (event.delta/24)
        if trackSettings.boxSize < 5:
            trackSettings.boxSize = 5
        print(trackSettings.boxSize)
        self.textbox.insert(END, str('Tracking box size: '+str(trackSettings.boxSize)+'\n'))
        self.textbox.see('end')

    def mouse_position(self, event):
        trackSettings.mousecoords = (event.x, event.y)
    
    def left_click(self, event):
        if trackSettings.setcenter is True:
            trackSettings.mainviewX = trackSettings.mousecoords[0]
            trackSettings.mainviewY = trackSettings.mousecoords[1]
            trackSettings.setcenter = False
        else:
            self.kfObj = KalmanFilter()
            self.predictedCoords = np.zeros((2, 1), np.float32)
            self.pretrack = True
            self.trackimg = Label(self.topframe, bg="black")
            self.imgtk = self.img.copy()
            self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
            #Set up a CLAHE histogram equalization for contrast enhancement of tracked feature
            if trackSettings.trackingtype == 'Features':
                clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(2,2))
                self.img = clahe.apply(self.img)
            self.imageroi = self.img[self.mousebox[0][1]:self.mousebox[1][1],self.mousebox[0][0]:self.mousebox[1][0]]
            #self.imageroi = cv2.cvtColor(self.imageroi, cv2.COLOR_BGR2GRAY)
            self.roibox = self.mousebox
            self.roiheight, self.roiwidth = self.imageroi.shape[:2]
            self.targetX = self.roibox[0][0]+(self.roiwidth/2)
            self.targetY = self.roibox[0][1]+(self.roiheight/2)
            self.roiboxlast = self.roibox
            self.dnow = datetime.datetime.utcnow()
            self.dlast = self.dnow - datetime.timedelta(seconds=0.01)
            #find brightness of the clicked pixel and use the median between that and the brightest pixel in the ROI as the threshhold cutoff for brightness
            trackSettings.clickpixel = self.img[trackSettings.mousecoords[1],trackSettings.mousecoords[0]]
            roiheight, roiwidth = self.imageroi.shape[:2]
            blurred = cv2.GaussianBlur(self.imageroi.copy(), (5, 5), 0)
            pixellast = 0
            for y in range(0,roiheight):
                for x in range(0,roiwidth):
                    pixel = blurred[y,x]
                    if pixel > pixellast:
                        pixellast = pixel
            medianpixel = np.median(blurred)
            trackSettings.minbright = ((pixellast - medianpixel)/2)+medianpixel
            #self.entryBright.delete(0, END)
            #self.entryBright.insert(0, trackSettings.minbright)
            trackSettings.objectfollow = True
    
    def right_click(self, event):
        trackSettings.objectfollow = False
        self.roibox = []
        self.roiboxlast = []
        self.imageroi = []
    
    def goup(self, event):
        trackSettings.mainviewY -= 1
    
    def godown(self, event):
        trackSettings.mainviewY += 1
        
    def goleft(self, event):
        trackSettings.mainviewX -= 1
    
    def goright(self, event):
        trackSettings.mainviewX +=1
    
    def prepare_img_for_tkinter(self):
        if self.collect_images is True:
            self.imgtk = []
            self.img = []
            ret, self.img = self.cap.read()
            if ret is True:
                self.height, self.width = self.img.shape[:2]
                if self.width > (trackSettings.screen_width*trackSettings.screenshrink):
                    shrinkfactor = (trackSettings.screen_width*trackSettings.screenshrink)/self.width
                    self.width = int(self.width * shrinkfactor)
                    self.height = int(self.height * shrinkfactor)
                    self.img = cv2.resize(self.img, (self.width, self.height), interpolation = cv2.INTER_AREA)
                elif self.height > (trackSettings.screen_height*trackSettings.screenshrink):
                    shrinkfactor = (trackSettings.screen_height*trackSettings.screenshrink)/self.height
                    self.width = int(self.width * shrinkfactor)
                    self.height = int(self.height * shrinkfactor)
                    self.img = cv2.resize(self.img, (self.width, self.height), interpolation = cv2.INTER_AREA)
                if trackSettings.flip == 'VerticalFlip':
                    self.img = cv2.flip(self.img, 0)
                if trackSettings.flip == 'HorizontalFlip':
                    self.img = cv2.flip(self.img, 1)
                if trackSettings.flip == 'VerticalHorizontalFlip':
                    self.img = cv2.flip(self.img, -1)
                self.img = imutils.rotate(self.img, trackSettings.rotate)                        
                #remember current time of the frame
                self.dnow = datetime.datetime.utcnow()
                self.height, self.width = self.img.shape[:2]
                self.datetimestring = str(self.dnow.strftime('%m%d%Y%H%M%S'))
                self.displayimg.bind("<MouseWheel>", self._on_mousewheel)
                self.displayimg.bind("<Motion>", self.mouse_position)
                self.displayimg.bind("<Button-1>", self.left_click)
                self.displayimg.bind("<Button-3>", self.right_click)
                self.mousebox = [(int(trackSettings.mousecoords[0]-(trackSettings.boxSize/2)),int(trackSettings.mousecoords[1]-(trackSettings.boxSize/2))),
                    (int(trackSettings.mousecoords[0]+(trackSettings.boxSize/2)),int(trackSettings.mousecoords[1]+(trackSettings.boxSize/2)))]
                self.centerbox = [(int(trackSettings.mainviewX-5),int(trackSettings.mainviewY - 5)),
                    (int(trackSettings.mainviewX+5),int(trackSettings.mainviewY+5))]
                self.crosshairbox = [(int(trackSettings.crosshairX-6),int(trackSettings.crosshairY - 6)),
                    (int(trackSettings.crosshairX+6),int(trackSettings.crosshairY+6))]
#make sure mouse coordinates are within bounds
                for idx, coord in enumerate(self.mousebox):
                    if coord[0] < 0:
                        x = 0
                    elif coord[0] > self.width:
                        x = self.width
                    else:
                        x = coord[0]
                    if coord[1] < 0:
                        y = 0
                    elif coord[1] > self.height:
                        y = self.height
                    else:
                        y = coord[1]
                    self.mousebox[idx] = (x,y)
                self.imgtk = self.img.copy()
                cv2.rectangle(self.imgtk,self.mousebox[0],self.mousebox[1],(255,0,0),2)
                cv2.rectangle(self.imgtk,self.centerbox[0],self.centerbox[1],(0,0,255),1)
                cv2.circle(self.imgtk,(trackSettings.crosshairX,trackSettings.crosshairY),8,(255,0,255),1)
                if trackSettings.objectfollow is True:
                    #trackSettings.minbright = self.entryBright.get()
                    self.roibox, self.imageroi = videotrak.get_x_y(self.img, self.roibox, self.imageroi)
                    #now check how much it's moved and calculate velocity.
                    self.xmotion = self.roibox[0][0] - self.roiboxlast[0][0]
                    self.ymotion = self.roibox[0][1] - self.roiboxlast[0][1]
                    #self.timedelta = self.dnow - self.dlast
                    #xpixelspersecond = self.xmotion/self.timedelta.total_seconds()
                    #ypixelspersecond = self.ymotion/self.timedelta.total_seconds()
                    #print(int(xpixelspersecond), ' pixels per second in X.', int(-1*ypixelspersecond), ' pixels per second in Y.', end='\r')
                    
                    self.roiboxlast = self.roibox
                    #self.dlast = self.dnow
                    if trackSettings.foundtarget is True:
                        cv2.rectangle(self.imgtk,self.roibox[0],self.roibox[1],(0,255,0),2)
                    else:
                        cv2.rectangle(self.imgtk,self.roibox[0],self.roibox[1],(0,255,255),2)
                    self.roiheight, self.roiwidth = self.imageroi.shape[:2]
                    self.targetX = self.roibox[0][0]+(self.roiwidth/2)
                    self.targetY = self.roibox[0][1]+(self.roiheight/2)

                    self.tracktkimg = PILImage.fromarray(self.imageroi)
                    self.tracktkimg = ImageTk.PhotoImage(image=self.tracktkimg)
                    self.trackimg.config(image=self.tracktkimg)
                    self.trackimg.img = self.tracktkimg
                    self.trackimg.grid(row = 0, column = 1)
                if self.recordvideo.get() == 1:
                    if trackSettings.previousrecord == 0:
                        self.fourcc = cv2.VideoWriter_fourcc(*str('mp4v'))
                        self.out = cv2.VideoWriter(str(self.datetimestring + '.mp4'),self.fourcc, 30, (self.width,self.height))
                        self.out.write(self.imgtk)
                        trackSettings.previousrecord = 1
                    else:
                        self.out.write(self.imgtk)
                else:
                    if trackSettings.previousrecord == 1:
                        self.out.release()
                        trackSettings.previousrecord = 0    
                self.b,self.g,self.r = cv2.split(self.imgtk)
                self.tkimg = cv2.merge((self.r,self.g,self.b))
                self.tkimg = PILImage.fromarray(self.tkimg)
                self.tkimg = ImageTk.PhotoImage(image=self.tkimg)
                self.displayimg.config(image=self.tkimg)
                self.displayimg.img = self.tkimg
                self.displayimg.grid(row = 0, column = 0)
            self._after_id = root.after(10, self.prepare_img_for_tkinter)
        else:
            print('Stopping Camera.')
            self.textbox.insert(END, str('Stopping Camera.\n'))
            self.textbox.see('end')
        
After = None
root = Tk()
b = buttons(root)
root.protocol("WM_DELETE_WINDOW", b.exitProg)
root.mainloop()
