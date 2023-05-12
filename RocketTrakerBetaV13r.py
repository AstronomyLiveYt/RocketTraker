from tkinter import *
from tkinter import filedialog
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
import geocoder
import serial
import io
import random
import threading
import win32com.client
import imutils
import pygame
import traceback
import launchlistdownloader
from PIL import Image as PILImage, ImageTk
from urllib.request import urlopen

class trackSettings:
    
    objectfollow = False
    telescopetype = 'LX200'
    mounttype = 'AltAz'
    tracking = False
    boxSize = 50
    mousecoords = (320,240)
    degorhours = 'Degrees'
    mainviewX = 320
    mainviewY = 240
    crosshairX = 320
    crosshairY = 240
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
    focuserCOM = 10
    focuserconnected = False
    trackingmode = 'Regular'


class KalmanFilter:

    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

    def Estimate(self, coordX, coordY):
        ''' This function estimates the position of the object'''
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        return predicted

class videotrak:
    
    def get_x_y(img, roibox, imageroi):
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
                key = cv2.waitKey(1) & 0xFF
                if key == ord('s'):
                    need_track_feature = True
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
    
    def __init__(self, master):
        trackSettings.screen_width = root.winfo_screenwidth()
        trackSettings.screen_height = root.winfo_screenheight()
        print(trackSettings.screen_width)
        print(trackSettings.screen_height)
        pygame.init()
        self.joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
        
        #Prepare for remote connection
        self.HEADERSIZE = 10
        self.remotejoystick = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.full_msg = ''
        self.new_msg = True
        
        self.collect_images = False
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
        self.entryLat.grid(row = 5, column = 1)
        self.labelLon = Label(self.bottomframe, text='Longitude (E+)')
        self.labelLon.grid(row=6, column = 0)
        self.entryLon = Entry(self.bottomframe)
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
            config.close()
        except Exception as e:
            print(e)
            print('Config file not present or corrupted.')
        
        try:
            geolocation = geocoder.ip('me')
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
        self.startButton5 = Button(self.bottomframe, text='Connect Scope', command=self.set_tracking)
        self.startButton5.grid(row=1, column = 1)
        self.startButtonCross = Button(self.bottomframe, text='Reset Crosshair', command=self.set_crosshair)
        self.startButtonCross.grid(row=7, column = 1)
        try:
            self.dropDown1 = OptionMenu(self.bottomframe, self.droplist, *self.LAUNCHES, command=self.LaunchSelect)
            self.dropDown1.grid(row=7, column = 0)
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
        
        self.NETLabel = Label(self.bottomframe, text='T0 Time')
        self.NETLabel.grid(row = 4, column = 3)
        self.entryNET = Entry(self.bottomframe)
        self.entryNET.grid(row = 5, column = 3)  
        
        self.reversexaxis = Checkbutton(self.bottomframe, text="Reverse Joystick X Axis", variable=self.joyxrev).grid(row=9, column = 0, sticky=W)
        self.reverseyaxis = Checkbutton(self.bottomframe, text="Reverse Joystick Y Axis", variable=self.joyyrev).grid(row=9, column = 1, sticky=W)
        self.logobservations = Checkbutton(self.bottomframe, text="Log Target Alt/Az", variable=self.logaltaz).grid(row=9, column = 2, sticky=W)
        self.logtelescopepos = Checkbutton(self.bottomframe, text="Log Telescope Alt/Az", variable=self.logtelpos).grid(row=9, column = 3, sticky=W)
        self.recordv = Checkbutton(self.bottomframe, text="Record Video", variable=self.recordvideo).grid(row=10, column = 0, sticky=W)
        self.countdownclockchoice = Checkbutton(self.bottomframe, text="Use Countdown Clock", variable=self.usecountdown).grid(row=10, column = 3, sticky=W)
        self.HoldLabel = Label(self.bottomframe, text='Hold Rate OFF')
        self.HoldLabel.grid(row = 10, column = 1)
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
        
        self.joystickMenu = Menu(self.menu)
        self.menu.add_cascade(label='Joystick', menu=self.joystickMenu)
        self.joystickMenu.add_command(label='Configure Joystick Bindings', command=self.rebindJoystick)
        
        self.focuserMenu = Menu(self.menu)
        self.menu.add_cascade(label='Focuser', menu=self.focuserMenu)
        self.focuserMenu.add_command(label="Configure Astro's Focuser", command=self.configFocuser)
        
        #Initialize T0 time in entry box
        indexnumber = self.LAUNCHES.index(self.droplist.get())
        self.entryNET.delete(0, 'end')
        self.entryNET.insert(0, self.NET[indexnumber]) 
    
    def setRegularMode(self):
        trackSettings.trackingmode = 'Regular'
        
    def setAdaptiveMode(self):
        trackSettings.trackingmode = 'Adaptive'
        
    def setHorizonMode(self):
        trackSettings.trackingmode = 'Horizon'
        
    def configFocuser(self):
        self.focuserwindow= Toplevel(root)
        self.focuserwindow.geometry("300x300")
        self.focuserwindow.title("Focuser Configuration")
        self.commandsLabel = Label(self.focuserwindow, text='Serial Port: ',font=('Arial 14 bold'))
        self.commandsLabel.grid(row = 0, column = 0)
        self.entryFocuserCom = Entry(self.focuserwindow)
        self.entryFocuserCom.grid(row = 0, column = 1)
        self.entryFocuserCom.insert(0, trackSettings.focuserCOM)
        if trackSettings.focuserconnected is False:
            self.focuserconnectButton = Button(self.focuserwindow, text=str('Connect'),command = self.connectFocuser,font=('Arial 14 bold'))
        if trackSettings.focuserconnected is True:
            self.focuserconnectButton = Button(self.focuserwindow, text=str('Disconnect'),command = self.connectFocuser,font=('Arial 14 bold'))
        self.focuserconnectButton.grid(row = 1, column = 1)
    
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
        
    def set_crosshair(self):
        trackSettings.crosshairX = trackSettings.mainviewX
        trackSettings.crosshairY = trackSettings.mainviewY        
        
    def filePicker(self):
        trackSettings.trajFile = filedialog.askopenfilename(initialdir = ".",title = "Select Trajectory file",filetypes = (("csv files","*.csv"),("all files","*.*")))
        trackSettings.fileSelected = True
        print(trackSettings.trajFile)
        self.textbox.insert(END, str(str(trackSettings.trajFile)+'\n'))
        self.textbox.see('end')
        
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
                self.get_remote_joy_data = threading.Thread(target=self.get_joy)
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
                self.get_remote_joy_data = threading.Thread(target=self.get_joy)
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
        config.close()
        if self.recordvideo.get() == 1:
            self.out.release()
        sys.exit()

    
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
                self.simcalc = threading.Thread(target=self.run_sim)
                self.simcalc.start()
                self.simthread = threading.Thread(target=self.simulate_launch)
                self.simulateButton.configure(text='Stop Simulation')
                self.simthread.start()
            
        else:
            trackSettings.runningsimulation = False
            trackSettings.feedingdata = False
            self.simulateButton.configure(text='Simulate Launch')

    def arm_launch_tracking(self):
        if trackSettings.runningsimulation is False and trackSettings.runninglaunch is False and trackSettings.fileSelected is True:
            trackSettings.runninglaunch = True
            if len(self.joysticks) == 0 and trackSettings.joystickconnected is False:
                print('Connect Joystick And Restart Program!')
                self.textbox.insert(END, 'Connect Joystick And Restart Program!\n')
                self.textbox.see('end')
                trackSettings.runninglaunch = False
            else:
                if trackSettings.joystickconnected is False:
                    self.joysticks[0].init()
                self.simcalc = threading.Thread(target=self.simulate_launch)
                self.simcalc.start()
                self.launchthread = threading.Thread(target=self.run_launch)
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
            self.launchButton.configure(text='Arm Launch Tracking')
    
    def run_launch(self):
        self.t0 = self.entryNET.get()
        self.t0 = datetime.datetime.strptime(self.t0, '%Y-%m-%dT%H:%M:%SZ')
        currenttime = datetime.datetime.utcnow()
        countdown = (self.t0 - currenttime).total_seconds()
        while trackSettings.buttonpushed is False and trackSettings.runninglaunch is True:
            currenttime = datetime.datetime.utcnow()
            countdown = (self.t0 - currenttime).total_seconds()
            hours = countdown // 3600
            minutes = (countdown % 3600) // 60
            seconds = countdown % 60
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
        initialtime = datetime.datetime.utcnow()
        endoffile = False
        timedeltalast = 0
        ####Do this if we're set to wait at the horizon mode###
        if trackSettings.trackingmode == 'Horizon':
            if trackSettings.fileSelected is True:
                df = pd.read_csv(trackSettings.trajFile, sep=',', encoding="utf-8")
                altlist1 = []
                azlist1 = []
                timelist = []
                ####################NEED TO LOCATE WHERE IT WILL HIT THE HORIZON AND WAIT THERE!
                horizonalt = -999
                horizonaz = -999
                waittime = 0
                for index, row in df.iterrows():
                    timelist.append(row['time'])
                    #Find the row that it rises on horizon and slew there
                    if row['elevationDegs'] > 0 and horizonalt < -990:
                        waittime = row['time']
                        startgoingtime = initialtime + datetime.timedelta(seconds=waittime)
                        horizonalt = row['elevationDegs']
                        horizonaz = row['azimuthDegs']
                        #Slew to pad location
                        launchobserver = ephem.Observer()
                        launchobserver.lat = (str(self.entryLat.get()))
                        launchobserver.lon = (str(self.entryLon.get()))
                        launchobserver.date = datetime.datetime.utcnow()
                        launchobserver.pressure = 0
                        launchobserver.epoch = launchobserver.date
                        raslew, decslew = launchobserver.radec_of(math.radians(horizonaz), math.radians(horizonalt))
                        self.tel.SlewToCoordinates((math.degrees(raslew)/15),math.degrees(decslew))
                        self.tel.MoveAxis(0, 0)
                        self.tel.MoveAxis(1, 0)
                    altlist1.append(row['elevationDegs'])
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
                    timedelta = (currenttime - initialtime).total_seconds()
                    hours = timedelta // 3600
                    minutes = (timedelta % 3600) // 60
                    seconds = timedelta % 60
                    self.countdowntext.set(str('t+ '+str(math.trunc(hours))+' hours '+str(math.trunc(minutes))+' minutes '+str(math.trunc(seconds))+' seconds'))
                    timedelta2 = timedelta - timedeltalast
                    timedeltalast = timedelta
                    df_sort = df.iloc[(df['time']-timedelta).abs().argsort()[:2]]
                    df_sort = df_sort.sort_values(by=['time'])
                    altlist1 = []
                    azlist1 = []
                    timelist = []
                    #Convert absolute coordinates to relative rates
                    index = 0
                    predictedalt = df_sort.iloc[index].loc['elevationDegs']
                    predictedaz = df_sort.iloc[index].loc['azimuthDegs']
                    altrate = df_sort.iloc[index+1].loc['elevationDegs'] - df_sort.iloc[index].loc['elevationDegs']
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
                        trackSettings.runninglaunch = False
                        trackSettings.buttonpushed = False
                        trackSettings.feedingdata = False
                        self.launchButton.configure(text='Arm Launch Tracking')
                        trackSettings.joytracking = True
                        self.trackthread = threading.Thread(target=self.track)
                        self.startButton4.configure(text='Stop Joystick Tracking')
                        self.trackthread.start()
                    if math.isinf(altrate) or math.isinf(azrate):
                        pass
                    else:
                        #print(timedelta, alt, az, altrate, azrate)
                        self.timedeltaout, self.altout, self.azout, self.altrateout, self.azrateout, self.predictalt, self.predictaz = timedelta, alt, az, altrate, azrate, predictedalt, predictedaz
                        trackSettings.feedingdata = True
                else:
                    timedelta = (startgoingtime - currenttime).total_seconds()
                    hours = timedelta // 3600
                    minutes = (timedelta % 3600) // 60
                    seconds = timedelta % 60
                    self.countdowntext.set(str('Rising in: '+str(math.trunc(hours))+' hours '+str(math.trunc(minutes))+' minutes '+str(math.trunc(seconds))+' seconds'))
                    if trackSettings.joystickconnected is False:
                        pygame.event.pump()
                        if self.joysticks[0].get_button(trackSettings.starttrackbutton) > 0:
                             startgoingtime = currenttime
                    else:
                        if self.remotejoybutton2 > 0:
                            startgoingtime = currenttime
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
                    altlist1.append(row['elevationDegs'])
                    azlist1.append(row['azimuthDegs'])
                    #Pre-Calculate rates
                    if (index+2)<r:
                        altrate = df.iloc[index+1].loc['elevationDegs'] - df.iloc[index].loc['elevationDegs']
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
                currentaz = self.tel.Azimuth
                currentalt = self.tel.Altitude
                try:
                    for index, row in df.iterrows():
                        currentdiff = math.sqrt(abs(row['elevationDegs']-currentalt)**2+abs(row['azimuthDegs']-currentaz)**2)+(abs(row['time']-currentindex))
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
                    if self.joysticks[0].get_button(7) > 0:
                        currentindex = r+999999999999999999999999999999
                    #check to see if we reached the end of the interpolation, remember to account for extra 20 seconds before launch)
                    if currentindex > (r):
                        endoffile = True
                        trackSettings.runninglaunch = False
                        trackSettings.buttonpushed = False
                        trackSettings.feedingdata = False
                        self.launchButton.configure(text='Arm Launch Tracking')
                        trackSettings.joytracking = True
                        self.trackthread = threading.Thread(target=self.track)
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
                    altlist1.append(row['elevationDegs'])
                    azlist1.append(row['azimuthDegs'])
                altlast = altlist1[0]
                azlast = azlist1[0]
            else:
                endoffile = True
            while trackSettings.runninglaunch is True and endoffile is False:
                time.sleep(0.001)
                r, c = df.shape
                currenttime = datetime.datetime.utcnow()
                timedelta = (currenttime - initialtime).total_seconds()
                hours = timedelta // 3600
                minutes = (timedelta % 3600) // 60
                seconds = timedelta % 60
                self.countdowntext.set(str('t+ '+str(math.trunc(hours))+' hours '+str(math.trunc(minutes))+' minutes '+str(math.trunc(seconds))+' seconds'))
                timedelta2 = timedelta - timedeltalast
                timedeltalast = timedelta
                df_sort = df.iloc[(df['time']-timedelta).abs().argsort()[:2]]
                df_sort = df_sort.sort_values(by=['time'])
                altlist1 = []
                azlist1 = []
                timelist = []
                #Convert absolute coordinates to relative rates
                index = 0
                altrate = df_sort.iloc[index+1].loc['elevationDegs'] - df_sort.iloc[index].loc['elevationDegs']
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
                if self.joysticks[0].get_button(7) > 0:
                    timedelta = r+1
                #check to see if we reached the end of the interpolation, remember to account for extra 20 seconds before launch)
                if timedelta > (r):
                    endoffile = True
                    trackSettings.runninglaunch = False
                    trackSettings.buttonpushed = False
                    trackSettings.feedingdata = False
                    self.launchButton.configure(text='Arm Launch Tracking')
                    trackSettings.joytracking = True
                    self.trackthread = threading.Thread(target=self.track)
                    self.startButton4.configure(text='Stop Joystick Tracking')
                    self.trackthread.start()
                if math.isinf(altrate) or math.isinf(azrate):
                    pass
                else:
                    #print(timedelta, alt, az, altrate, azrate)
                    self.timedeltaout, self.altout, self.azout, self.altrateout, self.azrateout = timedelta, alt, az, altrate, azrate
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
                altlist1.append(row['elevationDegs'])
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
            predictedalt = df_sort.iloc[index].loc['elevationDegs']
            predictedaz = df_sort.iloc[index].loc['azimuthDegs']
            altrate = df_sort.iloc[index+1].loc['elevationDegs'] - df_sort.iloc[index].loc['elevationDegs']
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
                self.trackthread = threading.Thread(target=self.track)
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
                        azrate = (joy0*throttle)*self.axis0rate + float(self.azrateout) + (0.000001*random.randrange(0, 5, 1))
                        altrate = (joy1*throttle*-1)*self.axis0rate + float(self.altrateout) + (0.000001*random.randrange(0, 5, 1))
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
                            azrate = 0.0 + float(self.azrateout) + (0.000001*random.randrange(0, 5, 1))
                            altrate = 0.0 + float(self.altrateout) + (0.000001*random.randrange(0, 5, 1))
                        try:                            
                            self.textbox.insert(END, str('Az Rate: ' + str(azrate) + ' Alt Rate: ' + str(altrate) +'\n'))
                            self.textbox.see('end')
                        except Exception as poop:
                            print(poop)
                            pass
                        #if holdrate is False:
                        if abs(abs(commandedazratelast) - abs(azrate)) > 0.001:
                            self.tel.MoveAxis(0, azrate)
                            commandedazratelast = azrate
                        if abs(abs(commandedaltratelast) - abs(altrate)) > 0.001:
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
                        currentaz = self.tel.Azimuth
                        currentalt = self.tel.Altitude
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
                                azrate =  float(self.azrateout) + (azdiff) 
                                #if math.fabs(self.diffaltlast) < math.fabs(altdiff):
                                if self.pretrack is True:
                                    self.altratelast = altrate
                                    altaccel = 0
                                #we're done with setup for the first pass through the loop so dump pretrack
                                    self.pretrack = False
                                if trackSettings.foundtarget is True:
                                    altaccel = altrate - self.altratelast
                                    self.altratelast = altrate
                                altrate =  float(self.altrateout) + (altdiff)
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
            self.trackthread = threading.Thread(target=self.track)
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
                        currentaz = self.tel.Azimuth
                        currentalt = self.tel.Altitude
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
                                rarate = rarate + math.degrees(radiff)
                                #if math.fabs(self.diffdeclast) < math.fabs(decdiff):
                                decrate = decrate + math.degrees(decdiff)
                                if rarate > self.axis0rate:
                                    rarate = self.axis0rate
                                if rarate < (-1*self.axis0rate):
                                    rarate = (-1*self.axis0rate)
                                if decrate > self.axis1rate:
                                    decrate = self.axis1rate
                                if decrate < (-1*self.axis1rate):
                                    decrate = (-1*self.axis1rate)
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
            self.tel.AbortSlew
    
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
            imagethread = threading.Thread(target=self.prepare_img_for_tkinter)
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
    
    def set_tracking(self):
        if trackSettings.tracking is False:
            trackSettings.tracking = True
            print('Connecting to Scope.')
            self.textbox.insert(END, 'Connecting to Scope.\n')
            self.textbox.see('end')
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
        calibthread = threading.Thread(target=self.set_calibration)
        calibthread.start()
    
    def set_calibration(self):
        if trackSettings.calibratestart is False:
            trackSettings.calibratestart = True
        else:
            if trackSettings.telescopetype == 'ASCOM':
                self.tel.MoveAxis(1, 0.0)
                #self.tel.AbortSlew()
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
                    self.X1 = math.radians(self.tel.Azimuth)
                    self.Y1 = math.radians(self.tel.Altitude)
                    startx = self.targetX
                    starty = self.targetY
                    if starty < (self.height/2):
                        distmoved = 0
                        self.tel.MoveAxis(1, trackSettings.calspeed)
                        while distmoved < 100 and trackSettings.calibratestart is True:
                            self.tel.MoveAxis(1, trackSettings.calspeed)
                            currentx = self.targetX
                            currenty = self.targetY
                            distmoved = math.sqrt((startx-currentx)**2+(starty-currenty)**2)
                            time.sleep(0.01)
                        self.tel.MoveAxis(1, 0.0)
                        #self.tel.AbortSlew()
                        self.X2 = math.radians(self.tel.Azimuth)
                        self.Y2 = math.radians(self.tel.Altitude)
                        self.separation_between_coordinates()
                        self.imagescale = self.separation/distmoved
                        print(self.imagescale, ' degrees per pixel.')
                        self.textbox.insert(END, str('Image scale: '+str(self.imagescale)+' degrees per pixel.\n'))
                        self.textbox.see('end')
                    else:
                        distmoved = 0
                        self.tel.MoveAxis(1, (-1*trackSettings.calspeed))
                        while distmoved < 100 and trackSettings.calibratestart is True:
                            self.tel.MoveAxis(1, (-1*trackSettings.calspeed))
                            currentx = self.targetX
                            currenty = self.targetY
                            distmoved = math.sqrt((startx-currentx)**2+(starty-currenty)**2)
                            time.sleep(0.01)
                        self.tel.MoveAxis(1, 0.0)
                        #self.tel.AbortSlew()
                        self.X2 = math.radians(self.tel.Azimuth)
                        self.Y2 = math.radians(self.tel.Altitude)
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
                            self.tel.MoveAxis(1, trackSettings.calspeed)
                            currentx = self.targetX
                            currenty = self.targetY
                            distmoved = math.sqrt((startx-currentx)**2+(starty-currenty)**2)
                            time.sleep(0.01)
                        self.tel.MoveAxis(1, 0.0)
                        #self.tel.AbortSlew()
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
                            self.tel.MoveAxis(1, (-1*trackSettings.calspeed))
                            currentx = self.targetX
                            currenty = self.targetY
                            distmoved = math.sqrt((startx-currentx)**2+(starty-currenty)**2)
                            time.sleep(0.01)
                        self.tel.MoveAxis(1, 0.0)
                        #self.tel.AbortSlew()
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
            self.dnow = datetime.datetime.now()
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
                if self.width > (trackSettings.screen_width*0.45):
                    shrinkfactor = (trackSettings.screen_width*0.45)/self.width
                    self.width = int(self.width * shrinkfactor)
                    self.height = int(self.height * shrinkfactor)
                    self.img = cv2.resize(self.img, (self.width, self.height), interpolation = cv2.INTER_AREA)
                elif self.height > (trackSettings.screen_height*0.45):
                    shrinkfactor = (trackSettings.screen_height*0.45)/self.height
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
                self.dnow = datetime.datetime.now()
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
            After = root.after(10,self.prepare_img_for_tkinter)
        else:
            print('Stopping Camera.')
            self.textbox.insert(END, str('Stopping Camera.\n'))
            self.textbox.see('end')
        
After = None
root = Tk()
b = buttons(root)
root.mainloop()
