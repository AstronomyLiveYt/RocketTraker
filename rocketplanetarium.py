import numpy as np
import datetime
import ephem
import math
import time
import base64
import os
import cv2
import pandas as pd
import os

def equatorial_to_horizon(dec, ra, lat, lst):
    hour = lst - ra
    if math.degrees(hour) < 0:
        hour = math.radians(math.degrees(hour) + 360)
    elif math.degrees(hour) > 360:
        hour = math.radians(math.degrees(hour) - 360)
    alt = math.asin(math.sin(dec)*math.sin(lat)+math.cos(dec)*math.cos(lat)*math.cos(hour))
    az = math.acos((math.sin(dec)-math.sin(lat)*math.sin(alt))/(math.cos(lat)*math.cos(alt)))
    if math.sin(hour)>0:
        az = (360 - (az * 180/math.pi))*(math.pi/180)
    az = math.radians(math.degrees(az)-90)
    if math.degrees(az) > 360:
        az = math.radians(math.degrees(az) - 360)
    return(alt, az)
    
def atm_refraction(altitude, pressure, temperature):
    if altitude>15:
        refraction = 0.00452*pressure*math.tan(math.radians((90-altitude)/(273+temperature)))
    else:
        refraction = (pressure*(0.1594+(0.0196*altitude)+(0.00002*altitude**2)))/((273+temperature)*(1+(0.505*altitude)+(0.0845*altitude**2)))
    return(refraction)

def render_sky(starcat,df,dnow,observatory,telalt,telaz,rocketpath,alignedpoints,rendersize, maglimit):
    O = (255, 180, 157)
    B = (255, 188, 167)
    A = (255, 209, 192)
    F = (255, 232, 228)
    G = (236, 245, 255)
    K = (174, 215, 255)
    M = (123, 187, 255)
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    halfrendersize = round(rendersize/2)
    blankimage = np.zeros(((rendersize),(rendersize),3), np.uint8)
    #dnow = dnowstart + datetime.timedelta(minutes=timedelta)
    #dnow = datetime.datetime.utcnow()
    #observatory.date = dnow
    #print(dnow)
    #Do constellation lines
    trueindex = -1
    for index, row in df.iterrows():
        trueindex +=1
        rah = row['ra']
        ra = math.radians(float(rah)*15)
        dec = math.radians(row['dec'])
        starj2000 = ephem.Equatorial(ra, dec, epoch=ephem.J2000)
        starEOD = ephem.Equatorial(starj2000, epoch=dnow)
        raEOD = (starEOD.ra)
        decEOD = (starEOD.dec)
        alt, az = equatorial_to_horizon(decEOD, raEOD, observatory.lat, observatory.sidereal_time())
        refract = atm_refraction(math.degrees(alt), observatory.pressure, observatory.temp)
        alt += math.radians(refract)
        if trueindex > 0:
            try:
                az1 = az
                zenith = 90-math.degrees(alt)
                x1 = int((zenith*math.cos(az1))*(halfrendersize/90)+halfrendersize)
                y1 = int((zenith*math.sin(az1))*(halfrendersize/90)+halfrendersize)
                
                az2 = azlast
                zenith = 90-math.degrees(altlast)
                x2 = int((zenith*math.cos(az2))*(halfrendersize/90)+halfrendersize)
                y2 = int((zenith*math.sin(az2))*(halfrendersize/90)+halfrendersize)
                if alt > 0 and altlast > 0:
                    cv2.line(blankimage,(x1,y1),(x2,y2),(255,255,255),1)
            except:
                trueindex = -1
                pass
        altlast = alt
        azlast = az
    #Load the rocket trajectory
    if len(rocketpath)>2:
        trueindex = -1
    for index, rocketcoord in enumerate(rocketpath):
        trueindex +=1
        alt = math.radians(rocketcoord[0])
        az = math.radians(float(rocketcoord[1]-90))
        if trueindex > 0:
            az1 = az
            zenith = 90-math.degrees(alt)
            x1 = int((zenith*math.cos(az1))*(halfrendersize/90)+halfrendersize)
            y1 = int((zenith*math.sin(az1))*(halfrendersize/90)+halfrendersize)
            
            az2 = azlast
            zenith = 90-math.degrees(altlast)
            x2 = int((zenith*math.cos(az2))*(halfrendersize/90)+halfrendersize)
            y2 = int((zenith*math.sin(az2))*(halfrendersize/90)+halfrendersize)
            if alt > 0 and altlast > 0:
                cv2.line(blankimage,(x1,y1),(x2,y2),(0,255,255),1)
        altlast = alt
        azlast = az
    #Load up the stars
    staraltazlist = []
    #Alt, az, name, mag
    for star in enumerate(starcat):
        rahour = (int(star[1][0]/15))
        ramin = (int(((star[1][0]/15)-rahour)*60))
        rasec = (round((((((star[1][0]/15)-rahour)*60)-ramin)*60),2))
        decdeg = (int(star[1][1]))
        decmin = (int(((star[1][1])-decdeg)*60))
        decsec = (round((((((star[1][1])-decdeg)*60)-decmin)*60),1))
        
        rahour = str(rahour)
        ramin = str(ramin)
        rasec = str(rasec)
        decdeg = str(decdeg)
        decmin = str(decmin)
        decsec = str(decsec)
        
        spectral = star[1][4]
        mag = star[1][3]
        try:
            pixsize = int(5.0/(2+float(mag)))+1 
            ra = str(rahour+':'+ramin+':'+rasec)
            dec = str(decdeg+':'+decmin+':'+decsec)
            #star = ephem.Equatorial(ra, dec, epoch=dnow)
            #ra = star.ra
            #dec = star.dec
            if spectral == 'O':
                starcolor = O
            if spectral == 'B':
                starcolor = B
            if spectral == 'A':
                starcolor = A
            if spectral == 'F':
                starcolor = F
            if spectral == 'G':
                starcolor = G
            if spectral == 'K':
                starcolor = K
            if spectral == 'M':
                starcolor = M
            dec = math.radians(star[1][1])
            ra = math.radians(star[1][0])
            alt,az = equatorial_to_horizon(dec, ra, observatory.lat, observatory.sidereal_time())
            refract = atm_refraction(math.degrees(alt), observatory.pressure, observatory.temp)
            alt += math.radians(refract)
            if math.degrees(alt) > 00 and float(mag) < maglimit:
                outalt = math.degrees(alt)
                #You're rotating 90 degrees at the step of converting to alt/az so fix this before sending the coordinates back to the main program
                outaz = math.degrees(az)+90
                staraltazlist.append((outalt,outaz,star[1][2],star[1][3]))
                zenith = 90-math.degrees(alt)
                x = (zenith*math.cos(az))*(halfrendersize/90)+halfrendersize
                y = (zenith*math.sin(az))*(halfrendersize/90)+halfrendersize
                cv2.circle(blankimage,(int(x),int(y)), int(pixsize), starcolor, -1)
                #print(x, y)
                #print(math.degrees(alt), math.degrees(az))
                #print(line)
                #print(rahour, ramin, rasec, decdeg, decmin, decsec, spectral, mag, int(pixsize), math.degrees(alt), math.degrees(az), int(x), int(y))
        except:
            pass
    #Load up the moon, sun, and planets
    blankimage = cv2.flip(blankimage, 1)
    celestial = ephem.Moon(observatory)
    alt = celestial.alt
    if math.degrees(alt) > 0:
        az = math.radians(math.degrees(celestial.az)-90)
        zenith = 90-math.degrees(alt)
        x = halfrendersize-(zenith*math.cos(az))*(halfrendersize/90)
        y = (zenith*math.sin(az))*(halfrendersize/90)+halfrendersize
        cv2.circle(blankimage,(int(x),int(y)), int(15), (200,200,200), -1)
        cv2.putText(blankimage,'Moon',(int(x+10),int(y)), font, 1,(255,255,255),1,cv2.LINE_AA)
    celestial = ephem.Sun(observatory)
    alt = celestial.alt
    if math.degrees(alt) > 0:
        az = math.radians(math.degrees(celestial.az)-90)
        zenith = 90-math.degrees(alt)
        x = halfrendersize-(zenith*math.cos(az))*(halfrendersize/90)
        y = (zenith*math.sin(az))*(halfrendersize/90)+halfrendersize
        cv2.circle(blankimage,(int(x),int(y)), int(15), (0,255,255), -1)
        cv2.putText(blankimage,'Sun',(int(x+10),int(y)), font, 1,(255,255,255),1,cv2.LINE_AA)
    celestial = ephem.Mercury(observatory)
    alt = celestial.alt
    if math.degrees(alt) > 0:
        az = math.radians(math.degrees(celestial.az)-90)
        zenith = 90-math.degrees(alt)
        x = halfrendersize-(zenith*math.cos(az))*(halfrendersize/90)
        y = (zenith*math.sin(az))*(halfrendersize/90)+halfrendersize
        cv2.circle(blankimage,(int(x),int(y)), int(5), (255,100,100), -1)
        cv2.putText(blankimage,'Mercury',(int(x+5),int(y)), font, 1,(255,255,255),1,cv2.LINE_AA)
    celestial = ephem.Venus(observatory)
    alt = celestial.alt
    if math.degrees(alt) > 0:
        az = math.radians(math.degrees(celestial.az)-90)
        zenith = 90-math.degrees(alt)
        x = halfrendersize-(zenith*math.cos(az))*(halfrendersize/90)
        y = (zenith*math.sin(az))*(halfrendersize/90)+halfrendersize
        cv2.circle(blankimage,(int(x),int(y)), int(5), (255,255,255), -1)
        cv2.putText(blankimage,'Venus',(int(x+5),int(y)), font, 1,(255,255,255),1,cv2.LINE_AA)
    celestial = ephem.Mars(observatory)
    alt = celestial.alt
    if math.degrees(alt) > 0:
        az = math.radians(math.degrees(celestial.az)-90)
        zenith = 90-math.degrees(alt)
        x = halfrendersize-(zenith*math.cos(az))*(halfrendersize/90)
        y = (zenith*math.sin(az))*(halfrendersize/90)+halfrendersize
        cv2.circle(blankimage,(int(x),int(y)), int(5), (50,50,255), -1)
        cv2.putText(blankimage,'Mars',(int(x+5),int(y)), font, 1,(255,255,255),1,cv2.LINE_AA)
    celestial = ephem.Jupiter(observatory)
    alt = celestial.alt
    if math.degrees(alt) > 0:
        az = math.radians(math.degrees(celestial.az)-90)
        zenith = 90-math.degrees(alt)
        x = halfrendersize-(zenith*math.cos(az))*(halfrendersize/90)
        y = (zenith*math.sin(az))*(halfrendersize/90)+halfrendersize
        cv2.circle(blankimage,(int(x),int(y)), int(5), (150,150,255), -1)
        cv2.putText(blankimage,'Jupiter',(int(x+5),int(y)), font, 1,(255,255,255),1,cv2.LINE_AA)
    celestial = ephem.Saturn(observatory)
    alt = celestial.alt
    if math.degrees(alt) > 0:
        az = math.radians(math.degrees(celestial.az)-90)
        zenith = 90-math.degrees(alt)
        x = halfrendersize-(zenith*math.cos(az))*(halfrendersize/90)
        y = (zenith*math.sin(az))*(halfrendersize/90)+halfrendersize
        cv2.circle(blankimage,(int(x),int(y)), int(5), (100,100,255), -1)
        cv2.putText(blankimage,'Saturn',(int(x+5),int(y)), font, 1,(255,255,255),1,cv2.LINE_AA)
    celestial = ephem.Uranus(observatory)
    alt = celestial.alt
    if math.degrees(alt) > 0:
        az = math.radians(math.degrees(celestial.az)-90)
        zenith = 90-math.degrees(alt)
        x = halfrendersize-(zenith*math.cos(az))*(halfrendersize/90)
        y = (zenith*math.sin(az))*(halfrendersize/90)+halfrendersize
        cv2.circle(blankimage,(int(x),int(y)), int(5), (255,50,50), -1)
        cv2.putText(blankimage,'Uranus',(int(x+5),int(y)), font, 1,(255,255,255),1,cv2.LINE_AA)
    celestial = ephem.Neptune(observatory)
    alt = celestial.alt
    if math.degrees(alt) > 0:
        az = math.radians(math.degrees(celestial.az)-90)
        zenith = 90-math.degrees(alt)
        x = halfrendersize-(zenith*math.cos(az))*(halfrendersize/90)
        y = (zenith*math.sin(az))*(halfrendersize/90)+halfrendersize
        cv2.circle(blankimage,(int(x),int(y)), int(5), (255,0,0), -1)
        cv2.putText(blankimage,'Neptune',(int(x+5),int(y)), font, 1,(255,255,255),1,cv2.LINE_AA)
    if telalt > 0:
        az = math.radians(telaz-90)
        zenith = 90-telalt
        x = halfrendersize-(zenith*math.cos(az))*(halfrendersize/90)
        y = (zenith*math.sin(az))*(halfrendersize/90)+halfrendersize
        cv2.circle(blankimage,(int(x),int(y)), int(5), (0,0,255), 2)
        #cv2.putText(blankimage,'Current Scope',(int(x+5),int(y)), font, 1,(255,255,255),1,cv2.LINE_AA)
    cv2.circle(blankimage,(int(halfrendersize),int(halfrendersize)), int(halfrendersize), (255,255,255), 1)
    #out.write(blankimage)
    #tlefile = ""
    return(blankimage,staraltazlist)
    #cv2.imshow('Output',blankimage)
    #cv2.waitKey(0)
