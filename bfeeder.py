#!/usr/bin/env python3

# Butterfly feeder - the code
# The manual: https://bit.ly/2pCb3GA
#
# Revision 0.1, first released version - 20200106/mik
#
# avionics@skyracer.net
 
import os
import sys
import time
import difflib
import pigpio
import time
import re
import serial
import datetime
import RPi.GPIO as GPIO
import binascii
from math import *
from shutil import copyfile

# The butterfly and the ADS-B reciver has different com speed
intComSpeedADSB = 115200

intComSpeedFLARM = 38400 # 19200
intComPinFLARM_RX = 18
intComPinFLARM_TX = 22

intComSpeedBUTTERFLY = 38400 # 57600
intComPinBUTTERFLY = 17

intSpeed = 0
intPlaneCounter = 0
listADSBSentences = []
listADSBID = []
listNMEA = []
listFLARMID = []
tmpNMEA = ""
strPFLAU = ""
count = -1
timerCount = 0
init = 1
go = 1

MyLat = 0.00000		# This units latitude
MyLong = 0.00000	# This units longitude
MyAlt = 0           # This units altitude
MySpeed = 0         # This units (ground) speed
MyTrack = 0         # This units track
MyTime = "000000"	# This units time
MyDate = ""			# This units date
MyID = ""			# This units identity

fileName = "/mnt/ram/data.txt"
logFile = "log.txt"
myIDFile = "/home/pi//butterflyfeeder/config/myID.txt"

ZoomFactor = 10
MaxRange = 60000 # Meters
MaxDeltaAlt = 20000

tmpNMEA = ""

# Properties of the com port
#serADSB = serial.Serial ("/dev/ttyS0")  # Open named port
#serADSB = serial.Serial ("/dev/ttyAMA0")  # Open named port

serADSB = serial.Serial ("/dev/ttyS0",intComSpeedADSB)  # Open named port

#serADSB.baudrate = intComSpeedADSB      # Set baud rate to 9600
serADSB.bytesize = serial.EIGHTBITS     # number of bits per bytes
serADSB.parity = serial.PARITY_NONE     # set parity check: no parity
serADSB.stopbits = serial.STOPBITS_ONE  # number of stop bits
serADSB.timeout = None                  # block read
serADSB.xonxoff = False                 # disable software flow control
serADSB.rtscts = False                  # disable hardware (RTS/CTS) flow control
serADSB.dsrdtr = False                  # disable hardware (DSR/DTR) flow control


class clRADIOIDMessage(object):
    """
	Time
	RecieverWarning
	Lat
	East
	Long
	North
	VelH
	Track
	Date
    """
    #def __init__(self, sentance, diffLat=0, diffLong=0):
    def __init__(self, sentance):
        import time

		# $PFLAC,A,RADIOID,1,4AD4F5*02
		# PFLAC: $PFLAC
		# RecieverWarning: A
		# RADIOID: RADIOID
		# Identity: 4AD4F5
        # CRC: *02

        (self.GPRMC,
		 self.PFLAC ,
		 self.RecieverWarning ,
		 self.RADIOID ,
		 self.Identity,
         self.CRC
        ) = sentance.replace("\r\n","").replace("*",",").split(",")

def subGetFLARM_ID(TX,RX):
	# This sub is supposed to identify the FLARM identity and the com speed

	stopWhile = 0
	strBlank = ""

	# The usual suspect com speed
	ComSpeed = [38400, 9600, 19200, 28800, 4800, 56000, 56000]
	ComSpeed_Pointer = 0

	while stopWhile == 0:

		if ComSpeed_Pointer > 4:

			# No joy, all of the com speeds has been tested
			stopWhile = 1

		try:
			# Opening the software serial port
			pi = pigpio.pi()
			pi.set_mode(RX, pigpio.INPUT)
			pi.set_mode(TX, pigpio.OUTPUT)
			pi.bb_serial_read_open(RX, ComSpeed[ComSpeed_Pointer], 8)
			print("Testing comspeed " + ComSpeed[ComSpeed_Pointer])

		except:
			# The software serial port is already opened, closing the port and opening it again
			pi.bb_serial_read_close(RX)
			pi.stop()

			pi = pigpio.pi()
			pi.set_mode(RX, pigpio.INPUT)
			pi.set_mode(TX, pigpio.OUTPUT)
			pi.bb_serial_read_open(RX, ComSpeed[ComSpeed_Pointer], 8)
			print("Testing comspeed " + str(ComSpeed[ComSpeed_Pointer]))


		# The magic word for obtaining the ID from the flarm
		strRequestID = "$PFLAC,R,RADIOID\r\n"

		# Transmit the data with "bit banging"
		pigpio.exceptions = True
		pi.wave_clear()
		pi.wave_add_serial(TX, ComSpeed[ComSpeed_Pointer], strRequestID)
		wid=pi.wave_create()
		pi.wave_send_once(wid)   # transmit serial data
		while pi.wave_tx_busy(): # wait until all data sent
			pass
		pi.wave_delete(wid)

		# Waiting for the response
		time.sleep(2)

		# Reading the response
		data = ""
		(count, slask) = pi.bb_serial_read(RX)
		if count > 0:
			# Translating the bit array into a string
			data = slask.decode("ascii", "ignore")

		while len(data) > 0 and not data.find("\r\n") == -1:

			if data.find("\r\n") > -1:
				# Yes, there is a carrage return in the sentence

				# In case there are any characters in front of the first $, these will be removed.
				data = data[(data.find("$")):]

				# Extracting the line
				newLine = data[:(data.find("\r\n"))]

				# Extract the provided checksum and calculate the cecksum as reference
				chkSumLine, chkCalculated = subCheckSum(newLine)

				# Time to check the checksum
				if chkSumLine == chkCalculated:
					# The checksum is correct

					# Is this the line the we are looking for? PFLAC is the message containing the FLARM-identity.
					if newLine[:6] == "$PFLAC":

						# Extracting the information within the string
						returnLine = clRADIOIDMessage(newLine)

						print("Com speed detected: " + str(ComSpeed[ComSpeed_Pointer]))
						print("ID: " + returnLine.Identity)
						strBlank = returnLine.Identity
						return ComSpeed[ComSpeed_Pointer], returnLine.Identity # Returning the comspeed and the identity of the FLARM
						stopWhile = 1

				else:
					print("Checksum failed, found: " + chkSumLine + " should be: " + chkCalculated + " line:" + newLine)

			# Cut away the handled sentence from the working material
			data = data[(2+(data.find("\r\n"))):]

		ComSpeed_Pointer = ComSpeed_Pointer + 1
		pi.bb_serial_read_close(RX)
		pi.stop() 
	
	if stopWhile == 1 and len(strBlank) == 0:
		print("No communication was detected, keeping the default values")
		return 0, strBlank

def subCutString(InString):
	# Procedure to extract information from a comma delimitted string

	OutPut = ""
	OutString = ""
	pos = InString.find(",")

	if pos > -1:
		if pos == 0:
			OutString = InString[1:]
			OutPut = ""

		else:
			OutPut = InString[:pos]
			OutString = InString[pos+1:]

	# OutPut = the extracted information, OutString = is the inputstring without the extracted information
	return OutPut,OutString

def subGeoCalc(LatMe,LongMe,LatTarget,LongTarget):
	# Calculate distance and bearing between two coordinates

	Base = 0
	bearing = 0
	skipGeo = 0
	bearingN = 0

	try:
		# Check if the variables are valid, if not the function is not excecuted
		float(LatTarget)
		float(LongTarget)

	except:
		#print "Float failed subGeoCalc"
		#subWrite2Log("Float failed subGeoCalc, LatTarget: " + LatTarget + " LongTarget: " + LongTarget)
		skipGeo = 1

	if skipGeo == 0 and (len(LatTarget) > 0 or len(LongTarget) > 0):
		# Variables are valid, enter the main function
		lat1 = LatMe
		lon1 = LongMe
		lat2 = float(LatTarget)
		lon2 = float(LongTarget)

		Aaltitude = 0
		bearing = 0
		Oppsite  = 20000

		#Haversine Formuala to find vertical angle and distance
		lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

		dlon = lon2 - lon1
		dlat = lat2 - lat1
		a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
		c = 2 * atan2(sqrt(a), sqrt(1-a))
		Base = 6371 * c

		#Horisontal Bearing
		def calcBearing(lat1, lon1, lat2, lon2):
			dLon = lon2 - lon1
			y = sin(dLon) * cos(lat2)
			x = cos(lat1) * sin(lat2)  - sin(lat1) * cos(lat2) * cos(dLon)
			return atan2(y, x)

		bearingN = calcBearing(lat1, lon1, lat2, lon2)

		bearingN = degrees(bearingN)

		bearing = bearingN

		#subWrite2LogsubWrite2Log("LatMe: " + str(LatMe) + "LongMe: " + str(LongMe) + " Bearing N : " + str(bearingN) + " Bearing: " + str(bearing) + " MyTrack: " + str(MyTrack))

	# Bearing and distance to the target
	return int(round(Base*1000)),int(round(bearingN))

class clAircraftMessage(object):
    """
    ICAO    ICAO number of aircraft (3 bytes) 3C65AC
    FLAGS   Flags (see below) 1
    CALL    Callsign of aircraft N61ZP
    SQ      SQUAWK of aircraft 7232
    LAT     Latitude in degrees 57.57634
    LON     Longitude in degrees 17.59554
    ALT     Pressure altitude in feets 5000
    TRACK   Track of aircraft in degrees [0,360) 35
    VELH    Horizontal velocity of aircraft in knots 464
    VELV    Vertical velocity of aircraft in ft/min -1344
    SIGS    Signal strength in mV 840
    SIGQ    Signal quality in mV 72
    FPS     Number of raw MODE-S frames received from aircraft during last second 5
    RES     Reserved for future use
    CRC     CRC16 (described in CRC section) 2D3E
	#A:4CA948,300,,2122,52.99750,13.76526,37000,169,442,0,814,72,3,,6F1C\r\n"
    """
    def __init__(self, sentance, diffLat=0, diffLong=0):
        import time

        (self.format,
         self.ICAO,
         self.Flags,
         self.Call,
         self.Squak,
         self.Lat,
         self.Long,
         self.Alt,
         self.Track,
         self.VelH,
         self.VelV ,
         self.SigS,
         self.SigQ,
         self.FPS,
         self.RES,
         self.CRC
        ) = sentance.replace("\r\n","").replace(":",",").split(",")
        # Then adding some info and calulations:
        #self.AltitudeGPSUnit = "M"
        #self.AltitudeGPS = float(self.Alt) * 0.3048 # According to Google..
        
        self.Distance =0
        self.Bearing = 0

        if diffLat != 0 and diffLong != 0:
            #Distance,Bearing = subGeoCalc(MyLat,MyLong,Lat,Long)
            self.Distance,self.Bearing = subGeoCalc(diffLat, diffLong,self.LAT,self.LON)

def subExtractADSBInfo(Sentence,MyLat,MyLong):

	# Recieves the MAVLink sentence. Time to assemble the stirng and make some sense of it
	strADSBSplit = clAircraftMessage(Sentence)

	Distance = 0.0
	Bearing = 0.0

	#print "MyLat: " + str(MyLat) + " MyLong: " + str(MyLong) + " Lat: " + str(Lat) + " Long: " + str(Long)
	Distance,Bearing = subGeoCalc(MyLat,MyLong,strADSBSplit.Lat,strADSBSplit.Long)

	Whatever = 0
	tempCall = "" 
	deltaNorth = 0.0
	deltaEast = 0.0
	deltaAlt = 0.0
	Empty = ""
	strPFLAAMsg = ""


	try:
		# Check if the numbers are valid
		float(strADSBSplit.Alt)
		float(strADSBSplit.VelH)
		float(strADSBSplit.VelV)

	except:	
		# Negative, the numbers were not valid
		#subWrite2Log("Float failed ADSB, Alt: " + Alt + " VelH: " + VelH + " VelV: " + VelV)
		Whatever = 1

	# Calculating the altitude difference
	#deltaAlt = ((float(Alt) / 3.2808399) - float(MyAlt))

	#print "ADSB ICAO: " + ICAO + " Flags: " + Flags + "Call: " + Call + " tempCall: " + tempCall + " Squak: " + Squak + " Lat: " + Lat + " Long: " + Long + " Alt: " + Alt + " Track: " + Track + " VelH: " + VelH + " VelV: " + VelV + " SigS: " + SigS + " SigQ: " + SigQ + "FPS: " + FPS

	#if Distance == 0 or Bearing == 0 or (Distance/ZoomFactor) > MaxRange or Call == MyID or str(VelH) == Empty or str(VelV) == Empty or deltaAlt > MaxDeltaAlt or deltaAlt < MaxDeltaAlt:
	if Distance == 0 or Bearing == 0 or (Distance/ZoomFactor) > MaxRange or strADSBSplit.ICAO == MyID or str(strADSBSplit.VelH) == Empty or str(strADSBSplit.VelV) == Empty:
		# Distance and bearing is zero, values are not valid OR the plane is too far away OR its my own signal OR VelV and VelH are empty: skip further handling
		Whatever = 1

		if (cos(radians(Bearing))*(Distance)/ZoomFactor) > MaxRange or (sin(radians(Bearing))*(Distance)/ZoomFactor) > MaxRange:
			print("Far far away ..." + str(Distance/ZoomFactor) + " : " + str(MaxRange) + " SigS: " + str(strADSBSplit.SigS))
			Whatever = Whatever

	else:
		# Numbers are valid, lets compile the string

		# Calculating the horisontal distance to the target
		deltaNorth = (cos(radians(Bearing))*(Distance)/ZoomFactor)

		# Calculating the horisontal distance to the target
		deltaEast = (sin(radians(Bearing))*(Distance)//ZoomFactor)

		# Calculating the altitude difference
		deltaAlt = ((float(strADSBSplit.Alt) / 3.2808399) - MyAlt)

		# Calculating timestamps in milliseconds
		intHrs = int(MyTime[:-4])
		intMin = int(MyTime[2:-2])
		intSec = int(MyTime[4:])
		TimeStamp = intHrs * 60 * 60 * 1000 + intMin * 60 * 1000 + intSec * 1000
		#print str(TimeStamp)

		#print "dN: " + str(int(deltaNorth)) + " dE: " + str(int(deltaEast)) +  " dA: " + str(deltaAlt) + " Call: " + Call + " Squak: " + Squak
		#subWrite2Log("Diff Call: " + Call + " n: " + str(int(deltaNorth)) + " e: " + str(int(deltaEast)) +  " a: " + str(deltaAlt))

		if strADSBSplit.Call != "":
			# The callsign to the butterfly display shall be in hexa decimal. 
			# Converting to hexa decimal and keep the last 6 characters

			#tempCall = Call.encode("hex")		# Python 2

			tempCall = binascii.b2a_hex(bytes(strADSBSplit.Call, 'utf-8'))	# Python 3

			tempCall = tempCall[-6:]
		else:
			# The callsign are not present, give it a dummy identity
			tempCall = "ABAAAA"
		#print "VelH: " + str(VelH) + " VelV: " + str(VelV)
		strVelH = str(round(int(float(strADSBSplit.VelH)*0.00508)))
		strVelV = str(round(int(float(strADSBSplit.VelV)/60)))

		# PFLAA-message
		strPFLAAMsg =  "$PFLAA,0," + str(int(deltaNorth)) + "," + str(int(deltaEast)) + "," + str(int(deltaAlt)) + ",1," + strADSBSplit.ICAO + "," + strADSBSplit.Track + ",0.0," + "20" + "," + strVelH + "," + strVelV + ",8"
		#listPlanes.append("PFLAA,0," + str(int(deltaNorth)) + "," + str(int(deltaEast)) + "," + str(int(deltaAlt)) + ",1," + tempCall + "," + Track + ",0.0," + "20" + "," + str(round(float(VelV)*0.00508)) + ",8")
	
	return strPFLAAMsg,strADSBSplit.ICAO

def subSendSentence(sentence):
	# Recives a sentence, adds its checsum and transmits it on the serial port

	chksum = 0
	calc_cksum = 0

	#sentence = sentence[1:]

	# Calculating checksum
	for s in sentence[1:]:
		calc_cksum ^= ord(s)
	chksum = hex(calc_cksum).upper()

	# Removing the  "0X" from the hex value
	chksum = chksum[2:]

	# If the checksum is a single digit, adding a zero in front of the digit
	if len(chksum) == 1:
		chksum = "0" + chksum

	# Transmit the data with "bit banging"
	pigpio.exceptions = True
	pi.wave_clear()
	pi.wave_add_serial(intComPinBUTTERFLY, intComSpeedBUTTERFLY, sentence + "*" + chksum + "\r\n")
	wid=pi.wave_create()
	pi.wave_send_once(wid)   # transmit serial data
	while pi.wave_tx_busy(): # wait until all data sent
		pass
	pi.wave_delete(wid)
	print(sentence + "*" + chksum)

def subCheckSum(sentence):

	# Saving the incoming checksum for reference
	strOriginal = sentence[-2:]

	# Remove checksum
	sentence = sentence[:-3]

	# Remove $
	sentence = sentence[1:]

	chksum = ""
	calc_cksum = 0

	# Calculating checksum

	for s in sentence:
		#print "s: " + str(s)
		calc_cksum ^= ord(s) 	#calc_cksum ^= ord(s)
		strCalculated = hex(calc_cksum).upper()

	# Removing the  "0X" from the hex value
	strCalculated = strCalculated[2:]

	# If the checksum is a single digit, adding a zero in front of the digit
	if len(strCalculated) == 1:
		strCalculated = "0" + strCalculated

	return strOriginal, strCalculated
	# Returning the provided checksum (from the original message) and the calculated 

def subGetNMEA(tmpNMEA):
	# This sub is reading a temporary text file and extracts the information

	# Check if the file is complete
	newData = ""
	slask = ""
	data = "xx"

	# # No Simulation, get the real deal
	(count, slask) = pi.bb_serial_read(intComPinFLARM_RX)
	if count > 0:
		data = slask.decode("ascii", "ignore")

	# Simulating position Sodertalje
	# print("Simulating position: Sodertalje")
	# data = "IJKLMNOP40\r\n$PFLAU,0,1,1,1,0,,0,,,*4F\r\n$GPRMC,141328.00,A,5911.22440,N,01739.41460,E,0.031,342.13,050219,,,A*6E\r\n$PGRMZ,153,F,2*3D\r\n$GPGGA,141328.00,5911.2244,N,01739.4146,E,1,12,2.72,36.6,M,24.1,M,,*66\r\n$ABCDEFGH"

	# Simulating position Billingehus
	#print "Simulating position: Billingehus"
	#data = "$PFLAU,0,1,1,1,0,,0,,,*4F\r\n$GPRMC,141328.00,A,5824.16090,N,01349.27640,E,0.031,342.13,050219,,,A*6E\r\n$PGRMZ,153,F,2*3D\r\n$GPGGA,141328.00,5824.16090,N,01349.27640,E,1,12,2.72,36.6,M,24.1,M,,*66\r\n"

	# If a string was recieved, lets add it to the leftover string from the last sentence
	if len(data) > 0:
			#newData = tmpNMEA + data + "$PFLAA,0,21,11,11,2,DDE605,0,,0,-0.1,1*32\r\n"
			newData = tmpNMEA + data

			tmpNMEA = ""

	while len(newData) > 0 and not newData.find("\r\n") == -1:

		# Entering this while, the variable newData is a long string with several embedded 
		# carrage returns. This part of the program is dividing the string into a list of 
		# NMEA senteces. The tricky part is that the information from the serial port might
		# be read during a trasmission. This means that the last part of the NMEA-sentence 
		# might be missing. Therefore if the last sentence of the string is not ended with 
		# a carrage return, that part will be saved in a variable (tmpNMEA) and will be 
		# added in the beggning of the next.
		# print(newData.find("\r\n"))

		if newData.find("\r\n") > -1:
		# Yes, there is a carrage return in the sentence

			# In case there are any characters in front of the first $, these will be removed.
			newData = newData[(newData.find("$")):]

			newLine = newData[:(newData.find("\r\n"))]

			# Extract the provided checksum and calculate the cecksum as reference
			chkSumLine, chkCalculated = subCheckSum(newLine)

			# Time to check the checksum
			if chkSumLine == chkCalculated:
				# Bang on! Lets add the line to the list
				listNMEA.append(newLine)
			else:
				print("Checksum failed, found: " + chkSumLine + " should be: " + chkCalculated + " line:" + newLine)

			# Cut away the appended sentence from the working material
			newData = newData[(2+(newData.find("\r\n"))):]

	if newData.find("\r\n") == -1:
	# If there are no more carrage returns in the string, it has to be a left over. Lets save it for the next time we check for new sentenses

		tmpNMEA = newData	#This is the 'leftover string'

	return tmpNMEA

class clNMEAMessage(object):
    """
	Time
	RecieverWarning
	Lat
	East
	Long
	North
	VelH
	Track
	Date
    """
    #def __init__(self, sentance, diffLat=0, diffLong=0):
    def __init__(self, sentance):
        import time

		# $GPRMC,141328.00,A,5911.22440,N,01739.41460,E,0.031,342.13,050219,,,A*6E
		# GPRMC: $GPRMC
		# Time: 141328.00
		# RecieverWarning: A
		# Lat: 5911.22440
		# N_or_S: N
		# Long: 01739.41460
		# E_or_W: E
		# VelH: 0.031
		# Track: 342.13
		# Time: 050219,
		# Magnetic: 
		# Magnetic_E_or_W
		# CRC: A*6E

        (self.GPRMC,
		 self.Time ,
		 self.RecieverWarning ,
		 self.Lat ,
		 self.N_or_S ,
		 self.Long ,
		 self.E_or_W ,
		 self.VelH ,
		 self.Track ,
		 self.Date ,
		 self.Magnetic ,
		 self.Magnetic_E_or_W ,
		 self.CRC
        ) = sentance.replace("\r\n","").replace(":",",").split(",")

        # # Then adding some info and calulations:
        # self.AltitudeGPSUnit = "M"
        # self.AltitudeGPS = float(self.ALTfeet) * 0.3048 # According to Google..
        
        # self.Distance =0
        # self.Bearing = 0

        # if diffLat != 0 and diffLong != 0:
        #     #Distance,Bearing = subGeoCalc(MyLat,MyLong,Lat,Long)
        #     self.Distance,self.Bearing = subGeoCalc(diffLat, diffLong,self.LAT,self.LON)

def subExtractNMEAInfo(Sentence):
	Lat = 0
	Long = 0
	Time = ""

	# Recieves the NMEA sentence. Time to assemble the stirng and make some sense of it

	skipGeo = 0

	strNMEASplit = clNMEAMessage(Sentence)
	#print(strNMEASplit.Date)

	Sentence=Sentence[:-6]

	#subWrite2Log("NMEA Time: " + strNMEASplit.Time + " , RW: " + strNMEASplit.RecieverWarning + " , lat: " + strNMEASplit.Lat +  " , long: " + strNMEASplit.Long + " , VelH: " + strNMEASplit.VelH + " , track: " + strNMEASplit.Track + " , date: " + strNMEASplit.Date)

	try:
		# Check if the variables are valid, if not the function is not excecuted
		x = float(strNMEASplit.Lat)
		x = float(strNMEASplit.Long)


		if (len(strNMEASplit.Lat) > 0 or len(strNMEASplit.Long) > 0):

			# Lat = DDMM.mmmmm shall be recalculated into DD.ddddddd
			if strNMEASplit.N_or_S == "S":				# Is this cricket? Scruteny please!
				LatDegrees = strNMEASplit.Lat[:2] * -1
			else:
				LatDegrees = strNMEASplit.Lat[:2]

			LatMinutes = strNMEASplit.Lat[-8:]
			Lat = float(LatDegrees) + (float(LatMinutes)/60)

			# Long = DDDMM.mmmmm shall be recalculated into DD.ddddddd
			if strNMEASplit.E_or_W == "W":				# Is this cricket? Scruteny please!
				LongDegrees = strNMEASplit.Long[:3]
			else:
				LongDegrees = strNMEASplit.Long[:3]

			LongMinutes = strNMEASplit.Long[-8:]
			Long = float(LongDegrees) + (float(LongMinutes)/60)

			MySpeed = strNMEASplit.VelH
			#MyTrack = Track

			Time = strNMEASplit.Time[:-3]
			Date = strNMEASplit.Date
			#print str(MyTime)
			#print str(MyTime)[:-4]		# Hours
			#print str(MyTime)[2:-2]	# Minutes
			#print str(MyTime)[4:]		# Seconds

			# This function is intended to adjust the time of the RasPi
			#if int(time.strftime("%H%M%S")) - int(MyTime) > 5 or int(MyTime)- int(time.strftime("%H%M%S")) > 5:
				#print "timeDiff: " + str(int(time.strftime("%H%M%S"))- int(MyTime))
				#PERIOD = yesterday.strftime ('%Y-%m-%d')
				#new_period = PERIOD.replace(hour = int(MyTime[:-7]), minute = int(MyTime[2:-5]), second = int(MyTime[4:-3]))

				# ..... to be continued .....

			#subWrite2Log("New own data, lat: " + str(MyLat) +  " , long: " + str(MyLong))
			#print "New own data, lat: " + str(MyLat) +  " , long: " + str(MyLong)

	except:
		jantar = 0

	return 	Lat,Long,Time


def subStoreFile(infile,outfile):

	#copyfile(pfile, "/log/adsb_" & date & time & ".log")
	source = open(infile, 'r')
	source.readlines

	destination = open(outfile, 'a')

	for line in source:
		destination.write(line)

	destination.close
	source.close
class clPFLAAMessage(object):
	"""
	Time
	RecieverWarning
	Lat
	East
	Long
	North
	VelH
	Track
	Date
	"""
	#def
	def __init__(self, sentance):
		import time

		strBlank = ""

		# $PFLAA,0,21,11,11,2,DDE605,0,,0,-0.1,1*32
		# PFLAA: $PFLAA
		# AlarmLevel: 0
		# RelativeNorth: 21
		# RelativeEast: 11
		# RelativeVertical: 11
		# IDType: 2
		# ID: DDE605
		# Track: 0
		# TurnRate: 
		# GroundSpeed: 0
		# ClimbRate: -0.1
		# Type: 1
		# CRC: *32


		(self.PFLAA, 
		self.AlarmLevel, 
		self.RelativeNorth, 
		self.RelativeEast, 
		self.RelativeVertical, 
		self.IDType, 
		self.ID, 
		self.Track, 
		self.TurnRate, 
		self.GroundSpeed, 
		self.ClimbRate, 
		self.Type, 
		self.CRC
		) = sentance.replace("\r\n","").replace("*",",").split(",")

def subGetPFLAU(Sentence):
	# "$PFLAU,1,1,1,1,0,,0,,,*4E"

	Sentence,SentenceX = subCutString(Sentence)
	SentenceX,Sentence = subCutString(SentenceX)
	#print Sentence
	return  Sentence

def subWrite2Log(info):
	#GPIO.setwarnings(False)
	#GPIO.setmode(GPIO.BCM)
	#GPIO.setup(12, GPIO.IN)
	#GPIO.setup(13, GPIO.IN)
	#GPIO.setup(16, GPIO.IN)
	the_log = open(logFile, 'a')
	the_log.write(info)
	#print info
	the_log.close
	X = 0

while go == 1:

	# Main function
	if init == 1:
		# Things to do before anything else
		init = 0

		# Setting up GPIO
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(12, GPIO.IN)
		GPIO.setup(13, GPIO.IN)
		GPIO.setup(16, GPIO.IN)
		GPIO.setup(19, GPIO.IN)

		intSpeed, MyID = subGetFLARM_ID(intComPinFLARM_TX,intComPinFLARM_RX)

		if len(MyID) > 3:
			# Communication with the FLARM has been established, correcting the comspeed
			intComSpeedFLARM = intSpeed
		else:
			# No identity was detected, getting the identity of the FLARM from the identity text file
			f = open(myIDFile, 'r')
			f.readlines
			for line in f:
				if line[:1]!="#" and len(line) > 0:
					MyID = line[:6]
					subWrite2Log("My ID: " + MyID)
					print("My ID: " + MyID)
			f.close

		# Opening the software managed serial port
		try:
			pi = pigpio.pi()
			pi.set_mode(intComPinFLARM_RX, pigpio.INPUT)
			pi.set_mode(intComPinBUTTERFLY, pigpio.OUTPUT)
			pi.bb_serial_read_open(intComPinFLARM_RX, intComSpeedFLARM, 8)

		except:
			# The software serial port is already opened, closing the port and opening it again
			pi.bb_serial_read_close(intComPinFLARM_RX)
			pi.stop()

			pi = pigpio.pi()
			pi.set_mode(intComPinFLARM_RX, pigpio.INPUT)
			pi.set_mode(intComPinBUTTERFLY, pigpio.OUTPUT)
			pi.bb_serial_read_open(intComPinFLARM_RX, intComSpeedFLARM, 8)

		strNMEALeftOvers = ""

	# Continous execution from here

	# Reading serial data from the ADS-B reciever. Format = MAVLink
	#strADSB=serADSB.readline()
	strADSB=serADSB.readline().decode('utf-8')

	print("strADSB: " + strADSB)

	# An A-sentece has been recieved, time to handle it
	#if strADSB.startswith("#A"):
	#if strADSB.startswith(b"#A"):
	if str(strADSB).startswith("#A"):

		# Exxract information from the MAV-link protocol
		# strPart = PFLAA-message
		# strPart2 = PAAVD-message
		# Id = the ICAO-identity
		strPart,Id = subExtractADSBInfo(strADSB,MyLat,MyLong)

		if len(strPart) > 5:
			# If the string is longer than 5 chars it is considered valid
			# Start counting the planes in the list
			count = count + 1

			# Add the sentence to the list of ADSB sentences, it will transmitted later
			listADSBSentences.append(strPart)

			# Add the identity to a list for later use
			listADSBID.append(Id)

	# A "S"-sentence has arrived. It is sent once every 1 second and this is used to trigger some functions:  time to send information to the butterfly display
	#if strADSB.startswith("#S"):
	if str(strADSB).startswith("#S"):

		# Clear screen
		#print "----------o0o----------" # ("\033c")

		#tmpTst, tmpTst1, tmpTst2 = subExtractADSBInfo("A:8962E5,1C00,ETD140,,59.53326,18.30945,35000,105,528,-64,676,67,4,,546B\r\n")
		#listADSBSentences.append(tmpTst)
		#count = count + 1

		# Reset the counter
		NoOfFlarmContacts = 0

		#Read the lastest NMEA-information
		strNMEALeftOvers = subGetNMEA(strNMEALeftOvers)

		# Extract information from the NMEA-sentence
		for NMEAline in listNMEA:

			if NMEAline[:6] == "$PFLAA":
				# The FLARM is detecting another tranciever in "da hood"
				NoOfFlarmContacts = NoOfFlarmContacts + 1

				# Extracting the identity if the other tranciever from the message
				strPFLAASplit = clPFLAAMessage(NMEAline)

				# Add the identity to a list, to be used in the future
				listFLARMID.append(strPFLAASplit.ID)

				# Sending the message to the COM-port
				subSendSentence(NMEAline)

			elif NMEAline[:6] == "$GPRMC":
				# GPRMC contains data such as time, long/lat, speed etc. This information needs to be extracted.
				# Extracting the infromation from the GPRMC-message
				MyLat,MyLong,MyTime = subExtractNMEAInfo(NMEAline)

				# Sending the message to the COM-port
				subSendSentence(NMEAline)

			elif NMEAline[:6] == "$PFLAU":
				# PFLAU contains FLARM-alarms of the most dangerous contact
				strPFLAU = subGetPFLAU(NMEAline) 			# Non-testing
				#strPFLAU = subGetPFLAU("$PFLAU,1,1,1,1,0,,0,10,10,*4E") # Testing

				#Please note that the PFLAU will be sent a bit later

			else:
				# All of the other messages
				subSendSentence(NMEAline)
			
		NoOfADSBContacts = count + 1 	#Addint one since -1 can be considered as 0

		while count > -1:
			# Time to empty the list of ADSB sentences

			SkipSentence = 0

			for Id in listFLARMID:
				# Check if there are any planes with FLARM and ADS-B, if so skip sending that ADS-B sentence
				if Id == listADSBID:
					SkipSentence = 1

			if SkipSentence == 0:
				subSendSentence(listADSBSentences[count])
				#subWrite2Log(listADSBSentences[count] + "\r\n")
				#print listPlanes[count] + "\r\n"
			count = count - 1

		# Compiling the $PAAVS,AT,ABS - Absolute Output Status Information
		# $PAAVS,AT,ABS,<TimeOfDumpCreation>,<NumFlarmTargets>,<NumADSBTargets>,<Latitude>,<Longitude>,<AltitudeWGS84>,<IDType>,<ID>
		intHrs = int(MyTime[:-4])
		intMin = int(MyTime[2:-2])
		intSec = int(MyTime[4:])
		TimeStamp = intHrs * 60 * 60 * 1000 + intMin * 60 * 1000 + intSec * 1000
		#subSendSentence("$PAAVS,AT,ABS," + str(TimeStamp) + "," + str(NoOfFlarmContacts) + "," + str(NoOfFlarmContacts) + "," + str(MyLat) + "," + str(MyLong) + "," + str(MyAlt) + ",1," + MyID)

		# Compiling the $PFLAU- Operating Status, Priority IntruderAnd ObstacleWarnings
		# $PFLAU,<RX>,<TX>,<GPS>,<Power>,<AlarmLevel>,<RelativeBearing>,<AlarmType>,<RelativeVertical>,<RelativeDistance>,<ID>

		subSendSentence("$PFLAU," + str(NoOfADSBContacts + NoOfFlarmContacts) + "," + strPFLAU)

		# Resetting lists, temp values and counters
		del listADSBSentences[:]
		del listNMEA[:]
		del listFLARMID[:]
		count = -1
		# Resetting the list of identeties heard the last scan
		listADSBID.clear()

# Closing the software serial port when exiting
pi.bb_serial_read_close(intComPinFLARM_RX)
pi.stop() 
