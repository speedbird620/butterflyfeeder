#!/usr/bin/env python3
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
from math import *
from shutil import copyfile

# The butterfly and the ADS-B reciver has different com speed
intComSpeedADSB = 115200

intComSpeedFLARM = 38400 # 19200
intComPinFLARM = 18

intComSpeedBUTTERFLY = 38400 # 57600
intComPinBUTTERFLY = 17

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
MyAlt = 0               # This units altitude
MySpeed = 0             # This units (ground) speed
MyTrack = 0             # This units track
MyTime = "000000"	# This units time
MyDate = ""		# This units date
MyID = ""		# This units identity

fileName = "/mnt/ram/data.txt"
logFile = "log.txt"
myIDFile = "/home/pi//butterflyfeeder/config/myID.txt"

ZoomFactor = 1
MaxRange = 6000 # Meters
MaxDeltaAlt = 2000

tmpNMEA = ""

strPFLAUMessage = ""
strPFLAUID = ""
intPFLAUDist = 0

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

		subWrite2Log("LatMe: " + str(LatMe) + "LongMe: " + str(LongMe) + " Bearing N : " + str(bearingN) + " Bearing: " + str(bearing) + " MyTrack: " + str(MyTrack))

	# Bearing and distance to the target
	return int(round(Base*1000)),int(round(bearingN))

def subExtractADSBInfo(Sentence):
	global MyLat
	global MyLong
	global strPFLAUMessage
	global strPFLAUID
	global intPFLAUDist

	# Recieves the MAVLink sentence. Time to assemble the stirng and make some sense of it

	# Remove the last 6 chars (checksum + carrage return)
	Sentence=Sentence[:-6]

	# Remove the three first chars ("#A:")
	Sentence=Sentence[3:]

	ICAO,Sentence = subCutString(Sentence) 	# Extracting the ICAO singal
	Flags,Sentence = subCutString(Sentence)	# Etc ...
	Call,Sentence = subCutString(Sentence)
	Squak,Sentence = subCutString(Sentence)
	Lat,Sentence = subCutString(Sentence)
	Long,Sentence = subCutString(Sentence)
	Alt,Sentence = subCutString(Sentence)
	Track,Sentence = subCutString(Sentence)
	VelH,Sentence = subCutString(Sentence)
	VelV,Sentence = subCutString(Sentence)
	SigS,Sentence = subCutString(Sentence)
	SigQ,Sentence = subCutString(Sentence)
	FPS,Sentence = subCutString(Sentence)

	#print "ICAO: " + ICAO
	#print "Flags: " + Flags
	#print "Call: " + Call
	#print "SQUAK: " + Squak
	#print "Lat: " + Lat
	#print "Long: " + Long
	#print "Alt: " + Alt
	#print "Track: " + Track
	#print "VelH: " + VelH
	#print "VelV: " + VelV
	#print "SigS: " + SigS
	#print "SigQ: " + SigQ
	#print "FPS: " + FPS

	#MyLat = 59.18712		# My position
	#MyLong = 17.65699	# My position
	#MyAlt = 0		# My position
	Distance = 0.0
	Bearing = 0.0

	#print "MyLat: " + str(MyLat) + " MyLong: " + str(MyLong) + " Lat: " + str(Lat) + " Long: " + str(Long)
	Distance,Bearing = subGeoCalc(MyLat,MyLong,Lat,Long)

	Whatever = 0
	tempCall = "" 
	deltaNorth = 0.0
	deltaEast = 0.0
	deltaAlt = 0.0
	Empty = ""

	try:
		# Check if the numbers are valid
		float(Alt)
		float(VelH)
		float(VelV)

	except:	
		# Negative, the numbers were not valid
		#subWrite2Log("Float failed ADSB, Alt: " + Alt + " VelH: " + VelH + " VelV: " + VelV)
		Whatever = 1

	# Calculating the altitude difference
	#deltaAlt = ((float(Alt) / 3.2808399) - float(MyAlt))

	#print "ADSB ICAO: " + ICAO + " Flags: " + Flags + "Call: " + Call + " tempCall: " + tempCall + " Squak: " + Squak + " Lat: " + Lat + " Long: " + Long + " Alt: " + Alt + " Track: " + Track + " VelH: " + VelH + " VelV: " + VelV + " SigS: " + SigS + " SigQ: " + SigQ + "FPS: " + FPS

	#if Distance == 0 or Bearing == 0 or (Distance/ZoomFactor) > MaxRange or Call == MyID or str(VelH) == Empty or str(VelV) == Empty or deltaAlt > MaxDeltaAlt or deltaAlt < MaxDeltaAlt:
	if Distance == 0 or Bearing == 0 or (Distance/ZoomFactor) > MaxRange or ICAO == MyID or str(VelH) == Empty or str(VelV) == Empty:
		# Distance and bearing is zero, values are not valid OR the plane is too far away OR its my own signal OR VelV and VelH are empty: skip further handling
		Whatever = 1

		if (cos(radians(Bearing))*(Distance)/ZoomFactor) > MaxRange or (sin(radians(Bearing))*(Distance)/ZoomFactor) > MaxRange:
			#print "Far far away ..." + str(Distance/ZoomFactor) + " : " + str(MaxRange) + " SigS: " + str(SigS)
			Whatever = Whatever
		return " ", " ", " "

	else:
		# Numbers are valid, lets compile the string

		# Calculating the horisontal distance to the target
		deltaNorth = (cos(radians(Bearing))*(Distance)/ZoomFactor)

		# Calculating the horisontal distance to the target
		deltaEast = (sin(radians(Bearing))*(Distance)//ZoomFactor)

		# Calculating the altitude difference
		deltaAlt = ((float(Alt) / 3.2808399) - MyAlt)

		# Calculating timestamps in milliseconds
		intHrs = int(MyTime[:-4])
		intMin = int(MyTime[2:-2])
		intSec = int(MyTime[4:])
		TimeStamp = intHrs * 60 * 60 * 1000 + intMin * 60 * 1000 + intSec * 1000
		#print str(TimeStamp)

		#print "dN: " + str(int(deltaNorth)) + " dE: " + str(int(deltaEast)) +  " dA: " + str(deltaAlt) + " Call: " + Call + " Squak: " + Squak
		subWrite2Log("Diff Call: " + Call + " n: " + str(int(deltaNorth)) + " e: " + str(int(deltaEast)) +  " a: " + str(deltaAlt))

		if Call != "":
			# The callsign to the butterfly display shall be in hexa decimal. 
			# Converting to hexa decimal and keep the last 6 characters
			tempCall = Call.encode("hex")
			tempCall = tempCall[-6:]
		else:
			# The callsign are not present, give it a dummy identity
			tempCall = "ABAAAA"
		#print "VelH: " + str(VelH) + " VelV: " + str(VelV)
		strVelH = str(round(int(float(VelH)*0.00508)))
		strVelV = str(round(int(float(VelV)/60)))

		#print "ADSB ICAO: " + ICAO + " Flags: " + Flags + "Call: " + Call + " tempCall: " + tempCall + " Squak: " + Squak + " Lat: " + Lat + " Long: " + Long + " Alt: " + Alt + " Track: " + Track + " VelH: " + VelH + " VelV: " + VelV + " SigS: " + SigS + " SigQ: " + SigQ + "FPS: " + FPS
		subWrite2Log("ADSB ICAO: " + ICAO + " Flags: " + Flags + "Call: " + Call + " tempCall: " + tempCall + " Squak: " + Squak + " Lat: " + Lat + " Long: " + Long + " Alt: " + Alt + " Track: " + Track + " VelH: " + VelH + " VelV: " + VelV + " SigS: " + SigS + " SigQ: " + SigQ + "FPS: " + FPS)


		# PFLAA-message
		part1 =  "$PFLAA,0," + str(int(deltaNorth)) + "," + str(int(deltaEast)) + "," + str(int(deltaAlt)) + ",1," + ICAO + "," + Track + ",0.0," + "20" + "," + strVelH + "," + strVelV + ",8"
		#part1 = " "
		#listPlanes.append("PFLAA,0," + str(int(deltaNorth)) + "," + str(int(deltaEast)) + "," + str(int(deltaAlt)) + ",1," + tempCall + "," + Track + ",0.0," + "20" + "," + str(round(float(VelV)*0.00508)) + ",8")

		# PAAVD,AT,A - Absolute ADS-B Target Data: str(Track)
		#$PAAVD,AT,A,<ModeSAddress>,<FlightID>,<EmitterCategory>,<ModeASquawkCode>,<Latitude>,<Longitude>,<AltitudeWGS84>,<AltitudeQNE>,<Track>,<GroundSpeed>,<ClimbRateWGS84>,<ClimbRateBaro>,<SignalStrength>, <PositionTimestamp>,<VelocityTimestamp>,<ModeSTimestamp>,<ADSBVersionNumber>,<QualityIndicator>
		#part2 = "$PAAVD,AT,A," + ICAO + "," +   Call + "," + "20" + "," + "7000" + "," + str(Lat) + "," + str(Long) + "," + str(Alt) + "," + str(Alt) + "," + "359.3" + "," + str(VelH) + "," + str(round(float(VelV)*0.00508)) + "," + str(round(float(VelV)*0.00508)) + "," + str(SigS) + "," + str(TimeStamp) + "," + str(TimeStamp) + "," + str(TimeStamp) + "," + "2"  + "," + "7A7C" #str(SigQ)
		part2 = " "
		#return part1,part2

		# Data of the closest ADS-B transmitter
		if intPFLAUDist == 0 or Distance < intPFLAUDist:
			strPFLAUID = ICAO
			intPFLAUDist = Distance
			strPFLAUMessage = "1,1,1,0," + str(Bearing) + ",0," + str(int(round(deltaAlt))) + "," + str(int(round(Distance)*1000))

		return part1,part2,ICAO

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

def subGetNMEA():
	# This sub is reading a temporary text file and extracts the information

	# Check if the file is complete
	newData = ""
	slask = ""
	data = "xx"
	global tmpNMEA

	# # No Simulation, get the real deal
	# (count, slask) = pi.bb_serial_read(intComPinFLARM)
	# # if count > 0:
	# #     data = slask.decode()
	
	# try:
	#     data = slask.decode(encoding="utf-8", errors="strict")
	#     data = slask.decode('ascii')
	# except
	#     data = ''
	
		

	# Simulating position Sodertalje
	print("Simulating position: Sodertalje")
	data = "IJKLMNOP40\r\n$PFLAU,0,1,1,1,0,,0,,,*4F\r\n$GPRMC,141328.00,A,5911.22440,N,01739.41460,E,0.031,342.13,050219,,,A*6E\r\n$PGRMZ,153,F,2*3D\r\n$GPGGA,141328.00,5911.2244,N,01739.4146,E,1,12,2.72,36.6,M,24.1,M,,*66\r\n$ABCDEFGH"

	# Simulating position Billingehus
	#print "Simulating position: Billingehus"
	#data = "$PFLAU,0,1,1,1,0,,0,,,*4F\r\n$GPRMC,141328.00,A,5824.16090,N,01349.27640,E,0.031,342.13,050219,,,A*6E\r\n$PGRMZ,153,F,2*3D\r\n$GPGGA,141328.00,5824.16090,N,01349.27640,E,1,12,2.72,36.6,M,24.1,M,,*66\r\n"

	# If a string was recieved, lets add it to the leftover string from the last sentence
	if len(data) > 0:
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
		print(newData.find("\r\n"))

		if newData.find("\r\n") > -1:
		# Yes, there is a carrage return in the sentence

			# In case there are any characters in front of the first $, these will be removed.
			newData = newData[(newData.find("$")):]

			newLine = newData[:(newData.find("\r\n"))]

			# Extract the provided checksum and calculate the cecksum as reference
			chkSumLine, chkCalculated = subCheckSum(newLine)

			# Time to check the checksum
			if chkSumLine == chkCalculated:
				# Bang on! Lets att the line to the list
				listNMEA.append(newLine)
			else:
				print("Checksum failed, found: " + chkSumLine + " should be: " + chkCalculated + " line:" + newLine)

			# Cut away the appended sentence from the working material
			newData = newData[(2+(newData.find("\r\n"))):]

	if newData.find("\r\n") == -1:
	# If there are no more carrage returns in the string, it has to be a left over. Lets save it for the next time we check for new sentenses

		tmpNMEA = newData	#This is the 'leftover string'

def subExtractNMEAInfo(Sentence):
	global MyLat
	global MyLong
	global MyTime

	# Recieves the NMEA sentence. Time to assemble the stirng and make some sense of it

	skipGeo = 0

	# Remove the last 6 chars (checksum + carrage return)
	Sentence=Sentence[:-6]

	whatEver,Sentence = subCutString(Sentence)             # Dummy only
	Time,Sentence = subCutString(Sentence)                # Extracting the time
	RecieverWarning,Sentence = subCutString(Sentence)      # Etc ...
	Lat,Sentence = subCutString(Sentence)
	East,Sentence = subCutString(Sentence)
	Long,Sentence = subCutString(Sentence)
	North,Sentence = subCutString(Sentence)
	VelH,Sentence = subCutString(Sentence)
	Track,Sentence = subCutString(Sentence)
	Date,Sentence = subCutString(Sentence)

	#subWrite2Log("NMEA Time: " + Time + " , RW: " + RecieverWarning + " , lat: " + Lat +  " , long: " + Long + " , VelH: " + VelH + " , track: " + Track + " , date: " + Date)

	try:
			# Check if the variables are valid, if not the function is not excecuted
			float(Lat)
			float(Long)

	except:
	#subWrite2Log("Float failed subExtractNMEAInfo, Lat: " + Lat + " Long: " + Long)
			skipGeo = 1

	if skipGeo == 0 and (len(Lat) > 0 or len(Long) > 0):
		# Lat = DDMM.mmmmm shall be recalculated into DD.ddddddd
		LatDegrees = Lat[:2]
		LatMinutes = Lat[-8:]
		MyLat = float(LatDegrees) + (float(LatMinutes)/60)

		# Long = DDDMM.mmmmm shall be recalculated into DD.ddddddd
		LongDegrees = Long[:3]
		LongMinutes = Long[-8:]
		MyLong = float(LongDegrees) + (float(LongMinutes)/60)

		MySpeed = VelH
		#MyTrack = Track

		MyTime = Time[:-3]
		MyDate = Date
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

		subWrite2Log("New own data, lat: " + str(MyLat) +  " , long: " + str(MyLong))
		#print "New own data, lat: " + str(MyLat) +  " , long: " + str(MyLong)

def subStoreFile(infile,outfile):

	#copyfile(pfile, "/log/adsb_" & date & time & ".log")
	source = open(infile, 'r')
	source.readlines

	destination = open(outfile, 'a')

	for line in source:
		destination.write(line)

	destination.close
	source.close


def subGetFLARMID(Sentence):
	# $PFLAA,0,21,11,11,2,DDE605,0,,0,-0.1,1*32

	Sentence,SentenceX = subCutString(Sentence)
	SentenceX,Sentence = subCutString(SentenceX)
	Sentence,SentenceX = subCutString(Sentence)
	SentenceX,Sentence = subCutString(SentenceX)
	Sentence,SentenceX = subCutString(Sentence)
	SentenceX,Sentence = subCutString(SentenceX)
	#print Sentence[:6]
	return Sentence[:6]

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
	if not GPIO.input(12) or not GPIO.input(13) or not GPIO.input(16) or not GPIO.input(19):
		the_log = open(logFile, 'a')
		the_log.write(info + "\r\n")
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

		# Getting the identity of the FLARM from the identity text file
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
			pi.set_mode(intComPinFLARM, pigpio.INPUT)
			pi.set_mode(intComPinBUTTERFLY, pigpio.OUTPUT)
			pi.bb_serial_read_open(intComPinFLARM, intComSpeedFLARM, 8)

		except:
			# The software serial port is already opened, closing the port and opening it again
			pi.bb_serial_read_close(intComPinFLARM)
			pi.stop()

			pi = pigpio.pi()
			pi.set_mode(intComPinFLARM, pigpio.INPUT)
			pi.set_mode(intComPinBUTTERFLY, pigpio.OUTPUT)
			pi.bb_serial_read_open(intComPinFLARM, intComSpeedFLARM, 8)

		tmpNMEA = ""

	# Continous execution from here

	# Reading serial data from the ADS-B reciever. Format = MAVLink
	#strADSB=serADSB.readline()
	strADSB=serADSB.readline().decode('utf-8')

	print("strADSB: " + strADSB)

	# An A-sentece has been recieved, time to handle it
	#if strADSB.startswith("#A"):
	#if strADSB.startswith(b"#A"):
	if str(strADSB).startswith("#A"):

		# Exract information from the MAV-link protocol
		# strPart = PFLAA-message
		# strPart2 = PAAVD-message
		# Id = the ICAO-identity
		strPart,strPart2,Id = subExtractADSBInfo(strADSB)
		
		if len(strPart) > 5:
			# If the string is longer than 5 chars it is considered valid
			# Start counting the planes in the list
			count = count + 1

			# Add the sentence to the list of ADSB sentences, it will transmitted later
			listADSBSentences.append(strPart)

			# Add the identity to a list for later use
			listADSBID.append(Id)


		if len(strPart2) > 5:
			# If the string is longer than 5 chars it is considered valid
			# Start counting the planes in the list
			count = count + 1

			# Add the sentence to the list of ADSB sentences
			listADSBSentences.append(strPart2)
			listADSBID.append(Id)

	# A "S"-sentence has arrived. It is sent once every 1 second and this is used to trigger some functions:  time to send information to the butterfly display
	#if strADSB.startswith("#S"):
	if str(strADSB).startswith("#S"):

		# Clear screen
		#print "----------o0o----------" # ("\033c")

		# Reset the counter
		NoOfFlarmContacts = 0

		#Read the lastest NMEA-information
		subGetNMEA()

		# Extract information from the NMEA-sentence
		for NMEAline in listNMEA:

			if NMEAline[:6] == "$PFLAA":
				# The FLARM is detecting another tranciever in "da hood"
				NoOfFlarmContacts = NoOfFlarmContacts + 1

				# Extracting the identity if the other tranciever from the message
				Id = subGetFLARMID(NMEAline)					# Non-testing
				#Id = subGetFLARMID("$PFLAA,0,21,11,11,2,DDE605,0,,0,-0.1,1*32") # Testing
				# Add the identity to a list, to be used in the future
				listFLARMID.append(Id)

				# Sending the message to the COM-port
				subSendSentence(NMEAline)

			elif NMEAline[:6] == "$GPRMC":
				# GPRMC contains data such as time, long/lat, speed etc. This information needs to be extracted.
				# Extracting the infromation from the GPRMC-message
				subExtractNMEAInfo(NMEAline)

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
		strPFLAUMessage = ""
		strPFLAUID = ""
		intPFLAUDist = 0
		count = -1

# Closing the software serial port when exiting
b_serial_read_close(intComPinFLARM)
pi.stop()