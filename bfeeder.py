#!/usr/bin/env python3

# Butterfly feeder - the code
# The manual: https://bit.ly/2pCb3GA
#
# Revision 0.1, first released version
# Revision 0.2, added harware watchdog
# Revision 0.3, removed confirmation of FLARM-ID in order to set comspeed
# Revision 0.4, added function to monitor the scantime
# Revision 0.5, bugfixed function to monitor the scantime
# Revision 0.6, writing input from butterfly to FLARM
#
# avionics@skyracer.net

from machine import UART, Pin, I2C
import machine
import ssd1306
import ubinascii
import time
import math
import pyb

Init = True
listNMEA = []		# List of NMEA-messages
listADSB = []		# List of ADSB-messages
listDisplay = []	# List of ADSB-messages for OLED-display

x = 0
MyLat = 0.0
MyLong = 0.0
ZoomFactor = 1
MaxRange = 60000 # Meters
MaxDeltaAlt = 20000
MyID = ""
MyAlt = 0
MessageFromNMEAPort = ""
MessageFromADSBPort = ""
LeftOverNMEAPort = ""
LeftOverADSBPort = ""
last = 0
tmax = 0
diff = 0
Scantime_Count = 0
StartScantime_Counter = False
StartScantime_Counter_Old = False
ScanTimeTest = 0

# For the scantime tester
#TestScan = Pin('X8', Pin.IN)
X8_IN = Pin('X8', Pin.IN, Pin.PULL_UP)

# For the uart ports
Butterfly_UART_No = 3
FLARM_UART_No = 2
ADSB_UART_No = 6

#Butterfly_UART_No = 4
#FLARM_UART_No = 6
#ADSB_UART_No = 2

# For the hardware watchdog
pinWD_OUT = Pin('X6', Pin.OUT)
pinPyReady = Pin('X5', Pin.OUT)
WatchDogValue = False

# For the OLED display
pinVCC = Pin('X10', Pin.OUT)
pinGND = Pin('X9', Pin.OUT)

pinVCC.high()
pinGND.low()

time.sleep(0.2)

psda = machine.Pin('Y7', machine.Pin.OUT_PP)
pscl = machine.Pin('Y8', machine.Pin.OUT_PP)

i2c = machine.I2C(scl=pscl, sda=psda)
oled = ssd1306.SSD1306_I2C(128, 64, i2c, 60)

def subCheckSum(sentence):

	strCalculated = ""

	# Saving the incoming checksum for reference
	strOriginal = sentence[-2:]

	# Remove checksum
	sentence = sentence[:-3]

	# Remove $
	sentence = sentence[1:]

	chksum = ""
	calc_cksum = 0

	#print("Scrutinized string: " + sentence)

	# Calculating checksum
	for s in sentence:

		#print "s: " + str(s)
		calc_cksum ^= ord(s) 	#calc_cksum ^= ord(s)
		strCalculated = hex(calc_cksum).upper()

	# Removing the  "0X" from the hex value
	try:
		strCalculated = strCalculated[2:]

		# If the checksum is a single digit, adding a zero in front of the digit
		if len(strCalculated) == 1:
			strCalculated = "0" + strCalculated

		#print("chksum: " + strCalculated + " " + " sentence: " + sentence)

	except:
		whatever = True

	# Returning the provided checksum (from the original message) and the calculated 
	return strOriginal, strCalculated

def subGeoCalc(LatMe,LongMe,LatTarget,LongTarget):
	# Calculate distance and bearing between two coordinates

	Base = 0
	bearing = 0
	bearingN = 0

	try:
		# Check if the variables are valid, if not the function is not excecuted
		float(LatTarget)
		float(LongTarget)

		if float(LatTarget) > 0 or float(LongTarget) > 0:
			# Variables are valid, enter the main function
			lat1 = LatMe
			lon1 = LongMe
			lat2 = float(LatTarget)
			lon2 = float(LongTarget)

			Aaltitude = 0
			bearing = 0
			Oppsite  = 20000

			#Haversine Formuala to find vertical angle and distance
			lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

			dlon = lon2 - lon1
			dlat = lat2 - lat1
			a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
			c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
			Base = 6371 * c

			#Horisontal Bearing
			def calcBearing(lat1, lon1, lat2, lon2):
				dLon = lon2 - lon1
				y = math.sin(dLon) * math.cos(lat2)
				x = math.cos(lat1) * math.sin(lat2)  - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
				return math.atan2(y, x)

			bearingN = calcBearing(lat1, lon1, lat2, lon2)

			bearingN = math.degrees(bearingN)

			bearing = bearingN

			#subWrite2LogsubWrite2Log("LatMe: " + str(LatMe) + "LongMe: " + str(LongMe) + " Bearing N : " + str(bearingN) + " Bearing: " + str(bearing) + " MyTrack: " + str(MyTrack))


	except:
		#print "Float failed subGeoCalc"
		whatever = True
		print("whatever")
	
	# Bearing and distance to the target
	return int(round(Base*1000)),int(round(bearingN))

class clADSBInfo(object):
	
	def __init__(self, sentance):

		# ICAO	ICAO number of aircraft (3 bytes) 3C65AC
		# FLAGS   Flags (see below) 1
		# CALL	Callsign of aircraft N61ZP
		# SQ	  SQUAWK of aircraft 7232
		# LAT	 Latitude in degrees 57.57634
		# LON	 Longitude in degrees 17.59554
		# ALT	 Pressure altitude in feets 5000
		# TRACK   Track of aircraft in degrees [0,360) 35
		# VELH	Horizontal velocity of aircraft in knots 464
		# VELV	Vertical velocity of aircraft in ft/min -1344
		# SIGS	Signal strength in mV 840
		# SIGQ	Signal quality in mV 72
		# FPS	 Number of raw MODE-S frames received from aircraft during last second 5
		# RES	 Reserved for future use
		# CRC	 CRC16 (described in CRC section) 2D3E
		#A:4CA948,300,,2122,52.99750,13.76526,37000,169,442,0,814,72,3,,6F1C\r\n"
		
		(self.ICAO,
		self.FLAGS ,
		self.Call ,
		self.Squak ,
		self.Lat ,
		self.Long ,
		self.Alt ,
		self.Track ,
		self.VelH ,
		self.VelV ,
		self.SigS ,
		self.SigQ ,
		self.FPS ,
		self.RES ,
		self.CRC
		) = sentance.replace("\r\n","").replace("*",",").split(",")

		try:
			# Check if the numbers are valid
			str(self.ICAO)
			float(self.Lat)
			float(self.Long)
			int(self.Alt)
			int(self.VelH)
			int(self.VelV)

		except:	
			# Negative, the numbers were not valid
			self.ICAO = ""
			self.Lat = 0
			self.Long = 0
			self.Alt = 0
			self.VelH = 0
			self.VelV = 0
		
		# Recalculate altitude into something understandable (meters)
		self.Alt = int((float(self.Alt) / 3.2808399))
		self.VelH = str(round(float(self.VelH)*0.514444))
		self.VelV = round(int(self.VelV)*0.00508,1)

class clNMEAMessage(object):

	def __init__(self, sentance):

		# $GPGGA,193756.00,5911.23142,N,01739.43039,E,1,09,1.30,13.4,M,24.1,M,,*62
		# Example $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh

		# hhmmss.ss = UTC of position
		# llll.ll = latitude of position
		# a = N or S
		# yyyyy.yy = Longitude of position
		# a = E or W
		# x = GPS Quality indicator (0=no fix, 1=GPS fix, 2=Dif. GPS fix)
		# xx = number of satellites in use
		# x.x = horizontal dilution of precision
		# x.x = Antenna altitude above mean-sea-level
		# M = units of antenna altitude, meters
		# x.x = Geoidal separation
		# M = units of geoidal separation, meters
		# x.x = Age of Differential GPS data (seconds)
		# CRC: A*6E

		(self.GPGGA,
		 self.Time,
		 self.Lat,
		 self.N_or_S,
		 self.Long,
		 self.E_or_W,
		 self.SigQ,
		 self.NumSat,
		 self.DiffH,
		 self.Alt,
		 self.AltUnit,
		 self.Geod,
		 self.GeodUnit,
		 self.Diff,
		 self.CRC
		) = sentance.replace("\r\n","").replace(":",",").split(",")

def subExtractNMEAInfo(Sentence):
	Lat = 0
	Long = 0
	Time = ""

	# Recieves the NMEA sentence. Time to assemble the stirng and make some sense of it

	strNMEASplit = clNMEAMessage(Sentence)
	#print(strNMEASplit.Date)

	Sentence=Sentence[:-6]

	#subWrite2Log("NMEA Time: " + strNMEASplit.Time + " , RW: " + strNMEASplit.RecieverWarning + " , lat: " + strNMEASplit.Lat +  " , long: " + strNMEASplit.Long + " , VelH: " + strNMEASplit.VelH + " , track: " + strNMEASplit.Track + " , date: " + strNMEASplit.Date)

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
			LongDegrees = strNMEASplit.Long[:3] * -1
		else:
			LongDegrees = strNMEASplit.Long[:3]

		LongMinutes = strNMEASplit.Long[-8:]
		Long = float(LongDegrees) + (float(LongMinutes)/60)

	return 	Lat,Long,strNMEASplit.Alt

def subExtractADSBInfo(Sentence,MyLat,MyLong,MyAlt):

	# Recieves the MAVLink sentence. Time to assemble the stirng and make some sense of it
	ADSBSplit = clADSBInfo(Sentence)

	Distance = 0.0
	Bearing = 0.0

	#print "MyLat: " + str(MyLat) + " MyLong: " + str(MyLong) + " Lat: " + str(Lat) + " Long: " + str(Long)
	Distance,Bearing = subGeoCalc(MyLat,MyLong,ADSBSplit.Lat,ADSBSplit.Long)

	Whatever = 0
	tempCall = "" 
	deltaNorth = 0.0
	deltaEast = 0.0
	deltaAlt = 0
	Empty = ""
	strPFLAAMsg = ""

	#print(ADSBSplit.ICAO)

	if len(ADSBSplit.ICAO) > 0 and len(ADSBSplit.Call) > 0:
		WriteOLED(ADSBSplit.ICAO[-3:] + " = " + ADSBSplit.Call)

	# Calculating the altitude difference
	try:
		# Quick and dirty, yes indeed. Sorry I cant get my head around why this fail once in 10 000 times ... if someone can explain why I ough this person a beer or a glass of wine. Ill have a Prosecco ... or two.
		deltaAlt = round(int(ADSBSplit.Alt) - float(MyAlt))
	except:
		deltaAlt = 0

	#if Distance == 0 or Bearing == 0 or (Distance/ZoomFactor) > MaxRange or Call == MyID or str(VelH) == Empty or str(VelV) == Empty or deltaAlt > MaxDeltaAlt or deltaAlt < MaxDeltaAlt:
	if Distance == 0 or (Distance/ZoomFactor) > MaxRange or ADSBSplit.ICAO == MyID:
		# Distance and bearing is zero, values are not valid OR the plane is too far away OR its my own signal OR VelV and VelH are empty: skip further handling

		if (math.cos(math.radians(Bearing))*(Distance)/ZoomFactor) > MaxRange or (math.sin(math.radians(Bearing))*(Distance)/ZoomFactor) > MaxRange:
			print("Far far away ..." + str(Distance/ZoomFactor) + " : " + str(MaxRange))
			

	else:
		# Numbers are valid, lets compile the string

		# Calculating the horisontal distance to the target
		deltaNorth = (math.cos(math.radians(Bearing))*(Distance)/ZoomFactor)

		# Calculating the horisontal distance to the target
		deltaEast = (math.sin(math.radians(Bearing))*(Distance)//ZoomFactor)

		print("dN: " + str(int(deltaNorth)) + " dE: " + str(int(deltaEast)) +  " dA: " + str(deltaAlt) + " Call: " + ADSBSplit.Call + " Squak: " + ADSBSplit.Squak)
		#subWrite2Log("Diff Call: " + Call + " n: " + str(int(deltaNorth)) + " e: " + str(int(deltaEast)) +  " a: " + str(deltaAlt))
		print(str(deltaAlt))
		# PFLAA-message
		# PFLAA,<AlarmLevel>,<RelativeNorth>,<RelativeEast>,<RelativeVertical>,<IDType>,<ID>,<Track>,<TurnRate>,<GroundSpeed>,<ClimbRate>,<Type>
		strPFLAAMsg =  "$PFLAA,0," + str(int(deltaNorth)) + "," + str(int(deltaEast)) + "," + str(deltaAlt) + ",1," + ADSBSplit.ICAO + "," + ADSBSplit.Track + ",0.0," + ADSBSplit.VelH + "," + str(ADSBSplit.VelV) + ",8"
		#listPlanes.append("PFLAA,0," + str(int(deltaNorth)) + "," + str(int(deltaEast)) + "," + str(int(deltaAlt)) + ",1," + tempCall + "," + Track + ",0.0," + "20" + "," + str(round(float(VelV)*0.00508)) + ",8")
	
	return strPFLAAMsg,ADSBSplit.ICAO

def WriteOLED(strOut):
	# This sub is supposed to write info to the OLED display

	Match = False

	for row in listDisplay:
		if strOut == row:
			print(row + " " + strOut)
			print(row)
			Match = True

	if not Match:

		#Blanking the display
		oled.fill(0)
		oled.show()

		# Append the string to the list
		listDisplay.append(strOut)

		# Keep the list short
		if len(listDisplay) > 5:
			flopp = listDisplay.pop(0)

		# Print out the list to the
		r = 0
		for row in listDisplay:
			oled.text(row, 0, r)
			r = r + 10
		oled.show()

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

def subGetFLARM_ID(UARTNo):
	# This sub is supposed to identify the FLARM identity and the com speed

	FoundID = ""

	# The usual suspect com speed
	ComSpeedArray = [19200, 4800, 9600, 14400, 28800, 38400, 56000, 57600, 115200, 230400]
	#ComSpeedArray = [14400, 19200, 57600]

	for ComSpeed in ComSpeedArray:

		print("Com check @"+ str(ComSpeed) + " on UART-port: " + str(UARTNo))

		WriteOLED("Checking " + str(ComSpeed))

		# Initializing FLARM-port
		tmpUART = UART(UARTNo, ComSpeed)
		tmpUART.init(ComSpeed, bits=8, parity=None, stop=1, rxbuf=512)

		# Waiting for the response
		time.sleep(2)

		if tmpUART.any() > 0:
			# Whoohoo! There are information in the UART2 buffer

			# Reading the information
			try:
				# Also quick and dirty, guilty as charged. This trice in five tries. Ff someone can explain why I ough this person a beer or a glass of wine. Ill have a bottle of Prosecco ... or two.
				MessageFromNMEAPort = tmpUART.read().decode()

				if MessageFromNMEAPort.find("\r\n") > -1:
					# Yes, there is a carrage return in the sentence

					# Extracting a sentence for further handling
					MessageFromNMEAPort = MessageFromNMEAPort[:(MessageFromNMEAPort.find("\r\n"))]

			except:
				MessageFromNMEAPort = ""

			print (MessageFromNMEAPort)

			chkSumLine, chkCalculated = subCheckSum(MessageFromNMEAPort)

			print(chkSumLine + " calculated: " + chkCalculated)

			# Time to check the checksum
			if chkSumLine == chkCalculated and chkCalculated != "" and chkSumLine != "":
				# Bang on! Lets ask the FLARM about its identity

				# Enabling the watchdog
				print("Enabled watchdog")

				pinPyReady.high()
				pinWD_OUT.low()
				time.sleep(0.2)
				pinWD_OUT.high()
				time.sleep(0.2)
				pinWD_OUT.low()
				time.sleep(0.2)
				pinWD_OUT.high()
				time.sleep(0.2)

				# Asking politely about the ID from the flarm
				tmpUART.write("$PFLAC,R,RADIOID\r\n")

				pinPyReady.low()

				# Waiting for the response
				time.sleep(2)

				if tmpUART.any() > 0:
					# Whoohoo! There are information in the UART2 buffer
					MessageFromNMEAPort = tmpUART.read().decode()

				print(MessageFromNMEAPort)
		
				#Example: $PFLAC,A,RADIOID,2,DDDAFC*77\r\n# $PFLAU,0,1,1,1,0,,0,,,*4F\r\n$GPRMC,110522.00,A,5911.22939,N,01739.41033,E,0.895,208.08,050121,,,A*6C\r\n$PGRMZ,-381,F,2*1D\r\n$GPGGA,110522.00,5911.22939,N,01739.41033,E,1,07,1.24,12.2,M,24.1,M,,*69\r\n$PFLAU,0,1,1,1,0,,0,,,*4F\r\n$GPRMC,110523.00,A,5911.22911,N,01739.40937,E,0.832,209.30,050121,,,A*6C\r\n$PGRMZ,-380,F,2*1C\r\n$GPGGA,110523.00,5911.22911,N,01739.40937,E,1,07,1.24,13.0,M,24.1,M,,*6D

				# Delete any chars before the start character. In this case $
				MessageFromNMEAPort = MessageFromNMEAPort[(MessageFromNMEAPort.find("$")):]


				while len(MessageFromNMEAPort) > 0 and not MessageFromNMEAPort.find("\r\n") == -1:

					# Entering this while, the variable MessageFromNMEAPort is a long string with several embedded 
					# carrage returns. This part of the program is dividing the string into a list of 
					# NMEA senteces. The tricky part is that the information from the serial port might
					# be read during a trasmission. This means that the last part of the NMEA-sentence 
					# might be missing. Therefore if the last sentence of the string is not ended with 
					# a carrage return, that part will be saved in a variable (ADSBEndFromLastMessage) 
					# and will be added in the beginning of the next.

					# Example: #S:1.4,,,4,,,95A7\r\n#A:49410D,,NJE542Q,,59.24611,17.46571,8425,209,253,2624,795,112,1,,9267\r\n

					# If the buffer was read during the message it will not be complete
					#ADSBData = ADSBEndFromLastMessage + MessageFromNMEAPort

					if MessageFromNMEAPort.find("\r\n") > -1:
						# Yes, there is a carrage return in the sentence

						# Extracting a sentence for further handling
						newLine = MessageFromNMEAPort[:(MessageFromNMEAPort.find("\r\n"))]

						#print("newLine: " + newLine)

						# Is this the line the we are looking for? PFLAC is the message containing the FLARM-identity.
						if newLine[:6] == "$PFLAC":

							# Extracting the information within the string
							returnLine = clRADIOIDMessage(newLine)

							print("Com speed detected: " + str(ComSpeed))
							print("ID: " + returnLine.Identity)
							FoundID = returnLine.Identity
							return ComSpeed, returnLine.Identity # Returning the comspeed and the identity of the FLARM
							break

							# <-- Rev 0.3

					# Cut away the handled sentence from the working material
					MessageFromNMEAPort = MessageFromNMEAPort[(2+(MessageFromNMEAPort.find("\r\n"))):]

				return ComSpeed, "000000" # Returning the comspeed and null as the identity of the FLARM

		tmpUART.deinit
	
	if FoundID == "":
		# No joy, sending response
		return -1, ""

while True:
	# Continious execution from here

	#machine.freq(84000000)

	if Init:
		Init = False

		FlarmSpeed = 0
		
		while FlarmSpeed <= 0:
			FlarmSpeed, MyID = subGetFLARM_ID(FLARM_UART_No)

		if FlarmSpeed < 1:
			# Communication with the FLARM has been established, correcting the comspeed
			print("No communication was detected, keeping the default values")
			FlarmSpeed = 38400
		else:
			# Writing to the OLED
			WriteOLED(MyID + " @ " + str(FlarmSpeed))

		# Initializing FLARM-port
		u2 = UART(FLARM_UART_No, FlarmSpeed)
		u2.init(FlarmSpeed, bits=8, parity=None, stop=1, rxbuf=2048)

		# Initializing Butterfly-port
		u3 = UART(Butterfly_UART_No, FlarmSpeed)
		u3.init(FlarmSpeed, bits=8, parity=None, stop=1, rxbuf=2048)


		# Initializing ADSB-port
		u6_speed = 115200
		u6 = UART(ADSB_UART_No, u6_speed)
		u6.init(u6_speed, bits=8, parity=None, stop=1, rxbuf=512)

		# Fiddling in order to charge up the hardware watchdog
		pinPyReady.high()
		pinWD_OUT.low()
		time.sleep(0.2)
		pinWD_OUT.high()
		time.sleep(0.2)
		pinWD_OUT.low()

		# Seding out a fake target just to show that the bfeeder har started
		u3.write("$PFLAA,0,9999,0,0,1,BBBBBB,0,0.0,0,0,8*51\r\n")

		last = pyb.millis()		# Rev 0.5

	if u2.any() > 0:
		# Whoohoo! There are information in the UART2 buffer

		# Reading the information

		try:
			MessageFromNMEAPort = u2.read().decode()
		except:
			#print("Exeption: " + MessageFromNMEAPort)
			whatver = True

		if len(MessageFromNMEAPort) > 5:

			#Example: $PFLAU,0,1,1,1,0,,0,,,*4F\r\n$GPRMC,104700.00,A,5911.23604,N,01739.44266,E,0.924,32.24,301220,,,A*57\r\n$PGRMZ,540,F,2*3B\r\n$GPGGA,104700.00,5911.23604,N,01739.44266,E,1,08,1.35,55.1,M,24.1,M,,*66\r\n

			# Adding the leftovers to the party
			MessageFromNMEAPort = LeftOverNMEAPort + MessageFromNMEAPort

			#print("NMEAPort: " + MessageFromNMEAPort)


			# Delete any chars before the start character. In this case $
			MessageFromNMEAPort = MessageFromNMEAPort[(MessageFromNMEAPort.find("$")):]


			while len(MessageFromNMEAPort) > 0 and not MessageFromNMEAPort.find("\r\n") == -1:

				# Entering this while, the variable MessageFromNMEAPort is a long string with several embedded 
				# carrage returns. This part of the program is dividing the string into a list of 
				# NMEA senteces. The tricky part is that the information from the serial port might
				# be read during a trasmission. This means that the last part of the NMEA-sentence 
				# might be missing. Therefore if the last sentence of the string is not ended with 
				# a carrage return, that part will be saved in a variable (ADSBEndFromLastMessage) 
				# and will be added in the beginning of the next.

				# Example: #S:1.4,,,4,,,95A7\r\n#A:49410D,,NJE542Q,,59.24611,17.46571,8425,209,253,2624,795,112,1,,9267\r\n

				# If the buffer was read during the message it will not be complete
				#ADSBData = ADSBEndFromLastMessage + MessageFromNMEAPort

				if MessageFromNMEAPort.find("\r\n") > -1:
					# Yes, there is a carrage return in the sentence

					# Extracting a sentence for further handling
					newLine = MessageFromNMEAPort[:(MessageFromNMEAPort.find("\r\n"))]

					if len(newLine) > 1:
						# Extract the provided checksum and calculate the cecksum as reference
						chkSumLine, chkCalculated = subCheckSum(newLine)

						# Time to check the checksum
						if chkSumLine == chkCalculated:
							# Bang on! Lets add the line to the list
							listNMEA.append(newLine + "\r\n")
						else:
							print("Checksum failed, found: " + chkSumLine + " should be: " + chkCalculated + " line:" + newLine)

					#print("Checksum, found: " + chkSumLine + " should be: " + chkCalculated + " line:" + newLine)

					# Cut away the appended sentence from the working material
					MessageFromNMEAPort = MessageFromNMEAPort[(2+(MessageFromNMEAPort.find("\r\n"))):]
			
			LeftOverNMEAPort = MessageFromNMEAPort
			#print("LeftOverNMEAPort: " + LeftOverNMEAPort)

		if not X8_IN.value():
			ScanTimeTest = ScanTimeTest + 10
			time.sleep_ms(ScanTimeTest)
			#print("TestScan = " + str(ScanTimeTest))
		#else:
		#	print("TestScan = False")


	if u6.any() > 0:
		# New message from ADSB-reciever is in the buffer

		# Read the buffer
		try:
			MessageFromADSBPort = u6.read().decode()
			#MessageFromADSBPort = "#S:1.6,,,14,,,C26C\r\n#A:4AC9E1,1F00,SAS436L,,59.22249,17.80600,6875,10,305,-1216,746,119,6,,99D5\r\n#A:502D0B,,BTI2HU,,59.10291,18.00650,38000,299,435,-64,676,57,0,,16B8\r\n#A:5110E7,1F00,SAS178,,59.20345,17.70914,7825,26,247,-1280,913,246,8,,FA7B\r\n#A:3944EC,,AFR1263,,59.52774,17.40575,18100,212,368,2240,668,56,0,,A94\r\n"

			print(MessageFromADSBPort)

		except:
			print("Except" + MessageFromADSBPort)

		TestString = ""

		#TestString = TestString + "#A:4400BE,,TAY4804,,59.60303,18.10153,34000,251,440,-1200,705,88,4,,D693\r\n"
		#TestString = TestString + "#A:45B4E1,300,MMD6265,,59.38678,17.65906,6375,241,287,1984,776,122,1,,8A1F\r\n"
		#TestString = TestString + "#A:49410D,,NJE542Q,,59.24611,17.46571,8425,209,253,2624,795,112,1,,9267\r\n"
		#TestString = TestString + "#A:4400BE,,,,59.44235,17.18289,27800,250,449,-4224,684,61,0,,E9CC\r\n"
		#TestString = TestString + "#A:4A9324,1D00,SAS41W,,59.11404,17.76792,8500,10,254,-1536,767,120,6,,9385\r\n"
		#TestString = TestString + "#A:4601F5,,FIN868L,,59.43837,17.27382,19000,58,268,0,693,73,0,,1E2F\r\n"
		#TestString = TestString + "#A:4AC9E1,1F00,SAS436L,,59.22249,17.80600,6875,10,305,-1216,746,119,6,,99D5\r\n"
		#TestString = TestString + "#A:4AC9E1,1F00,SAS436L,,59.22249,17.80600,6875,10,305,-1216,746,119,6,,99D5\r\n"
		#TestString = TestString + "#A:4A9099,,SEDDY,,59.22902,17.86074,4900,109,243,-2112,784,128,0,,3500\r\n"
		#TestString = TestString + "#A:4A9099,,SEDDY,,59.22902,17.86074,4900,109,243,-2112,679,64,0,,E422\r\n"
		#TestString = TestString + "#A:4AC9B2,1F00,JET5,,59.38515,17.65920,7925,240,283,2752,786,133,11,,97E4\r\n"
		#TestString = TestString + "\r\n"
		#TestString = TestString + "\r\n"
		#TestString = TestString + "\r\n"
		#TestString = TestString + "\r\n"
		#TestString = TestString + "\r\n"
		#TestString = TestString + "\r\n"
		#TestString = TestString + "\r\n"
		#TestString = TestString + "\r\n"

		MessageFromADSBPort = MessageFromADSBPort + TestString

		# The sentence is MAV-link protocol, this is an example:
		# "#S:1.4,,,4,,,95A7\r\n#A:49410D,,NJE542Q,,59.24611,17.46571,8425,209,253,2624,795,112,1,,9267\r\n"

		#while len(MessageFromADSBPort) > 0 and not MessageFromADSBPort.find("\r\n") == -1:
		while len(MessageFromADSBPort) > 0:

			# Entering this while, the variable MessageFromADSBPort is a long string with several embedded 
			# carrage returns. This part of the program is dividing the string into a list of 
			# NMEA senteces. The tricky part is that the information from the serial port might
			# be read during a trasmission. This means that the last part of the NMEA-sentence 
			# might be missing. Therefore if the last sentence of the string is not ended with 
			# a carrage return, that part will be saved in a variable (ADSBEndFromLastMessage) 
			# and will be added in the beginning of the next.

			# Example: #S:1.4,,,4,,,95A7\r\n#A:49410D,,NJE542Q,,59.24611,17.46571,8425,209,253,2624,795,112,1,,9267\r\n

			# If the buffer was read during the message it will not be complete
			#ADSBData = ADSBEndFromLastMessage + MessageFromADSBPort

			# Delete any chars before the start character. In this case #
			MessageFromADSBPort = MessageFromADSBPort[(MessageFromADSBPort.find("#")):]

			#print("x: " + MessageFromADSBPort)

			if MessageFromADSBPort.find("\r\n") > -1 and str(MessageFromADSBPort).startswith("#A"):
				# Yes, there is a carrage return in the sentence

				# Extracting a sentence for further handling
				newLine = MessageFromADSBPort[:(MessageFromADSBPort.find("\r\n"))]

				# Removing '#A:' from the string
				newLine = newLine[3:]

				# Exxract information from the MAV-link protocol
				# strPart = PFLAA-message
				# strPart2 = PAAVD-message
				# Id = the ICAO-identitx§y
				strPart,Id = subExtractADSBInfo(newLine,MyLat,MyLong,MyAlt)

				print("toDisplay: " + strPart)

				if len(strPart) > 0:
					# If the string is longer than 5 chars it is considered valid

					# Lets grab us a checksum
					chkSumLine, chkCalculated = subCheckSum(strPart + "*4E")

					# Add the sentence to the list of ADSB sentences, it will transmitted later
					listNMEA.append(strPart + "*" + chkCalculated + "\r\n")

				print("ADSBPort: " + newLine)

			if MessageFromADSBPort.find("\r\n") > -1 and str(MessageFromADSBPort).startswith("#S"):
				if StartScantime_Counter == True:
					print("Scantime_Count: " + str(Scantime_Count))
					Scantime_Count = Scantime_Count + 1
					if Scantime_Count > 20:
						StartScantime_Counter = False
						ScanTimeTest = 0

						# Engage watchdog
						pinPyReady.high()
					
						# Seding out a fake target just to show that the bfeeder has started
						bootMSG = "$PFLAA,0,9999,0,0,1,BBBBBB,0,0.0,0,0,8*51"
						u3.write(bootMSG + "\r\n")

						print("engane watchdog: " + str(diff))
						WriteOLED("WD started: " + str(diff))  # Added in rev 0.5

			# Cut away the appended sentence from the working material
			MessageFromADSBPort = MessageFromADSBPort[(2+(MessageFromADSBPort.find("\r\n"))):]

	# ---> rev 0.6
	if u3.any() > 0:

		# Read the buffer
		try:
			MessageFromButterflyPort = u3.read().decode()
			#MessageFromADSBPort = "#S:1.6,,,14,,,C26C\r\n#A:4AC9E1,1F00,SAS436L,,59.22249,17.80600,6875,10,305,-1216,746,119,6,,99D5\r\n#A:502D0B,,BTI2HU,,59.10291,18.00650,38000,299,435,-64,676,57,0,,16B8\r\n#A:5110E7,1F00,SAS178,,59.20345,17.70914,7825,26,247,-1280,913,246,8,,FA7B\r\n#A:3944EC,,AFR1263,,59.52774,17.40575,18100,212,368,2240,668,56,0,,A94\r\n"

			print("U3: " + MessageFromButterflyPort)

		except:
			print("U3 except" + MessageFromButterflyPort)


		# Write messageto serial port
		u2.write(MessageFromButterflyPort)
	# <--- rev 0.6



	# Extract information from the NMEA-sentence
	while len(listNMEA) > 0:

		# Extracting a line from the list
		lineNMEA = listNMEA.pop()

		# Write messageto serial port
		u3.write(lineNMEA)
		
		#print("lineNMEA: " + lineNMEA)

		if lineNMEA[:6] == "$GPGGA":
			# GPGGA contains data such as time, long/lat, speed etc. This information needs to be extracted.
			# Extracting the infromation from the GPRMC-message
			
			MyLat,MyLong,MyAlt = subExtractNMEAInfo(lineNMEA)
			# print("MyLat: " + str(MyLat))
			# print("MyLong: " + str(MyLong))
			# print("MyAlt: " + MyAlt)

	# Scantime handler
	
	# Calculating the scan time
	now = pyb.millis()
	
	# diff is the time of the last scan
	diff = now - last 
	
	if ((diff > tmax)):
		# The scan has set a new max length
		tmax = diff
		print("Execution: " + str(tmax))
	
	if diff > 100:
		# The scan is more than 100 ms
		print("diff: " + str(diff))

	# Rev 0.4 -->

	if diff > 400:
		# The scan is more than 900 ms, time to bypass information
		StartScantime_Counter = True
		Scantime_Count = 0
		
	if StartScantime_Counter and not StartScantime_Counter_Old:
		# bootMSG = "$PFLAA,0,9999,0,0,1,BBBBBB,0,0.0,0,0,8*51"
		# bootMSG = "$PFLAA,0,-9999,0,0,1,CCCCCC,0,0.0,0,0,8*7C"

		# Seding out a fake target just to show that the bfeeder has stopped 	# Added in rev 0.5
		u3.write("$PFLAA,0,-9999,0,0,1,CCCCCC,0,0.0,0,0,8*7C\r\n")				# Added in rev 0.5

		print("disengane watchdog: " + str(diff))
		WriteOLED("WD stopped: " + str(diff))	# Added in rev 0.5

		# Disengage watchdog
		pinPyReady.low()

	# <--- Rev 0.4

	last = now
	StartScantime_Counter_Old = StartScantime_Counter

	# Setting the watchdog output
	WatchDogValue = not WatchDogValue
	pinWD_OUT.value(WatchDogValue)

	# Setting the Ready-signal for the hardware watchdog
	#pinPyReady.value(True)
	pinPyReady.high()

#pinPyReady.value(False)
pinPyReady.low()
