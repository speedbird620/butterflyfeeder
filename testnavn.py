#from math import *
from math import radians, sin, cos, atan, atan2, sqrt, degrees

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

		Altitude = 0
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

		# subWrite2Log("LatMe: " + str(LatMe) + "LongMe: " + str(LongMe) + " Bearing N : " + str(bearingN) + " Bearing: " + str(bearing) + " MyTrack: " + str(MyTrack))

	# Bearing and distance to the target
	return int(round(Base*1000)),int(round(bearingN))


# #A:4CA948,300,,2122,52.99750,13.76526,37000,169,442,0,814,72,3,,6F1C\r\n
# #A:424313,,,2362,52.43431,14.84535,37000,65,456,0,806,61,0,,6843\r\n

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
    """
    def __init__(self, sentance, diffLat=0, diffLong=0):
        import time

        (self.format,
         self.ICAO ,
         self.FLAGS,
         self.CALL ,
         self.SQ   ,
         self.LAT  ,
         self.LON  ,
         self.ALTfeet,
         self.TRACK,
         self.VELH ,
         self.VELV ,
         self.SIGS ,
         self.SIGQ ,
         self.FPS  ,
         self.RES  ,
         self.CRC
        ) = sentance.replace("\r\n","").replace(":",",").split(",")
        # Then adding some info and calulations:
        self.AltitudeGPSUnit = "M"
        self.AltitudeGPS = float(self.ALTfeet) * 0.3048 # According to Google..
        
        self.Distance =0
        self.Bearing = 0

        if diffLat != 0 and diffLong != 0:
            #Distance,Bearing = subGeoCalc(MyLat,MyLong,Lat,Long)
            self.Distance,self.Bearing = subGeoCalc(diffLat, diffLong,self.LAT,self.LON)
        
            

af01 = clAircraftMessage("#A:4CA948,300,,2122,52.99750,13.76526,37000,169,442,0,814,72,3,,6F1C\r\n",59.5673684, 010.4793243)
print(af01.ICAO + " Dist: " + str(af01.Distance) + " Bearing: " + str(af01.Bearing))


af02 = clAircraftMessage("#A:424313,,,2362,52.43431,14.84535,37000,65,456,0,806,61,0,,6843\r\n")
print(af02.ICAO)
