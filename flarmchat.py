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
import binascii
from math import *
from shutil import copyfile

# The butterfly and the ADS-B reciver has different com speed
intComSpeedADSB = 115200

intComSpeedFLARM = 38400 # 19200
intComPinFLARM_RX = 18

intComSpeedBUTTERFLY = 38400 # 57600
#intComPinBUTTERFLY_TX = 17
intComPinFLARM_TX = 22

go = 1
data = "xx"
count = 0
init = 1

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
    GiveUp = 0

    # The usual suspect com speed
    ComSpeed = [4800, 9600, 19200, 28800, 38400, 56000, 56000]
    ComSpeed_Pointer = 0

    while GiveUp == 0:

        if ComSpeed_Pointer > 4:
            # No joy, all of the com speeds has been tested
            GiveUp = 1

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
                        return ComSpeed[ComSpeed_Pointer], returnLine.Identity # Returning the comspeed and the identity of the FLARM
                        GiveUp = 1

                else:
                    print("Checksum failed, found: " + chkSumLine + " should be: " + chkCalculated + " line:" + newLine)

                # Cut away the handled sentence from the working material
                data = data[(2+(data.find("\r\n"))):]

        ComSpeed_Pointer = ComSpeed_Pointer + 1
        pi.bb_serial_read_close(RX)
        pi.stop() 



while go == 1:
    if init == 1:
        init = 0

        intComSpeedFLARM, MyID = subGetFLARM_ID(intComPinFLARM_TX,intComPinFLARM_RX)

        try:
            pi = pigpio.pi()
            pi.set_mode(intComPinFLARM_RX, pigpio.INPUT)
            pi.set_mode(intComPinFLARM_TX, pigpio.OUTPUT)
            pi.bb_serial_read_open(intComPinFLARM_RX, intComSpeedFLARM, 8)

        except:
            # The software serial port is already opened, closing the port and opening it again
            pi.bb_serial_read_close(intComPinFLARM_RX)
            pi.stop()

            pi = pigpio.pi()
            pi.set_mode(intComPinFLARM_RX, pigpio.INPUT)
            pi.set_mode(intComPinFLARM_TX, pigpio.OUTPUT)
            pi.bb_serial_read_open(intComPinFLARM_RX, intComSpeedFLARM, 8)

    time.sleep(1)

    # No Simulation, get the real deal
    data = ""
    (count, slask) = pi.bb_serial_read(intComPinFLARM_RX)
    if count > 0:
        data = slask.decode("ascii", "ignore")

    print(data)
    
    #subComCheckFLARM(pi)

    # #strRequestID = "$PFLAC,R,RADIOID*56\r\n"
    # strRequestID = "$PFLAC,R,RADIOID\r\n"
    
    # # Transmit the data with "bit banging"
    # pigpio.exceptions = True
    # pi.wave_clear()
    # pi.wave_add_serial(intComPinFLARM_TX, intComSpeedFLARM, strRequestID)
    # wid=pi.wave_create()
    # pi.wave_send_once(wid)   # transmit serial data
    # while pi.wave_tx_busy(): # wait until all data sent
    #     pass
    # pi.wave_delete(wid)


# Closing the software serial port when exiting
pi.b_serial_read_close(intComPinFLARM_RX)
pi.stop() 