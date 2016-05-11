from __future__ import division
from Adafruit_BME280 import * # Enables environmental data
import RPi.GPIO as GPIO  # Enables reading of proximity sensors
# Enable the SPI interface:
import spidev
# Enable the servo control
import wiringpi
# More standard libraries
import numpy
from math import * # Enables sine/cosine
from time import sleep
import random

def mobileThermostat():
    print 'mobileThermostat'
    # Initialize system (Calibrate servo, connect bluetooth)
    conn = startSPI()
    cal = calibrateServo(conn)
    connectBlue()
    position = [0,0]
    newPosition = [0,0]
    while True:
        try:
            # Get the room data, filter noise, and verify the position of the robot with the estimated position
            roomData = mapRoom(cal, conn)
            print len(roomData)
            print roomData
            roomData = filterMap(roomData)
            newPosition = localization(newPosition, roomData)
            position = newPosition

            # Send the position and map data
            sendMapData(roomData,position)

            # Get and send the environmental data and the position of the robot
            envData = getEnvData()
            sendEnvData(envData, position)

            # Decide on a new position to move the robot to and move the robot
            newPosition = nextPosDec(roomData, position)
            moveRobot(newPosition, position)
        except KeyboardInterrupt:
            print "Exiting mobileThermostat"
            break




def startSPI():
    conn = spidev.SpiDev(0, 0)
    return conn



def calibrateServo(conn):
    print 'calibrateServo'
    # Set up the servo
    wiringpi.wiringPiSetupGpio()
    wiringpi.pinMode(19,1)
    wiringpi.digitalWrite(19,0)

    cal = [0,0]
    min1 = []
    min2 = []

    period_time = 0.02
    # This pulse time puts the servo at 0 degrees
    pulse_time = 0.00037
    for i in range(1,100):
        wiringpi.digitalWrite(19,1)
        sleep(pulse_time)
        wiringpi.digitalWrite(19,0)
        sleep(period_time-pulse_time)
    min1.append(readServo(conn))
    # This pulse time puts teh servo at 180 degrees
    pulse_time = 0.0023
    for i in range(101,200):
        wiringpi.digitalWrite(19,1)
        sleep(pulse_time)
        wiringpi.digitalWrite(19,0)
        sleep(period_time-pulse_time)
    min2.append(readServo(conn))
        # This pulse time puts the servo at 0 degrees
    pulse_time = 0.00037
    for i in range(1,100):
        wiringpi.digitalWrite(19,1)
        sleep(pulse_time)
        wiringpi.digitalWrite(19,0)
        sleep(period_time-pulse_time)
    min1.append(readServo(conn))
    # This pulse time puts teh servo at 180 degrees
    pulse_time = 0.0023
    for i in range(101,200):
        wiringpi.digitalWrite(19,1)
        sleep(pulse_time)
        wiringpi.digitalWrite(19,0)
        sleep(period_time-pulse_time)
    min2.append(readServo(conn))


    cal[0] = numpy.mean(min1)
    cal[1] = numpy.mean(min2)

    print cal
    return cal




def connectBlue():
    print 'connectBlue'





def mapRoom(cal, conn):
    print 'mapRoom'
    # This function assumes the robot returns to face the same direction with each move

    proxR = 1 # This will be constant - its the distance from the origin on the robot to the end of each prox sensor

    servoStart = 0
    servoEnd = 180

    xyList = []

    for i in range(0,180):

        try:
            # Move the servo to incremental angle
            servoPos = readServo(conn)
            moveServo(i, servoPos, cal, conn)

            # Get distances from proximity sensors
            R = readDists() # readDists should return a list of the two distances

            # Convert to cartesian points
            xy1 = [round((proxR + R[0]) * sin(radians(i)),2), round((proxR + R[0]) * cos(radians(i)),2)]
            xy2 = [round(-(proxR + R[1]) * sin(radians(i)),2), round(-(proxR + R[1]) * cos(radians(i)),2)]

            # Add xy points to xyList
            xyList.append(xy1)
            xyList.append(xy2)
        except KeyboardInterrupt:
            break

    return xyList




def readServo(conn):
    print 'readServo'
    if __name__ == '__main__':
        spiOut = []
        spiOut.append(readSPI(conn))
        spiOut.append(readSPI(conn))
        spiOut.append(readSPI(conn))
        spiOut.append(readSPI(conn))
        spiOut.append(readSPI(conn))

        #print str(numpy.mean(spiOut)) + ' from SPI'

    return numpy.mean(spiOut)


def bitstringSPI(n):
    s = bin(n)[2:]
    return '0'*(8-len(s)) + s

def readSPI(conn,adc_channel=0, spi_channel=0):

    conn.max_speed_hz = 1200000 # 1.2 MHz
    cmd = 128
    if adc_channel:
        cmd += 32
    reply_bytes = conn.xfer2([cmd, 0])
    reply_bitstring = ''.join(bitstringSPI(n) for n in reply_bytes)
    reply = reply_bitstring[5:15]
    #Need to close the conn object somehow...
    return int(reply, 2) / 2**10



def readDists():
    print 'readDists'
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    dists1 = []
    dists2 = []

    # Prox 2
    pin1 = 4
    pin2 = 5
    # Prox 1
    pin3 = 12
    pin4 = 13

    # Enable pin1 and pin3 to read prox output
    GPIO.setup(pin1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(pin3, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # Enable pin2 and pin4 to power the prox sensors at the appropriate time so they don't compete
    GPIO.setup(pin2, GPIO.OUT)
    GPIO.setup(pin4, GPIO.OUT)
    # Enable prox 2 to start first
    GPIO.output(pin2, True)
    GPIO.output(pin4, False)

    # Read prox 2
    sleep(0.25) # Prox start up takes 0.25 seconds
    GPIO.wait_for_edge(pin1, GPIO.FALLING)
    GPIO.wait_for_edge(pin1, GPIO.RISING)
    t1 = time.time()
    GPIO.wait_for_edge(pin1, GPIO.FALLING)
    t2 = time.time()
    # Compute distance
    dist2 = (t2-t1)*1000000/147
    dists2.append(dist2)

    GPIO.wait_for_edge(pin1, GPIO.FALLING)
    GPIO.wait_for_edge(pin1, GPIO.RISING)
    t1 = time.time()
    GPIO.wait_for_edge(pin1, GPIO.FALLING)
    t2 = time.time()
    # Compute distance
    dist2 = (t2-t1)*1000000/147
    dists2.append(dist2)

    GPIO.output(pin2, False)

    # Now for prox1:
    GPIO.output(pin4, True)
    # Read prox 1
    sleep(0.25) # Prox start up takes 0.25 seconds
    GPIO.wait_for_edge(pin3, GPIO.FALLING)
    GPIO.wait_for_edge(pin3, GPIO.RISING)
    t1 = time.time()
    GPIO.wait_for_edge(pin3, GPIO.FALLING)
    t2 = time.time()
    #Compute distance
    dist1 = (t2-t1)*1000000/147
    dists1.append(dist1)

    GPIO.wait_for_edge(pin3, GPIO.FALLING)
    GPIO.wait_for_edge(pin3, GPIO.RISING)
    t1 = time.time()
    GPIO.wait_for_edge(pin3, GPIO.FALLING)
    t2 = time.time()
    #Compute distance
    dist1 = (t2-t1)*1000000/147
    dists1.append(dist1)

    GPIO.output(pin4, False)
    GPIO.cleanup()
    print "prox1 distance: " + str(min(dists1)) + " inches"
    print "prox2 distance: " + str(min(dists2)) + " inches"

    return [min(dists1), min(dists2)]




def moveServo(newPos, servoPos, cal, conn):
    print 'moveServo'

    posTol = 2 # acceptable tolerance in degrees
    posErr = 1000000 # assume huge error to start

    # -2 degrees at most CCW
    # 194 degrees at most CW
    # angDiff = 194 + 2

    # 0.00037 sec pulse for 0 deg
    # 0.00230 sec pulse for 180 deg
    pulse1 = 0.00037
    pulse2 = 0.00230
    timeInc = (pulse2 - pulse1)/180
    # 0.975 at most CCW
    # 0.685 at most CW
    spiInc = (cal[1] - cal[0])/180

    period_time = 0.02 # seconds
    pulse_time = ((newPos*timeInc)+pulse1)

#     print str(servoPos)
#     print str(((newPos*spiInc)+cal[0]))
#     print str(posTol*spiInc)

    timeout = 0

    while ((servoPos > ((newPos*spiInc)+cal[0]) + posTol*spiInc) | (servoPos < ((newPos*spiInc)+cal[0]) - posTol*spiInc)) & (timeout < 100):
        # Move in the direction of newPos

        #### NOT Correct at the moment:
        if servoPos > ((newPos*spiInc)+cal[0]) + posTol*spiInc:
            try:
                wiringpi.digitalWrite(19,1)
                sleep(0.0003)
                wiringpi.digitalWrite(19,0)
                #sleep(period_time-pulse_time)

            except KeyboardInterrupt:
                break
        elif servoPos < ((newPos*spiInc)+cal[0]) - posTol*spiInc:
            try:
                wiringpi.digitalWrite(19,1)
                sleep(0.003)
                wiringpi.digitalWrite(19,0)
                #sleep(period_time-pulse_time)

            except KeyboardInterrupt:
                break

        servoPos = readServo(conn)
        print "Current pos: " + str((servoPos-cal[0])/spiInc) + " degrees"
        print "Desired pos: " + str(newPos) + " degrees"
        timeout = timeout + 1



def filterMap(roomData):
    print 'filterMap'
    return roomData





def localization(newPosition, roomData):
    print 'localization'
    # If we trust the robot to move accurately, then no verification necessary.
    # Otherwise, we can look at the new roomData.

    return newPosition





def getEnvData():
    print 'getEnvData'
    envData = random.sample(range(0, 10000),3)
    sensor = BME280(mode=BME280_OSAMPLE_8)

    degrees = sensor.read_temperature()
    pascals = sensor.read_pressure()
    hectopascals = pascals / 100
    humidity = sensor.read_humidity()

    print 'Timestamp = {0:0.3f}'.format(sensor.t_fine)
    print 'Temp      = {0:0.3f} deg C'.format(degrees)
    print 'Pressure  = {0:0.2f} hPa'.format(hectopascals)
    print 'Humidity  = {0:0.2f} %'.format(humidity)

    envData = [degrees, humidity, hectopascals]
    return envData





def sendMapData(roomData, position):
    print 'sendMapData'





def sendEnvData(envData, position):
    print 'sendEnvData'





def nextPosDec(roomData, position):
    print 'nextPosDec'





def moveRobot(newPosition, position):
    print 'moveRobot'



#######################
## Run the program:

mobileThermostat()