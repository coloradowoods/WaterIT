# main.py - a script for making a plant watering thing, running using a Raspberry Pi Pico
# First prototype is using an OLED, rotary encoder and a relay switch (linked to water pump device of some sort)
# The display uses drivers made by Peter Hinch [link](https://github.com/peterhinch/micropython-nano-gui)

# Released under the GPL 3.0

# Fonts for Writer (generated using https://github.com/peterhinch/micropython-font-to-py)
import gui.fonts.freesans20 as freesans20
import gui.fonts.quantico40 as quantico40
from gui.core.writer import CWriter
from gui.core.nanogui import refresh
import utime
from machine import Pin, I2C, SPI, ADC, reset
#from rp2 import PIO, StateMachine, asm_pio
import sys
import math
import gc
from drivers.ssd1351.ssd1351_16bit import SSD1351 as SSD
import uasyncio as asyncio
from primitives.pushbutton import Pushbutton


def save(level1, level2, level3, level4):
    file = open("level.csv", "w")
    file.write(str(level1)+","+str(level2)+","+str(level3)+","+str(level4))
    file.close()
def load():
    file = open("level.csv", "r")
    data = file.readline()
    file.close()
    return data

def splash(string):
    wri = CWriter(ssd, freesans20, fgcolor=SSD.rgb(
        50, 50, 0), bgcolor=0, verbose=False)
    CWriter.set_textpos(ssd, 90, 25)
    wri.printstring('InnoGraft')
    ssd.show()
    utime.sleep(.3)
    for x in range(10):
        wri = CWriter(ssd, freesans20, fgcolor=SSD.rgb(
            25*x, 25*x, 25*x), bgcolor=0, verbose=False)
        CWriter.set_textpos(ssd, 55, 25)
        wri.printstring(string)
        wri = CWriter(ssd, freesans20, fgcolor=SSD.rgb(
            50-x, 50-x, 0), bgcolor=0, verbose=False)
        CWriter.set_textpos(ssd, 90, 25)
        wri.printstring('InnoGraft')
        ssd.show()
    utime.sleep(2)
    for x in range(10, 0, -1):
        wri = CWriter(ssd, freesans20, fgcolor=SSD.rgb(
            25*x, 25*x, 25*x), bgcolor=0, verbose=False)
        CWriter.set_textpos(ssd, 55, 25)
        wri.printstring(string)
        wri = CWriter(ssd, freesans20, fgcolor=SSD.rgb(
            50-x, 50-x, 0), bgcolor=0, verbose=False)
        CWriter.set_textpos(ssd, 90, 25)
        wri.printstring('InnoGraft')
        ssd.show()
    wri = CWriter(ssd, freesans20, fgcolor=SSD.rgb(
        50, 50, 0), bgcolor=0, verbose=False)
    CWriter.set_textpos(ssd, 90, 25)
    wri.printstring('InnoGraft')
    ssd.show()
    utime.sleep(.3)
    return
# function for short button press - currently just a placeholder
def button():
    print('Button short press: Boop')
    return

# function for long button press - currently just a placeholder


def buttonlong():
    print('Button long press: Reset')
    return

# Screen to display on OLED during heating


def displaynum(channel, num, value):
    # This needs to be fast for nice responsive increments
    # 100 increments?
    ssd.fill(0)
    delta = num-value
    text = SSD.rgb(0, 255, 0)
    if delta >= .5:
        text = SSD.rgb(165, 42, 42)
    if delta <= -.5:
        text = SSD.rgb(0, 255, 255)
    wri = CWriter(ssd, quantico40, fgcolor=text, bgcolor=0, verbose=False)
    # verbose = False to suppress console output
    CWriter.set_textpos(ssd, 50, 0)
    wri.printstring(str("{:.0f}".format(num)))
    wrimem = CWriter(ssd, freesans20, fgcolor=SSD.rgb(
        255, 255, 255), bgcolor=0, verbose=False)
    CWriter.set_textpos(ssd, 100, 0)
    wrimem.printstring('now at: '+str("{:.0f}".format(value))+"/ 10")
    CWriter.set_textpos(ssd, 0, 0)
    wrimem = CWriter(ssd, freesans20, fgcolor=SSD.rgb(
        155, 155, 155), bgcolor=0)
    wrimem.printstring('Channel: '+str("{:.0f}".format(channel)))
    CWriter.set_textpos(ssd, 20, 0)
    wrimem.printstring('Target:')
    ssd.show()
    return


def beanaproblem(string):
    refresh(ssd, True)  # Clear any prior image
    wri = CWriter(ssd, freesans20, fgcolor=SSD.rgb(
        250, 250, 250), bgcolor=0, verbose=False)
    CWriter.set_textpos(ssd, 55, 25)
    wri.printstring(string)
    ssd.show()
    relaypin = Pin(15, mode=Pin.OUT, value=0)
    utime.sleep(2)


# Setup display
height = 128
pdc = Pin(20, Pin.OUT, value=0)
pcs = Pin(17, Pin.OUT, value=1)
prst = Pin(21, Pin.OUT, value=1)
spi = SPI(0,
          baudrate=10000000,
          polarity=1,
          phase=1,
          bits=8,
          firstbit=SPI.MSB,
          sck=Pin(18),
          mosi=Pin(19),
          miso=Pin(16))
gc.collect()  # Precaution before instantiating framebuf

ssd = SSD(spi, pcs, pdc, prst, height)  # Create a display instance

splash("WaterIt")

# Define relay and LED pins

# Onboard led on GPIO 25, not currently used, but who doesnt love a controllable led?
ledPin = Pin(25, mode=Pin.OUT, value=0)


class Encoder:
    def __init__(self, clk, dt, sw, min, max):
        self.clk = clk
        self.dt = dt
        self.sw = sw
        self.min = min
        self.max = max
        # define class variables
        self.counter = 0   # counter updates when encoder rotates
        self.direction = ""  # empty string for registering direction change
        self.outA_last = 0  # registers the last state of outA pin / CLK pin
        self.outA_current = 0  # registers the current state of outA pin / CLK pin
        # define encoder pins
        self.btn = Pin(self.sw, Pin.IN, Pin.PULL_UP)  # Adapt for your hardware
        self.pb = Pushbutton(self.btn, suppress=True)
        self.outA = Pin(self.clk, mode=Pin.IN)  # Pin CLK of encoder
        self.outB = Pin(self.dt, mode=Pin.IN)  # Pin DT of encoder
        # attach interrupt to the outA pin ( CLK pin of encoder module )
        self.outA.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
         handler=self.encoder)
        # attach interrupt to the outB pin ( DT pin of encoder module )
        self.outB.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
                 handler=self.encoder)
    def encoder(cls, pin):
        # read the value of current state of outA pin / CLK pin
        try:
            cls.outA = Pin(cls.clk, mode=Pin.IN)  # Pin CLK of encoder
            cls.outA_current = cls.outA.value()
        except:
            print('outA not defined')
            cls.outA_current = 0
            cls.outA_last = 0
        # if current state is not same as the last stare , encoder has rotated
        if cls.outA_current != cls.outA_last:
            # read outB pin/ DT pin
            # if DT value is not equal to CLK value
            # rotation is clockwise [or Counterclockwise ---> sensor dependent]
            if cls.outB.value() != cls.outA_current:
                cls.counter += .5
            else:
                cls.counter -= .5

            # print the data on screen
            print("Class Counter : ", cls.counter, "     |   Direction : ",cls.direction)
            print("\n")

        # update the last state of outA pin / CLK pin with the current state
        cls.outA_last = cls.outA_current
        cls.counter = min(cls.max, cls.counter)
        cls.counter = max(cls.min, cls.counter)
        return(cls.counter)

class Channel:
    def __init__(self, sensor, relay, calibratewet, calibratedry, startlevel):
        self.sensor = sensor
        self.relay = relay
        self.calibratewet = calibratewet
        self.calibratedry = calibratedry
        self.wetness = ADC(self.sensor)
        self.integral = 0
        self.derivative = 0
        self.error = 0
        self.lasterror = 0
        # In terms of steering a sailboat:
        # Kp is steering harder the further off course you are,
        # Ki is steering into the wind to counteract a drift
        # Kd is slowing the turn as you approach your course
        # Proportional term - Basic steering (This is the first parameter you should tune for a particular setup)
        self.Kp = 2
        self.Ki = 0   # Integral term - Compensate for heat loss by vessel
        self.Kd = 0  # Derivative term - to prevent overshoot due to inertia - if it is zooming towards setpoint this
        # will cancel out the proportional term due to the large negative gradient
        self.output = 0
        self.offstate = True
        self.level = int(startlevel)
        relaypin = Pin(self.relay, mode=Pin.OUT, value=1)
    def read(cls):
        # Get wetness
        cls.imwet = cls.wetness.read_u16()
        cls.howdry = min(10, max(0, 10*(cls.imwet-cls.calibratedry) / 
                             (cls.calibratewet-cls.calibratedry)))
        #print("raw:"+str(cls.imwet)+" normalized:"+str(cls.howdry))
        return cls.howdry
    def pid(cls, dt):
        cls.error = cls.level-cls.howdry
        cls.integral = cls.integral + dt * cls.error
        cls.derivative = (cls.error - cls.lasterror)/dt
        cls.output = cls.Kp * cls.error + cls.Ki * cls.integral + cls.Kd * cls.derivative
        print(str(cls.output)+"= Kp term: "+str(cls.Kp*cls.error)+" + Ki term:" +
              str(cls.Ki*cls.integral) + "+ Kd term: " + str(cls.Kd*cls.derivative))
        # Clamp output between 0 and 100
        cls.output = max(min(100, cls.output), 0)
        #print(cls.output)
        if cls.output > 0:
            print('ON')
            relaypin = Pin(cls.relay, mode=Pin.OUT, value=0)
            cls.offstate = False
        else:
            print('OFF')
            relaypin = Pin(cls.relay, mode=Pin.OUT, value=1)
            cls.offstate = True
        cls.lasterror = cls.error
        


# Main Logic

async def main():
    # The Tweakable values that will help tune for our use case. TODO: Make accessible via menu on OLED
    calibratewet = 20000  # ADC value for a very wet thing
    calibratedry = 50000  # ADC value for a very dry thing
    checkin = 5

    # Setup Level Encoder
    level = Encoder(2,3,4,0,9)
    
    #Setup Chsnnrl Selector
    selector = Encoder(6,7,8,1,3)
    
    short_press = level.pb.release_func(button, ())
    long_press = level.pb.long_func(buttonlong, ())
    
    #local variables
    pin = 0
    ##integral = 0
    lastupdate = utime.time()
    refresh(ssd, True)  # Initialise and clear display.
    # Load levels for file
    data = str.split(load(), ',')
    
    channel = []
    channel.append(Channel(26, 15, calibratewet, calibratedry, data[0]))
    channel.append(Channel(27, 14, calibratewet, calibratedry, data[1]))
    channel.append(Channel(28, 13, calibratewet, calibratedry, data[2]))
    #channel.append(Channel(22, 12, calibratewet, calibratedry))
    data = str.split(load(), ',')
    print(data)
    channel[0].read()
    channel[1].read()
    channel[2].read()
    #channel[3].read()
    lastchannel = -1
    
    # PID loop - Default behaviour
    powerup = True
    while True:
        if powerup:
            try:
                counter = level.encoder(0)
                currentchannel = int(selector.encoder(0))
                if lastchannel != currentchannel:
                    lastchannel = currentchannel
                    counter = channel[currentchannel - 1].level
                    level.counter = counter
                if channel[currentchannel - 1].level != counter:
                    channel[currentchannel - 1].level = counter
                    save(channel[0].level, channel[1].level, channel[2].level, 0)
                # Get wetness
                channel[0].read()
                displaynum(currentchannel, counter, float(channel[currentchannel - 1].howdry))
                now = utime.time()
                dt = now-lastupdate
                if dt > checkin:
                    channel[0].pid(dt)
                    utime.sleep(.1)
                    lastupdate = now

            except Exception as e:
                # Put something to output to OLED screen
                beanaproblem('error.')
                print('error encountered:'+str(e))
                utime.sleep(checkin)
        else:
            refresh(ssd, True)  # Clear any prior image
            relaypin = Pin(15, mode=Pin.OUT, value=0)
        await asyncio.sleep(.01)

asyncio.run(main())