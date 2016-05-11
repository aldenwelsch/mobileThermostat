# Plot map from data points and environmental data

import matplotlib.pyplot as plt
import time
import random
plt.ion()
class DynamicUpdate():

    envData = [0, 0, 0]
    dataText = ""
    txt = None

    def set_text(self, pos, envData):
        self.dataText = "\n \n" + "Robot Position: (" + str(pos[0]) + "," + str(pos[1]) + ")\n" + "Temperature: " + str(self.envData[0]) + " deg C     Humidity: " + str(self.envData[1]) + " %      Pressure: " + str(self.envData[2]) + " hPa \n"

    def on_launch(self):
        #Set up plot
        self.figure, self.ax = plt.subplots()
        self.lines, = self.ax.plot([],[], 'bo')
        self.robot, = self.ax.plot(0,0, 'ro')
        # Set limits to +-256 since prox sensor doesn't go past 255in and rooms are generally smaller than 255in in any direction
        self.ax.set_xlim(-256, 256)
        self.ax.set_ylim(-256, 256)
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.grid()
        #self.ax.legend(loc='upper center')
        self.txt = self.figure.text(0, -0.01, self.dataText)
        self.figure.set_size_inches(10,10, forward=True)

    def on_running(self, xdata, ydata, pos, envData):
        #Update data (with the new _and_ the old points)
        self.lines.set_xdata(xdata)
        self.lines.set_ydata(ydata)
        self.robot.set_xdata(pos[0])
        self.robot.set_ydata(pos[1])
        self.envData = envData
        self.set_text(pos, envData)
        self.txt.remove()
        self.txt = self.figure.text(0, -0.015, self.dataText)
        #Need both of these in order to rescale
        #self.ax.relim()
        #self.ax.autoscale_view()
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()



# Create instance of the object
d = DynamicUpdate()
# Initialize it
d.on_launch()
while 1:
    try:
        # There will be a serial/bluetooth listener here that will collect data when it comes over the serial
        x = random.sample(range(-255, 255), 10)
        y = random.sample(range(-255, 255), 10)
        posx = random.sample(range(-255, 255), 1)
        posy = random.sample(range(-255, 255), 1)
        pos = [posx[0], posy[0]]
        envData = random.sample(range(0, 10000),3)
        d.on_running(x, y, pos, envData)
        time.sleep(1)
    except KeyboardInterrupt:
        print 'exiting...'
        break

