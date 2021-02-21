import time
import serial
import matplotlib.pyplot as plt
import numpy as np

x = np.linspace(0, 5, 100)
y = np.cos(x)*5

plt.ion()

figure, ax = plt.subplots(figsize=(8, 6))
line1, = ax.plot(x, y)
line2, =ax.plot(x,y)

plt.title("Dynamic Plot", fontsize=25)

plt.xlabel("X", fontsize=18)
plt.ylabel("afstand", fontsize=18)

try:
    arduino = serial.Serial("COM5",115200)
except:
    print("please check the port")
while(True):
    info = str(arduino.readline())
    print(info)
    gesplitst = info.split("_")
    eerste = gesplitst[0][2:]
    tweede = gesplitst[1][:4]
    eersteGetal = float(eerste)
    tweedeGetal = float(tweede)
    print("eerste: ")
    print(eersteGetal)
    print("tweede: ")
    print(tweedeGetal)
    tweedeGetal = 2.60 - tweedeGetal


    line1.set_xdata(x)
    line1.set_ydata(eersteGetal)
    line2.set_ydata(tweedeGetal)

    figure.canvas.draw()

    figure.canvas.flush_events()
    time.sleep(0.1)
