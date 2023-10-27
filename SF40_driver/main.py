# --------------------------------------------------------------------------------------------------------------
# PPER 2023
# --------------------------------------------------------------------------------------------------------------
# Description:
#   This script communicates with the SF-40 360-degree Lidar.
#
# Notes:
# 	Requires the pySerial module.
# --------------------------------------------------------------------------------------------------------------

import serial
import numpy as np
from PyQt6 import QtCore, QtWidgets
import pyqtgraph as pg
import threading
from processPacket import Packet



# --------------------------------------------------------------------------------------------------------------
# LWNX library functions.
# --------------------------------------------------------------------------------------------------------------
packetParseState = 0
packetPayloadSize = 0
packetSize = 0
packetData = []

responseStream = []
pointCloud = []
prevCloud = []

xRev = []
yRev = []
zRev = []


def output_init():
    with open('scanCloud.xyz', 'w') as f:
        f.write('')



def update():
    print('Running LWNX sample.')

    # Make a connection to the com port.
    serialPortName = '/dev/tty.usbserial-0001'
    serialPortBaudRate = 921600
    port = serial.Serial(serialPortName, serialPortBaudRate, timeout=0.1)

    packet = Packet()

    # Get product information.
    packet.execute_command(port, 0, 0, [], timeout=1)
    print('Product: ' + packet.read_str_16())

    packet.execute_command(port, 3, 0, [], timeout=1)
    print('Serial Number: ' + packet.read_str_16())

    # Set output rate to 22010 points per second
    packet.execute_command(port, 108, 1, [0], timeout=1)

    packet.execute_command(port, 109, 1, [0, 0], timeout=1)

    # Write Stream command [30] to the SF-40.
    packet.execute_command(port, 30, 1, [3, 0, 0, 0])

    # Write distance output command [48].
    packet.execute_command(port, 48, 0, [], timeout=1)
    global responseStream
    responseStream = packet.wait_for_packet(port, 48, timeout=10)

    # Initialize output .txt files
    output_init()
    xWork = []
    yWork = []
    # zWork = []  # TODO: implement 3D scan.

    # Loop to continuously receive data from LiDAR
    while True:
        stream = packet.wait_for_packet(port, 48, timeout=10)

        if stream is not None:
            (alarmState, pointsPerSec, forwardOffset, motorVoltage, revIndex, pointTotal, pointCount, pointStartIndex,
             pointDistances) = packet.read_signal_data()

            print('Alarms: {}  PPS: {}  Offset: {}  Motor Voltage: {}  Rev Index: {}  Point Total: {}  '
                  'Point Count : {}  Point Start Index: {}'
                  '  Point Distances: {} '.format(bin(alarmState), pointsPerSec, forwardOffset,
                                                  motorVoltage, revIndex, pointTotal, pointCount, pointStartIndex,
                                                  pointDistances)
                  )

            alarmState = bin(alarmState)

            if alarmState != '0b0':
                if alarmState[3] == '1':  # Check alarmState for zone 1 (2.5 m; 40 deg range centered at 0 degrees)
                    # print('Obstacle within 2.5 m of drone!')
                    pass  # frontAlarm = 1
                    # TODO: Send command to flight controller to avoid obstacle.
                elif alarmState[4]:  # Check alarmState for zone 2 (1.5 m; 70 deg range centered at 55 degrees)
                    # print('Obstacle within 1.5 m of drone!')
                    pass  # sideAlarm1 = 1
                    # TODO: Send command to flight controller to move right or left (double check how lidar is mounted)
                elif alarmState[9]:  # Check alarmState for zone 6 (1.5 m; 70 deg range centered at 305 degrees)
                    # print('Obstacle within 1.5 m of drone!')
                    pass  # sideAlarm2 = 1
                    # TODO: Send command to flight controller to move right or left (double check how lidar is mounted)

            # Get angles in degrees and distances in cm
            [angles, distances] = packet.get_polar_distance_data()




            x = distances * np.cos(np.radians(angles))
            x = -x  # Mirror the x-axis if the Lidar is mounted on top
            y = distances * np.sin(np.radians(angles))
            z = 0  # TODO: incorporate z-direction for 3d scan

            global xRev
            global yRev

            if len(xWork) > 3570:
                [xRev, yRev] = [xWork, yWork]
                xWork.clear()
                yWork.clear()

            xWork.extend(x)
            yWork.extend(y)

            # Write to output file
            with open('scanCloud.xyz', 'a') as af:
                i = 0
                while i < len(x):
                    af.write('{} {} {}\n'.format(x[i], y[i], z))
                    i += 1

        else:
            packet.execute_command(port, 30, 1, [0, 0, 0, 0])
            port.close()
            break


class MyWidget(pg.GraphicsLayoutWidget):
    def __init__(self, parent=None):
        super().__init__(parent=parent)

        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)
        self.viewport().setAttribute(QtCore.Qt.WidgetAttribute.WA_AcceptTouchEvents, False)

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(50)  # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.onNewData)

        self.plotItem = self.addPlot(title="Lidar points")
        self.plotItem.vb.setXRange(-500, 500)
        self.plotItem.vb.setYRange(-500, 500)
        # self.plotItem.plot.setData(xRev,yRev)

        self.plotDataItem = self.plotItem.plot([], pen=None,
                                               symbolBrush=(255, 0, 0), symbolSize=5, symbolPen=None)

    def setData(self, xx, yy):
        self.plotDataItem.setData(xx, yy)

    def onNewData(self):
        xx = xRev
        yy = yRev
        self.setData(xx, yy)

    def clearData(self):
        self.plotDataItem.clear()


# --------------------------------------------------------------------------------------------------------------
# Main application.
# --------------------------------------------------------------------------------------------------------------
def main():

    updateThread = threading.Thread(target=update, name='updateThread')
    updateThread.start()

    # Initialize viewer widget
    app = QtWidgets.QApplication([])
    pg.setConfigOptions(antialias=False)  # True seems to work as well

    win = MyWidget()

    win.show()
    win.resize(800, 600)
    win.raise_()
    app.exec()

    updateThread.join()


if __name__ == "__main__":
    main()
