from PyQt5 import QtWidgets, QtSerialPort, QtCore, QtGui
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import threading
import numpy as np

from ui import mainWindow
import sys


class hostApp(mainWindow.Ui_MainWindow, QtWidgets.QMainWindow):
    def __init__(self):
        super(hostApp, self).__init__()
        self.setupUi(self)
        self.setWindowTitle("BLDC control App")

        self.pb_searchPort.clicked.connect(self.searchButtonClicked)
        self.pb_connect.clicked.connect(self.connectButtonClicked)
        self.pb_start.clicked.connect(self.startButtonClicked)
        self.chb_iacurr.clicked.connect(self.chb_currentAdata_clicked)
        self.chb_ibcurr.clicked.connect(self.chb_currentBdata_clicked)
        self.chb_iccurr.clicked.connect(self.chb_currentCdata_clicked)
        self.chb_currentIq.clicked.connect(self.chb_currentIq_clicked)
        self.chb_currentId.clicked.connect(self.chb_currentId_clicked)
        self.chb_desiredIq.clicked.connect(self.chb_desiredIq_clicked)
        self.chb_currentPos.clicked.connect(self.chb_currentPos_clicked)
        self.chb_desiredPos.clicked.connect(self.chb_desiredPos_clicked)
        self.chb_currentSpeed.clicked.connect(self.chb_currentSpeed_clicked)
        self.chb_desiredSpeed.clicked.connect(self.chb_desiredSpeed_clicked)
        self.chb_currentIab.clicked.connect(self.chb_currentIab_clicked)
        self.chb_currentUab.clicked.connect(self.chb_currentUab_clicked)
        self.chb_currentUdq.clicked.connect(self.chb_currentUdq_clicked)
        self.pb_clearData.clicked.connect(self.pb_clearData_clicked)
        self.serialConnected = 0
        self.graphWidget.setBackground('w')

        # self.receiveProcessHandler = threading.Thread(target=self.serialReceiveThread, args=())
        # self.updatePlotHandler = threading.Thread(target=self.updatePlotThread, args=())
        self.stopReceivingProcessEvent = threading.Event()
        self.stopUpdatingPlotEvent = threading.Event()
        self.updatePlotTimer = QtCore.QTimer(self)
        self.updatePlotTimer.setInterval(20)
        self.updatePlotTimer.timeout.connect(self.updatePlot)
        self.graphWidget.showGrid(x = True, y = True, alpha = 0.3)
        self.runningActive = 0
        self.currentAdataLine = pg.PlotCurveItem(pen=pg.mkPen({'color': "FF5733"}), width=10, name="current Ia")
        self.chb_iacurr.setStyleSheet("QCheckBox {background: #FF5733;}")
        self.currentBdataLine = pg.PlotCurveItem(pen=pg.mkPen({'color': "2418E3"}), width = 10)
        self.chb_ibcurr.setStyleSheet("QCheckBox {background:#2418E3 }")
        self.currentCdataLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "C118E3"}), width = 2)
        self.chb_iccurr.setStyleSheet("QCheckBox {background:#C118E3}")
        self.currentIdLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "18BE3E"}), width = 2)
        self.chb_currentIq.setStyleSheet("QCheckBox {background:#18BE3E}")
        self.currentIqLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "44B68C"}), width = 2)
        self.chb_currentId.setStyleSheet("QCheckBox {background:#44B68C}")
        self.desiredIqLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "5B3DCA"}), width = 2)
        self.chb_desiredIq.setStyleSheet("QCheckBox {background:#5B3DCA}")
        self.currentPositionLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "C1B717"}), width = 2)
        self.chb_currentPos.setStyleSheet("QCheckBox {background:#C1B717}")
        self.desiredPositionLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "E70BB1"}), width = 2)
        self.chb_desiredPos.setStyleSheet("QCheckBox {background:#E70BB1}")
        self.currentSpeedLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "A25DDC"}), width = 2)
        self.chb_currentSpeed.setStyleSheet("QCheckBox {background:#A25DDC}")
        self.desiredSpeedLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "55C00F"}), width = 2)
        self.chb_desiredSpeed.setStyleSheet("QCheckBox {background:#55C00F}")
        self.currentIabLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "0BBFBD"}), width = 2)
        self.chb_currentIab.setStyleSheet("QCheckBox {background:#0BBFBD}")
        self.currentUabLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "47D562"}), width = 2)
        self.chb_currentUab.setStyleSheet("QCheckBox {background:#47D562}")
        self.currentUdqLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "C01BB8"}), width = 2)
        self.chb_currentUdq.setStyleSheet("QCheckBox {background:#C01BB8}")

        self.serialData = QtCore.QByteArray()


    def searchButtonClicked(self):
        for ind in range(self.cb_ports.__len__()):
            self.cb_ports.removeItem(ind)
        for port in QtSerialPort.QSerialPortInfo.availablePorts():
            self.cb_ports.addItem(port.portName())
        if(self.cb_ports.__len__()):
            self.lb_status.setText(str(self.cb_ports.__len__()) + " port(s) found")
            self.lb_status.setStyleSheet("QLabel {color : green; }")
            self.pb_connect.setEnabled(1)
        else:
            self.lb_status.setText("No ports were found")
            self.lb_status.setStyleSheet("QLabel {color : red; }")
            self.pb_connect.setEnabled(0)


    def connectButtonClicked(self):
        if(self.serialConnected == 0):
            selectedPort = self.cb_ports.currentText()
            self.serialPort = QtSerialPort.QSerialPort(selectedPort,
                                                baudRate=QtSerialPort.QSerialPort.Baud115200,
                                               readyRead=self.serialReceiveThread)
            try:
                self.serialPort.open(QtCore.QIODevice.ReadWrite)
            except:
                self.lb_status.setText("Can't open " + selectedPort)
                self.lb_status.setStyleSheet("QLabel {color : red; }")

            if(self.serialPort.isOpen()):
                self.lb_status.setText(selectedPort + " opened")
                self.lb_status.setStyleSheet("QLabel {color : green; }")
                self.serialConnected = 1
                self.pb_connect.setText("Disconnect")
                self.pb_start.setEnabled(1)
                plotsize = int(self.le_plotPoints.text())
                self.currentAdata = np.zeros([plotsize])
                self.currentBdata = np.zeros([plotsize])
                self.currentCdata = np.zeros([plotsize])
                self.currentCdataRaw = np.zeros([plotsize])
                self.currentAbsoluteAngle = np.zeros([plotsize])
                self.currentSpeed = np.zeros([plotsize])
                self.desireSpeed = np.zeros([plotsize])
                self.currentIq = np.zeros([plotsize])
                self.currentId = np.zeros([plotsize])
                self.desiredIq = np.zeros([plotsize])
                self.desiredPosition = np.zeros([plotsize])
                self.currentPosition = np.zeros([plotsize])
                self.desiredSpeed = np.zeros([plotsize])
                self.currentSpeed = np.zeros([plotsize])
                self.currentIab = np.zeros([plotsize])
                self.currentUab = np.zeros([plotsize])
                self.currentUdq = np.zeros([plotsize])
            else:
                self.lb_status.setText("Can't open " + selectedPort)
                self.lb_status.setStyleSheet("QLabel {color : red; }")
        else:
            self.serialConnected = 0
            self.pb_start.setEnabled(0)
            self.pb_connect.setText("Connect")
            self.lb_status.setText(self.cb_ports.currentText() + " closed")
            self.lb_status.setStyleSheet("QLabel {color : green; }")
            self.serialPort.close()

    def serialReceiveThread(self):
        if(self.serialPort.bytesAvailable()):
            if(self.runningActive == 1):
                ind = 0
                self.serialData = self.serialData.append(self.serialPort.readAll())
                code = b'H'
                for byte in self.serialData:
                    if(byte == code):
                        break
                    ind = ind + 1

                self.serialData.remove(0, ind)
                while(self.serialData.size() >= 9):
                    posData = int.from_bytes(self.serialData[1], "little")*256 + int.from_bytes(self.serialData[2], "little")
                    currA  = int.from_bytes(self.serialData[3], "little")*256 + int.from_bytes(self.serialData[4], "little")
                    currB = int.from_bytes(self.serialData[5], "little")*256 + int.from_bytes(self.serialData[6], "little")
                    currC = int.from_bytes(self.serialData[7], "little")*256 + int.from_bytes(self.serialData[8], "little")
                    self.serialData.remove(0, 9)
                    self.currentAdata = np.append(self.currentAdata, currA)
                    self.currentBdata = np.append(self.currentBdata, currB)
                    self.currentCdata = np.append(self.currentCdata, currC)
                    self.currentPosition = np.append(self.currentPosition, posData)
                    self.currentAdata = self.currentAdata[1:]
                    self.currentBdata = self.currentBdata[1:]
                    self.currentCdata = self.currentCdata[1:]
                    self.currentPosition = self.currentPosition[1:]

            else:
                self.serialPort.readAll()



    def updatePlot(self):
        if self.chb_iacurr.isChecked():
            self.currentAdataLine.setData(self.currentAdata)
        if self.chb_ibcurr.isChecked():
            self.currentBdataLine.setData(self.currentBdata)
        if self.chb_iccurr.isChecked():
            self.currentCdataLine.setData(self.currentCdata)
        if self.chb_currentIq.isChecked():
            self.currentIqLine.setData(self.currentIq)
        if self.chb_currentId.isChecked():
            self.currentIdLine.setData(self.currentId)
        if self.chb_desiredIq.isChecked():
            self.desiredIqine.setData(self.desiredIq)
        if self.chb_currentPos.isChecked():
            self.currentPositionLine.setData(self.currentPosition)
        if self.chb_desiredPos.isChecked():
            self.desiredPositionLine.setData(self.desiredPosition)
        if self.chb_currentSpeed.isChecked():
            self.currentSpeedLine.setData(self.desiredSpeed)
        if self.chb_desiredSpeed.isChecked():
            self.desiredSpeedLine.setData(self.currentSpeed)
        if self.chb_currentIab.isChecked():
            self.currentIabLine.setData(self.currentIab)
        if self.chb_currentUab.isChecked():
            self.currentUabLine.setData(self.currentUab)
        if self.chb_currentUdq.isChecked():
            self.currentUdqLine.setData(self.currentUdq)

        # redraw data lines

    def startButtonClicked(self):
        if(self.runningActive == 0):
            # data arrays
            plotsize = int(self.le_plotPoints.text())

            if(self.currentAdata.shape[0] < plotsize):
                self.currentAdata = np.append(self.currentAdata, np.zeros([plotsize-self.currentAdata.shape[0]]))
            if(self.currentBdata.shape[0] < plotsize):
                self.currentBdata = np.append(self.currentBdata, np.zeros([plotsize-self.currentBdata.shape[0]]))
            if(self.currentCdata.shape[0] < plotsize):
                self.currentCdata = np.append(self.currentCdata, np.zeros([plotsize-self.currentCdata.shape[0]]))
            if(self.currentIq.shape[0] < plotsize):
                self.currentIq = np.append(self.currentIq, np.zeros([plotsize-self.currentIq.shape[0]]))
            if(self.currentId.shape[0] < plotsize):
                self.currentId = np.append(self.currentId, np.zeros([plotsize-self.currentId.shape[0]]))
            if(self.desiredIq.shape[0] < plotsize):
                self.desiredIq = np.append(self.desiredIq, np.zeros([plotsize-self.desiredIq.shape[0]]))
            if(self.currentPosition.shape[0] < plotsize):
                self.currentPosition = np.append(self.currentPosition, np.zeros([plotsize-self.currentPosition.shape[0]]))
            if(self.desiredPosition.shape[0] < plotsize):
                self.desiredPosition = np.append(self.desiredPosition, np.zeros([plotsize-self.desiredPosition.shape[0]]))
            if(self.desiredSpeed.shape[0] < plotsize):
                self.desiredSpeed = np.append(self.desiredSpeed, np.zeros([plotsize-self.desiredSpeed.shape[0]]))
            if(self.currentSpeed.shape[0] < plotsize):
                self.currentSpeed = np.append(self.currentSpeed, np.zeros([plotsize-self.currentSpeed.shape[0]]))
            if(self.currentIab.shape[0] < plotsize):
                self.currentIab = np.append(self.currentIab, np.zeros([plotsize-self.currentIab.shape[0]]))
            if(self.currentUab.shape[0] < plotsize):
                self.currentUab = np.append(self.currentUab, np.zeros([plotsize-self.currentUab.shape[0]]))
            if(self.currentUdq.shape[0] < plotsize):
                self.currentUdq = np.append(self.currentUdq, np.zeros([plotsize-self.currentUdq.shape[0]]))


            self.runningActive = 1
            self.pb_start.setText('Stop')
            self.le_plotPoints.setDisabled(1)
            self.graphWidget.setXRange(0, plotsize, padding=0)
            # self.graphWidget.clear()
            self.updatePlotTimer.start()
        else:
            self.runningActive = 0
            self.pb_start.setText('Start')
            self.updatePlotTimer.stop()
            self.le_plotPoints.setDisabled(0)

    def redrawLines(self):
        if self.chb_iacurr.isChecked():
            self.graphWidget.addItem(self.currentAdataLine)
        if self.chb_ibcurr.isChecked():
            self.graphWidget.addItem(self.currentBdataLine)
        if self.chb_iccurr.isChecked():
            self.graphWidget.addItem(self.currentCdataLine)
        if self.chb_currentIq.isChecked():
            self.graphWidget.addItem(self.currentIqLine)
        if self.chb_currentId.isChecked():
            self.graphWidget.addItem(self.currentIdLine)
        if self.chb_desiredIq.isChecked():
            self.graphWidget.addItem(self.desiredIqine)
        if self.chb_currentPos.isChecked():
            self.graphWidget.addItem(self.currentPositionLine)
        if self.chb_desiredPos.isChecked():
            self.graphWidget.addItem(self.desiredPositionLine)
        if self.chb_currentSpeed.isChecked():
            self.graphWidget.addItem(self.currentSpeedLine)
        if self.chb_desiredSpeed.isChecked():
            self.graphWidget.addItem(self.desiredSpeedLine)
        if self.chb_currentIab.isChecked():
            self.graphWidget.addItem(self.currentIabLine)
        if self.chb_currentUab.isChecked():
            self.graphWidget.addItem(self.currentUabLine)
        if self.chb_currentUdq.isChecked():
            self.graphWidget.addItem(self.currentUdqLine)

    def chb_currentAdata_clicked(self):
        if(self.chb_iacurr.isChecked()):
            self.graphWidget.addItem(self.currentAdataLine)
        else:
            self.graphWidget.removeItem(self.currentAdataLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_currentBdata_clicked(self):
        if(self.chb_ibcurr.isChecked()):
            self.graphWidget.addItem(self.currentBdataLine)
        else:
            self.graphWidget.removeItem(self.currentBdataLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_currentCdata_clicked(self):
        if(self.chb_iccurr.isChecked()):
            self.graphWidget.addItem(self.currentCdataLine)
        else:
            self.graphWidget.removeItem(self.currentCdataLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_currentIq_clicked(self):
        if(self.chb_currentIq.isChecked()):
            self.graphWidget.addItem(self.currentIqLine)
        else:
            self.graphWidget.removeItem(self.currentIqLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_currentId_clicked(self):
        if(self.chb_currentIds.isChecked()):
            self.graphWidget.addItem(self.currentIdLine)
        else:
            self.graphWidget.removeItem(self.currentIdLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_desiredIq_clicked(self):
        if(self.chb_desiredIq.isChecked()):
            self.graphWidget.addItem(self.desiredIqLine)
        else:
            self.graphWidget.removeItem(self.desiredIqLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_currentPos_clicked(self):
        if(self.chb_currentPos.isChecked()):
            self.graphWidget.addItem(self.currentPositionLine)
        else:
            self.graphWidget.removeItem(self.currentPositionLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_desiredPos_clicked(self):
        if(self.chb_desiredPos.isChecked()):
            self.graphWidget.addItem(self.desiredPositionLine)
        else:
            self.graphWidget.removeItem(self.desiredPositionLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_currentSpeed_clicked(self):
        if(self.chb_currentSpeed.isChecked()):
            self.graphWidget.addItem(self.currentSpeedLine)
        else:
            self.graphWidget.removeItem(self.currentSpeedLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_desiredSpeed_clicked(self):
        if(self.chb_desiredSpeed.isChecked()):
            self.graphWidget.addItem(self.desiredSpeedLine)
        else:
            self.graphWidget.removeItem(self.desiredSpeedLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_currentIab_clicked(self):
        if (self.chb_currentIab.isChecked()):
            self.graphWidget.addItem(self.currentIabLine)
        else:
            self.graphWidget.removeItem(self.currentIabLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_currentUab_clicked(self):
        if (self.chb_currentUab.isChecked()):
            self.graphWidget.addItem(self.currentUabLine)
        else:
            self.graphWidget.removeItem(self.currentUabLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_currentUdq_clicked(self):
        if (self.chb_currentUdq.isChecked()):
            self.graphWidget.addItem(self.currentUdqLine)
        else:
            self.graphWidget.removeItem(self.currentUdqLine)
            self.graphWidget.clear()
            self.redrawLines()

    def pb_clearData_clicked(self):
        plotsize = int(self.le_plotPoints.text())
        self.currentAdata = np.zeros([plotsize])
        self.currentBdata = np.zeros([plotsize])
        self.currentCdata = np.zeros([plotsize])
        self.currentCdataRaw = np.zeros([plotsize])
        self.currentAbsoluteAngle = np.zeros([plotsize])
        self.currentSpeed = np.zeros([plotsize])
        self.desireSpeed = np.zeros([plotsize])
        self.currentIq = np.zeros([plotsize])
        self.currentId = np.zeros([plotsize])
        self.desiredIq = np.zeros([plotsize])
        self.desiredPosition = np.zeros([plotsize])
        self.currentPosition = np.zeros([plotsize])
        self.desiredSpeed = np.zeros([plotsize])
        self.currentSpeed = np.zeros([plotsize])
        self.currentIab = np.zeros([plotsize])
        self.currentUab = np.zeros([plotsize])
        self.currentUdq = np.zeros([plotsize])


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    qt_app = hostApp()
    qt_app.show()
    sys.exit(app.exec_())
