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
        self.serialConnected = 0
        self.graphWidget.setBackground('w')

        # self.receiveProcessHandler = threading.Thread(target=self.serialReceiveThread, args=())
        # self.updatePlotHandler = threading.Thread(target=self.updatePlotThread, args=())
        self.stopReceivingProcessEvent = threading.Event()
        self.stopUpdatingPlotEvent = threading.Event()
        self.updatePlotTimer = QtCore.QTimer(self)
        self.updatePlotTimer.setInterval(20)
        self.updatePlotTimer.timeout.connect(self.updatePlot)
        # self.graphWidget.addLegend()
        self.runningActive = 0
        self.currentAdataLine = pg.PlotCurveItem(pen=pg.mkPen({'color': "FF5733"}), width=10, name="current Ia")
        self.chb_iacurr.setStyleSheet("QCheckBox {background: #FF5733;}")
        self.currentBdataLine = pg.PlotCurveItem(pen=pg.mkPen({'color': "2418E3"}), width = 10)
        self.chb_ibcurr.setStyleSheet("QCheckBox {background:#2418E3 }")
        self.currentCdataLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "C118E3"}), width = 2,name="Red")
        self.chb_iccurr.setStyleSheet("QCheckBox {background:#C118E3}")
        self.currentIdLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "18BE3E"}), width = 2, name="Red")
        self.chb_currentIq.setStyleSheet("QCheckBox {background:#18BE3E}")
        self.currentIqLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "44B68C"}), width = 2, name="Red")
        self.chb_currentId.setStyleSheet("QCheckBox {background:#44B68C}")
        self.desiredIqLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "5B3DCA"}), width = 2, name="Red")
        self.chb_desiredIq.setStyleSheet("QCheckBox {background:#5B3DCA}")
        self.currentPositionLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "C1B717"}), width = 2, name="Red")
        self.chb_currentPos.setStyleSheet("QCheckBox {background:#C1B717}")
        self.desiredPositionLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "E70BB1"}), width = 2, name="Red")
        self.chb_desiredPos.setStyleSheet("QCheckBox {background:#E70BB1}")
        self.currentSpeedLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "A25DDC"}), width = 2, name="Red")
        self.chb_currentSpeed.setStyleSheet("QCheckBox {background:#A25DDC}")
        self.desiredSpeedLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "55C00F"}), width = 2, name="Red")
        self.chb_desiredSpeed.setStyleSheet("QCheckBox {background:#55C00F}")


    def searchButtonClicked(self):
        for ind in range(self.cb_ports.__len__()):
            self.cb_ports.removeItem(ind)
        for port in QtSerialPort.QSerialPortInfo.availablePorts():
            self.cb_ports.addItem(port.portName())
        if(self.cb_ports.__len__()):
            self.lb_status.setText(str(self.cb_ports.__len__()) + " port(s) found")
            self.lb_status.setStyleSheet("QLabel {color : green; }")
        else:
            self.lb_status.setText("No ports were found")
            self.lb_status.setStyleSheet("QLabel {color : red; }")


    def connectButtonClicked(self):
        if(self.serialConnected == 0):
            selectedPort = self.cb_ports.currentText()
            self.serialPort = QtSerialPort.QSerialPort(selectedPort,
                                                baudRate=QtSerialPort.QSerialPort.Baud115200,
                                               readyRead=self.serialReceive)
            flagOpen = 1
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
            else:
                self.lb_status.setText("Can't open " + selectedPort)
                self.lb_status.setStyleSheet("QLabel {color : red; }")
        else:
            self.serialConnected = 0
            self.pb_connect.setText("Connect")
            self.lb_status.setText(self.cb_ports.currentText() + " closed")
            self.lb_status.setStyleSheet("QLabel {color : green; }")
            self.serialPort.close()

    def serialReceiveThread(self):
        # receive data parse, update data arrays
        print("data")

    def updatePlot(self):
        plotsize = int(self.le_plotPoints.text())
        # self.graphWidget.clear()
        if self.chb_iacurr.isChecked():
            self.currentAdata = self.currentAdata[1:]
            self.currentAdata = np.append(self.currentAdata,np.random.normal(size=1))
            self.currentAdataLine.setData(self.currentAdata)
        if self.chb_ibcurr.isChecked():
            self.currentBdata = self.currentBdata[1:]
            self.currentBdata = np.append(self.currentBdata, np.random.normal(size=1))
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

        # redraw data lines

    def startButtonClicked(self):
        if(self.runningActive == 0):
            # data arrays
            plotsize = int(self.le_plotPoints.text())

            self.currentAdata = np.random.normal(size = plotsize) #np.zeros([1, plotsize])
            self.currentBdata = np.random.normal(size = plotsize) #np.zeros([1, plotsize])
            self.currentCdata = np.random.normal(size = plotsize) #np.zeros([1, plotsize])
            self.currentCdataRaw = np.random.normal(size = plotsize) #np.zeros([1, plotsize])
            self.currentAbsoluteAngle = np.random.normal(size = plotsize) #np.zeros([1, plotsize])
            self.currentSpeed = np.random.normal(size = plotsize) #np.zeros([1, plotsize])
            self.desireSpeed = np.random.normal(size = plotsize) #np.zeros([1, plotsize])
            self.currentIq = np.random.normal(size = plotsize) #np.zeros([1, plotsize])
            self.currentId = np.random.normal(size = plotsize) #np.zeros([1, plotsize])
            self.desiredIq = np.random.normal(size = plotsize) #np.zeros([1, plotsize])
            self.desiredPosition = np.random.normal(size = plotsize) #np.zeros([1, plotsize])
            self.currentPosition = np.random.normal(size = plotsize) #np.zeros([1, plotsize])
            self.desiredSpeed = np.random.normal(size = plotsize) #np.zeros([1, plotsize])
            self.currentSpeed = np.random.normal(size = plotsize) #np.zeros([1, plotsize])
            self.runningActive = 1
            self.pb_start.setText('Stop')
            self.le_plotPoints.setDisabled(1)
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


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    qt_app = hostApp()
    qt_app.show()
    sys.exit(app.exec_())
