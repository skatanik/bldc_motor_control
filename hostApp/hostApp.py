from PyQt5 import QtWidgets, QtSerialPort, QtCore, QtGui
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import threading
import numpy as np
from numpy import asarray
from numpy import savetxt
import time

from ui import mainWindow
import sys


# venv\Scripts\pyuic5.exe -x ui\mainWindow.ui -o ui\mainWindow.py

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
        self.chb_alfaI.clicked.connect(self.chb_alfaI_clicked)
        self.chb_betaI.clicked.connect(self.chb_betaI_clicked)
        self.chb_currentIq.clicked.connect(self.chb_currentIq_clicked)
        self.chb_currentId.clicked.connect(self.chb_currentId_clicked)
        self.chb_desiredIq.clicked.connect(self.chb_desiredIq_clicked)
        self.chb_errorQ.clicked.connect(self.chb_errorQ_clicked)
        self.chb_errorD.clicked.connect(self.chb_errorD_clicked)
        self.chb_resQ.clicked.connect(self.chb_resQ_clicked)
        self.chb_resD.clicked.connect(self.chb_resD_clicked)
        self.chb_alfaV.clicked.connect(self.chb_alfaV_clicked)
        self.chb_betaV.clicked.connect(self.chb_betaV_clicked)
        self.chb_resAmpl.clicked.connect(self.chb_resAmpl_clicked)
        self.chb_resPhase.clicked.connect(self.chb_resPhase_clicked)
        self.chb_currentSpeed.clicked.connect(self.chb_currentSpeed_clicked)
        self.chb_desiredSpeed.clicked.connect(self.chb_desiredSpeed_clicked)
        self.chb_desiredPos.clicked.connect(self.chb_desiredPos_clicked)
        self.chb_currentPos.clicked.connect(self.chb_currentPos_clicked)
        self.pb_clearData.clicked.connect(self.pb_clearData_clicked)
        self.pb_runMotor.clicked.connect(self.pb_runMotor_clicked)
        self.cb_sendData.clicked.connect(self.cb_sendData_clicked)
        self.hs_speedVal.valueChanged.connect(self.hs_valueChanged)
        self.pb_saveData.clicked.connect(self.pb_saveData_clicked)
        self.pb_resetMotor.clicked.connect(self.pb_resetMotor_clicked)
        self.hs_currQval.valueChanged.connect(self.hs_currQval_valChanged)
        self.sb_currDkD.valueChanged.connect(self.sb_currDkD_valChanged)
        self.sb_currDkI.valueChanged.connect(self.sb_currDkI_valChanged)
        self.sb_currDkP.valueChanged.connect(self.sb_currDkP_valChanged)
        self.sb_currQkD.valueChanged.connect(self.sb_currQkD_valChanged)
        self.sb_currQkI.valueChanged.connect(self.sb_currQkI_valChanged)
        self.sb_currQkP.valueChanged.connect(self.sb_currQkP_valChanged)
        self.hs_currQval.valueChanged.connect(self.hs_currQval_valChanged)
        self.chb_testMode.clicked.connect(self.chb_testMode_clicked)
        self.serialConnected = 0
        self.graphWidget.setBackground('w')
        self.motorRunning = 0

        # self.receiveProcessHandler = threading.Thread(target=self.serialReceiveThread, args=())
        # self.updatePlotHandler = threading.Thread(target=self.updatePlotThread, args=())
        self.stopReceivingProcessEvent = threading.Event()
        self.stopUpdatingPlotEvent = threading.Event()
        self.updatePlotTimer = QtCore.QTimer(self)
        self.updatePlotTimer.setInterval(20)
        self.updatePlotTimer.timeout.connect(self.updatePlot)
        self.graphWidget.showGrid(x=True, y=True, alpha=0.3)
        self.runningActive = 0
        self.currentAdataLine = pg.PlotCurveItem(pen=pg.mkPen({'color': "FF5733"}), width=10, name="current Ia")
        self.chb_iacurr.setStyleSheet("QCheckBox {background: #FF5733;}")
        self.currentBdataLine = pg.PlotCurveItem(pen=pg.mkPen({'color': "2418E3"}), width=10)
        self.chb_ibcurr.setStyleSheet("QCheckBox {background:#2418E3 }")
        self.currentCdataLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "C118E3"}), width=2)
        self.chb_iccurr.setStyleSheet("QCheckBox {background:#C118E3}")
        self.alfaILine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "18BE3E"}), width=2)
        self.chb_alfaI.setStyleSheet("QCheckBox {background:#18BE3E}")
        self.betaILine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "44B68C"}), width=2)
        self.chb_betaI.setStyleSheet("QCheckBox {background:#44B68C}")
        self.currentIqLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "5B3DCA"}), width=2)
        self.chb_currentIq.setStyleSheet("QCheckBox {background:#5B3DCA}")
        self.currentIdLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "0BBFBD"}), width=2)
        self.chb_currentId.setStyleSheet("QCheckBox {background:#0BBFBD}")
        self.desiredIqLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "47D562"}), width=2)
        self.chb_desiredIq.setStyleSheet("QCheckBox {background:#47D562}")
        self.errorQLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "C01BB8"}), width=2)
        self.chb_errorQ.setStyleSheet("QCheckBox {background:#C01BB8}")
        self.errorDLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "45552d"}), width=2)
        self.chb_errorD.setStyleSheet("QCheckBox {background:#45552d}")
        self.resultQLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "5a11b8"}), width=2)
        self.chb_resQ.setStyleSheet("QCheckBox {background:#5a11b8}")
        self.resultDLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "6d191b"}), width=2)
        self.chb_resD.setStyleSheet("QCheckBox {background:#6d191b}")
        self.alfaVLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "a43c69"}), width=2)
        self.chb_alfaV.setStyleSheet("QCheckBox {background:#a43c69}")
        self.betaVLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "d0301c"}), width=2)
        self.chb_betaV.setStyleSheet("QCheckBox {background:#d0301c}")
        self.resultAmplLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "5807b0"}), width=2)
        self.chb_resAmpl.setStyleSheet("QCheckBox {background:#5807b0}")
        self.resultPhaseLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "45552d"}), width=2)
        self.chb_resPhase.setStyleSheet("QCheckBox {background:#45552d}")
        self.currentSpeedLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "C1B717"}), width=2)
        self.chb_currentSpeed.setStyleSheet("QCheckBox {background:#C1B717}")
        self.desiredSpeedLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "E70BB1"}), width=2)
        self.chb_desiredSpeed.setStyleSheet("QCheckBox {background:#E70BB1}")
        self.desiredPositionLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "A25DDC"}), width=2)
        self.chb_desiredPos.setStyleSheet("QCheckBox {background:#A25DDC}")
        self.currentPositionLine = pg.PlotCurveItem(clear=True, pen=pg.mkPen({'color': "55C00F"}), width=2)
        self.chb_currentPos.setStyleSheet("QCheckBox {background:#55C00F}")

        self.pb_runMotor.setText("Run Motor")
        self.pb_runMotor.setStyleSheet("QPushButton  {background : green; }")

        self.serialData = QtCore.QByteArray()

    def searchButtonClicked(self):
        for ind in range(self.cb_ports.__len__()):
            self.cb_ports.removeItem(ind)
        for port in QtSerialPort.QSerialPortInfo.availablePorts():
            self.cb_ports.addItem(port.portName())
        if (self.cb_ports.__len__()):
            self.lb_status.setText(str(self.cb_ports.__len__()) + " port(s) found")
            self.lb_status.setStyleSheet("QLabel {color : green; }")
            self.pb_connect.setEnabled(1)
        else:
            self.lb_status.setText("No ports were found")
            self.lb_status.setStyleSheet("QLabel {color : red; }")
            self.pb_connect.setEnabled(0)

    def connectButtonClicked(self):
        if (self.serialConnected == 0):
            selectedPort = self.cb_ports.currentText()
            self.serialPort = QtSerialPort.QSerialPort(selectedPort,
                                                       baudRate=1024000, #QtSerialPort.QSerialPort.Baud115200
                                                       readyRead=self.serialReceiveThread)
            try:
                self.serialPort.open(QtCore.QIODevice.ReadWrite)
            except:
                self.lb_status.setText("Can't open " + selectedPort)
                self.lb_status.setStyleSheet("QLabel {color : red; }")

            if (self.serialPort.isOpen()):
                self.lb_status.setText(selectedPort + " opened")
                self.lb_status.setStyleSheet("QLabel {color : green; }")
                self.serialConnected = 1
                self.pb_connect.setText("Disconnect")
                self.pb_start.setEnabled(1)
                self.pb_clearData_clicked()
                self.pb_runMotor.setEnabled(1)
                self.pb_resetMotor.setEnabled(1)
                self.hs_speedVal.setEnabled(1)
                self.cb_sendData.setEnabled(1)
                self.chb_testMode.setEnabled(1)
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
            self.pb_runMotor.setEnabled(0)
            self.hs_speedVal.setEnabled(0)
            self.cb_sendData.setChecked(0)
            self.cb_sendData.setEnabled(0)
            self.pb_resetMotor.setEnabled(0)
            self.chb_testMode.setEnabled(0)

    def serialReceiveThread(self):
        if (self.serialPort.bytesAvailable()):
            if (self.runningActive == 1):
                self.serialData = self.serialData.append(self.serialPort.readAll())

                while (self.serialData.size() >= 4):
                    ind = 0
                    for byte in self.serialData:
                        if (byte == b'H'):
                            break
                        ind = ind + 1

                    self.serialData.remove(0, ind + 1)

                    if(int.from_bytes(self.serialData[0], "little") == 0x0B):
                        data = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        if (data & (1 << (16 - 1))) != 0:
                            data = data - (1 << 16)
                        self.currentAdata = np.append(self.currentAdata, data)
                        self.currentAdata = self.currentAdata[1:]

                    if(int.from_bytes(self.serialData[0], "little") == 0x0C):
                        data = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        if (data & (1 << (16 - 1))) != 0:
                            data = data - (1 << 16)
                        self.currentBdata = np.append(self.currentBdata, data)
                        self.currentBdata = self.currentBdata[1:]

                    if(int.from_bytes(self.serialData[0], "little") == 0x0D):
                        data = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        if (data & (1 << (16 - 1))) != 0:
                            data = data - (1 << 16)
                        self.currentCdata = np.append(self.currentCdata, data)
                        self.currentCdata = self.currentCdata[1:]

                    if(int.from_bytes(self.serialData[0], "little") == 0x0E):
                        posData = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        self.currentPosition = np.append(self.currentPosition, posData)
                        self.currentPosition = self.currentPosition[1:]

                    if(int.from_bytes(self.serialData[0], "little") == 0x0F):
                        data = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        if (data & (1 << (16 - 1))) != 0:
                            data = data - (1 << 16)
                        self.alfaI = np.append(self.alfaI, data)
                        self.alfaI = self.alfaI[1:]
                    if(int.from_bytes(self.serialData[0], "little") == 0x10):
                        data = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        if (data & (1 << (16 - 1))) != 0:
                            data = data - (1 << 16)
                        self.betaI = np.append(self.betaI, data)
                        self.betaI = self.betaI[1:]
                    if(int.from_bytes(self.serialData[0], "little") == 0x11):
                        data = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        if (data & (1 << (16 - 1))) != 0:
                            data = data - (1 << 16)
                        self.currentIq = np.append(self.currentIq, data)
                        self.currentIq = self.currentIq[1:]
                    if(int.from_bytes(self.serialData[0], "little") == 0x12):
                        data = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        if (data & (1 << (16 - 1))) != 0:
                            data = data - (1 << 16)
                        self.currentId = np.append(self.currentId, data)
                        self.currentId = self.currentId[1:]
                    if (int.from_bytes(self.serialData[0], "little") == 0x15):
                        data = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        if (data & (1 << (16 - 1))) != 0:
                            data = data - (1 << 16)
                        self.desiredIq = np.append(self.desiredIq, data)
                        self.desiredIq = self.desiredIq[1:]
                    if(int.from_bytes(self.serialData[0], "little") == 0x13):
                        data = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        if (data & (1 << (16 - 1))) != 0:
                            data = data - (1 << 16)
                        self.errorQ = np.append(self.errorQ, data)
                        self.errorQ = self.errorQ[1:]
                    if(int.from_bytes(self.serialData[0], "little") == 0x14):
                        data = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        if (data & (1 << (16 - 1))) != 0:
                            data = data - (1 << 16)
                        self.errorD = np.append(self.errorD, data)
                        self.errorD = self.errorD[1:]
                    if(int.from_bytes(self.serialData[0], "little") == 0x16):
                        data = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        if (data & (1 << (16 - 1))) != 0:
                            data = data - (1 << 16)
                        self.resultQ = np.append(self.resultQ, data)
                        self.resultQ = self.resultQ[1:]
                    if(int.from_bytes(self.serialData[0], "little") == 0x17):
                        data = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        if (data & (1 << (16 - 1))) != 0:
                            data = data - (1 << 16)
                        self.resultD = np.append(self.resultD, data)
                        self.resultD = self.resultD[1:]
                    if(int.from_bytes(self.serialData[0], "little") == 0x18):
                        data = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        if (data & (1 << (16 - 1))) != 0:
                            data = data - (1 << 16)
                        self.alfaV = np.append(self.alfaV, data)
                        self.alfaV = self.alfaV[1:]
                    if(int.from_bytes(self.serialData[0], "little") == 0x19):
                        data = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        if (data & (1 << (16 - 1))) != 0:
                            data = data - (1 << 16)
                        self.betaV = np.append(self.betaV, data)
                        self.betaV = self.betaV[1:]
                    if(int.from_bytes(self.serialData[0], "little") == 0x1A):
                        data = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        if (data & (1 << (16 - 1))) != 0:
                            data = data - (1 << 16)
                        self.resultAmpl = np.append(self.resultAmpl, data)
                        self.resultAmpl = self.resultAmpl[1:]
                    if(int.from_bytes(self.serialData[0], "little") == 0x1B):
                        data = int.from_bytes(self.serialData[1], "little") * 256 + int.from_bytes(self.serialData[2], "little")
                        if (data & (1 << (16 - 1))) != 0:
                            data = data - (1 << 16)
                        self.resultPhase = np.append(self.resultPhase, data)
                        self.resultPhase = self.resultPhase[1:]

                    self.serialData.remove(0, 3)
            else:
                self.serialPort.readAll()

    def updatePlot(self):
        if self.chb_iacurr.isChecked():
            self.currentAdataLine.setData(self.currentAdata)
        if self.chb_ibcurr.isChecked():
            self.currentBdataLine.setData(self.currentBdata)
        if self.chb_iccurr.isChecked():
            self.currentCdataLine.setData(self.currentCdata)
        if self.chb_alfaI.isChecked():
            self.alfaILine.setData(self.alfaI)
        if self.chb_betaI.isChecked():
            self.betaILine.setData(self.betaI)
        if self.chb_currentIq.isChecked():
            self.currentIqLine.setData(self.currentIq)
        if self.chb_currentId.isChecked():
            self.currentIdLine.setData(self.currentId)
        if self.chb_desiredIq.isChecked():
            self.desiredIqLine.setData(self.desiredIq)
        if self.chb_errorQ.isChecked():
            self.errorQLine.setData(self.errorQ)
        if self.chb_errorD.isChecked():
            self.errorDLine.setData(self.errorD)
        if self.chb_resQ.isChecked():
            self.resultQLine.setData(self.resultQ)
        if self.chb_resD.isChecked():
            self.resultDLine.setData(self.resultD)
        if self.chb_alfaV.isChecked():
            self.alfaVLine.setData(self.alfaV)
        if self.chb_betaV.isChecked():
            self.betaVLine.setData(self.betaV)
        if self.chb_resAmpl.isChecked():
            self.resultAmplLine.setData(self.resultAmpl)
        if self.chb_resPhase.isChecked():
            self.resultPhaseLine.setData(self.resultPhase)
        if self.chb_currentSpeed.isChecked():
            self.currentSpeedLine.setData(self.currentSpeed)
        if self.chb_desiredSpeed.isChecked():
            self.desiredSpeedLine.setData(self.desiredSpeed)
        if self.chb_desiredPos.isChecked():
            self.desiredPositionLine.setData(self.desiredPosition)
        if self.chb_currentPos.isChecked():
            self.currentPositionLine.setData(self.currentPosition)

        # redraw data lines

    def startButtonClicked(self):
        if (self.runningActive == 0):
            # data arrays
            plotsize = int(self.le_plotPoints.text())

            if (self.currentAdata.shape[0] < plotsize):
                self.currentAdata = np.append(self.currentAdata, np.zeros([plotsize - self.currentAdata.shape[0]]))
            if (self.currentBdata.shape[0] < plotsize):
                self.currentBdata = np.append(self.currentBdata, np.zeros([plotsize - self.currentBdata.shape[0]]))
            if (self.currentCdata.shape[0] < plotsize):
                self.currentCdata = np.append(self.currentCdata, np.zeros([plotsize - self.currentCdata.shape[0]]))
            if (self.alfaI.shape[0] < plotsize):
                self.alfaI = np.append(self.alfaI, np.zeros([plotsize - self.alfaI.shape[0]]))
            if (self.betaI.shape[0] < plotsize):
                self.betaI = np.append(self.betaI, np.zeros([plotsize - self.betaI.shape[0]]))
            if (self.currentIq.shape[0] < plotsize):
                self.currentIq = np.append(self.currentIq, np.zeros([plotsize - self.currentIq.shape[0]]))
            if (self.currentId.shape[0] < plotsize):
                self.currentId = np.append(self.currentId, np.zeros([plotsize - self.currentId.shape[0]]))
            if (self.desiredIq.shape[0] < plotsize):
                self.desiredIq = np.append(self.desiredIq, np.zeros([plotsize - self.desiredIq.shape[0]]))
            if (self.errorQ.shape[0] < plotsize):
                self.errorQ = np.append(self.errorQ, np.zeros([plotsize - self.errorQ.shape[0]]))
            if (self.errorD.shape[0] < plotsize):
                self.errorD = np.append(self.errorD, np.zeros([plotsize - self.errorD.shape[0]]))
            if (self.resultQ.shape[0] < plotsize):
                self.resultQ = np.append(self.resultQ, np.zeros([plotsize - self.resultQ.shape[0]]))
            if (self.resultD.shape[0] < plotsize):
                self.resultD = np.append(self.resultD, np.zeros([plotsize - self.resultD.shape[0]]))
            if (self.alfaV.shape[0] < plotsize):
                self.alfaV = np.append(self.alfaV, np.zeros([plotsize - self.alfaV.shape[0]]))
            if (self.betaV.shape[0] < plotsize):
                self.betaV = np.append(self.betaV, np.zeros([plotsize - self.betaV.shape[0]]))
            if (self.resultAmpl.shape[0] < plotsize):
                self.resultAmpl = np.append(self.resultAmpl, np.zeros([plotsize - self.resultAmpl.shape[0]]))
            if (self.resultPhase.shape[0] < plotsize):
                self.resultPhase = np.append(self.resultPhase, np.zeros([plotsize - self.resultPhase.shape[0]]))
            if (self.currentSpeed.shape[0] < plotsize):
                self.currentSpeed = np.append(self.currentSpeed, np.zeros([plotsize - self.currentSpeed.shape[0]]))
            if (self.desiredSpeed.shape[0] < plotsize):
                self.desiredSpeed = np.append(self.desiredSpeed, np.zeros([plotsize - self.desiredSpeed.shape[0]]))
            if (self.desiredPosition.shape[0] < plotsize):
                self.desiredPosition = np.append(self.desiredPosition, np.zeros([plotsize - self.desiredPosition.shape[0]]))
            if (self.currentPosition.shape[0] < plotsize):
                self.currentPosition = np.append(self.currentPosition, np.zeros([plotsize - self.currentPosition.shape[0]]))

            self.runningActive = 1
            self.pb_start.setText('Stop graph')
            self.le_plotPoints.setDisabled(1)
            self.graphWidget.setXRange(0, plotsize, padding=0)
            # self.graphWidget.clear()
            self.updatePlotTimer.start()
        else:
            self.runningActive = 0
            self.pb_start.setText('Start graph')
            self.updatePlotTimer.stop()
            self.le_plotPoints.setDisabled(0)

    def redrawLines(self):
        if self.chb_iacurr.isChecked():
            self.graphWidget.addItem(self.currentAdataLine)
        if self.chb_ibcurr.isChecked():
            self.graphWidget.addItem(self.currentBdataLine)
        if self.chb_iccurr.isChecked():
            self.graphWidget.addItem(self.currentCdataLine)
        if self.chb_alfaI.isChecked():
            self.graphWidget.addItem(self.alfaILine)
        if self.chb_betaI.isChecked():
            self.graphWidget.addItem(self.betaILine)
        if self.chb_currentIq.isChecked():
            self.graphWidget.addItem(self.currentIqLine)
        if self.chb_currentId.isChecked():
            self.graphWidget.addItem(self.currentIdLine)
        if self.chb_desiredIq.isChecked():
            self.graphWidget.addItem(self.desiredIqLine)
        if self.chb_errorQ.isChecked():
            self.graphWidget.addItem(self.errorQLine)
        if self.chb_errorD.isChecked():
            self.graphWidget.addItem(self.errorDLine)
        if self.chb_resQ.isChecked():
            self.graphWidget.addItem(self.resultQLine)
        if self.chb_resD.isChecked():
            self.graphWidget.addItem(self.resultDLine)
        if self.chb_alfaV.isChecked():
            self.graphWidget.addItem(self.alfaVLine)
        if self.chb_betaV.isChecked():
            self.graphWidget.addItem(self.betaVLine)
        if self.chb_resAmpl.isChecked():
            self.graphWidget.addItem(self.resultAmplLine)
        if self.chb_resPhase.isChecked():
            self.graphWidget.addItem(self.resultPhaseLine)
        if self.chb_currentSpeed.isChecked():
            self.graphWidget.addItem(self.currentSpeedLine)
        if self.chb_desiredSpeed.isChecked():
            self.graphWidget.addItem(self.desiredSpeedLine)
        if self.chb_desiredPos.isChecked():
            self.graphWidget.addItem(self.desiredPositionLine)
        if self.chb_currentPos.isChecked():
            self.graphWidget.addItem(self.currentPositionLine)

    def chb_currentAdata_clicked(self):
        if (self.chb_iacurr.isChecked()):
            self.graphWidget.addItem(self.currentAdataLine)
            self.sendData([0x49, 0x80+0x0B, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x0B, 0x00, 0x00])
            self.graphWidget.removeItem(self.currentAdataLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_currentBdata_clicked(self):
        if (self.chb_ibcurr.isChecked()):
            self.graphWidget.addItem(self.currentBdataLine)
            self.sendData([0x49, 0x80+0x0C, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x0C, 0x00, 0x00])
            self.graphWidget.removeItem(self.currentBdataLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_currentCdata_clicked(self):
        if (self.chb_iccurr.isChecked()):
            self.graphWidget.addItem(self.currentCdataLine)
            self.sendData([0x49, 0x80+0x0D, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x0D, 0x00, 0x00])
            self.graphWidget.removeItem(self.currentCdataLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_alfaI_clicked(self):
        if (self.chb_alfaI.isChecked()):
            self.graphWidget.addItem(self.alfaILine)
            self.sendData([0x49, 0x80+0x0F, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x0F, 0x00, 0x00])
            self.graphWidget.removeItem(self.alfaILine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_betaI_clicked(self):
        if (self.chb_betaI.isChecked()):
            self.graphWidget.addItem(self.betaILine)
            self.sendData([0x49, 0x80+0x10, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x10, 0x00, 0x00])
            self.graphWidget.removeItem(self.betaILine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_currentIq_clicked(self):
        if (self.chb_currentIq.isChecked()):
            self.graphWidget.addItem(self.currentIqLine)
            self.sendData([0x49, 0x80+0x11, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x11, 0x00, 0x00])
            self.graphWidget.removeItem(self.currentIqLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_currentId_clicked(self):
        if (self.chb_currentId.isChecked()):
            self.graphWidget.addItem(self.currentIdLine)
            self.sendData([0x49, 0x80+0x12, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x12, 0x00, 0x00])
            self.graphWidget.removeItem(self.currentIdLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_desiredIq_clicked(self):
        if (self.chb_desiredIq.isChecked()):
            self.graphWidget.addItem(self.desiredIqLine)
            self.sendData([0x49, 0x80+0x15, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x15, 0x00, 0x00])
            self.graphWidget.removeItem(self.desiredIqLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_errorQ_clicked(self):
        if (self.chb_errorQ.isChecked()):
            self.graphWidget.addItem(self.errorQLine)
            self.sendData([0x49, 0x80+0x13, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x13, 0x00, 0x00])
            self.graphWidget.removeItem(self.errorQLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_errorD_clicked(self):
        if (self.chb_errorD.isChecked()):
            self.graphWidget.addItem(self.errorDLine)
            self.sendData([0x49, 0x80+0x14, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x14, 0x00, 0x00])
            self.graphWidget.removeItem(self.errorDLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_resQ_clicked(self):
        if (self.chb_resQ.isChecked()):
            self.graphWidget.addItem(self.resultQLine)
            self.sendData([0x49, 0x80+0x16, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x16, 0x00, 0x00])
            self.graphWidget.removeItem(self.resultQLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_resD_clicked(self):
        if (self.chb_resD.isChecked()):
            self.graphWidget.addItem(self.resultDLine)
            self.sendData([0x49, 0x80+0x17, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x17, 0x00, 0x00])
            self.graphWidget.removeItem(self.resultDLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_alfaV_clicked(self):
        if (self.chb_alfaV.isChecked()):
            self.graphWidget.addItem(self.alfaVLine)
            self.sendData([0x49, 0x80+0x18, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x18, 0x00, 0x00])
            self.graphWidget.removeItem(self.alfaVLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_betaV_clicked(self):
        if (self.chb_betaV.isChecked()):
            self.graphWidget.addItem(self.betaVLine)
            self.sendData([0x49, 0x80+0x19, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x19, 0x00, 0x00])
            self.graphWidget.removeItem(self.betaVLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_resAmpl_clicked(self):
        if (self.chb_resAmpl.isChecked()):
            self.graphWidget.addItem(self.resultAmplLine)
            self.sendData([0x49, 0x80+0x1A, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x1A, 0x00, 0x00])
            self.graphWidget.removeItem(self.resultAmplLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_resPhase_clicked(self):
        if (self.chb_resPhase.isChecked()):
            self.graphWidget.addItem(self.resultPhaseLine)
            self.sendData([0x49, 0x80+0x1B, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x1B, 0x00, 0x00])
            self.graphWidget.removeItem(self.resultPhaseLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_currentSpeed_clicked(self):
        if (self.chb_currentSpeed.isChecked()):
            self.graphWidget.addItem(self.currentSpeedLine)
        else:
            self.graphWidget.removeItem(self.currentSpeedLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_desiredSpeed_clicked(self):
        if (self.chb_desiredSpeed.isChecked()):
            self.graphWidget.addItem(self.desiredSpeedLine)
        else:
            self.graphWidget.removeItem(self.desiredSpeedLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_desiredPos_clicked(self):
        if (self.chb_desiredPos.isChecked()):
            self.graphWidget.addItem(self.desiredPositionLine)
        else:
            self.graphWidget.removeItem(self.desiredPositionLine)
            self.graphWidget.clear()
            self.redrawLines()

    def chb_currentPos_clicked(self):
        if (self.chb_currentPos.isChecked()):
            self.graphWidget.addItem(self.currentPositionLine)
            self.sendData([0x49, 0x80+0x0E, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x0E, 0x00, 0x00])
            self.graphWidget.removeItem(self.currentPositionLine)
            self.graphWidget.clear()
            self.redrawLines()

    def pb_clearData_clicked(self):
        plotsize = int(self.le_plotPoints.text())
        self.currentAdata = np.zeros([plotsize])
        self.currentBdata = np.zeros([plotsize])
        self.currentCdata = np.zeros([plotsize])
        self.alfaI = np.zeros([plotsize])
        self.betaI = np.zeros([plotsize])
        self.currentIq = np.zeros([plotsize])
        self.currentId = np.zeros([plotsize])
        self.desiredIq = np.zeros([plotsize])
        self.errorQ = np.zeros([plotsize])
        self.errorD = np.zeros([plotsize])
        self.resultQ = np.zeros([plotsize])
        self.resultD = np.zeros([plotsize])
        self.alfaV = np.zeros([plotsize])
        self.betaV = np.zeros([plotsize])
        self.resultAmpl = np.zeros([plotsize])
        self.resultPhase = np.zeros([plotsize])
        self.currentSpeed = np.zeros([plotsize])
        self.desiredSpeed = np.zeros([plotsize])
        self.desiredPosition = np.zeros([plotsize])
        self.currentPosition = np.zeros([plotsize])

    def pb_runMotor_clicked(self):
        if (not self.motorRunning):
            self.motorRunning = 1
            self.pb_runMotor.setText("Stop Motor")
            self.pb_runMotor.setStyleSheet("QPushButton  {background : red; }")
            self.hs_speedVal.setEnabled(1)

            if (self.cb_sendData.isChecked()):
                self.sendData([0x49, 0x82, 0x00, 0x01])

            data_low = self.hs_speedVal.value() % 256
            data_high = self.hs_speedVal.value() // 256
            self.sendData([0x49, 0x81, data_high, data_low])

            self.sendData([0x49, 0x80, 0x00, 0x01])

        else:
            self.motorRunning = 0
            self.sendData([0x49, 0x80, 0x00, 0x00])
            self.pb_runMotor.setText("Run Motor")
            self.pb_runMotor.setStyleSheet("QPushButton  {background : green; }")

    def sendData(self, data):
        self.serialPort.write(bytes(data))
        self.serialPort.waitForBytesWritten(10)

    def cb_sendData_clicked(self):
        if (self.cb_sendData.isChecked()):
            self.sendData([0x49, 0x82, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x82, 0x00, 0x00])

    def hs_valueChanged(self):
        data_low = self.hs_speedVal.value() % 256
        data_high = self.hs_speedVal.value() // 256
        self.sendData([0x49, 0x81, data_high, data_low])

    def pb_resetMotor_clicked(self):
        if(self.motorRunning):
            self.pb_runMotor_clicked()
        self.sendData([0x49, 0x83, 0x00, 0x00])
        self.cb_sendData.setChecked(0)

    def sb_currDkD_valChanged(self):
        data_low = self.sb_currDkD.value() % 256
        data_high = self.sb_currDkD.value() // 256
        self.sendData([0x49, 0x84, data_high, data_low])

    def sb_currDkI_valChanged(self):
        data_low = self.sb_currDkI.value() % 256
        data_high = self.sb_currDkI.value() // 256
        self.sendData([0x49, 0x85, data_high, data_low])

    def sb_currDkP_valChanged(self):
        data_low = self.sb_currDkP.value() % 256
        data_high = self.sb_currDkP.value() // 256
        self.sendData([0x49, 0x86, data_high, data_low])

    def sb_currQkD_valChanged(self):
        data_low = self.sb_currQkD.value() % 256
        data_high = self.sb_currQkD.value() // 256
        self.sendData([0x49, 0x87, data_high, data_low])

    def sb_currQkI_valChanged(self):
        data_low = self.sb_currQkI.value() % 256
        data_high = self.sb_currQkI.value() // 256
        self.sendData([0x49, 0x88, data_high, data_low])

    def sb_currQkP_valChanged(self):
        data_low = self.sb_currQkP.value() % 256
        data_high = self.sb_currQkP.value() // 256
        self.sendData([0x49, 0x89, data_high, data_low])

    def hs_currQval_valChanged(self):
        data_full = self.hs_currQval.value()
        if (data_full >= 0):
            data_low = self.hs_currQval.value() % 256
            data_high = self.hs_currQval.value() // 256
        else:
            data_full = data_full + (1 << 16)
            data_low = data_full % 256
            data_high = data_full // 256

        self.sendData([0x49, 0x8A, data_high, data_low])

    def chb_testMode_clicked(self):
        if (self.chb_testMode.isChecked()):
            self.sendData([0x49, 0x80+0x1C, 0x00, 0x01])
        else:
            self.sendData([0x49, 0x80+0x1C, 0x00, 0x00])


    def pb_saveData_clicked(self):
        if self.chb_iacurr.isChecked():
            savetxt('iacurr.csv', self.currentAdata, delimiter=', ')
        if self.chb_ibcurr.isChecked():
            savetxt('ibcurr.csv', self.currentBdata, delimiter=', ')
        if self.chb_iccurr.isChecked():
            savetxt('iccurr.csv', self.currentCdata, delimiter=', ')
        if self.chb_currentIq.isChecked():
            savetxt('currentIq.csv', self.currentIq, delimiter=', ')
        if self.chb_currentId.isChecked():
            savetxt('currentId.csv', self.currentId, delimiter=', ')
        if self.chb_desiredIq.isChecked():
            savetxt('desiredIq.csv', self.desiredIq, delimiter=', ')
        if self.chb_currentPos.isChecked():
            savetxt('currentPos.csv', self.currentPosition, delimiter=', ')
        if self.chb_desiredPos.isChecked():
            savetxt('desiredPos.csv', self.desiredPosition, delimiter=', ')
        if self.chb_currentSpeed.isChecked():
            savetxt('currentSpeed.csv', self.desiredSpeed, delimiter=', ')
        if self.chb_desiredSpeed.isChecked():
            savetxt('desiredSpeed.csv', self.currentSpeed, delimiter=', ')
        if self.chb_currentIab.isChecked():
            savetxt('currentIab.csv', self.currentIab, delimiter=', ')
        if self.chb_currentUab.isChecked():
            savetxt('currentUab.csv', self.currentUab, delimiter=', ')
        if self.chb_currentUdq.isChecked():
            savetxt('currentUdq.csv', self.currentUdq, delimiter=', ')


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    qt_app = hostApp()
    qt_app.show()
    sys.exit(app.exec_())
