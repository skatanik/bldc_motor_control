from PyQt5 import QtCore, QtSerialPort

class communicatorClass(QtSerialPort.QSerialPort):
    def __init__(self):
        super(communicatorClass, self).__init__(readyRead=self.dataReceived)

    def openConnection(self, portName):
        self.setBaudRate(QtSerialPort.QSerialPort.Baud115200)
        self.setPort(portName)
        self.open(QtCore.QIODevice.ReadWrite)

    def closeConnection(self):
        if(self.isOpen()):
            self.close()


    def dataReceived(self):
        print("Hello")

    # def parseData(self):
    #
    # def composeData(self):
