############################################################################################
#
#         +-------------+----------------+----------------+-------------------+
#         | Preambule   |                |                |                   |
#         | 0x48        |   cmd word     |   address      |  1..N bytes       |
#         |             |                |                |                   |
#         +-----------------------------------------------+-------------------+
#                       |                |
#                       |                |
#    +------------------+                |
#    |                              +----+
#    |                              |
#    +---------------+--------------+
#    |               |              |
#    |     2 bit     |  4 bits      |
#    |    cmd type   |  data size   |
#    |               |    bytes     |
#    +---------------+--------------+
#
############################################################################################


from PyQt5 import QtCore, QtSerialPort

class communicatorClass(QtSerialPort.QSerialPort):
    def __init__(self):
        super(communicatorClass, self).__init__(readyRead=self.dataReceived)

    def openConnection(self, portName):
        self.setBaudRate(QtSerialPort.QSerialPort.Baud115200)
        self.setPortName(portName)
        try:
            self.open(QtCore.QIODevice.ReadWrite)
            return 1
        except:
            print("Can't open " + portName)
            return -1

    def closeConnection(self):
        if(self.isOpen()):
            self.close()


    def dataReceived(self):
        print("Hello")

    # def parseData(self):
    #
    # def composeData(self):
