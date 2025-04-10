import dynamixel_sdk as dxl

commPort = str("/dev/ttyNECK")
socket = dxl.PortHandler(commPort)

socket.openPort()