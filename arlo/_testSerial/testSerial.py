import serial
test_string = "hello world\n"
port_list = ["/dev/ttyAMA0", "/dev/ttyS0"]
for port in port_list:
	try:
		serialPort = serial.Serial(port,115200, timeout = 2)
		print("bytesize= ", serialPort.bytesize, ", parity= ", serialPort.parity, ", stop bit = ", serialPort.stopbits)
		print("Port serie ", port, " ouvert pour le test :")
		bytes_sent = serialPort.write(test_string.encode())
		print("Envoye ", bytes_sent, " octets")
		loopback = serialPort.read(bytes_sent)
		if loopback.decode('utf-8') == test_string:
			print("Recu ", len(loopback), " octets identiques\n")
		else:
			print("Reception donnees incorrectes : ", loopback )

		serialPort.close()
	except IOError:
		print("Erreur sur ", port, "\n")

