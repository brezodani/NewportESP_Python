from newportESP import ESP
print("HelloWorld from Python Applicaiton in Docker")
esp = ESP('/dev/ttyUSB0')  # open communication with controller
stage = esp.axis(1)        # open axis no 1
stage.id                   # print stage ID
stage.move_to(1.2)         # Move to position 1.2 mm