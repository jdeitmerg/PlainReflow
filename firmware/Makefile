PROJ_NAME=firmware.ino
HEXFILE=$(PROJ_NAME).eigthanaloginpus.hex

$(HEXFILE): $(PROJ_NAME)
	arduino --verify --board arduino:avr:nano $(PROJ_NAME)
	@# ToDo: If this fails, check for libraries and install them

deploy: $(PROJ_NAME)
	arduino --upload --board arduino:avr:nano --port /dev/ttyUSB0 $(PROJ_NAME)

clean:
	rm -f *.hex

