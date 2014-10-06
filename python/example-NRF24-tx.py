# NRF24 examples not released yet (V0.8)
# Promise: "It works wonderfully! (via virtGPIO's GPIO.SpiDev)"
# Payload sizes variable up to 32 bytes.

# Slightly tweaked version of Joao Paulo Barraca' python RF24 library
#     (& that was a python port (for BeagleBone Black) of the now classic maniacbug arduino C++ library)

# These do all talk to each other happily over $1 NRF24L01+ radios (2.4GHz):
# 1. Raspberry Pi (python) -> virtGPIO device -> NRF24
# 2. PC linux (python) -> virtGPIO device -> NRF24
# 3. PC Windows (python) -> virtGPIO device -> NRF24
# 4. Independent arduino (INO sketch, maniacbug library) -> NRF24

# anecdotal:
# I'm currently watching 100 2-way transactions/sec (dynamic-packet ack-payload mode),
# 2MHz speed, packets 6 to 18 bytes variable,
# running for hours without missing a beat.
# PTX end = laptop-virtGPIO-SPI-RF24,   PRX end = RaspberryPi-(native)SPI-RF24,
# RF24s physically 2 stories apart.

# Wait ...

#           Wait ...
