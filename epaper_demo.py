#!/bin/python

# sends a buffer of 5808 bytes to the microcontroller over USART3
# That is enugh data to fill the screen in Black and White Mode

import serial
import time

DISPLAY_WIDTH = 264
DISPLAY_HEIGHT = 176

ser = serial.Serial(
	port = "/mnt/stm32_stlink", #adjust the port to something like COM5
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS)

quarter_black = bytes.fromhex("00"*(DISPLAY_WIDTH*DISPLAY_HEIGHT//32))
quarter_checker = bytes.fromhex((("aa"*(DISPLAY_WIDTH//4))+("55"*(DISPLAY_WIDTH//4)))*(DISPLAY_HEIGHT//16))
quarter_lines = bytes.fromhex("aa"*(DISPLAY_WIDTH*DISPLAY_HEIGHT//32))
quarter_white = bytes.fromhex("ff"*(DISPLAY_WIDTH*DISPLAY_HEIGHT//32))

ser.write(quarter_black)
ser.write(quarter_checker)
ser.write(quarter_lines)
ser.write(quarter_white)
