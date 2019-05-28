# ------------------------------------------------------------------------------ #
#
# Description: Python Embedded Project Example for Morpheus
#
# (C) 2001 - 2018 Extreme Telematics Corp
# ------------------------------------------------------------------------------ #

# Friendly Names ----------------------------------------------------------------#
#
# AI1 = A_1
# AI2 = Analog2
# DO = LED1
# SENPWR = SENPWR
# COM1 = COM1 (functionality = MODBUS)
# COM2 = COM2 (functionality = MODBUS)
#
# ------------------------------------------------------------------------------ #


# Header files ----------------------------------------------------------------- #
from api_client import http
from api_client import modbus
from api_client import analog
from api_client import digital
from api_client import return_values
from time import sleep
import threading 

# Global Variables ------------------------------------------------------------- #
#Thread Task Flags
A_THREAD_RUNNING = False
B_THREAD_RUNNING = False
C_THREAD_RUNNING = False
D_THREAD_RUNNING = False
F_THREAD_RUNNING = False

# Function Definitions --------------------------------------------------------- #	
def app_run():
	global A_THREAD_RUNNING
	global B_THREAD_RUNNING
	global C_THREAD_RUNNING	
	global D_THREAD_RUNNING
	global F_THREAD_RUNNING

	action = raw_input("""Select Task Example and Press Enter, typing  App letter again will Kill the Task
	a = Run Blinky LED Task
	b = Run Ratiometric 5V Sensor Task
	c = Run Digital Input Monitor Task
        d = Run Modbus Polling Task COM1
	e = Read/Write Modbus COM1
	f = Run Modbus Polling Task COM2""")
	
	action = str(action)
	
	if action == 'a':
		if (not(A_THREAD_RUNNING)):
			A_THREAD_RUNNING = True
			a = threading.Thread(target = blinky_task, args=())
                       	a.start()
		else:
			A_THREAD_RUNNING = False

	elif action == 'b':
		if(not(B_THREAD_RUNNING)):
			B_THREAD_RUNNING = True
			b = threading.Thread(target = sensor_read_task, args=())
			b.start()
		else:
			B_THREAD_RUNNING = False	
	elif action == 'c':
		if(not(C_THREAD_RUNNING)):
			C_THREAD_RUNNING = True
			c = threading.Thread(target = read_din_task, args=())
			c.start()
		else:
			C_THREAD_RUNNING = False
	elif action == 'd':
		if(not(D_THREAD_RUNNING)):
			D_THREAD_RUNNING = True
			try:
				id = int(raw_input("Modbus ID?"))
			except:
				print("Modbus ID Error")
				D_THREAD_RUNNING = False
				return	
			try:
				reg_addr = int(raw_input("Start Address?"))
			except:
				print("Modbus Start Address Error")
				D_THREAD_RUNNING = False
				return		
			d = threading.Thread(target = Com1_modbus_read_task, args=(id,reg_addr))
			d.start()
        	else:
			D_THREAD_RUNNING = False
		
    	elif action == 'e':
		Com1_read_write_modbus()

	elif action == 'f':
		if(not(F_THREAD_RUNNING)):
                        F_THREAD_RUNNING = True
			try:
				id = int(raw_input("Modbus ID?"))
			except:
				print("Modbus ID Error")
				F_THREAD_RUNNING = False
				return	
			try:
				reg_addr = int(raw_input("Start Address?"))
			except:
				print("Modbus Start Address Error")
				F_THREAD_RUNNING = False
				return	
                        f = threading.Thread(target = Com2_modbus_read_task, args=(id,reg_addr))
                        f.start()
                else:
                        F_THREAD_RUNNING = False
	else:
		print("invalid option")
	

def blinky_task():
	global A_THREAD_RUNNING
	led1 = digital.DO('LED1')	
	while (A_THREAD_RUNNING):
		led1.set()
		print("led_pulse")
		sleep(.5)
		led1.clear()
		sleep(.5)
	#turn off LED when thread exits
	led1.set()
		

def sensor_read_task():
	global B_THREAD_RUNNING
	senpwr = digital.SensorPower("SENPWR")
	a_1 = analog.AI("A_1")
	analog2 = analog.AI("Analog2") 
	#read settings of analog from Hardware Configuration
	success, result, payload  = analog.read_analog_process_config()
	
	#Hardcoded numbers
	a_1_min_voltage = 500
	a_1_max_voltage = 4500
	a_1_min_sensor = 0
	a_1_max_sensor = 100		#Sensor Connected to A_1 is a 0-100 psi sensor

	analog2_min_count = 200
	analog2_max_count = 2048	#2048 is the max analog count for 11 bit ADC on Morpheus
	analog2_min_sensor = 0
	analog2_max_sensor = 200	#Sensor Connected to Analog2 is a 0-200 psi sensor

	while (B_THREAD_RUNNING):
		senpwr.set()
		sleep(1) #let sensor signal stabilize 
		a_1_volts = a_1.read_milli_volts()		#read raw ADC value in milli volts
		analog2_counts = analog2.read_counts()		#read raw ADC value in ADC counts
		print("\nSENPWR High. a_1 = %i mV. Analog2 = %i ADC counts." %(a_1_volts,analog2_counts))	
		#calculate rationmetric A_1
		if(a_1_volts  >= a_1_max_voltage):
			a_1_sensor = a_1_max_sensor
		elif(a_1_volts <= a_1_min_voltage):
			a_1_sensor = a_1_min_sensor
		else:
			a_1_sensor = (float(a_1_volts - a_1_min_voltage)/(a_1_max_voltage-a_1_min_voltage))*a_1_max_sensor
		#calculate rationmetric Analog2
		if(analog2_counts  >= analog2_max_count):
                        analog2_sensor = analog2_max_sensor
                elif(analog2_counts <= analog2_min_count):
                        analog2_sensor = analog2_min_sensor
                else:
                        analog2_sensor = (float(analog2_counts-analog2_min_count)/(analog2_max_count-analog2_min_count))*analog2_max_sensor
		
		print("a_1 = %i PSI, Analog2 = %i PSI." %(a_1_sensor, analog2_sensor))

		
		senpwr.clear()
		print("SENPWR Low")
		sleep(5)
		
edge2_event = threading.Event()

def edge_detect2():
	edge2_event.set()	

def read_din_task():
	global C_THREAD_RUNNING 
	
	digital_input_1 = digital.DI('Digital_Input_1')
	input_1 = digital.DI('Input_2')
	input_1.set_edge_detect(digital.RISING, edge_detect2)

	while (C_THREAD_RUNNING):

		digital_input_1_val = digital_input_1.read()
                print("\nDigital_Input_1 = %i" %(digital_input_1_val))

		event2_is_set = edge2_event.wait(1)
	
		if event2_is_set:
			print("Input_2: Falling Edge detected")
		else:
			print("Input_2: Falling Edge not detected")	
		edge2_event.clear()
		sleep(5)

def Com1_modbus_read_task(id,reg_addr):
	global D_THREAD_RUNNING
	print("")
        while (D_THREAD_RUNNING):
		
		read_val = modbus.read_holding("COM1", id, reg_addr,1)	#Return value is not a UINT16 but a list of Modbus message details check api_client->Modbus for more details.
		if(read_val[0] != 200):	# read_val[0] = docker message result, read_val[1] = SUCCESS or Modbus Error , read_val[2] = is the start of the list for the modbus registers read. 
			print("modbus read failed")
		else:	
			print("read value = %i" %  (read_val[2][0])) #3 is the location of the register value
		sleep(5) 		
		
def Com1_read_write_modbus():

	try:
		cmd = raw_input("read or  write ?")
	except:
		print("Modbus Command Error")
		return		
	try:
		id = int(raw_input("Modbus ID?"))
	except:
		print("Modbus Command Error")
		return
	try:	
		reg_addr = int(raw_input("Start address?"))	
	except:
		print("Modbus Command Error")
		return
		
	write_val = [0]
	if cmd == 'write':
		write_val[0] = int(raw_input("value to write?"))	#Cast the raw_input as string since it is a string when the user types a value in.
		if( modbus.write_holding("COM1", int(id), int(reg_addr), write_val)):
			print("Modbus Write Success")
		else:
			print("Modbus Write Failed")
	elif cmd == 'read':
		read_val = modbus.read_holding("COM1", int(id), int(reg_addr), 1)
		if( read_val[0] != 200):
			print("modbus read failed")
		else:
			print("read value = %i" % read_val[2][0]) #read_val is in [2]=payload, [0]=first value in the list  
		
	else:
		print("Modbus Command Error") 

def Com2_modbus_read_task(id, reg_addr):
        global F_THREAD_RUNNING
        print("")
        while (F_THREAD_RUNNING):
                read_val = modbus.read_holding("COM2", id, reg_addr,1)
                if(read_val[0] != 200):
			print(read_val)
                        print("modbus read failed")
                else:
                        print("read value = %i" %  (read_val[2][0])) #2 is the location of the register
                sleep(5)

# Main ------------------------------------------------------------------------- #				
if __name__ == "__main__":
	
		
	print("Example Application Start")
	raw_input("Press Enter to Begin")
	

	while True:
		app_run()
		
	while True:
		led1.set()
		sleep(.5)
		led1.clear()
		sleep(.5)		
