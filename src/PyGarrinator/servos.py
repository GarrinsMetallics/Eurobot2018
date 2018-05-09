import RPi.GPIO as IO        # calling for header file for GPIO’s of PI
import time                           # calling for time to provide delays in program
IO.setwarnings(False)          # do not show any warnings
IO.setmode (IO.BCM)            # programming the GPIO by BCM pin numbers. (like PIN29 as‘GPIO5’)
IO.setup(27,IO.OUT)             # initialize GPIO19 as an output
IO.setup(22,IO.OUT)
IO.setup(10,IO.OUT)
IO.setup(5,IO.IN)
escombrat = IO.PWM(27,50)              # GPIO19 as PWM output, with 50Hz frequency
escombrat.start(7.5)                             # generate PWM signal with 7.5% duty cycle
tap = IO.PWM(22,50)              
tap.start(7.5)

while IO.input(5)==0:
    continue

esc = IO.PWM(10,50)              
esc.start(7.5)
esc.ChangeDutyCycle(5)
time.sleep(1)

for a in range (12,2):
    esc.ChangeDutyCycle(a)
    time.sleep(0.01)

time.sleep(1)

for a in range (2,12):
    esc.ChangeDutyCycle(a)
    time.sleep(0.01)
time.sleep(1)

for a in range (12,5):
    esc.ChangeDutyCycle(a)
    time.sleep(0.01)

time.sleep(0.2)
for a in range (50,59):
    esc.ChangeDutyCycle(a/10)
    time.sleep(0.01)

t_end = time.time() + 10 #10seconds

while time.time() < t_end:          
    #esc.ChangeDutyCycle(8.5)                                         
    tap.ChangeDutyCycle(8.5)                        #110º
    escombrat.ChangeDutyCycle(12.5)                 #180º
    time.sleep(1)                                     
    escombrat.ChangeDutyCycle(2.5)                  # 0º
    time.sleep(1)                                     
    tap.ChangeDutyCycle(12.5)                       #180º
    time.sleep(0.17)


escombrat.ChangeDutyCycle(12.5)                 #180º
tap.ChangeDutyCycle(8.5)                        #110º
for a in range (59,55):
    esc.ChangeDutyCycle(a/10)
    time.sleep(0.01)