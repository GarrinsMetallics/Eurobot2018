import serial
import time
import sys

def decoder(cad,num):
    fail=0
    readp=cad.split()#separem porqueria
    b="".join(readp)#juntem el numero      
    try:#evitem errors "akjhd511"
     q=float(b)#convertim a float per operar
     return q#obtenim el numero per operar
    except:#evitem errors 
        fail=fail
        return num
        

ardu1 = serial.Serial('/dev/ttyACM0',9600)
#ardu2 = serial.Serial('/dev/ttyACM1',9600)
#ardu3 = serial.Serial('/dev/ttyACM1',9600)
# PER SAPIGUER ON ESTAN CONECTATS cmd: ls /dev/tty* sortiran el ACM on estan.
write=float
save=1
while 1:#"void loop" 

    readA1=ardu1.readline()###desbloqueja l'ardu1 per utilitzarlo
    #readA2=ardu1.readline()###desbloqueja l'ardu2 per utilitzarlo
    #readA3=ardu1.readline()###desbloqueja l'ardu3 per utilitzarlo
    numardu1=decoder(readA1,save)###desbloqueja l'ardu1 per utilitzarlo
    #numardu2=decoder(readA2,save)###desbloqueja l'ardu2 per utilitzarlo
    #numardu3=decoder(readA3,save)###desbloqueja l'ardu3 per utilitzarlo

    
    ######################  OPERACIONS AMB VALORS DELS ARDUS##############

