	
	#Receives data from phone and stores in CSV file( cotains IMU information - acceleration, magnetometer, gyroscope)
	# -------------------------------------------------------
import socket, traceback, string
from sys import stderr
import csv
from collections import OrderedDict

host = ''
port = 5555
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind((host, port))

# print one blank line
print 
c=csv.writer(open('gpu_exp_8.csv',"wb"))
while 1:
    print 'hi'
    try:
        message, address = s.recvfrom(8192)
	print 'message'
        print message

# split records using comma as delimiter (data are streamed in CSV format)
        inp = message.split( "," )
	#print inp

	a1=0
	a2=0
	a3=0

	t = inp[0]
	sensorID = int(inp[1])
        id1 = sensorID
	if sensorID==3:     # sensor ID for the Aeccelerometer
		a1=1
		ax, ay, az = inp[2], inp[3], inp[4]
		
	sensorID = int(inp[5])
        id2 = sensorID
	if sensorID==4:     # sensor ID for the Gyroscope
		a2=1
		gx, gy, gz = inp[6],inp[7], inp[8]
		
	sensorID = int(inp[9])
        id3 = sensorID
	#print '#'
	#print 'inp9' 
	if sensorID==5:     # sensor ID for the Magnetometer
		
		a3=1
		mx, my, mz = inp[10], inp[11], inp[12]
		#stdout=data_array
	#print 'after inp9'
	print '#'

	if (a1==1&a2==1&a3==1):
		data_array=[t,ax,ay,az,gx,gy,gz,mx,my,mz]
		#self.data_array=[self.t,self.ax,self.ay,self.az,self.gx,self.gy,self.gz,self.mx,self.my,self.mz]
		stdout=data_array
		
		c.writerow([t,id1,ax,ay,az,id2,gx,gy,gz,id3,mx,my,mz,])
		
		#print data_array
		#with open('gpu_stream_data.csv','w') as outfile:
			#field=OrderedDict([('t',None),('id1',None),('ax',None),('ay',None),('az',None),('id2',None),('gx',None),('gy',None),('gz',None),('id3',None),('mx',None),('my',None),('mz',None)])
			#field = ["t","id1","ax","ay","az","id2","gx","gy","gz","id3","mx","my","mz"]
			#writer=csv.DictWriter(outfile,filednames=field,restval ="",)
			#writer.writeheader()
			#for x in cursor:
			#	writer.writerow(x)

        #stderr.write("\r t = %s s |  (ax,ay,az) = (%s,%s,%s) m/s^2" % (self.t, self.ax, self.ay, self.az) )
        #stderr.flush()
			

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
# -------------------------------------------------------
