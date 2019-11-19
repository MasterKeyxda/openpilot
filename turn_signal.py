from panda import Panda
from checksum import calc_checksum
import binascii
from time import sleep

p = Panda()
#p.REQUEST_OUT
p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

#p.set_safety_mode(Panda.SAFETY_SUBARU)

# right turn signal
addr = 0x45 #blinker
#addr = 0x0e
#addr_2 = 0x03
data = [0x00, 0x00, 0x01, 0x00, 0x00, 0x00] #blinker
#data = [0x00, 0x02, 0x30, 0x00, 0x00, 0x07, 0x00, 0x00]

#data = [0x1f, 0xcc, 0x3f, 0xff, 0x08, 0xff] #torque
#data_2 = [0x0f, 0xf6, 0x10, 0x00, 0x04, 0xff] #rate
bus = 0
i = 0
while True:
  #p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  #p.connect()
  #p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  r = p.can_recv()
  #p.can_send(addr, bytearray(data), bus)
  
  i = 0
  for addr_, _, data_, bus_ in r:
    if addr_ == addr:
      if i < 16:
        # increment counter, calc checksum

        #print(len(data_))
        #print(binascii.hexlify(data_[7]))
        #print(str(data_).encode("utf-8").hex())
        #print(str(data_).encode("utf-8"))
        cnt = int(str(data_).encode("utf-8").hex()[12:13], base=16)
        
        #cnt = (cnt + 1) << 4 if cnt < 15 else 0
        cnt = (cnt + 1) << 4 if cnt < 15 else 0
        #cnt = data_[7]
        #cnit = (cnt+1)  << 4 if cnt < 15 else 0
        #print(i)\
        #i = (cnt+1)
        #cnt = i
        i_n = i*16
        i_n = int(i_n)
        #print(int('0010000',2))
        #print(i)
        #i = i << 4 
        dat = data[:]
        dat.append(i_n)
       
        

        cksum = int(calc_checksum(dat))
        dat.append(cksum)
        #print(dat)
        # print dat, repr(str(bytearray(dat)))

        #cnt += 1
        #print(len(dat))
        #print(bytearray(dat))
        p.can_send(addr, bytearray(dat), bus)
        i+=1
        sleep(0.05)
        #print(i)
      else:
        i = 0
        i = int(i)
        dat = data[:]
        dat.append(i)
        cksum = int(calc_checksum(dat))
        dat.append(cksum)
        p.can_send(addr, bytearray(dat), bus)
        i+=1
        sleep(0.05)
'''
    if addr_ == addr_2:
      #print('ready')
      if i < 16:
        # increment counter, calc checksum

        #print(len(data_))
        #print(binascii.hexlify(data_[7]))
        #print(str(data_).encode("utf-8").hex())
        cnt = int(str(data_).encode("utf-8").hex()[12:13], base=16)
        #cnt = (cnt + 1) << 4 if cnt < 15 else 0
        cnt = (cnt + 1) << 4 if cnt < 15 else 0
        #cnt = data_[7]
        #cnit = (cnt+1)  << 4 if cnt < 15 else 0
        #print(i)\
        #i = (cnt+1)
        #cnt = i
        i_n = i*8
        i_n = int(i_n)
        #print(int('0010000',2))
        #print(i)
        #i = i << 4 
        dat = data_2[:]
        dat.append(i_n)
        

        cksum = int(calc_checksum(dat))
        dat.append(cksum)
        #print(dat)
        # print dat, repr(str(bytearray(dat)))

        #cnt += 1
        #print(len(dat))
        p.can_send(addr_2, bytearray(dat), bus)
        i+=1
        sleep(0.05)
        #print(i)
      else:
        i = 0
    '''
  #if cnts > 1: print("cnts", cnts)
  #p.reset()
  #p.close()
  #p.reconnect()
  #print('sent')
  #sleep(0.02)
  