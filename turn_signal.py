from panda import Panda
from checksum import calc_checksum
import binascii

p = Panda()
#p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
p.set_safety_mode(Panda.SAFETY_FORD)

# right turn signal
addr = 0x45
data = [0x00, 0xff, 0x02, 0x00, 0x00, 0x00]
bus = 0

while True:
  r = p.can_recv()

  cnts = 0

  for addr_, _, data_, bus_ in r:
    if addr_ == addr:
      # increment counter, calc checksum
      print(len(data_))
      import numpy as np
      #print(str(data_)[18:20])
      #cnt = int(binascii.hexlify(data_[12:13]))
      #print(int.from_byte(cnt))
      cnt = int(str(data_).encode("utf-8").hex()[12:13], base=16)
      cnt = (cnt + 1) << 4 if cnt < 15 else 0

      dat = data[:]
      dat.append(cnt)

      cksum = int(calc_checksum(dat))
      dat.append(cksum)

      # print dat, repr(str(bytearray(dat)))

      cnt += 1
      #print(len(dat))
      p.can_send(addr, bytearray(dat), bus)
  if cnts > 1: print("cnts", cnts)