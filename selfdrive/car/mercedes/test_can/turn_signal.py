from panda import Panda
from checksum import calc_checksum

p = Panda()
p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

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
      cnt = int(str(data_).encode('hex')[12:13], base=16)
      print (cnt)
      cnt = (cnt + 1) << 4 if cnt < 15 else 0

      dat = data[:]
      dat.append(cnt)

      cksum = int(calc_checksum(dat))
      dat.append(cksum)

      # print dat, repr(str(bytearray(dat)))

      cnt += 1

      p.can_send(addr, str(bytearray(dat)), bus)
  if cnts > 1: print("cnts", cnts)