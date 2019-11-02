def calc_checksum(data):
  checksum = 0xFF
  temp_chk = 0
  bit_sum = 0

  for byte in data:
    shift = 0x80
    iterate = 8
    while iterate:
      iterate -= 1
      bit_sum = byte & shift
      temp_chk = checksum & 0x80
      if bit_sum != 0:
        bit_sum = 0x1C
        if (temp_chk != 0):
          bit_sum = 1
        checksum = checksum << 1
        temp_chk = checksum | 1
        bit_sum ^= temp_chk
      else:
        if temp_chk != 0:
          bit_sum = 0x1D
        checksum = checksum << 1
        bit_sum ^= checksum
      checksum = bit_sum
      shift = shift >> 1
  return ~checksum & 0xFF


test = {
  "c": [0x17, 0x60, 0x4b, 0xbe],
  "d": [
    [0x67, 0xf6, 0x40, 0x00, 0x00, 0x00, 0xd0],
    [0x20, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80],
    [0x69, 0x24, 0x40, 0x00, 0x00, 0x00, 0x10],
    [0x20, 0x00, 0x80, 0x00, 0x00, 0x00, 0xd0]
  ]
}

test = {
  "c": [0x75, 0x3f, 0x83],
  "d": [
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0],
    [0x16, 0x07, 0x3f, 0xff, 0x08, 0xff, 0x60]
  ]
}

if __name__ == "__main__":
  for c, d in zip(test["c"], test["d"]):
    print (calc_checksum(d), c, calc_checksum(d) == c)
import binascii


data = binascii.a2b_hex('4202')
checksum = calc_checksum(data)
print(checksum)