data='AA 01 31 31 34 35 FF 0D 0A'
data[2]="1"
d=bytes.fromhex(data)
print(d)