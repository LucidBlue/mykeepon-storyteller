def readnumber(f):
  s=0
  c = f.read(4)
  for i in range(4):
    s += ord(c[i])*256**i
#  print "                   ***", repr(c),"EEE"
  return s

def readchunk(f,level=0,searchcues=False):
  pos = f.tell()
  name= f.read(4)
  leng= readnumber(f)
  totleng = leng+8
  print "   "*level,name,"len-8 =%8d"%leng,"   start of chunk =",pos,"bytes"

  if name in ("RIFF","list"):
      print "   "*level,f.read(4),"recursive sublist" 
      sublen = leng-4
      while sublen>0:
         sublen -= readchunk(f,level+1,searchcues)
      if sublen !=0:
         print "ERROR:",sublen
  elif searchcues and name=="cue ":
    sublen=leng-4
    num = readnumber(f)
    print num,"MARKER(S) *********"
    for i in range(num):
      sublen -= 24
      cuename = readnumber(f)
      cuepos = readnumber(f)
      cuechunk = f.read(4)
      cuechunkstart = readnumber(f)
      cueblockstart = readnumber(f)
      cueoffset = readnumber(f)
      if not (cuechunkstart==0 and cueblockstart==0 and
          cuechunk=="data" and cuename==i+1 and cuepos==cueoffset):
        print "unexpected marker data", cuename, cuepos, cuechunk,\
                  cuechunkstart, cueblockstart, cueoffset
      else:
        print "Marker#",i+1,"   offset =",cueoffset,"bytes ***"
    if sublen !=0:
      print "ERROR:",sublen
  elif searchcues and name=="labl" and level==2:
    sublen=leng-4
    labelname = readnumber(f)
    labeltext = f.read(sublen)
    print "Label #",labelname,"  name = >>"+labeltext.rstrip("\x00")+"<<"
  else:
    f.seek(leng,1) # relative skip
  return totleng

def allchunks(f):
  readchunk(f)
  c = f.read()
  if c != '':
     print "error", len(c), c[:20]

def cues(f):
  readchunk(f,searchcues=True)

if __name__ == "__main__":
  from sys import argv
  if len(argv)<=1:
    print "Usage: python", argv[0],"WAV-file"
    exit()
  filename = argv[1]
  print "analyze chunk structure from WAV-file",filename
  f = open(filename,"r")
  cues(f)
  f.close()
