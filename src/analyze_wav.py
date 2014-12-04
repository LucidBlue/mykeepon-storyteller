#!/usr/bin/python

def formatline(position1,position2,name): # for the labels.txt file
  num1 = "%8.6f" % position1
  num2 = "%8.6f" % position2
  return num1+ "\t"+ num2 +"\t" +name+"\n"


from sys import exit,argv
import wave

def error(s):
  print (s)
  exit()

def readnumber(f):
  c = f.read(4)
  if len(c)<4:
     error("Sorry, no cue information found.")
  return sum(ord(c[i])*256**i for i in range(4))

def findcues(filename):
  f = wave.open(filename,"r")
  framerate = f.getframerate()
  channels = f.getnchannels()
  bytespersample = f.getsampwidth()
  totalframes = f.getnframes()
  totalduration = float(totalframes)/framerate
  byterate = framerate * channels * bytespersample
  print (str(framerate) + " samples = "+str(byterate)+ " bytes per second")
  f.close()

  f = open(filename,"r")
  if f.read(4) != "RIFF":
      error("Unknown file format (not RIFF)")
  f.read(4)
  if f.read(4) != "WAVE":
      error("Unknown file format (not WAVE)")
  name = f.read(4)
  while name != "cue ":
    leng= readnumber(f)
    f.seek(leng,1) # relative skip
    name = f.read(4)

  leng= readnumber(f)
  num = readnumber(f)
  if leng != 4+24*num:
    error("Inconsistent length of cue chunk")
  print (str(num) + "MARKER(S) found *********")
  if num>0:
   oldmarker = 0.0
   out=open("labels.txt","w")
   for i in range(1,num+1):
      cuename = readnumber(f)
      cuepos = readnumber(f)
      cuechunk = f.read(4)
      cuechunkstart = readnumber(f)
      cueblockstart = readnumber(f)
      cueoffset = readnumber(f)
      if not (cuechunkstart==0 and cueblockstart==0 and
          cuechunk=="data" and cuename==i and cuepos==cueoffset):
        print (cuename, cuepos, cuechunk,
                  cuechunkstart, cueblockstart, cueoffset)
        error("unexpected marker data")
      else:
        position = float(cueoffset)/framerate
        print("Marker",i,
           "   offset =",cueoffset,"samples =",position,"seconds")
        if position>oldmarker:
          # prefer to mark them as regions (intervals)
          out.write(formatline(oldmarker,position, "Section "+str(i)))
        else:
          out.write(formatline(position,position, "Marker "+str(i)))
        oldmarker = position
   if totalduration>oldmarker:
      out.write(formatline(oldmarker,totalduration, "Section "+str(num+1)))
   out.close()
   print ("Marker data was written to labels.txt")
  else:
   print ("No marker data file was written.")
  f.close()

if __name__ == "__main__":
  if len(argv)<=1:
    print ("Usage: python "+ argv[0] + " WAV-file")
    exit()
  filename = argv[1]
  print ("extract position markers from file " +filename)
  findcues(filename)
