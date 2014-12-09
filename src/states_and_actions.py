# overall range is 0d to 180d (20 to 160 actually?)
# valence {positive, negative} each represented by a range over posture
#		positive: {80, 140} center at 110
#		negative: {40, 100} center at 70
# intensity {strong, weak} represented by more or less exaggerated motions
#		strong: +20%
#		weak: 	-20%


# actuator ranges
# constants. MOV_RANGE is base number of degrees of a movement. modulated by intensity

FRAMES = 20
PRP_MAX = 180
PRP_MIN = 0

POS_VAL = 120 #(80.0, 180.0)
NEG_VAL = 60  #(0.0, 100.0)
NEU_VAL = 90  #(40.0, 140.0)

STRONG_INT	= 1.2
WEAK_INT	= 0.8
NEU_INT		= 1.0

MOV_RANGE = (PRP_MAX - POS_VAL)/STRONG_INT

# (pan, tilt, roll bop), neutral values
default_pos = (90, 90, 90, 0)
current_pos = [90, 90, 90, 0]

# primitive action list:

# current_pos is always a 4-element vector representing absolute
# positions for all DoF

def goto_resting(valence, intensity, speed=1.0):
	frames = int(FRAMES/speed)
	tilt_range = MOV_RANGE*intensity
	mov_per_frame = int(tilt_range/frames)
	
	mov = list()
	for i in range(frames):
		mov.append([0, mov_per_frame, 0, 0])
	return mov

def lookup(valence, intensity, speed=1.0,):
	frames = int(FRAMES/speed)
	tilt_range = MOV_RANGE*intensity
	mov_per_frame = int(tilt_range/frames)
	
	mov = list()
	for i in range(frames):
		mov.append([0, mov_per_frame, 0, 0])
	return mov

def lookdown(valence, intensity, speed=1.0):
	frames = int(FRAMES/speed)
	tilt_range = MOV_RANGE*intensity
	mov_per_frame = int(-(tilt_range/frames))
	
	mov = list()
	for i in range(frames):
		mov.append([0, mov_per_frame, 0, 0])
	return mov

def lookleft(valence, intensity, speed=1.0):
	frames = int(FRAMES/speed)
	pan_range = 15*intensity
	mov_per_frame = int(pan_range/frames)
	
	mov = list()
	for i in range(frames):
		mov.append([mov_per_frame, 0, 0, 0])
	return mov
	
def lookright(valence, intensity, speed=1.0):
	frames = int(FRAMES/speed)
	pan_range = 30*intensity
	mov_per_frame = int(-(pan_range/frames))
	
	mov = list()
	for i in range(frames):
		mov.append([mov_per_frame, 0, 0, 0])
	return mov
	
def lieleft(valence, intensity, speed=1.0):
	frames = int(FRAMES/speed)
	tilt_range = MOV_RANGE*intensity
	mov_per_frame = int(tilt_range/frames)
	
	mov = list()
	for i in range(frames):
		mov.append([0, 0, mov_per_frame, 0])
	return mov
	
def lieright(valence, intensity, speed=1.0):
	frames = int(FRAMES/speed)
	tilt_range = MOV_RANGE*intensity*1.2
	mov_per_frame = -1*(int(tilt_range/frames))
	
	mov = list()
	for i in range(frames):
		mov.append([0, 0, mov_per_frame, 0])
	return mov

def bopup(valence, intensity, speed=1.0):
	center = valence
	current_pos = [center, center, center, 1]
	return current_pos

def bopdown(valence, intensity, speed=1.0):
	center = valence
	current_pos = [center, center, center, 0]
	return current_pos


# dictionaries mapping string input from audio controller to valence, intensity, and action
# all functions must have the same two arguments: (valence, intensity) for this to work

valence_list = {
	'stop'		: NEU_VAL,
	'positive'	: POS_VAL,
	'negative'	: NEG_VAL}

intensity_list = {
	'stop'		: NEU_INT,
	'strong'	: STRONG_INT,
	'weak'		: WEAK_INT}

action_list = {
	'stop'		: goto_resting,
	'look up'	: lookup,
	'look down'	: lookdown,
	'look left'	: lookleft,
	'look right': lookright,
	'tilt left' : lieleft,
	'tilt right': lieright,
	'extend' 	: bopup,
	'shrink' 	: bopdown}

if __name__ == "__main__":
	print(lookup(NEU_VAL, NEU_INT))

