from psychopy.sound.backend_ptb import SoundPTB as Sound
from psychtoolbox import GetSecs, WaitSecs
from RTBox import RTBox
import numpy as np

TEST_MODE = False # set to true for timing tests, so same stimulus always plays

# I'll often set the seed to the subject number so trial order for  
np.random.seed(0) # each subject can be recreated exactly

box = RTBox() # device handle with which to send TTL triggers

# randomly assign trials to oddball condition
oddball = True
trials = np.random.choice(
	np.array([oddball, not oddball]), 
	size = 200, 
	p = [1., 0.] if TEST_MODE else [.25, .75]
	)


# start the experiment
WaitSecs(5.)
for i, trial in enumerate(trials):

	print("Starting trial %d . . . %s"%(i, trial is oddball))

	# select stimulus
	if trial is oddball:
		snd = Sound('A')
		trigger = 0b10
	else:
		snd = Sound('C')
		trigger = 0b100

	## Note that it's important to have the same code handle stimulus
	## presentation in both conditions, so that timing cannot vary
	## systematically between conditions!

	# schedule sound 
	now = GetSecs()
	snd.play(when = now + 0.5) 

	# try to send TTL trigger at same time as sound
	WaitSecs(.499)
	box.TTL(trigger) 

	# wait for next trial
	WaitSecs(.5 + np.random.uniform(0, 1))

box.close()
print("Done.")








