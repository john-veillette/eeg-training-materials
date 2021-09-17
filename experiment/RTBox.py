#!/usr/bin/env python
"""This controls the response time box (RTBox) shown at
https://github.com/xiangruili/RTBox_py 
    box = RTBox.RTBox() # open RTBox
Then check the methods for box for detail information.
Packages required: numpy and pynput
170402 By Xiangrui Li (xiangrui.li at gmail.com)
170505 ready to publish to users
171023 start to use ftd2xx lib
200614 Impement digitalIn for version 5.23 and 6.12 """

__version__ = '2020.06.16'
_instances = [] # store RTBox instances

import numpy as np
from pynput import keyboard
try: import serFTDI; use_serFTDI = serFTDI.Accessible()
except: use_serFTDI = False
if not use_serFTDI: 
    try: del serFTDI
    except: pass

def sys_cmd(cmd): # only needed for latency timer for now
    """ Call system command 'cmd', and return screen output as string """
    import subprocess as sp
    try:
        si = sp.STARTUPINFO()
        si.dwFlags |= sp.STARTF_USESHOWWINDOW # avoid showing cmd window
        out = sp.check_output(cmd, startupinfo=si, stderr=sp.STDOUT) # faster than os.popen
        return out.decode('utf-8') # decode() needed for python3    
    except:
        return sp.os.popen(cmd).read() # fallback for earlier python    

def keyName():
    """Show the name of a pressed key on the keyboard. The name can be used
    for waitKeys(keys) and the RTBox button names at keyboard simulation mode.
    Caps lock will change the letter keys to upper case (A...Z). Some unknown
    keys will be reported as 'None'.
    """
    def on_press(key):
        try: print(key.char)
        except: print(key.name)
        return False # stop listener after a keypress
     
    print('Press a key on keyboard to show its name')
    with keyboard.Listener(on_press=on_press) as lis: lis.join()

def _esc_exit(lis):
    if not lis.esc_exit: return
    if lis.running: lis.stop()
    raise KeyboardInterrupt('User pressed ESC. Exiting ...')

class RTBox(object):
    """Open serial port, return RTBox instance for later access.
    One of the optional input, host_clock, is the host clock for RTBox, like
        box = RTBox.RTBox(time.perf_counter) # after clock imported in script        
    This is the clock used for stimulus onset. Its syntax is the same as the
    method you get the time in seconds in your code. For example, if you import 
        from psychopy import core
    you will do t=core.getTime(), then you should provide the clock to RTBox by
        box = RTBox.RTBox(core.getTime) # WITHOUT parenthesis
    If no clock is provided during opening, RTBox tries following in order:
        1. psychopy.core.getTime()
        2. time.perf_counter()
        3. time.time()
    After opening, box.hostClock() will show the host clock used by RTBox.
    If the clock is chosen by RTBox, the method to get stimulus onset time is to
    call box.hostSecs() after stimulus onset:
        win.flip() # turn stimulus on
        tOnset = box.hostSecs() # get time immediately after flip()
    Or put 2 commands in one line:
        win.flip(); tOnset = box.hostSecs()
    Another optional input, boxID (default 0), is the ID of the box. It is useful
    at two cases. One case is to use multiple RTBox. To open 2nd RTBox, boxID
    needs to be 1 (different from default 0). For example:
        box0 = RTBox.RTBox() # open 1st box with default boxID=0
        box1 = RTBox.RTBox(boxID=1) # open 2nd one with boxID=1 (can be str too)
    Another case is to use RTBox.py at fake mode by setting boxID to None or '':
        box = RTBox.RTBox(boxID='') # keyboard simulates RTBox
    This allows to test stimulus code without RTBox hardware connected. Then one
    can use keyboard to simulate response without RTBox-not-found exception.
    """

    def __new__(cls, host_clock=None, boxID=0):
        for box in _instances:
            if box._p.boxID==boxID: return box # existing instance
        return super(RTBox,cls).__new__(cls) # create new instance
       
    def __init__(self, host_clock=None, boxID=0):
        if self in _instances: return
        
        class _default:
            """ Default parameters for RTBox """
            TTLresting = [0, 1] # persist in firmware. Info only at host side
            TTLWidth = 0.00097
            debounceInterval = 0.05
            threshold = 1
            
            nEventsRead = 1 # in host code only
            untilTimeout = False
            events = ['1', '2', '3', '4', '1', '2', '3', '4', 'sound', 'light', '5', 'aux', 'serial']
            enabled = [1, ['press', 'release', 'sound', 'light', 'tr', 'aux']] # press enabled
            is_open = False
            
            MAC = bytearray([0]*7) # host computer related params
            from uuid import getnode
            mac = getnode()
            for i in range(6): MAC[6-i] = mac>>(i<<3) & 0xff
            clkRatio = np.float64(1)
            latencyTimer = 0.016
            hostClock = None
            import platform as pf
            sysinfo = pf.platform() + '\nPython ' + pf.python_version()
            
        self._p = _default()
        self._p.boxID = boxID
        self._p.fake = boxID==None or boxID=='' or boxID==[]
        self.hostClock(host_clock)
        if self._p.fake: return
            
        (ser, v) = self._openBox()
        if v<=1.9 or (v>2 and v<4.3):
            ser.write(b'x')
            ser.close()
            raise EnvironmentError('Unsupported firmware version. Please get\n' +
                    'the latest firmware at http://lobes.osu.edu/rt-box.php')
        self._p.version = v
        self._p.is_open = True
        self._ser = ser

        if use_serFTDI:
            self._p.latencyTimer = self._ser._info.LatencyTimer         
        else:
            lat = self._latencyTimer()
            self._p.latencyTimer = lat / 1000.0
            if lat>2: print('Please change the USB serial port latency timer to 2 ms ' +
                            'following the instruction in User\'s Manual')
        
        try: # get TTL params, threshold, debounceInterval from EEPROM
            b = self._readEEPROM(224, 6)
            self._p.TTLWidth = (255-b[0]) / 7200.0 if b[0]<255 else float('Inf')
            self._p.TTLresting = [b[1]&1, b[1]>>1 & 1]
            self._p.threshold = (b[1]>>3 & 1) + (b[1]>>5 & 2) + 1
            self._p.debounceInterval = sum([int(b[i+2])<<(i<<3) for i in range(4)]) / 921600.0
        except: pass
        
        try: # get clkRatio from EEPROM 
            for i in range(16): # max number of MAC to store
                b14 = self._readEEPROM(i*14, 14);
                if b14[:8].count(b'\0xff') ==8: break # EEPROM not written
                if self._p.MAC[1:]==b14[8:] : break # found its MAC loc
            
            if i==15: i = 0 # all slots written
            self._p.MAC[0] = i*14 # ratio location in EEPROM
            ratio = np.frombuffer(b14[:8])[0]
            if np.sys.byteorder == 'big': ratio = np.frombuffer(b14[7::-1])[0]
            if abs(ratio-1)<0.01: self._p.clkRatio = ratio
        except: self._p.MAC[0] = 28 # avoid first slot
        
        if self._p.clkRatio==1:
            print ('Please run box.clockRatio() for better accurracy')
        self._p.sync = self.clockDiff()
        if self._p.sync[1] > 2.6e6: self.reset() # 1 month power on
        _instances.append(self)

    def close(self):
        """ Close RTBox serial port """
        if self in _instances: _instances.remove(self)
        if not self._p.is_open: return
        self._ser.write(b'x') # switch to simple mode
        self._p.is_open = False
        self._ser.close()
        #print('RTBox %s at %s closed' % (self._p.boxID, self._ser.port))
    
    def reset(self):
        """Restart RTBox firmware, so set its clock to zero. This is rarely needed."""
        if self._p.fake: return
        self._ser.write(b'xBS') # simple mode, boot, bootID
        self._ser.write(b'R') # return, so restart
        self._ser.write(b'X') # advanced mode
        self.clear() # also restore events

    def clockDiff(self, n=9):
        """Measure difference between host clock and RTBox clock without applying it.
            Input: number of trials to measure
            Return: [clockDiff, boxSecs_of_measurement, clockDiff_upper_bound] """
        if self._p.fake: return [0.0, self.hostSecs(), 0.0]
        self._enableEvents(0) # disable all
        if n>585:
            print('nSyncTrial too big. Reduced to 585')
            n = 585
        t = np.empty((n,3)) # tpre, tpost, tbox
        for j in range(5): # in case of error, try several times
            for i in range(n):
                self.waitSecs(0.001*(np.random.rand()+0.7)) # 0.7 allow 7-byte finish
                (t[i,0], t[i,1]) = self._write_time(b'Y')
                
            b = bytearray(self._ser.read(7*n))
            if len(b)==7*n and all(x==89 for x in b[::7]): break
            if j==4: raise EnvironmentError('\n Failed to communicate with device.')
            self._purge()
            
        for i in range(n): t[i,2] = self._bytes2secs(b[i*7:(i+1)*7]) # tbox
        self._enableEvents() # restore event detection
        dt = t[:,0] - t[:,2] # tpre-tbox, expect constant
        i = np.argmax(dt) # closest to end of 1ms write cycle
        dt = dt[i] + 10.0/115200 # 8+2 bits serial write
        return [dt, t[i,2], t[i,1]-t[i,0]]

    def eventsAvailable(self):
        """ Return number of events in serial buffer.
        Number in fraction means event is still coming. """
        if not self._p.is_open: return 0.0
        return self._bytesAvailable() / 7.0

    def _write_time(self, b): # take care of pySerial
        if use_serFTDI:
            return self._ser.write(b)
        else:            
            tpre = self.hostSecs()
            self._ser.write(b)
            while self._ser.out_waiting>0: pass
            tpost = self.hostSecs()
            return (tpre, tpost)
 
    def TTL(self, eventCode=1):
        """Send TTL to pins 1~8 at DB25 port.
            Input: event code to send, 0~255 (0~15 for version before 5)
            Return: sending time and its upper bound.
        Examples:
            [tSend, ub] = box.TTL(0b1011) # use binary event code
            box.TTL(255) # in BioSemi system, 255/254 controls recording
        """
        if self._p.fake: return [self.hostSecs(), 0.0]
        b = int(eventCode)
        v = self._p.version
        b = [1, b] if v>=5 else [b]
        (tpre, tpost) = self._write_time(bytearray(b))

        b = b[-1] # check error after sending anyway
        t_write = 20.0/115200 if v>=5 else 10.0/115200
        if b<0 or (b>15 and v<5) or b!=round(eventCode):
            raise ValueError('Invalid TTL code %s' % eventCode)
        return [tpre+t_write, tpost-tpre]
    
    def clockRatio(self, n=30):
        """ Measure and apply clock ratio for better timing accuracy.
            Input: number of measure trials (seconds)
            Return: clock unit ratio: hostSecs/boxSecs"""
        if n<1 or self._p.fake: return self._p.clkRatio
        print('Clock ratio test will take %g seconds. Press ESC to stop' % n)
        lis = self.keyQueue([]) # only check ESC
        t = np.empty((n,2))
        for i in range(n): # sync once every second
            self.waitSecs(0.98) # each measure takes ~20ms
            _esc_exit(lis)
            t[i,:] = self.clockDiff(20)[:2]
        lis.stop()
        
        t[:,0] -= np.mean(t[:,0])
        t[:,1] -= np.mean(t[:,1])
        c = np.polyfit(t[:,1], t[:,0], 1) # linear fit
        self._p.clkRatio *= 1+c[0] # apply slope
        self._p.sync = self.clockDiff() # sync with new ratio
        
        if n>=20: # save ratio to EEPROM only when n is large
            b = list(np.frombuffer(self._p.clkRatio, np.uint8))
            if np.sys.byteorder == 'big': b = b[::-1]
            self._writeEEPROM(self._p.MAC[0], bytearray(b)+self._p.MAC[1:])
        return self._p.clkRatio
    
    def enable(self, events=None):
        """Enable detection of one or more events.
            Input: one or more of ['press', 'release', 'sound', 'light', 'TR', 'aux']
              If there no input, it means to query enabled events.
            Return: enabled events """
        return self._enable_disable(events, True)

    def disable(self, events=None):
        """ Disable detection of one or more events.
            Input: one or more of ['press', 'release', 'sound', 'light', 'TR', 'aux'].
                      Input 'all' will disable all events.
               If there no input, it means to query enabled events.
            Return: enabled events """
        return self._enable_disable(events, False)
        
    def _eventsEnabled(self, byt=None):
        if byt==None: byt = self._p.enabled[0]
        return [self._p.enabled[1][i] for i in range(6) if byt&(1<<i)]
        
    def _enable_disable(self, events, isEnable):
        """ called by enable() and disable() """
        if events==None: return self._eventsEnabled()
        
        if isinstance(events, str) and events.lower()=='all':
            events = self._p.enabled[1]
        if not isinstance(events, list): events = [events] 
        byt = self._p.enabled[0]
        for ev in events:
            try: i = self._p.enabled[1].index(ev.lower())
            except: raise ValueError('Invalid event input: '+ev)
            if isEnable: byt |= 1<<i
            else: byt &= ~(1<<i)
        self._p.enabled[0] = byt
        if (byt & 3) == 3: # both press and release
            for i in range(4,8): self._p.events[i] = self._p.events[i-4] + 'up'
        elif (byt & 2) == 2: # relese, but not press
            self._p.events[4:8] = self._p.events[:4]
        if not self._p.fake: self._enableEvents()
        return self._eventsEnabled()

    def clear(self, nSyncTrial=9):
        """Clear serial buffer to prepare for a trial. 
        This is designed to be called right before the stimulus onset.
        Also implicitly synchronize clocks and enable trigger if applicable.
            Input: number of trial to synchronize clocks.
            Return: none """
        if self._p.fake: return
        if nSyncTrial>0: self._p.sync = self.clockDiff(nSyncTrial)
        elif self._p.enabled[0]>3: self._enableEvents() # trigger enabled
        else: self._purge()

    def _read(self, tout, cmd):
        """Wrapper called by other read functions"""
        if self._p.untilTimeout: dt = tout - self.hostSecs()
        else: dt = tout; tout += self.hostSecs()
        nEventsRead = self._p.nEventsRead
        try: 
            iCmd = self._p.enabled[1].index(cmd)
            nEventsRead += 1 # detect 1 more event for trigger
        except ValueError:
            iCmd = False # not trigger 
        
        if iCmd: # error check only
            if iCmd<2:
                raise ValueError('Invalid trigger ' + cmd)
            if not (self._p.enabled[0] & (1<<iCmd)):
                raise ValueError('Trigger %s is disabled' % cmd)
                
        if self._p.fake: return self.waitKeys(self._p.events[:4])
            
        empty = (np.array([]), [])
        tt = np.array([]); ev = []
        nB = self._bytesAvailable()
        if dt>0.2: lis = self.keyQueue([]) # detec ESC if long wait
        while nB<nEventsRead*7 and self.hostSecs()<tout:
            self.waitSecs(0.001+self._p.latencyTimer)
            if dt>0.2: _esc_exit(lis)
            nB = self._bytesAvailable()
        if dt>0.2: lis.stop()
        
        nEvent = nB // 7
        if nEvent<nEventsRead: return empty # give up if not enough events
            
        CODES = [49, 51, 53, 55, 50, 52, 54, 56, 97, 48, 57, 98, 89] # for _p.events
        for i in range(nEvent): # decode event and time
            b7 = bytearray(self._ser.read(7))
            try:
                ind = CODES.index(b7[0])
            except ValueError:
                print('Invalid event code: %g' % b7[0])
                break # not continue, rest must be messed up
            
            ev.append(self._p.events[ind])
            tt = np.append(tt, self._bytes2secs(b7))
            
        if len(tt)<1: return empty
        
        if cmd=='secs': # convert into host time
            if self._p.clkRatio==1 and tt[-1]-self._p.sync[1]>9 : # sync too long ago
                sync = np.empty((2, 2))
                sync[0,:] = self._p.sync[:2] # get previous sync
                self._p.sync = self.clockDiff() # update sync
                sync[1,:] = self._p.sync[:2] # new sync
                tdiff = np.interp(tt, sync[:,1], sync[:,0]) # linear interp
            else:
                tdiff = self._p.sync[0]
            tt = tt + tdiff # boxSecs to hostSecs
        elif iCmd: # relative to trigger
            if len(tt)<2: return empty # if only trigger event, return empty
            try: ind = ev.index(cmd) # trigger index
            except: return empty
            trigT = tt[ind] # time of trigger event
            tt = np.delete(tt, ind); ev.pop(ind) # omit trigger and its time from output
            tt = tt - trigT # relative to trigger time
        
        return (tt, ev)

    def secs(self, timeout=0.1):
        """Read events (buttons and more) and get time based on box.hostClock().
            Input: timeout for the read.
            Return: (secs, events), where secs is an array of time, and 
                    events is list of events. Both have the same length, 
                    and each entry  is for an event.
        Example: (secs, events) = box.secs(2)
           Read response, and return either when required number of response (defined by
           box.nEventsRead()) is avaible, or 2 seconds elapse. If there is no response, 
           both output will have len of 0. The returned secs is based on box.hostClock().
           If the stimulus onset is based on another clock, one must use that clock when
           opening the RTBox:
             box = RTBox.RTBox(your_host_clock)
        During event wait, pressing ESC will raise KeyboardInterrupt exception."""
        return self._read(timeout, 'secs')

    def boxSecs(self, timeout=0.1):
        """Read events (buttons and more) and get time based on RTBox clock.
        The input and return are the same as box.secs()."""
        return self._read(timeout, 'boxsecs')

    def sound(self, timeout=0.1):
        """Read events (buttons and more) and get time relative to sound trigger.
        The input and return are the same as box.secs(). """
        return self._read(timeout, 'sound')

    def light(self, timeout=0.1):
        """Read events (buttons and more) and get time relative to light trigger.
        The input and return are the same as box.secs(). """
        return self._read(timeout, 'light')

    def aux(self, timeout=0.1):
        """Read events (buttons and more) and get time relative to aux trigger.
        The input and return are the same as box.secs(). """
        return self._read(timeout, 'aux')

    def TR(self, timeout=0.1):
        """Read events (buttons and more) and get time relative to TR trigger.
        The input and return are the same as box.secs()."""
        return self._read(timeout, 'tr')

    def waitTR(self):
        """Wait for TR trigger or key press of TR key (default '5'), and return the time.
        There is no need to enable/disable TR for this. ESC will break the wait. """
        lis = self.keyQueue(self._p.events[10])
        if self._p.fake:
            while len(lis.key_time)<1:
                _esc_exit(lis)
                self.waitSecs(0.008)
            lis.stop()
            return lis.key_time[0]
        
        self._enableEvents(16) # only TR enabled
        while self._bytesAvailable()<7 and len(lis.key_time)<1:
            self.waitSecs(0.008)
            if lis.esc_exit: self._enableEvents(); _esc_exit(lis)
        lis.stop()
            
        if self._bytesAvailable()<7: # TR key pressed
            self._enableEvents()
            return lis.key_time[0]
                
        b7 = bytearray(self._ser.read(7))
        self._p.sync = self.clockDiff() # sync since normally long wait, enable events
        return self._bytes2secs(b7)+self._p.sync[0] 

    def buttonDown(self):
        """Check if buttons are pressed. Return array of length 4, where 1 means pressed.
        """
        if self._p.fake: return [0]*4
        if self._p.enabled[0]: self._enableEvents(0) # disable all
        self._ser.write(b'?') # ask button state
        b = bytearray(self._ser.read(2)) # returns 2 bytes
        self._enableEvents() # enable detection
        if b[1]==63: b[1] = b[0] # '?' can be 2nd byte for old version 
        v = self._p.version
        if v>=4.7 or (v>1.9 and v<2): return [b[1]>>i & 1 for i in range(4)]
        else: return [b[1]>>i & 1 for i in range(4,8)]

    def digitalIn(self, toReverse=False):
        """Return the digital input from pins 1~8 at DA-15 port.
        The pins 1~4 are also connected to button 1~4. All 8 pins are pulled up,
        so the original high level means resting state. If the optional input,
        toReverse is provided to and is ture, this will return reversed level.
        """
        if self._p.fake: return None
        v = self._p.version
        if v<5: print ('digitalIn supported for version 5+.'); return None
        if v<5.22 or (v>6 and v<6.12): print ('Please update the RTBox firmware.'); return None

        while True:
            self._purge()
            self._ser.write(chr(8))
            b = bytearray(self._ser.read(2)) # return 2 bytes
            if len(b)==2 and b[0]==8: break
        if toReverse: return np.uint8(255-b[1])
        return np.uint8(b[1])

    def enableState(self):
        """Return the enabled events in the hardware.
        This may not be consistent with those returned by box.enable(),
        since an external trigger will disable the detection of itself in 
        the hardware. box.clear() will enable the detection implicitly.
        This is mainly for debug purpose. """
        if self._p.fake: return self._eventsEnabled()
            
        while True: # can't disable events
            self._ser.write(b'E') # ask enable state
            b = bytearray(self._ser.read(2)) # return 2 bytes
            if len(b)==2 and b[0]==69: break
            self._purge()
        return self._eventsEnabled(b[1])

    def threshold(self, thr=None):
        """Get/set sound and light threshold for hardware version >=5.
        It can be 1, 2, 3 or 4. If the threshold is so low that the background 
        light trigger the light detection, one can set this to a higher value.
           box.threshold() # query the threshold
           old = box.threshold(4) # set to highest, return previous threshold
        The setting will persist even after power off. """
        oldVal = self._p.threshold
        if thr==None: return oldVal
        if not isinstance(thr,(int, float)) and len(thr)<1: thr = 1
        elif isinstance(thr, list) and len(val)>1: thr = thr[0]
        if thr<1 or thr>4: raise ValueError('Invalid threshold input')
        thr = int(thr) 
        self._p.threshold = thr
        thr -= 1
        pol = self._p.TTLresting
        b = ((thr&2)<<5) + ((thr&1)<<3) + (pol[1]<<1) + pol[0]
        self._writeEEPROM(225, [b])
        return oldVal

    def TTLResting(self, pol=None):
        """Get/set TTL resting level (polarity) in the hardware. There are two values
        like [0, 1], the first for polarity at DB25 port pins 1~8, and the second 
        for that at pins 17~24. Value 0 means low resting, and 1 high resting.
           box.TTLResting() # query the polarity
           box.TTLResting([0, 0]) # set both polarity to low resting 
        The setting will persist even after power off. """
        oldVal = self._p.TTLresting
        if pol==None: return oldVal
        if not isinstance(pol,(int, float)) and len(pol)<1: pol = [0,1]
        if len(pol)<2: pol = [pol, 1]
        self._p.TTLresting = pol
        thr = self._p.threshold - 1
        b = ((thr&2)<<5) + ((thr&1)<<3) + (pol[1]<<1) + pol[0]
        self._writeEEPROM(225, [b])
        return oldVal

    def TTLWidth(self, width=None):
        """Get/set TTL width in seconds in the hardware.
           box.TTLWidth() # query width, normally about 1~2 ms.
           old = box.TTLWidth(0.01) # set width to 10 ms. 
        The setting will persist even after power off.
        Valid width ranges from 0.14 to 35 ms. Infinite width is also supported. 
        Infinite width means the TTL level will stay until changed by next TTL 
        command. """
        oldVal = self._p.TTLWidth
        if width==None: return oldVal
        if not isinstance(width,(int, float)) and len(width)<1: width = 0.000972
        b0 = width * 7200.0 # 7200 timer2 rescaler
        b = max(1, b0); b = min(b, 255); b = np.uint8(b+0.499) # 1 to 255
        realW = b / 7200.0
        if width == float('Inf'): # special case
            realW = width
            b = 0
        elif b0<1 or b0>255:
            print('TTL width out of range. Adjusted to %.2g seconds' % realW)
        self._writeEEPROM(224, [255-b])
        self._p.TTLWidth = realW
        return oldVal

    def debounceInterval(self, intvl=None):
        """Get/set debouncing interval in seconds in the hardware.
           box.debounceInterval() # query interval
           old = box.debounceInterval(0.1) # set interval to 100 ms. 
        The setting will persist even after power off. """
        oldVal = self._p.debounceInterval
        if intvl==None: return oldVal
        if not isinstance(intvl,(int, float)) and len(intvl)<1: intvl = 0.05
        if intvl<0: raise ValueError('Invalid debounce interval value')
        b = int(intvl*921600 + 0.5)
        b = [b>>i & 0xff for i in range(0,32,8)]
        self._writeEEPROM(226, b)
        self._p.debounceInterval = intvl
        return oldVal

    def untilTimeout(self, useUntil=None):
        """Get/set the timeout method for read command.
          box.untilTimeout() # get current method, default False, meaning 
            (secs, event) = box.secs(time_out) will wait time_out seconds if no response.
          box.untilTimeout(True) # Use until-time, meaning read command will wait till 
             box.hostSecs() reach time_out. """
        oldVal = self._p.untilTimeout
        if useUntil==None: return oldVal
        if not isinstance(useUntil,(int, float)) and len(useUntil)<1:
            useUntil = False
        self._p.untilTimeout = bool(useUntil)
        return oldVal

    def nEventsRead(self, nRead=None):
        """Get/set number of event for all read command.
          box.nEventsRead() # get current number, default 1
          box.nEventsRead(2) # all read command will wait for two events
        """
        oldVal = self._p.nEventsRead
        if nRead==None: return oldVal
        if not isinstance(nRead,(int, float)) and len(nRead)<1: nRead = 1
        self._p.nEventsRead = nRead
        return oldVal

    def buttonNames(self, newNames=None):
        """Set/get four button names. The default names are ['1', '2', '3', '4']. You
        can use any names except those reserved for other events like 'sound',
        'pulse', 'light', '5', and 'serial'. If no names are passed, it means to 
        query current button names. Following example makes four buttons like two:
            box.buttonNames(['left', 'left', 'right', 'right'])
        If you use keyboard to simulate RTBox buttons, the button names must be key
        names you will use. The above example will allow to use left and right
        arrow keys to simulate if you set to RTBox fake mode 
            box = RTBox.RTBox(fake=True)
        RTBox.keyName() can give the name of a key. """
        oldVal = self._p.events[:4]
        if newNames==None: return oldVal
        if not isinstance(newNames,list) or len(newNames)!=4:
            raise ValueError('Invalid input for 4 button names')
        self._p.events[:4] = map(str, newNames) # in case input are numbers
        if (self._p.enabled[0] & 3) == 3: # both press and release enabled
            for i in range(4,8): self._p.events[i] = self._p.events[i-4] + 'up'
        else:
            self._p.events[4:8] = self._p.events[:4]
        return oldVal

    def TRKey(self, newKey=None):
        """Set/get the keyboard key equivalent to TR trigger. The default is '5'. This 
        name is used as the event name for TR trigger, as well as the key press during
            box.waitTR()
        Note that the change won't persist, and needs to do it each time a new
        instance is created."""
        oldVal = self._p.events[10]
        if newKey==None: return oldVal
        if not isinstance(newKey, str): raise ValueError('Invalid input for TR key')
        self._p.events[10] = newKey
        return oldVal

    def info(self):
        """Display some information of the system and device.
        Provide the screen output when you report a problem."""
        print(self._p.sysinfo)
        print(('MAC address(%g): '+'%02X-'*5+'%02X') % tuple(self._p.MAC))
        print('RTBox.py Rev: '+ __version__)
        print('Host clock: ' + self._p.hostClock)
        print('Events enabled: %s' % self.enable())
        print('Number of event to wait: %g' % self._p.nEventsRead)
        print('Use until-timeout for read: %s' % self._p.untilTimeout)
        if self._p.fake:
            print('RTBox.py at fake mode: True')
            return
        print('boxID: %s, v%g' % (self._p.boxID, self._p.version))
        print('Serial port: %s' % self._ser.port)
        print('hostClock/boxClock unit ratio - 1: %.2g' % (self._p.clkRatio-1))
        print('hostSecs-boxSecs offset: %.5f + %.5f' % (self._p.sync[0],self._p.sync[2]))
        print('Latency Timer: %g' % self._p.latencyTimer)
        print('Debouncing interval: %.2g' % self._p.debounceInterval)
        if self._p.version>=3:
            print('TTL resting: %s' % self._p.TTLresting)
            print('TTL width: %.2g' % self._p.TTLWidth)
        if self._p.version>=5:
            print('Sound/light threshold: %g' % self._p.threshold)
        print('Number of events available: %.2g' % self.eventsAvailable())

    def _bytesAvailable(self): # take care of pySerial version change
        if use_serFTDI:
            return self._ser.bytesAvailable()
        else:
            try: return self._ser.in_waiting
            except: return self._ser.inWaiting()

    def _purge(self):
        n, n1 = -1, self._bytesAvailable()
        while n1>n:
            self.waitSecs(self._p.latencyTimer+0.002)
            n, n1 = n1, self._bytesAvailable()
        if n>0: self._ser.read(n) # avoid _ser.purge()

    def _readEEPROM(self, addr, nBytes):
        """Read 'nBytes' bytes from RTBox EEPROM at address 'addr'"""
        self._ser.write(bytearray([17, addr, nBytes]))
        return bytearray(self._ser.read(nBytes)) 

    def _writeEEPROM(self, addr, bList):
        """Write byte list to RTBox EEPROM at address 'addr'"""
        if self._p.fake: return
        self._ser.write(bytearray([16, addr, len(bList)]))
        self._ser.write(bytearray(bList))
        self._ser.write(bytearray([3, 2])) # extra 2 useless bytes
        self._purge()

    def _enableEvents(self, byt=None):
        """ Send a byte to control event detection in firmware """
        if byt==None: byt = self._p.enabled[0]
        self._purge()
        self._ser.write(bytearray([101, byt])) # 'e'
        self._ser.read(1) # clear returned 'e'
    
    def _bytes2secs(self, b):
        """ Convert 7-byte data into box secs """
        ticks = sum([int(b[6-i]) << (i<<3) for i in range(6)])
        return ticks/921600.0*self._p.clkRatio
        
    def _latencyTimer(self):
        """Return USB serial port latency timer in ms"""
        port = self._ser.port
        myOS = self._p.sysinfo.lower()
        try:
            if myOS.startswith('windows'):
                port = port.rsplit('\\', 1)[-1]
                ftdi = 'HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\FTDIBUS'
                s = sys_cmd('reg.exe query "%s" /s /e /f %s' % (ftdi, port))
                if len(s)<1: # old reg.exe
                    p = '| findstr /e "' + port + ' Parameters"'
                    s = sys_cmd('reg.exe query %s /s %s' % (ftdi, p))
                s = s.rsplit(port, 1)[0].rsplit(ftdi, 1)[-1]
                key = ftdi + s.splitlines()[0] # path for the port info
                s = sys_cmd('reg.exe query "%s" /v LatencyTimer' % key)
                s = s.strip().rsplit(' ', 1) # last is 0x2 or 0x10
                if len(s)<2 or not s[-1].startswith('0x'): return 16
                return int(s[-1], 16) # hex type
            elif myOS.startswith('linux'):
                import os
                nam = '/etc/udev/rules.d/psychtoolbox.rules'
                if os.path.isfile(nam):
                    with open(nam) as fid: s = fid.read()
                    s = s.rsplit('ATTR{latency_timer}="', 1)[-1].split('"', 1)[0]
                    return int(s) # suppose it is 1 ms, so no change needed

                pth = os.path.dirname(os.path.abspath(RTBox.__file__))
                print('Please run following once at shell to set short latency_timer:\n' +
                      'sudo cp "' + pth + '/psychtoolbox.rules" /etc/udev/rules.d/.')
                
                port = port.rsplit('/', 1)[-1]
                s = '/sys/bus/usb-serial/devices/' + port + '/latency_timer'
                lat = int(sys_cmd('cat ' + s)) # error if empty
                if lat<=2: return lat
                
                sys_cmd('echo 2 > ' + s) # change it to 2
                return int(sys_cmd('cat ' + s))
            elif myOS.startswith('darwin'):
                from os.path import isfile
                useFTDI = True # use FTDI driver
                nam = '/Library/Extensions/FTDIUSBSerialDriver.kext/Contents/Info.plist' # later OS
                if not isfile(nam):
                    nam = '/System/Library/Extensions/FTDIUSBSerialDriver.kext/Contents/Info.plist'
                if not isfile(nam):
                    useFTDI = False # driver from Apple: different keys
                    nam = '/System/Library/Extensions/AppleUSBFTDI.kext/Contents/Info.plist'
                if not isfile(nam):
                    useFTDI = False # driver from Apple: different keys
                    nam = '/System/Library/Extensions/IOUSBFamily.kext/Contents/PlugIns/AppleUSBFTDI.kext/Contents/Info.plist'
                if not isfile(nam): 
                    print('Don\'t know Info.plist folder used by OS')
                    return 16
                #print('Info.plist used by OS: ' + nam)
                with open(nam) as fid: s = fid.read()
                
                if useFTDI:
                    i = s.find('<key>FTDI2XXB')
                    if i==-1: i = s.find('<key>FT2XXB')
                else:
                    i = s.find('<key>AppleUSBEFTDI-6001')

                s = s[i:].split('</dict>', 1)[0] # end at ConfigData or FTDI key
                i = s.find('<key>LatencyTimer</key>')
                i = s[i:].find('<integer>') + i + 9
                return int(s[i:].split('</integer>', 1)[0])
            else: print('Unknown system: ' + self._p.sysinfo)
        except: return 16


    def hostClock(self, host_clock=None):
        """hostClock = box.hostClock() # if no input, query the host clock
        box.hostClock(time.perf_counter) # set host clock 
          The clock must be the same as that used for stimulus onset..
        The other option is to provide the clock when opening the RTBox:
          box = RTBox.RTBox(time.perf_counter) """
        hasClock = host_clock!=None
        if (not hasClock) and self._p.hostClock!=None:
            return self._p.hostClock
            
        for clk in [host_clock]: # fake loop so break out
            if hasClock: # no try keyword here: error out if not available
                if isinstance(clk, (float, int)):
                    raise ValueError('Invalid host clock input. Do not include "()"')
                elif isinstance(clk, str): # discouraged
                    modu, func = clk.split('(', 1)[0].rsplit('.', 1)
                    modu = __import__(modu, fromlist=(func))
                    clk = eval('modu.' + func); t0 = clk()
                    break
                else: # module
                    t0 = clk()
                    break
            try:
                from psychopy.core import getTime as clk
                break
            except: pass
                
            try:
                from time import perf_counter as clk
                t0 = clk() # start the clock
                break
            except:
                from time import time as clk # last resort, poor precision
                break
      
        def waitSecs(secs=0.001):
            """Wait for secs. Accuracy depends on hostSecs() accuracy."""
            from time import sleep
            tout = clk() + secs
            if secs>0.2: sleep(secs-0.2) # better for CPU?
            while clk() < tout: pass # tight polling for last 0.2 sec
        
        self.waitSecs = waitSecs # Time wait function used by RTBox
        self.hostSecs = clk # Host time used by RTBox
        if use_serFTDI:
            try: self._ser.hostSecs = clk # Host time for serFTDI
            except: pass 
        self._p.hostClock = clk.__module__ + '.' + clk.__name__
        if self._p.is_open: self._p.sync = self.clockDiff()
        return self._p.hostClock # info only

    def test(self):
        """Quick command line check for events. This will wait for incoming event,
        and display event name and time when available.
        """
        if self._p.fake: 
            print('RTBox.py is running at fake mode')
            return
        esc = self.keyQueue([])
        t0 = self.hostSecs() - self._p.sync[0]
        print(' Waiting for events. Press ESC to stop.')
        print('%9s%9s-%.4f' % ('Event', 'secs', t0))
        while not esc.esc_exit:
            (t, ev) = self.boxSecs() # avoid listening key in _read()
            for i in range(len(t)): print('%9s%12.4f' % (ev[i], t[i]-t0))

    def keyQueue(self, keys='all'):
        """lis = box.keyQueue(['space', 'q']) # queue space and q
        lis = box.keyQueue([]) # only check lis.esc_exit
        Start to queue key press on a new thread, and return the listener instance.
        Input: key list to queue, default to all keys.
        The listener needs to be stopped after done: lis.stop().
        ESC press will set lis.esc_exit to True regardless ESC is in 'keys' or not.
        """
        def on_press(key): # listener callback: store key and time
            try: k = str(key.char) # letters, numbers or symbols
            except: k = key.name # other keys
            if isinstance(keys, str) or k in keys:
                lis.key_time.append(self.hostSecs()) # key press time
                lis.key_pressed.append(k)
            if k=='esc': lis.esc_exit = True
         
        if isinstance(keys, str) and keys.lower()=='all': pass # all keys
        elif not isinstance(keys, list): keys = [keys]
        lis = keyboard.Listener(on_press=on_press) # pynput listener
        lis.key_time= []
        lis.key_pressed = []
        lis.esc_exit = False
        lis.start()
        return lis

    def waitKeys(self, keys='space'):
        """Wait for a keyboard key press, and return the key press time and key name
        in format of (time, name).
        The input can be a single key or a list of keys. RTBox.keyName() will
        show the name of a pressed key. Only the specified keys will be detected.
        Pressing ESC will raise KeyboardInterrupt exception. """
        if not isinstance(keys, list): keys = [keys]
        lis = self.keyQueue(keys) 
        while len(lis.key_time) < 1:
            _esc_exit(lis)
            self.waitSecs(0.05)
        lis.stop()
        return (np.array(lis.key_time), lis.key_pressed)

    def _openBox(self):
        """(ser, ver) = _openBox()
        Open RTBox serial port, and return the port instance and RTBox firmware version.
        Raise exception if no available RTBox port is found.
        """
        if use_serFTDI:
            self._p.sysinfo += '\nserFTDI '+ serFTDI.__version__
            ports = range(serFTDI.NumberOfPorts())
        else:
            import serial
            from serial.tools.list_ports import comports
            # p has ['COM4', 'USB Serial Port (COM4)', 'USB VID:PID=0403:6001 SER=6']
            ports = [p[0] for p in comports() if '0403:6001' in p[2]]
            try: import fcntl # take care of multiple open in unix
            except: pass
            self._p.sysinfo += '\npySerial '+ serial.VERSION
            
        inUse = [] # for error message
        for box in _instances: inUse.append(box._ser.port)
        for p in ports:
            try:
                if p in inUse: continue
                if use_serFTDI:
                    ser = serFTDI.FTD2XX(p, self.hostSecs)
                else:
                    ser = serial.Serial(p, 115200, timeout=0.3)
                    if 'fcntl' in locals():
                        try: # PTB ioctl(TIOCEXCL) the same as this?
                            fcntl.flock(ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                        except IOError: # already in use
                            ser.close() # buffer cleared when open, not ideal
                            raise serial.SerialException(p[0] + ' in use') # record inUse 
                
                ser.write(b'X')
                ID = ser.read(21) # 'USTCRTBOX,921600,v###'
                if ID.startswith(b'?'): # maybe in ADC or boot
                    ser.write(b'R')
                    ser.read(2)
                    ser.write(b'X')
                    ID = ser.read(21)
                if ID.startswith(b'USTCRTBOX'):
                    v = float(ID[18:])
                    if v>100: v /= 100
                    return (ser, v)
                ser.close()
            except: inUse.append(p) # normally denied access

        # if not return yet, get some useful error info   
        if len(inUse)==0:
            err = 'No RTBox serial port found. Make sure FTDI driver is installed'
        elif len(inUse)==1:
            err = 'Possible RTBox port %s is already in use' % inUse[0]
        else: 
            err = 'Possible RTBox ports %s are already in use' % inUse
        raise EnvironmentError('\n' + self._p.sysinfo + '\n' + err)
