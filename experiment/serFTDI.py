"""
Python module to control FTDI USB-serial port via D2XX driver.
171021 Xiangrui.Li at gmail.com simplify code from https://github.com/ctrl-shift-esc/ftd2xx
"""
__version__ = '2019.11.06'
_PORTS = [] # open port indice

import os
import ctypes as c
DWORD = c.c_ulong
ULONG = c.c_ulong
UCHAR = c.c_ubyte
FT_HANDLE = c.c_void_p

libPath = os.path.dirname(os.path.realpath(__file__)) + '/lib/'
if os.sys.platform.startswith('win'):
    from ctypes.wintypes import DWORD, ULONG
    try: dll = c.WinDLL('ftd2xx.dll')
    except: dll = c.WinDLL(libPath+'ftd2xx.dll')
elif os.sys.platform.startswith('linux'):
    try: dll = c.CDLL('libftd2xx.so')
    except:
        import glob
        dll = c.CDLL(glob.glob(libPath+'libftd2xx.so*')[0])
else:
    try: dll = CDLL('libftd2xx.dylib')
    except:
        import glob
        dll = c.CDLL(glob.glob(libPath+'libftd2xx*.dylib')[0])

msgs = ['OK', 'INVALID_HANDLE', 'DEVICE_NOT_FOUND', 'DEVICE_NOT_OPENED',
        'IO_ERROR', 'INSUFFICIENT_RESOURCES', 'INVALID_PARAMETER',
        'INVALID_BAUD_RATE', 'DEVICE_NOT_OPENED_FOR_ERASE',
        'DEVICE_NOT_OPENED_FOR_WRITE', 'FAILED_TO_WRITE_DEVICE0',
        'EEPROM_READ_FAILED', 'EEPROM_WRITE_FAILED', 'EEPROM_ERASE_FAILED',
        'EEPROM_NOT_PRESENT', 'EEPROM_NOT_PROGRAMMED', 'INVALID_ARGS',
        'NOT_SUPPORTED', 'OTHER_ERROR']

class DeviceError(Exception):
    """Exception class for error messages"""
    def __init__(self, msg):
        self.message = msg

    def __str__(self):
        return self.message

def call_ft(function, *args):
    """Call a D2XX function and check the status. Raise exception on error"""
    err = function(*args)
    if err: raise DeviceError(msgs[err])

def NumberOfPorts():
    """Return number of ports connected"""
    n = DWORD()
    call_ft(dll.FT_ListDevices, c.byref(n), None, DWORD(0x80000000)) # NUMBER_ONLY
    return n.value

def Accessible():
    """Check if D2XX driver can access the ports"""
    if len(_PORTS)>0 or NumberOfPorts()==0: return True
    h = FT_HANDLE()
    try:
        call_ft(dll.FT_Open, 0, c.byref(h))
        call_ft(dll.FT_Close, h)
        return True
    except: return False

# Class for communicating with an FTDI device
class FTD2XX(object):
    """Open a USB-serial port by index and return an FTD2XX instance for it.
    Optional hostClock is used to timestamp events.
    All other port parameters have common default."""
    def __init__(self, index=0, hostClock=None, BaudRate=115200, 
                 ReadTimeout=0.3, WriteTimeout=0.3, LatencyTimer=0.002, 
                 DataBits=8, StopBits=1, Parity='None', FlowControl='None'):
        self.port = index
        self.open()

        # store clock
        if hostClock is None: from time import time as hostClock
        self.hostSecs = hostClock

        # store parameters
        class PortInfo(type): pass
        self._info = PortInfo
        self._info.BaudRate = BaudRate
        self._info.LatencyTimer = LatencyTimer
        self._info.ReadTimeout = ReadTimeout
        self._info.WriteTimeout = WriteTimeout
        self._info.DataBits = DataBits
        self._info.StopBits = StopBits
        self._info.Parity = Parity
        self._info.FlowControl = FlowControl
       
        # set port parameters
        self.setBaudRate(BaudRate)
        self.setLatencyTimer(LatencyTimer)
        self.setTimeouts(ReadTimeout, WriteTimeout)
        self.setDataCharacteristics(DataBits, StopBits, Parity)
        self.setFlowControl(FlowControl, XOn=0x11, XOff=0x13)

    def open(self):
        """Open a port without setting parameters"""
        if self.port in _PORTS: 
            raise DeviceError('Port ' + str(self.port) + ' is already open')
        h = FT_HANDLE()
        call_ft(dll.FT_Open, self.port, c.byref(h))
        self.handle = h
        self.is_open = True
        _PORTS.append(self.port)

    def bytesAvailable(self):
        """Get number of bytes in receive queue."""
        n = DWORD()
        call_ft(dll.FT_GetQueueStatus, self.handle, c.byref(n))
        return n.value

    def write(self, data, blocking=True):
        """Send the data to the device. data is string or integer list.
        blocking means the function will return after the data is written."""
        n = DWORD()
        if type(data) is not str: data = bytes(bytearray(data))
        nW = DWORD(len(data))
        dwMask = DWORD()
        tPre = self.hostSecs()
        call_ft(dll.FT_Write, self.handle, data, nW, c.byref(n))
        if n.value<nW.value: DeviceError('Only '+str(n.value)+' of '+str(nW.value)+' bytes written')
        while blocking and n.value>0:
            call_ft(dll.FT_GetStatus, self.handle, c.byref(nW), c.byref(n), c.byref(dwMask))
        tPost = self.hostSecs()
        return (tPre, tPost)

    def read(self, nBytes=None):
        """Read up to nBytes bytes of data from the device. Can return fewer if
        timedout. If nBytes not provided, read all bytes currently in buffer."""
        if nBytes is None: nBytes = self.bytesAvailable()
        n = DWORD()
        buf = c.c_buffer(nBytes)
        call_ft(dll.FT_Read, self.handle, buf, nBytes, c.byref(n))
        return buf.raw[:n.value]

    def close(self):
        """Close the port"""
        if self.port in _PORTS: _PORTS.remove(self.port)
        self.is_open = False
        call_ft(dll.FT_Close, self.handle)

    def setLatencyTimer(self, LatencyTimer=0.002):
        """Set latency timer in seconds (1 to 255 ms)"""
        LatencyTimer = int(LatencyTimer*1000+0.5)
        call_ft(dll.FT_SetLatencyTimer, self.handle, UCHAR(LatencyTimer))
        self._info.LatencyTimer = LatencyTimer/1000.0

    def setBaudRate(self, BaudRate=115200):
        """Set the baud rate"""
        call_ft(dll.FT_SetBaudRate, self.handle, DWORD(BaudRate))
        self._info.BaudRate = BaudRate

    def purge(self, mask=3):
        """Purge receive (mask=1), transmit (mask=2) buffer, or both (mask=3)."""
        call_ft(dll.FT_Purge, self.handle, DWORD(mask))

    def setTimeouts(self, ReadTimeout=None, WriteTimeout=None):
        """Set receive and transmit timeout in seconds.
        Default (None) means to keep current value"""
        if ReadTimeout  is None: ReadTimeout  = self._info.ReadTimeout
        if WriteTimeout is None: WriteTimeout = self._info.WriteTimeout
        ReadTimeout  = int(ReadTimeout *1000+0.5)
        WriteTimeout = int(WriteTimeout*1000+0.5)
        call_ft(dll.FT_SetTimeouts, self.handle, DWORD(ReadTimeout), DWORD(WriteTimeout))
        self._info.ReadTimeout  = ReadTimeout /1000.0
        self._info.WriteTimeout = WriteTimeout/1000.0

    def setDataCharacteristics(self, DataBits=None, StopBits=None, Parity=None):
        """Set data characteristics: DataBits (7 or 8), StopBits (1 or 2), and
        Parity (one of string None, Odd, Even, Mark, or Space)"""
        if DataBits is None: DataBits = self._info.DataBits
        if StopBits is None: StopBits = self._info.StopBits
        if Parity   is None: Parity   = self._info.Parity
        if   StopBits==1: sbits = 0
        elif StopBits==2: sbits = 2
        else: raise ValueError('StopBits must be 1 or 2')     
        try: prty = ['None', 'Odd', 'Even', 'Mark', 'Space'].index(Parity)
        except: raise ValueError('Valid Parity is None, Odd, Even, Mark, or Space')
        call_ft(dll.FT_SetDataCharacteristics, self.handle, UCHAR(DataBits), UCHAR(sbits), UCHAR(prty))
        self._info.DataBits = DataBits
        self._info.StopBits = StopBits
        self._info.Parity   = Parity

    def setFlowControl(self, FlowControl=None, XOn=None, XOff=None):
        """Set flow control: one of None, RTS_CTS (Hardware), DTR_DSR, or XON_XOFF (Software),
        and xon and xoff is flow control is XON_XOFF"""
        if FlowControl is None: FlowControl = self._info.FlowControl
        if XOn is None:  XOn  = self._info.XOn
        if XOff is None: XOff = self._info.XOff
        if   (FlowControl == 'None'):                                    u16 = 0x0000
        elif (FlowControl == 'RTS_CTS') or (FlowControl == 'Hardware'):  u16 = 0x0100
        elif (FlowControl == 'DTR_DSR'):                                 u16 = 0x0200
        elif (FlowControl == 'XON_XOFF') or (FlowControl == 'Software'): u16 = 0x0400
        else: raise ValueError('Valid flow control is None, RTS_CTS, DTR_DSR, or XON_XOFF.')
        call_ft(dll.FT_SetFlowControl, self.handle, c.c_ushort(u16), UCHAR(XOn), UCHAR(XOff))
        self._info.FlowControl = FlowControl
        self._info.XOn = XOn
        self._info.XOff = XOff

    def setDtr(self, secs=None):
        """ Set the Data Terminal Ready (DTR) line.
        If secs is set, the line will be cleared after secs.
        setDtr(0.005) # output 5 ms TTL at DTR line"""
        call_ft(dll.FT_SetDtr, self.handle)
        if secs is None: return
        tEnd = self.hostSecs() + secs
        while self.hostSecs()<tEnd: pass
        self.clrDtr()

    def clrDtr(self):
        """ Clear the Data Terminal Ready (DTR) control signal """
        call_ft(dll.FT_ClrDtr, self.handle)

    def setRts(self, secs=None):
        """ Set the Request To Send (RTS) control signal.
        If secs is set, the line will be cleared after secs """
        call_ft(dll.FT_SetRts, self.handle)
        if secs is None: return
        tEnd = self.hostSecs() + secs
        while self.hostSecs()<tEnd: pass
        self.clrRts()

    def clrRts(self):
        """ Clear the Request To Send (RTS) control signal """
        call_ft(dll.FT_ClrRts, self.handle)

    def resetDevice(self):
        """ Send a reset command to the device """
        call_ft(dll.FT_ResetDevice, self.handle)

    def getPortInfo(self, screenOutput=False):
        """Get information and parameters of the port. Print to screen if screenOutput is Ture"""
        ftDev = ULONG()
        devID = DWORD()
        SN = c.c_buffer(16)
        desc = c.c_buffer(64)
        call_ft(dll.FT_GetDeviceInfo, self.handle, c.byref(ftDev), c.byref(devID), SN, desc, None)
        devs = ['BM', 'AM', '100AX', 'UNKNOWN', '2232C', '232R', '2232H', '4232H', '232H',
                'X_SERIES', '4222H_0', '4222H_1_2', '4222H_3', '4222_PROG', '900', '930', 'UMFTPD3A']
        self._info.DeviceType = devs[ftDev.value]
        self._info.VIDPID = '0x%08X' % devID.value # '0x04036001'
        self._info.SerialNumber = SN.value.decode() # empty?
        self._info.Description = desc.value.decode() # 'USB <-> Serial'
        if os.sys.platform.startswith('win'):
            portNum = c.c_long()
            call_ft(dll.FT_GetComPortNumber, self.handle, c.byref(portNum))
            self._info.ComPort = 'COM%i' % portNum.value
        if not screenOutput:
            class copy1(self._info): pass # changing the copy won't affect _info
            return copy1
        for f in dir(self._info):
            if f.startswith('_') or f=='mro': continue
            print('%20s: %s' % (f, getattr(self._info, f)))

__all__ = ['Accessible', 'FTD2XX', 'NumberOfPorts']
