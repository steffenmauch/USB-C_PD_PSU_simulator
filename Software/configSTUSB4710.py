#!/usr/bin/env python3

#MIT License

#Copyright (c) 2022 Steffen Mauch

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.
"""
Python script to control STUSB4710 via CP2112
designed for https://github.com/steffenmauch/USB-C_PD_PSU_simulator
"""

import ctypes as ct
import sys
import time
import math
from pprint import pprint
from collections import namedtuple
import argparse

__version__ = "0.1"
__date__ = "20 Dec 2022"

#==============================================================================
# Constants
#==============================================================================

class HID_SMBUS:
    VID = 0x10C4
    PID = 0xEA90

    VID_STR = 0x01
    PID_STR = 0x02
    PATH_STR = 0x03
    SERIAL_STR = 0x04
    MANUFACTURER_STR = 0x05
    PRODUCT_STR = 0x06

class HID_SMBUS_S0:
    IDLE = 0x00
    BUSY = 0x01
    COMPLETE = 0x02
    ERROR = 0x03

class HID_SMBUS_S1:
    BUSY_ADDRESS_ACKED = 0x00
    BUSY_ADDRESS_NACKED = 0x01
    BUSY_READING = 0x02
    BUSY_WRITING = 0x03

    ERROR_TIMEOUT_NACK = 0x00
    ERROR_TIMEOUT_BUS_NOT_FREE = 0x01
    ERROR_ARB_LOST = 0x02
    ERROR_READ_INCOMPLETE = 0x03
    ERROR_WRITE_INCOMPLETE = 0x04
    ERROR_SUCCESS_AFTER_RETRY = 0x05


#==============================================================================
# Error Handling
#==============================================================================
HID_SMBUS_STATUS_DESC = {
    0x00 : "HID_SMBUS_SUCCESS",
    0x01 : "HID_SMBUS_DEVICE_NOT_FOUND",
    0x02 : "HID_SMBUS_INVALID_HANDLE",
    0x03 : "HID_SMBUS_INVALID_DEVICE_OBJECT",
    0x04 : "HID_SMBUS_INVALID_PARAMETER",
    0x05 : "HID_SMBUS_INVALID_REQUEST_LENGTH",
    0x10 : "HID_SMBUS_READ_ERROR",
    0x11 : "HID_SMBUS_WRITE_ERROR",
    0x12 : "HID_SMBUS_READ_TIMED_OUT",
    0x13 : "HID_SMBUS_WRITE_TIMED_OUT",
    0x14 : "HID_SMBUS_DEVICE_IO_FAILED",
    0x15 : "HID_SMBUS_DEVICE_ACCESS_ERROR",
    0x16 : "HID_SMBUS_DEVICE_NOT_SUPPORTED",
    0xFF : "HID_SMBUS_UNKNOWN_ERROR",
}

class HidSmbusError(Exception):
    def __init__(self, status):
        self.status = status
        try:
            self.name = HID_SMBUS_STATUS_DESC[status]
        except:
            self.name = "HID_UART_STATUS_UNKNOWN: " + hex(status)
    def __str__(self):
        return self.name

def hidsmb_errcheck(result, func, args):
    if result != 0:
        raise HidSmbusError(result)

#==============================================================================
# CP2112 HIDtoSMBus DLL
#==============================================================================

if sys.platform == 'win32':
    _DLL = ct.windll.LoadLibrary("./SLABHIDtoSMBus.dll")
elif sys.platform.startswith('linux'):
    _DLL_prev = ct.CDLL("./libslabhiddevice.so.1.0", mode=ct.RTLD_GLOBAL)
    _DLL = ct.cdll.LoadLibrary("./libslabhidtosmbus.so.1.0")
elif sys.platform == 'darwin':
    _DLL = ct.cdll.LoadLibrary("libSLABHIDtoSMBus.dylib")


#==============================================================================
# Library Functions
#==============================================================================

# HidSmbus_GetNumDevices(DWORD* numDevices, WORD vid, WORD pid);
def GetNumDevices(vid=HID_SMBUS.VID, pid=HID_SMBUS.PID):
    """Returns the number of devices connected to the host with matching VID/PID."""
    ndev = ct.c_ulong()
    _DLL.HidSmbus_GetNumDevices(ct.byref(ndev), vid, pid)
    return ndev.value

# HidSmbus_GetAttributes(DWORD deviceNum, WORD vid, WORD pid, WORD* deviceVid, WORD* devicePid, WORD* deviceReleaseNumber);
def GetAttributes(index=0, vid=HID_SMBUS.VID, pid=HID_SMBUS.PID):
    """Returns VID, PID and release number for the indexed device with matching VID/PID."""
    dev_vid = ct.c_ushort()
    dev_pid = ct.c_ushort()
    dev_rel = ct.c_ushort()
    _DLL.HidSmbus_GetAttributes(index, vid, pid, ct.byref(dev_vid), ct.byref(dev_pid), ct.byref(dev_rel))
    return (dev_vid.value, dev_pid.value, dev_rel.value)

# HidSmbus_GetString(DWORD deviceNum, WORD vid, WORD pid, char* deviceString, DWORD options);
def GetString(index=0, vid=HID_SMBUS.VID, pid=HID_SMBUS.PID, opt=HID_SMBUS.SERIAL_STR):
    """Returns the selected string for the indexed device with matching VID/PID."""
    buf = ct.create_string_buffer(512)
    _DLL.HidSmbus_GetString(index, vid, pid, buf, opt)
    return buf.value.decode()

def IsOpened(index=0, vid=HID_SMBUS.VID, pid=HID_SMBUS.PID):
    """Checks if the indexed device with matching VID/PID is already open."""
    status = 0
    try:
        GetAttributes(index, vid, pid)
    except HidSmbusError as e:
        status = e.status
    # 0x15 : "HID_SMBUS_DEVICE_ACCESS_ERROR"
    return bool(status == 0x15)


#==============================================================================
# HidSmb Class
#==============================================================================

class HidSmbusDevice:

    def __init__(self):
        self.handle = ct.c_void_p(0)
        self._S0 = ct.c_byte(0)
        self._S1 = ct.c_byte(0)
        GetNumDevices()

    @property
    def S0(self):
        return self._S0.value

    @property
    def S1(self):
        return self._S1.value

    # HidSmbus_Open(HID_SMBUS_DEVICE* device, DWORD deviceNum, WORD vid, WORD pid);
    def Open(self, index=0, vid=HID_SMBUS.VID, pid=HID_SMBUS.PID):
        _DLL.HidSmbus_Open(ct.byref(self.handle), index, vid, pid)

    # HidSmbus_Close(HID_SMBUS_DEVICE device);
    def Close(self):
        if self.handle:
            _DLL.HidSmbus_Close(self.handle)
            self.handle.value = 0

    # HidSmbus_IsOpened(HID_SMBUS_DEVICE device, BOOL* opened);
    def IsOpened(self):
        opened = ct.c_long(0)
        if self.handle:
            _DLL.HidSmbus_IsOpened(self.handle, ct.byref(opened))
        return bool(opened.value)

    # HidSmbus_GetOpenedAttributes(HID_SMBUS_DEVICE device, WORD* deviceVid, WORD* devicePid, WORD* deviceReleaseNumber);
    def GetAttributes(self):
        vid = ct.c_ushort(0)
        pid = ct.c_ushort(0)
        rel = ct.c_ushort(0)
        _DLL.HidSmbus_GetOpenedAttributes(self.handle, ct.byref(vid), ct.byref(pid), ct.byref(rel))
        return (vid.value, pid.value, rel.value)

    # HidSmbus_GetOpenedString(HID_SMBUS_DEVICE device, char* deviceString, DWORD options);
    def GetString(self, opt=HID_SMBUS.SERIAL_STR):
        buf = ct.create_string_buffer(512)
        _DLL.HidSmbus_GetOpenedString(self.handle, buf, opt)
        return buf.value.decode()

    # HidSmbus_CancelTransfer(HID_SMBUS_DEVICE device);
    def CancelTransfer(self):
        _DLL.HidSmbus_CancelTransfer(self.handle)

    # HidSmbus_Reset(HID_SMBUS_DEVICE device);
    def Reset(self):
        _DLL.HidSmbus_Reset(self.handle)
        _DLL.HidSmbus_Close(self.handle)
        self.handle.value = 0

    # HidSmbus_AddressReadRequest(HID_SMBUS_DEVICE device, BYTE slaveAddress, WORD numBytesToRead, BYTE targetAddressSize, BYTE targetAddress[16]);
    def AddressReadRequest(self, address=2, count=64, offset_size=2, offset=b'\x00\x00'):
        buf = ct.create_string_buffer(bytes(offset), size=16)
        _DLL.HidSmbus_AddressReadRequest(self.handle, address, count, offset_size, buf)

    # HidSmbus_ForceReadResponse(HID_SMBUS_DEVICE device, WORD numBytesToRead);
    def ForceReadResponse(self, count=64):
        _DLL.HidSmbus_ForceReadResponse(self.handle, count)

    # HidSmbus_GetReadResponse(HID_SMBUS_DEVICE device, HID_SMBUS_S0* status, BYTE* buffer, BYTE bufferSize, BYTE* numBytesRead);
    def GetReadResponse(self):
        count = 61
        buf = ct.create_string_buffer(count)
        n = ct.c_ulong(0)
        try:
            _DLL.HidSmbus_GetReadResponse(self.handle, ct.byref(self._S0), buf, count, ct.byref(n))
        except HidSmbusError as e:
            # Ignore timeout, return the data that was read
            if e.status != 0x12:
                raise
        a = bytes(buf)
        temp = (n.value, a[:n.value])
        #print( temp )
        return temp

    # HidSmbus_WriteRequest(HID_SMBUS_DEVICE device, BYTE slaveAddress, BYTE* buffer, BYTE numBytesToWrite);
    def WriteRequest(self, address, buffer, count=None):
        if count is None:
            count = len(buffer)
        _DLL.HidSmbus_WriteRequest(self.handle, address, bytes(buffer), count)

    # HidSmbus_TransferStatusRequest(HID_SMBUS_DEVICE device);
    def TransferStatusRequest(self):
        _DLL.HidSmbus_TransferStatusRequest(self.handle)

    # HidSmbus_GetTransferStatusResponse(HID_SMBUS_DEVICE device, HID_SMBUS_S0* status, HID_SMBUS_S1* detailedStatus, WORD* numRetries, WORD* bytesRead);
    def GetTransferStatusResponse(self):
        tries = ct.c_ushort(0)
        count = ct.c_ushort(0)
        _DLL.HidSmbus_GetTransferStatusResponse(self.handle, ct.byref(self._S0), ct.byref(self._S1), ct.byref(tries), ct.byref(count))
        return (self._S0.value, self._S1.value, tries.value, count.value)

    # HidSmbus_SetTimeouts(HID_SMBUS_DEVICE device, DWORD responseTimeout);
    def SetTimeouts(self, timeout=1000):
        _DLL.HidSmbus_SetTimeouts(self.handle, timeout)

    # HidSmbus_GetTimeouts(HID_SMBUS_DEVICE device, DWORD* responseTimeout);
    def GetTimeouts(self):
        timeout = ct.c_ulong(0)
        _DLL.HidSmbus_GetTimeouts(self.handle, ct.byref(timeout))
        return timeout.value

    # HidSmbus_SetSmbusConfig(HID_SMBUS_DEVICE device, DWORD bitRate, BYTE address, BOOL autoReadRespond, WORD writeTimeout, WORD readTimeout, BOOL sclLowTimeout, WORD transferRetries);
    def SetSmbusConfig(self, bitRate=100000, address=2, autoReadRespond=False, writeTimeout=0, readTimeout=0, sclLowTimeout=False, transferRetries=0):
        _DLL.HidSmbus_SetSmbusConfig(self.handle, 
            bitRate, address, autoReadRespond, writeTimeout, readTimeout, sclLowTimeout, transferRetries)

    # HidSmbus_GetSmbusConfig(HID_SMBUS_DEVICE device, DWORD* bitRate, BYTE* address, BOOL* autoReadRespond, WORD* writeTimeout, WORD* readTimeout, BOOL* sclLowtimeout, WORD* transferRetries);
    def GetSmbusConfig(self):
        rate = ct.c_ulong()
        addr = ct.c_byte()
        auto = ct.c_bool()
        wto = ct.c_ushort()
        rto = ct.c_ushort()
        scl = ct.c_bool()
        retry = ct.c_ushort()
        _DLL.HidSmbus_GetSmbusConfig(self.handle, 
            ct.byref(rate), ct.byref(addr), ct.byref(auto), ct.byref(wto), 
            ct.byref(rto), ct.byref(scl), ct.byref(retry))
        return (rate.value, addr.value, auto.value, wto.value, rto.value, scl.value, retry.value)


#==============================================================================
# STUSB4710 Class
#==============================================================================

class STUSB4710:

    def stusb47x0_nvm_lock(self, device, lock=False):
      if lock is False:
         device.WriteRequest(address=80, buffer=[0x95, 0x47], count=2)
      else:
         device.WriteRequest(address=80, buffer=[0x95, 0x00], count=2)
   
    # https://github.com/timkruse/stusb4500/blob/master/sw/stusb45.py
    def nvm_dump(self, device):
        self.stusb47x0_nvm_lock(device, False) # unlock NVM
        
        while True:
            device.AddressReadRequest(address=80, count=1, offset_size=1, offset=b'\x96')
            time.sleep(0.05)
            device.ForceReadResponse(count=1)
            reg8 = device.GetReadResponse()
            if int.from_bytes(reg8[1], "big") & 0x10 == 0x00:
                break

        sector_data = []
        for num_sector in range(0, 5):
            # send command opcode READ(0x00) to FTP_CTRL_1(0x97)
            device.WriteRequest(address=80, buffer=[0x97, 0 & 0x07], count=2)
            # execute command

            device.WriteRequest(address=80, buffer=[0x96, ((num_sector & 0x07) | 0x80 | 0x40 | 0x10)], count=2)

            time.sleep(0.2)
            device.TransferStatusRequest()

            # read 8 bytes that are copied from nvm to 0x53-0x5a
            device.AddressReadRequest(address=80, count=8, offset_size=1, offset=b'\x53')
            time.sleep(0.05)
            device.ForceReadResponse(count=8)
            sector = device.GetReadResponse()
            device.TransferStatusRequest()

            sector_data.append(sector[1])

        self.stusb47x0_nvm_lock(device, True) # lock NVM

        # nicely print out the values
        sec = 0
        print( '\nnvm_dump' )
        for sector in sector_data:
            line = "%d: [" % sec
            sec += 1
            line += ''.join(' 0x{:02x}'.format(x) for x in bytearray(sector)) + ", "
            line = line[:-2] # remove trailing comma
            line += "]"
            print(line)

    def vbus_ctrl(self, device):
        # Defining types
        Vbus = namedtuple("VBUS", "discharge_0v, discharge_trans, vbus_discharge, vsrc_discharge, sink_vbus_en")
        Vbus.__str__ = lambda v: "VBUS(discharge_time_transition=" + str(v.discharge_trans*24) + "ms" +\
                        ", discharge_to_0V=" + str(v.discharge_0v*84) + "ms" +\
                        ", vbus_discharge=" + ("Enabled" if v.vbus_discharge else "Disabled") +\
                        ", vsrc_discharge=" + ("Enabled" if v.vsrc_discharge else "Disabled") +\
                        ", sink_vbus_en=" + ("Enabled" if v.sink_vbus_en else "Disabled") +\
                        ")"
        Vbus.__repr__ = Vbus.__str__

        # Read out relevant registers
        device.AddressReadRequest(address=80, count=1, offset_size=1, offset=b'\x25')
        time.sleep(0.05)
        device.ForceReadResponse(count=1)
        VBUS_DISCHARGE_TIME_CTRL = int.from_bytes(device.GetReadResponse()[1], "big")
        
        device.AddressReadRequest(address=80, count=1, offset_size=1, offset=b'\x26')
        time.sleep(0.05)
        device.ForceReadResponse(count=1)
        VBUS_DISCHARGE_CTRL = int.from_bytes(device.GetReadResponse()[1], "big")
        
        device.AddressReadRequest(address=80, count=1, offset_size=1, offset=b'\x27')
        time.sleep(0.05)
        device.ForceReadResponse(count=1)
        VBUS_CTRL = int.from_bytes(device.GetReadResponse()[1], "big")

        # Map data to the type
        return Vbus(discharge_0v=VBUS_DISCHARGE_TIME_CTRL>>4 & 0x0f, discharge_trans=VBUS_DISCHARGE_TIME_CTRL & 0x0f, vbus_discharge=True if VBUS_DISCHARGE_CTRL >> 7 & 0x01 else False, vsrc_discharge=True if VBUS_DISCHARGE_CTRL >> 6 & 0x01 else False, sink_vbus_en=True if VBUS_CTRL >> 1 & 0x01 else False)

    # \brief Reads the current Status of the port (Sink-Source connection)
    def port_status(self, device):
        device.AddressReadRequest(address=80, count=1, offset_size=1, offset=b'\x0d')
        time.sleep(0.05)
        device.ForceReadResponse(count=1)
        PORT_STATUS_0 = int.from_bytes(device.GetReadResponse()[1], "big")

        device.AddressReadRequest(address=80, count=1, offset_size=1, offset=b'\x0d')
        time.sleep(0.05)
        device.ForceReadResponse(count=1)
        PORT_STATUS_1 = int.from_bytes(device.GetReadResponse()[1], "big")

        attachedDeviceAsString = ["None", "Sink", "Source", "Debug Accessory", "Audio Accessory", "Power Accessory"]

        PortStatus = namedtuple("PortStatus", "stateChanged, attachedDevice, lowPowerStandby, powerMode, dataMode, attached")
        PortStatus.__str__ = lambda ps: "PortStatus(stateChanged=" + ("True" if ps.stateChanged == 1 else "False") + \
         ", attachedDevice=" + (attachedDeviceAsString[ps.attachedDevice] if (ps.attachedDevice >= 0 and ps.attachedDevice <= 5) else "undefined(" + str(ps.attachedDevice) + ")") + \
         ", lowPowerStandby=" +  ("standby mode" if ps.lowPowerStandby == 1 else "normal mode") + \
         ", powerMode=" +  ("Source" if ps.powerMode == 1 else "Sink") + \
         ", dataMode=" +  ("DFP" if ps.dataMode == 1 else "UFP") + \
         ", attached=" +  ("True" if ps.attached == 1 else "False") + ")"
        PortStatus.__repr__ = PortStatus.__str__
       
        return PortStatus(stateChanged=PORT_STATUS_0 & 0x01, attachedDevice=PORT_STATUS_1 >> 5 & 0x07, lowPowerStandby=PORT_STATUS_1 >> 4 & 0x01, powerMode=PORT_STATUS_1 >> 3 & 0x01, dataMode=PORT_STATUS_1 >> 2 & 0x01, attached=PORT_STATUS_1 & 0x01)

    # \brief Reads typec revision and usbpd revision of the chip
    def version(self, device):
        device.AddressReadRequest(address=80, count=1, offset_size=1, offset=b'\x06')
        time.sleep(0.05)
        device.ForceReadResponse(count=1)
        BCD_TYPEC_REV_LOW = int.from_bytes(device.GetReadResponse()[1], "big")

        device.AddressReadRequest(address=80, count=1, offset_size=1, offset=b'\x07')
        time.sleep(0.05)
        device.ForceReadResponse(count=1)
        BCD_TYPEC_REV_HIGH = int.from_bytes(device.GetReadResponse()[1], "big")
        typec_rev = BCD_TYPEC_REV_HIGH << 8 | BCD_TYPEC_REV_LOW

        device.AddressReadRequest(address=80, count=1, offset_size=1, offset=b'\x08')
        time.sleep(0.05)
        device.ForceReadResponse(count=1)
        BCD_USBPD_REV_LOW = int.from_bytes(device.GetReadResponse()[1], "big")

        device.AddressReadRequest(address=80, count=1, offset_size=1, offset=b'\x09')
        time.sleep(0.05)
        device.ForceReadResponse(count=1)
        BCD_USBPD_REV_HIGH = int.from_bytes(device.GetReadResponse()[1], "big")
        usbpd_rev = BCD_USBPD_REV_HIGH << 8 | BCD_USBPD_REV_LOW
       
        Version = namedtuple("Version", "typec_rev, usbpd_rev")
        Version.__str__ = lambda v: "Version(typec_rev=" + hex(v.typec_rev) + ", usbpd_rev=" + hex(v.usbpd_rev)+ ")"
        Version.__repr__ = Version.__str__

        return Version(typec_rev=typec_rev, usbpd_rev=usbpd_rev)   

    # \brief Reads all currently configured PDOs from the chip
    def read_pdo(self, device, base_reg):
        device.AddressReadRequest(address=80, count=20, offset_size=1, offset=base_reg.to_bytes(1, "big"))
        time.sleep(0.05)
        device.ForceReadResponse(count=20)
        temp = device.GetReadResponse();
        #print( ''.join(' 0x{:02x}'.format(x) for x in bytearray(temp[1])) ) 
      
        bvalues = temp[1]
      
        supplyStr = ["Fixed", "Variable", "Battery"]

        PdoSinkFix = namedtuple("PdoSinkFix", "current, voltage, fastRoleReqCur, dualRoleData, usbCommunicationsCapable, unconstrainedPower, higherCapability, dualRolePower, supply, raw")
        PdoSinkFix.__str__ = lambda ps: "PdoSink("+ \
            "voltage=" + str(ps.voltage / 20.0) + "V" + \
            ", current=" + str(ps.current / 100.0) + "A" + \
            ", fastRoleReqCur=" + str(ps.fastRoleReqCur) + \
            ", dualRoleData=" + str(ps.dualRoleData) + \
            ", usbCommunicationsCapable=" + str(ps.usbCommunicationsCapable) + \
            ", unconstrainedPower=" + str(ps.unconstrainedPower) + \
            ", higherCapability=" + str(ps.higherCapability) + \
            ", dualRolePower=" + str(ps.dualRolePower) + \
            ", supply=" + (supplyStr[ps.supply] if ps.supply >= 0 and ps.supply < 3 else "Undefined") + \
            ", raw=0x" + format(ps.raw, '08x') + \
        ")"
        PdoSinkFix.__repr__ = PdoSinkFix.__str__

        PdoSinkVar = namedtuple("PdoSinkVar", "min_voltage, max_voltage, current, supply, raw")
        PdoSinkVar.__str__ = lambda ps: "PdoSink("+\
            "voltage=[" + str(ps.min_voltage / 20.0) + "V-" + str(ps.max_voltage / 20.0) + "V]" + \
            ", current=" + str(ps.current / 100.0) + "A" + \
            ", supply=" + (supplyStr[ps.supply] if ps.supply >= 0 and ps.supply < 3 else "Undefined") + \
            ", raw=0x" + format(ps.raw, '08x') + \
            ")"
        PdoSinkVar.__repr__ = PdoSinkVar.__str__

        PdoSinkBat = namedtuple("PdoSinkBat", "min_voltage, max_voltage, power, supply, raw")
        PdoSinkBat.__str__ = lambda ps: "PdoSink("+\
            "voltage=[" + str(ps.min_voltage / 20.0) + "V-" + str(ps.max_voltage / 20.0) + "V]" + \
            ", power=" + str(ps.power) + "W" + \
            ", supply=" + (supplyStr[ps.supply] if ps.supply >= 0 and ps.supply < 3 else "Undefined") + \
            ", raw=0x" + format(ps.raw, '08x') + \
            ")"
        PdoSinkBat.__repr__ = PdoSinkBat.__str__
        
        pdo = {}
        for i in range(0, 5):
            reg = bvalues[i * 4 + 3] << 24 | bvalues[i * 4 + 2] << 16 | bvalues[i * 4 + 1] << 8 | bvalues[i * 4]
            supply = reg >> 30 & 0x3
            if supply == 0: #  fixed
                pdo[i+1] = PdoSinkFix(supply=supply, dualRolePower=reg>>29 & 0x1, higherCapability=reg>>28 & 0x1, unconstrainedPower=reg>>27 & 0x1, usbCommunicationsCapable=reg>>26 & 0x1, dualRoleData=reg>>25 & 0x1, fastRoleReqCur=reg>>23 & 0x3, voltage=reg>>10 & 0x3ff, current=reg & 0x3ff, raw=reg)
            elif supply == 1: # variable
                pdo[i+1] = PdoSinkVar(supply=supply, max_voltage=reg>>20 & 0x3ff, min_voltage=reg>>10 & 0x3ff, current=reg&0x3ff, raw=reg)
            elif supply == 2: # battery
                pdo[i+1] = PdoSinkBat(supply=supply, max_voltage=reg>>20 & 0x3ff, min_voltage=reg>>10 & 0x3ff, power=reg&0x3ff, raw=reg)
        return pdo

    # \brief Reads and then prints the Power Data Object
    def print_pdo(self, device):
      for k, v in self.read_pdo(device, 0x71).items():
         print( "PDO SRC#" + str(k) + ": ", v)

    # \brief Read out the Requested Data Object (RDO)
    # https://github.com/usb-c/STUSB4500/blob/master/Firmware/Project/Inc/USB_PD_defines_STUSB-GEN1S.h
    # en.STSWSTUSB004v1_1_0.zip
    def read_rdo(self, device):
        device.AddressReadRequest(address=80, count=4, offset_size=1, offset=b'\x91')
        time.sleep(0.05)
        device.ForceReadResponse(count=4)
        temp = device.GetReadResponse();

        bvalues = temp[1]
        
        device.AddressReadRequest(address=80, count=1, offset_size=1, offset=b'\x21')
        time.sleep(0.05)
        device.ForceReadResponse(count=1)
        temp = device.GetReadResponse(); # *100mV
        
        requested_voltage = int.from_bytes(temp[1], "big")/10.0 # I want it in Volt not milli volt

        reg = bvalues[3] << 24 | bvalues[2] << 16 | bvalues[1] << 8 | bvalues[0]
        Rdo = namedtuple("RDO", "voltage, current, maxCurrent, unchunkedMess_sup, usbSuspend, usbComCap, capaMismatch, giveBack, objectPos, raw")
        Rdo.__str__ = lambda ps: "RDO("+ \
            "voltage=" + str(requested_voltage) + "V" + \
            ", current=" + str(ps.current / 100.0) + "A" + \
            ", maxCurrent=" + str(ps.maxCurrent / 100.0) + "A" + \
            ", unchunkedMess_sup=" + str(ps.unchunkedMess_sup) + \
            ", usbSuspend=" + str(ps.usbSuspend) + \
            ", usbComCap=" + str(ps.usbComCap) + \
            ", capaMismatch=" + str(ps.capaMismatch) + \
            ", giveBack=" + str(ps.giveBack) + \
            ", objectPos=" + str(ps.objectPos) + \
            ", raw=0x" + format(ps.raw, '08x') + \
            ")"
        Rdo.__repr__ = Rdo.__str__
      
        return Rdo(voltage=requested_voltage, objectPos=reg >> 28 & 0x7, giveBack=reg>>27 & 0x1, capaMismatch=reg>>26 & 0x1, usbComCap=reg>>25 & 0x1, usbSuspend=reg>>24 & 0x1, unchunkedMess_sup=reg>>23 & 0x1, current=reg>>10 & 0x3ff, maxCurrent=reg & 0x3ff, raw=reg)
   
    def read_monitoring_ctrl_2(self, device):
        device.AddressReadRequest(address=80, count=1, offset_size=1, offset=b'\x22')
        time.sleep(0.05)
        device.ForceReadResponse(count=1)
        temp = device.GetReadResponse();
        return temp[1]
    
    def printVBUSTolerance( self, value ):
        tempA = int.from_bytes( value, "big" )
        ovp = ( (tempA >> 4) & 0xF ) + 5
        uvp = ( (tempA >> 4) & 0xF ) + 5
        print( "overvoltage protection: ", ovp ,"% ; undervoltage protection", uvp ,"%" )
        
    def write_monitoring_ctrl_2(self, device, vshift_low, vshift_high):
        device.WriteRequest(address=80, buffer=[0x22, vshift_high<<4 | vshift_low], count=2)
        time.sleep(0.05)
      
    def modifyPDO(self, device, pos, data):
        if (pos > 0) and (pos < 6):
            buffer = [0x71+(pos-1)*4]
            buffer.extend(data)
            device.WriteRequest(address=80, buffer=buffer, count=len(buffer))
            time.sleep(0.05)
    
    def reset(self, device):
        device.WriteRequest(address=80, buffer=[0x23, 0x01], count=2)
        time.sleep(0.05)
        device.WriteRequest(address=80, buffer=[0x23, 0x00], count=2)
        time.sleep(0.05)
      
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Configure STUSB4710 for USB-C PD power supply simulation; (c) 12/2022 Steffen Mauch; MIT license')
    parser.add_argument('-r', '--resetSTUSB', action="store_true",
                        help='reset STUSB4710 via command')
    parser.add_argument('--setToleranceVBUS', dest='setToleranceVBUS', type=int, choices=range(0,16), 
                        metavar="[0-15]", default=-1, help='set additional VBUS tolerance in percent of STUSB4710')
    parser.add_argument('--setPDO', required=False,  nargs='+', 
                        help='modify PDO of STUSB4710; first argument selects #PDO;'+
                        'second argument voltage in [mv]; third argument current in [mA]'+
                        ' e.g. 1 5000 2000 will set PDO1 to 5V and 2A')
    parser.add_argument('--printRDO', dest='printRDO', action="store_true",
                        help='shows actual settings requested data object (RDO) of STUSB4710')
    parser.add_argument('--printPDO', dest='printPDO', action="store_true",
                        help='shows actual settings power data object (PDO) of STUSB4710')
    parser.add_argument('--printNVM', dest='printNVM', action="store_true",
                        help='shows actual content of non volatile memory (NVM) of STUSB4710')
    parser.add_argument('--printToleranceVBUS', action="store_true",
                        help='get full VBUS tolerance in percent of STUSB4710 [incl. 5 percent default]')
    parser.add_argument('--printVBUSControl', action="store_true",
                        help='get VBUS control register of STUSB4710')
    parser.add_argument('--printPortStatus', action="store_true",
                        help='get port status register of STUSB4710')
    parser.add_argument('--printSTUSBVersion', action="store_true",
                        help='get version register of STUSB4710')
    parser.add_argument('-v', '--version', help='show version of python script', action="store_true")

    
    args = vars(parser.parse_args())
    
    if len(sys.argv)==1:
        parser.print_help()
        sys.exit(1)
        
    try:
        smb = HidSmbusDevice()
        smb.Open(0)
        smb.SetSmbusConfig(bitRate=400000, address=2, autoReadRespond=False, writeTimeout=0, readTimeout=0, sclLowTimeout=False, transferRetries=2)
        stusb = STUSB4710()
        serialNbCP2112 = smb.GetString()
        if not serialNbCP2112:
            print( "\n   no CP2112 found; please connect CP2112 to computer!\n" )
            sys.exit()
        
        print()
        #print(args)
        for arg in args:
            if arg == 'resetSTUSB' and args[arg] == True:
                stusb.reset(smb)
            elif arg == 'setToleranceVBUS' and args[arg] != -1:
                stusb.write_monitoring_ctrl_2(smb, args['setToleranceVBUS'], args['setToleranceVBUS'])
            elif arg == 'printToleranceVBUS' and args[arg] == True:
                stusb.printVBUSTolerance( stusb.read_monitoring_ctrl_2(smb) )
                print("---------------------")
            elif arg == 'setPDO' and args[arg] != None:
            
                if len(args[arg]) != 3:
                    print( "  setSRC must have three additional parameters" )
                    break
                    
                pos = int( args[arg][0] )
                volt = math.floor( int( args[arg][1] ) / 50 ) & 0x3ff
                current = math.floor( int( args[arg][2] ) / 10 ) & 0x3ff
                
                val = current + volt * 2**10
                arrayVal = val.to_bytes(3, 'little');
                if( pos < 1 or pos > 5 ):
                    print( "  position of SRC must be between 1 and 5" )
                    break
                
                stusb.modifyPDO(smb, pos, arrayVal )
                time.sleep(0.05)
            elif arg == 'printRDO' and args[arg] == True:
                print( stusb.read_rdo(smb) )
                print("---------------------")
            elif arg == 'printPDO' and args[arg] == True:
                stusb.print_pdo(smb)
                print("---------------------")
            elif arg == 'printNVM' and args[arg] == True:
                stusb.nvm_dump(smb)
                print("---------------------")
            elif arg == 'printVBUSControl' and args[arg] == True:
                print( stusb.vbus_ctrl(smb) )
                print("---------------------")
            elif arg == 'printPortStatus' and args[arg] == True:
                print( stusb.port_status(smb) )
                print("---------------------")
            elif arg == 'printSTUSBVersion' and args[arg] == True:
                print( stusb.version(smb) )
                print("---------------------")
            elif arg == 'version' and args[arg] == True:
                print("script version "+__version__+" from "+__date__)
                print("---------------------")
            
    except HidSmbusError as e:
        print("Device Error:", e, "-", hex(e.status))
    finally:
        smb.Close()
