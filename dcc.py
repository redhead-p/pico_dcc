"""DCC module
    :author: Paul Redhead

        Copyright Paul Redhead, 2023

This module contains the functions and classes for DCC generation.

An RP2040 Peripheral Input Output (PIO) block is used to generate the DCC signal. A 'booster'
is required to enable to track to be powered.  One of the many DC motor H-bridge chips (e.g. DRV8874)
should be suitable. 

A command comprises a preamble, one or more instruction/data bytes and an error detection (checksum) byte.
Each byte is preceeded by a single '0' bit.
The checksum is followed by a single '1' bit which may be the initial bit of
the next preamble. The preamble is at least 14 '1' bits.


The DCC signal is a series of '1' & '0' bits.  Each bit is encoded
into a complete DCC output cycle.  The half cycle length for a '1' is 58us, a '0' is 100us.
The PIO FIFO buffer is 32 bits wide - 1 word.  The RX FIFO is 
not used so is joined to the TX FIFO.  This gives a total FIFO of 8 words. 7 words max are required for
a normal DCC command packets.

If the FIFO is empty, the PIO doesn't stall but continues outputing '0' bits until the FIFO is 
loaded with a new command.

This DCC implementation has a limited set of features.  E.g. it doesn't include bit stretching
for DC vehicles on address 0, 14/28 speed steps, any service mode functions or accessory controller
commands. We allow for a maximum command sequnece of 6 bytes including check sum. 

NMRA Standards S 9.2 and S 9.2.1 partly apply. S 9.2.1.1 is not supported.

"""

from machine import Pin,   Timer
import time
from micropython import const
import array

import rp2

# module constants - not for importing elsewhere
_PIO_FREQ = const(500_000)          # 500 kHz - 2 micro sec. tick period

"""NMRA Standard S 9.2 (2004) Section C applies.
The time between packets addressed to the same decoder must exceed 5ms. This is the time the packet
end bit and the following packet start bit.

The period between complete packet transmission (start bit to start bit) must be less or equal to
30ms."""
_PACKET_PERIOD = const(15)          # ms between each DCC packet

_MAX_LONG_ADDR = const(0x27FF)      # DCC long address upper limit (inclusive)
_MIN_LONG_ADDR = const(128)         # DCC long address lower limit (inclusive)
_SPD_128 = const(0x3f)              # 1st instruction byte for 128 speed packet
_F_GROUP_1 = const(0x80)            # instruction byte for Function group 1
_F_NUM_ENCODE = [0x10, 0x01, 0x02, 0x04, 0x08] # translate Group 1 function no. to mask
_DCC_PREAMBLE = const(0x8000ffff)   # DCC preamble - 16 '1' bits (minumum 14 bits)
_DCC_PADDING = const(0x80000000)    # DCC padding - 16 '0' bits for delay between packets if needed
_DCC_LAST_BYTE = const(0x40000000)  # last byte marker - packet end '1' bit appended by PIO



@rp2.asm_pio(sideset_init=rp2.PIO.OUT_HIGH,  
             out_shiftdir=rp2.PIO.SHIFT_LEFT, 
               fifo_join = rp2.PIO.JOIN_TX)
def _dcc_gen():
    """PIO Generate DCC
    
    This is the PIO DCC output generator. A 'sideset' pin is used for the DCC output. The 
    OSR is set to shift left so the Most Significant bits are shifted out first.
    The side pin is set to the DCC signal by .side(x).

    As this is output only we can use the Input Shift Register (ISR) as an additional store.
    """

    wrap_target()
    # do nominally high side of the cycle first
    #  2 ticks high for short (1) and long (0)
    mov(x, isr)             .side(1)    [0]     # isr will be set later - assumed 0 first time
    jmp(not_x, "nxt_byte")  .side(1)    [0]     # if 0 - packet end bit not set on preceeding byte
    # packet end bit '1' to do - do the high side - 27 ticks here + 2
    set(y, 0)               .side(1)    [8]     # loop counter 1 bit only
    mov(isr, y)             .side(1)    [8]     # and clear isr too to mark end bit done
    jmp("short_low")        .side(1)    [8]     # normal short bit code for low half cycle


    label("nxt_byte")
    #  set x to default value for OSR to be used if FIFO empty 
    #  7 ticks from here to 'nxt_bit' for both paths - 9 cumulative
    set(x, 0)               .side(1)    [0]
    # pull word from FIFO
    # 32 bits are pulled
    # if FIFO is empty register x (0) is copied to OSR
    # so we send '0's
    pull(noblock)           .side(1)    [0]
    out(x,1)                .side(1)    [0]    # get first bit
    jmp(not_x, "data_byte") .side(1)    [0]    # if 0 it's normal data

    # pre-amble or delay padding
    out(null, 15)           .side(1)    [0]     # discard unused bits
    set(y, 15)              .side(1)    [0]     # load loop counter for 16 bits
    jmp("nxt_bit")          .side(1)    [0]
    

    label ("data_byte")
    out(isr,1)              .side(1)    [0]     # put the next bit in the isr for next loop
    out(null, 21)           .side(1)    [0]     # discard unused bits
    set(y, 8)               .side(1)    [0]     # load loop counter for 9 bits

    label ("nxt_bit")
    # 18 ticks high high for short (1) and long (0) - cummulative 27 
    out(x,1)                .side(1)    [8]    # get next output bit
    jmp(not_x, "long_high") .side(1)    [8]    # if 0 it's long
    
    # 2 ticks high for short (1) only - cummulative 29 (full short 1st half cycle)
    jmp("short_low")        .side(1)    [1]     # it's short - low half cycle next
    
    label("long_high")
     # 23 ticks high for long (0) only - cummulative 50 (full long 1st half cycle)
    nop()                   .side(1)    [10]    # long half cycle is 21 ticks longer
    nop()                   .side(1)    [11]

     # 21 ticks low for long (0) only - cummulative low 21 (2nd half cycle) 
    nop()                   .side(0)    [9]     # for both high & low op
    nop()                   .side(0)    [10]

    label("short_low")
    # 29 ticks low for short (1) and long (0) - cummulative short 29, long 50 (full cycles)
    nop()                   .side(0)    [14]    # common part - both 1 & 0
    jmp(y_dec, "not_done")  .side(0)    [13]    # test bit count
    wrap()                                      # start next single bit/ byte group



    label("not_done")
    # 9 ticks high for short (1) and long (0) - same as 1st  instructions(skipped)
    jmp("nxt_bit")          .side(1)    [8]     # done this bit - do next bit            


class DCCGen:
    """DCC Output Generator

    This singleton class manages the generation of DCC command packets.

    The PIO has to be fed with words.  If the top bit (bit 31) is set the word contains
    pre-amble ('1's) or filler ('0's).  16 bits are output. If bit 31 is clear the word contains 
    a  preceeding bit and output byte. 9 bits are otput. Bit 30 set indicates the last byte
    (usually checksum). Following this yhe packet end '1' bit is appended by the PIO.
    

    Attributes:
        IDLE_PACKET:    A DCC idle packet.  To be transmitted if no other packets in list.
        FWD:    Forward direction
        REV:    Reverse direction
        ON:     Power On
        OFF:    Power Off
        
    """

    # class constants - may be imported 

    IDLE_PACKET = bytes(b'\xff\x00')  # Address 255, instruction/data 0

    FWD = const(1)
    REV = const(-1)

    ON = const(1)
    OFF = const(0)

     

    # class variables

    _this_dcc = None # the singleton DCC Generator instance
    
    @classmethod
    def get_instance(cls):
        """ Get the DCC generator instance.

        
        args:
            cls:

        Returns:
            The DCC generator instance
        """

        return cls._this_dcc

    
    
    def __init__(self, sm_num, DCC_pn, enable_pn):
        """DCC generator constructor
        
        This initialises the DCC generator singleton.  An attempt to create a 2nd
        instance will cause a runtime error.  The PIO statemachine is allocated and initialised.
        
        Pins and timers are initialised. The dictionary for the packet list is created and the 
        FIFO buffer allocated.
        
        Args:
            self:
            sm_num: PIO statemachine number to be used.
            DCC_pn: Pin number allocated for DCC output.
            enable_pn: Pin number allocated to enable the booster."""

        if not DCCGen._this_dcc is None:
            raise RuntimeError ('Attempt to create 2nd DCC gen')
        DCCGen._this_dcc = self
        
        self._sm = rp2.StateMachine(sm_num, _dcc_gen, freq= _PIO_FREQ, sideset_base = Pin(DCC_pn, Pin.OUT))                    
        self._enable_pin = Pin(enable_pn, Pin.OUT, value = 0)
        self._packet_timer = Timer()
        self._packet_list = {}              # create empty dictionary
        self._FIFO_buff = array.array('I', range(8)) # FIFO buffer is 8 words but 'I' not 'L'
        
       

    def power(self, p = None):
        """DCC Power On/Off
        
        On power on we
            - assert the booster enable pin
            - start the state machine
            - start the packet timer 

        The packet timer callback will generate the next packet in the packet list.

        On power off we
            - de-assert the booster enable pin
            - stop the statemachine
            - deinitialise the packet timer

        The packet list is retained while power is off and may be amended using other 
        commands.

        args:
            p: 1 for power on, 0 for power off, None for get power status

        returns:
            power status as held by the enable pin

        """
        if (p is None):
            return self._enable_pin()
        if p == DCCGen.ON:
            self._enable_pin(1)
            self._sm.active(True)
            self._packet_timer.init(mode = Timer.PERIODIC, period = _PACKET_PERIOD, callback = self._nxt_packet)
            # set an iterator up so we can do one packet per timer callback
            self._packet_iter = iter(self._packet_list)
        else:
            self._enable_pin(0)
            self._sm.active(False)
            self._packet_timer.deinit()
        return self._enable_pin()
    
    def set_speed(self, address, dir, speed = 0):
        """Set Speed (including direction)
        
        If there is a speed packet for the adressed decoder in the list already
        the packet is updated otherwise a new packet is inserted.  The input is validated.
        The packet generated will be for a 128 step speed setting and
        decoders must be configured for 28/128 speed steps.

        See NMRA S-9.2.1  Section 2.3.2.1
        
        args:
            self:
            address: the address of the decoder - may be short or long
            dir:    the direction - forward or reverse
            speed: the speed to be set - range 0 to 127 - default 0
              
        returns:
            True if validation is passed and the packet is added to the list or modified. False
            if validation fails.
        """
        # a bit of defensive programming
        if address < 1 or address > _MAX_LONG_ADDR:
            return False
        if speed < 0 or speed > 127:
            return False
        if not dir in (DCCGen.FWD, DCCGen.REV):
            return False
        
        # speed direction packet list entry's key is 'S', address
        # speed / direction packet - 1 or 2 address bytes, 2 instruction bytes
    
        
        inst_2 = (0x80 if dir == DCCGen.FWD else 0) | (speed & 0x7f)
        try:
            # modify instruction byte 2 (the last byte) with new dir/speed
            self._packet_list[('S', address)][-1] = inst_2
        except KeyError:
            # new entry
            if address < _MIN_LONG_ADDR:
                # short address - 1 byte, instruction 1, instruction 2
                self._packet_list[('S', address)] = bytearray((address, _SPD_128, inst_2))
            else:
                # long address - 2 bytes, instruction 1, instruction 2
                msb_address = (0xc000 | address) >> 8
                self._packet_list[('S', address)] = bytearray((msb_address, address & 0xff, _SPD_128, inst_2))
        return True
            
    def set_fg1(self, address, f_num, state):
        """Set Function Group 1
        
        This sets or clears a function in group 1.  The forward light is usually function 
        number 0.

        If there is a function group 1 command in the packet list for the addressed decoder it is
        updated. Otherwise the command is added to the list.

        See NMRA S-9.2.1  Section 2.3.4
        
        args:
            self:
            address: the address of the decoder - may be short or long
            f_num:  function number to set or clear
            state:  1 for set, 0 for clear

        returns:
            True if validation is passed and the packet is added to the list or modified. False
            if validation fails.   
        """
        # a bit of defensive programming
        if address < 1 or address > _MAX_LONG_ADDR:
            return False
        if f_num < 0 or f_num > 4:
            return False
        if not state in (0, 1):
            return False
        # translate the function number to mask
        mask = _F_NUM_ENCODE[f_num]


        # function group 1 packet - single instruction byte
        try:
            # get current intruction byte (it's at the end)
            inst_1 = self._packet_list[('F', address)][-1]
            # set or clear function bit
            self._packet_list[('F', address)][-1] = (inst_1 | mask) if state == 1 else  (inst_1 &  ~mask)
        except KeyError:
            # new entry - no current bits to retain
            inst_1 = _F_GROUP_1 | (mask if state ==1 else 0)
            if address < _MIN_LONG_ADDR:
                # short address - 1 byte, instruction 1
                self._packet_list[('F', address)] = bytearray((address, inst_1))
            else:
                # long address - 2 bytes, instruction 1
                msb_address = (0xc000 | address) >> 8
                self._packet_list[('F', address)] = bytearray((msb_address, address & 0xff, inst_1))

        return True
        


    
    def _nxt_packet(self, _):
        """ Generate next packet callback.
        
        This is called when the next packet timer has expired and the next packet in the
        list is to be serialised out on the DCC interface. If the list is empty the DCC
        Idle packet is serialised.
        
        This function retreives a bytearray of byte values from
        the list and passes it to the send routine for loading into the PIO FIFO.
        """
        if len(self._packet_list) == 0:
            # list empty send the DCC idle packet
            self._send_packet(DCCGen.IDLE_PACKET)
            return
        try:
            # get the next packet in the list
            packet_key = next(self._packet_iter)
        except StopIteration:
            # at end of list - renew iterator
            self._packet_iter = iter(self._packet_list)
            packet_key = next(self._packet_iter)
        self._send_packet(self._packet_list[packet_key])
        return


        
    def _send_packet(self, byte_list):
        """Send a packet.
        
        Set up the FIFO buffer with preample, data from the byte list, 
        and checksum.  Then transfer to PIO FIFO.
        """
        byte_count = len(byte_list)
        if byte_count > 5: # max is 6 but that includes chksum
            raise RuntimeError ('DCC command too long')
        self._FIFO_buff[0] = _DCC_PREAMBLE
        count = 1
        err_detect = 0  # initialise error detection checksum
        for b in byte_list:
            self._FIFO_buff[count] = b
            count += 1
            err_detect ^= b
        self._FIFO_buff[1 + byte_count] = err_detect | _DCC_LAST_BYTE
        self._sm.put(memoryview(self._FIFO_buff[0:byte_count + 2]))
        
        
        
if __name__ == '__main__':
    dcc = DCCGen(0, 20, 19) # state machine 0, DCC pin 20, enable pin 19
