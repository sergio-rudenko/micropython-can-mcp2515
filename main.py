"""
Following SPI drivers are supported, please adjust by your hardware
from .src import SPIESP8286 as SPI
from .src import SPIESP32 as SPI


The CAN driver can be initialized with default baudrate 10MHz
CAN(SPI(cs=YOUR_SPI_CS_PIN)) or
CAN(SPI(cs=YOUR_SPI_CS_PIN, baudrate=YOUR_DESIRED_BAUDRATE))

And here is an example to set filter for extended frame with ID 0x12345678
can.setFilter(RXF.RXF0, True, 0x12345678 | CAN_EFF_FLAG)
can.setFilterMask(MASK.MASK0, True, 0x1FFFFFFF | CAN_EFF_FLAG)
can.setFilterMask(MASK.MASK1, True, 0x1FFFFFFF | CAN_EFF_FLAG)
"""
import time

from lib.mcp2515 import (
    CAN,
    CanWithQueue,
    CAN_CLOCK,
    CAN_EFF_FLAG,
    CAN_ERR_FLAG,
    CAN_RTR_FLAG,
    CAN_SPEED,
    ERROR,
)
from lib.mcp2515 import SPIESP32 as SPI
from lib.mcp2515 import CANFrame

import micropython
import uasyncio as asyncio

SPI_ESP32_CS_PIN = 23


def print_error(text: str):
    print(f'ERROR: {text}')


# Initialization
can = CanWithQueue(SPI(cs=SPI_ESP32_CS_PIN), int_gpio=18, queue_size=5)

# Configuration
if can.reset() != ERROR.ERROR_OK:
    print_error("Can not reset for MCP2515")

if can.setBitrate(CAN_SPEED.CAN_125KBPS, CAN_CLOCK.MCP_8MHZ) != ERROR.ERROR_OK:
    print_error("Can not set bitrate for MCP2515")

if can.setNormalMode() != ERROR.ERROR_OK:
    print_error("Can not set mode for MCP2515")

# Prepare frames
data = b"\x12\x34\x56\x78\x9A\xBC\xDE\xF0"
sff_frame = CANFrame(can_id=0x7FF, data=data)
sff_none_data_frame = CANFrame(can_id=0x7FF)
err_frame = CANFrame(can_id=0x7FF | CAN_ERR_FLAG, data=data)
eff_frame = CANFrame(can_id=0x12345678 | CAN_EFF_FLAG, data=data)
eff_none_data_frame = CANFrame(can_id=0x12345678 | CAN_EFF_FLAG)
rtr_frame = CANFrame(can_id=0x7FF | CAN_RTR_FLAG)
rtr_with_eid_frame = CANFrame(can_id=0x12345678 | CAN_RTR_FLAG | CAN_EFF_FLAG)
rtr_with_data_frame = CANFrame(can_id=0x7FF | CAN_RTR_FLAG, data=data)
frames = [
    sff_frame,
    sff_none_data_frame,
    err_frame,
    eff_frame,
    eff_none_data_frame,
    rtr_frame,
    rtr_with_eid_frame,
    rtr_with_data_frame,
]

# Read all the time and send message in each second
end_time, n = time.ticks_add(time.ticks_ms(), 1000), -1

        # if not can.empty():
        #     print(f'MSG -> {can.get_nowait()}')
        # time.sleep_ms(10)
        # error, iframe = can.readMessage()
        # if error == ERROR.ERROR_OK:
            # print("RX  {}".format(iframe))
            # if prev_iframe != iframe:
            #     prev_iframe = iframe
            # else:
            #     print("lost frame!")

        # else:
        #     print_error(f'num: {error}')

        # time.sleep_ms(500)

        # if time.ticks_diff(time.ticks_ms(), end_time) >= 0:
        #     end_time = time.ticks_add(time.ticks_ms(), 1000)
        #     n += 1
        #     n %= len(frames)
        #
        #     error = can.sendMessage(frames[n])
        #     if error == ERROR.ERROR_OK:
        #         print("TX  {}".format(frames[n]))
        #     else:
        #         print("TX failed with error code {}".format(error))


# asyncio.create_task(aprint())
prev = None

import micropython
micropython.alloc_emergency_exception_buf(100)


async def start():
    while not can.task_is_stopped():
        try:
            print(f'Read from queue -> {await can.queue_get()}')
            await asyncio.sleep_ms(1000)
        except KeyboardInterrupt:
            exit(0)
    # counter = 0
    #
    # print('Analiz start')
    # while not can.empty():
    #     global prev
    #     counter += 1
    #     now = can.get_nowait()
    #     print(now)
    #     if prev is None:
    #         prev = now
    #     elif prev == now:
    #         print('Error on step ', counter)
    #
    #     prev = now

asyncio.run(start())
