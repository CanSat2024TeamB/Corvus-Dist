import sys
from pathlib import Path
from wire.wirehandler import WireHandler
import asyncio

async def main():
    nichrome_pin_no_1 = 25
    nichrome_pin_no_2 = 23
    nichrome_pin_duration = 10
    wire = WireHandler()

    await asyncio.sleep(10)
    print('start')
    wire.nichrome_cut(nichrome_pin_no_2,nichrome_pin_duration)
    await asyncio.sleep(5)
    wire.nichrome_cut(nichrome_pin_no_2,nichrome_pin_duration)
    #print('start')
    #wire.nichrome_cut(nichrome_pin_no_2,nichrome_pin_duration)
    #print('done')
 
    wire.cleanup()
    print('cleanup done')


if __name__ == "__main__":
    asyncio.run(main())