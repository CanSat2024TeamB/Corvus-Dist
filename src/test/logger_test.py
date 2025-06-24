import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

from logger.logger import Logger
import asyncio

async def logger_test():
    logger = Logger()
    while True:
        await asyncio.sleep(1)
        logger.write("hello", "world", "everyone")

asyncio.run(logger_test())