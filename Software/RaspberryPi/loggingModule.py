import logging
import os

def create_shared_logger():
    os.makedirs('logs', exist_ok=True)

    logger = logging.getLogger("shared_logger")
    logger.setLevel(logging.DEBUG)

    if not logger.hasHandlers():
        handler = logging.FileHandler('logs/shared_logs.csv', mode='w')
        formatter = logging.Formatter('%(asctime)s - %(module)s - %(lineno)d - %(levelname)s - %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p')
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    return logger

if __name__ == "__main__":
    logger = create_shared_logger()
    logger.error("DOES THIS WORK?")