#!/usr/bin/env python3

# Pull active low reset pin of the MCU on the HUT high.
# Currently only relevant for Duckiebots with HUTv3.1.

import os
import logging


def init_logging():
    logfile = os.path.join(os.environ.get("HOME"), 
        "duckietown/logs/enable_mcu.log")
    logging.basicConfig(
        filename=logfile,
        filemode='a',
        format='[%(levelname)s] %(asctime)s : %(message)s',
        datefmt='%d-%m-%y %H:%M:%S',
        level=logging.DEBUG)


def main():
    init_logging()

    try:
        import Jetson.GPIO as GPIO
    except ImportError as err:
        logging.fatal(err)
        return 1

    envs = {key : os.environ.get(key) for key in
        ["ROBOT_HARDWARE", "ROBOT_CONFIGURATION", "HUT_MCU_ENABLE_PIN"]}

    if None in envs.values():
        logging.fatal(f"Mising environment variable(s) in {envs}")
        return 1

    if (envs["ROBOT_HARDWARE"] == "jetson_nano"
        and envs["ROBOT_CONFIGURATION"] == "DB21M"):
        GPIO.setmode(GPIO.BCM)
        pin = int(envs["HUT_MCU_ENABLE_PIN"])
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.cleanup()
        logging.info(f"Set {pin} high, enabled MCU")
    else:
        logging.warn(f"Unsupported hardware: {envs}")


if __name__ == "__main__":
    main()
