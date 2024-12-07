# SPDX-FileCopyrightText: Copyright (c) 2023 Jose D. Montoya
#
# SPDX-License-Identifier: MIT
"""
`bmi270`
================================================================================

MicroPython Driver for the Bosch BMI270 Accelerometer/Gyro Sensor


* Author(s): Jose D. Montoya


"""

import time
from micropython import const
from bmi270.i2c_helpers import CBits, RegisterStruct
import os

try:
    from typing import Tuple
except ImportError:
    pass

# pylint: disable=import-outside-toplevel

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/jposada202020/MicroPython_BMI270.git"

_SPI_READ_FLAG = const(0x80)
_REG_WHOAMI = const(0x00)
_ERROR_CODE = const(0x02)
_COMMAND = const(0x7E)
_ACC_RANGE = const(0x41)
_PWR_CTRL = const(0x7D)
_GYRO_RANGE = const(0x43)

_STANDARD_GRAVITY = const(9.80665)

# Acceleration Data
ACC_X_LSB = const(0x0C)
ACC_Y_LSB = const(0x0E)
ACC_Z_LSB = const(0x10)

# Gyro Data
GYRO_X_LSB = const(0x12)
GYRO_Y_LSB = const(0x14)
GYRO_Z_LSB = const(0x16)

# Acceleration Range
ACCEL_RANGE_2G = const(0b00)
ACCEL_RANGE_4G = const(0b01)
ACCEL_RANGE_8G = const(0b10)
ACCEL_RANGE_16G = const(0b11)
acceleration_range_values = (
    ACCEL_RANGE_2G,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G,
)

ACCELERATOR_DISABLED = const(0b0)
ACCELERATOR_ENABLED = const(0b1)
acceleration_operation_mode_values = (ACCELERATOR_DISABLED, ACCELERATOR_ENABLED)

GYRO_RANGE_2000 = const(0b000)
GYRO_RANGE_1000 = const(0b001)
GYRO_RANGE_500 = const(0b010)
GYRO_RANGE_250 = const(0b011)
GYRO_RANGE_125 = const(0b100)
gyro_range_values = (
    GYRO_RANGE_2000,
    GYRO_RANGE_1000,
    GYRO_RANGE_500,
    GYRO_RANGE_250,
    GYRO_RANGE_125,
)

# RESET Command
RESET_COMMAND = const(0xB6)

_PWR_CONF = const(0x7C)
_INIT_CTRL = const(0x59)
_INIT_ADDR_0 = const(0x5B)
_INIT_ADDR_1 = const(0x5C)
_INIT_DATA = const(0x5E)

# interrupt control
_INT1_IO_CTRL = const(0x53)
_INT2_IO_CTRL = const(0x54)
_INT1_MAP_FEAT = const(0x56)
_INT2_MAP_FEAT = const(0x57)
_INT_STATUS_0  = const(0x1C)
_INT_STATUS_1  = const(0x1D)
_FEAT_PAGE  = const(0x2F)
_FEATURES   = const(0x30)
_FEATURE_0  = const(0x30)
_FEATURE_1  = const(0x32)
_FEATURE_2  = const(0x34)
_FEATURE_3  = const(0x36)
_FEATURE_4  = const(0x38)
_FEATURE_5  = const(0x3A)
_FEATURE_6  = const(0x3C)
_FEATURE_7  = const(0x3E)

#interrupt and features enable masks
SIG_MOTION_MSK      = const(0x1)
STEP_COUNTER_MSK    = const(0x2)
ACTIVITY_MSK        = const(0x4)
WRIST_WEAR_MSK      = const(0x8)
WRIST_GESTURE_MSK   = const(0x10)
NO_MOTION_MSK       = const(0x20)
ANY_MOTION_MSK      = const(0x40)


SENSOR_DATA_PAGE    = const(0x0)
SETTINGS_PAGE       = const(0x1)
MOTION_PAGE         = const(0x2)
STEP_COUNTER_PAGE1  = const(0x3)
STEP_COUNTER_PAGE2  = const(0x4)
STEP_COUNTER_PAGE3  = const(0x5)
STEP_COUNTER_PAGE4  = const(0x6)
WRIST_GESTURE_PAGE  = const(0X6)
WRIST_WEAR_PAGE     = const(0X7)


class BMI270:
    """Driver for the BMI270 Sensor connected over I2C.

    :param ~machine.I2C i2c: The I2C bus the BMI270 is connected to.
    :param int address: The I2C device address. Defaults to :const:`0x68`

    :raises RuntimeError: if the sensor is not found

    **Quickstart: Importing and using the device**

    Here is an example of using the :class:`BMI270` class.
    First you will need to import the libraries to use the sensor

    .. code-block:: python

        from machine import Pin, I2C
        from micropython_bmi270 import bmi270

    Once this is done you can define your `machine.I2C` object and define your sensor object

    .. code-block:: python

        i2c = I2C(1, sda=Pin(2), scl=Pin(3))
        bmi270 = bmi270.BMI270(i2c)

    Now you have access to the attributes

    .. code-block:: python

        acc_x, acc_y, acc_z = bmi270.acceleration

    """

    _device_id = RegisterStruct(_REG_WHOAMI, "B")
    _error_code = RegisterStruct(_ERROR_CODE, "B")
    _soft_reset = RegisterStruct(_COMMAND, "B")
    _read = RegisterStruct(_COMMAND, "B")

    power_control = RegisterStruct(_PWR_CTRL, "B")
    power_config = RegisterStruct(0x7C, "B")

    _acc_data_x = RegisterStruct(ACC_X_LSB, "<h")
    _acc_data_y = RegisterStruct(ACC_Y_LSB, "<h")
    _acc_data_z = RegisterStruct(ACC_Z_LSB, "<h")

    # Gyro Data
    _gyro_data_x = RegisterStruct(GYRO_X_LSB, "<h")
    _gyro_data_y = RegisterStruct(GYRO_Y_LSB, "<h")
    _gyro_data_z = RegisterStruct(GYRO_Z_LSB, "<h")

    # GYRO_RANGE Register (0x43)
    _gyro_range = CBits(3, _GYRO_RANGE, 0)
    gyro_scale = (16.4, 32.8, 65.6, 131.2, 262.4)

    # ACC_RANGE Register (0x41)
    # The register allows the selection of the accelerometer g-range
    _acceleration_range = CBits(2, _ACC_RANGE, 0)
    acceleration_scale = (16384, 8192, 4096, 2048)

    _acceleration_operation_mode = CBits(1, _PWR_CTRL, 2)

    _power_configuration = RegisterStruct(_PWR_CONF, "B")

    internal_status = RegisterStruct(0x21, "B")

    _init_control = RegisterStruct(_INIT_CTRL, "B")
    
    interrupt_status0 = RegisterStruct(_INT_STATUS_0, "B")
    interrupt_status1 = RegisterStruct(_INT_STATUS_1, "B")
    _interrupt_1 = RegisterStruct(_INT1_IO_CTRL, "B")
    _interrupt_2 = RegisterStruct(_INT2_IO_CTRL, "B")
    _interrupt_feat_1 = RegisterStruct(_INT1_MAP_FEAT, "B")
    _interrupt_feat_2 = RegisterStruct(_INT2_MAP_FEAT, "B")
    
    feature_page = RegisterStruct(_FEAT_PAGE, "B")
    feature_condata = RegisterStruct(_FEATURES, "<HHHHHHHH")
    feature_reg_0 = RegisterStruct(_FEATURE_0, "<H")
    feature_reg_1 = RegisterStruct(_FEATURE_1, "<H")
    feature_reg_2 = RegisterStruct(_FEATURE_2, "<H")
    feature_reg_3 = RegisterStruct(_FEATURE_3, "<H")
    feature_reg_4 = RegisterStruct(_FEATURE_4, "<H")
    feature_reg_5 = RegisterStruct(_FEATURE_5, "<H")
    feature_reg_6 = RegisterStruct(_FEATURE_6, "<H")
    feature_reg_7 = RegisterStruct(_FEATURE_7, "<H")

    step_counter_l = RegisterStruct(_FEATURE_0, "<H")
    step_counter_h = RegisterStruct(_FEATURE_1, "<H")
    activity = RegisterStruct(_FEATURE_2, "<H")
    gesture = RegisterStruct(_FEATURE_3, "<H")

    _init_address_0 = RegisterStruct(_INIT_ADDR_0, "B")
    _init_address_1 = RegisterStruct(_INIT_ADDR_1, "B")
    _init_data = RegisterStruct(_INIT_DATA, ">HHHHHHHHHHHHHHHH")

    def __init__(self, i2c=None, address: int = 0x68, spi=None, cs= None) -> None:
        self._i2c = i2c
        self._spi = spi
        self._cs = cs
        if self._cs:
            self._cs.on()
        self._address = address

        if self._device_id != 0x24:
             raise RuntimeError("Failed to find BMI270")

        self.soft_reset()

        self.load_config_file()
        self.power_control = 0x06
        time.sleep(0.1)
        self.power_config = 0x00
        time.sleep(0.1)
        self.acceleration_range = ACCEL_RANGE_2G
        self.gyro_range = GYRO_RANGE_250

    def error_code(self) -> None:
        """
        The register is meant for debug purposes, not for regular verification
        if an operation completed successfully.

        Fatal Error: Error during bootup. This flag will not be cleared after
        reading the register.The only way to clear the flag is a POR.


        """

        errors = self._error_code
        i2c_err = (errors & 0x80) >> 7
        fifo_err = (errors & 0x40) >> 6
        internal_error = (errors & 0x1E) >> 1
        fatal_error = errors & 0x01
        if i2c_err:
            print("Error in I2C-Master detected. This flag will be reset when read.")
        if fifo_err:
            print(
                "Error when a frame is read in streaming mode (so skipping is not possible) \n"
                + "and fifo is overfilled (with virtual and/or regular frames). This flag will\n"
                + "be reset when read."
            )
        if internal_error != 0:
            print("Internal Sensor Error")
        if fatal_error:
            print("Fatal Error. This flag will be reset when read")

    def soft_reset(self) -> None:
        """
        Performs a Soft Reset

        :return: None

        """
        self._soft_reset = RESET_COMMAND
        time.sleep(0.015)
        
    def enable_interrupt(self, num=1, active_high=True, open_drain = False, mask=STEP_COUNTER_MSK) -> None:
        """
        Enable interrup pins and features

        :return: None

        """      
        io_config = 0x8
        if open_drain:
            io_config |= 0x4
        if active_high:
            io_config |= 0x2
        if num == 1:
            self._interrupt_1 = io_config
            time.sleep_ms(1)
            self._interrupt_feat_1 |= mask
            time.sleep_ms(1)
        else:
            self._interrupt_2 = io_config
            time.sleep_ms(1)
            self._interrupt_feat_2 |= mask
            time.sleep_ms(1)
        
    def disable_interrupt(self) -> None:
        """
        disable interrup features

        :return: None

        """
        self._interrupt_feat_1 = 0
        self._interrupt_feat_2 = 0
        
        
    def enable_features(self, f_mask, enable_interrupt = True) -> None:
        """
        Enable Features
        f_mask: masked features to enable
        enable_interrupt: automatic enable feature interrupt
        :return: None

        """
        if f_mask & STEP_COUNTER_MSK:
            self.feature_page = STEP_COUNTER_PAGE4
            self.feature_reg_1 |= 0x1002
            
        if f_mask & ACTIVITY_MSK:
            self.feature_page = STEP_COUNTER_PAGE4
            self.feature_reg_1 |= 0x2000
            
        if f_mask & WRIST_WEAR_MSK:
            self.feature_page = WRIST_WEAR_PAGE
            self.feature_reg_0 |= 0x10
            
        if f_mask & WRIST_GESTURE_MSK:
            self.feature_page = STEP_COUNTER_PAGE4
            self.feature_reg_6 |= 0x20
            
        if f_mask & SIG_MOTION_MSK:
            self.feature_page = MOTION_PAGE
            self.feature_reg_7 |= 0x1
            
        if f_mask & NO_MOTION_MSK:
            self.feature_page = MOTION_PAGE
            self.feature_reg_1 |= 0x8000
            
        if f_mask & ANY_MOTION_MSK:
            self.feature_page = SETTINGS_PAGE
            self.feature_reg_7 |= 0x8000
            
        self.feature_page = 0
            
        if enable_interrupt:
            self.enable_interrupt(mask=f_mask)
            
            
    def disable_features(self) -> None:
        """
        Disable all Features
        :return: None

        """
        self.disable_interrupt()
        
        #disable STEP_COUNTER, ACTIVITY_MSK, WRIST_GESTURE
        self.feature_page = STEP_COUNTER_PAGE4
        self.feature_reg_1 = 0
        reg = self.feature_reg_6
        reg &= ~0x20
        self.feature_reg_6 = reg
        
        #disable WRIST_WEAR
        self.feature_page = WRIST_WEAR_PAGE
        reg = self.feature_reg_0
        reg |= 0x10
        self.feature_reg_0 = reg
        
        #disable NO_MOTION, SIG_MOTION
        self.feature_page = MOTION_PAGE
        reg = self.feature_reg_7
        reg &= ~0x1
        self.feature_reg_7 = reg
        reg = self.feature_reg_1
        reg &= ~0x8000
        self.feature_reg_1 = reg
        
        #disable ANY_MOTION
        self.feature_page = SETTINGS_PAGE
        reg = self.feature_reg_7
        reg &= ~0x8000
        self.feature_reg_7 = reg
        
        self.feature_page = 0
            
            

    @property
    def acceleration(self) -> Tuple[float, float, float]:
        """
        Sensor Acceleration in :math:`m/s^2`
        """

        x = self._acc_data_x / self._acceleration_factor_cached * _STANDARD_GRAVITY
        y = self._acc_data_y / self._acceleration_factor_cached * _STANDARD_GRAVITY
        z = self._acc_data_z / self._acceleration_factor_cached * _STANDARD_GRAVITY

        return x, y, z

    @property
    def acceleration_range(self) -> str:
        """
        Sensor acceleration_range

        +------------------------------------+------------------+
        | Mode                               | Value            |
        +====================================+==================+
        | :py:const:`bmi270.ACCEL_RANGE_2G`  | :py:const:`0b00` |
        +------------------------------------+------------------+
        | :py:const:`bmi270.ACCEL_RANGE_4G`  | :py:const:`0b01` |
        +------------------------------------+------------------+
        | :py:const:`bmi270.ACCEL_RANGE_8G`  | :py:const:`0b10` |
        +------------------------------------+------------------+
        | :py:const:`bmi270.ACCEL_RANGE_16G` | :py:const:`0b11` |
        +------------------------------------+------------------+
        """
        values = (
            "ACCEL_RANGE_2G",
            "ACCEL_RANGE_4G",
            "ACCEL_RANGE_8G",
            "ACCEL_RANGE_16G",
        )
        return values[self._acceleration_range]

    @acceleration_range.setter
    def acceleration_range(self, value: int) -> None:
        if value not in acceleration_range_values:
            raise ValueError("Value must be a valid acceleration_range setting")
        self._acceleration_range = value
        self._acceleration_factor_cached = self.acceleration_scale[value]

    @property
    def acceleration_operation_mode(self) -> str:
        """
        Sensor acceleration_operation_mode

        +-----------------------------------------+-----------------+
        | Mode                                    | Value           |
        +=========================================+=================+
        | :py:const:`bmi270.ACCELERATOR_DISABLED` | :py:const:`0b0` |
        +-----------------------------------------+-----------------+
        | :py:const:`bmi270.ACCELERATOR_ENABLED`  | :py:const:`0b1` |
        +-----------------------------------------+-----------------+
        """
        values = ("ACCELERATOR_DISABLED", "ACCELERATOR_ENABLED")
        return values[self._acceleration_operation_mode]

    @acceleration_operation_mode.setter
    def acceleration_operation_mode(self, value: int) -> None:
        if value not in acceleration_operation_mode_values:
            raise ValueError(
                "Value must be a valid acceleration_operation_mode setting"
            )
        self._acceleration_operation_mode = value

    def load_config_file(self) -> None:
        """
        Load configuration file. This is necessary to use the sensor.
        Script adapted to use with MicroPython from:
        https://github.com/CoRoLab-Berlin/bmi270_python
        (c) 2023 MIT License Kevin Sommler
        """
        if self.internal_status == 0x01:
            print(hex(self._address), " -->BMI 270 Initialization already done")
        else:
            #from bmi270.config_file import bmi270_maximum_fifo_config_file as bmi270_config_file
            fw = open(os.getcwd()+'/bmi270fw.bin','r')
            print(hex(self._address), " -->BMI 270 Initializing...")
            self._power_configuration = 0x07
            time.sleep(0.00045)
            self._init_control = 0x00
            index = 0
            fw_buf = bytearray(257)
            fw_buf[0] = _INIT_DATA
            fw_mv = memoryview(fw_buf)
            while index < 8192:
                self._init_address_0 = ((index // 2) & 0xF)
                time.sleep_ms(1) 
                self._init_address_1 = ((index // 2) >> 4)
                time.sleep_ms(1) 
                if self._i2c:
                    fw.readinto(fw_mv[1:])
                    self._i2c.writeto_mem(self._address, _INIT_DATA, fw_mv[1:])
                else:
                    fw.readinto(fw_mv[1:])
                    self._cs.off()
                    self._spi.write(fw_mv)
                    self._cs.on()
                index += 256
                time.sleep_ms(1)
            self._init_control = 0x01
            time.sleep(0.02)
            if self.internal_status == 1:
                print( hex(self._address), " -->BMI 270 Initialization status OK")
            else:
                print( hex(self._address), " -->BMI 270 Initialization ERROR")

    @property
    def gyro_range(self) -> str:
        """
        Sensor gyro_range

        +------------------------------------+-------------------+
        | Mode                               | Value             |
        +====================================+===================+
        | :py:const:`bmi270.GYRO_RANGE_2000` | :py:const:`0b000` |
        +------------------------------------+-------------------+
        | :py:const:`bmi270.GYRO_RANGE_1000` | :py:const:`0b001` |
        +------------------------------------+-------------------+
        | :py:const:`bmi270.GYRO_RANGE_500`  | :py:const:`0b010` |
        +------------------------------------+-------------------+
        | :py:const:`bmi270.GYRO_RANGE_250`  | :py:const:`0b011` |
        +------------------------------------+-------------------+
        | :py:const:`bmi270.GYRO_RANGE_125`  | :py:const:`0b100` |
        +------------------------------------+-------------------+
        """
        values = (
            "GYRO_RANGE_2000",
            "GYRO_RANGE_1000",
            "GYRO_RANGE_500",
            "GYRO_RANGE_250",
            "GYRO_RANGE_125",
        )
        return values[self._gyro_range]

    @gyro_range.setter
    def gyro_range(self, value: int) -> None:
        if value not in gyro_range_values:
            raise ValueError("Value must be a valid gyro_range setting")
        self._gyro_range = value
        self._gyro_factor_cached = self.gyro_scale[value]

    @property
    def gyro(self) -> Tuple[float, float, float]:
        """
        Gyro values
        """

        x = self._gyro_data_x / self._gyro_factor_cached
        y = self._gyro_data_y / self._gyro_factor_cached
        z = self._gyro_data_z / self._gyro_factor_cached
        return x, y, z
    