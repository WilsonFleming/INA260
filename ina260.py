"""MicroPython library for the INA260 sensor.
This library supports the INA260 sensor from Texas Instruments with
MicroPython using the I2C bus.
"""
import utime
from machine import I2C
from math import trunc
from micropython import const


class INA260:
    __REG_CONFIG = const(0x00)  # CONFIGURATION REGISTER (R/W)
    __REG_CURRENT = const(0x01)  # SHUNT VOLTAGE REGISTER (R)
    __REG_BUSVOLTAGE = const(0x02)  # BUS VOLTAGE REGISTER (R)
    __REG_POWER = const(0x03)  # POWER REGISTER (R)
    __REG_MASK_ENABLE = const(0x06)  # MASK ENABLE REGISTER (R/W)
    __REG_ALERT_LIMIT = const(0x07)  # ALERT LIMIT REGISTER (R/W)
    __REG_MFG_UID = const(0xFE)  # MANUFACTURER UNIQUE ID REGISTER (R)
    __REG_DIE_UID = const(0xFF)  # DIE UNIQUE ID REGISTER (R)

    """Modes avaible to be set
    +--------------------+---------------------------------------------------------------------+
    | Mode               | Description                                                         |
    +====================+=====================================================================+
    | ``Mode.CONTINUOUS``| Default: The sensor will continuously measure the bus voltage and   |
    |                    | shunt voltage across the shunt resistor to calculate ``power`` and  |
    |                    | ``current``                                                         |
    +--------------------+---------------------------------------------------------------------+
    | ``Mode.TRIGGERED`` | The sensor will immediately begin measuring and calculating current,|
    |                    | bus voltage, and power. Re-set this mode to initiate another        |
    |                    | measurement                                                         |
    +--------------------+---------------------------------------------------------------------+
    | ``Mode.SHUTDOWN``  |  Shutdown the sensor, reducing the quiescent current and turning off|
    |                    |  current into the device inputs. Set another mode to re-enable      |
    +--------------------+---------------------------------------------------------------------+
    """
    SHUTDOWN = const(0x0)
    TRIGGERED = const(0x3)
    CONTINUOUS = const(0x7)

    _OPERATING_MODES = [ SHUTDOWN, TRIGGERED, CONTINUOUS ]

    """Options for ``current_conversion_time`` or ``voltage_conversion_time``
    +----------------------------------+------------------+
    | ``ConversionTime``               | Time             |
    +==================================+==================+
    | ``ConversionTime.TIME_140_us``   | 140 us           |
    +----------------------------------+------------------+
    | ``ConversionTime.TIME_204_us``   | 204 us           |
    +----------------------------------+------------------+
    | ``ConversionTime.TIME_332_us``   | 332 us           |
    +----------------------------------+------------------+
    | ``ConversionTime.TIME_558_us``   | 588 us           |
    +----------------------------------+------------------+
    | ``ConversionTime.TIME_1_1_ms``   | 1.1 ms (Default) |
    +----------------------------------+------------------+
    | ``ConversionTime.TIME_2_116_ms`` | 2.116 ms         |
    +----------------------------------+------------------+
    | ``ConversionTime.TIME_4_156_ms`` | 4.156 ms         |
    +----------------------------------+------------------+
    | ``ConversionTime.TIME_8_244_ms`` | 8.244 ms         |
    +----------------------------------+------------------+
    """

    TIME_140_us = const(0x0)
    TIME_204_us = const(0x1)
    TIME_332_us = const(0x2)
    TIME_558_us = const(0x3)
    TIME_1_1_ms = const(0x4)
    TIME_2_116_ms = const(0x5)
    TIME_4_156_ms = const(0x6)
    TIME_8_244_ms = const(0x7)

    _TIMES = [ TIME_140_us, TIME_204_us, TIME_332_us, TIME_558_us, TIME_1_1_ms, TIME_2_116_ms, TIME_4_156_ms, TIME_8_244_ms ]

    ## TODO : add function for time enum

    """Options for ``averaging_count``
    +-------------------------------+------------------------------------+
    | ``AveragingCount``            | Number of measurements to average  |
    +===============================+====================================+
    | ``AveragingCount.COUNT_1``    | 1 (Default)                        |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_4``    | 4                                  |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_16``   | 16                                 |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_64``   | 64                                 |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_128``  | 128                                |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_256``  | 256                                |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_512``  | 512                                |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_1024`` | 1024                               |
    +-------------------------------+------------------------------------+
    """
    COUNT_1 = const(0x0)
    COUNT_4 = const(0x1)
    COUNT_16 = const(0x2)
    COUNT_64 = const(0x3)
    COUNT_128 = const(0x4)
    COUNT_256 = const(0x5)
    COUNT_512 = const(0x6)
    COUNT_1024 = const(0x7)

    AVERAGING_VALS = [ COUNT_1, COUNT_4, COUNT_16, COUNT_64, COUNT_128, COUNT_256, COUNT_512, COUNT_1024 ]

    _AVG2_VALUE = 11
    _AVG1_VALUE = 10
    _AVG0_VALUE = 9
    _VBUSCT2_VALUE = 8
    _VBUSCT1_VALUE = 7
    _VBUSCT0_VALUE = 6
    _ISHCT2_VALUE = 5
    _ISHCT1_VALUE = 4
    _ISHCT0_VALUE = 3
    _MODE3_VALUE = 2
    _MODE2_VALUE = 1
    _MODE1_VALUE = 0

    TEXAS_INSTRUMENT_ID = const(0x5449)
    INA260_ID = const(0x227)

    # LSB 1.25 mV
    _VOLTAGE_LSB = 1.25
    # LSB 10 mW - maximum that can be returned in 419.43 W
    _POWER_LSB = 10
    # LSB 1.25 mA
    _CURRENT_LSB = 1.25

    ## TODO : Add function for averaging count

    def __init__(self, i2c = None, address=0x40, sda='P9', scl='P10'):
        if i2c is not None:
            self._i2c = i2c
        else:
            self.i2c = I2C(0, mode=I2C.MASTER, pins=(sda, scl))
        
        self._sda = sda
        self._scl = scl
        self._i2c = i2c
        self._address = address

        self.temp_store = 0xFFFF

        self._manufacturer_id = self.__read_register(__REG_MFG_UID)

        if self._manufacturer_id != TEXAS_INSTRUMENT_ID:
            raise RuntimeError(
                "Failed to find Texas InstrumentID, read {} while expected {}"
                " - check your wiring!".format(
                    self._manufactuerer_id,
                    TEXAS_INSTRUMENT_ID
                )
            )

        self._device_id = ( self.__read_register(__REG_DIE_UID) >> 4 )

        if self._device_id != INA260_ID:
            raise RuntimeError (
                "Failed to find INA260 ID, read {} while expected {}"
                " - check your wiring".format(
                    self._device_id,
                    self.INA260_ID)
            )
        
        
    def configure(self):
        ## TODO : Complete configure
        return

    def current(self):
        """
        Returns the bus current in milliamps
        """
        return self.__raw_current() * self._CURRENT_LSB

    def voltage(self):
        """
        Returns the bus voltage in milliVolts
        """
        return self.__raw_voltage() * self._VOLTAGE_LSB

    def power(self):
        """
        Returns the power consumed in mW
        """
        # TODO : Check register for power overflow
        power = self.__raw_power() * self._POWER_LSB
        return power

    def reset(self):
        """
        Generates a system reset of the INA260 that is the same
        as the power-on reset.
        Resets all resgiters to default values.
        Register self clears.
        """
        self.__set_register_bit(__REG_CONFIG, 15, 1)

    def set_averaging_mode(self, average):
        if average not in self.AVERAGING_VALS:
            # TODO: Logging / Debugging
            return
        
        # Get register value
        reg = self.__read_register(__REG_CONFIG)

        mask = 0xE00

        reg = (reg & ~mask) | ((average << 9) & mask)

        # Write register value
        self.__write_register(__REG_CONFIG, reg)

    def set_operating_mode(self, mode):
        if mode not in self._OPERATING_MODES:
            # TODO: Logging/Debug
            return

        reg = self.__read_register(__REG_CONFIG)

        mask = 0x7

        reg = (reg & ~mask) | (mode & mask)

        self.__write_register(__REG_CONFIG, reg)

    def set_vbus_conversion_timer(self, conversion_time):
        if conversion_time not in self._TIMES:
            print("Failed")
            # TODO: Logging/Debugging
            return

        reg = self.__read_register(__REG_CONFIG)

        mask = 0x1c0

        reg = (reg & ~mask) | (( conversion_time << 6 ) & mask)

        self.__write_register(__REG_CONFIG, reg)

    def set_shunt_current_conversion_timer(self, conversion_time):
        if conversion_time not in self._TIMES:
            print("Failed")
            # TODO: Logging
            return

        reg = self.__read_register(__REG_CONFIG)

        mask = 0x38

        reg = (reg & ~mask) | ((conversion_time << 3 ) & mask)

        self.__write_register(__REG_CONFIG, reg)

    def get_configuration(self):
        return self.__read_register(__REG_CONFIG)

    def __raw_current(self):
        return self.__read_register(__REG_CURRENT, True)

    def __raw_voltage(self):
        return self.__read_register(__REG_BUSVOLTAGE)

    def __raw_power(self):
        return self.__read_register(__REG_POWER)

    def __configuration_register(self, register_value):
        self.__write_register(__REG_CONFIG, register_value)

    def __read_register_bit(self, register, register_bit):
        configuration = self.__read_register(register)
        return ((configuration & ( 1 << register_bit)) >> register_bit)

    def __set_register_bit(self, register, register_bit, value):
        reg = self.__read_register(self, register)
        if ((reg & ( 1<< register_bit)) >> register_bit) != value:
            register_bytes = reg ^ (1 << register_bit)
            self.__write_register(register, register_bytes)

    def __read_configuration(self):
        return self.__read_register(self.__REG_CONFIG)

    def __write_register(self, register, register_value):
        register_bytes = self.__to_bytes(register_value)
        self._i2c.writeto_mem(self._address, register, register_bytes)

    def __to_bytes(self, register_value):
        return bytearray([(register_value >> 8) & 0xFF, register_value & 0xFF])

    def __read_register(self, register, negative_value_supported=False):
        register_bytes = self._i2c.readfrom_mem(self._address, register, 2)
        register_value = int.from_bytes(register_bytes, 'big')
        if negative_value_supported:
            # Two's compliment
            if register_value > 32767:
                register_value -= 65536

        return register_value

    def __to_bytes(self, register_value):
        return bytearray([(register_value >> 8) & 0xFF, register_value & 0xFF])

