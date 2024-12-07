# SPDX-FileCopyrightText: Copyright (c) 2023 Jose D. Montoya
#
# SPDX-License-Identifier: MIT
"""
`i2c_helpers`
================================================================================

I2C Communications helpers


* Author(s): Jose D. Montoya

Based on

* adafruit_register.i2c_struct. Author(s): Scott Shawcroft
* adafruit_register.i2c_bits.  Author(s): Scott Shawcroft

MIT License

Copyright (c) 2016 Adafruit Industries

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

"""
# pylint: disable=too-many-arguments
import struct


class CBits:
    """
    Changes bits from a byte register
    """

    def __init__(
        self,
        num_bits: int,
        register_address: int,
        start_bit: int,
        register_width=1,
        lsb_first=True,
    ) -> None:
        self.bit_mask = ((1 << num_bits) - 1) << start_bit
        self.register = register_address
        self.star_bit = start_bit
        self.lenght = register_width
        self.lsb_first = lsb_first

    def __get__(
        self,
        obj,
        objtype=None,
    ) -> int:
        if obj._i2c:
            mem_value = obj._i2c.readfrom_mem(obj._address, self.register, self.lenght)
        else:
            obj._cs.off()
            mem_value = obj._spi.read(self.lenght +2, self.register | 0x80)
            obj._cs.on()
            mem_value = mem_value[2:]

        reg = 0
        order = range(len(mem_value) - 1, -1, -1)
        if not self.lsb_first:
            order = reversed(order)
        for i in order:
            reg = (reg << 8) | mem_value[i]

        reg = (reg & self.bit_mask) >> self.star_bit

        return reg

    def __set__(self, obj, value: int) -> None:
        if obj._i2c:
            memory_value = obj._i2c.readfrom_mem(obj._address, self.register, self.lenght)
        else:
            obj._cs.off()
            memory_value = obj._spi.read(self.lenght+2, self.register | 0x80)
            obj._cs.on()
            memory_value = memory_value[2:]

        reg = 0
        order = range(len(memory_value) - 1, -1, -1)
        if not self.lsb_first:
            order = range(0, len(memory_value))
        for i in order:
            reg = (reg << 8) | memory_value[i]
        reg &= ~self.bit_mask

        value <<= self.star_bit
        reg |= value
        reg = reg.to_bytes(self.lenght, "big")
        if obj._i2c:
            obj._i2c.writeto_mem(obj._address, self.register, reg)
        else:
            obj._cs.off()
            obj._spi.write(bytes([self.register])+reg)
            obj._cs.on()


class RegisterStruct:
    """
    Register Struct
    """

    def __init__(self, register_address: int, formr: str) -> None:
        self.format = formr
        self.register = register_address
        self.lenght = struct.calcsize(formr)

    def __get__(
        self,
        obj,
        objtype=None,
    ):
        if self.lenght <= 2:
            if obj._i2c:
                mem = memoryview(
                    obj._i2c.readfrom_mem(obj._address, self.register, self.lenght)
                ),
            else:
                obj._cs.off()
                mem = obj._spi.read(self.lenght+2, self.register | 0x80)
                obj._cs.on()
                mem = memoryview(mem[2:])
            value = struct.unpack(
                self.format,
                mem,
            )[0]
        else:
            if obj._i2c:
                mem = memoryview(
                    obj._i2c.readfrom_mem(obj._address, self.register, self.lenght)
                ),
            else:
                obj._cs.off()
                mem = obj._spi.read(self.lenght+2, self.register | 0x80)
                obj._cs.on()
                mem = memoryview(mem[2:])
            value = struct.unpack(
                self.format,
                mem,
            )
        return value

    def __set__(self, obj, value):
        try:
            mem_value = struct.pack(self.format, value)
        except:
            mem_value = struct.pack(self.format, *value)
        if obj._i2c:
            obj._i2c.writeto_mem(obj._address, self.register, mem_value)
        else:
            obj._cs.off()
            obj._spi.write(bytes([self.register])+mem_value)
            obj._cs.on()
