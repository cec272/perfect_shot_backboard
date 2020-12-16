import asyncio
from constants import Descriptors, ACK_VALUE, Commands, getCommandName
from threading import Lock
import time


class DuplicateBackendError(Exception):
    pass


class Robot:
    def __init__(self, btif=None, pygatt=False, bleak=False):
        self.bt_interface = btif
        self.pygatt = pygatt
        self.bleak = bleak
        if(pygatt and bleak):
            raise DuplicateBackendError(
                "Cannot use pygatt and bleak backends concurrently")
        self.messageQueue = list()
        self.theLock = Lock()
        self.mValues = {"left": 0, "right": 0}
        self.lastTime = time.time()
        self.keyStatus = {"up": False, "down": False,
                          "left": False, "right": False}
        self.updateFlag = False
        if self.bleak:
            self.sendMessage = self.__bleak_sendMessage
            self.ping = self.__bleak_ping
            self.testByteStream = self.__bleak_testByteStream
            self.sendCommand = self.__bleak_sendCommand
        if self.pygatt:
            self.sendMessage = self.__pygatt_sendMessage

    async def __bleak_sendCommand(self, cmd, length=0, data=bytearray([])):
        await self.bt_interface.write_gatt_char(
            Descriptors["RX_CHAR_UUID"].value,
            bytearray([cmd.value, length]) + data)

    def setKey(self, key):
        try:
            prev = self.keyStatus[key]
            self.keyStatus[key] = True
            if not prev:
                self.keyboardUpdate()
        except KeyError:
            pass

    def unsetKey(self, key):
        try:
            prev = self.keyStatus[key]
            self.keyStatus[key] = False
            if prev:
                self.keyboardUpdate()
        except KeyError:
            pass

    async def __bleak_sendMessage(self, msg):
        await self.bt_interface.write_gatt_char(
            Descriptors["RX_CHAR_UUID"].value,
            bytearray([Commands.SER_RX.value, len(msg) + 1]) +
            msg.encode() + b'\x00')

    async def __bleak_ping(self):
        self.now = time.time()
        await self.bt_interface.write_gatt_char(
            Descriptors["RX_CHAR_UUID"].value,
            bytearray([Commands.PING.value] + 98*[0]))

    async def __bleak_testByteStream(self, length):
        print(f"Length is {length}")
        await self.bt_interface.write_gatt_char(
            Descriptors["RX_CHAR_UUID"].value,
            bytearray([Commands.START_BYTESTREAM_TX.value] + [1] + [length] + 96*[0]))

    def updateMotor(self, m, value):
        with self.theLock:
            try:
                self.mValues[m] = value
                # self.refreshMotors()
            except KeyError:
                pass

    def keyboardUpdate(self):
        newState = {"left": 0, "right": 0}
        if self.keyStatus["up"]:
            newState["left"] += 127
            newState["right"] += 127
        if self.keyStatus["left"]:
            newState["left"] -= 127
            newState["right"] += 127
        if self.keyStatus["right"]:
            newState["left"] += 127
            newState["right"] -= 127
        if self.keyStatus["down"]:
            newState["left"] -= 127
            newState["right"] -= 127
        for k, v in newState.items():
            if (v > 127):
                v = 127
            if (v < -127):
                v = -127
            if v < 0:
                v += 256
            self.mValues[k] = v
        self.updateFlag = True

    def refreshMotors(self):
        now = time.time()
        if (now-self.lastTime > 0.05):
            print("Refreshed Motors")
            self.lastTime = time.time()

    async def loopTask(self):
        if (self.updateFlag):
            now = time.time()
            if (now-self.lastTime > 0.05):
                await self.__bleak_setMotors(self.mValues["left"], self.mValues["right"])
                self.lastTime = time.time()
            self.updateFlag = False

    def __pygatt_setMotors(self, l, r):
        self.bt_interface.char_write(
            Descriptors["RX_CHAR_UUID"].value,
            bytearray([Commands.SET_MOTORS.value, 2, l, r]))

    async def __bleak_setMotors(self, l, r):
        await self.bt_interface.write_gatt_char(
            Descriptors["RX_CHAR_UUID"].value,
            bytearray([Commands.SET_MOTORS.value, 2, l, r]))

    def __pygatt_sendMessage(self, msg):
        # print("Trying to send a message")
        self.bt_interface.char_write(
            Descriptors["RX_CHAR_UUID"].value,
            bytearray([Commands.SER_RX.value, len(msg) + 1]) +
            msg.encode() + b'\x00')

    def pushMessage(self, msg):
        self.messageQueue.append(msg)

    def availMessage(self):
        return len(self.messageQueue) > 0

    def getMessage(self):
        if(len(self.messageQueue) > 0):
            return self.messageQueue.pop(0)
        else:
            return 0
