import asyncio
import struct
import rtmidi

from bleak import BleakScanner, BleakClient, BleakGATTCharacteristic
import PyObjCTools

midiout = rtmidi.MidiOut()
availablePorts = midiout.get_ports()

if availablePorts:
    midiout.open_port(0)
    print("available")
else:
    midiout.open_virtual_port("My virtual output")


async def main():
    myDevice = None
    devices = await BleakScanner.discover()
    for d in devices:
        if PyObjCTools.KeyValueCoding.getKey(d.details, 'name')[0] == 'Arduino':
            myDevice = d
            print('Found it')

    print(PyObjCTools.KeyValueCoding.getKey(myDevice.details, 'identifier'))

    address = str(PyObjCTools.KeyValueCoding.getKey(
        myDevice.details, 'identifier')[0])
    async with BleakClient(address) as client:
        # svcs = await client.get_services()
        # for service in svcs:
        #     print(service)
        # await client.start_notify('19B10001-E8F2-537E-4F6C-D104768A1214', callback)
        # await asyncio.sleep(10)
        while True:
            accX = await client.read_gatt_char('0099b53a-b473-11ed-afa1-0242ac120002')
            accY = await client.read_gatt_char('0099b8a0-b473-11ed-afa1-0242ac120002')
            accZ = await client.read_gatt_char('0099ba12-b473-11ed-afa1-0242ac120002')
            # print(led)
            # print(int.from_bytes(led,byteorder='big'))
            accXfloat = struct.unpack('<f', accX)[0]
            accYfloat = struct.unpack('<f', accY)[0]
            accZfloat = struct.unpack('<f', accZ)[0]
            print("X: " + str(accXfloat))
            print("Y: " + str(accYfloat))
            print("Z: " + str(accZfloat))

            if (.95 < accXfloat < 1.15) and (-0.1 < accYfloat < 0.1) and (-0.1 < accZfloat < 0.1):
                midiout.send_message([0x90, 60, 100])
                print("x positive")
            # else:
            #     midiout.send_message([0x80, 60, 0])
            if (.95 < accYfloat < 1.15) and (-0.1 < accXfloat < 0.1) and (-0.1 < accZfloat < 0.1):
                midiout.send_message([0x90, 62, 100])
                print("y positive")
            # else:
            #     midiout.send_message([0x80, 62, 0])
            if (.95 < accZfloat < 1.15) and (-0.1 < accXfloat < 0.1) and (-0.1 < accYfloat < 0.1):
                midiout.send_message([0x90, 64, 100])
                print("z positive")
            # else:
            #     midiout.send_message([0x80, 64, 0])
            if (-1.15 < accXfloat < -.95) and (-0.1 < accYfloat < 0.1) and (-0.1 < accZfloat < 0.1):
                midiout.send_message([0x90, 65, 100])
                print("x negative")
            # else:
            #     midiout.send_message([0x80, 65, 0])
            if (-1.15 < accYfloat < -.95) and (-0.1 < accXfloat < 0.1) and (-0.1 < accZfloat < 0.1):
                midiout.send_message([0x90, 67, 100])
                print("y negative")
            # else:
            #     midiout.send_message([0x80, 67, 0])
            if (-1.15 < accZfloat < -.95) and (-0.1 < accXfloat < 0.1) and (-0.1 < accYfloat < 0.1):
                midiout.send_message([0x90, 69, 100])
                print("z negative")
            # else:
            #     midiout.send_message([0x80, 69, 0])
            await asyncio.sleep(.1)


def callback(sender: BleakGATTCharacteristic, data: bytearray):
    # print(f"{sender}: {data}")
    print(struct.unpack('<f', data))


asyncio.run(main())
