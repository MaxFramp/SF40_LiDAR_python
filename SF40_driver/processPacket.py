import time


def create_crc(data):
    crc = 0

    for i in data:
        code = crc >> 8
        code ^= int(i)
        code ^= code >> 4
        crc = crc << 8
        crc ^= code
        code = code << 5
        crc ^= code
        code = code << 7
        crc ^= code
        crc &= 0xFFFF

    return crc


def build_packet(command, write, data):
    payload_length = 1 + len(data)
    flags = (payload_length << 6) | (write & 0x1)
    packet_bytes = [0xAA, flags & 0xFF, (flags >> 8) & 0xFF, command]
    packet_bytes.extend(data)
    crc = create_crc(packet_bytes)
    packet_bytes.append(crc & 0xFF)
    packet_bytes.append((crc >> 8) & 0xFF)

    return bytearray(packet_bytes)





class Packet:
    def __init__(self):
        self.packetParseState = 0
        self.packetPayloadSize = 0
        self.packetSize = 0
        self.packetData = []


    def execute_command(self, serial_port, command, write, data, timeout=1):
        packet = build_packet(command, write, data)
        retries = 20

        while retries > 0:
            retries -= 1

            serial_port.write(packet)

            serial_response = Packet.wait_for_packet(self, serial_port, command, timeout)

            if serial_response is not None:
                self.packetData = serial_response
                return serial_response


        raise Exception('LWNX command failed to receive a response.')

    def parse_packet(self, byte):
        if self.packetParseState == 0:
            if byte == 0xAA:
                self.packetParseState = 1
                self.packetData = [0xAA]

        elif self.packetParseState == 1:
            self.packetParseState = 2
            self.packetData.append(byte)

        elif self.packetParseState == 2:
            self.packetParseState = 3
            self.packetData.append(byte)
            self.packetPayloadSize = (self.packetData[1] | (self.packetData[2] << 8)) >> 6
            self.packetPayloadSize += 2
            self.packetSize = 3

            if self.packetPayloadSize > 1019:
                self.packetParseState = 0

        elif self.packetParseState == 3:
            self.packetData.append(byte)
            self.packetSize += 1
            self.packetPayloadSize -= 1

            if self.packetPayloadSize == 0:
                self.packetParseState = 0
                crc = self.packetData[self.packetSize - 2] | (self.packetData[self.packetSize - 1] << 8)
                verify_crc = create_crc(self.packetData[0:-2])

                if crc == verify_crc:
                    return True

        return False

    def wait_for_packet(self, serial_port, command, timeout=1):
        self.packetParseState = 0
        self.packetData = []
        self.packetPayloadSize = 0
        self.packetSize = 0

        end_time = time.time() + timeout

        while True:
            if time.time() >= end_time:
                return None

            c = serial_port.read(1)

            if len(c) != 0:
                b = ord(c)
                if self.parse_packet(b):
                    if self.packetData[3] == command:
                        return self.packetData



    def read_str_16(self):  # Extract a 16-byte string from the data packet
        str16 = ''
        for i in range(0, 16):
            if self.packetData[4 + i] == 0:
                break
            else:
                str16 += chr(self.packetData[4 + i])

        return str16

    def read_signal_data(self):  # Extract signal data from data packet.
        alarm_state = self.packetData[4] << 0

        points_per_second = self.packetData[5] << 0
        points_per_second |= self.packetData[5 + 1] << 8

        forward_offset = self.packetData[7] << 0
        forward_offset |= self.packetData[7 + 1] << 8

        motor_voltage = self.packetData[9] << 0
        motor_voltage |= self.packetData[9 + 1] << 8

        revolution_index = self.packetData[11] << 0

        point_total = self.packetData[12] << 0
        point_total |= self.packetData[12 + 1] << 8

        point_count = self.packetData[14] << 0
        point_count |= self.packetData[14 + 1] << 8

        point_start_index = self.packetData[16]
        point_start_index |= self.packetData[16 + 1] << 8

        point_distances = self.packetData[18] << 0
        point_distances |= self.packetData[18 + 1] << 8

        return (alarm_state, points_per_second, forward_offset, motor_voltage, revolution_index, point_total,
                point_count, point_start_index, point_distances)


    def get_polar_distance_data(self):  # Extract angle and distance from data packet.
        point_total = self.packetData[12] << 0
        point_total |= self.packetData[12 + 1] << 8
        point_count = self.packetData[14] << 0
        point_count |= self.packetData[14 + 1] << 8
        point_start_index = self.packetData[16]
        point_start_index |= self.packetData[16 + 1] << 8

        point_distances = []
        point_index = []
        point_angles = []

        for i in range(point_count):
            dist = self.packetData[18 + 2 * i] << 0
            dist |= self.packetData[18 + 1 + 2 * i] << 8
            point_distances.append(dist)
            point_index.append(point_start_index + i + 1)
            point_angles.append(((point_index[i] / point_total) * 360))

        return point_angles, point_distances
