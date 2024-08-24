import serial
import struct

# ttyTHS1 포트를 115200 baudrate로 엽니다.
ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=1)

def calculate_checksum(packet):
    """Xbus 프로토콜의 체크섬 계산"""
    checksum = 0
    for byte in packet:
        checksum += byte
    checksum = 0xFF & ~checksum  # 1의 보수
    return checksum

def parse_xbus_packet(packet):
    """Xbus 패킷을 파싱하고, 데이터를 출력합니다."""
    if len(packet) < 5:
        return False  # 최소 길이보다 짧은 경우 무시
    
    # 패킷 구조에 맞게 데이터 분해
    preamble = packet[0]
    bid = packet[1]
    mid = packet[2]
    length = packet[3]
    data = packet[4:-1]
    checksum = packet[-1]

    # 체크섬 확인
    # if calculate_checksum(packet[:-1]) != checksum:
    #     print("Checksum mismatch, packet ignored.")
    #     return False

    # 데이터 해석 (예: MID에 따라 다르게 처리)
    # if mid == 0x36:  # 예시로 MID 0x36 처리
    #     print(f"Received data: {data}")
    # else:
    #     print(f"Unknown MID: {mid}")
    print(data)
    return True

try:
    while True:
        # 패킷 시작이 올 때까지 읽기
        start_byte = ser.read(1)
        if start_byte == b'\xFA':  # Xbus 패킷의 PREAMBLE은 0xFA
            # 패킷의 나머지 부분 읽기
            packet_length = 4  # 기본 길이(헤더 + 체크섬)
            packet = ser.read(packet_length)

            # 데이터 길이를 포함해 전체 패킷 읽기
            if len(packet) >= packet_length:
                length = packet[3]  # 데이터 길이
                packet += ser.read(length + 1)  # 데이터와 체크섬까지 읽기

                # 패킷 파싱
                parse_xbus_packet(packet)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    ser.close()
