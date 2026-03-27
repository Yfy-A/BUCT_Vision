import argparse
import glob
import time

import serial as ser


SEND_TEXT = "12345"
SEND_NUMBER = 678
SEND_SUFFIX = " I'm is xxxx.."
# DEFAULT_CANDIDATES = ["/dev/ttyTHS1", "/dev/ttyTHS2", "/dev/ttyUSB0", "/dev/ttyACM0"]
DEFAULT_CANDIDATES = ["/dev/ttyCH341USB0"]


def find_serial_port(preferred_port: str | None) -> str:
    if preferred_port:
        if not glob.glob(preferred_port):
            raise FileNotFoundError(
                f"指定串口不存在: {preferred_port}。当前可用串口: {', '.join(sorted(list_available_ports())) or '无'}"
            )
        return preferred_port

    available_ports = list_available_ports()
    for candidate in DEFAULT_CANDIDATES:
        if candidate in available_ports:
            return candidate

    raise FileNotFoundError(
        f"未找到可用串口。当前可用串口: {', '.join(available_ports) or '无'}"
    )


def list_available_ports() -> list[str]:
    patterns = (
        "/dev/ttyTHS*",
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
        "/dev/ttyS*",
        "/dev/ttyCH341USB*",
        "/dev/ttyCH34*",
    )
    ports: list[str] = []
    for pattern in patterns:
        ports.extend(glob.glob(pattern))
    return sorted(set(ports))


def main() -> None:
    parser = argparse.ArgumentParser(description="Basic UART send/receive test")
    parser.add_argument("--port", help="串口设备，例如 /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=9600, help="波特率，默认 9600")
    args = parser.parse_args()

    port = find_serial_port(args.port)
    print(f"using serial port: {port}")

    try:
        with ser.Serial(port, args.baudrate, timeout=0.5) as serial_conn:
            while True:
                serial_conn.write(SEND_TEXT.encode())
                # serial_conn.write(str(SEND_NUMBER).encode("GB2312"))
                # serial_conn.write(SEND_SUFFIX.encode("GB2312"))
                # serial_conn.write("\r\n".encode())

                received = serial_conn.readline().decode("GB2312", errors="replace").strip()
                # print(received if received else "<no response>")
                print("send ok!")
                print(serial_conn.is_open)
                time.sleep(0.2)
    except ser.SerialException as exc:
        available_ports = ", ".join(list_available_ports()) or "无"
        if "Permission denied" in str(exc):
            raise SystemExit(
                "串口打开失败: 权限不足。"
                "请将当前用户加入 dialout 组后重新登录，例如执行: sudo usermod -aG dialout $USER。"
                f"当前可用串口: {available_ports}"
            ) from exc
        raise SystemExit(f"串口打开失败: {exc}。当前可用串口: {available_ports}") from exc


if __name__ == "__main__":
    main()
    