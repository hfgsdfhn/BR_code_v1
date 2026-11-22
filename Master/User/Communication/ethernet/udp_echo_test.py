import argparse
import socket
import sys
import time


def main():
    parser = argparse.ArgumentParser(description="W5500 UDP echo test")
    parser.add_argument("--ip", default="192.168.1.100", help="W5500 IP address")
    parser.add_argument("--port", type=int, default=5000, help="W5500 UDP port")
    parser.add_argument("--bind", default="192.168.1.10", help="Local Ethernet IPv4 address")
    parser.add_argument("--count", type=int, default=10, help="Number of test packets")
    parser.add_argument("--interval", type=float, default=0.2, help="Delay between packets")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    sock.bind((args.bind, 0))

    print("Testing {}:{} from {}".format(args.ip, args.port, args.bind))
    success_count = 0

    for index in range(1, args.count + 1):
        payload = "W5500 test packet {}".format(index).encode("utf-8")
        started = time.perf_counter()

        try:
            sock.sendto(payload, (args.ip, args.port))
            response, remote = sock.recvfrom(2048)
            elapsed_ms = (time.perf_counter() - started) * 1000.0

            if response == payload:
                success_count += 1
                print("PASS packet={}: {} bytes, {:.1f} ms, from {}".format(
                    index, len(response), elapsed_ms, remote))
            else:
                print("FAIL packet={}: unexpected response {}".format(index, response))
        except socket.timeout:
            print("FAIL packet={}: timeout".format(index))
        except OSError as error:
            print("FAIL packet={}: {}".format(index, error))

        time.sleep(args.interval)

    sock.close()
    print("Result: {}/{} packets passed".format(success_count, args.count))
    sys.exit(0 if success_count == args.count else 1)


if __name__ == "__main__":
    main()
