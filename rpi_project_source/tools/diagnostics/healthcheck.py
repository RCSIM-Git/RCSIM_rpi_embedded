import json
import socket
import sys

# Simple healthcheck: send PING to local Supervisor UDP and expect PONG
HOST = "127.0.0.1"
PORT = 12348
MSG = json.dumps({"cmd": "PING"}).encode("utf-8")

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2.0)
try:
    s.sendto(MSG, (HOST, PORT))
    data, _ = s.recvfrom(1024)
    try:
        resp = json.loads(data.decode("utf-8"))
        if resp.get("status") == "PONG":
            print("OK")
            sys.exit(0)
    except Exception:
        pass
    print("NO_PONG")
    sys.exit(1)
except Exception as e:
    print("ERROR", e)
    sys.exit(1)
finally:
    s.close()
