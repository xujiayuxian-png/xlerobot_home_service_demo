"""Compare TCP reachability inside an inference-only network-restricted unit."""

import argparse
import errno
import ipaddress
import json
import socket


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--address', required=True)
    parser.add_argument('--port', required=True, type=int)
    parser.add_argument('--expect-blocked', action='store_true')
    args = parser.parse_args()
    target = ipaddress.ip_address(args.address)
    if target.version != 4 or target.is_loopback or not 1 <= args.port <= 65535:
        parser.error('a non-loopback literal IPv4 address and bounded port are required')
    with socket.socket() as client:
        client.settimeout(2)
        code = client.connect_ex((str(target), args.port))
    print(json.dumps({'connected': code == 0, 'errno': code, 'error': errno.errorcode.get(code, '')}))
    if (code == 0) == args.expect_blocked:
        raise SystemExit('network isolation expectation failed')


if __name__ == '__main__':
    main()
