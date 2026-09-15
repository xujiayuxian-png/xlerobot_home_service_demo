"""Render the manual-only demo service. Rendering never starts hardware."""

import argparse
from pathlib import Path
import pwd


def unit_text(root, user):
    root = str(Path(root).resolve())
    if any(character in root for character in '\n\r"%\\'):
        raise ValueError('unsupported character in service working directory')
    account = pwd.getpwnam(user)
    if account.pw_uid == 0:
        raise ValueError('the demo service must run as a non-root operator')
    return f'''[Unit]
Description=XLeRobot manual demo on X1
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User={user}
WorkingDirectory={root}
Environment=XLEROBOT_MANAGED_DEMO=1
Environment=PYTHONUNBUFFERED=1
Environment="HOME={account.pw_dir}"
ExecStart="{root}/tools/run" demo --hardware --config "{root}/config/local.yaml"
Restart=no
KillMode=control-group
KillSignal=SIGINT
TimeoutStopSec=30
SendSIGKILL=yes
LimitRTPRIO=50
LimitMEMLOCK=infinity
UMask=0077
StandardOutput=journal
StandardError=journal
'''


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', required=True)
    parser.add_argument('--user', required=True)
    args = parser.parse_args()
    print(unit_text(args.repo, args.user), end='')
