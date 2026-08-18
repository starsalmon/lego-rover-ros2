#!/usr/bin/env bash
# Re-enable services after agent build.
set -eo pipefail

for svc in bluetooth avahi-daemon unattended-upgrades fwupd; do
  sudo systemctl enable "$svc" 2>/dev/null || true
  sudo systemctl start "$svc" 2>/dev/null || true
done

echo "Services restored."
