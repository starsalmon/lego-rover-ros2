#!/usr/bin/env bash
# Write FastDDS initial-peers profile for cross-host discovery (Mac <-> Pi).
# Usage: write_fastdds_peer.sh <peer_ip> <output_xml>
set -eo pipefail

PEER_IP="${1:?peer IP required}"
OUT="${2:?output path required}"

cat >"$OUT" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="rover_peer" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
          <locator>
            <udpv4>
              <address>${PEER_IP}</address>
            </udpv4>
          </locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
EOF

echo "$OUT"
