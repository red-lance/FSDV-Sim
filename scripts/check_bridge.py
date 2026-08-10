#!/usr/bin/env python3
"""Health check for the Foxglove <-> sim pipeline. Dev tool, not a ROS node.

Connects to foxglove_bridge as a websocket client (the same way Foxglove
Studio does) and reports what the bridge is actually advertising. Answers the
recurring question "is the map issue the sim, the bridge, or the client?":

  - connection refused        -> bridge not running (or still in wait_for_sim)
  - get_map NOT advertised    -> bridge snapshot is stale: restart the stack
  - get_map advertised        -> server side is fine: reconnect/restart
                                 Foxglove Studio, the client state is stale

Usage: python3 scripts/check_bridge.py   (needs: pip install websockets)
"""

import asyncio
import json
import sys

try:
    import websockets
except ImportError:
    sys.exit("pip install --user websockets")

URI = "ws://localhost:8765"
WATCH = "/eufs_sim2/get_map"


async def main():
    try:
        ws = await websockets.connect(URI, subprotocols=["foxglove.sdk.v1"], compression=None)
    except Exception as e:
        sys.exit("cannot connect to %s (%s) -- bridge not running?" % (URI, e))

    channels, services = 0, []
    loop = asyncio.get_event_loop()
    deadline = loop.time() + 5
    async with ws:
        while (left := deadline - loop.time()) > 0:
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=left)
            except Exception:
                break
            if isinstance(raw, bytes):
                continue
            msg = json.loads(raw)
            if msg.get("op") == "advertise":
                channels += len(msg.get("channels", []))
            elif msg.get("op") == "advertiseServices":
                services.extend(msg.get("services", []))

    names = [s.get("name") for s in services]
    print("bridge OK: %d channels, %d services advertised" % (channels, len(services)))
    if WATCH in names:
        print("%s advertised -- server side is fine." % WATCH)
        print("If Foxglove still shows the map issue: reconnect or restart Foxglove Studio.")
    else:
        print("%s NOT advertised -- stale bridge snapshot." % WATCH)
        print("Restart the stack launch (and make sure the sim is up first).")


asyncio.run(main())
