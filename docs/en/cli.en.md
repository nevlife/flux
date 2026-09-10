# flux command

The `flux` command shipped by `flux_cli`. It looks into the flux channels on this host.

It only reads. Enumeration takes no lock, and the statistics path takes neither a lock nor a borrow. So attaching it to a running system does not disturb it, and the target process cannot tell that this command ran.

It uses neither ROS nor DDS. It is daemonless. The data lives in `/dev/shm`, so it runs as is on a deployment without ROS installed.

## Commands

```bash
flux topic list                      # list channels
flux topic info /cam/left            # details of one channel
flux topic hz /cam/left              # publish rate
flux domain list                      # domains on this host and mine
```

`--domain` picks the domain to look at, and `--all-domains` looks at all of them. The default is decided by the same rule a node uses (`FLUX_DOMAIN`, else `ROS_DOMAIN_ID`, else `0`). The two flags attach to the verb. `flux topic list --all-domains` is where a person types it.

## list

```text
CHANNEL        DOMAIN  SEGMENT  ENDPOINTS
-------------  -----  -------  ------------
/cam/left      lab    up       1 pub, 1 sub
/lidar/points  lab    down     1 sub
```

A `SEGMENT` of `down` is not a fault. The signpost is permanent, so the name remains even with no publisher.

If `(from name)` follows the channel name, that name was read back from the `/dev/shm` name rather than from a live participant. Names replace every non-alnum character with `.`, so `/a/b` and `.a.b` are one name. Without the marker it is the real key.

Lookup accepts both. Giving `/cam/left` or `.cam.left` to `info` and `hz` finds the same channel. A channel whose publisher died is visible only in the read-back spelling, and that is usually the situation in which one looks up a channel.

## info

```text
channel      /cam/left
domain        lab
fingerprint  0x0000000000000abc
signpost     /flux.v7.slab..cam.left.0000000000000abc
segment      up, epoch 1
slots        8 x 1 MiB
storage      host
published    1594 frames on this segment
parked       0 subscriber(s) on the wake gate
endpoints
  ROLE  PID      LABEL
  ----  -------  -----
  pub   1108498  -
  sub   1108498  -
```

`LABEL` is the value the boundary layer announced. `flux_cpp` puts in the node's fully qualified name, and `flux_py` leaves it empty because there is no node.

If there are two channels with the same key but different fingerprints, they cannot be told apart by name, so both are shown and the lookup is refused. Those two are separate channels and do not communicate with each other.

## hz

```text
sampling /cam/left every 1s (Ctrl-C to stop)
     29.99 Hz   (30 frames in 1.00s)
     29.99 Hz   (30 frames in 1.00s)
```

It reads `publish_seq` in the segment header twice and takes the difference. That value is a monotone ticket issuer, so the result is not an estimate but the exact frame count for that interval. It does not attach as a subscriber, so it occupies neither `max_borrow` nor the holder table, and the endpoint count shown by `flux topic info` does not change while `hz` runs.

If `epoch` changes, the publisher group restarted. The counter is new, so a difference across that boundary is not a frame count. It is not counted and is reported instead.

## domain list

```text
DOMAIN      CHANNELS  UP  ENDPOINTS
---------  --------  --  ---------
0          96        0   0
10 (here)  2         1   2

more than one domain here: processes in different domains never meet.
domain comes from FLUX_DOMAIN, else ROS_DOMAIN_ID, else '0'.
```

`(here)` is the domain this process resolved. It does not accept `--domain`. What this command answers is "where does the environment put me", and overriding it would answer a question that was not asked.

It is decided by the same rule and at the same moment as the value a node sees (`process_domain()`). So the `(here)` of this command and the `domain` of a node started from the same shell cannot disagree.

The resolved domain stays as a row even when empty. Its absence is exactly the value one came to check. If there is more than one row, that fact is stated as a sentence.

There is one situation this answers. A subscription receives no frames and `refused` is all zero. A publisher that does not exist yet and a publisher running in a different domain look the same from the outside. In the latter case that publisher appears in another row. The node-side way to ask the same thing is `domain` and `attached` on `Subscription` (`api.en.md`).

## What is missing

There is no `echo`. Seeing frames requires attaching as a real subscriber, which then occupies `max_borrow` and the holder table. That is the point where observation starts to disturb the target, so it was not put in the same set as the three above.
