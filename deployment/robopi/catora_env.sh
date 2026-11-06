#!/bin/sh
. /data/aerohub/as2/board_env.sh
for prefix in /data/aerohub/lib/catora/install/*; do
  [ -d "$prefix/share" ] || continue
  export AMENT_PREFIX_PATH="$prefix:$AMENT_PREFIX_PATH"
  export LD_LIBRARY_PATH="$prefix/lib:$LD_LIBRARY_PATH"
  export PYTHONPATH="$prefix/lib/python3.12/site-packages:$PYTHONPATH"
done
