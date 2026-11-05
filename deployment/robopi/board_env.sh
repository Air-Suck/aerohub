#!/bin/sh
# Source on the OHOS board after extracting the private AS2 overlay.
export PATH=/sys_prod/robot/out/bin:/sys_prod/robot/install/bin:$PATH
export LD_LIBRARY_PATH=/sys_prod/robot/install/lib:/sys_prod/robot/out/lib
export PYTHONPATH=/sys_prod/robot/install/lib/python3.12/site-packages:/sys_prod/robot/out/lib/python3.12/site-packages
export AMENT_PREFIX_PATH=/sys_prod/robot/install
for prefix in /data/local/tmp/aerohub-as2/install/*; do
  [ -d "$prefix/share" ] || continue
  export AMENT_PREFIX_PATH="$prefix:$AMENT_PREFIX_PATH"
  export LD_LIBRARY_PATH="$prefix/lib:$LD_LIBRARY_PATH"
  export PYTHONPATH="$prefix/lib/python3.12/site-packages:$PYTHONPATH"
done
export PYTHONDONTWRITEBYTECODE=1
