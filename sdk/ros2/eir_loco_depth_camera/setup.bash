# Source after ROS 2 and depth_transport_msgs; resolves this prefix at runtime.
_eir_depth_prefix="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export AMENT_PREFIX_PATH="${_eir_depth_prefix}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
export PYTHONPATH="${_eir_depth_prefix}/lib/python3.10/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
unset _eir_depth_prefix
