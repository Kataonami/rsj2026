#!/usr/bin/env bash

_rsj2026_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_cmeel_prefix="${_rsj2026_root}/.pinocchio-3.9/cmeel.prefix"

export CMAKE_PREFIX_PATH="${_cmeel_prefix}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${_cmeel_prefix}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PYTHONPATH="${_rsj2026_root}/.pinocchio-3.9:${_cmeel_prefix}/lib/python3.10/site-packages${PYTHONPATH:+:${PYTHONPATH}}"

unset _cmeel_prefix
unset _rsj2026_root
