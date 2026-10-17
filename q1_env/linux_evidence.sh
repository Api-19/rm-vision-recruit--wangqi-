#!/usr/bin/env bash
# 在已安装的Ubuntu里运行；不替你安装虚拟机，不在macOS伪造Linux结果。
set -eu
if [ "$(uname -s)" != Linux ]; then
  echo '请在Ubuntu 22.04终端运行本脚本。' >&2
  exit 1
fi
cat /etc/os-release
ls -la
top -b -n 1 | head -20
head /proc/cpuinfo
