#!/bin/bash
# Sets up the local (ROS-free) test environment for the mrim_planner:
#   - a Python venv with numpy/scipy/scikit-learn/toppra
#   - a locally compiled LKH 2.0.10 TSP solver
# Usage: ./local_eval/setup.sh

set -e

MY_PATH=$(dirname "$0")
MY_PATH=$(cd "$MY_PATH" && pwd)

# 1) Python venv
if [ ! -f "$MY_PATH/venv/bin/python" ]; then
  echo ">>> creating venv..."
  python3 -m venv "$MY_PATH/venv"
fi
"$MY_PATH/venv/bin/pip" install --upgrade pip > /dev/null
"$MY_PATH/venv/bin/pip" install numpy scipy scikit-learn pyyaml matplotlib toppra

# 2) LKH solver
if [ ! -x "$MY_PATH/LKH-2.0.10/LKH" ]; then
  echo ">>> building LKH-2.0.10..."
  cd "$MY_PATH"
  if [ ! -f LKH-2.0.10.tgz ]; then
    wget -q http://webhotel4.ruc.dk/~keld/research/LKH/LKH-2.0.10.tgz
  fi
  tar xzf LKH-2.0.10.tgz
  cd LKH-2.0.10 && make
fi

echo ""
echo ">>> all set. Run e.g.:"
echo "    $MY_PATH/venv/bin/python $MY_PATH/harness/run_planner.py --problem apocalypse_small.problem"
