#!/usr/bin/env bash
set -e

echo "========================================================="
echo " Autonomous AI Mobile Robot - Full Benchmark Pipeline"
echo "========================================================="

PYTHON_BIN="py"
if command -v python3 &> /dev/null; then
    PYTHON_BIN="python3"
fi

if [ -f "$HOME/.agent-reach-venv/Scripts/python.exe" ]; then
    PYTHON_BIN="$HOME/.agent-reach-venv/Scripts/python.exe"
fi

echo "Using Python binary: $PYTHON_BIN"

echo "[1/3] Running Unit Test Suite..."
$PYTHON_BIN -m unittest discover -s tests -p "test_*.py"

echo "[2/3] Generating Engineering Benchmark Graphs & Report..."
$PYTHON_BIN src/robot_benchmarking/robot_benchmarking/generate_report.py

echo "[3/3] Generating Animated Visualisation GIFs..."
$PYTHON_BIN src/robot_benchmarking/robot_benchmarking/generate_gifs.py

echo "========================================================="
echo " Benchmark Pipeline Complete!"
echo " Results available in results/graphs, results/gifs, and results/report."
echo "========================================================="
