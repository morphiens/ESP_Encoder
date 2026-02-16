#!/bin/bash
# ESP_Encoder - Environment Setup Script
# Sets up a shared Python virtual environment for all projects in this repo

set -e

echo "=== ESP Encoder Setup ==="
echo

# Check if Python 3 is available
if ! command -v python3 &> /dev/null; then
    echo "Error: python3 is not installed"
    exit 1
fi

PYTHON_VERSION=$(python3 --version)
echo "Found: $PYTHON_VERSION"

# Create virtual environment
if [ -d ".venv" ]; then
    echo "Removing existing .venv..."
    rm -rf .venv
fi

echo "Creating virtual environment..."
python3 -m venv .venv

# Activate and install
echo "Installing dependencies..."
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r requirements.txt

echo
echo "=== Setup Complete ==="
echo
echo "To activate the environment:"
echo "  source .venv/bin/activate"
echo
echo "Then navigate to any project directory to run its scripts."
echo "See each project's README for usage details."
echo
