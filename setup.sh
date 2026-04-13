#!/bin/bash

echo "Setting up Ball Balancing Robot environment..."

# 1. System dependencies (MUST be installed before Python libs)
echo "Installing system dependencies..."
sudo apt update
sudo apt install -y \
python3-rpi.gpio \
python3-rpi-lgpio \
build-essential \
python3-dev

# 2. Create virtual environment
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
else
    echo "Virtual environment already exists"
fi

# 3. Activate venv
echo "Activating virtual environment..."
source venv/bin/activate

# 4. Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# 5. Install Python dependencies
if [ -f "requirements.txt" ]; then
    echo "Installing requirements..."
    pip install -r requirements.txt
else
    echo "requirements.txt not found"
    exit 1
fi

# 6. Run the program
echo "Running robot code..."
python ball_tracker.py
