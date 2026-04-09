#!/bin/bash

echo "🚀 Setting up Ball Balancing Robot environment..."

# 1. Create virtual environment
if [ ! -d "venv" ]; then
    echo "📦 Creating virtual environment..."
    python3 -m venv venv
else
    echo "✅ Virtual environment already exists"
fi

# 2. Activate venv
echo "⚡ Activating virtual environment..."
source venv/bin/activate

# 3. Upgrade pip
echo "⬆️ Upgrading pip..."
pip install --upgrade pip

# 4. Install dependencies
if [ -f "requirements.txt" ]; then
    echo "📥 Installing requirements..."
    pip install -r requirements.txt
else
    echo "requirements.txt not found!"
    exit 1
fi

# 5. Run the program
echo "Running robot code..."
python ball_tracker.py