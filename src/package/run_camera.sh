#!/bin/bash

cd camera

# if the directory "env" does not exist, create a virtual environment
if [ ! -d "env" ]; then
	echo "Creating virtual environment..."
	python3 -m venv env
	source env/bin/activate
	echo "Installing dependencies..."
	pip install -r requirements.txt > /dev/null
fi

source env/bin/activate

python3 main.py
