#!/bin/bash

# ESSE SCRIPT DEVE RODAR DENTRO DO CONTAINER, NAO RODE NA SUA MAQUINA LOCAL

cp -r /opt/venv/* /host_data/

/opt/venv/bin/python -m gunicorn --bind 0.0.0.0:8000 -k uvicorn.workers.UvicornWorker main:app