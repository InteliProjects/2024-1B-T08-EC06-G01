# For more information, please refer to https://aka.ms/vscode-docker-python
FROM python:3.11-bullseye

EXPOSE 8000

# Keeps Python from generating .pyc files in the container
ENV PYTHONDONTWRITEBYTECODE=1

# Turns off buffering for easier container logging
ENV PYTHONUNBUFFERED=1


# Install libGL for OpenCV support
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# RUN apt-get install python3.10-venv

# Install pip requirements
# RUN python -m pip install -U pip
# RUN python -m pip install -U setuptools
COPY requirements.txt .

# mount a venv to ./.venv to speed up builds
COPY .venv /opt/venv
RUN python -m venv /opt/venv

RUN /opt/venv/bin/python -m pip install -r requirements.txt

# save the venv to the mounted volume
# i dont know what to do here

WORKDIR /app
COPY src/ /app

# COPY ../ia/models/yolo_v8_n_dirt_detection.pt /app/client/

# Creates a non-root user with an explicit UID and adds permission to access the /app folder
# For more info, please refer to https://aka.ms/vscode-docker-python-configure-containers
# RUN adduser -u 5678 --disabled-password --gecos "" appuser && chown -R appuser /app
# USER appuser

# During debugging, this entry point will be overridden. For more information, please refer to https://aka.ms/vscode-docker-python-debug
# CMD ["cp", "-R" ,"/opt/venv/*", "/host_data/ &&", "/opt/venv/bin/python", "-m", "gunicorn", "--bind", "0.0.0.0:8000", "-k", "uvicorn.workers.UvicornWorker", "main:app"]
COPY run-inside-docker.sh /app/run.sh

CMD ["/app/run.sh"]