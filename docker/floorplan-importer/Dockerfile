# Floorplan importer (compose service `floorplan_importer`, companion/laptop, :9004).
# Standalone like docker/fleet-status and docker/scout-skills: no ROS, no DDS, no Docker
# socket. It turns a PDF into an occupancy-grid pair and POSTs it to the Pi's fleet_status
# over HTTP; nothing here touches the robot, so it needs no privileges.
#
# numpy and pymupdf ship manylinux wheels for amd64 and arm64 (nothing compiles). pymupdf 2.x
# declares onnxruntime/networkx/... as dependencies but plain `import pymupdf` needs none of
# them, hence --no-deps (requirements.txt lists numpy, which has no dependencies of its own).
FROM python:3.12-slim

RUN useradd --system --uid 10001 --no-create-home --shell /usr/sbin/nologin importer
WORKDIR /app
COPY requirements.txt /app/
RUN pip install --no-cache-dir --no-deps -r requirements.txt
COPY *.py /app/

# CI passes the commit sha so the sidecar of every installed map names the importer build.
ARG GIT_SHA=dev
ENV IMPORTER_VERSION=${GIT_SHA} \
    IMPORTER_PORT=9004 \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

USER importer
EXPOSE 9004
HEALTHCHECK --interval=30s --timeout=5s --start-period=10s --retries=3 \
    CMD ["python", "-c", "import os,urllib.request; urllib.request.urlopen('http://127.0.0.1:%s/api/health' % os.environ.get('IMPORTER_PORT', '9004'), timeout=4)"]
CMD ["python", "/app/server.py"]
