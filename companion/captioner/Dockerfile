# Batch anomaly captioner (inspection F2). Own image per the project's
# one-image-per-heavy-dep pattern — torch-cpu is ~2 GB and nothing else on the
# companion needs it. Deliberately NO ROS: the captioner reads the run's .mcap
# directly (mcap + mcap-ros2-support decode CDR without a ROS install).
FROM python:3.11-slim

RUN pip install --no-cache-dir \
        --index-url https://download.pytorch.org/whl/cpu torch \
    && pip install --no-cache-dir \
        transformers pillow einops timm \
        mcap mcap-ros2-support

# Bake the weights so runs are fully offline (corp VLAN / no registry pulls at
# inspection time). Florence-2 needs trust_remote_code for its modeling files.
RUN python -c "from huggingface_hub import snapshot_download; \
    snapshot_download('microsoft/Florence-2-base')"

COPY captioner.py /app/captioner.py
WORKDIR /captures/inspection
ENTRYPOINT ["python", "/app/captioner.py"]
