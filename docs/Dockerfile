FROM python:3.11-slim

WORKDIR /docs

RUN pip install --no-cache-dir \
    sphinx==7.3.7 \
    sphinx-rtd-theme==2.0.0 \
    sphinx-autobuild==2024.2.4 \
    myst-parser==3.0.1

COPY source/ source/

# Default: live-reload server on :8000
# Override CMD for one-shot HTML build: sphinx-build -b html source build/html
CMD ["sphinx-autobuild", "source", "build/html", \
     "--host", "0.0.0.0", "--port", "8000", \
     "--watch", "source"]
