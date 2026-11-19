FROM python:3.12-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY app.py setup.py ./
COPY static ./static
RUN useradd -r app && chown -R app /app
USER app
EXPOSE 8443
CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "8443", \
     "--ssl-keyfile", "certs/server.key", "--ssl-certfile", "certs/server.crt"]
