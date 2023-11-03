FROM docker.io/library/python:3.12 AS poetry-exporter

WORKDIR /work

RUN python -m pip install poetry poetry-plugin-export

COPY pyproject.toml pyproject.toml
COPY poetry.lock poetry.lock

RUN poetry export -o requirements.txt

FROM docker.io/library/python:3.12

# Fix numpy issues
ENV OPENBLAS_CORETYPE=AARCH64

WORKDIR /app

COPY --from=poetry-exporter /work/requirements.txt requirements.txt
RUN python -m pip install pip wheel --upgrade \
 && python -m pip install -r requirements.txt

COPY src .

CMD ["python", "fusion.py"]