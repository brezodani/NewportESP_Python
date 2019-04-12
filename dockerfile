FROM python
COPY . /src
COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt
CMD ["python", "/src/python-example.py"]