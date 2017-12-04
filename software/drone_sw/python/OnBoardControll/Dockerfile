FROM python:2.7.14-jessie

SHELL ["/bin/bash", "-c"]
RUN mkdir -p /usr/src/
ADD . /usr/src/app/
WORKDIR /usr/src/app/

#Install python requirements
RUN pip install future
RUN pip install -r requirements.txt
RUN ["chmod", "+x", "run.sh"]
RUN echo "deb http://ftp.debian.org/debian jessie-backports main" >> /etc/apt/sources.list
RUN apt update -y && apt -t jessie-backports install  socat psmisc -y

EXPOSE 14541
ENTRYPOINT ["/usr/src/app/run.sh"]
CMD ["run"]
