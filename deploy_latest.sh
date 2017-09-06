#!/bin/bash
docker login -u="$DOCKER_USERNAME" -p="$DOCKER_PASSWORD"
docker push stefanrvo/rmuasdweb:latest
docker push stefanrvo/rmuasd_groundcontrol:latest
