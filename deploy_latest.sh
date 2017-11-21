#!/bin/bash
docker login -u="$DOCKER_USERNAME" -p="$DOCKER_PASSWORD"
docker push stefanrvo/rmuasd_webapp:latest
docker push stefanrvo/rmuasd_groundcontrol:latest
docker push stefanrvo/rmuasd_drone:latest
docker push stefanrvo/rmuasd_gazebo:latest
