#!/bin/bash
containername=$1
docker login -u="$DOCKER_USERNAME" -p="$DOCKER_PASSWORD"
docker tag stefanrvo/rmuasd_$containername:latest stefanrvo/rmuasd_$containername:$TRAVIS_BUILD_NUMBER
docker tag stefanrvo/rmuasd_$containername:latest stefanrvo/rmuasd_$containername:$TRAVIS_COMMIT
docker push stefanrvo/rmuasd_$containername:$TRAVIS_BUILD_NUMBER
docker push stefanrvo/rmuasd_$containername:$TRAVIS_COMMIT

