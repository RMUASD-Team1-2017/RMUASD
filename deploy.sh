#!/bin/bash
containername=$1
docker login -u="$DOCKER_USERNAME" -p="$DOCKER_PASSWORD"
docker tag stefanrvo/$containername:latest stefanrvo/$containername:$TRAVIS_BUILD_NUMBER
docker tag stefanrvo/$containername:latest stefanrvo/$containername:$TRAVIS_COMMIT
docker push stefanrvo/$containername:$TRAVIS_BUILD_NUMBER
docker push stefanrvo/$containername:$TRAVIS_COMMIT

