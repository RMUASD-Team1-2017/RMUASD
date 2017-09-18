#!/bin/bash
docker login -u="$DOCKER_USERNAME" -p="$DOCKER_PASSWORD"
docker tag stefanrvo/rmuasdweb:latest stefanrvo/rmuasdweb:$TRAVIS_BUILD_NUMBER
docker tag stefanrvo/rmuasdweb:latest stefanrvo/rmuasdweb:$TRAVIS_COMMIT
docker tag stefanrvo/rmuasd_groundcontrol:latest stefanrvo/rmuasd_groundcontrol:$TRAVIS_BUILD_NUMBER
docker tag stefanrvo/rmuasd_groundcontrol:latest stefanrvo/rmuasd_groundcontrol:$TRAVIS_COMMIT
docker push stefanrvo/rmuasdweb:$TRAVIS_BUILD_NUMBER
docker push stefanrvo/rmuasdweb:$TRAVIS_COMMIT
docker push stefanrvo/rmuasd_groundcontrol:$TRAVIS_BUILD_NUMBER
docker push stefanrvo/rmuasd_groundcontrol:$TRAVIS_COMMIT
