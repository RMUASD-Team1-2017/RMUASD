docker pull stefanrvo/rmuasd_$containername:$TRAVIS_COMMIT
docker tag stefanrvo/rmuasd_$containername:$TRAVIS_COMMIT stefanrvo/rmuasd_$containername:latest
