#!/bin/bash
CLIENT_IMAGE=${CLIENT_IMAGE:-test}
docker build . -t $CLIENT_IMAGE

# CLIENT_IMAGE=meta_sim/test:v1.2 bash scripts/build.sh