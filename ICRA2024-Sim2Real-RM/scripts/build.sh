#!/bin/bash
CLIENT_IMAGE=${CLIENT_IMAGE:-test}
docker build . -t $CLIENT_IMAGE 


# CLIENT_IMAGE=shockley/meta_sim:v2.7 bash scripts/build.sh