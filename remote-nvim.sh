#!/usr/bin/bash

docker start rmcs-develop
docker-compose exec rmcs-develop /opt/nvim-linux-x86_64/bin/nvim
