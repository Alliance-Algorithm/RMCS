#!/bin/bash

docker start rmcs-develop
neovide --neovim-bin="docker-compose exec rmcs-develop /opt/nvim-linux-x86_64/bin/nvim"
