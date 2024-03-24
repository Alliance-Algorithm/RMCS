#!/bin/bash

SRC_DIR=rmcs_ws/install
DST_DIR=ssh://remote//rmcs_install

unison -auto -batch -repeat watch -times ${SRC_DIR} ${DST_DIR} -force ${SRC_DIR}