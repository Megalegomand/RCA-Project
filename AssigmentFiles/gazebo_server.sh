#!/bin/bash

# Call: ./gazebo_server world.world


DIR="$( cd "$( dirname "$1" )" >/dev/null && pwd )"
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH=$DIR/models/

export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/:$GAZEBO_PLUGIN_PATH
export GAZEBO_PLUGIN_PATH=$DIR/marble_contact_plugin/build:$GAZEBO_PLUGIN_PATH

gzserver --verbose $1

