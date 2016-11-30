#!/bin/bash

# this kills all processes that contain the string 'ros'

ps aux | grep ros | awk '{print $2}' | xargs kill
