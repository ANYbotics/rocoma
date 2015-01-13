#!/bin/bash

#-------------------------------------------------------------------------------
#
# Script to spawn all the existing screens with xterms
#
# Advanced Robotics Department
# Fondazione Istituto Italiano di Tecnologia
#
# Author: Jesus Ortiz
# email : jesus.ortiz@iit.it
# Date  : 24-Oct-2012
#
#-------------------------------------------------------------------------------

SCRIPT_FOLDER=$(dirname $0)


# Function to reset the terminal colors
function color_reset
{
  echo -ne "\033[0m"
}

# List of usefull colors
COLOR_RESET="\033[0m"
COLOR_INFO="\033[0;32m"
COLOR_ITEM="\033[1;34m"
COLOR_QUES="\033[1;32m"
COLOR_WARN="\033[0;33m"
COLOR_CODE="\033[0m"
COLOR_BOLD="\033[1m"
COLOR_UNDE="\033[4m"

color_reset
echo -e "${COLOR_INFO}Kill detached processes${COLOR_RESET}"

# Get list of detached screens
#SCREENS_DETACHED=$(screen -ls | awk '$4 == "(Detached)" {print $1}')
SCREENS_DETACHED=$(screen -ls | grep "Detached" | awk '{print $1}')


# Recover screens
for SCREEN in $SCREENS_DETACHED
do
  # Get screen name
  screen -S "$SCREEN" -X quit
  SCREEN=$(echo $SCREEN | awk 'BEGIN {FS="."}{print $2}')
  #screen -S "$SCREEN" -X quit
done

