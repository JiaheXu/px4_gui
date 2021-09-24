#!/bin/bash
cd ../
cd ../
pwd
gnome-terminal --geometry 60x20+10+10 -- bash env.sh & sleep 12
gnome-terminal --geometry 60x20+10+50 -- bash test_routine.sh & sleep 10
gnome-terminal --geometry 60x20+10+90 -- bash gui.sh
gnome-terminal --geometry 60x20+10+130 -- bash arm.sh & sleep 3
gnome-terminal --geometry 60x20+10+130 -- bash arm.sh & sleep 3

