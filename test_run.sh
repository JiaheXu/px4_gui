#!/bin/bash
gnome-terminal --geometry=10x10+10+10 -- bash env.sh & sleep 12
gnome-terminal --geometry=10x10+10+10 -- bash test_routine.sh & sleep 10
gnome-terminal --geometry=10x10+10+10 -- bash arm.sh 
gnome-terminal --geometry=10x10+10+10 -- bash arm.sh 
