#!/usr/bin/env tclsh

package require genomix
genomix::connect
genomix1 load /home/theworld/openrobots/lib/genom/ros/plugins/optitrack.so
after 1000
optitrack::connect {host marey host_port 1510 mcast 239.192.168.30 mcast_port 1511}