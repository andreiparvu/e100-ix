#!/bin/sh

ifconfig ixeth0 up
ifconfig ixeth0 172.20.0.2 netmask 255.255.255.0 broadcast 0.0.0.0 hw ether 52:54:00:12:34:56

ifconfig ixeth1 up
ifconfig ixeth1 172.20.0.3 netmask 255.255.255.0 broadcast 0.0.0.0 hw ether 52:54:00:12:34:56

