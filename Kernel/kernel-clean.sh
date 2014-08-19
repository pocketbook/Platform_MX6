#!/bin/bash
echo "process command :make clean"
make clean

sleep 2

echo "process make distclean"
make distclean

exit 0
