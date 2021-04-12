#!/bin/zsh

if [ $# -eq 0 ]
  then
    echo "No arguments supplied."
    exit 1
fi

ERRCODE=0
make || ERRCODE=$?

if [ $ERRCODE -ne 0 ]
  then
    exit $ERRCODE
fi

cd MyRayTracer
#echo "$@.p3f" > ../gdb_in.txt && gdb ../main -x ../gdb.txt
echo "$@.p3f" | ../main
