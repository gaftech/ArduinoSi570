#!/bin/sh

DIR=".."
BASENAME="ArduinoSi570"
SUFFIX=_`git describe --tags --dirty`.zip

FULLNAME="$DIR/$BASENAME$SUFFIX"

echo "zipping * to $FULLNAME..."

zip $FULLNAME *

echo "$FULLNAME ready !"
