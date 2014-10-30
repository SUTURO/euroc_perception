#!/bin/bash

grep -e '^gen\.add' cfg/SuturoPerception.cfg | perl -pe 's/gen\.add\("(.*?)".*", (.*?),.*/\1 = \2;/g'