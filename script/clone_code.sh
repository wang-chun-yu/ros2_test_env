#!/bin/bash

mkdir -p src
vcs import --input source.repos src
vcs pull -n src