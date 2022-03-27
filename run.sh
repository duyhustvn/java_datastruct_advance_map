#!/bin/sh

echo "Build"
javac -cp src:libs/javax.json-1.0.4.jar src/application/MapApp.java

echo "Run"
java -cp src:libs/javax.json-1.0.4.jar application.MapApp
