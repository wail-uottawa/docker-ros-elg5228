#!/usr/bin/env bash
set -e

echo "Install TigerVNC server"
wget -qO- https://eecs.uottawa.ca/~wgueaieb/Teaching/tigervnc-1.8.0.x86_64.tar.gz | tar -xz --strip 1  -C / 
# wget -qO- https://github.com/TigerVNC/tigervnc/archive/refs/tags/v1.11.0.tar.gz | tar -xz --strip 1  -C / 
# wget -qO- https://dl.bintray.com/tigervnc/stable/tigervnc-1.8.0.x86_64.tar.gz | tar -xz --strip 1 -C /
