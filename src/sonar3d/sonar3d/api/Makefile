# Makefile for building the Sonar 3D protocol buffer files
# This Makefile is used to download and install the protoc compiler
# and generate Python code from the .proto files.

.PHONY: all
all: protoc/bin/protoc
	@echo "BUILD PROTOC"
	protoc/bin/protoc --version
	protoc/bin/protoc --python_out=. sonar-3d-15-protocol.proto

protoc/bin/protoc:
	@echo "INSTALL PROTOC"
# Install protoc
	curl -L -o /tmp/protoc.zip https://github.com/protocolbuffers/protobuf/releases/download/v24.2/protoc-24.2-linux-x86_64.zip
	unzip -o /tmp/protoc.zip -d ./protoc
	rm -f /tmp/protoc.zip
