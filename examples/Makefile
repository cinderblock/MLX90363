# Build the MLX90363 example firmware via uMaker. Used in CI to verify that
# the templated header compiles + links against AVR++ on a real AVR target.

cppNames = main

F_CPU  = 16000000UL
MCU    = atmega32u4
TARGET = mlx-example

# C++17 — the header uses C++17 `inline static` for header-only statics.
Build_LanguageStandard_GXX = gnu++17

# Locate uMaker + AVR++ relative to this Makefile (we're at examples/).
uMakerPath = ../uMaker/
AVRpp_BaseDir = ../AVR++

# AVR++ source files needed by this example: just gccGuard for the C++
# runtime stubs (the rest of AVR++ we use is header-only — IOpin, SPI, etc).
AVRpp_SRC = gccGuard

# Include the example's source folder (we're using main.cpp in this dir) and
# the MLX90363 header at the repo root.
Source_Path = ./
INCLUDES = .. ./

all: build

include $(uMakerPath)tools/paths.mk
include $(uMakerPath)tools/source.mk
include $(uMakerPath)tools/checks.mk
include $(uMakerPath)vars/AVR.mk
include $(uMakerPath)tools/AVR/AVR++.mk
include $(uMakerPath)tools/build.mk
include $(uMakerPath)tools/mkdir.mk

.PHONY: all
