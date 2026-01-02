#pragma once
#include <Arduino.h>

// Per-component demo entry points. Call one pair from setup()/loop() to run that demo.
void motorsDemoSetup();
void motorsDemoLoop();

void ultrasonicDemoSetup();
void ultrasonicDemoLoop();

void grayscaleDemoSetup();
void grayscaleDemoLoop();

void rgbDemoSetup();
void rgbDemoLoop();

void irDemoSetup();
void irDemoLoop();
