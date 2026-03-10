#ifndef POV_DISPLAY_H
#define POV_DISPLAY_H

#include <Arduino.h>
#include "commands.h"

// Initialize POV display system
void initPOV();
void initPOVTask(); // Call after initPOV — starts the render task

// Called from encoder ISR on each transition
void IRAM_ATTR povEncoderUpdate();

// Called from encoder ISR on each transition — updates pixel buffer only (no strip.show)
void IRAM_ATTR displayPOVColumn();

// Called from main loop — calls strip.show() if buffer was updated
void povShowTask();

// POV control functions
void setPOVEnable(bool enable);
bool getPOVEnable();
void selectAnimation(uint8_t animationId);
void setFrameTiming(uint16_t revolutionsPerFrame);
uint32_t getRevolutionCount();
void resetRevolutionCount();
uint16_t getCurrentFrame();

// Animation management
bool startAnimationUpload(uint8_t animationId, uint16_t totalFrames);
bool uploadFrameData(uint16_t frameNum, uint8_t column, const uint8_t* rgbData);
bool endAnimationUpload();

// Pre-compiled animations
void loadTestAnimation();       // Animation 0: Rainbow disc
void loadSpokesAnimation();     // Animation 1: White spokes
void loadBullseyeAnimation();   // Animation 2: Concentric rings
void loadSpiralAnimation();     // Animation 3: Color spiral
void loadClockAnimation();      // Animation 4: Clock face (static placeholder)
void updateClockAnimation(uint8_t hour12, uint8_t minute, uint8_t second);

// External access to POV state
extern POVState povState;

#endif // POV_DISPLAY_H
