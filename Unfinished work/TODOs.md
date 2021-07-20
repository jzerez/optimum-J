# TODOs

## Functionality
* Create value functions for turning measured characteristics to fitness
    * Gunna be a bit qualitative
    * Takes characteristics from
* Create handler for taking a tweaking the suspension
    * Able to move points fluently between different members
    * Able to calculate gradient quickly
* Rework rocker implementation
  * Nodes instead of points
  * No ARB
  * 2D coordinates

## Optimizing Runtime
* Change how the rack is swept back and forth
* Need to decrease runtime by a factor of 10 to be really useful, oof
* Examine rack sweeping code. Introduced largest performance drop
* solve for a-arm rotation angle direction beforehand to avoid having to recalculate rotation.
* probably get rid of quatternion rotations in rocker and a-arm in favor of simpler stuff (rotation constrained to a plane)

## Useful Features
* Saving barebones information about suspensions along the way to trace optimization path
    * Able to return to old local maxima
    * Save characteristics, hard points from the static position, and fitness
