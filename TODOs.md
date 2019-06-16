# TODOs

## Functionality
* Create value functions for turning measured characteristics to fitness
    * Gunna be a bit qualitative
    * Takes characteristics from 
* Create handler for taking a tweaking the suspension
    * Able to move points fluently between different members
    * Able to calculate gradient quickly
    
## Optimizing Runtime
* Change how the rack is swept back and forth
* Need to decrease runtime by a factor of 10 to be really useful, oof
* Examine rack sweeping code. Introduced largest performance drop

## Useful Features
* Saving barebones information about suspensions along the way to trace optimization path
    * Able to return to old local maxima
    * Save characteristics, hard points from the static position, and fitness
