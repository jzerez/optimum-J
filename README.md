# suspension-optimization
This is an independent project exploring the use of geometric modeling to try to optimize the suspension for an FSAE vehicle.

## General Framework
Optimizing a suspension geometry requires moving points around in space until they reach their optimal positions. In this project, the user can define regions in space that specific points can inhabit. These regions are determined by packaging or logistics concerns. The program then creates a suspension geometry base on initial points provided by the user and a set of baked in general rules and guidelines. Then, it sweeps the geometry through its range of motion, and measures specific parameters, firstly rules required specifications, then physical and geometric constraints, and then finally performance characteristics. If the design does not comply with geometric restrictions, the rules, or physical limitations, then the design is automatically disregarded.

## Details
### Geometric restrictions
The goal is to make a suspension that would make Claude Rouelle proud. That means that a certain set of geometric restrictions are imposed upon the system. Below, the major restrictions used are listed:
* **Inboard pickup points must terminate at nodes of the chassis:** This ensures that none of the chassis members are in bending and reduces compliance. Specifically, the axial projections of the linkages continue on to intersect nodes of the chassis
* **The pushrod, rocker, and other related members must be co-planar:** This ensures that none of these components (the pushrod, the rocker, the shock, the anti-roll-bar) will be in bending, reducing how heavy they need to be.
* **All points need to be within their bounding regions:** This ensures that points must be at least reasonably placed.

The code is implemented in such a way that any potential suspension geometry must comply to these geometric restrictions before any testing or optimizing can take place.

### Rules restrictions
The suspension needs to be rules compliant. Below are the rules that are checked for:
* A serious effort must be put into designing a suspension

Below are the rules that need to be checked in the future
* Vertical Wheel Travel (50mm min)
* Minimum turning radius (9m???)

### Physical restrictions
The suspension can't intersect with itself. We need to be able to physically build it. Below are the physical restrictions that are checked for:
* Linkages can't intersect with each other (toe link, pushrod, control arms)
* Other members can't interfere with anything else (half shaft, boot)
* The wheel can't interfere with suspension members in any position

### Performance Characteristics
If a suspension is deemed to be both rules compliant, and physically possible, then it is on to computing performance metrics. These can include
* Bump Steer
* Camber gain (in any scenario)
* Roll Center (against a user defined desired value)
* Total Wheel Scrub
* Scrub Radius
* Spindle Length
