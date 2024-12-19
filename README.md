Pantograph Robot: Autonomous Drawing System
===========================================

Overview
--------

This project presents the design and implementation of a pantograph-inspired robot for precision writing and drawing tasks. The robot combines modular mechanical components, advanced control algorithms, and efficient navigation strategies. The repository includes design documentation and the source code for the robot.

* * * * *

1\. Robot Design
----------------

### Design Choices

-   **Pantograph Structure:**

    -   The robot's linkage system is based on a pantograph mechanism, ensuring precise scaling and replication of movements.
    -   Utilizes five modular links connected by pivot points for movement in a two-dimensional plane.
-   **Mechanical Features:**

    -   **Pen Holder:** Includes a stable, vertically mounted end-effector for accurate writing.
    -   **Gear Mechanism:** Gears reduce backlash and improve accuracy of motorized movements.
-   **Motorized Control:**

    -   Powered by concealed motors driving the linkage system.
    -   Central programmable hub for controlling motor movements and processing inputs.
-   **Modular and Stable Design:**

    -   Easily assembled modular components (e.g., LEGO-compatible).
    -   Rigid base and level drawing surface for minimized vibrations.

* * * * *

2\. Navigation and Motion Planning
----------------------------------

### Navigation Strategy

-   **Waypoint Interpolation:**

    -   Calculates intermediate points between target positions using a specified step size for smooth transitions.
    -   Ensures continuous and precise movement along defined trajectories.
-   **Collision Avoidance:**

    -   Uses sensors for detecting obstructions, recalibrating motion paths dynamically.

* * * * *

3\. Calibration and Control
---------------------------

### Calibration

-   **Motor Calibration:**

    -   Utilizes proportional-integral (PI) control loops for accurate motor angle adjustments.
    -   Functions like `TurnMotorForAngle` and `TurnMotorsForAngle` enable precise position control.
-   **Gear Ratio:**

    -   Predefined gear ratios improve accuracy and ensure consistent movements.

### Control Mechanisms

-   **Inverse Kinematics:**
    -   Computes joint angles for desired end-effector positions using trigonometric equations like the cosine rule.
    -   Avoids linkage crossing by selecting appropriate solutions for joint configurations.

* * * * *

4\. Code Overview
-----------------

### Core Files

-   **`Picasso.py`**

    -   Implements the inverse kinematics, interpolation, and plotting functionality.
    -   Includes helper functions for motor control using PI loops.
-   **`Roboplotter.py`**

    -   Demonstrates the drawing capabilities by plotting predefined shapes and paths.

### Key Functions

#### Inverse Kinematics

```
class Picasso:
    def inverseKinematics(self, xd, yd):
        # Calculate distances and joint angles
        ...

```

#### Interpolation

```
def InterpolateMoves(self, points, stepsizecm=0.1):
    # Generate intermediate waypoints for smooth motion
    ...

```

#### Motor Control

```
def TurnMotorForAngle(motor, angle, threshold=1, Kp=3, Ki=1/1000):
    # PI control loop for motor angle adjustment
    ...

```

* * * * *

5\. Example Usage
-----------------

### Predefined Path

-   A sample path for drawing a triangle:

```
points = [
    [0, 0, False],
    [0, 0, True],
    [4, 0, True],
    [6, 4, True],
    [2, 4, True],
    [0, 0, True]
]

```

-   Interpolate and plot:

```
path_plan = roboClient.InterpolateMoves(points=points, stepsizecm=0.5)
roboClient.plot(path_plan)

```

* * * * *

6\. Running the Project
-----------------------

1.  Connect the robot hardware.
2.  Install dependencies.
3.  Run the Python script:

    ```
    python3 Roboplotter.py

    ```

* * * * *

7\. Future Work
---------------

-   **Enhanced Accuracy:** Implement advanced PID control for smoother operations.
-   **Dynamic Drawing Inputs:** Allow real-time drawing patterns via external inputs.
-   **Extended Capabilities:** Add features like obstacle avoidance and dynamic scaling.

* * * * *

8\. License
-----------

This project is licensed under the MIT License.

* * * * *

9\. Contact
-----------

For inquiries or contributions, email [<your_email@example.com>].
