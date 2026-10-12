# ME4242 Soft Robotics

## Cable-driven soft robot control

<!-- Highlight the summary / overview of the lab -->
::: highlight
##### Overview
Control a cable-driven soft robot with the 4 motors of Emio, either from a SOFA simulation or directly on the hardware.
:::

<!--Direct hardware control-->
::: highlight
#icon("warning") **Warning:**

The tools below move real motors and pull real cables. Install the `emioapi` package first, keep the robot free of obstacles, and do not run them at the same time as a SOFA simulation connected to the robot.
:::

::: exercise
**Direct cable control:**

1. Install the Python packages of this lab (once):

#python-button(pyargs=["-m", "pip", "install", "--target", "assets/labs/lab_me4242/modules/site-packages", "-r", "assets/labs/lab_me4242/requirements.txt"])

2. Open the graphical controller (sliders, D-pad and automated motion tests). It falls back to a simulation mode if no robot is detected:

#python-button(file="assets/labs/lab_me4242/cabledriven_gui.py", extrapythonpath=["assets/labs/lab_me4242/modules/site-packages", "assets/labs/lab_me4242"])
:::

<!--Highlight warnings -->
::: highlight
#icon("warning") **Warning:**

Use the `highlight` container to display warning messages. 
:::

<!--Highlight information -->
::: highlight
#icon("info") **Info:**

Also use the `highlight` container to display any information you want to highlight. 
:::

<!--Add sections -->
#include(assets/labs/lab_me4242/sections/section1.md)
#include(assets/labs/lab_me4242/sections/section2.md)
#include(assets/labs/lab_me4242/sections/section3.md)

<!--Python Packages-->
:::: collapse Install Additional Python Packages
#include(assets/labs/lab_me4242/modules/site-packages/README.md)
::::

#include(assets/labs/lab_me4242/sections/authors.md)



