In our localization experiments, we use light signal to obtain localization result of robots.

After 10 iterations, robots will do the following things sequencially:

1. Flash both LEDs three times.
2. Lit up green LED.
3. Turn off green LED after 10+(X localization result) seconds, note that X localization result needs to be scaled properly to represent actual physical coordinate of robot.
4. Flash both LEDs two times.
5. Lit up blue LED.
6. Turn off blue LED after 10+(y coordinate) seconds.

Then we use a camera to record the flashing process (for example see "example raw video_20.mp4" in this folder) and then extract the light-on time. Data in 10th, 100th, 200th and 300th iterations are given in four .xlsx files in this folder. In each .xlsx file, two tabs records X and Y localization result respectively, and in each tab the the time when lights are lit up is given in the sixth column, the time when light is off is given in the first column, and the corresponding robot position in pixels are given in the second and third columns.