# ProjectButton
Button Development Scripts.
Only the scripts! 

# Running the Code
Install the required libs if not installed. Try to install it within a python environement.

### Automate running the scripts in the background.
While there are N number of ways to do this, what worked in our case is running the command in the backend in a .bashrc script. 

### Edit the .bashrc file and write the following command 
#### "sudo python3 Button.py"

#### Note - Change the path given to the variable name mission_dir. found here : https://github.com/RedWvn/ProjectButton/blob/main/Button.py#L69 to a desired mission folder file where all the return leg missions are stored.

# Connection Diagram

#### Below is the Block Diagram between AutoPilot and the RasPI : https://whimsical.com/button-connection-diagram-HdRsdC3J8zB2EeQNHnM4aq

![image](https://github.com/RedWvn/ProjectButton/assets/107253723/77b6f911-43b6-4c25-8969-8ed6031e81ae)



#### Below is the Block Diagram for the SITL 

![image](https://github.com/RedWvn/ProjectButton/assets/107253723/46f1dbee-cc77-4ea2-ab07-ab169001011b)


# Read the Docs 
1. All the Flow chart, code structure and algorithmic flow can be found here : https://whimsical.com/project-button-VbkkievEqcbPYSst83mRjL
2. The step by step process of the excecution can be found here : https://docs.google.com/document/d/1XBgoppZF0EFci7simicCUdtDdfHlXCPFS23KDnjdrUI/edit#heading=h.79eomy8q7b8z
 
## Required Libs:
1. pymavlink
2. neopixel 
3. RPi.GPIO
