## turtlebot3 Setup

```bash
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
```
```bash
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git

```
```bash
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

<b>Model config</b><br>
There are two kinds of model for Turtlebot3. If you don’t choose one of them, the program will not run. For general purpose, we choose to use “burger” model.

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```