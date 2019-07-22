<!-- markdownlint-disable MD012 MD014 MD026 -->

# Guide to dt-catkin-template

A template for a Docker container for a ROS Python Node that supplements the Duckietown ROS core.

This template can be used as a starting point when developing nodes to supplement the Duckietown ROS system leveraging the distributed capabilities of Docker and ROS.  The template comes with a basic node `joy_cli` that allows controlling the robot through the terminal.  Additionally, the template includes a basic node `vel_func_node` that allows controlling the robot velocity rhrough a pre-defined function.  Note that to function properly it requires that the `duckietown/rpi-duckiebot-joystick-demo:master18` and `duckietown/rpi-ros-kinetic-roscore:master18` containers are running on the Duckiebot.  


## What's in the template?

The template is set up to build as a `catkin` workspace.  That is why there's the `.catkin_workspace` file that signals this to `catkin_make`.  When you run `catkin_make` for the first time, it will build two more directories: `/devel` and `/build`.  When pushing the code to GitHub, sharing it with others or building a Docker image, the `/devel` and `/build` directories are not necessary.  Don't try to reuse them as they contain paths that are specific to the system where they were build.  To make sure that `git` doesn't track them, they are included in the `.gitignore` file.

This template does not contain any of the core Duckiebot functionality.  It assumes that you run the `duckietown/rpi-duckiebot-joystick-demo:master18` container as a ROS master and via it it can launch, connect and use core nodes, services, etc.  This keeps the add-on nodes separate from the Duckiebot core which in turn allows for easy portability and stacking of add-ons.

In the `/src` directory you will find three packages: `duckietown_msgs`, `joy_cli`, and `vel_func_node`. Typically, you won't have to edit the `CMakeLists.txt` file.  The `duckietown_msgs` containes the ROS message definitions needed to communicate with the other Duckiebot ROS nodes.  Therefore in almost all circumstances you will need to use it.  The `joy_cli` package contains an example of a very basic ROS node that allows controlling the robot through the terminal.  The `vel_func_node` package contains another basic node that allows controlling the robot via a pre-defined function in the Python script within.  You can see how these are implemented and use them as a basis for your nodes.  Once the workspace is built by executing `catkin_make` and the environemnt is set (more on this later), the `joy_cli` node can be started by running (substitute `duckiebot` for your bot's name):

```bash
$ roslaunch joy_cli joy_cli.launch veh:=duckiebot
```

Likewise, the `vel_func_node` node can be started by running (substitute `duckiebot` for your bot's name):

```bash
$ roslaunch joy_cli joy_cli.launch veh:=duckiebot
```


## Getting Started

Here are the first steps you should take for making your own node.

1. Set up an empty Github repository.
2. Clone the contents of this repository into your repository.  You can do that by runnning this sequence of commands:

   ```bash
   $ git clone --bare https://github.com/doctorwho55/dt-catkin-template.git
   $ cd dt-catkin-template.git
   $ git push --mirror https://github.com/yourusername/your-repository-name.git
   $ cd ..
   $ rm -rf dt-catkin-template.git
   ```

3. Build your directory with `catkin` for the first time.  Navigate to your desired `catkin` directory and then run:

    ```bash
    $ catkin_make
    ```


## Using Git

Git is a useful tool for version control in development.  

### Creating a working branch

See <https://docs.duckietown.org/DT19/software_devel/out/github_basics.html>.

TODO: Directions to create working branch.

### Keeping your local branch updated

See <https://docs.duckietown.org/DT19/software_devel/out/github_basics.html>.

TODO: Directions to fetch/pull changes from remote repository.

### Commiting changes

The status of changes in your git branch can be checked as follows:

```bash
git status
```

Changes listed here can then be staged for commit. Individual files can be staged by running (replace `filename` with the desired file or files seperated by a space):

```bash
git add filename
```

Alternatively, all changed files can be staged:

```bash
git add --all
```

Once satisfied with the changes made, they can be committed to the working branch of the git repository with a message.

```bash
git commit -m "message"
```

These commits can then be pushed to the working branch of the remote repository (replace `working-branch` with the working branch name):

```bash
git push origin working-branch
```

### Merging to master branch

Once work on a working branch is complete, the changes from that branch can be merged to the master branch (replace `master-branch` with the master branch name).  From the working branch, run:

```bash
git push origin master-branch
```


## Developing and Testing

It is generally easier to develop your code and debug it if you are working on your computer and not on the Duckiebot.  This can be easily done thanks to the distributed nature of ROS.  You will still need your Duckiebot to be on and to have the `duckietown/rpi-duckiebot-joystick-demo:master18` container running.  Then, you can setup your laptop ROS environment such that it uses the Duckiebot ROS master.  Here are the steps to do that (assuming you have ROS Kinetic already).

1. Run `catkin_make` in your node folder if you haven't already.
2. Open a terminal in your node folder (should conatain `devel`, `build` and `src` folders) and run `source devel/setup.bash`. That will setup all the ROS functionality you need in your terminal.
3. Run `export ROS_MASTER_URI=http://duckiebot:11311`, substituting `duckiebot` with your bot's name. This will instruct ROS not to establish a local master but to instead connect to your Duckiebot's master.
4. Use the `export` command to set up any other environemnt variables that your nodes require.
5. Now you can launch your nodes with `roslaunch`, `rosrun`, etc (see the example above).
6. Don't forget to commit your changes to `git` from time to time!


## Deploying Code Using Docker

So far your new nodes were running on your computer. As you can imagine, while easy for development, that is not a great solution for portability. Also, you might be unknowingly _cheating_ with the extra computational resources on your computer. The deployment standard in Duckietown is Docker containers. By building a Docker container from your nodes and testing it with, you can be (almost) sure that your work is truly portable, well-functioning and will be usable by others easily. So how do you do that?

1. Take a look at the `Dockerfile` in your main directory. It was setup for the `vel_func_node` example. Update your maintainer label, add explanations on any other environment variables or special options that need to be passed when your container is run, and add commands to install any additional packages your nodes might be needing. Update which node gets launch by default in the `node_launch.sh` file. You can always override that by passing a different command when executing `docker run`. For example you might want a simple `bash` terminal to explore the contents of your container.

    Revise the `Dockerfile` as needed.  Be sure to mark any `*.sh` file as executable by including this command within the `RUN [ “cross-build*” ]` lines (replace `filepath` with the desired filepath):

    ```docker
    RUN /bin/bash -c “chmod +x filepath.sh”
    ```

2. From within the catkin workspace directory, build the docker image (replace `dockerhub_username`, `repository`, and `tag` with the respective Dockerhub username, repository, and tag):

    ```bash
    $ docker build -t dockerhub_username/repository:tag .
    ```

3. Push the newly built image to Dockerhub using the same information as used to build:

    ```bash
    $ docker push dockerhub_username/repository:tag
    ```

4. When your image is already on Dockerhub, you can pull it directly on your Duckiebot over an internet connection.  To pull your image to the robot, run (replace `hostname` with the Duckiebot hostname):

    ```bash
    $ docker -H hostname.local pull dockerhub_username/repository:tag
    ```

    This same procedure can be used to update the image on the Duckiebot after changes are made to the remote repository.

5. Once the image is pulled to the duckiebot, it can be run from the host computer:

    ```bash
    $ docker -H hostname.local run -it --network host -e ROS_MASTER_URI='http://hostname:11311' -e DUCKIEBOT_NAME='hostname' dockerhub_username/repository:tag
    ```

   Remember to add any additional environment variables or options you might need. If you want to execute a command different from the one set in `node_launch.sh`, you can add it at the end of the command. If your conatiner doesn't need an interactive terminal you can omit the `-it` options. Bear in mind that if your node tries to start a graphical interface, you will need to pass a few extra arguments to the command.

On the duckiebot, you can find your workspace in `/node-ws` in the running container. It is important to never rename this folder as that will destroy a number of links.


<!-- markdownlint-restore -->