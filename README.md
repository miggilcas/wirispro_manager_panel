# Wiris Pro Manager RViz Panel

<!-- TABLE OF CONTENTS 
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>
-->

## Overview

This package provides a custom RViz panel for managing the Workswell Wiris Pro camera in the context of SIMAR project. The panel allows users to make a simple use of the camera features, such as start/stop recording video, capture images, enable the ethernet video Stream, Zoom Adjustement, etc; using as a driver package in ROS as a backend. 



## Features

- **Video Start/Stop Controls:** Start Recording, Stop recording
- **Zoom Adjustment:** Change the Zoom of the WirisPro Camera, from -10x to 10x.
- **Information Display:**
  - XX
  - XX

## Dependencies installation

---

### Build essentials and cmake
```bash
sudo apt-get install build-essential cmake
```

### Qt5
```bash
sudo apt-get install qt5-default
```

### ROS (Robot Operative System)

Follow up your ROS distro installation guide: [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) or [Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)



## Usage

1. Clone this repository into your ROS workspace and compile it using cmake:
    ```bash
    mkdir -p your_ros_ws/src && your_ros_ws/src && git clone https://github.com/...

    cd .. && catkin_make
    ```

1. Source the workspace and launch RViz:

    ```bash
    source devel/setup.zsh && rviz

    # just in case you don't know about zsh and you use bash :(
    source devel/setup.zsh && rviz
    ```

2. Add the custom panel to your RViz layout:

    - Click on the "Panels" tab in RViz.
    - Select "Add New Panel" and choose it from the list.

...

Define more steps to follow

## Help / Contribution

* Contact: **Miguel Gil Castilla** (mgil8@us.es)


* Found a bug? Create an ISSUE!

* Do you want to contribute? Create a PULL-REQUEST!

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

Finally I want to thank the following packages or URLs that inspired us to create the plugin.

* [Choose an Open Source License](https://choosealicense.com)
* [GitHub Emoji Cheat Sheet](https://www.webpagefx.com/tools/emoji-cheat-sheet)
* [Malven's Flexbox Cheatsheet](https://flexbox.malven.co/)
* [Malven's Grid Cheatsheet](https://grid.malven.co/)
* [Img Shields](https://shields.io)
* [GitHub Pages](https://pages.github.com)
* [Font Awesome](https://fontawesome.com)
* [React Icons](https://react-icons.github.io/react-icons/search)

---
---

