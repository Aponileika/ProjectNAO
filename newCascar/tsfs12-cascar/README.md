# Control Autonomous Systems CAR - Cascar

Cascar is the ROS 2 platform for **Learning, Planning, and Control of Autonomous Vehicles (TSFS12)** at Linköping University. Use it to develop planning and control algorithms, test them in simulation, and run them on the physical cars.

## :books: Documentation

**[Read the Cascar documentation →](https://davax85.gitlab-pages.liu.se/tsfs12-cascar/)**

Start with the setup guide, then follow the workflow from simulation to the physical cars.

| Guide | What you’ll find |
| --- | --- |
| [Get started](https://davax85.gitlab-pages.liu.se/tsfs12-cascar/getting-started/) | Installation and workspace setup. |
| [Run Cascar](https://davax85.gitlab-pages.liu.se/tsfs12-cascar/running/) | Running the simulator, example controllers, and physical cars. |
| [Reference](https://davax85.gitlab-pages.liu.se/tsfs12-cascar/reference/) | Configuration, ROS interfaces, and package documentation. |
| [FAQ](https://davax85.gitlab-pages.liu.se/tsfs12-cascar/faq/) | Answers to common setup and development questions. |

The documentation is built with **Zensical** and published on **GitLab Pages**.

## :calendar: Update history

### 2026

* Publish the documentation with Zensical on GitLab Pages. Remove the old GitLab wiki.
* Rework of the project structure with the goal of making it easier to use and understand for students.
* Clean up the codebase and remove unnecessary files.
* Update how the RPI is configured (not relevant for students), which includes easier lidar usage if needed. The RPIs run a Docker image transferred locally from the laptop over SSH and start it automatically on boot.

For earlier updates, see the [full changelog](https://davax85.gitlab-pages.liu.se/tsfs12-cascar/changelog/).
