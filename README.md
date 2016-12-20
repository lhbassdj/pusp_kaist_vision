# PUSP_KAIST_vision

Welcome!

This is Dongjin Kim, xs3d laboratory member in KAIST IE, Republic of Korea. My research topic is vision-processing and system optimization.

PUSP stands Persistant UAV Security Companion in KAIST, Republic of Korea. UAV swarm can be used widely such as border patrol, rescue work, or security companion for returning home safe.

This package contains tum_ardrone ROS package and tracking node. In version 3.0, tracking node supports automatic landing and manual detecting.

## Installation

Install tum_ardrone in your ROS environment, add src/tracking folder, and replace CMakeLists txt file. 

## Quick start

#### Launch the nodes

``` bash
roslaunch tum_ardrone ardrone_driver.launch
roslaunch tum_ardrone tum_ardrone_whole.launch
```

## Added node from tum_ardrone

### drone_tracking

Convert ardrone's raw image to OpenCV matrix format and process with it(Enabling detection).

#### Subscribed topics

- /ardrone/navdata
- /ardrone/image_raw
- /ardrone/land
- /cmd_vel
- /tum_ardrone/com

#### Published topics

- /ardrone/predictedPose
- /tum_ardrone/com

#### Services

None

#### Parameters

- ~publishFreq: frequency, at which the drone's estimated position is calculated & published. Default: 30Hz
- ~calibFile: camera calibration file. If not set, the defaults are used (camcalib/ardroneX_default.txt).
- UseControlGains: whether to use control gains for EKF prediction.
- UsePTAM: whether to use PTAM pose estimates as EKF update
- UseNavdata: whether to use Navdata information for EKF update
> UsePTAM and UseNavdata are set to false, the EKF is never updated and acts as a pure simulator, prediciting the pose based on the control commands received (on /cmd_vel). Nice for experimenting.

- PTAMMapLock: lock PTAM map (no more KF)
- PTAMSyncLock: lock PTAM map sync (fix scale and pose offsets etc.)
- PTAMMaxKF: maximum amount of KF PTAM takes. 

- PTAMMinKFDist: min. distance between two KF (in meters)
- PTAMMinKFWiggleDist: min. distance between two KF (relative to mean scene depth).
- PTAMMinKFTimeDiff: min time between two KF.
> PTAM takes a new KF if (PTAMMinKFTimeDiff AND (PTAMMinKFDist OR PTAMMinKFWiggleDist)), and tracking is good etc.

- RescaleFixOrigin: If the scale of the Map is reestimated, only one point in the mapping PTAM <-> World remains fixed.
	If RescaleFixOrigin == false, this is the current pos. of the drone (to avoid sudden, large "jumps"). this however makes the map "drift".
	If RescaleFixOrigin == true, by default this is the initialization point where the second KF has been taken (drone pos. may jump suddenly, but map remains fixed.). The fixpoint may be set by the command "lockScaleFP".
                  
- c1 ... c8: prediction model parameters of the EKF. see "Camera-Based Navigation of a Low-Cost Quadrocopter"

#### Required tf transforms

TODO

#### Provided tf transforms

TODO

#### Using it

To properly estimate PTAM's scale, it is best to fly up and down a little bit (e.g. 1m up and 1m down) immediately after initialization.
There are two windows, one shows the video and PTAM's map points, the other one the map. To issue key commands, focus the respective window and hit a key. This generates a command on /tum_ardrone/com, which in turn is parsed and does something.

###### Video Window

![Video window](http://wiki.ros.org/tum_ardrone/drone_stateestimation?action=AttachFile&do=get&target=video.png)

| Key   | /tum_adrone/com message | Action  |
|-------|-------------------------|---------|
| r     | "p reset"               | resets PTAM |
| u     | "p toggleUI"            | changes view |
| space | "p space"               | takes first / second Keyframe for PTAM's initialization |
| k     | "p keyframe"            | forces PTAM to take a keyframe |
| l     | "toggleLog"             | starts / stops extensive logging of all kinds of values to a file |
| m     | "p toggleLockMap"       | locks map, equivalent to parameter PTAMMapLock |
| n     | "p toggleLockSync"      | locks sync, equivalent to parameter PTAMSyncLock |

Clicking on the video window will generate waypoints, which are sent to drone_autopilot (if running):
- left-click: fly (x,y,0)m relative to the current position. image-center is (0,0), borders are 2m respectively.
- right-click: fly (0,0,y)m and rotate yaw by x degree. image-center is (0,0), borders are 2m and 90 degree respectively.

###### Map Window

![Map window](http://wiki.ros.org/tum_ardrone/drone_stateestimation?action=AttachFile&do=get&target=map.png)

| Key   | /tum_adrone/com message | Action  |
|-------|-------------------------|---------|
| r     | "f reset"               | resets EKF and PTAM |
| u     | "m toggleUI"            | changes view |
| v     | "m resetView"           | resets viewpoint of viewer |
| l     | "toggleLog"             | starts / stops extensive logging of all kinds of values to a file |
| v     | "m clearTrail"          | clears green drone-trail |


### Rest of Nodes

There are drone_stateestimation, drone_gui, and drone_autopilot nodes. Those are described on tum_ardrone homepage. [tum_ardrone package homepage](https://github.com/tum-vision/tum_ardrone)


## Licence

The major part of this software package - that is everything except PTAM - is licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html. PTAM (comprised of all files in /src/stateestimation/PTAM) has it's own licence, see http://www.robots.ox.ac.uk/~gk/PTAM/download.html. This licence in particular prohibits commercial use of the software.

Tum_ardrone is licensed under GPLv3, Color filtering altorithm is licensed under MIT.
