# mc_aliengo
mc-rtc Robot Module package for the Unitree Aliengo robot.

This is a robot module that allows you to use `Aliengo` prototype within `mc_rtc`.  

### Dependencies:
- [mc_rtc](https://gite.lirmm.fr/multi-contact/mc_rtc)
- [aliengo_description](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/aliengo_description)

Make sure that the description package has been built, then:
```bash
mkdir build
cd build
cmake ..
make
(sudo) make install
```

Then, use the "aliengo" key in your controller's configuration file `~/.config/mc_rtc/mc_rtc.conf` to use Aliengo.
