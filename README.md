## BirdsEye
No longer top sekret, but I have not written a readme yet, good luck

### Features
- 25hz logging straight to SD card
- track selection
- course selection
- speed
- current lap
- best lap
- pace off best lap (current session)
- lap history (current session)

### System
- MCU: nrf52840 (xiao seeed) (64mhz + FPU + bluetooth)
- GPS: UBLOX sam-m10q (matek drone GPS, they always use official, and its SMOL)
- SD Card board thinggy, the 10cent ones
- 2.42 inch 128x64 OLED LCD
- 1500mAh 103050 Lipo
	- 13~ hours constant on full charge

### Readmes
- datalogger: readme coming
	- It's kind of easy to add custom pages if you just look at the others
- tachometer readme coming soon
- 3dp Box: readme coming soon
	- It's a rough fit if you just wing it, go slow
	- Yes, glue required, and a couple jewelers screws

#### Bonus
I think this broke, but its a circuit sim thingy, that was working at some point
[https://wokwi.com/projects/367285364247620609](https://wokwi.com/projects/367285364247620609)

---

DataViewer: [https://github.com/TheAngryRaven/DovesDataViewer](DovesDataViewer)
  -- Preview: [HackTheTrack.net](HackTheTrack.net)
Core GPS Timing library: [https://github.com/TheAngryRaven/DovesLapTimer](DovesLapTimer)