# Open Source RC Lost Model Alarm 
* Two modes -- Normal (manual switch) or Inactivity
* Modes programmable from TX. 
* Resonated PIEZO
* Loud!

Quick notes ...

The hardware design created using free, open source KiCAD EDA suite.

The electronics employs an ATmega88PA (5x5mm chip) to intelligently drive a piezo with autotransformer, at 2.8KHz resonant frequency, for super loud output -- something in excess of 100dB, I suspect.

The piezo and autotransformer were sourced from a cheap "personal alarm". The latter is particularly difficult to bug anywhere else that I can find, though one could wind their own on a suitable former. The piezo has a low profile, aluminium sheet metal resonance cavity permanently attached to it, making for a small overall package.

The circuit board is 16x33mm and the piezo is 28mm diameter and 3mm thick. The autotransformer (11mm height) can be mounted either on the top or bottom of the PCB.

There are three status LEDs.

Programming and status tones are emitted (at lower, close range tolerable levels!) using Morse code. Mostly just single letters. Great for people who know Morse code. Still better than counting beeps for those who don't, once you learn just a few letters and a question mark and how they sound. Easy peasy. ;-)

The alarm can work in either Normal or Inactivity mode. In Inactivity mode, the alarm will sound if there has been no activity (for a minute or two) on the RC channel the module is connected to. Useful for cases where a dedicate RC channel is not available. Just connect to aileron or elevator channel.

## KiCAD v4 PCB Render
<img src="img/kicad_render.png" width="320">

Autotransformer and off-board quartz piezo not shown.
