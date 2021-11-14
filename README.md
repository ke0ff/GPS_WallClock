# GPS_WallClock
LED wall clock that accepts GPS input data from a Trimble timing GPS and displays local/GPS time.  See: https://ke0ff.github.io/GPSDWC.pdf
for hardware details and software description.

Revnotes in main.c

To compile:
1) Open CCS6 and create a new (empty or with main.c) project for the Tiva TM4C123GH6PM (close all the project files)
2) Copy all files into the new folder (including the "inc" folder).  Replace all duplicate files.

The project should then compile without error as all files needed are included in the repo.  Newer versions of CCS my be used, but the project creation process may be different.
