# System Description
# Main functions:
1. Receive energy from a solar panel at a voltage of about 30V and transfer it to a battery at about 12V. 
2. Supply power from the battery to a load consiston of LED floodlights. The LED floodlights should come on when it gets dark in the evening, and turn off either when the next day starts, or if the battery reaches a threshold voltage indicating it has been discharged sufficiently so that further discharge may cause damage. 
3. Switch on a mains supply input at a time close to the end of the day if the battery has not been sufficiently charged during the day as indicated by the charge voltage not reaching a threshold (mains connect threshold). Keep the mains supply connected until the battery reaches a mains disconnect threshold.
4. Perform management of the battery so that it can reach its maximum useful life. This involves: a) boost charge of up to 1 hour per day; b) float charge for the remainder of the day; c) load disconnect threshold to stop further discharge; d) load reconnect threshold to allow reconnection of the load; e) adjustment of voltage thresholds according to the battery temperature.
5. Self protection against transient voltage surges from the solar panel input.
6. Overload protection against excessive temperature.

# Peripheral functions:
7. LCD display of operating values and status (voltages, currents, connections).
8. Data logging of operating values and status to an SD card, with predefined file structure (day files or week files or month files or several?)
9. Real time clock to provide a time stamp to logged data.
