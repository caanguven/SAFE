import re
import matplotlib.pyplot as plt

# Sample log data
log_data = """
rol Signal: 59.54%, Set Position: 4.49°
[ADCReader] Channel 0 ADC Value: 356, Average: 95.87 degrees
[Motor 1] Mapped ADC Average to 95.87 degrees
[Motor 1] Sawtooth Wave Set Position: 7.52°
[Motor 1] Calculated Error: -88.35° (Set Position: 7.52°, Current Angle: 95.87°)
[PIDController] Error: -88.35, Integral: -601.56, Derivative: -122.11, Control Signal: -41.48
[Motor] Set speed to 41.484084014703015% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 95.87°, Error: -88.35°, Control Signal: 41.48%, Set Position: 7.52°
[ADCReader] Channel 2 ADC Value: 472, Average: 173.98 degrees
[Motor 3] Mapped ADC Average to 173.98 degrees
[Motor 3] Sawtooth Wave Set Position: 7.85°
[Motor 3] Calculated Error: 163.87° (Set Position: 7.85°, Current Angle: 173.98°)
[PIDController] Error: 163.87, Integral: 744.84, Derivative: 278.33, Control Signal: 60.99
[Motor] Set speed to 60.990741454498036% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 173.98°, Error: 163.87°, Control Signal: 60.99%, Set Position: 7.85°
[ADCReader] Channel 0 ADC Value: 386, Average: 105.36 degrees
[Motor 1] Mapped ADC Average to 105.36 degrees
[Motor 1] Sawtooth Wave Set Position: 10.89°
[Motor 1] Calculated Error: -94.47° (Set Position: 10.89°, Current Angle: 105.36°)
[PIDController] Error: -94.47, Integral: -606.38, Derivative: -119.86, Control Signal: -41.98
[Motor] Set speed to 41.980219130711774% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 105.36°, Error: -94.47°, Control Signal: 41.98%, Set Position: 10.89°
[ADCReader] Channel 2 ADC Value: 439, Average: 162.91 degrees
[Motor 3] Mapped ADC Average to 162.91 degrees
[Motor 3] Sawtooth Wave Set Position: 11.22°
[Motor 3] Calculated Error: -151.69° (Set Position: 11.22°, Current Angle: 162.91°)
[PIDController] Error: -151.69, Integral: 737.04, Derivative: -6141.30, Control Signal: -100.00
[Motor] Set speed to 50.990741454498036% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 162.91°, Error: -151.69°, Control Signal: 50.99%, Set Position: 11.22°
[ADCReader] Channel 0 ADC Value: 415, Average: 114.90 degrees
[Motor 1] Mapped ADC Average to 114.90 degrees
[Motor 1] Sawtooth Wave Set Position: 14.26°
[Motor 1] Calculated Error: -100.65° (Set Position: 14.26°, Current Angle: 114.90°)
[PIDController] Error: -100.65, Integral: -611.52, Derivative: -121.09, Control Signal: -42.67
[Motor] Set speed to 42.66960876357354% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 114.90°, Error: -100.65°, Control Signal: 42.67%, Set Position: 14.26°
[ADCReader] Channel 2 ADC Value: 403, Average: 152.08 degrees
[Motor 3] Mapped ADC Average to 152.08 degrees
[Motor 3] Sawtooth Wave Set Position: 14.65°
[Motor 3] Calculated Error: -137.43° (Set Position: 14.65°, Current Angle: 152.08°)
[PIDController] Error: -137.43, Integral: 730.00, Derivative: 278.17, Control Signal: 42.16
[Motor] Set speed to 42.163129143106865% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 152.08°, Error: -137.43°, Control Signal: 42.16%, Set Position: 14.65°
[ADCReader] Channel 0 ADC Value: 445, Average: 124.45 degrees
[Motor 1] Mapped ADC Average to 124.45 degrees
[Motor 1] Sawtooth Wave Set Position: 17.62°
[Motor 1] Calculated Error: -106.83° (Set Position: 17.62°, Current Angle: 124.45°)
[PIDController] Error: -106.83, Integral: -616.97, Derivative: -121.10, Control Signal: -43.31
[Motor] Set speed to 43.31323432681278% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 124.45°, Error: -106.83°, Control Signal: 43.31%, Set Position: 17.62°
[ADCReader] Channel 2 ADC Value: 371, Average: 141.42 degrees
[Motor 3] Mapped ADC Average to 141.42 degrees
[Motor 3] Sawtooth Wave Set Position: 18.02°
[Motor 3] Calculated Error: -123.40° (Set Position: 18.02°, Current Angle: 141.42°)
[PIDController] Error: -123.40, Integral: 723.70, Derivative: 274.61, Control Signal: 42.51
[Motor] Set speed to 42.511487686193604% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 141.42°, Error: -123.40°, Control Signal: 42.51%, Set Position: 18.02°
[ADCReader] Channel 0 ADC Value: 474, Average: 133.94 degrees
[Motor 1] Mapped ADC Average to 133.94 degrees
[Motor 1] Sawtooth Wave Set Position: 20.99°
[Motor 1] Calculated Error: -112.95° (Set Position: 20.99°, Current Angle: 133.94°)
[PIDController] Error: -112.95, Integral: -622.74, Derivative: -119.79, Control Signal: -43.90
[Motor] Set speed to 43.903525176346655% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 133.94°, Error: -112.95°, Control Signal: 43.90%, Set Position: 20.99°
[ADCReader] Channel 2 ADC Value: 337, Average: 130.45 degrees
[Motor 3] Mapped ADC Average to 130.45 degrees
[Motor 3] Sawtooth Wave Set Position: 21.38°
[Motor 3] Calculated Error: -109.07° (Set Position: 21.38°, Current Angle: 130.45°)
[PIDController] Error: -109.07, Integral: 718.13, Derivative: 280.69, Control Signal: 43.40
[Motor] Set speed to 43.396959579906266% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 130.45°, Error: -109.07°, Control Signal: 43.40%, Set Position: 21.38°
[ADCReader] Channel 0 ADC Value: 502, Average: 143.36 degrees
[Motor 1] Mapped ADC Average to 143.36 degrees
[Motor 1] Sawtooth Wave Set Position: 24.35°
[Motor 1] Calculated Error: -119.00° (Set Position: 24.35°, Current Angle: 143.36°)
[PIDController] Error: -119.00, Integral: -628.82, Derivative: -118.63, Control Signal: -44.51
[Motor] Set speed to 44.512778866054816% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 143.36°, Error: -119.00°, Control Signal: 44.51%, Set Position: 24.35°
[ADCReader] Channel 2 ADC Value: 305, Average: 119.67 degrees
[Motor 3] Mapped ADC Average to 119.67 degrees
[Motor 3] Sawtooth Wave Set Position: 24.75°
[Motor 3] Calculated Error: -94.92° (Set Position: 24.75°, Current Angle: 119.67°)
[PIDController] Error: -94.92, Integral: 713.28, Derivative: 276.73, Control Signal: 43.81
[Motor] Set speed to 43.805144873983% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 119.67°, Error: -94.92°, Control Signal: 43.81%, Set Position: 24.75°
[ADCReader] Channel 0 ADC Value: 533, Average: 152.84 degrees
[Motor 1] Mapped ADC Average to 152.84 degrees
[Motor 1] Sawtooth Wave Set Position: 27.72°
[Motor 1] Calculated Error: -125.12° (Set Position: 27.72°, Current Angle: 152.84°)
[PIDController] Error: -125.12, Integral: -635.20, Derivative: -119.86, Control Signal: -45.26
[Motor] Set speed to 45.259861447777176% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 152.84°, Error: -125.12°, Control Signal: 45.26%, Set Position: 27.72°
[ADCReader] Channel 2 ADC Value: 270, Average: 108.78 degrees
[Motor 3] Mapped ADC Average to 108.78 degrees
[Motor 3] Sawtooth Wave Set Position: 28.12°
[Motor 3] Calculated Error: -80.66° (Set Position: 28.12°, Current Angle: 108.78°)
[PIDController] Error: -80.66, Integral: 709.13, Derivative: 277.59, Control Signal: 44.50
[Motor] Set speed to 44.49653404379759% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 108.78°, Error: -80.66°, Control Signal: 44.50%, Set Position: 28.12°
[ADCReader] Channel 0 ADC Value: 560, Average: 162.19 degrees
[Motor 1] Mapped ADC Average to 162.19 degrees
[Motor 1] Sawtooth Wave Set Position: 31.09°
[Motor 1] Calculated Error: -131.11° (Set Position: 31.09°, Current Angle: 162.19°)
[PIDController] Error: -131.11, Integral: -641.89, Derivative: -117.39, Control Signal: -45.83
[Motor] Set speed to 45.83041059543354% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 162.19°, Error: -131.11°, Control Signal: 45.83%, Set Position: 31.09°
[ADCReader] Channel 2 ADC Value: 235, Average: 97.94 degrees
[Motor 3] Mapped ADC Average to 97.94 degrees
[Motor 3] Sawtooth Wave Set Position: 31.55°
[Motor 3] Calculated Error: -66.39° (Set Position: 31.55°, Current Angle: 97.94°)
[PIDController] Error: -66.39, Integral: 705.74, Derivative: 279.41, Control Signal: 45.27
[Motor] Set speed to 45.274103952283426% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 97.94°, Error: -66.39°, Control Signal: 45.27%, Set Position: 31.55°
[ADCReader] Channel 0 ADC Value: 589, Average: 171.49 degrees
[Motor 1] Mapped ADC Average to 171.49 degrees
[Motor 1] Sawtooth Wave Set Position: 34.45°
[Motor 1] Calculated Error: -137.03° (Set Position: 34.45°, Current Angle: 171.49°)
[PIDController] Error: -137.03, Integral: -648.89, Derivative: -115.99, Control Signal: -46.47
[Motor] Set speed to 46.4659948195051% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 171.49°, Error: -137.03°, Control Signal: 46.47%, Set Position: 34.45°
[ADCReader] Channel 2 ADC Value: 200, Average: 86.91 degrees
[Motor 3] Mapped ADC Average to 86.91 degrees
[Motor 3] Sawtooth Wave Set Position: 34.91°
[Motor 3] Calculated Error: -52.00° (Set Position: 34.91°, Current Angle: 86.91°)
[PIDController] Error: -52.00, Integral: 703.08, Derivative: 281.39, Control Signal: 46.10
[Motor] Set speed to 46.103798089687146% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 86.91°, Error: -52.00°, Control Signal: 46.10%, Set Position: 34.91°
[ADCReader] Channel 0 ADC Value: 617, Average: 180.71 degrees
[Motor 1] Mapped ADC Average to 180.71 degrees
[Motor 1] Sawtooth Wave Set Position: 37.82°
[Motor 1] Calculated Error: -142.90° (Set Position: 37.82°, Current Angle: 180.71°)
[PIDController] Error: -142.90, Integral: -656.18, Derivative: -114.90, Control Signal: -47.13
[Motor] Set speed to 47.127627194092774% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 180.71°, Error: -142.90°, Control Signal: 47.13%, Set Position: 37.82°
[ADCReader] Channel 2 ADC Value: 167, Average: 75.93 degrees
[Motor 3] Mapped ADC Average to 75.93 degrees
[Motor 3] Sawtooth Wave Set Position: 38.28°
[Motor 3] Calculated Error: -37.65° (Set Position: 38.28°, Current Angle: 75.93°)
[PIDController] Error: -37.65, Integral: 701.16, Derivative: 281.28, Control Signal: 46.86
[Motor] Set speed to 46.862803243279856% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 75.93°, Error: -37.65°, Control Signal: 46.86%, Set Position: 38.28°
[ADCReader] Channel 0 ADC Value: 645, Average: 189.93 degrees
[Motor 1] Mapped ADC Average to 189.93 degrees
[Motor 1] Sawtooth Wave Set Position: 41.18°
[Motor 1] Calculated Error: -148.75° (Set Position: 41.18°, Current Angle: 189.93°)
[PIDController] Error: -148.75, Integral: -663.78, Derivative: -114.68, Control Signal: -47.85
[Motor] Set speed to 47.848104416599256% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 189.93°, Error: -148.75°, Control Signal: 47.85%, Set Position: 41.18°
[ADCReader] Channel 2 ADC Value: 134, Average: 64.90 degrees
[Motor 3] Mapped ADC Average to 64.90 degrees
[Motor 3] Sawtooth Wave Set Position: 41.65°
[Motor 3] Calculated Error: -23.25° (Set Position: 41.65°, Current Angle: 64.90°)
[PIDController] Error: -23.25, Integral: 699.98, Derivative: 282.32, Control Signal: 47.72
[Motor] Set speed to 47.71977931754381% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 64.90°, Error: -23.25°, Control Signal: 47.72%, Set Position: 41.65°
[ADCReader] Channel 0 ADC Value: 675, Average: 199.10 degrees
[Motor 1] Mapped ADC Average to 199.10 degrees
[Motor 1] Sawtooth Wave Set Position: 44.55°
[Motor 1] Calculated Error: -154.55° (Set Position: 44.55°, Current Angle: 199.10°)
[PIDController] Error: -154.55, Integral: -671.67, Derivative: -113.51, Control Signal: -48.53
[Motor] Set speed to 48.531516019664004% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 199.10°, Error: -154.55°, Control Signal: 48.53%, Set Position: 44.55°
[ADCReader] Channel 2 ADC Value: 101, Average: 54.00 degrees
[Motor 3] Mapped ADC Average to 54.00 degrees
[Motor 3] Sawtooth Wave Set Position: 45.01°
[Motor 3] Calculated Error: -8.98° (Set Position: 45.01°, Current Angle: 54.00°)
[PIDController] Error: -8.98, Integral: 699.52, Derivative: 279.51, Control Signal: 48.41
[Motor] Set speed to 48.41240341999984% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 54.00°, Error: -8.98°, Control Signal: 48.41%, Set Position: 45.01°
[ADCReader] Channel 0 ADC Value: 708, Average: 208.63 degrees
[Motor 1] Mapped ADC Average to 208.63 degrees
[Motor 1] Sawtooth Wave Set Position: 47.92°
[Motor 1] Calculated Error: -160.72° (Set Position: 47.92°, Current Angle: 208.63°)
[PIDController] Error: -160.72, Integral: -679.88, Derivative: -120.77, Control Signal: -49.68
[Motor] Set speed to 49.675701266904944% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 208.63°, Error: -160.72°, Control Signal: 49.68%, Set Position: 47.92°
[ADCReader] Channel 2 ADC Value: 69, Average: 43.29 degrees
[Motor 3] Mapped ADC Average to 43.29 degrees
[Motor 3] Sawtooth Wave Set Position: 48.38°
[Motor 3] Calculated Error: 5.09° (Set Position: 48.38°, Current Angle: 43.29°)
[PIDController] Error: 5.09, Integral: 699.78, Derivative: 275.67, Control Signal: 49.08
[Motor] Set speed to 49.077670338008375% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 43.29°, Error: 5.09°, Control Signal: 49.08%, Set Position: 48.38°
[ADCReader] Channel 0 ADC Value: 740, Average: 218.38 degrees
[Motor 1] Mapped ADC Average to 218.38 degrees
[Motor 1] Sawtooth Wave Set Position: 51.28°
[Motor 1] Calculated Error: 162.91° (Set Position: 51.28°, Current Angle: 218.38°)
[PIDController] Error: 162.91, Integral: -671.56, Derivative: 6339.06, Control Signal: 100.00
[Motor] Set speed to 39.675701266904944% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 218.38°, Error: 162.91°, Control Signal: 39.68%, Set Position: 51.28°
[ADCReader] Channel 2 ADC Value: 35, Average: 32.65 degrees
[Motor 3] Mapped ADC Average to 32.65 degrees
[Motor 3] Sawtooth Wave Set Position: 51.74°
[Motor 3] Calculated Error: 19.10° (Set Position: 51.74°, Current Angle: 32.65°)
[PIDController] Error: 19.10, Integral: 700.75, Derivative: 274.56, Control Signal: 49.91
[Motor] Set speed to 49.91131052361767% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 32.65°, Error: 19.10°, Control Signal: 49.91%, Set Position: 51.74°
[ADCReader] Channel 0 ADC Value: 769, Average: 228.20 degrees
[Motor 1] Mapped ADC Average to 228.20 degrees
[Motor 1] Sawtooth Wave Set Position: 54.71°
[Motor 1] Calculated Error: 156.52° (Set Position: 54.71°, Current Angle: 228.20°)
[PIDController] Error: 156.52, Integral: -663.57, Derivative: -125.02, Control Signal: -30.04
[Motor] Set speed to 30.038059591843915% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 228.20°, Error: 156.52°, Control Signal: 30.04%, Set Position: 54.71°
[ADCReader] Channel 2 ADC Value: 1, Average: 21.94 degrees
[Motor 3] Mapped ADC Average to 21.94 degrees
[Motor 3] Sawtooth Wave Set Position: 55.11°
[Motor 3] Calculated Error: 33.17° (Set Position: 55.11°, Current Angle: 21.94°)
[PIDController] Error: 33.17, Integral: 702.44, Derivative: 276.03, Control Signal: 50.91
[Motor] Set speed to 50.91397658047366% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 21.94°, Error: 33.17°, Control Signal: 50.91%, Set Position: 55.11°
[ADCReader] Channel 0 ADC Value: 798, Average: 238.08 degrees
[Motor 1] Mapped ADC Average to 238.08 degrees
[Motor 1] Sawtooth Wave Set Position: 58.08°
[Motor 1] Calculated Error: 150.00° (Set Position: 58.08°, Current Angle: 238.08°)
[PIDController] Error: 150.00, Integral: -655.91, Derivative: -127.69, Control Signal: -30.18
[Motor] Set speed to 30.17999552786362% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 238.08°, Error: 150.00°, Control Signal: 30.18%, Set Position: 58.08°
[ADCReader] Channel 2 ADC Value: 1, Average: 13.32 degrees
[Motor 3] Mapped ADC Average to 13.32 degrees
[Motor 3] Sawtooth Wave Set Position: 58.48°
[Motor 3] Calculated Error: 45.16° (Set Position: 58.48°, Current Angle: 13.32°)
[PIDController] Error: 45.16, Integral: 704.75, Derivative: 234.98, Control Signal: 49.70
[Motor] Set speed to 49.6954452939449% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 13.32°, Error: 45.16°, Control Signal: 49.70%, Set Position: 58.48°
[ADCReader] Channel 0 ADC Value: 825, Average: 247.75 degrees
[Motor 1] Mapped ADC Average to 247.75 degrees
[Motor 1] Sawtooth Wave Set Position: 61.45°
[Motor 1] Calculated Error: 143.70° (Set Position: 61.45°, Current Angle: 247.75°)
[PIDController] Error: 143.70, Integral: -648.58, Derivative: -123.67, Control Signal: -29.99
[Motor] Set speed to 29.991141239336045% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 247.75°, Error: 143.70°, Control Signal: 29.99%, Set Position: 61.45°
[ADCReader] Channel 2 ADC Value: 1020, Average: 0.86 degrees
[Motor 3] Mapped ADC Average to 0.86 degrees
[Motor 3] Sawtooth Wave Set Position: 61.84°
[Motor 3] Calculated Error: 60.98° (Set Position: 61.84°, Current Angle: 0.86°)
[PIDController] Error: 60.98, Integral: 707.86, Derivative: 310.11, Control Signal: 54.56
[Motor] Set speed to 54.557633445748216% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 0.86°, Error: 60.98°, Control Signal: 54.56%, Set Position: 61.84°
[ADCReader] Channel 0 ADC Value: 852, Average: 257.04 degrees
[Motor 1] Mapped ADC Average to 257.04 degrees
[Motor 1] Sawtooth Wave Set Position: 64.81°
[Motor 1] Calculated Error: 137.77° (Set Position: 64.81°, Current Angle: 257.04°)
[PIDController] Error: 137.77, Integral: -641.51, Derivative: -115.23, Control Signal: -29.57
[Motor] Set speed to 29.570104830935282% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 257.04°, Error: 137.77°, Control Signal: 29.57%, Set Position: 64.81°
[ADCReader] Channel 2 ADC Value: 1000, Average: 18.82 degrees
[Motor 3] Mapped ADC Average to 18.82 degrees
[Motor 3] Sawtooth Wave Set Position: 65.21°
[Motor 3] Calculated Error: 46.38° (Set Position: 65.21°, Current Angle: 18.82°)
[PIDController] Error: 46.38, Integral: 710.22, Derivative: -286.41, Control Signal: 23.97
[Motor] Set speed to 44.557633445748216% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 18.82°, Error: 46.38°, Control Signal: 44.56%, Set Position: 65.21°
[ADCReader] Channel 0 ADC Value: 880, Average: 266.07 degrees
[Motor 1] Mapped ADC Average to 266.07 degrees
[Motor 1] Sawtooth Wave Set Position: 68.18°
[Motor 1] Calculated Error: 132.11° (Set Position: 68.18°, Current Angle: 266.07°)
[PIDController] Error: 132.11, Integral: -634.75, Derivative: -110.81, Control Signal: -29.35
[Motor] Set speed to 29.35164420603447% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 266.07°, Error: 132.11°, Control Signal: 29.35%, Set Position: 68.18°
[ADCReader] Channel 2 ADC Value: 969, Average: 6.89 degrees
[Motor 3] Mapped ADC Average to 6.89 degrees
[Motor 3] Sawtooth Wave Set Position: 68.57°
[Motor 3] Calculated Error: 61.69° (Set Position: 68.57°, Current Angle: 6.89°)
[PIDController] Error: 61.69, Integral: 713.37, Derivative: 299.65, Control Signal: 54.35
[Motor] Set speed to 54.35232557640302% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 6.89°, Error: 61.69°, Control Signal: 54.35%, Set Position: 68.57°
[ADCReader] Channel 0 ADC Value: 904, Average: 274.78 degrees
[Motor 1] Mapped ADC Average to 274.78 degrees
[Motor 1] Sawtooth Wave Set Position: 71.54°
[Motor 1] Calculated Error: 126.77° (Set Position: 71.54°, Current Angle: 274.78°)
[PIDController] Error: 126.77, Integral: -628.28, Derivative: -104.70, Control Signal: -29.04
[Motor] Set speed to 29.04299374843848% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 274.78°, Error: 126.77°, Control Signal: 29.04%, Set Position: 71.54°
[ADCReader] Channel 2 ADC Value: 933, Average: 324.80 degrees
[Motor 3] Mapped ADC Average to 324.80 degrees
[Motor 3] Sawtooth Wave Set Position: 71.94°
[Motor 3] Calculated Error: 77.14° (Set Position: 71.94°, Current Angle: 324.80°)
[PIDController] Error: 77.14, Integral: 717.31, Derivative: 302.82, Control Signal: 55.63
[Motor] Set speed to 55.63433189228323% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 324.80°, Error: 77.14°, Control Signal: 55.63%, Set Position: 71.94°
[ADCReader] Channel 0 ADC Value: 927, Average: 283.10 degrees
[Motor 1] Mapped ADC Average to 283.10 degrees
[Motor 1] Sawtooth Wave Set Position: 74.91°
[Motor 1] Calculated Error: 121.81° (Set Position: 74.91°, Current Angle: 283.10°)
[PIDController] Error: 121.81, Integral: -622.06, Derivative: -97.18, Control Signal: -28.65
[Motor] Set speed to 28.653686687585907% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 283.10°, Error: 121.81°, Control Signal: 28.65%, Set Position: 74.91°
[ADCReader] Channel 2 ADC Value: 902, Average: 311.25 degrees
[Motor 3] Mapped ADC Average to 311.25 degrees
[Motor 3] Sawtooth Wave Set Position: 75.31°
[Motor 3] Calculated Error: 94.06° (Set Position: 75.31°, Current Angle: 311.25°)
[PIDController] Error: 94.06, Integral: 722.11, Derivative: 331.72, Control Signal: 58.33
[Motor] Set speed to 58.33458919166435% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 311.25°, Error: 94.06°, Control Signal: 58.33%, Set Position: 75.31°
[ADCReader] Channel 0 ADC Value: 950, Average: 291.17 degrees
[Motor 1] Mapped ADC Average to 291.17 degrees
[Motor 1] Sawtooth Wave Set Position: 78.28°
[Motor 1] Calculated Error: 117.11° (Set Position: 78.28°, Current Angle: 291.17°)
[PIDController] Error: 117.11, Integral: -616.08, Derivative: -91.94, Control Signal: -28.37
[Motor] Set speed to 28.37438969760077% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 291.17°, Error: 117.11°, Control Signal: 28.37%, Set Position: 78.28°
[ADCReader] Channel 2 ADC Value: 866, Average: 301.30 degrees
[Motor 3] Mapped ADC Average to 301.30 degrees
[Motor 3] Sawtooth Wave Set Position: 78.67°
[Motor 3] Calculated Error: 107.38° (Set Position: 78.67°, Current Angle: 301.30°)
[PIDController] Error: 107.38, Integral: 727.59, Derivative: 260.95, Control Signal: 55.87
[Motor] Set speed to 55.86954206875027% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 301.30°, Error: 107.38°, Control Signal: 55.87%, Set Position: 78.67°
[ADCReader] Channel 0 ADC Value: 971, Average: 298.84 degrees
[Motor 1] Mapped ADC Average to 298.84 degrees
[Motor 1] Sawtooth Wave Set Position: 81.64°
[Motor 1] Calculated Error: 112.80° (Set Position: 81.64°, Current Angle: 298.84°)
[PIDController] Error: 112.80, Integral: -610.31, Derivative: -84.29, Control Signal: -27.96
[Motor] Set speed to 27.962101799951075% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 298.84°, Error: 112.80°, Control Signal: 27.96%, Set Position: 81.64°
[ADCReader] Channel 2 ADC Value: 832, Average: 290.45 degrees
[Motor 3] Mapped ADC Average to 290.45 degrees
[Motor 3] Sawtooth Wave Set Position: 82.04°
[Motor 3] Calculated Error: 121.59° (Set Position: 82.04°, Current Angle: 290.45°)
[PIDController] Error: 121.59, Integral: 733.80, Derivative: 278.12, Control Signal: 57.89
[Motor] Set speed to 57.89099747154887% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 290.45°, Error: 121.59°, Control Signal: 57.89%, Set Position: 82.04°
[ADCReader] Channel 0 ADC Value: 994, Average: 306.19 degrees
[Motor 1] Mapped ADC Average to 306.19 degrees
[Motor 1] Sawtooth Wave Set Position: 85.07°
[Motor 1] Calculated Error: 108.88° (Set Position: 85.07°, Current Angle: 306.19°)
[PIDController] Error: 108.88, Integral: -604.72, Derivative: -76.28, Control Signal: -27.52
[Motor] Set speed to 27.517410015218246% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 306.19°, Error: 108.88°, Control Signal: 27.52%, Set Position: 85.07°
[ADCReader] Channel 2 ADC Value: 791, Average: 278.99 degrees
[Motor 3] Mapped ADC Average to 278.99 degrees
[Motor 3] Sawtooth Wave Set Position: 85.40°
[Motor 3] Calculated Error: 136.41° (Set Position: 85.40°, Current Angle: 278.99°)
[PIDController] Error: 136.41, Integral: 740.76, Derivative: 290.52, Control Signal: 59.75
[Motor] Set speed to 59.748673355337885% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 278.99°, Error: 136.41°, Control Signal: 59.75%, Set Position: 85.40°
[ADCReader] Channel 0 ADC Value: 1017, Average: 313.48 degrees
[Motor 1] Mapped ADC Average to 313.48 degrees
[Motor 1] Sawtooth Wave Set Position: 88.44°
[Motor 1] Calculated Error: 104.96° (Set Position: 88.44°, Current Angle: 313.48°)
[PIDController] Error: 104.96, Integral: -599.35, Derivative: -76.62, Control Signal: -27.50
[Motor] Set speed to 27.50071990116154% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 313.48°, Error: 104.96°, Control Signal: 27.50%, Set Position: 88.44°
[ADCReader] Channel 2 ADC Value: 753, Average: 267.37 degrees
[Motor 3] Mapped ADC Average to 267.37 degrees
[Motor 3] Sawtooth Wave Set Position: 88.77°
[Motor 3] Calculated Error: 151.40° (Set Position: 88.77°, Current Angle: 267.37°)
[PIDController] Error: 151.40, Integral: 748.54, Derivative: 291.81, Control Signal: 61.10
[Motor] Set speed to 61.1011322106835% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 267.37°, Error: 151.40°, Control Signal: 61.10%, Set Position: 88.77°
[ADCReader] Channel 0 ADC Value: 1019, Average: 319.43 degrees
[Motor 1] Mapped ADC Average to 319.43 degrees
[Motor 1] Sawtooth Wave Set Position: 91.81°
[Motor 1] Calculated Error: 102.38° (Set Position: 91.81°, Current Angle: 319.43°)
[PIDController] Error: 102.38, Integral: -594.12, Derivative: -50.51, Control Signal: -26.09
[Motor] Set speed to 26.088536214527185% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 319.43°, Error: 102.38°, Control Signal: 26.09%, Set Position: 91.81°
[ADCReader] Channel 2 ADC Value: 716, Average: 255.36 degrees
[Motor 3] Mapped ADC Average to 255.36 degrees
[Motor 3] Sawtooth Wave Set Position: 92.20°
[Motor 3] Calculated Error: -163.16° (Set Position: 92.20°, Current Angle: 255.36°)
[PIDController] Error: -163.16, Integral: 740.19, Derivative: -6150.81, Control Signal: -100.00
[Motor] Set speed to 51.1011322106835% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 255.36°, Error: -163.16°, Control Signal: 51.10%, Set Position: 92.20°
[ADCReader] Channel 0 ADC Value: 442, Average: 322.70 degrees
[Motor 1] Mapped ADC Average to 322.70 degrees
[Motor 1] Sawtooth Wave Set Position: 95.17°
[Motor 1] Calculated Error: 102.48° (Set Position: 95.17°, Current Angle: 322.70°)
[PIDController] Error: 102.48, Integral: -588.85, Derivative: 1.93, Control Signal: -23.20
[Motor] Set speed to 23.19789314815937% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 322.70°, Error: 102.48°, Control Signal: 23.20%, Set Position: 95.17°
[ADCReader] Channel 2 ADC Value: 680, Average: 243.34 degrees
[Motor 3] Mapped ADC Average to 243.34 degrees
[Motor 3] Sawtooth Wave Set Position: 95.57°
[Motor 3] Calculated Error: -147.77° (Set Position: 95.57°, Current Angle: 243.34°)
[PIDController] Error: -147.77, Integral: 732.65, Derivative: 301.42, Control Signal: 42.84
[Motor] Set speed to 42.837366449731185% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 243.34°, Error: -147.77°, Control Signal: 42.84%, Set Position: 95.57°
[ADCReader] Channel 0 ADC Value: 247, Average: 27.20 degrees
[Motor 1] Mapped ADC Average to 27.20 degrees
[Motor 1] Sawtooth Wave Set Position: 98.60°
[Motor 1] Calculated Error: 71.40° (Set Position: 98.60°, Current Angle: 27.20°)
[PIDController] Error: 71.40, Integral: -585.20, Derivative: -607.15, Control Signal: -55.33
[Motor] Set speed to 33.197893148159366% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 27.20°, Error: 71.40°, Control Signal: 33.20%, Set Position: 98.60°
[ADCReader] Channel 2 ADC Value: 649, Average: 231.53 degrees
[Motor 3] Mapped ADC Average to 231.53 degrees
[Motor 3] Sawtooth Wave Set Position: 98.93°
[Motor 3] Calculated Error: -132.59° (Set Position: 98.93°, Current Angle: 231.53°)
[PIDController] Error: -132.59, Integral: 725.87, Derivative: 296.94, Control Signal: 43.19
[Motor] Set speed to 43.185157013751045% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 231.53°, Error: -132.59°, Control Signal: 43.19%, Set Position: 98.93°
[ADCReader] Channel 0 ADC Value: 0, Average: 14.58 degrees
[Motor 1] Mapped ADC Average to 14.58 degrees
[Motor 1] Sawtooth Wave Set Position: 101.97°
[Motor 1] Calculated Error: 87.39° (Set Position: 101.97°, Current Angle: 14.58°)
[PIDController] Error: 87.39, Integral: -580.73, Derivative: 312.82, Control Signal: -8.15
[Motor] Set speed to 23.197893148159366% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 14.58°, Error: 87.39°, Control Signal: 23.20%, Set Position: 101.97°
[ADCReader] Channel 2 ADC Value: 616, Average: 220.24 degrees
[Motor 3] Mapped ADC Average to 220.24 degrees
[Motor 3] Sawtooth Wave Set Position: 102.30°
[Motor 3] Calculated Error: -117.94° (Set Position: 102.30°, Current Angle: 220.24°)
[PIDController] Error: -117.94, Integral: 719.84, Derivative: 286.38, Control Signal: 43.23
[Motor] Set speed to 43.234243081784186% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 220.24°, Error: -117.94°, Control Signal: 43.23%, Set Position: 102.30°
[ADCReader] Channel 0 ADC Value: 13, Average: 27.11 degrees
[Motor 1] Mapped ADC Average to 27.11 degrees
[Motor 1] Sawtooth Wave Set Position: 105.34°
[Motor 1] Calculated Error: 78.22° (Set Position: 105.34°, Current Angle: 27.11°)
[PIDController] Error: 78.22, Integral: -576.74, Derivative: -179.50, Control Signal: -33.12
[Motor] Set speed to 33.11829612528431% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 27.11°, Error: 78.22°, Control Signal: 33.12%, Set Position: 105.34°
[ADCReader] Channel 2 ADC Value: 584, Average: 209.35 degrees
[Motor 3] Mapped ADC Average to 209.35 degrees
[Motor 3] Sawtooth Wave Set Position: 105.67°
[Motor 3] Calculated Error: -103.68° (Set Position: 105.67°, Current Angle: 209.35°)
[PIDController] Error: -103.68, Integral: 714.54, Derivative: 279.23, Control Signal: 43.47
[Motor] Set speed to 43.46801420285155% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 209.35°, Error: -103.68°, Control Signal: 43.47%, Set Position: 105.67°
[ADCReader] Channel 0 ADC Value: 35, Average: 38.21 degrees
[Motor 1] Mapped ADC Average to 38.21 degrees
[Motor 1] Sawtooth Wave Set Position: 108.70°
[Motor 1] Calculated Error: 70.49° (Set Position: 108.70°, Current Angle: 38.21°)
[PIDController] Error: 70.49, Integral: -573.14, Derivative: -151.57, Control Signal: -32.01
[Motor] Set speed to 32.00618470290966% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 38.21°, Error: 70.49°, Control Signal: 32.01%, Set Position: 108.70°
[ADCReader] Channel 2 ADC Value: 556, Average: 199.03 degrees
[Motor 3] Mapped ADC Average to 199.03 degrees
[Motor 3] Sawtooth Wave Set Position: 109.03°
[Motor 3] Calculated Error: -89.99° (Set Position: 109.03°, Current Angle: 199.03°)
[PIDController] Error: -89.99, Integral: 709.96, Derivative: 268.71, Control Signal: 43.53
[Motor] Set speed to 43.533746743691296% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 199.03°, Error: -89.99°, Control Signal: 43.53%, Set Position: 109.03°
[ADCReader] Channel 0 ADC Value: 57, Average: 20.90 degrees
[Motor 1] Mapped ADC Average to 20.90 degrees
[Motor 1] Sawtooth Wave Set Position: 112.07°
[Motor 1] Calculated Error: 91.17° (Set Position: 112.07°, Current Angle: 20.90°)
[PIDController] Error: 91.17, Integral: -568.46, Derivative: 402.85, Control Signal: -2.81
[Motor] Set speed to 22.006184702909657% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 20.90°, Error: 91.17°, Control Signal: 22.01%, Set Position: 112.07°
[ADCReader] Channel 2 ADC Value: 527, Average: 189.15 degrees
[Motor 3] Mapped ADC Average to 189.15 degrees
[Motor 3] Sawtooth Wave Set Position: 112.40°
[Motor 3] Calculated Error: -76.75° (Set Position: 112.40°, Current Angle: 189.15°)
[PIDController] Error: -76.75, Integral: 706.02, Derivative: 258.11, Control Signal: 43.60
[Motor] Set speed to 43.60171655519355% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 189.15°, Error: -76.75°, Control Signal: 43.60%, Set Position: 112.40°
[ADCReader] Channel 0 ADC Value: 79, Average: 11.86 degrees
[Motor 1] Mapped ADC Average to 11.86 degrees
[Motor 1] Sawtooth Wave Set Position: 115.43°
[Motor 1] Calculated Error: 103.57° (Set Position: 115.43°, Current Angle: 11.86°)
[PIDController] Error: 103.57, Integral: -563.15, Derivative: 241.65, Control Signal: -9.86
[Motor] Set speed to 12.006184702909657% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 11.86°, Error: 103.57°, Control Signal: 12.01%, Set Position: 115.43°
[ADCReader] Channel 2 ADC Value: 494, Average: 179.16 degrees
[Motor 3] Mapped ADC Average to 179.16 degrees
[Motor 3] Sawtooth Wave Set Position: 115.76°
[Motor 3] Calculated Error: -63.40° (Set Position: 115.76°, Current Angle: 179.16°)
[PIDController] Error: -63.40, Integral: 702.79, Derivative: 261.61, Control Signal: 44.42
[Motor] Set speed to 44.41602429646635% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 179.16°, Error: -63.40°, Control Signal: 44.42%, Set Position: 115.76°
[ADCReader] Channel 0 ADC Value: 96, Average: 18.07 degrees
[Motor 1] Mapped ADC Average to 18.07 degrees
[Motor 1] Sawtooth Wave Set Position: 118.87°
[Motor 1] Calculated Error: 100.80° (Set Position: 118.87°, Current Angle: 18.07°)
[PIDController] Error: 100.80, Integral: -557.99, Derivative: -54.16, Control Signal: -24.56
[Motor] Set speed to 22.006184702909657% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 18.07°, Error: 100.80°, Control Signal: 22.01%, Set Position: 118.87°
[ADCReader] Channel 2 ADC Value: 462, Average: 169.24 degrees
[Motor 3] Mapped ADC Average to 169.24 degrees
[Motor 3] Sawtooth Wave Set Position: 119.13°
[Motor 3] Calculated Error: -50.11° (Set Position: 119.13°, Current Angle: 169.24°)
[PIDController] Error: -50.11, Integral: 700.23, Derivative: 260.41, Control Signal: 45.03
[Motor] Set speed to 45.02536548913527% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 169.24°, Error: -50.11°, Control Signal: 45.03%, Set Position: 119.13°
[ADCReader] Channel 0 ADC Value: 113, Average: 24.52 degrees
[Motor 1] Mapped ADC Average to 24.52 degrees
[Motor 1] Sawtooth Wave Set Position: 122.23°
[Motor 1] Calculated Error: 97.71° (Set Position: 122.23°, Current Angle: 24.52°)
[PIDController] Error: 97.71, Integral: -553.00, Derivative: -60.50, Control Signal: -24.81
[Motor] Set speed to 24.81256313637603% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 24.52°, Error: 97.71°, Control Signal: 24.81%, Set Position: 122.23°
[ADCReader] Channel 2 ADC Value: 429, Average: 159.23 degrees
[Motor 3] Mapped ADC Average to 159.23 degrees
[Motor 3] Sawtooth Wave Set Position: 122.50°
[Motor 3] Calculated Error: -36.74° (Set Position: 122.50°, Current Angle: 159.23°)
[PIDController] Error: -36.74, Integral: 698.36, Derivative: 262.28, Control Signal: 45.83
[Motor] Set speed to 45.82735356445464% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 159.23°, Error: -36.74°, Control Signal: 45.83%, Set Position: 122.50°
[ADCReader] Channel 0 ADC Value: 130, Average: 30.65 degrees
[Motor 1] Mapped ADC Average to 30.65 degrees
[Motor 1] Sawtooth Wave Set Position: 125.60°
[Motor 1] Calculated Error: 94.95° (Set Position: 125.60°, Current Angle: 30.65°)
[PIDController] Error: 94.95, Integral: -548.15, Derivative: -54.05, Control Signal: -24.41
[Motor] Set speed to 24.41322264177584% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 30.65°, Error: 94.95°, Control Signal: 24.41%, Set Position: 125.60°
[ADCReader] Channel 2 ADC Value: 398, Average: 149.03 degrees
[Motor 3] Mapped ADC Average to 149.03 degrees
[Motor 3] Sawtooth Wave Set Position: 125.86°
[Motor 3] Calculated Error: -23.17° (Set Position: 125.86°, Current Angle: 149.03°)
[PIDController] Error: -23.17, Integral: 697.17, Derivative: 266.13, Control Signal: 46.78
[Motor] Set speed to 46.77519609746839% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 149.03°, Error: -23.17°, Control Signal: 46.78%, Set Position: 125.86°
[ADCReader] Channel 0 ADC Value: 151, Average: 36.71 degrees
[Motor 1] Mapped ADC Average to 36.71 degrees
[Motor 1] Sawtooth Wave Set Position: 128.96°
[Motor 1] Calculated Error: 92.26° (Set Position: 128.96°, Current Angle: 36.71°)
[PIDController] Error: 92.26, Integral: -543.44, Derivative: -52.70, Control Signal: -24.27
[Motor] Set speed to 24.271192832396274% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 36.71°, Error: 92.26°, Control Signal: 24.27%, Set Position: 128.96°
[ADCReader] Channel 2 ADC Value: 368, Average: 138.77 degrees
[Motor 3] Mapped ADC Average to 138.77 degrees
[Motor 3] Sawtooth Wave Set Position: 129.23°
[Motor 3] Calculated Error: -9.54° (Set Position: 129.23°, Current Angle: 138.77°)
[PIDController] Error: -9.54, Integral: 696.69, Derivative: 267.22, Control Signal: 47.62
[Motor] Set speed to 47.62305157654545% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 138.77°, Error: -9.54°, Control Signal: 47.62%, Set Position: 129.23°
[ADCReader] Channel 0 ADC Value: 172, Average: 42.71 degrees
[Motor 1] Mapped ADC Average to 42.71 degrees
[Motor 1] Sawtooth Wave Set Position: 132.33°
[Motor 1] Calculated Error: 89.62° (Set Position: 132.33°, Current Angle: 42.71°)
[PIDController] Error: 89.62, Integral: -538.86, Derivative: -51.57, Control Signal: -24.14
[Motor] Set speed to 24.143966121666097% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 42.71°, Error: 89.62°, Control Signal: 24.14%, Set Position: 132.33°
[ADCReader] Channel 2 ADC Value: 332, Average: 128.33 degrees
[Motor 3] Mapped ADC Average to 128.33 degrees
[Motor 3] Sawtooth Wave Set Position: 132.59°
[Motor 3] Calculated Error: 4.27° (Set Position: 132.59°, Current Angle: 128.33°)
[PIDController] Error: 4.27, Integral: 696.91, Derivative: 270.79, Control Signal: 48.64
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 0 ADC Value: 192, Average: 48.90 degrees
[Motor 1] Mapped ADC Average to 48.90 degrees
[Motor 1] Sawtooth Wave Set Position: 135.70°
[Motor 1] Calculated Error: 86.80° (Set Position: 135.70°, Current Angle: 48.90°)
[PIDController] Error: 86.80, Integral: -534.43, Derivative: -55.48, Control Signal: -24.29
[Motor] Set speed to 24.287952392474608% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 48.90°, Error: 86.80°, Control Signal: 24.29%, Set Position: 135.70°
[ADCReader] Channel 2 ADC Value: 304, Average: 118.13 degrees
[Motor 3] Mapped ADC Average to 118.13 degrees
[Motor 3] Sawtooth Wave Set Position: 135.96°
[Motor 3] Calculated Error: 17.83° (Set Position: 135.96°, Current Angle: 118.13°)
[PIDController] Error: 17.83, Integral: 697.81, Derivative: 266.14, Control Signal: 49.27
[Motor] Set speed to 49.26750716495556% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 118.13°, Error: 17.83°, Control Signal: 49.27%, Set Position: 135.96°
[ADCReader] Channel 0 ADC Value: 215, Average: 55.48 degrees
[Motor 1] Mapped ADC Average to 55.48 degrees
[Motor 1] Sawtooth Wave Set Position: 139.06°
[Motor 1] Calculated Error: 83.58° (Set Position: 139.06°, Current Angle: 55.48°)
[PIDController] Error: 83.58, Integral: -530.17, Derivative: -63.02, Control Signal: -24.64
[Motor] Set speed to 24.64451543699439% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 55.48°, Error: 83.58°, Control Signal: 24.64%, Set Position: 139.06°
[ADCReader] Channel 2 ADC Value: 276, Average: 108.25 degrees
[Motor 3] Mapped ADC Average to 108.25 degrees
[Motor 3] Sawtooth Wave Set Position: 139.33°
[Motor 3] Calculated Error: 31.08° (Set Position: 139.33°, Current Angle: 108.25°)
[PIDController] Error: 31.08, Integral: 699.40, Derivative: 259.88, Control Signal: 49.83
[Motor] Set speed to 49.82863162524562% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 108.25°, Error: 31.08°, Control Signal: 49.83%, Set Position: 139.33°
[ADCReader] Channel 0 ADC Value: 240, Average: 62.57 degrees
[Motor 1] Mapped ADC Average to 62.57 degrees
[Motor 1] Sawtooth Wave Set Position: 142.43°
[Motor 1] Calculated Error: 79.85° (Set Position: 142.43°, Current Angle: 62.57°)
[PIDController] Error: 79.85, Integral: -526.09, Derivative: -72.87, Control Signal: -25.16
[Motor] Set speed to 25.156554807895834% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 62.57°, Error: 79.85°, Control Signal: 25.16%, Set Position: 142.43°
[ADCReader] Channel 2 ADC Value: 242, Average: 98.19 degrees
[Motor 3] Mapped ADC Average to 98.19 degrees
[Motor 3] Sawtooth Wave Set Position: 142.69°
[Motor 3] Calculated Error: 44.50° (Set Position: 142.69°, Current Angle: 98.19°)
[PIDController] Error: 44.50, Integral: 701.67, Derivative: 263.24, Control Signal: 50.92
[Motor] Set speed to 50.91553514905796% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 98.19°, Error: 44.50°, Control Signal: 50.92%, Set Position: 142.69°
[ADCReader] Channel 0 ADC Value: 259, Average: 69.55 degrees
[Motor 1] Mapped ADC Average to 69.55 degrees
[Motor 1] Sawtooth Wave Set Position: 145.79°
[Motor 1] Calculated Error: 76.25° (Set Position: 145.79°, Current Angle: 69.55°)
[PIDController] Error: 76.25, Integral: -522.20, Derivative: -70.70, Control Signal: -25.07
[Motor] Set speed to 25.07018666838324% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 69.55°, Error: 76.25°, Control Signal: 25.07%, Set Position: 145.79°
[ADCReader] Channel 2 ADC Value: 206, Average: 87.76 degrees
[Motor 3] Mapped ADC Average to 87.76 degrees
[Motor 3] Sawtooth Wave Set Position: 146.06°
[Motor 3] Calculated Error: 58.29° (Set Position: 146.06°, Current Angle: 87.76°)
[PIDController] Error: 58.29, Integral: 704.64, Derivative: 270.53, Control Signal: 52.26
[Motor] Set speed to 52.255992655589% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 87.76°, Error: 58.29°, Control Signal: 52.26%, Set Position: 146.06°
[ADCReader] Channel 0 ADC Value: 281, Average: 76.58 degrees
[Motor 1] Mapped ADC Average to 76.58 degrees
[Motor 1] Sawtooth Wave Set Position: 149.16°
[Motor 1] Calculated Error: 72.58° (Set Position: 149.16°, Current Angle: 76.58°)
[PIDController] Error: 72.58, Integral: -518.49, Derivative: -71.92, Control Signal: -25.17
[Motor] Set speed to 25.166003684284828% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 76.58°, Error: 72.58°, Control Signal: 25.17%, Set Position: 149.16°
[ADCReader] Channel 2 ADC Value: 171, Average: 77.37 degrees
[Motor 3] Mapped ADC Average to 77.37 degrees
[Motor 3] Sawtooth Wave Set Position: 149.42°
[Motor 3] Calculated Error: 72.05° (Set Position: 149.42°, Current Angle: 77.37°)
[PIDController] Error: 72.05, Integral: 708.31, Derivative: 269.80, Control Signal: 53.23
[Motor] Set speed to 53.22881801651758% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 77.37°, Error: 72.05°, Control Signal: 53.23%, Set Position: 149.42°
[ADCReader] Channel 0 ADC Value: 305, Average: 83.87 degrees
[Motor 1] Mapped ADC Average to 83.87 degrees
[Motor 1] Sawtooth Wave Set Position: 152.53°
[Motor 1] Calculated Error: 68.66° (Set Position: 152.53°, Current Angle: 83.87°)
[PIDController] Error: 68.66, Integral: -514.99, Derivative: -76.80, Control Signal: -25.47
[Motor] Set speed to 25.470167455772426% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 83.87°, Error: 68.66°, Control Signal: 25.47%, Set Position: 152.53°
[ADCReader] Channel 2 ADC Value: 136, Average: 66.52 degrees
[Motor 3] Mapped ADC Average to 66.52 degrees
[Motor 3] Sawtooth Wave Set Position: 152.79°
[Motor 3] Calculated Error: 86.27° (Set Position: 152.79°, Current Angle: 66.52°)
[PIDController] Error: 86.27, Integral: 712.71, Derivative: 278.79, Control Signal: 54.75
[Motor] Set speed to 54.7516040330957% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 66.52°, Error: 86.27°, Control Signal: 54.75%, Set Position: 152.79°
[ADCReader] Channel 0 ADC Value: 326, Average: 91.03 degrees
[Motor 1] Mapped ADC Average to 91.03 degrees
[Motor 1] Sawtooth Wave Set Position: 155.89°
[Motor 1] Calculated Error: 64.86° (Set Position: 155.89°, Current Angle: 91.03°)
[PIDController] Error: 64.86, Integral: -511.67, Derivative: -74.23, Control Signal: -25.40
[Motor] Set speed to 25.4034437785498% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 91.03°, Error: 64.86°, Control Signal: 25.40%, Set Position: 155.89°
[ADCReader] Channel 2 ADC Value: 102, Average: 55.29 degrees
[Motor 3] Mapped ADC Average to 55.29 degrees
[Motor 3] Sawtooth Wave Set Position: 156.16°
[Motor 3] Calculated Error: 100.87° (Set Position: 156.16°, Current Angle: 55.29°)
[PIDController] Error: 100.87, Integral: 717.86, Derivative: 285.96, Control Signal: 56.24
[Motor] Set speed to 56.24353514975023% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 55.29°, Error: 100.87°, Control Signal: 56.24%, Set Position: 156.16°
[ADCReader] Channel 0 ADC Value: 346, Average: 97.87 degrees
[Motor 1] Mapped ADC Average to 97.87 degrees
[Motor 1] Sawtooth Wave Set Position: 159.26°
[Motor 1] Calculated Error: 61.38° (Set Position: 159.26°, Current Angle: 97.87°)
[PIDController] Error: 61.38, Integral: -508.54, Derivative: -68.13, Control Signal: -25.15
[Motor] Set speed to 25.15060725084806% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 97.87°, Error: 61.38°, Control Signal: 25.15%, Set Position: 159.26°
[ADCReader] Channel 2 ADC Value: 68, Average: 44.06 degrees
[Motor 3] Mapped ADC Average to 44.06 degrees
[Motor 3] Sawtooth Wave Set Position: 159.52°
[Motor 3] Calculated Error: 115.46° (Set Position: 159.52°, Current Angle: 44.06°)
[PIDController] Error: 115.46, Integral: 723.75, Derivative: 286.38, Control Signal: 57.43
[Motor] Set speed to 57.43388817770612% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 44.06°, Error: 115.46°, Control Signal: 57.43%, Set Position: 159.52°
[ADCReader] Channel 0 ADC Value: 367, Average: 104.84 degrees
[Motor 1] Mapped ADC Average to 104.84 degrees
[Motor 1] Sawtooth Wave Set Position: 162.62°
[Motor 1] Calculated Error: 57.78° (Set Position: 162.62°, Current Angle: 104.84°)
[PIDController] Error: 57.78, Integral: -505.59, Derivative: -70.55, Control Signal: -25.34
[Motor] Set speed to 25.340127445305768% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 104.84°, Error: 57.78°, Control Signal: 25.34%, Set Position: 162.62°
[ADCReader] Channel 2 ADC Value: 31, Average: 32.78 degrees
[Motor 3] Mapped ADC Average to 32.78 degrees
[Motor 3] Sawtooth Wave Set Position: 162.89°
[Motor 3] Calculated Error: 130.11° (Set Position: 162.89°, Current Angle: 32.78°)
[PIDController] Error: 130.11, Integral: 730.38, Derivative: 287.50, Control Signal: 58.70
[Motor] Set speed to 58.70047967088894% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 32.78°, Error: 130.11°, Control Signal: 58.70%, Set Position: 162.89°
[ADCReader] Channel 0 ADC Value: 389, Average: 111.81 degrees
[Motor 1] Mapped ADC Average to 111.81 degrees
[Motor 1] Sawtooth Wave Set Position: 165.99°
[Motor 1] Calculated Error: 54.18° (Set Position: 165.99°, Current Angle: 111.81°)
[PIDController] Error: 54.18, Integral: -502.82, Derivative: -70.38, Control Signal: -25.41
[Motor] Set speed to 25.408674731770034% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 111.81°, Error: 54.18°, Control Signal: 25.41%, Set Position: 165.99°
[ADCReader] Channel 2 ADC Value: 0, Average: 21.74 degrees
[Motor 3] Mapped ADC Average to 21.74 degrees
[Motor 3] Sawtooth Wave Set Position: 166.25°
[Motor 3] Calculated Error: 144.52° (Set Position: 166.25°, Current Angle: 21.74°)
[PIDController] Error: 144.52, Integral: 737.75, Derivative: 282.49, Control Signal: 59.68
[Motor] Set speed to 59.682675039072265% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 21.74°, Error: 144.52°, Control Signal: 59.68%, Set Position: 166.25°
[ADCReader] Channel 0 ADC Value: 410, Average: 118.58 degrees
[Motor 1] Mapped ADC Average to 118.58 degrees
[Motor 1] Sawtooth Wave Set Position: 169.42°
[Motor 1] Calculated Error: 50.84° (Set Position: 169.42°, Current Angle: 118.58°)
[PIDController] Error: 50.84, Integral: -500.22, Derivative: -65.45, Control Signal: -25.23
[Motor] Set speed to 25.23289972823009% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 118.58°, Error: 50.84°, Control Signal: 25.23%, Set Position: 169.42°
[ADCReader] Channel 2 ADC Value: 10, Average: 13.57 degrees
[Motor 3] Mapped ADC Average to 13.57 degrees
[Motor 3] Sawtooth Wave Set Position: 169.62°
[Motor 3] Calculated Error: 156.05° (Set Position: 169.62°, Current Angle: 13.57°)
[PIDController] Error: 156.05, Integral: 745.71, Derivative: 226.09, Control Signal: 57.95
[Motor] Set speed to 57.95250446907201% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 13.57°, Error: 156.05°, Control Signal: 57.95%, Set Position: 169.62°
[ADCReader] Channel 0 ADC Value: 432, Average: 125.42 degrees
[Motor 1] Mapped ADC Average to 125.42 degrees
[Motor 1] Sawtooth Wave Set Position: 172.79°
[Motor 1] Calculated Error: 47.37° (Set Position: 172.79°, Current Angle: 125.42°)
[PIDController] Error: 47.37, Integral: -497.81, Derivative: -68.05, Control Signal: -25.45
[Motor] Set speed to 25.450831060892384% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 125.42°, Error: 47.37°, Control Signal: 25.45%, Set Position: 172.79°
[ADCReader] Channel 2 ADC Value: 1014, Average: 0.73 degrees
[Motor 3] Mapped ADC Average to 0.73 degrees
[Motor 3] Sawtooth Wave Set Position: 172.99°
[Motor 3] Calculated Error: -157.74° (Set Position: 172.99°, Current Angle: 0.73°)
[PIDController] Error: -157.74, Integral: 737.66, Derivative: -6152.44, Control Signal: -100.00
[Motor] Set speed to 47.95250446907201% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 0.73°, Error: -157.74°, Control Signal: 47.95%, Set Position: 172.99°
[ADCReader] Channel 0 ADC Value: 454, Average: 132.39 degrees
[Motor 1] Mapped ADC Average to 132.39 degrees
[Motor 1] Sawtooth Wave Set Position: 176.15°
[Motor 1] Calculated Error: 43.77° (Set Position: 176.15°, Current Angle: 132.39°)
[PIDController] Error: 43.77, Integral: -495.57, Derivative: -70.43, Control Signal: -25.67
[Motor] Set speed to 25.673900833660547% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 132.39°, Error: 43.77°, Control Signal: 25.67%, Set Position: 176.15°
[ADCReader] Channel 2 ADC Value: 991, Average: 18.17 degrees
[Motor 3] Mapped ADC Average to 18.17 degrees
[Motor 3] Sawtooth Wave Set Position: 176.35°
[Motor 3] Calculated Error: 158.18° (Set Position: 176.35°, Current Angle: 18.17°)
[PIDController] Error: 158.18, Integral: 745.73, Derivative: 6190.61, Control Signal: 100.00
[Motor] Set speed to 57.95250446907201% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 18.17°, Error: 158.18°, Control Signal: 57.95%, Set Position: 176.35°
[ADCReader] Channel 0 ADC Value: 476, Average: 139.42 degrees
[Motor 1] Mapped ADC Average to 139.42 degrees
[Motor 1] Sawtooth Wave Set Position: 179.52°
[Motor 1] Calculated Error: 40.10° (Set Position: 179.52°, Current Angle: 139.42°)
[PIDController] Error: 40.10, Integral: -493.52, Derivative: -71.77, Control Signal: -25.86
[Motor] Set speed to 25.858607112435564% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 139.42°, Error: 40.10°, Control Signal: 25.86%, Set Position: 179.52°
[ADCReader] Channel 2 ADC Value: 957, Average: 5.63 degrees
[Motor 3] Mapped ADC Average to 5.63 degrees
[Motor 3] Sawtooth Wave Set Position: 179.72°
[Motor 3] Calculated Error: -155.91° (Set Position: 179.72°, Current Angle: 5.63°)
[PIDController] Error: -155.91, Integral: 737.78, Derivative: -6160.62, Control Signal: -100.00
[Motor] Set speed to 47.95250446907201% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 5.63°, Error: -155.91°, Control Signal: 47.95%, Set Position: 179.72°
[ADCReader] Channel 0 ADC Value: 498, Average: 146.45 degrees
[Motor 1] Mapped ADC Average to 146.45 degrees
[Motor 1] Sawtooth Wave Set Position: 182.89°
[Motor 1] Calculated Error: 36.43° (Set Position: 182.89°, Current Angle: 146.45°)
[PIDController] Error: 36.43, Integral: -491.66, Derivative: -71.93, Control Signal: -25.99
[Motor] Set speed to 25.99379515752027% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 146.45°, Error: 36.43°, Control Signal: 25.99%, Set Position: 182.89°
[ADCReader] Channel 2 ADC Value: 924, Average: 322.89 degrees
[Motor 3] Mapped ADC Average to 322.89 degrees
[Motor 3] Sawtooth Wave Set Position: 183.08°
[Motor 3] Calculated Error: -139.80° (Set Position: 183.08°, Current Angle: 322.89°)
[PIDController] Error: -139.80, Integral: 730.61, Derivative: 314.07, Control Signal: 43.85
[Motor] Set speed to 43.845981168834854% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 322.89°, Error: -139.80°, Control Signal: 43.85%, Set Position: 183.08°
[ADCReader] Channel 0 ADC Value: 518, Average: 153.42 degrees
[Motor 1] Mapped ADC Average to 153.42 degrees
[Motor 1] Sawtooth Wave Set Position: 186.25°
[Motor 1] Calculated Error: 32.83° (Set Position: 186.25°, Current Angle: 153.42°)
[PIDController] Error: 32.83, Integral: -489.98, Derivative: -70.49, Control Signal: -26.05
[Motor] Set speed to 26.0537602458871% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 153.42°, Error: 32.83°, Control Signal: 26.05%, Set Position: 186.25°
[ADCReader] Channel 2 ADC Value: 884, Average: 307.78 degrees
[Motor 3] Mapped ADC Average to 307.78 degrees
[Motor 3] Sawtooth Wave Set Position: 186.52°
[Motor 3] Calculated Error: -121.26° (Set Position: 186.52°, Current Angle: 307.78°)
[PIDController] Error: -121.26, Integral: 724.42, Derivative: 363.03, Control Signal: 47.10
[Motor] Set speed to 47.09680228031757% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 307.78°, Error: -121.26°, Control Signal: 47.10%, Set Position: 186.52°
[ADCReader] Channel 0 ADC Value: 540, Average: 160.39 degrees
[Motor 1] Mapped ADC Average to 160.39 degrees
[Motor 1] Sawtooth Wave Set Position: 189.62°
[Motor 1] Calculated Error: 29.23° (Set Position: 189.62°, Current Angle: 160.39°)
[PIDController] Error: 29.23, Integral: -488.49, Derivative: -70.39, Control Signal: -26.19
[Motor] Set speed to 26.19037434572998% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 160.39°, Error: 29.23°, Control Signal: 26.19%, Set Position: 189.62°
[ADCReader] Channel 2 ADC Value: 858, Average: 297.67 degrees
[Motor 3] Mapped ADC Average to 297.67 degrees
[Motor 3] Sawtooth Wave Set Position: 189.88°
[Motor 3] Calculated Error: -107.79° (Set Position: 189.88°, Current Angle: 297.67°)
[PIDController] Error: -107.79, Integral: 718.92, Derivative: 263.98, Control Signal: 42.68
[Motor] Set speed to 42.67794039966088% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 297.67°, Error: -107.79°, Control Signal: 42.68%, Set Position: 189.88°
[ADCReader] Channel 0 ADC Value: 559, Average: 167.16 degrees
[Motor 1] Mapped ADC Average to 167.16 degrees
[Motor 1] Sawtooth Wave Set Position: 192.98°
[Motor 1] Calculated Error: 25.82° (Set Position: 192.98°, Current Angle: 167.16°)
[PIDController] Error: 25.82, Integral: -487.17, Derivative: -66.77, Control Signal: -26.15
[Motor] Set speed to 26.147620298343373% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 167.16°, Error: 25.82°, Control Signal: 26.15%, Set Position: 192.98°
[ADCReader] Channel 2 ADC Value: 824, Average: 286.89 degrees
[Motor 3] Mapped ADC Average to 286.89 degrees
[Motor 3] Sawtooth Wave Set Position: 193.25°
[Motor 3] Calculated Error: -93.64° (Set Position: 193.25°, Current Angle: 286.89°)
[PIDController] Error: -93.64, Integral: 714.14, Derivative: 277.39, Control Signal: 43.96
[Motor] Set speed to 43.958390735133655% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 286.89°, Error: -93.64°, Control Signal: 43.96%, Set Position: 193.25°
[ADCReader] Channel 0 ADC Value: 580, Average: 173.87 degrees
[Motor 1] Mapped ADC Average to 173.87 degrees
[Motor 1] Sawtooth Wave Set Position: 196.35°
[Motor 1] Calculated Error: 22.48° (Set Position: 196.35°, Current Angle: 173.87°)
[PIDController] Error: 22.48, Integral: -486.02, Derivative: -65.09, Control Signal: -26.21
[Motor] Set speed to 26.206500877916113% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 173.87°, Error: 22.48°, Control Signal: 26.21%, Set Position: 196.35°
[ADCReader] Channel 2 ADC Value: 788, Average: 276.00 degrees
[Motor 3] Mapped ADC Average to 276.00 degrees
[Motor 3] Sawtooth Wave Set Position: 196.61°
[Motor 3] Calculated Error: -79.39° (Set Position: 196.61°, Current Angle: 276.00°)
[PIDController] Error: -79.39, Integral: 710.10, Derivative: 279.54, Control Signal: 44.72
[Motor] Set speed to 44.71885649207026% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 276.00°, Error: -79.39°, Control Signal: 44.72%, Set Position: 196.61°
[ADCReader] Channel 0 ADC Value: 600, Average: 180.45 degrees
[Motor 1] Mapped ADC Average to 180.45 degrees
[Motor 1] Sawtooth Wave Set Position: 199.78°
[Motor 1] Calculated Error: 19.33° (Set Position: 199.78°, Current Angle: 180.45°)
[PIDController] Error: 19.33, Integral: -485.03, Derivative: -61.64, Control Signal: -26.17
[Motor] Set speed to 26.17342268496924% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 180.45°, Error: 19.33°, Control Signal: 26.17%, Set Position: 199.78°
[ADCReader] Channel 2 ADC Value: 756, Average: 265.18 degrees
[Motor 3] Mapped ADC Average to 265.18 degrees
[Motor 3] Sawtooth Wave Set Position: 199.98°
[Motor 3] Calculated Error: -65.20° (Set Position: 199.98°, Current Angle: 265.18°)
[PIDController] Error: -65.20, Integral: 706.76, Derivative: 277.67, Control Signal: 45.31
[Motor] Set speed to 45.31017850562601% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 265.18°, Error: -65.20°, Control Signal: 45.31%, Set Position: 199.98°
[ADCReader] Channel 0 ADC Value: 619, Average: 186.97 degrees
[Motor 1] Mapped ADC Average to 186.97 degrees
[Motor 1] Sawtooth Wave Set Position: 203.15°
[Motor 1] Calculated Error: 16.18° (Set Position: 203.15°, Current Angle: 186.97°)
[PIDController] Error: 16.18, Integral: -484.20, Derivative: -61.59, Control Signal: -26.32
[Motor] Set speed to 26.31901073536406% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 186.97°, Error: 16.18°, Control Signal: 26.32%, Set Position: 203.15°
[ADCReader] Channel 2 ADC Value: 720, Average: 254.58 degrees
[Motor 3] Mapped ADC Average to 254.58 degrees
[Motor 3] Sawtooth Wave Set Position: 203.35°
[Motor 3] Calculated Error: -51.23° (Set Position: 203.35°, Current Angle: 254.58°)
[PIDController] Error: -51.23, Integral: 704.15, Derivative: 273.30, Control Signal: 45.80
[Motor] Set speed to 45.798474263686714% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 254.58°, Error: -51.23°, Control Signal: 45.80%, Set Position: 203.35°
[ADCReader] Channel 0 ADC Value: 640, Average: 193.42 degrees
[Motor 1] Mapped ADC Average to 193.42 degrees
[Motor 1] Sawtooth Wave Set Position: 206.51°
[Motor 1] Calculated Error: 13.09° (Set Position: 206.51°, Current Angle: 193.42°)
[PIDController] Error: 13.09, Integral: -483.53, Derivative: -60.40, Control Signal: -26.41
[Motor] Set speed to 26.41101123782851% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 193.42°, Error: 13.09°, Control Signal: 26.41%, Set Position: 206.51°
[ADCReader] Channel 2 ADC Value: 690, Average: 243.73 degrees
[Motor 3] Mapped ADC Average to 243.73 degrees
[Motor 3] Sawtooth Wave Set Position: 206.71°
[Motor 3] Calculated Error: -37.02° (Set Position: 206.71°, Current Angle: 243.73°)
[PIDController] Error: -37.02, Integral: 702.26, Derivative: 278.90, Control Signal: 46.84
[Motor] Set speed to 46.836538174257235% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 243.73°, Error: -37.02°, Control Signal: 46.84%, Set Position: 206.71°
[ADCReader] Channel 0 ADC Value: 658, Average: 199.81 degrees
[Motor 1] Mapped ADC Average to 199.81 degrees
[Motor 1] Sawtooth Wave Set Position: 209.88°
[Motor 1] Calculated Error: 10.07° (Set Position: 209.88°, Current Angle: 199.81°)
[PIDController] Error: 10.07, Integral: -483.02, Derivative: -59.25, Control Signal: -26.51
[Motor] Set speed to 26.50895105144955% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 199.81°, Error: 10.07°, Control Signal: 26.51%, Set Position: 209.88°
[ADCReader] Channel 2 ADC Value: 653, Average: 232.71 degrees
[Motor 3] Mapped ADC Average to 232.71 degrees
[Motor 3] Sawtooth Wave Set Position: 210.08°
[Motor 3] Calculated Error: -22.64° (Set Position: 210.08°, Current Angle: 232.71°)
[PIDController] Error: -22.64, Integral: 701.11, Derivative: 282.17, Control Signal: 47.81
[Motor] Set speed to 47.80558803168301% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 232.71°, Error: -22.64°, Control Signal: 47.81%, Set Position: 210.08°
[ADCReader] Channel 0 ADC Value: 678, Average: 206.13 degrees
[Motor 1] Mapped ADC Average to 206.13 degrees
[Motor 1] Sawtooth Wave Set Position: 213.25°
[Motor 1] Calculated Error: 7.12° (Set Position: 213.25°, Current Angle: 206.13°)
[PIDController] Error: 7.12, Integral: -482.66, Derivative: -57.95, Control Signal: -26.60
[Motor] Set speed to 26.60319418900135% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 206.13°, Error: 7.12°, Control Signal: 26.60%, Set Position: 213.25°
[ADCReader] Channel 2 ADC Value: 624, Average: 222.12 degrees
[Motor 3] Mapped ADC Average to 222.12 degrees
[Motor 3] Sawtooth Wave Set Position: 213.44°
[Motor 3] Calculated Error: -8.68° (Set Position: 213.44°, Current Angle: 222.12°)
[PIDController] Error: -8.68, Integral: 700.66, Derivative: 273.56, Control Signal: 48.19
[Motor] Set speed to 48.19044351351457% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 222.12°, Error: -8.68°, Control Signal: 48.19%, Set Position: 213.44°
[ADCReader] Channel 0 ADC Value: 699, Average: 212.52 degrees
[Motor 1] Mapped ADC Average to 212.52 degrees
[Motor 1] Sawtooth Wave Set Position: 216.61°
[Motor 1] Calculated Error: 4.10° (Set Position: 216.61°, Current Angle: 212.52°)
[PIDController] Error: 4.10, Integral: -482.45, Derivative: -59.09, Control Signal: -26.83
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 2 ADC Value: 592, Average: 211.54 degrees
[Motor 3] Mapped ADC Average to 211.54 degrees
[Motor 3] Sawtooth Wave Set Position: 216.81°
[Motor 3] Calculated Error: 5.27° (Set Position: 216.81°, Current Angle: 211.54°)
[PIDController] Error: 5.27, Integral: 700.93, Derivative: 273.52, Control Signal: 49.04
[Motor] Set speed to 49.03869805917889% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 211.54°, Error: 5.27°, Control Signal: 49.04%, Set Position: 216.81°
[ADCReader] Channel 0 ADC Value: 715, Average: 218.71 degrees
[Motor 1] Mapped ADC Average to 218.71 degrees
[Motor 1] Sawtooth Wave Set Position: 219.98°
[Motor 1] Calculated Error: 1.27° (Set Position: 219.98°, Current Angle: 218.71°)
[PIDController] Error: 1.27, Integral: -482.38, Derivative: -55.44, Control Signal: -26.82
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 2 ADC Value: 560, Average: 201.22 degrees
[Motor 3] Mapped ADC Average to 201.22 degrees
[Motor 3] Sawtooth Wave Set Position: 220.18°
[Motor 3] Calculated Error: 18.96° (Set Position: 220.18°, Current Angle: 201.22°)
[PIDController] Error: 18.96, Integral: 701.90, Derivative: 268.67, Control Signal: 49.67
[Motor] Set speed to 49.665902759859314% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 201.22°, Error: 18.96°, Control Signal: 49.67%, Set Position: 220.18°
[ADCReader] Channel 0 ADC Value: 724, Average: 224.14 degrees
[Motor 1] Mapped ADC Average to 224.14 degrees
[Motor 1] Sawtooth Wave Set Position: 223.34°
[Motor 1] Calculated Error: -0.79° (Set Position: 223.34°, Current Angle: 224.14°)
[PIDController] Error: -0.79, Integral: -482.42, Derivative: -40.40, Control Signal: -26.19
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 2 ADC Value: 530, Average: 190.91 degrees
[Motor 3] Mapped ADC Average to 190.91 degrees
[Motor 3] Sawtooth Wave Set Position: 223.54°
[Motor 3] Calculated Error: 32.64° (Set Position: 223.54°, Current Angle: 190.91°)
[PIDController] Error: 32.64, Integral: 703.56, Derivative: 268.27, Control Signal: 50.55
[Motor] Set speed to 50.54960833055934% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 190.91°, Error: 32.64°, Control Signal: 50.55%, Set Position: 223.54°
[ADCReader] Channel 0 ADC Value: 722, Average: 228.26 degrees
[Motor 1] Mapped ADC Average to 228.26 degrees
[Motor 1] Sawtooth Wave Set Position: 226.71°
[Motor 1] Calculated Error: -1.55° (Set Position: 226.71°, Current Angle: 228.26°)
[PIDController] Error: -1.55, Integral: -482.50, Derivative: -14.95, Control Signal: -24.97
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 2 ADC Value: 497, Average: 180.84 degrees
[Motor 3] Mapped ADC Average to 180.84 degrees
[Motor 3] Sawtooth Wave Set Position: 226.91°
[Motor 3] Calculated Error: 46.07° (Set Position: 226.91°, Current Angle: 180.84°)
[PIDController] Error: 46.07, Integral: 705.91, Derivative: 263.35, Control Signal: 51.23
[Motor] Set speed to 51.227333147434756% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 180.84°, Error: 46.07°, Control Signal: 51.23%, Set Position: 226.91°
[ADCReader] Channel 0 ADC Value: 722, Average: 231.10 degrees
[Motor 1] Mapped ADC Average to 231.10 degrees
[Motor 1] Sawtooth Wave Set Position: 230.08°
[Motor 1] Calculated Error: -1.02° (Set Position: 230.08°, Current Angle: 231.10°)
[PIDController] Error: -1.02, Integral: -482.55, Derivative: 10.42, Control Signal: -23.67
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 2 ADC Value: 464, Average: 170.52 degrees
[Motor 3] Mapped ADC Average to 170.52 degrees
[Motor 3] Sawtooth Wave Set Position: 230.27°
[Motor 3] Calculated Error: 59.75° (Set Position: 230.27°, Current Angle: 170.52°)
[PIDController] Error: 59.75, Integral: 708.96, Derivative: 268.30, Control Signal: 52.45
[Motor] Set speed to 52.44791898175471% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 170.52°, Error: 59.75°, Control Signal: 52.45%, Set Position: 230.27°
[ADCReader] Channel 0 ADC Value: 721, Average: 232.52 degrees
[Motor 1] Mapped ADC Average to 232.52 degrees
[Motor 1] Sawtooth Wave Set Position: 233.44°
[Motor 1] Calculated Error: 0.93° (Set Position: 233.44°, Current Angle: 232.52°)
[PIDController] Error: 0.93, Integral: -482.51, Derivative: 37.97, Control Signal: -22.17
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 2 ADC Value: 430, Average: 160.07 degrees
[Motor 3] Mapped ADC Average to 160.07 degrees
[Motor 3] Sawtooth Wave Set Position: 233.64°
[Motor 3] Calculated Error: 73.57° (Set Position: 233.64°, Current Angle: 160.07°)
[PIDController] Error: 73.57, Integral: 712.71, Derivative: 270.87, Control Signal: 53.59
[Motor] Set speed to 53.593028998159056% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 160.07°, Error: 73.57°, Control Signal: 53.59%, Set Position: 233.64°
[ADCReader] Channel 0 ADC Value: 722, Average: 232.97 degrees
[Motor 1] Mapped ADC Average to 232.97 degrees
[Motor 1] Sawtooth Wave Set Position: 236.81°
[Motor 1] Calculated Error: 3.84° (Set Position: 236.81°, Current Angle: 232.97°)
[PIDController] Error: 3.84, Integral: -482.31, Derivative: 56.90, Control Signal: -21.04
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 2 ADC Value: 396, Average: 149.49 degrees
[Motor 3] Mapped ADC Average to 149.49 degrees
[Motor 3] Sawtooth Wave Set Position: 237.01°
[Motor 3] Calculated Error: 87.52° (Set Position: 237.01°, Current Angle: 149.49°)
[PIDController] Error: 87.52, Integral: 717.20, Derivative: 272.05, Control Signal: 54.71
[Motor] Set speed to 54.71376031217466% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 149.49°, Error: 87.52°, Control Signal: 54.71%, Set Position: 237.01°
[ADCReader] Channel 0 ADC Value: 722, Average: 232.84 degrees
[Motor 1] Mapped ADC Average to 232.84 degrees
[Motor 1] Sawtooth Wave Set Position: 240.24°
[Motor 1] Calculated Error: 7.40° (Set Position: 240.24°, Current Angle: 232.84°)
[PIDController] Error: 7.40, Integral: -481.93, Derivative: 69.49, Control Signal: -20.18
[Motor] Set speed to 20.17813504409284% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 232.84°, Error: 7.40°, Control Signal: 20.18%, Set Position: 240.24°
[ADCReader] Channel 2 ADC Value: 362, Average: 138.65 degrees
[Motor 3] Mapped ADC Average to 138.65 degrees
[Motor 3] Sawtooth Wave Set Position: 240.37°
[Motor 3] Calculated Error: 101.72° (Set Position: 240.37°, Current Angle: 138.65°)
[PIDController] Error: 101.72, Integral: 722.43, Derivative: 276.22, Control Signal: 56.04
[Motor] Set speed to 56.035946106815786% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 138.65°, Error: 101.72°, Control Signal: 56.04%, Set Position: 240.37°
[ADCReader] Channel 0 ADC Value: 725, Average: 233.03 degrees
[Motor 1] Mapped ADC Average to 233.03 degrees
[Motor 1] Sawtooth Wave Set Position: 243.61°
[Motor 1] Calculated Error: 10.57° (Set Position: 243.61°, Current Angle: 233.03°)
[PIDController] Error: 10.57, Integral: -481.39, Derivative: 62.04, Control Signal: -20.33
[Motor] Set speed to 20.333005463219475% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 233.03°, Error: 10.57°, Control Signal: 20.33%, Set Position: 243.61°
[ADCReader] Channel 2 ADC Value: 326, Average: 127.62 degrees
[Motor 3] Mapped ADC Average to 127.62 degrees
[Motor 3] Sawtooth Wave Set Position: 243.74°
[Motor 3] Calculated Error: 116.12° (Set Position: 243.74°, Current Angle: 127.62°)
[PIDController] Error: 116.12, Integral: 728.34, Derivative: 282.75, Control Signal: 57.52
[Motor] Set speed to 57.52205442700175% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 127.62°, Error: 116.12°, Control Signal: 57.52%, Set Position: 243.74°
[ADCReader] Channel 0 ADC Value: 735, Average: 233.87 degrees
[Motor 1] Mapped ADC Average to 233.87 degrees
[Motor 1] Sawtooth Wave Set Position: 246.97°
[Motor 1] Calculated Error: 13.10° (Set Position: 246.97°, Current Angle: 233.87°)
[PIDController] Error: 13.10, Integral: -480.72, Derivative: 49.47, Control Signal: -20.78
[Motor] Set speed to 20.776484078096992% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 233.87°, Error: 13.10°, Control Signal: 20.78%, Set Position: 246.97°
[ADCReader] Channel 2 ADC Value: 289, Average: 116.33 degrees
[Motor 3] Mapped ADC Average to 116.33 degrees
[Motor 3] Sawtooth Wave Set Position: 247.10°
[Motor 3] Calculated Error: 130.77° (Set Position: 247.10°, Current Angle: 116.33°)
[PIDController] Error: 130.77, Integral: 735.01, Derivative: 287.15, Control Signal: 58.95
[Motor] Set speed to 58.95461770810428% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 116.33°, Error: 130.77°, Control Signal: 58.95%, Set Position: 247.10°
[ADCReader] Channel 0 ADC Value: 746, Average: 235.48 degrees
[Motor 1] Mapped ADC Average to 235.48 degrees
[Motor 1] Sawtooth Wave Set Position: 250.34°
[Motor 1] Calculated Error: 14.86° (Set Position: 250.34°, Current Angle: 235.48°)
[PIDController] Error: 14.86, Integral: -479.96, Derivative: 34.31, Control Signal: -21.39
[Motor] Set speed to 21.39100238784972% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 235.48°, Error: 14.86°, Control Signal: 21.39%, Set Position: 250.34°
[ADCReader] Channel 2 ADC Value: 251, Average: 104.79 degrees
[Motor 3] Mapped ADC Average to 104.79 degrees
[Motor 3] Sawtooth Wave Set Position: 250.47°
[Motor 3] Calculated Error: 145.68° (Set Position: 250.47°, Current Angle: 104.79°)
[PIDController] Error: 145.68, Integral: 742.47, Derivative: 291.42, Control Signal: 60.44
[Motor] Set speed to 60.435649066909725% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 104.79°, Error: 145.68°, Control Signal: 60.44%, Set Position: 250.47°
[ADCReader] Channel 0 ADC Value: 760, Average: 237.93 degrees
[Motor 1] Mapped ADC Average to 237.93 degrees
[Motor 1] Sawtooth Wave Set Position: 253.70°
[Motor 1] Calculated Error: 15.77° (Set Position: 253.70°, Current Angle: 237.93°)
[PIDController] Error: 15.77, Integral: -479.16, Derivative: 17.93, Control Signal: -22.11
[Motor] Set speed to 22.11493046897719% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 237.93°, Error: 15.77°, Control Signal: 22.11%, Set Position: 253.70°
[ADCReader] Channel 2 ADC Value: 213, Average: 92.98 degrees
[Motor 3] Mapped ADC Average to 92.98 degrees
[Motor 3] Sawtooth Wave Set Position: 253.84°
[Motor 3] Calculated Error: 160.86° (Set Position: 253.84°, Current Angle: 92.98°)
[PIDController] Error: 160.86, Integral: 750.67, Derivative: 297.80, Control Signal: 62.08
[Motor] Set speed to 62.075009529504285% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 92.98°, Error: 160.86°, Control Signal: 62.08%, Set Position: 253.84°
[ADCReader] Channel 0 ADC Value: 780, Average: 241.67 degrees
[Motor 1] Mapped ADC Average to 241.67 degrees
[Motor 1] Sawtooth Wave Set Position: 257.07°
[Motor 1] Calculated Error: 15.40° (Set Position: 257.07°, Current Angle: 241.67°)
[PIDController] Error: 15.40, Integral: -478.37, Derivative: -7.32, Control Signal: -23.36
[Motor] Set speed to 23.36041069733293% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 241.67°, Error: 15.40°, Control Signal: 23.36%, Set Position: 257.07°
[ADCReader] Channel 2 ADC Value: 175, Average: 80.91 degrees
[Motor 3] Mapped ADC Average to 80.91 degrees
[Motor 3] Sawtooth Wave Set Position: 257.20°
[Motor 3] Calculated Error: -153.70° (Set Position: 257.20°, Current Angle: 80.91°)
[PIDController] Error: -153.70, Integral: 742.84, Derivative: -6173.39, Control Signal: -100.00
[Motor] Set speed to 52.075009529504285% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 80.91°, Error: -153.70°, Control Signal: 52.08%, Set Position: 257.20°
[ADCReader] Channel 0 ADC Value: 794, Average: 246.13 degrees
[Motor 1] Mapped ADC Average to 246.13 degrees
[Motor 1] Sawtooth Wave Set Position: 260.44°
[Motor 1] Calculated Error: 14.31° (Set Position: 260.44°, Current Angle: 246.13°)
[PIDController] Error: 14.31, Integral: -477.64, Derivative: -21.30, Control Signal: -24.09
[Motor] Set speed to 24.088213483656638% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 246.13°, Error: 14.31°, Control Signal: 24.09%, Set Position: 260.44°
[ADCReader] Channel 2 ADC Value: 138, Average: 68.77 degrees
[Motor 3] Mapped ADC Average to 68.77 degrees
[Motor 3] Sawtooth Wave Set Position: 260.57°
[Motor 3] Calculated Error: -138.20° (Set Position: 260.57°, Current Angle: 68.77°)
[PIDController] Error: -138.20, Integral: 735.78, Derivative: 303.64, Control Signal: 43.68
[Motor] Set speed to 43.678832948489415% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 68.77°, Error: -138.20°, Control Signal: 43.68%, Set Position: 260.57°
[ADCReader] Channel 0 ADC Value: 814, Average: 251.22 degrees
[Motor 1] Mapped ADC Average to 251.22 degrees
[Motor 1] Sawtooth Wave Set Position: 263.80°
[Motor 1] Calculated Error: 12.58° (Set Position: 263.80°, Current Angle: 251.22°)
[PIDController] Error: 12.58, Integral: -477.00, Derivative: -33.93, Control Signal: -24.79
[Motor] Set speed to 24.7915368742263% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 251.22°, Error: 12.58°, Control Signal: 24.79%, Set Position: 263.80°
[ADCReader] Channel 2 ADC Value: 104, Average: 56.83 degrees
[Motor 3] Mapped ADC Average to 56.83 degrees
[Motor 3] Sawtooth Wave Set Position: 263.93°
[Motor 3] Calculated Error: -122.89° (Set Position: 263.93°, Current Angle: 56.83°)
[PIDController] Error: -122.89, Integral: 729.51, Derivative: 300.19, Control Signal: 44.11
[Motor] Set speed to 44.11185594305663% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 56.83°, Error: -122.89°, Control Signal: 44.11%, Set Position: 263.93°
[ADCReader] Channel 0 ADC Value: 832, Average: 256.77 degrees
[Motor 1] Mapped ADC Average to 256.77 degrees
[Motor 1] Sawtooth Wave Set Position: 267.17°
[Motor 1] Calculated Error: 10.39° (Set Position: 267.17°, Current Angle: 256.77°)
[PIDController] Error: 10.39, Integral: -476.47, Derivative: -42.81, Control Signal: -25.34
[Motor] Set speed to 25.33991023583377% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 256.77°, Error: 10.39°, Control Signal: 25.34%, Set Position: 267.17°
[ADCReader] Channel 2 ADC Value: 72, Average: 45.27 degrees
[Motor 3] Mapped ADC Average to 45.27 degrees
[Motor 3] Sawtooth Wave Set Position: 267.30°
[Motor 3] Calculated Error: -107.97° (Set Position: 267.30°, Current Angle: 45.27°)
[PIDController] Error: -107.97, Integral: 724.01, Derivative: 292.79, Control Signal: 44.36
[Motor] Set speed to 44.36168141245772% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 45.27°, Error: -107.97°, Control Signal: 44.36%, Set Position: 267.30°
[ADCReader] Channel 0 ADC Value: 855, Average: 262.90 degrees
[Motor 1] Mapped ADC Average to 262.90 degrees
[Motor 1] Sawtooth Wave Set Position: 270.53°
[Motor 1] Calculated Error: 7.64° (Set Position: 270.53°, Current Angle: 262.90°)
[PIDController] Error: 7.64, Integral: -476.08, Derivative: -54.09, Control Signal: -26.05
[Motor] Set speed to 26.04991625549611% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 262.90°, Error: 7.64°, Control Signal: 26.05%, Set Position: 270.53°
[ADCReader] Channel 2 ADC Value: 38, Average: 33.99 degrees
[Motor 3] Mapped ADC Average to 33.99 degrees
[Motor 3] Sawtooth Wave Set Position: 270.67°
[Motor 3] Calculated Error: -93.32° (Set Position: 270.67°, Current Angle: 33.99°)
[PIDController] Error: -93.32, Integral: 719.26, Derivative: 287.52, Control Signal: 44.74
[Motor] Set speed to 44.73930362685231% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 33.99°, Error: -93.32°, Control Signal: 44.74%, Set Position: 270.67°
[ADCReader] Channel 0 ADC Value: 880, Average: 269.35 degrees
[Motor 1] Mapped ADC Average to 269.35 degrees
[Motor 1] Sawtooth Wave Set Position: 273.90°
[Motor 1] Calculated Error: 4.55° (Set Position: 273.90°, Current Angle: 269.35°)
[PIDController] Error: 4.55, Integral: -475.84, Derivative: -60.45, Control Signal: -26.54
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 2 ADC Value: 5, Average: 23.03 degrees
[Motor 3] Mapped ADC Average to 23.03 degrees
[Motor 3] Sawtooth Wave Set Position: 274.03°
[Motor 3] Calculated Error: -79.00° (Set Position: 274.03°, Current Angle: 23.03°)
[PIDController] Error: -79.00, Integral: 715.22, Derivative: 280.55, Control Signal: 45.05
[Motor] Set speed to 45.048403735482886% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 23.03°, Error: -79.00°, Control Signal: 45.05%, Set Position: 274.03°
[ADCReader] Channel 0 ADC Value: 894, Average: 275.81 degrees
[Motor 1] Mapped ADC Average to 275.81 degrees
[Motor 1] Sawtooth Wave Set Position: 277.27°
[Motor 1] Calculated Error: 1.46° (Set Position: 277.27°, Current Angle: 275.81°)
[PIDController] Error: 1.46, Integral: -475.77, Derivative: -60.63, Control Signal: -26.73
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 2 ADC Value: 2, Average: 14.22 degrees
[Motor 3] Mapped ADC Average to 14.22 degrees
[Motor 3] Sawtooth Wave Set Position: 277.40°
[Motor 3] Calculated Error: -66.83° (Set Position: 277.40°, Current Angle: 14.22°)
[PIDController] Error: -66.83, Integral: 711.82, Derivative: 238.99, Control Signal: 43.53
[Motor] Set speed to 43.530697872917244% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 14.22°, Error: -66.83°, Control Signal: 43.53%, Set Position: 277.40°
[ADCReader] Channel 0 ADC Value: 900, Average: 281.37 degrees
[Motor 1] Mapped ADC Average to 281.37 degrees
[Motor 1] Sawtooth Wave Set Position: 280.63°
[Motor 1] Calculated Error: -0.74° (Set Position: 280.63°, Current Angle: 281.37°)
[PIDController] Error: -0.74, Integral: -475.81, Derivative: -43.06, Control Signal: -25.99
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 2 ADC Value: 7, Average: 7.97 degrees
[Motor 3] Mapped ADC Average to 7.97 degrees
[Motor 3] Sawtooth Wave Set Position: 280.76°
[Motor 3] Calculated Error: -57.21° (Set Position: 280.76°, Current Angle: 7.97°)
[PIDController] Error: -57.21, Integral: 708.91, Derivative: 189.02, Control Signal: 41.46
[Motor] Set speed to 41.46406861908044% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 7.97°, Error: -57.21°, Control Signal: 41.46%, Set Position: 280.76°
[ADCReader] Channel 0 ADC Value: 904, Average: 286.01 degrees
[Motor 1] Mapped ADC Average to 286.01 degrees
[Motor 1] Sawtooth Wave Set Position: 284.00°
[Motor 1] Calculated Error: -2.01° (Set Position: 284.00°, Current Angle: 286.01°)
[PIDController] Error: -2.01, Integral: -475.91, Derivative: -25.06, Control Signal: -25.17
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 2 ADC Value: 1010, Average: 26.77 degrees
[Motor 3] Mapped ADC Average to 26.77 degrees
[Motor 3] Sawtooth Wave Set Position: 284.13°
[Motor 3] Calculated Error: -72.64° (Set Position: 284.13°, Current Angle: 26.77°)
[PIDController] Error: -72.64, Integral: 705.20, Derivative: -302.61, Control Signal: 15.77
[Motor] Set speed to 31.464068619080443% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 26.77°, Error: -72.64°, Control Signal: 31.46%, Set Position: 284.13°
[ADCReader] Channel 0 ADC Value: 903, Average: 289.10 degrees
[Motor 1] Mapped ADC Average to 289.10 degrees
[Motor 1] Sawtooth Wave Set Position: 287.36°
[Motor 1] Calculated Error: -1.73° (Set Position: 287.36°, Current Angle: 289.10°)
[PIDController] Error: -1.73, Integral: -476.00, Derivative: 5.40, Control Signal: -23.63
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 2 ADC Value: 980, Average: 15.50 degrees
[Motor 3] Mapped ADC Average to 15.50 degrees
[Motor 3] Sawtooth Wave Set Position: 287.50°
[Motor 3] Calculated Error: -58.00° (Set Position: 287.50°, Current Angle: 15.50°)
[PIDController] Error: -58.00, Integral: 702.25, Derivative: 287.44, Control Signal: 46.00
[Motor] Set speed to 41.46406861908044% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 15.50°, Error: -58.00°, Control Signal: 41.46%, Set Position: 287.50°
[ADCReader] Channel 0 ADC Value: 904, Average: 290.65 degrees
[Motor 1] Mapped ADC Average to 290.65 degrees
[Motor 1] Sawtooth Wave Set Position: 290.73°
[Motor 1] Calculated Error: 0.08° (Set Position: 290.73°, Current Angle: 290.65°)
[PIDController] Error: 0.08, Integral: -475.99, Derivative: 35.65, Control Signal: -22.01
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 2 ADC Value: 952, Average: 4.25 degrees
[Motor 3] Mapped ADC Average to 4.25 degrees
[Motor 3] Sawtooth Wave Set Position: 290.86°
[Motor 3] Calculated Error: -43.39° (Set Position: 290.86°, Current Angle: 4.25°)
[PIDController] Error: -43.39, Integral: 700.04, Derivative: 286.73, Control Signal: 46.74
[Motor] Set speed to 46.73522626644731% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 4.25°, Error: -43.39°, Control Signal: 46.74%, Set Position: 290.86°
[ADCReader] Channel 0 ADC Value: 904, Average: 291.29 degrees
[Motor 1] Mapped ADC Average to 291.29 degrees
[Motor 1] Sawtooth Wave Set Position: 294.10°
[Motor 1] Calculated Error: 2.81° (Set Position: 294.10°, Current Angle: 291.29°)
[PIDController] Error: 2.81, Integral: -475.85, Derivative: 53.37, Control Signal: -20.96
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 2 ADC Value: 920, Average: 321.10 degrees
[Motor 3] Mapped ADC Average to 321.10 degrees
[Motor 3] Sawtooth Wave Set Position: 294.23°
[Motor 3] Calculated Error: -26.87° (Set Position: 294.23°, Current Angle: 321.10°)
[PIDController] Error: -26.87, Integral: 698.67, Derivative: 323.71, Control Signal: 49.51
[Motor] Set speed to 49.506776266212285% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 321.10°, Error: -26.87°, Control Signal: 49.51%, Set Position: 294.23°
[ADCReader] Channel 0 ADC Value: 905, Average: 291.61 degrees
[Motor 1] Mapped ADC Average to 291.61 degrees
[Motor 1] Sawtooth Wave Set Position: 297.46°
[Motor 1] Calculated Error: 5.85° (Set Position: 297.46°, Current Angle: 291.61°)
[PIDController] Error: 5.85, Integral: -475.55, Derivative: 59.66, Control Signal: -20.44
[Motor] Set speed to 20.443640840187047% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 291.61°, Error: 5.85°, Control Signal: 20.44%, Set Position: 297.46°
[ADCReader] Channel 2 ADC Value: 888, Average: 306.46 degrees
[Motor 3] Mapped ADC Average to 306.46 degrees
[Motor 3] Sawtooth Wave Set Position: 297.59°
[Motor 3] Calculated Error: -8.87° (Set Position: 297.59°, Current Angle: 306.46°)
[PIDController] Error: -8.87, Integral: 698.21, Derivative: 353.00, Control Signal: 52.03
[Motor] Set speed to 52.028939500082636% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 306.46°, Error: -8.87°, Control Signal: 52.03%, Set Position: 297.59°
[ADCReader] Channel 0 ADC Value: 904, Average: 291.61 degrees
[Motor 1] Mapped ADC Average to 291.61 degrees
[Motor 1] Sawtooth Wave Set Position: 300.83°
[Motor 1] Calculated Error: 9.22° (Set Position: 300.83°, Current Angle: 291.61°)
[PIDController] Error: 9.22, Integral: -475.08, Derivative: 65.85, Control Signal: -19.91
[Motor] Set speed to 19.90875556170079% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 291.61°, Error: 9.22°, Control Signal: 19.91%, Set Position: 300.83°
[ADCReader] Channel 2 ADC Value: 855, Average: 296.46 degrees
[Motor 3] Mapped ADC Average to 296.46 degrees
[Motor 3] Sawtooth Wave Set Position: 300.96°
[Motor 3] Calculated Error: 4.50° (Set Position: 300.96°, Current Angle: 296.46°)
[PIDController] Error: 4.50, Integral: 698.44, Derivative: 262.07, Control Signal: 48.30
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 0 ADC Value: 912, Average: 292.19 degrees
[Motor 1] Mapped ADC Average to 292.19 degrees
[Motor 1] Sawtooth Wave Set Position: 304.19°
[Motor 1] Calculated Error: 12.00° (Set Position: 304.19°, Current Angle: 292.19°)
[PIDController] Error: 12.00, Integral: -474.47, Derivative: 54.59, Control Signal: -20.27
[Motor] Set speed to 20.274006037342787% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 292.19°, Error: 12.00°, Control Signal: 20.27%, Set Position: 304.19°
[ADCReader] Channel 2 ADC Value: 821, Average: 286.20 degrees
[Motor 3] Mapped ADC Average to 286.20 degrees
[Motor 3] Sawtooth Wave Set Position: 304.33°
[Motor 3] Calculated Error: 18.13° (Set Position: 304.33°, Current Angle: 286.20°)
[PIDController] Error: 18.13, Integral: 699.37, Derivative: 267.81, Control Signal: 49.45
[Motor] Set speed to 49.44657968512893% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 286.20°, Error: 18.13°, Control Signal: 49.45%, Set Position: 304.33°
[ADCReader] Channel 0 ADC Value: 926, Average: 293.61 degrees
[Motor 1] Mapped ADC Average to 293.61 degrees
[Motor 1] Sawtooth Wave Set Position: 307.56°
[Motor 1] Calculated Error: 13.95° (Set Position: 307.56°, Current Angle: 293.61°)
[PIDController] Error: 13.95, Integral: -473.76, Derivative: 38.14, Control Signal: -20.94
[Motor] Set speed to 20.943885301384228% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 293.61°, Error: 13.95°, Control Signal: 20.94%, Set Position: 307.56°
[ADCReader] Channel 2 ADC Value: 791, Average: 275.80 degrees
[Motor 3] Mapped ADC Average to 275.80 degrees
[Motor 3] Sawtooth Wave Set Position: 307.69°
[Motor 3] Calculated Error: 31.89° (Set Position: 307.69°, Current Angle: 275.80°)
[PIDController] Error: 31.89, Integral: 700.99, Derivative: 269.56, Control Signal: 50.44
[Motor] Set speed to 50.44088865905333% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 275.80°, Error: 31.89°, Control Signal: 50.44%, Set Position: 307.69°
[ADCReader] Channel 0 ADC Value: 936, Average: 295.68 degrees
[Motor 1] Mapped ADC Average to 295.68 degrees
[Motor 1] Sawtooth Wave Set Position: 310.93°
[Motor 1] Calculated Error: 15.25° (Set Position: 310.93°, Current Angle: 295.68°)
[PIDController] Error: 15.25, Integral: -472.98, Derivative: 25.48, Control Signal: -21.46
[Motor] Set speed to 21.46012148568274% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 295.68°, Error: 15.25°, Control Signal: 21.46%, Set Position: 310.93°
[ADCReader] Channel 2 ADC Value: 758, Average: 265.35 degrees
[Motor 3] Mapped ADC Average to 265.35 degrees
[Motor 3] Sawtooth Wave Set Position: 311.06°
[Motor 3] Calculated Error: 45.71° (Set Position: 311.06°, Current Angle: 265.35°)
[PIDController] Error: 45.71, Integral: 703.32, Derivative: 270.96, Control Signal: 51.46
[Motor] Set speed to 51.456594559724024% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 265.35°, Error: 45.71°, Control Signal: 51.46%, Set Position: 311.06°
[ADCReader] Channel 0 ADC Value: 951, Average: 298.64 degrees
[Motor 1] Mapped ADC Average to 298.64 degrees
[Motor 1] Sawtooth Wave Set Position: 314.36°
[Motor 1] Calculated Error: 15.71° (Set Position: 314.36°, Current Angle: 298.64°)
[PIDController] Error: 15.71, Integral: -472.18, Derivative: 9.08, Control Signal: -22.21
[Motor] Set speed to 22.21166315443431% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 298.64°, Error: 15.71°, Control Signal: 22.21%, Set Position: 314.36°
[ADCReader] Channel 2 ADC Value: 726, Average: 254.90 degrees
[Motor 3] Mapped ADC Average to 254.90 degrees
[Motor 3] Sawtooth Wave Set Position: 314.42°
[Motor 3] Calculated Error: 59.52° (Set Position: 314.42°, Current Angle: 254.90°)
[PIDController] Error: 59.52, Integral: 706.38, Derivative: 269.46, Control Signal: 52.36
[Motor] Set speed to 52.36328198716328% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 254.90°, Error: 59.52°, Control Signal: 52.36%, Set Position: 314.42°
[ADCReader] Channel 0 ADC Value: 965, Average: 302.58 degrees
[Motor 1] Mapped ADC Average to 302.58 degrees
[Motor 1] Sawtooth Wave Set Position: 317.72°
[Motor 1] Calculated Error: 15.14° (Set Position: 317.72°, Current Angle: 302.58°)
[PIDController] Error: 15.14, Integral: -471.40, Derivative: -11.15, Control Signal: -23.22
[Motor] Set speed to 23.21914012866962% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 302.58°, Error: 15.14°, Control Signal: 23.22%, Set Position: 317.72°
[ADCReader] Channel 2 ADC Value: 690, Average: 244.27 degrees
[Motor 3] Mapped ADC Average to 244.27 degrees
[Motor 3] Sawtooth Wave Set Position: 317.79°
[Motor 3] Calculated Error: 73.52° (Set Position: 317.79°, Current Angle: 244.27°)
[PIDController] Error: 73.52, Integral: 710.13, Derivative: 273.84, Control Signal: 53.61
[Motor] Set speed to 53.61004947349004% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 244.27°, Error: 73.52°, Control Signal: 53.61%, Set Position: 317.79°
[ADCReader] Channel 0 ADC Value: 985, Average: 307.29 degrees
[Motor 1] Mapped ADC Average to 307.29 degrees
[Motor 1] Sawtooth Wave Set Position: 321.09°
[Motor 1] Calculated Error: 13.80° (Set Position: 321.09°, Current Angle: 307.29°)
[PIDController] Error: 13.80, Integral: -470.70, Derivative: -26.21, Control Signal: -24.02
[Motor] Set speed to 24.016848609543782% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 307.29°, Error: 13.80°, Control Signal: 24.02%, Set Position: 321.09°
[ADCReader] Channel 2 ADC Value: 653, Average: 233.43 degrees
[Motor 3] Mapped ADC Average to 233.43 degrees
[Motor 3] Sawtooth Wave Set Position: 321.22°
[Motor 3] Calculated Error: 87.79° (Set Position: 321.22°, Current Angle: 233.43°)
[PIDController] Error: 87.79, Integral: 714.62, Derivative: 279.42, Control Signal: 54.97
[Motor] Set speed to 54.96912920467828% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 233.43°, Error: 87.79°, Control Signal: 54.97%, Set Position: 321.22°
[ADCReader] Channel 0 ADC Value: 998, Average: 311.93 degrees
[Motor 1] Mapped ADC Average to 311.93 degrees
[Motor 1] Sawtooth Wave Set Position: 324.46°
[Motor 1] Calculated Error: 12.52° (Set Position: 324.46°, Current Angle: 311.93°)
[PIDController] Error: 12.52, Integral: -470.05, Derivative: -25.04, Control Signal: -24.00
[Motor] Set speed to 24.003517959261544% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving reverse: Potentiometer Value: 311.93°, Error: 12.52°, Control Signal: 24.00%, Set Position: 324.46°
[ADCReader] Channel 2 ADC Value: 622, Average: 222.52 degrees
[Motor 3] Mapped ADC Average to 222.52 degrees
[Motor 3] Sawtooth Wave Set Position: 324.59°
[Motor 3] Calculated Error: 102.07° (Set Position: 324.59°, Current Angle: 222.52°)
[PIDController] Error: 102.07, Integral: 719.84, Derivative: 278.92, Control Signal: 56.06
[Motor] Set speed to 56.062648298876596% duty cycle
[Motor] Moving forward
[Motor 3] Moving forward: Potentiometer Value: 222.52°, Error: 102.07°, Control Signal: 56.06%, Set Position: 324.59°

"""

# Initialize data storage
data = {
    "Motor 1": {"Set Position": [], "Current Angle": [], "Control Signal": [], "Error": []},
    "Motor 3": {"Set Position": [], "Current Angle": [], "Control Signal": [], "Error": []}
}

# Regular expressions to capture the needed values
motor_regex = re.compile(r'\[Motor (\d+)\] Calculated Error: ([\d\.\-]+)° \(Set Position: ([\d\.\-]+)°, Current Angle: ([\d\.\-]+)°\)')
control_signal_regex = re.compile(r'\[Motor (\d+)\] Moving motor: Potentiometer Value: [\d\.\-]+°, Error: [\d\.\-]+°, Control Signal: ([\d\.\-]+)%')

# Parse the data
for line in log_data.splitlines():
    motor_match = motor_regex.search(line)
    control_signal_match = control_signal_regex.search(line)
    
    if motor_match:
        motor_id = f"Motor {motor_match.group(1)}"
        error = float(motor_match.group(2))
        set_position = float(motor_match.group(3))
        current_angle = float(motor_match.group(4))
        data[motor_id]["Error"].append(error)
        data[motor_id]["Set Position"].append(set_position)
        data[motor_id]["Current Angle"].append(current_angle)
    
    if control_signal_match:
        motor_id = f"Motor {control_signal_match.group(1)}"
        control_signal = float(control_signal_match.group(2))
        data[motor_id]["Control Signal"].append(control_signal)

# Plotting
for motor, metrics in data.items():
    plt.figure(figsize=(10, 6))
    plt.plot(metrics["Set Position"], label="Set Position")
    plt.plot(metrics["Current Angle"], label="Current Angle")
    plt.plot(metrics["Control Signal"], label="Control Signal")
    plt.plot(metrics["Error"], label="Error")
    plt.title(f"{motor} Metrics")
    plt.xlabel("Time (arbitrary units)")
    plt.ylabel("Degrees / Percentage")
    plt.legend()
    plt.show()
