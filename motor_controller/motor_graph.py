import re
import matplotlib.pyplot as plt

# Sample log data
log_data = """
1.44%
[ADCReader] Channel 2 ADC Value: 998, Average: 21.43 degrees
[Motor 3] Mapped ADC Average to 21.43 degrees
[Motor 3] Calculated Error: 96.25° (Set Position: 117.67800000000001°, Current Angle: 21.43°)
[PIDController] Error: 96.25, Integral: 954.14, Derivative: -245.35, Control Signal: 88.92
[Motor] Set speed to 90% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 21.43°, Error: 96.25°, Control Signal: 90.00%
[ADCReader] Channel 0 ADC Value: 499, Average: 143.28 degrees
[Motor 1] Mapped ADC Average to 143.28 degrees
[Motor 1] Calculated Error: -23.03° (Set Position: 120.25200000000001°, Current Angle: 143.28°)
[PIDController] Error: -23.03, Integral: -647.31, Derivative: -113.00, Control Signal: -71.76
[Motor] Set speed to 71.76288265821654% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 143.28°, Error: -23.03°, Control Signal: -71.76%
[ADCReader] Channel 2 ADC Value: 961, Average: 7.43 degrees
[Motor 3] Mapped ADC Average to 7.43 degrees
[Motor 3] Calculated Error: 113.62° (Set Position: 121.04400000000001°, Current Angle: 7.43°)
[PIDController] Error: 113.62, Integral: 959.93, Derivative: 341.30, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 7.43°, Error: 113.62°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 533, Average: 152.23 degrees
[Motor 1] Mapped ADC Average to 152.23 degrees
[Motor 1] Calculated Error: -28.61° (Set Position: 123.61800000000001°, Current Angle: 152.23°)
[PIDController] Error: -28.61, Integral: -648.78, Derivative: -109.40, Control Signal: -72.06
[Motor] Set speed to 72.06398862548446% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 152.23°, Error: -28.61°, Control Signal: -72.06%
[ADCReader] Channel 2 ADC Value: 924, Average: 323.69 degrees
[Motor 3] Mapped ADC Average to 323.69 degrees
[Motor 3] Calculated Error: 130.65° (Set Position: 124.34400000000001°, Current Angle: 323.69°)
[PIDController] Error: 130.65, Integral: 966.58, Derivative: 334.36, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 323.69°, Error: 130.65°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 565, Average: 161.47 degrees
[Motor 1] Mapped ADC Average to 161.47 degrees
[Motor 1] Calculated Error: -34.49° (Set Position: 126.98400000000001°, Current Angle: 161.47°)
[PIDController] Error: -34.49, Integral: -650.54, Derivative: -115.15, Control Signal: -72.88
[Motor] Set speed to 72.88014611419733% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 161.47°, Error: -34.49°, Control Signal: -72.88%
[ADCReader] Channel 2 ADC Value: 882, Average: 308.57 degrees
[Motor 3] Mapped ADC Average to 308.57 degrees
[Motor 3] Calculated Error: 149.21° (Set Position: 127.77600000000001°, Current Angle: 308.57°)
[PIDController] Error: 149.21, Integral: 974.20, Derivative: 363.27, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 308.57°, Error: 149.21°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 599, Average: 171.74 degrees
[Motor 1] Mapped ADC Average to 171.74 degrees
[Motor 1] Calculated Error: -41.39° (Set Position: 130.35°, Current Angle: 171.74°)
[PIDController] Error: -41.39, Integral: -652.64, Derivative: -135.60, Control Signal: -74.53
[Motor] Set speed to 74.5279008489602% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 171.74°, Error: -41.39°, Control Signal: -74.53%
[ADCReader] Channel 2 ADC Value: 841, Average: 297.18 degrees
[Motor 3] Mapped ADC Average to 297.18 degrees
[Motor 3] Calculated Error: 163.89° (Set Position: 131.076°, Current Angle: 297.18°)
[PIDController] Error: 163.89, Integral: 982.54, Derivative: 288.63, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 297.18°, Error: 163.89°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 633, Average: 182.51 degrees
[Motor 1] Mapped ADC Average to 182.51 degrees
[Motor 1] Calculated Error: -48.80° (Set Position: 133.716°, Current Angle: 182.51°)
[PIDController] Error: -48.80, Integral: -655.13, Derivative: -145.60, Control Signal: -75.72
[Motor] Set speed to 75.72032956275635% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 182.51°, Error: -48.80°, Control Signal: -75.72%
[ADCReader] Channel 2 ADC Value: 797, Average: 284.22 degrees
[Motor 3] Mapped ADC Average to 284.22 degrees
[Motor 3] Calculated Error: -149.77° (Set Position: 134.442°, Current Angle: 284.22°)
[PIDController] Error: -149.77, Integral: 974.92, Derivative: -6163.42, Control Signal: -100.00
[Motor] Set speed to 90% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 284.22°, Error: -149.77°, Control Signal: 90.00%
[ADCReader] Channel 0 ADC Value: 669, Average: 193.47 degrees
[Motor 1] Mapped ADC Average to 193.47 degrees
[Motor 1] Calculated Error: -56.39° (Set Position: 137.082°, Current Angle: 193.47°)
[PIDController] Error: -56.39, Integral: -658.00, Derivative: -148.88, Control Signal: -76.63
[Motor] Set speed to 76.62794385170233% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 193.47°, Error: -56.39°, Control Signal: -76.63%
[ADCReader] Channel 2 ADC Value: 756, Average: 270.97 degrees
[Motor 3] Mapped ADC Average to 270.97 degrees
[Motor 3] Calculated Error: -133.16° (Set Position: 137.808°, Current Angle: 270.97°)
[PIDController] Error: -133.16, Integral: 968.14, Derivative: 326.41, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 270.97°, Error: -133.16°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 704, Average: 204.51 degrees
[Motor 1] Mapped ADC Average to 204.51 degrees
[Motor 1] Calculated Error: -64.06° (Set Position: 140.448°, Current Angle: 204.51°)
[PIDController] Error: -64.06, Integral: -661.27, Derivative: -150.36, Control Signal: -77.49
[Motor] Set speed to 77.48901222336775% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 204.51°, Error: -64.06°, Control Signal: -77.49%
[ADCReader] Channel 2 ADC Value: 716, Average: 257.54 degrees
[Motor 3] Mapped ADC Average to 257.54 degrees
[Motor 3] Calculated Error: -116.37° (Set Position: 141.174°, Current Angle: 257.54°)
[PIDController] Error: -116.37, Integral: 962.21, Derivative: 329.61, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 257.54°, Error: -116.37°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 741, Average: 215.86 degrees
[Motor 1] Mapped ADC Average to 215.86 degrees
[Motor 1] Calculated Error: -72.05° (Set Position: 143.814°, Current Angle: 215.86°)
[PIDController] Error: -72.05, Integral: -664.94, Derivative: -156.86, Control Signal: -78.66
[Motor] Set speed to 78.65993569830552% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 215.86°, Error: -72.05°, Control Signal: -78.66%
[ADCReader] Channel 2 ADC Value: 678, Average: 244.36 degrees
[Motor 3] Mapped ADC Average to 244.36 degrees
[Motor 3] Calculated Error: -99.82° (Set Position: 144.54000000000002°, Current Angle: 244.36°)
[PIDController] Error: -99.82, Integral: 957.13, Derivative: 325.02, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 244.36°, Error: -99.82°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 777, Average: 227.35 degrees
[Motor 1] Mapped ADC Average to 227.35 degrees
[Motor 1] Calculated Error: -80.17° (Set Position: 147.18°, Current Angle: 227.35°)
[PIDController] Error: -80.17, Integral: -669.02, Derivative: -159.69, Control Signal: -79.70
[Motor] Set speed to 79.69624299290577% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 227.35°, Error: -80.17°, Control Signal: -79.70%
[ADCReader] Channel 2 ADC Value: 639, Average: 231.34 degrees
[Motor 3] Mapped ADC Average to 231.34 degrees
[Motor 3] Calculated Error: -83.44° (Set Position: 147.906°, Current Angle: 231.34°)
[PIDController] Error: -83.44, Integral: 952.89, Derivative: 322.20, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 231.34°, Error: -83.44°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 815, Average: 239.09 degrees
[Motor 1] Mapped ADC Average to 239.09 degrees
[Motor 1] Calculated Error: -88.61° (Set Position: 150.48000000000002°, Current Angle: 239.09°)
[PIDController] Error: -88.61, Integral: -673.53, Derivative: -165.75, Control Signal: -80.96
[Motor] Set speed to 80.95686371699222% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 239.09°, Error: -88.61°, Control Signal: -80.96%
[ADCReader] Channel 2 ADC Value: 601, Average: 218.70 degrees
[Motor 3] Mapped ADC Average to 218.70 degrees
[Motor 3] Calculated Error: -67.43° (Set Position: 151.27200000000002°, Current Angle: 218.70°)
[PIDController] Error: -67.43, Integral: 949.46, Derivative: 314.14, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 218.70°, Error: -67.43°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 850, Average: 250.78 degrees
[Motor 1] Mapped ADC Average to 250.78 degrees
[Motor 1] Calculated Error: -96.86° (Set Position: 153.912°, Current Angle: 250.78°)
[PIDController] Error: -96.86, Integral: -678.48, Derivative: -161.59, Control Signal: -81.74
[Motor] Set speed to 81.7389467045569% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 250.78°, Error: -96.86°, Control Signal: -81.74%
[ADCReader] Channel 2 ADC Value: 563, Average: 206.26 degrees
[Motor 3] Mapped ADC Average to 206.26 degrees
[Motor 3] Calculated Error: -51.62° (Set Position: 154.638°, Current Angle: 206.26°)
[PIDController] Error: -51.62, Integral: 946.83, Derivative: 310.59, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 206.26°, Error: -51.62°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 886, Average: 262.52 degrees
[Motor 1] Mapped ADC Average to 262.52 degrees
[Motor 1] Calculated Error: -105.24° (Set Position: 157.27800000000002°, Current Angle: 262.52°)
[PIDController] Error: -105.24, Integral: -683.87, Derivative: -163.55, Control Signal: -82.88
[Motor] Set speed to 82.87885392765678% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 262.52°, Error: -105.24°, Control Signal: -82.88%
[ADCReader] Channel 2 ADC Value: 526, Average: 193.99 degrees
[Motor 3] Mapped ADC Average to 193.99 degrees
[Motor 3] Calculated Error: -35.99° (Set Position: 158.00400000000002°, Current Angle: 193.99°)
[PIDController] Error: -35.99, Integral: 944.99, Derivative: 306.64, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 193.99°, Error: -35.99°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 920, Average: 274.07 degrees
[Motor 1] Mapped ADC Average to 274.07 degrees
[Motor 1] Calculated Error: -113.43° (Set Position: 160.644°, Current Angle: 274.07°)
[PIDController] Error: -113.43, Integral: -689.70, Derivative: -159.09, Control Signal: -83.73
[Motor] Set speed to 83.73085812242667% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 274.07°, Error: -113.43°, Control Signal: -83.73%
[ADCReader] Channel 2 ADC Value: 486, Average: 181.62 degrees
[Motor 3] Mapped ADC Average to 181.62 degrees
[Motor 3] Calculated Error: -20.25° (Set Position: 161.37°, Current Angle: 181.62°)
[PIDController] Error: -20.25, Integral: 943.96, Derivative: 308.45, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 181.62°, Error: -20.25°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 960, Average: 285.86 degrees
[Motor 1] Mapped ADC Average to 285.86 degrees
[Motor 1] Calculated Error: -121.85° (Set Position: 164.01000000000002°, Current Angle: 285.86°)
[PIDController] Error: -121.85, Integral: -695.92, Derivative: -165.01, Control Signal: -85.15
[Motor] Set speed to 85.15382674562733% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 285.86°, Error: -121.85°, Control Signal: -85.15%
[ADCReader] Channel 2 ADC Value: 450, Average: 169.42 degrees
[Motor 3] Mapped ADC Average to 169.42 degrees
[Motor 3] Calculated Error: -4.68° (Set Position: 164.73600000000002°, Current Angle: 169.42°)
[PIDController] Error: -4.68, Integral: 943.72, Derivative: 305.46, Control Signal: 100.00
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 0 ADC Value: 992, Average: 297.29 degrees
[Motor 1] Mapped ADC Average to 297.29 degrees
[Motor 1] Calculated Error: -129.92° (Set Position: 167.376°, Current Angle: 297.29°)
[PIDController] Error: -129.92, Integral: -702.55, Derivative: -158.19, Control Signal: -85.96
[Motor] Set speed to 85.95924268308266% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 297.29°, Error: -129.92°, Control Signal: -85.96%
[ADCReader] Channel 2 ADC Value: 414, Average: 157.35 degrees
[Motor 3] Mapped ADC Average to 157.35 degrees
[Motor 3] Calculated Error: 10.76° (Set Position: 168.102°, Current Angle: 157.35°)
[PIDController] Error: 10.76, Integral: 944.27, Derivative: 303.19, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 157.35°, Error: 10.76°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 1019, Average: 308.22 degrees
[Motor 1] Mapped ADC Average to 308.22 degrees
[Motor 1] Calculated Error: -137.48° (Set Position: 170.74200000000002°, Current Angle: 308.22°)
[PIDController] Error: -137.48, Integral: -709.55, Derivative: -148.30, Control Signal: -86.62
[Motor] Set speed to 86.6191631362815% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 308.22°, Error: -137.48°, Control Signal: -86.62%
[ADCReader] Channel 2 ADC Value: 378, Average: 145.41 degrees
[Motor 3] Mapped ADC Average to 145.41 degrees
[Motor 3] Calculated Error: 26.06° (Set Position: 171.46800000000002°, Current Angle: 145.41°)
[PIDController] Error: 26.06, Integral: 945.59, Derivative: 300.71, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 145.41°, Error: 26.06°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 430, Average: 312.13 degrees
[Motor 1] Mapped ADC Average to 312.13 degrees
[Motor 1] Calculated Error: -138.02° (Set Position: 174.108°, Current Angle: 312.13°)
[PIDController] Error: -138.02, Integral: -716.59, Derivative: -10.72, Control Signal: -80.48
[Motor] Set speed to 80.47694425947006% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 312.13°, Error: -138.02°, Control Signal: -80.48%
[ADCReader] Channel 2 ADC Value: 340, Average: 133.42 degrees
[Motor 3] Mapped ADC Average to 133.42 degrees
[Motor 3] Calculated Error: 41.41° (Set Position: 174.834°, Current Angle: 133.42°)
[PIDController] Error: 41.41, Integral: 947.70, Derivative: 301.29, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 133.42°, Error: 41.41°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 0, Average: 3.11 degrees
[Motor 1] Mapped ADC Average to 3.11 degrees
[Motor 1] Calculated Error: -155.63° (Set Position: 177.47400000000002°, Current Angle: 3.11°)
[PIDController] Error: -155.63, Integral: -724.53, Derivative: -345.33, Control Signal: -99.06
[Motor] Set speed to 90.47694425947006% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 3.11°, Error: -155.63°, Control Signal: -90.48%
[ADCReader] Channel 2 ADC Value: 300, Average: 121.43 degrees
[Motor 3] Mapped ADC Average to 121.43 degrees
[Motor 3] Calculated Error: 56.70° (Set Position: 178.13400000000001°, Current Angle: 121.43°)
[PIDController] Error: 56.70, Integral: 950.59, Derivative: 300.07, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 121.43°, Error: 56.70°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 30, Average: 23.32 degrees
[Motor 1] Mapped ADC Average to 23.32 degrees
[Motor 1] Calculated Error: 157.52° (Set Position: 180.84°, Current Angle: 23.32°)
[PIDController] Error: 157.52, Integral: -716.50, Derivative: 6145.65, Control Signal: 100.00
[Motor] Set speed to 80.47694425947006% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 23.32°, Error: 157.52°, Control Signal: -80.48%
[ADCReader] Channel 2 ADC Value: 256, Average: 108.93 degrees
[Motor 3] Mapped ADC Average to 108.93 degrees
[Motor 3] Calculated Error: 72.57° (Set Position: 181.5°, Current Angle: 108.93°)
[PIDController] Error: 72.57, Integral: 954.29, Derivative: 311.44, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 108.93°, Error: 72.57°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 65, Average: 12.43 degrees
[Motor 1] Mapped ADC Average to 12.43 degrees
[Motor 1] Calculated Error: -158.23° (Set Position: 184.20600000000002°, Current Angle: 12.43°)
[PIDController] Error: -158.23, Integral: -724.57, Derivative: -6193.13, Control Signal: -100.00
[Motor] Set speed to 90.47694425947006% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 12.43°, Error: -158.23°, Control Signal: -90.48%
[ADCReader] Channel 2 ADC Value: 213, Average: 95.96 degrees
[Motor 3] Mapped ADC Average to 95.96 degrees
[Motor 3] Calculated Error: 88.91° (Set Position: 184.866°, Current Angle: 95.96°)
[PIDController] Error: 88.91, Integral: 958.82, Derivative: 320.77, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 95.96°, Error: 88.91°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 101, Average: 29.80 degrees
[Motor 1] Mapped ADC Average to 29.80 degrees
[Motor 1] Calculated Error: 157.77° (Set Position: 187.572°, Current Angle: 29.80°)
[PIDController] Error: 157.77, Integral: -716.53, Derivative: 6202.87, Control Signal: 100.00
[Motor] Set speed to 80.47694425947006% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 29.80°, Error: 157.77°, Control Signal: -80.48%
[ADCReader] Channel 2 ADC Value: 172, Average: 82.65 degrees
[Motor 3] Mapped ADC Average to 82.65 degrees
[Motor 3] Calculated Error: 105.59° (Set Position: 188.232°, Current Angle: 82.65°)
[PIDController] Error: 105.59, Integral: 964.19, Derivative: 327.80, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 82.65°, Error: 105.59°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 140, Average: 21.65 degrees
[Motor 1] Mapped ADC Average to 21.65 degrees
[Motor 1] Calculated Error: -160.72° (Set Position: 190.93800000000002°, Current Angle: 21.65°)
[PIDController] Error: -160.72, Integral: -724.72, Derivative: -6247.31, Control Signal: -100.00
[Motor] Set speed to 90.47694425947006% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 21.65°, Error: -160.72°, Control Signal: -90.48%
[ADCReader] Channel 2 ADC Value: 132, Average: 69.21 degrees
[Motor 3] Mapped ADC Average to 69.21 degrees
[Motor 3] Calculated Error: 122.39° (Set Position: 191.598°, Current Angle: 69.21°)
[PIDController] Error: 122.39, Integral: 970.43, Derivative: 329.94, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 69.21°, Error: 122.39°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 179, Average: 33.21 degrees
[Motor 1] Mapped ADC Average to 33.21 degrees
[Motor 1] Calculated Error: 161.09° (Set Position: 194.304°, Current Angle: 33.21°)
[PIDController] Error: 161.09, Integral: -716.51, Derivative: 6310.56, Control Signal: 100.00
[Motor] Set speed to 80.47694425947006% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 33.21°, Error: 161.09°, Control Signal: -80.48%
[ADCReader] Channel 2 ADC Value: 92, Average: 55.79 degrees
[Motor 3] Mapped ADC Average to 55.79 degrees
[Motor 3] Calculated Error: 139.24° (Set Position: 195.03°, Current Angle: 55.79°)
[PIDController] Error: 139.24, Integral: 977.60, Derivative: 326.88, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 55.79°, Error: 139.24°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 220, Average: 45.47 degrees
[Motor 1] Mapped ADC Average to 45.47 degrees
[Motor 1] Calculated Error: 152.20° (Set Position: 197.67000000000002°, Current Angle: 45.47°)
[PIDController] Error: 152.20, Integral: -708.76, Derivative: -174.61, Control Signal: -70.47
[Motor] Set speed to 70.47694425947006% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 45.47°, Error: 152.20°, Control Signal: -70.48%
[ADCReader] Channel 2 ADC Value: 53, Average: 42.70 degrees
[Motor 3] Mapped ADC Average to 42.70 degrees
[Motor 3] Calculated Error: 155.69° (Set Position: 198.39600000000002°, Current Angle: 42.70°)
[PIDController] Error: 155.69, Integral: 985.52, Derivative: 323.41, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 42.70°, Error: 155.69°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 257, Average: 57.87 degrees
[Motor 1] Mapped ADC Average to 57.87 degrees
[Motor 1] Calculated Error: 143.16° (Set Position: 201.036°, Current Angle: 57.87°)
[PIDController] Error: 143.16, Integral: -701.47, Derivative: -177.55, Control Signal: -70.43
[Motor] Set speed to 70.4348693608988% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 57.87°, Error: 143.16°, Control Signal: -70.43%
[ADCReader] Channel 2 ADC Value: 11, Average: 29.68 degrees
[Motor 3] Mapped ADC Average to 29.68 degrees
[Motor 3] Calculated Error: -157.92° (Set Position: 201.762°, Current Angle: 29.68°)
[PIDController] Error: -157.92, Integral: 977.48, Derivative: -6156.07, Control Signal: -100.00
[Motor] Set speed to 90% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 29.68°, Error: -157.92°, Control Signal: 90.00%
[ADCReader] Channel 0 ADC Value: 295, Average: 70.40 degrees
[Motor 1] Mapped ADC Average to 70.40 degrees
[Motor 1] Calculated Error: 134.01° (Set Position: 204.40200000000002°, Current Angle: 70.40°)
[PIDController] Error: 134.01, Integral: -694.63, Derivative: -179.40, Control Signal: -70.39
[Motor] Set speed to 70.39280547035992% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 70.40°, Error: 134.01°, Control Signal: -70.39%
[ADCReader] Channel 2 ADC Value: 0, Average: 18.52 degrees
[Motor 3] Mapped ADC Average to 18.52 degrees
[Motor 3] Calculated Error: -143.40° (Set Position: 205.12800000000001°, Current Angle: 18.52°)
[PIDController] Error: -143.40, Integral: 970.16, Derivative: 284.67, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 18.52°, Error: -143.40°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 332, Average: 82.79 degrees
[Motor 1] Mapped ADC Average to 82.79 degrees
[Motor 1] Calculated Error: 124.98° (Set Position: 207.768°, Current Angle: 82.79°)
[PIDController] Error: 124.98, Integral: -688.25, Derivative: -176.86, Control Signal: -70.17
[Motor] Set speed to 70.16933177627101% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 82.79°, Error: 124.98°, Control Signal: -70.17%
[ADCReader] Channel 2 ADC Value: 1015, Average: 3.83 degrees
[Motor 3] Mapped ADC Average to 3.83 degrees
[Motor 3] Calculated Error: -125.34° (Set Position: 208.494°, Current Angle: 3.83°)
[PIDController] Error: -125.34, Integral: 963.76, Derivative: 353.39, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 3.83°, Error: -125.34°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 366, Average: 94.85 degrees
[Motor 1] Mapped ADC Average to 94.85 degrees
[Motor 1] Calculated Error: 116.29° (Set Position: 211.13400000000001°, Current Angle: 94.85°)
[PIDController] Error: 116.29, Integral: -682.33, Derivative: -170.70, Control Signal: -69.79
[Motor] Set speed to 69.79084333690797% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 94.85°, Error: 116.29°, Control Signal: -69.79%
[ADCReader] Channel 2 ADC Value: 994, Average: 19.87 degrees
[Motor 3] Mapped ADC Average to 19.87 degrees
[Motor 3] Calculated Error: -138.01° (Set Position: 211.86°, Current Angle: 19.87°)
[PIDController] Error: -138.01, Integral: 956.73, Derivative: -248.78, Control Signal: 74.95
[Motor] Set speed to 90% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 19.87°, Error: -138.01°, Control Signal: 90.00%
[ADCReader] Channel 0 ADC Value: 401, Average: 106.53 degrees
[Motor 1] Mapped ADC Average to 106.53 degrees
[Motor 1] Calculated Error: 107.97° (Set Position: 214.5°, Current Angle: 106.53°)
[PIDController] Error: 107.97, Integral: -676.82, Derivative: -163.03, Control Signal: -69.36
[Motor] Set speed to 69.35533910730835% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 106.53°, Error: 107.97°, Control Signal: -69.36%
[ADCReader] Channel 2 ADC Value: 958, Average: 6.02 degrees
[Motor 3] Mapped ADC Average to 6.02 degrees
[Motor 3] Calculated Error: -120.80° (Set Position: 215.226°, Current Angle: 6.02°)
[PIDController] Error: -120.80, Integral: 950.51, Derivative: 334.37, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 6.02°, Error: -120.80°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 435, Average: 118.01 degrees
[Motor 1] Mapped ADC Average to 118.01 degrees
[Motor 1] Calculated Error: 99.86° (Set Position: 217.866°, Current Angle: 118.01°)
[PIDController] Error: 99.86, Integral: -671.73, Derivative: -158.94, Control Signal: -69.13
[Motor] Set speed to 69.12828171569299% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 118.01°, Error: 99.86°, Control Signal: -69.13%
[ADCReader] Channel 2 ADC Value: 918, Average: 322.32 degrees
[Motor 3] Mapped ADC Average to 322.32 degrees
[Motor 3] Calculated Error: -103.72° (Set Position: 218.592°, Current Angle: 322.32°)
[PIDController] Error: -103.72, Integral: 945.23, Derivative: 335.30, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 322.32°, Error: -103.72°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 469, Average: 129.23 degrees
[Motor 1] Mapped ADC Average to 129.23 degrees
[Motor 1] Calculated Error: 92.00° (Set Position: 221.232°, Current Angle: 129.23°)
[PIDController] Error: 92.00, Integral: -667.04, Derivative: -154.17, Control Signal: -68.89
[Motor] Set speed to 68.89214593452746% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 129.23°, Error: 92.00°, Control Signal: -68.89%
[ADCReader] Channel 2 ADC Value: 877, Average: 307.28 degrees
[Motor 3] Mapped ADC Average to 307.28 degrees
[Motor 3] Calculated Error: -85.32° (Set Position: 221.958°, Current Angle: 307.28°)
[PIDController] Error: -85.32, Integral: 940.85, Derivative: 358.09, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 307.28°, Error: -85.32°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 504, Average: 140.32 degrees
[Motor 1] Mapped ADC Average to 140.32 degrees
[Motor 1] Calculated Error: 84.28° (Set Position: 224.598°, Current Angle: 140.32°)
[PIDController] Error: 84.28, Integral: -662.74, Derivative: -151.40, Control Signal: -68.79
[Motor] Set speed to 68.78721957679863% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 140.32°, Error: 84.28°, Control Signal: -68.79%
[ADCReader] Channel 2 ADC Value: 839, Average: 295.88 degrees
[Motor 3] Mapped ADC Average to 295.88 degrees
[Motor 3] Calculated Error: -70.55° (Set Position: 225.324°, Current Angle: 295.88°)
[PIDController] Error: -70.55, Integral: 937.25, Derivative: 289.66, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 295.88°, Error: -70.55°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 540, Average: 151.54 degrees
[Motor 1] Mapped ADC Average to 151.54 degrees
[Motor 1] Calculated Error: 76.42° (Set Position: 227.964°, Current Angle: 151.54°)
[PIDController] Error: 76.42, Integral: -658.83, Derivative: -153.63, Control Signal: -68.98
[Motor] Set speed to 68.9790896109289% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 151.54°, Error: 76.42°, Control Signal: -68.98%
[ADCReader] Channel 2 ADC Value: 792, Average: 282.86 degrees
[Motor 3] Mapped ADC Average to 282.86 degrees
[Motor 3] Calculated Error: -54.10° (Set Position: 228.756°, Current Angle: 282.86°)
[PIDController] Error: -54.10, Integral: 934.47, Derivative: 320.47, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 282.86°, Error: -54.10°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 569, Average: 162.40 degrees
[Motor 1] Mapped ADC Average to 162.40 degrees
[Motor 1] Calculated Error: 68.93° (Set Position: 231.33°, Current Angle: 162.40°)
[PIDController] Error: 68.93, Integral: -655.31, Derivative: -146.79, Control Signal: -68.74
[Motor] Set speed to 68.73500697434206% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 162.40°, Error: 68.93°, Control Signal: -68.74%
[ADCReader] Channel 2 ADC Value: 750, Average: 269.44 degrees
[Motor 3] Mapped ADC Average to 269.44 degrees
[Motor 3] Calculated Error: -37.32° (Set Position: 232.122°, Current Angle: 269.44°)
[PIDController] Error: -37.32, Integral: 932.55, Derivative: 326.18, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 269.44°, Error: -37.32°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 604, Average: 173.30 degrees
[Motor 1] Mapped ADC Average to 173.30 degrees
[Motor 1] Calculated Error: 61.40° (Set Position: 234.696°, Current Angle: 173.30°)
[PIDController] Error: 61.40, Integral: -652.18, Derivative: -147.87, Control Signal: -68.93
[Motor] Set speed to 68.92813540163368% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 173.30°, Error: 61.40°, Control Signal: -68.93%
[ADCReader] Channel 2 ADC Value: 707, Average: 255.82 degrees
[Motor 3] Mapped ADC Average to 255.82 degrees
[Motor 3] Calculated Error: -20.26° (Set Position: 235.554°, Current Angle: 255.82°)
[PIDController] Error: -20.26, Integral: 931.50, Derivative: 329.39, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 255.82°, Error: -20.26°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 636, Average: 184.07 degrees
[Motor 1] Mapped ADC Average to 184.07 degrees
[Motor 1] Calculated Error: 54.00° (Set Position: 238.062°, Current Angle: 184.07°)
[PIDController] Error: 54.00, Integral: -649.44, Derivative: -145.58, Control Signal: -68.98
[Motor] Set speed to 68.98296520893493% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 184.07°, Error: 54.00°, Control Signal: -68.98%
[ADCReader] Channel 2 ADC Value: 668, Average: 242.29 degrees
[Motor 3] Mapped ADC Average to 242.29 degrees
[Motor 3] Calculated Error: -3.37° (Set Position: 238.92000000000002°, Current Angle: 242.29°)
[PIDController] Error: -3.37, Integral: 931.33, Derivative: 332.43, Control Signal: 100.00
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 0 ADC Value: 671, Average: 194.83 degrees
[Motor 1] Mapped ADC Average to 194.83 degrees
[Motor 1] Calculated Error: 46.60° (Set Position: 241.428°, Current Angle: 194.83°)
[PIDController] Error: 46.60, Integral: -647.06, Derivative: -145.14, Control Signal: -69.17
[Motor] Set speed to 69.16731985127097% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 194.83°, Error: 46.60°, Control Signal: -69.17%
[ADCReader] Channel 2 ADC Value: 634, Average: 229.06 degrees
[Motor 3] Mapped ADC Average to 229.06 degrees
[Motor 3] Calculated Error: 13.22° (Set Position: 242.286°, Current Angle: 229.06°)
[PIDController] Error: 13.22, Integral: 932.01, Derivative: 322.46, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 229.06°, Error: 13.22°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 708, Average: 205.67 degrees
[Motor 1] Mapped ADC Average to 205.67 degrees
[Motor 1] Calculated Error: 39.13° (Set Position: 244.794°, Current Angle: 205.67°)
[PIDController] Error: 39.13, Integral: -645.07, Derivative: -146.44, Control Signal: -69.48
[Motor] Set speed to 69.48110187761888% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 205.67°, Error: 39.13°, Control Signal: -69.48%
[ADCReader] Channel 2 ADC Value: 600, Average: 216.68 degrees
[Motor 3] Mapped ADC Average to 216.68 degrees
[Motor 3] Calculated Error: 29.04° (Set Position: 245.71800000000002°, Current Angle: 216.68°)
[PIDController] Error: 29.04, Integral: 933.52, Derivative: 304.27, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 216.68°, Error: 29.04°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 742, Average: 216.83 degrees
[Motor 1] Mapped ADC Average to 216.83 degrees
[Motor 1] Calculated Error: 31.33° (Set Position: 248.16000000000003°, Current Angle: 216.83°)
[PIDController] Error: 31.33, Integral: -643.47, Derivative: -153.21, Control Signal: -70.13
[Motor] Set speed to 70.12796845581195% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 216.83°, Error: 31.33°, Control Signal: -70.13%
[ADCReader] Channel 2 ADC Value: 562, Average: 204.58 degrees
[Motor 3] Mapped ADC Average to 204.58 degrees
[Motor 3] Calculated Error: 44.51° (Set Position: 249.084°, Current Angle: 204.58°)
[PIDController] Error: 44.51, Integral: 935.80, Derivative: 302.65, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 204.58°, Error: 44.51°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 777, Average: 228.00 degrees
[Motor 1] Mapped ADC Average to 228.00 degrees
[Motor 1] Calculated Error: 23.52° (Set Position: 251.526°, Current Angle: 228.00°)
[PIDController] Error: 23.52, Integral: -642.28, Derivative: -153.56, Control Signal: -70.49
[Motor] Set speed to 70.49436525591327% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 228.00°, Error: 23.52°, Control Signal: -70.49%
[ADCReader] Channel 2 ADC Value: 525, Average: 192.85 degrees
[Motor 3] Mapped ADC Average to 192.85 degrees
[Motor 3] Calculated Error: 59.60° (Set Position: 252.45000000000002°, Current Angle: 192.85°)
[PIDController] Error: 59.60, Integral: 938.85, Derivative: 294.39, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 192.85°, Error: 59.60°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 814, Average: 239.48 degrees
[Motor 1] Mapped ADC Average to 239.48 degrees
[Motor 1] Calculated Error: 15.41° (Set Position: 254.89200000000002°, Current Angle: 239.48°)
[PIDController] Error: 15.41, Integral: -641.49, Derivative: -159.21, Control Signal: -71.18
[Motor] Set speed to 71.18498305592666% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 239.48°, Error: 15.41°, Control Signal: -71.18%
[ADCReader] Channel 2 ADC Value: 487, Average: 181.17 degrees
[Motor 3] Mapped ADC Average to 181.17 degrees
[Motor 3] Calculated Error: 74.71° (Set Position: 255.882°, Current Angle: 181.17°)
[PIDController] Error: 74.71, Integral: 942.66, Derivative: 296.13, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 181.17°, Error: 74.71°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 850, Average: 251.02 degrees
[Motor 1] Mapped ADC Average to 251.02 degrees
[Motor 1] Calculated Error: 7.23° (Set Position: 258.25800000000004°, Current Angle: 251.02°)
[PIDController] Error: 7.23, Integral: -641.12, Derivative: -160.46, Control Signal: -71.70
[Motor] Set speed to 71.70153006203526% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 251.02°, Error: 7.23°, Control Signal: -71.70%
[ADCReader] Channel 2 ADC Value: 448, Average: 169.17 degrees
[Motor 3] Mapped ADC Average to 169.17 degrees
[Motor 3] Calculated Error: 90.08° (Set Position: 259.248°, Current Angle: 169.17°)
[PIDController] Error: 90.08, Integral: 947.28, Derivative: 299.97, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 169.17°, Error: 90.08°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 882, Average: 262.27 degrees
[Motor 1] Mapped ADC Average to 262.27 degrees
[Motor 1] Calculated Error: -0.64° (Set Position: 261.624°, Current Angle: 262.27°)
[PIDController] Error: -0.64, Integral: -641.16, Derivative: -154.29, Control Signal: -71.87
[Motor] Set speed to 0% duty cycle
[Motor] Set speed to 0% duty cycle
[Motor] Stopped
[ADCReader] Channel 2 ADC Value: 408, Average: 156.79 degrees
[Motor 3] Mapped ADC Average to 156.79 degrees
[Motor 3] Calculated Error: 105.83° (Set Position: 262.61400000000003°, Current Angle: 156.79°)
[PIDController] Error: 105.83, Integral: 952.73, Derivative: 305.82, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 156.79°, Error: 105.83°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 910, Average: 273.12 degrees
[Motor 1] Mapped ADC Average to 273.12 degrees
[Motor 1] Calculated Error: -8.13° (Set Position: 264.99°, Current Angle: 273.12°)
[PIDController] Error: -8.13, Integral: -641.57, Derivative: -147.01, Control Signal: -72.00
[Motor] Set speed to 71.9955417394727% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 273.12°, Error: -8.13°, Control Signal: -72.00%
[ADCReader] Channel 2 ADC Value: 369, Average: 144.33 degrees
[Motor 3] Mapped ADC Average to 144.33 degrees
[Motor 3] Calculated Error: 121.65° (Set Position: 265.98°, Current Angle: 144.33°)
[PIDController] Error: 121.65, Integral: 958.93, Derivative: 310.37, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 144.33°, Error: 121.65°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 936, Average: 283.38 degrees
[Motor 1] Mapped ADC Average to 283.38 degrees
[Motor 1] Calculated Error: -15.02° (Set Position: 268.356°, Current Angle: 283.38°)
[PIDController] Error: -15.02, Integral: -642.34, Derivative: -134.75, Control Signal: -71.87
[Motor] Set speed to 71.8724652765091% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 283.38°, Error: -15.02°, Control Signal: -71.87%
[ADCReader] Channel 2 ADC Value: 328, Average: 131.62 degrees
[Motor 3] Mapped ADC Average to 131.62 degrees
[Motor 3] Calculated Error: 137.79° (Set Position: 269.41200000000003°, Current Angle: 131.62°)
[PIDController] Error: 137.79, Integral: 965.99, Derivative: 315.20, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 131.62°, Error: 137.79°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 972, Average: 293.54 degrees
[Motor 1] Mapped ADC Average to 293.54 degrees
[Motor 1] Calculated Error: -21.82° (Set Position: 271.72200000000004°, Current Angle: 293.54°)
[PIDController] Error: -21.82, Integral: -643.45, Derivative: -132.87, Control Signal: -72.30
[Motor] Set speed to 72.29828624284745% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 293.54°, Error: -21.82°, Control Signal: -72.30%
[ADCReader] Channel 2 ADC Value: 288, Average: 118.78 degrees
[Motor 3] Mapped ADC Average to 118.78 degrees
[Motor 3] Calculated Error: 154.00° (Set Position: 272.778°, Current Angle: 118.78°)
[PIDController] Error: 154.00, Integral: 973.88, Derivative: 316.34, Control Signal: 100.00
[Motor] Set speed to 100% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 118.78°, Error: 154.00°, Control Signal: 100.00%
[ADCReader] Channel 0 ADC Value: 1008, Average: 303.71 degrees
[Motor 1] Mapped ADC Average to 303.71 degrees
[Motor 1] Calculated Error: -28.63° (Set Position: 275.088°, Current Angle: 303.71°)
[PIDController] Error: -28.63, Integral: -644.92, Derivative: -133.43, Control Signal: -72.88
[Motor] Set speed to 72.8804395272816% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 303.71°, Error: -28.63°, Control Signal: -72.88%
[ADCReader] Channel 2 ADC Value: 246, Average: 105.75 degrees
[Motor 3] Mapped ADC Average to 105.75 degrees
[Motor 3] Calculated Error: -159.61° (Set Position: 276.144°, Current Angle: 105.75°)
[PIDController] Error: -159.61, Integral: 965.68, Derivative: -6103.49, Control Signal: -100.00
[Motor] Set speed to 90% duty cycle
[Motor] Moving forward
[Motor 3] Moving motor: Potentiometer Value: 105.75°, Error: -159.61°, Control Signal: 90.00%
[ADCReader] Channel 0 ADC Value: 1022, Average: 312.79 degrees
[Motor 1] Mapped ADC Average to 312.79 degrees
[Motor 1] Calculated Error: -34.34° (Set Position: 278.454°, Current Angle: 312.79°)
[PIDController] Error: -34.34, Integral: -646.67, Derivative: -112.05, Control Signal: -72.33
[Motor] Set speed to 72.32926165131754% duty cycle
[Motor] Moving in reverse
[Motor 1] Moving motor: Potentiometer Value: 312.79°, Error: -34.34°, Control Signal: -72.33%

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
