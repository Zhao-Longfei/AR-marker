# AR-marker
Use AR markers to test pose accuracy,include eye to hand calibration
step1:Load csv files

step2:use data of AR marker and Potentiometer to do eye-to-hand calibration(file handeye)
calibrate their alignment by solving AX=XB.

step3:apply calibrated rotation to AR data
A=XBX^(-1)

Then, we can directly compare the XYZ euler angles of AR and potentiometer

