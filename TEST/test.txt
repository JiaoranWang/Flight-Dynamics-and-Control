cd C:\Program Files\FlightGear\bin\win64

SET FG_ROOT=C:\Program Files\FlightGear\data
SET FG_SCENERY=C:\Program Files\FlightGear\data\Scenery
fgfs  --fdm=null --enable-auto-coordination --native-fdm=socket,in,30,localhost,5502,udp --fog-disable --altitude=8000 --heading=0 --offset-distance=0 --offset-azimuth=0