
# Created by Joe Sanford

print "start script"
import sys
import math


target_lat = 39.000000
target_lon = -119.000000
wind_dir = 310
math.radians(wind_dir)   
print  math.radians(wind_dir)  

wind_vel = 5
fall_time = 7

ground_course = 300
math.radians(ground_course)
print math.radians(ground_course)

if ground_course <= 90 and 180 < wind_dir <= 270:
	print "ground_course <= 90 and 180 < wind_dir <= 270 "
	theta = wind_dir - 180
	if theta  <= ground_course:
		print "Left cross tail wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat-lat_correction
		print ip_lat
		ip_lon = target_lon-lon_correction
		print ip_lon
	else: 
		theta = 270 - wind_dir
		print " right cross tail wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat + lat_correction
		print ip_lat
		ip_lon = target_lon-lon_correction
		print ip_lon
		

if wind_dir < 90 and ground_course < 90:
	print "wind_dir < 90 and ground_course < 90 "
	theta = wind_dir
	print theta
	if theta <= ground_course:
		print " right cross head wind"
		o = wind_vel*math.sin(theta)
		a = wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = a*fall_time*.00000275
		lon_correction = o*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat - lat_correction
		print ip_lat
		ip_lon = target_lon+lon_correction
		print ip_lon	 
	if theta > ground_course:	
		print " left cross head wind"
		o = wind_vel*math.sin(theta)
		a = wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = a*fall_time*.00000275
		lon_correction = o*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat - lat_correction
		print ip_lat
		ip_lon = target_lon+lon_correction
		print ip_lon	
		
if ground_course <= 90 and 270 < wind_dir <= 360:
	print "ground_course <= 90 and 270 < wind_dir <= 360  "
	theta = wind_dir - 270
	if wind_dir - 180 - ground_course >= 90:
		print "Left cross head wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		print  o
		print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		print lat_correction #Lat Correction West"
		print lon_correction #Lon Correction North"
		ip_lat = target_lat + lat_correction
		print ip_lat
		ip_lon = target_lon-lon_correction
		print ip_lon
	else: 
		print " left cross tail wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		print  o
		print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		print lat_correction #Lat Correction West"
		print lon_correction #Lon Correction North"
		ip_lat = target_lat - lat_correction
		print ip_lat
		ip_lon = target_lon-lon_correction
		print ip_lon
				
if ground_course <= 90 and 90 < wind_dir <= 180:
	print "ground_course <= 90 and 90 < wind_dir <= 180 "
	theta = wind_dir - 90
	if wind_dir - ground_course >= 90:
		print "Right cross tail wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		print  o
		print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		print lat_correction #Lat Correction West"
		print lon_correction #Lon Correction North"
		ip_lat = target_lat + lat_correction
		print ip_lat
		ip_lon = target_lon-lon_correction
		print ip_lon
	else: 
		print " Right cross head wind wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		print  o
		print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		print lat_correction #Lat Correction West"
		print lon_correction #Lon Correction North"
		ip_lat = target_lat + lat_correction
		print ip_lat
		ip_lon = target_lon-lon_correction
		print ip_lon
		
if 90 < ground_course <= 180 and wind_dir > 270:
	print "90 < ground_course <= 180 and wind_dir > 270  "
	theta = wind_dir - 90
	if theta + 90  <= ground_course:
		print "right cross tail wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat-lat_correction
		print ip_lat
		ip_lon = target_lon-lon_correction
		print ip_lon
	else: 
		theta = 180 - wind_dir
		print " left cross tail wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat - lat_correction
		print ip_lat
		ip_lon = target_lon-lon_correction
		print ip_lon
		
if 90 < ground_course <= 180 and  90 < wind_dir < 180:
	print " 90 < ground_course <= 180 and  90 < wind_dir < 180  "
	theta = wind_dir - 90
	if theta + 90  <= ground_course:
		print "Left cross head wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat-lat_correction
		print ip_lat
		ip_lon = target_lon-lon_correction
		print ip_lon
	else: 
		Theta = 180 - wind_dir
		print " right cross head wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat + lat_correction
		print ip_lat
		ip_lon = target_lon-lon_correction
		print ip_lon
		
if 90 < ground_course <= 180 and  0 < wind_dir < 90:
	print " 90 < ground_course <= 180 and  0 < wind_dir < 90"
	theta = 90 - wind_dir
	if theta + 90  <= ground_course:
		print "Left cross head wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat-lat_correction
		print ip_lat
		ip_lon = target_lon+lon_correction
		print ip_lon
	else: 
		print " Left cross tail wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat - lat_correction
		print ip_lat
		ip_lon = target_lon-lon_correction
		print ip_lon		
		
if 90 < ground_course <= 180 and  180 < wind_dir <= 270:
	print " 90 < ground_course <= 180 and  180 < wind_dir <= 270 "
	theta =  wind_dir - 180
	if wind_dir - 90  <= ground_course:
		print "Right cross head wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat-lat_correction
		print ip_lat
		ip_lon = target_lon-lon_correction
		print ip_lon
	else: 
		print " Right cross tail wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat - lat_correction
		print ip_lat
		ip_lon = target_lon-lon_correction
		print ip_lon
		
		

if 180 < ground_course <= 270 and   wind_dir < 90:
	print "if 180 < ground_course <= 270 and wind_dir < 90"
	theta =  90 - wind_dir
	if wind_dir +180  <= ground_course:
		print "right cross tail wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat-lat_correction
		print ip_lat
		ip_lon = target_lon-lon_correction
		print ip_lon
	else: 
		print "Left cross tail wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat - lat_correction
		print ip_lat
		ip_lon = target_lon+lon_correction
		print ip_lon	

if 180 < ground_course <= 270 and  180 < wind_dir <= 270:
	print "180 < ground_course <= 270 and  180 < wind_dir <= 270  "
	theta = wind_dir - 180
	if wind_dir <= ground_course:
		print "left cross head wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat+lat_correction
		print ip_lat
		ip_lon = target_lon+lon_correction
		print ip_lon
	else:  
		theta = 270 - wind_dir
		print "Right cross head wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat + lat_correction
		print ip_lat
		ip_lon = target_lon+lon_correction
		print ip_lon	

if 180 < ground_course <= 270 and  270 < wind_dir <= 360:
	print "180 < ground_course <= 270 and  270 < wind_dir <= 360 "
	theta = 360 - wind_dir 
	if theta + 180 <= ground_course:
		print "Right cross head wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat+lat_correction
		print ip_lat
		ip_lon = target_lon+lon_correction
		print ip_lon
	else:  
		theta = 270 - wind_dir
		print "Right cross head wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat + lat_correction
		print ip_lat
		ip_lon = target_lon-lon_correction
		print ip_lon
		
if 180 < ground_course <= 270 and  90 < wind_dir <= 180:
	print "180 < ground_course <= 270 and  90 < wind_dir <= 180 "
	theta = 180 - wind_dir 
	if theta + 180 >= ground_course:
		print "Left cross tail wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat+lat_correction
		print ip_lat
		ip_lon = target_lon+lon_correction
		print ip_lon
	else:  
		theta = 270 - wind_dir
		print "Right cross head wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat + lat_correction
		print ip_lat
		ip_lon = target_lon+lon_correction
		print ip_lon			

if 270 < ground_course <= 360 and  90 < wind_dir <= 180:
	print "270 < ground_course <= 360 and  90 < wind_dir <= 180 "
	theta = wind_dir - 90
	if theta + 270 < ground_course:
		print "Left cross tail wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat+lat_correction
		print ip_lat
		ip_lon = target_lon-lon_correction
		print ip_lon
	else:  
		theta = 360 - wind_dir + 180
		print "Right cross tail wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat + lat_correction
		print ip_lat
		ip_lon = target_lon+lon_correction
		print ip_lon
		
if 270 < ground_course <= 360 and  270 < wind_dir <= 360:
	print "270 < ground_course <= 360 and  270 < wind_dir <= 360"
	theta = wind_dir - 270
	if theta + 270 < ground_course:
		print "Left cross head wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat-lat_correction
		print ip_lat
		ip_lon = target_lon+lon_correction
		print ip_lon
	else:  
		theta = 360 - wind_dir
		print "Right cross head wind"
		o = wind_vel*math.sin(theta)
		a= wind_vel*math.cos(theta)
		##print  o
		##print  a
		lat_correction = o*fall_time*.00000275
		lon_correction = a*fall_time*.00000275
		##print lat_correction #Lat Correction West"
		##print lon_correction #Lon Correction North"
		ip_lat = target_lat - lat_correction
		print ip_lat
		ip_lon = target_lon - lon_correction
		print ip_lon			


