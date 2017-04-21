filename = "/dev/ttyAMA0"
nmea = open(filename)

for sentence in nmea:
  try:
    pNMEA = pynmea2.parse(sentence)
    if isinstance(pNMEA, pynmea2.types.talker.BOD):
      print "BOD"
      print '\tBearing True:', pNMEA.bearing_t
      print '\tBearing True Type:', pNMEA.bearing_t_type
      print '\tBearing Magnetic:', pNMEA.bearing_mag
      print '\tBearing Magnetic Type:', pNMEA.bearing_mag_type
      print '\tDestination:', pNMEA.dest
      print '\tStart:', pNMEA.start

    if isinstance(pNMEA, pynmea2.types.talker.RMC):
      print "RMC"
      print "\tTimestamp:", pNMEA.timestamp
      print '\tStatus:', pNMEA.status
      print "\tLatitude:", pNMEA.lat
      print "\tLatitude Direction:", pNMEA.lat_dir
      print "\tLongitude:", pNMEA.lon
      print "\tLongitude Direction:", pNMEA.lon_dir
      print "\tSpeed Over Ground:", pNMEA.spd_over_grnd
      print "\tTrue Course:", pNMEA.true_course
      print "\tDatestamp:", pNMEA.datestamp
      print "\tMagnetic Variation:", pNMEA.mag_variation
      print "\tMagnetic Variation Direction:", pNMEA.mag_var_dir

    if isinstance(pNMEA, pynmea2.types.talker.RMB):
      print "RMB"
      print '\tStatus:', pNMEA.status
      print "\tCross Track Error:", pNMEA.cross_track_error
      print "\tCross Track Error, direction to correct:", pNMEA.cte_correction_dir
      print "\tOrigin Waypoint ID:", pNMEA.origin_waypoint_id
      print "\tDestination Waypoint ID:", pNMEA.dest_waypoint_id
      print "\tDestination Waypoint Latitude:", pNMEA.dest_lat
      print "\tDestination Waypoint Lat Direction:", pNMEA.dest_lat_dir
      print "\tDestination Waypoint Longitude:", pNMEA.dest_lon
      print "\tDestination Waypoint Lon Direction:", pNMEA.dest_lon_dir
      print "\tRange to Destination:", pNMEA.dest_range
      print "\tTrue Bearing to Destination:", pNMEA.dest_true_bearing
      print "\tVelocity Towards Destination:", pNMEA.dest_velocity
      print "\tArrival Alarm:", pNMEA.arrival_alarm

    if isinstance(pNMEA, pynmea2.types.talker.RTE):
      print "RTE"
      print "\tNumber of sentences in sequence:", pNMEA.num_in_seq
      print "\tSentence Number:", pNMEA.sen_num
      print "\tStart Type:", pNMEA.start_type
      print "\tName or Number of Active Route:", pNMEA.active_route_id

    if isinstance(pNMEA, pynmea2.types.talker.GLL):
      print "GLL"
      print '\tLatitude:', pNMEA.lat
      print '\tLatitude Direction:', pNMEA.lat_dir
      print '\tLongitude:', pNMEA.lon
      print '\tLongitude Direction:', pNMEA.lon_dir
      print '\tTimestamp:', pNMEA.timestamp
      print '\tStatus:', pNMEA.status
      print "\tFAA mode indicator:", pNMEA.faa_mode

    if isinstance(pNMEA, pynmea2.types.talker.GSA):
      print "GSA"
      print '\tMode:', pNMEA.mode
      print '\tMode fix type:', pNMEA.mode_fix_type
      print '\tSV ID01:', pNMEA.sv_id01
      print '\tSV ID02:', pNMEA.sv_id02
      print '\tSV ID03:', pNMEA.sv_id03
      print '\tSV ID04:', pNMEA.sv_id04
      print '\tSV ID05:', pNMEA.sv_id05
      print '\tSV ID06:', pNMEA.sv_id06
      print '\tSV ID07:', pNMEA.sv_id07
      print '\tSV ID08:', pNMEA.sv_id08
      print '\tSV ID09:', pNMEA.sv_id09
      print '\tSV ID10:', pNMEA.sv_id10
      print '\tSV ID11:', pNMEA.sv_id11
      print '\tSV ID12:', pNMEA.sv_id12
      print '\tPDOP (Dilution of precision):', pNMEA.pdop
      print '\tHDOP (Horizontal DOP):', pNMEA.hdop
      print '\tVDOP (Vertical DOP):', pNMEA.vdop

    if isinstance(pNMEA, pynmea2.types.talker.GSV):
      print "GSV"
      print '\tNumber of messages of type in cycle:', pNMEA.num_messages
      print '\tMessage Number:', pNMEA.msg_num
      print '\tTotal number of SVs in view:', pNMEA.num_sv_in_view
      if int(pNMEA.snr_1) > 0:
        print '\tSV #1'
        print '\t\tSV PRN number:', pNMEA.sv_prn_num_1
        print '\t\tElevation in degrees:', pNMEA.elevation_deg_1
        print '\t\tAzimuth, deg from true north:',pNMEA.azimuth_1
        print '\t\tSNR:', pNMEA.snr_1
      if int(pNMEA.snr_2) > 0:
        print '\tSV #2'
        print '\t\tSV PRN number:',pNMEA.sv_prn_num_2
        print '\t\tElevation in degrees:', pNMEA.elevation_deg_2
        print '\t\tAzimuth, deg from true north:', pNMEA.azimuth_2
        print '\t\tSNR:', pNMEA.snr_2
      if int(pNMEA.snr_3) > 0:
        print '\tSV #3'
        print '\t\tSV PRN number:', pNMEA.sv_prn_num_3
        print '\t\tElevation in degrees:', pNMEA.elevation_deg_3
        print '\t\tAzimuth, deg from true north:', pNMEA.azimuth_3
        print '\t\tSNR:', pNMEA.snr_3
      if int(pNMEA.snr_4) > 0:
        print '\tSV #4'
        print '\t\tSV PRN number:', pNMEA.sv_prn_num_4
        print '\t\tElevation in degrees:', pNMEA.elevation_deg_4
        print '\t\tAzimuth, deg from true north:', pNMEA.azimuth_4
        print '\t\tSNR:', pNMEA.snr_4

    if isinstance(pNMEA, pynmea2.types.talker.GGA):
      print "GGA"
      print '\tTimestamp:', pNMEA.timestamp
      print '\tLatitude:', pNMEA.lat
      print '\tLatitude Direction:', pNMEA.lat_dir
      print '\tLongitude:', pNMEA.lon
      print '\tLongitude Direction:',pNMEA.lon_dir
      print '\tGPS Quality Indicator:', pNMEA.gps_qual
      print '\tNumber of Satellites in use:', pNMEA.num_sats
      print '\tHorizontal Dilution of Precision:', pNMEA.horizontal_dil
      print '\tAntenna Alt above sea level (mean):', pNMEA.altitude
      print '\tUnits of altitude (metres):', pNMEA.altitude_units
      print '\tGeoidal Separation:', pNMEA.geo_sep
      print '\tUnits of Geoidal Separation:', pNMEA.geo_sep_units
      print '\tAge of Differential GPS Data (secs):', pNMEA.age_gps_data
      print '\tDifferential Reference Station ID:', pNMEA.ref_station_id

  except:
    pass

nmea.close()
