#!/usr/bin/env python3
import serial, threading, time, math

class Gps:
    def __init__(self, port='/dev/ttyS0', baud=9600):
        self.ser = serial.Serial(port, baud, timeout=0.5)
        self.lat = None
        self.lon = None
        self.speed = 0.0    # m/s
        self.course = 0.0   # deg true
        self.fix = 0
        threading.Thread(target=self._parse, daemon=True).start()

    # ---------- NMEA parser ----------
    def _parse(self):
        while True:
            try:
                line = self.ser.readline().decode('ascii', errors='ignore')
                if line.startswith('$GNRMC') or line.startswith('$GPRMC'):
                    parts = line.split(',')
                    if parts[2] == 'A':                # valid fix
                        self.fix = 1
                        # time
                        # lat
                        lat_deg = float(parts[3][:2])
                        lat_min = float(parts[3][2:])
                        self.lat = lat_deg + lat_min / 60.0 * (1 if parts[4] == 'N' else -1)
                        # lon
                        lon_deg = float(parts[5][:3])
                        lon_min = float(parts[5][3:])
                        self.lon = lon_deg + lon_min / 60.0 * (1 if parts[6] == 'E' else -1)
                        # speed knots → m/s
                        self.speed = float(parts[7]) * 0.514444
                        # course
                        self.course = float(parts[8]) if parts[8] else 0.0
                    else:
                        self.fix = 0
            except:
                pass

    # ---------- helpers ----------
    @staticmethod
    def haversine(lat1, lon1, lat2, lon2):
        R = 6371000  # Earth radius m
        φ1, φ2 = math.radians(lat1), math.radians(lat2)
        Δφ = math.radians(lat2 - lat1)
        Δλ = math.radians(lon2 - lon1)
        a = math.sin(Δφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(Δλ/2)**2
        return 2 * R * math.asin(math.sqrt(a))   # metres

    def dist_bearing(self, tgt_lat, tgt_lon):
        if self.lat is None:
            return None, None
        d = self.haversine(self.lat, self.lon, tgt_lat, tgt_lon)
        y = math.sin(math.radians(tgt_lon - self.lon)) * math.cos(math.radians(tgt_lat))
        x = math.cos(math.radians(self.lat))*math.sin(math.radians(tgt_lat)) - \
            math.sin(math.radians(self.lat))*math.cos(math.radians(tgt_lat))*math.cos(math.radians(tgt_lon - self.lon))
        brg = math.degrees(math.atan2(y, x)) % 360
        return d, brg