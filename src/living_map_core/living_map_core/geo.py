"""Local SLAM frame <-> WGS84 (entrance anchor), with an uncertainty estimate."""
import math
from dataclasses import dataclass
R_E = 6378137.0
_F = 1 / 298.257223563
_E2 = 2 * _F - _F * _F

@dataclass
class Anchor:
    lat0: float = 36.8065
    lon0: float = 10.1815
    yaw_deg: float = 45.0      # theta0
    alt: float = 15.0
    def _radii(self):
        """WGS84 meridian (M) and prime-vertical (N) radii at the anchor latitude."""
        s2 = math.sin(math.radians(self.lat0)) ** 2
        return R_E * (1 - _E2) / (1 - _E2 * s2) ** 1.5 + self.alt, R_E / math.sqrt(1 - _E2 * s2) + self.alt
    def to_wgs84(self, x, y):
        th = math.radians(self.yaw_deg)
        e = x * math.cos(th) - y * math.sin(th)
        n = x * math.sin(th) + y * math.cos(th)
        M, N = self._radii()
        lat = self.lat0 + math.degrees(n / M)
        lon = self.lon0 + math.degrees(e / (N * math.cos(math.radians(self.lat0))))
        return lat, lon
    def to_local(self, lat, lon):
        M, N = self._radii()
        n = math.radians(lat - self.lat0) * M
        e = math.radians(lon - self.lon0) * N * math.cos(math.radians(self.lat0))
        th = math.radians(self.yaw_deg)
        return e * math.cos(th) + n * math.sin(th), -e * math.sin(th) + n * math.cos(th)
    def sigma(self, x, y, s_anchor=0.5, k_slam=0.02, s_theta_deg=2.0):
        d = math.hypot(x, y)
        return math.sqrt(s_anchor ** 2 + (k_slam * d) ** 2 + (d * math.radians(s_theta_deg)) ** 2)
    def as_dict(self): return dict(lat0=self.lat0, lon0=self.lon0, yaw_deg=self.yaw_deg, alt=self.alt)
