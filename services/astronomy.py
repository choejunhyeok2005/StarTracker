"""
services/astronomy.py

Astronomy Coordinate Service

- Observer location 설정
- RA/DEC -> ALT/AZ 변환
- GNSS UTC 또는 시스템 UTC 사용
"""

from datetime import datetime
from typing import Optional

import astropy.units as u

from astropy.coordinates import AltAz, EarthLocation, SkyCoord
from astropy.time import Time


class Astronomy:

    def __init__(self):

        self.location: Optional[EarthLocation] = None

    # ==================================================
    # Observer Location
    # ==================================================

    def set_location(
        self,
        latitude: float,
        longitude: float,
        altitude: float = 0.0,
    ) -> None:

        self.location = EarthLocation(
            lat=latitude * u.deg,
            lon=longitude * u.deg,
            height=altitude * u.m,
        )

        print(
            "[Astronomy] Location Set: "
            f"LAT={latitude:.6f}, "
            f"LON={longitude:.6f}, "
            f"ALT={altitude:.1f}m"
        )

    # ==================================================
    # RA / DEC -> ALT / AZ
    # ==================================================

    def radec_to_altaz(
        self,
        ra: float,
        dec: float,
        observation_time: Optional[Time] = None,
    ) -> tuple[float, float]:

        if self.location is None:
            raise RuntimeError(
                "Observer location is not set."
            )

        if observation_time is None:
            observation_time = Time.now()

        target = SkyCoord(
            ra=ra * u.deg,
            dec=dec * u.deg,
            frame="icrs",
        )

        altaz_frame = AltAz(
            obstime=observation_time,
            location=self.location,
        )

        result = target.transform_to(altaz_frame)

        return (
            float(result.alt.degree),
            float(result.az.degree),
        )

    # ==================================================
    # Observation Time
    # ==================================================

    def get_current_time(
        self,
        utc_datetime: Optional[datetime] = None,
    ) -> Time:
        """
        GNSS UTC가 있으면 GNSS 시간을 사용한다.

        GNSS UTC가 아직 없으면 시스템 UTC를 사용한다.
        """

        if utc_datetime is not None:
            return Time(utc_datetime)

        return Time.now()