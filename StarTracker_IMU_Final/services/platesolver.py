"""
services/platesolver.py

Offline Plate Solver Service

사용:
- Astrometry.net solve-field
- Offline index files
- 현재 테스트 환경: index-4115

반환값:
- RA
- Dec
- Rotation
- FOV
- Pixel Scale
- Image Size
- Parity

중요:
이 클래스는 Plate Solving 실행 전 이전 결과 파일을 삭제하여
과거의 .solved / .wcs 파일 때문에 성공 여부를 잘못 판단하는 것을 방지한다.
"""

import os
import subprocess
from pathlib import Path
from typing import Optional

import astropy.units as u
import numpy as np

from astropy.io import fits
from astropy.wcs import WCS

import config


class PlateSolver:
    """Astrometry.net 기반 오프라인 Plate Solver."""

    # solve-field가 생성할 수 있는 파일 확장자
    GENERATED_EXTENSIONS = (
        "axy",
        "corr",
        "match",
        "rdls",
        "xyls",
        "new",
        "wcs",
        "solved",
    )

    def __init__(self):

        self.last_result: Optional[dict] = None

    # ==================================================
    # Plate Solve
    # ==================================================

    def solve(self, image_path: str) -> Optional[dict]:
        """
        입력 이미지를 Plate Solving하고 화면 중심의 RA/Dec 등을 반환한다.

        Parameters
        ----------
        image_path : str
            Plate Solving할 이미지 경로

        Returns
        -------
        dict | None
            성공 시 Plate Solving 결과.
            실패 시 None.
        """

        image = Path(image_path)

        if not image.exists():
            print(
                f"[PlateSolver] Image Not Found: {image}"
            )
            return None

        # 출력파일을 원본 이미지와 같은 폴더에 생성
        output_dir = image.parent
        base = output_dir / image.stem

        # --------------------------------------------------
        # 1. 이전 Plate Solving 결과 제거
        # --------------------------------------------------

        self._remove_previous_results(base)

        # --------------------------------------------------
        # 2. solve-field 명령 생성
        # --------------------------------------------------

        command = [
            "solve-field",
            str(image),

            "--dir",
            str(output_dir),

            "--scale-units",
            config.PLATE_SCALE_UNITS,

            "--scale-low",
            str(config.PLATE_SCALE_LOW),

            "--scale-high",
            str(config.PLATE_SCALE_HIGH),

            "--downsample",
            str(config.PLATE_DOWNSAMPLE),

            "--overwrite",
            "--no-plots",
        ]

        print(
            f"[PlateSolver] Solving Started: {image}"
        )

        # --------------------------------------------------
        # 3. solve-field 실행
        # --------------------------------------------------

        try:

            process = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=config.PLATE_SOLVE_TIMEOUT,
                check=False,
            )

        except subprocess.TimeoutExpired:

            print(
                "[PlateSolver] Failed: "
                f"Timeout after "
                f"{config.PLATE_SOLVE_TIMEOUT} seconds."
            )

            self._cleanup_intermediate_files(base)

            return None

        except FileNotFoundError:

            print(
                "[PlateSolver] Failed: "
                "'solve-field' command was not found."
            )

            return None

        except Exception as error:

            print(
                f"[PlateSolver] Execution Error: {error}"
            )

            self._cleanup_intermediate_files(base)

            return None

        # --------------------------------------------------
        # 4. 이번 실행의 성공 여부 확인
        # --------------------------------------------------

        solved_path = Path(f"{base}.solved")
        wcs_path = Path(f"{base}.wcs")
        new_path = Path(f"{base}.new")

        if (
            process.returncode != 0
            or not solved_path.exists()
            or not wcs_path.exists()
            or not new_path.exists()
        ):

            print("[PlateSolver] Solving Failed")

            if process.stderr.strip():

                print(
                    "[PlateSolver] STDERR:"
                )

                print(
                    process.stderr.strip()
                )

            self._cleanup_intermediate_files(base)

            return None

        # --------------------------------------------------
        # 5. WCS 결과 분석
        # --------------------------------------------------

        try:

            result = self._read_solution(
                wcs_path=wcs_path,
                new_path=new_path,
            )

        except Exception as error:

            print(
                f"[PlateSolver] WCS Analysis Failed: "
                f"{error}"
            )

            self._cleanup_intermediate_files(base)

            return None

        # --------------------------------------------------
        # 6. 중간 파일 정리
        # --------------------------------------------------

        self._cleanup_intermediate_files(base)

        self.last_result = result

        print("[PlateSolver] Solving Success")

        print(
            f"[PlateSolver] RA  : "
            f"{result['ra']:.6f} deg"
        )

        print(
            f"[PlateSolver] DEC : "
            f"{result['dec']:.6f} deg"
        )

        print(
            f"[PlateSolver] FOV : "
            f"{result['fov_x']:.3f} x "
            f"{result['fov_y']:.3f} deg"
        )

        return result

    # ==================================================
    # Read WCS Solution
    # ==================================================

    def _read_solution(
        self,
        wcs_path: Path,
        new_path: Path,
    ) -> dict:
        """Astrometry.net 결과 파일에서 천문 정보를 읽는다."""

        # WCS 정보
        with fits.open(wcs_path) as hdul:

            wcs = WCS(
                hdul[0].header,
                naxis=2,
            )

        # 이미지 크기
        with fits.open(new_path) as hdul:

            image_header = hdul[0].header

            width = int(
                image_header["NAXIS1"]
            )

            height = int(
                image_header["NAXIS2"]
            )

        # 이미지 중앙 픽셀
        center_x = width / 2.0
        center_y = height / 2.0

        # 화면 중앙의 천구좌표
        center = wcs.pixel_to_world(
            center_x,
            center_y,
        )

        # 픽셀 스케일
        pixel_scale_x, pixel_scale_y = (
            wcs.proj_plane_pixel_scales()
        )

        pixel_scale_x_arcsec = (
            pixel_scale_x.to(u.arcsec).value
        )

        pixel_scale_y_arcsec = (
            pixel_scale_y.to(u.arcsec).value
        )

        # FOV
        fov_x = (
            pixel_scale_x * width
        ).to(u.deg).value

        fov_y = (
            pixel_scale_y * height
        ).to(u.deg).value

        # 회전각
        pixel_scale_matrix = (
            wcs.pixel_scale_matrix
        )

        rotation_rad = np.arctan2(
            pixel_scale_matrix[0, 1],
            pixel_scale_matrix[0, 0],
        )

        rotation_deg = float(
            np.degrees(rotation_rad)
        )

        # Parity
        determinant = np.linalg.det(
            pixel_scale_matrix
        )

        parity = (
            "normal"
            if determinant > 0
            else "flipped"
        )

        return {
            "ra": float(center.ra.deg),
            "dec": float(center.dec.deg),

            "rotation": rotation_deg,

            "fov_x": float(fov_x),
            "fov_y": float(fov_y),

            "pixel_scale_x": float(
                pixel_scale_x_arcsec
            ),

            "pixel_scale_y": float(
                pixel_scale_y_arcsec
            ),

            "width": width,
            "height": height,

            "parity": parity,
        }

    # ==================================================
    # Remove Previous Results
    # ==================================================

    def _remove_previous_results(
        self,
        base: Path,
    ) -> None:
        """
        이전 실행의 모든 결과 파일을 삭제한다.

        특히 이전 .solved 파일 때문에 새 실행이 실패했는데도
        성공으로 잘못 판단하는 문제를 방지한다.
        """

        for extension in self.GENERATED_EXTENSIONS:

            path = Path(
                f"{base}.{extension}"
            )

            self._safe_remove(path)

        index_file = Path(
            f"{base}-indx.xyls"
        )

        self._safe_remove(index_file)

    # ==================================================
    # Cleanup Intermediate Files
    # ==================================================

    def _cleanup_intermediate_files(
        self,
        base: Path,
    ) -> None:
        """
        디버깅과 재사용에 필요한 .new, .wcs, .solved는 유지하고
        불필요한 중간 파일만 삭제한다.
        """

        intermediate_extensions = (
            "axy",
            "corr",
            "match",
            "rdls",
            "xyls",
        )

        for extension in intermediate_extensions:

            path = Path(
                f"{base}.{extension}"
            )

            self._safe_remove(path)

        index_file = Path(
            f"{base}-indx.xyls"
        )

        self._safe_remove(index_file)

    # ==================================================
    # Safe File Remove
    # ==================================================

    @staticmethod
    def _safe_remove(path: Path) -> None:

        try:

            if path.exists():

                path.unlink()

        except Exception as error:

            print(
                f"[PlateSolver] Cleanup Warning: "
                f"{path} -> {error}"
            )