#ifndef GEODETIC_CONV_HPP
#define GEODETIC_CONV_HPP

#include <cmath>

namespace frame_transforms
{

// A simple geodetic converter class based on WGS84 ellipsoid model.
class GeodeticConverter
{
public:
  GeodeticConverter() = default;

  void initialiseReference(double latitude, double longitude, double altitude)
  {
    lat_ref_ = latitude * M_PI / 180.0;
    lon_ref_ = longitude * M_PI / 180.0;
    alt_ref_ = altitude;

    geodetic2Ecef(latitude, longitude, altitude, &x_ref_, &y_ref_, &z_ref_);
    is_reference_set_ = true;
  }

  void geodetic2Ecef(double latitude, double longitude, double altitude, double* x, double* y, double* z)
  {
    double lat_rad = latitude * M_PI / 180.0;
    double lon_rad = longitude * M_PI / 180.0;

    double N = a_ / sqrt(1 - e_sq_ * sin(lat_rad) * sin(lat_rad));

    *x = (N + altitude) * cos(lat_rad) * cos(lon_rad);
    *y = (N + altitude) * cos(lat_rad) * sin(lon_rad);
    *z = ((1 - e_sq_) * N + altitude) * sin(lat_rad);
  }

  void ecef2Geodetic(double x, double y, double z, double* latitude, double* longitude, double* altitude)
  {
    double p = sqrt(x * x + y * y);
    double theta = atan2(z * a_, p * b_);

    *longitude = atan2(y, x);
    *latitude = atan2(z + e_prime_sq_ * b_ * pow(sin(theta), 3), p - e_sq_ * a_ * pow(cos(theta), 3));

    double N = a_ / sqrt(1 - e_sq_ * sin(*latitude) * sin(*latitude));
    *altitude = p / cos(*latitude) - N;

    *longitude = *longitude * 180.0 / M_PI;
    *latitude = *latitude * 180.0 / M_PI;
  }

  void ecef2Enu(double x, double y, double z, double* east, double* north, double* up)
  {
    double dx = x - x_ref_;
    double dy = y - y_ref_;
    double dz = z - z_ref_;

    *east = -sin(lon_ref_) * dx + cos(lon_ref_) * dy;
    *north = -sin(lat_ref_) * cos(lon_ref_) * dx - sin(lat_ref_) * sin(lon_ref_) * dy + cos(lat_ref_) * dz;
    *up = cos(lat_ref_) * cos(lon_ref_) * dx + cos(lat_ref_) * sin(lon_ref_) * dy + sin(lat_ref_) * dz;
  }

  void geodetic2Enu(double latitude, double longitude, double altitude, double* east, double* north, double* up)
  {
    double x, y, z;
    geodetic2Ecef(latitude, longitude, altitude, &x, &y, &z);
    ecef2Enu(x, y, z, east, north, up);
  }

  bool isReferenceSet() const
  {
    return is_reference_set_;
  }
  double getReferenceEcefX() const
  {
    return x_ref_;
  }
  double getReferenceEcefY() const
  {
    return y_ref_;
  }
  double getReferenceEcefZ() const
  {
    return z_ref_;
  }

private:
  // WGS84 ellipsoid parameters
  const double a_ = 6378137.0;       // semi-major axis
  const double b_ = 6356752.314245;  // semi-minor axis
  const double f_ = (a_ - b_) / a_;
  const double e_sq_ = f_ * (2 - f_);
  const double e_prime_sq_ = (a_ * a_ - b_ * b_) / (b_ * b_);

  double lat_ref_ = 0.0, lon_ref_ = 0.0, alt_ref_ = 0.0;
  double x_ref_ = 0.0, y_ref_ = 0.0, z_ref_ = 0.0;
  bool is_reference_set_ = false;
};

}  // namespace frame_transforms

#endif  // GEODETIC_CONV_HPP
