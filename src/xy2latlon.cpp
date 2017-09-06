#include <iostream>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <math.h>
#include <csv.h>
#include <iomanip>      // std::setprecision

using namespace std;


/// Coordinates struct
struct Coordinates
{
    long double latitude;
    long double longitude;
    long double altitude;
};


// How to get UTM from Longitude:
// UTM = 1.0 + floor((lngd + 180.0) / 6.0);
//
// (italy -> UTM = 32, southern = false)
Coordinates xy2latlon_helper(double x, double y, double utmz, bool southern)
{

    // WGS 84 datum
    long double eqRad = 6378137.0;
    long double flat = 298.2572236;

    // constants used in calculations:
    long double a = eqRad;           // equatorial radius in meters
    long double f = 1.0 / flat;        // polar flattening
    long double b = a * (1.0 - f);     // polar radius
    long double e = sqrt(1.0 - (pow(b, 2) / pow(a, 2))); // eccentricity
    long double k0 = 0.9996;
    long double k = 1;
    long double drad = M_PI / 180.0;

    long double esq = (1.0 - (b / a) * (b / a));
    long double e0sq = e * e / (1.0 - e * e);
    long double zcm = 3.0 + 6.0 * (utmz - 1.0) - 180.0;                         // Central meridian of zone
    long double e1 = (1 - sqrt(1 - e * e) ) / (1.0 + sqrt(1 - e * e));
    long double M0 = 0.0;
    long double M = 0.0;

    if (!southern)
        M = M0 + y / k0;    // Arc length along standard meridian.
    else
        M = M0 + (y - 10000000.0) / k;

    long double mu = M / (a * (1.0 - esq * (1.0 / 4.0 + esq * (3.0 / 64.0 + 5.0 * esq / 256.0))));
    long double phi1 = mu + e1 * (3.0 / 2.0 - 27.0 * e1 * e1 / 32.0) * sin(2.0 * mu) + e1 * e1 * (21.0 / 16.0 - 55.0 * e1 * e1 / 32.0) * sin(4.0 * mu);   //Footprint Latitude
    phi1 = phi1 + e1 * e1 * e1 * (sin(6.0 * mu) * 151.0 / 96.0 + e1 * sin(8.0 * mu) * 1097.0 / 512.0);

    long double C1 = e0sq * cos(phi1) * cos(phi1);
    long double T1 = tan(phi1) * tan(phi1);
    long double N1 = a / sqrt(1.0f - pow(e * sin(phi1), 2.0f));
    long double R1 = N1 * (1.0f - pow(e, 2.0f)) / (1.0f - pow(e * sin(phi1), 2.0f));
    long double D = (x - 500000.0) / (N1 * k0);
    long double phi = (D * D) * (1.0 / 2.0 - D * D * (5.0 + 3.0 * T1 + 10.0 * C1 - 4.0 * C1 * C1 - 9.0 * e0sq) / 24.0);
    phi = phi + pow(D, 6.0) * (61.0 + 90.0 * T1 + 298.0 * C1 + 45.0 * T1 * T1 - 252.0 * e0sq - 3.0 * C1 * C1) / 720.0;
    phi = phi1 - (N1 * tan(phi1) / R1) * phi;

    long double lat = floor(1000000.0 * phi / drad) / 1000000.0;
    long double lng = D * (1.0 + D * D * ((-1.0 - 2.0 * T1 - C1) / 6.0 + D * D * (5.0 - 2.0 * C1 + 28.0 * T1 - 3.0 * C1 * C1 + 8.0 * e0sq + 24.0 * T1 * T1) / 120.0)) / cos(phi1);
    long double lngd = zcm + lng / drad;

    Coordinates coords = { lat, lngd };
    return coords;
}

int main(int argc, char *argv[])
{
    io::CSVReader<2, io::trim_chars<>, io::no_quote_escape<';'> > latlonfile(argv[1]);

    long double x, x1, y1, y = 0.0f;
    while (latlonfile.read_row(x, y))
    {
        long double alpha = 2.57f;
        x1 = cos(alpha) * x - sin(alpha) * y;
        y1 = sin(alpha) * x + cos(alpha) * y;
        x1 = x1 + 382401.12f ; //381961.34f; //382071.82f;
        y1 = y1 + 4982193.27f; // 4981491.40f; // 4981699.72f;
        struct Coordinates coordinate = xy2latlon_helper(x1, y1, 32.0f, false);
        cout << std::setprecision(9) << coordinate.latitude << ";"  << coordinate.longitude << endl;

    }

}
