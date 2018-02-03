#ifndef COORDINATETRANSFORM_H_
#define COORDINATETRANSFORM_H_
#include <math.h>

#define GEOa  6378137.0
#define GEOb  6356752.3
#define GEOf  0.00335281289
#define GEOee 0.00669438442

struct FrameCoordinates {

double first; //X or LONGITUDE
double second;//Y or LATITUDE
double third;//Z or ALTITUDE

};

struct Quaternions {

double W; 
double X;
double Y;
double Z;

};


class CoordinateTransform
{
public:

CoordinateTransform();

void set_initialGeodetic(FrameCoordinates coords);

FrameCoordinates Geodetic_to_ECEF(FrameCoordinates coords);
FrameCoordinates ECEF_to_NED (FrameCoordinates coords);
FrameCoordinates Geodetic_to_NED (FrameCoordinates coords);

FrameCoordinates Bframe_to_NED(Quaternions q, FrameCoordinates accBody);//Takes three values in the Body frame and converts them to the NED frame. Could be position, velocity, etc as long as it is X, Y, Z 
   

private:

FrameCoordinates initialGeodetic;
FrameCoordinates initialECEF;
double init_rad_latitude;
double init_rad_longitude;
double init_sin_lat;
double init_sin_long;
double init_cos_lat;
double init_cos_long;


double calc_N(double latitude);



   
};

#endif /* COORDINATETRANSFORM_H_ */
