#include "CoordinateTransform.h"

CoordinateTransform::CoordinateTransform() {

}

void CoordinateTransform::set_initialGeodetic(FrameCoordinates coords) {
initialGeodetic.first = coords.first;
initialGeodetic.second = coords.second;
initialGeodetic.third = coords.third;

init_rad_latitude = M_PI/180.0*coords.second;
init_rad_longitude = M_PI/180.0*coords.first;

init_sin_lat = sin(init_rad_latitude);
init_sin_long = sin(init_rad_longitude);
init_cos_lat = cos(init_rad_latitude);
init_cos_long = cos(init_rad_longitude);

initialECEF = this->Geodetic_to_ECEF(coords);

}

FrameCoordinates CoordinateTransform::Geodetic_to_ECEF(FrameCoordinates coords) {

    FrameCoordinates  ECEFframe; //X, Y, Z

double rad_latitude = M_PI/180.0*coords.second;
double rad_longitude = M_PI/180.0*coords.first;

double sin_lat = sin(rad_latitude);
double sin_long = sin(rad_longitude);
double cos_lat = cos(rad_latitude);
double cos_long = cos(rad_longitude);


double N = this->calc_N(rad_latitude);

ECEFframe.first=  (N+coords.third)*cos_lat*cos_long;
ECEFframe.second = (N+coords.third)*cos_lat*sin_long;
ECEFframe.third = ((1-GEOee)*N+coords.third)*sin_lat;



   return ECEFframe;


}


FrameCoordinates CoordinateTransform::ECEF_to_NED (FrameCoordinates coords) {

 FrameCoordinates  NEDframe; //X, Y, Z
	

double X = coords.first-initialECEF.first;
double Y = coords.second-initialECEF.second;
double Z = coords.third-initialECEF.third;


NEDframe.first =  (-1*init_sin_lat*init_cos_long*X)+(-1*init_sin_lat*init_sin_long*Y)+(init_cos_lat*Z);
NEDframe.second = (-1*init_sin_long*X)+(init_cos_long*Y);
NEDframe.third = (-1*init_cos_lat*init_cos_long*X)+(-1*init_cos_lat*init_sin_long*Y)+(-1*init_sin_lat*Z);



   return NEDframe;



}
FrameCoordinates CoordinateTransform::Geodetic_to_NED (FrameCoordinates coords) {

return this->ECEF_to_NED(this->Geodetic_to_ECEF(coords));


}

double CoordinateTransform::calc_N(double latitude){

return GEOa/sqrt(1-GEOee*(sin(latitude)*sin(latitude)));

}


FrameCoordinates CoordinateTransform::Bframe_to_NED(Quaternions q, FrameCoordinates accBody) {

   FrameCoordinates  acc_NEDframe; //X, Y, Z

acc_NEDframe.first = (q.W*q.W+q.X*q.X-q.Y*q.Y-q.Z*q.Z)*accBody.first+2*(q.X*q.Y-q.W*q.Z)*accBody.second+2*(q.W*q.Y+q.X*q.Z)*accBody.third;
acc_NEDframe.second = 2*(q.X*q.Y+q.W*q.Z)*accBody.first+(q.W*q.W-q.X*q.X+q.Y*q.Y-q.Z*q.Z)*accBody.second+2*(q.Y*q.Z-q.W*q.X)*accBody.third;
acc_NEDframe.third = 2*(q.X*q.Z+q.W*q.Y)*accBody.first+2*(q.Y*q.Z+q.W*q.X)*accBody.second+(q.W*q.W-q.X*q.X-q.Y*q.Y+q.Z*q.Z)*accBody.third;



   return acc_NEDframe;
}

