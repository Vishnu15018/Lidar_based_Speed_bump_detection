#include <iostream>
#include <math.h>
#include <vector>
# define M_PI  3.14159265358979323846 
//#define a 6378137.0000 	//earth radius in meters	(WGS84)
//#define b 6356752.3142  //earth semiminor in meters	(WGS84)
//#define e sqrt(1-pow((b/a),2))

const double a=6378137.0000;
const double b=6356752.3142; 
const double e=sqrt(1-pow((b/a),2));


std::vector<double> llh2xyz(std::vector<double> llh){
    std::vector<double> xyz(3);
    double lat_rad=llh[0]*(M_PI/180);
    double lon_rad=llh[1]*(M_PI/180);
    double N_phi= a/sqrt( 1 - pow( (e*sin(lat_rad)) , 2 ) );
    xyz[0]=(N_phi + llh[2])*cos(lat_rad)*cos(lon_rad);
    xyz[1]=(N_phi + llh[2])*cos(lat_rad)*sin(lon_rad);
    xyz[2]=( pow( (b/a),2 )*N_phi +llh[2] ) * sin(lat_rad);
    
    return xyz;
} 


std::vector<double> xyz2llh(std::vector<double> xyz){

    double x2= pow(xyz[0],2);
    double y2=pow(xyz[1],2);
    double z2=pow(xyz[2],2);
    double b2=b*b;
    double e2=e*e;
    double ep=e*(a/b);
    double r=sqrt(x2+y2);
    double r2=r*r;
    double E2=pow(a,2)-pow(b,2);
    double F=54*b2*z2;
    double G=r2+(1-e2)*z2 -e2*E2;
    double c=(e2*e2*F*r2)/(G*G*G);
     double s= pow( (1+c+sqrt(c*c + 2*c)),(1/3) );
    double P= F/( 3*pow( (s+1/s+1),2 )*G*G );
    double Q= sqrt(1+2*e2*e2*P);
    double ro= -(P*e2*r)/(1+Q) + sqrt((a*a/2)*(1+1/Q)-(P*(1-e2)*z2)/(Q*(1+Q))-P*r2/2);
    double tmp=pow((r-e2*ro),2);
    double U=sqrt(tmp+z2);
    double V=sqrt(tmp+(1-e2)*z2);
    double zo=(b2*xyz[2])/(a*V);
    std::vector<double> llh(3);
    llh[2]=U*(1-b2/(a*V));
    llh[0]=atan((xyz[2]+ep*ep*zo)/r)/M_PI*180;
    llh[1]=atan2(xyz[1],xyz[0])/M_PI*180;
    return llh;

}

std::vector<double> enu2llh(std::vector<double> data,std::vector<double> org_llh){
    std::vector<double> data_xyz(3);
    std::vector<double> org_xyz=llh2xyz(org_llh);
    double phi=org_llh[0]*M_PI/180;
    double lam=org_llh[1]*M_PI/180;
    data_xyz[0]=-sin(phi)*data[0]-sin(phi)*cos(lam)*data[1]+cos(phi)*cos(lam)*data[2] + org_xyz[0];
    data_xyz[1]=cos(lam)*data[0]-sin(phi)*sin(lam)*data[1]+cos(phi)*cos(lam)*data[2] + org_xyz[1];
    data_xyz[2]=cos(phi)*data[1] + sin(phi)*data[2] + org_xyz[2];
     std::vector<double> data_llh=xyz2llh(data_xyz);
     return data_llh;
}