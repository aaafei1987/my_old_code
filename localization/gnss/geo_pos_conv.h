#ifndef __GEO_POS_CONV__
#define __GEO_POS_CONV__

#include <math.h>

class geo_pos_conv {
private:
	double m_x0;  //m
	double m_y0;  //m
	double m_z0;  //m
    
	double m_x;  //m
	double m_y;  //m
	double m_z;  //m
    

	double m_lat;  //latitude
	double m_lon; //longitude
	double m_h;

	double m_OLo;
	double m_OLat;

	bool m_initialized = false;

public:
	geo_pos_conv();
 
        bool hasInitialized() { return m_initialized;}

	double x() const;
	double y() const;
	double z() const;

	void set_origin(double lat, double lon);
	void set_xyz(double cx,   double cy,   double cz);

	//set llh in nmea degrees
	void set_llh_nmea_degrees(double latd,double lond, double h);

    void llh_to_xyz(double lat, double lon, double ele);

	void conv_llh2xyz(void);
	void conv_xyz2llh(void);
};

#endif
