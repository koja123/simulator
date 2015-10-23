#ifndef ITM_H
#define ITM_H

namespace ITM {

void point_to_point(
    double elev[],
    double tht_m,
    double rht_m,
    double eps_dielect,
    double sgm_conductivity,
    double eno,
    double frq_mhz,
    int radio_climate,
    int pol,
    double conf,               // 0.01 .. .99, Fractions of situations
    double rel,                // 0.01 .. .99, Fractions of time
    double &dbloss,
    int &errnum);

}

#endif
