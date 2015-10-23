//ScenSim-Port://#ifndef _structures_h
//ScenSim-Port://#define _structures_h

#ifndef S_HULL_H //ScenSim-Port://
#define S_HULL_H //ScenSim-Port://


// for FILE

//ScenSim-Port://#include <stdlib.h>
#include <cstdlib>//ScenSim-Port://
#include <vector>
#include <set>
#include <iostream>//ScenSim-Port://

namespace SHull {//ScenSim-Port://

/* 
   for use in s_hull.C

S-hull, Copyright (c) 2010
Dr David SInclair
Cambridge, UK

email david@s-hull.org

The software includes the S-hull programs.
S-hull is copyrighted as above.
S-hull is free software and may be obtained from www.s-hull.org.
It may be freely copied, modified, 
and redistributed under the following conditions:

S-hull is free software and may be obtained from www.s-hull.org.
It may be freely copied, modified, 
and redistributed under the following conditions which might loosely be termed a contribtors beerware license:
1. All copyright notices must remain intact in all files.
2. A copy of this text file must be distributed along with any copies 
   of S-hull that you redistribute; this includes copies that you have 
   modified, or copies of programs or other software products that 
   include S-hull where distributed as source.

3. If you modify S-hull, you must include a notice giving the
   name of the person performing the modification, the date of
   modification, and the reason for such modification.

4. If you are distributing a binary or compiled version of s-hull it
	    is not necessary to include any acknowledgement or reference
	    to s-hull.
5. There is no warranty or other guarantee of fitness for S-hull, it is 
   provided solely "as is".  Bug reports or fixes may be sent to 
   bugs@s-hull.org; the authors may or may not act on them as 
   they desire.
6. By copying or compliing the code for S-hull you explicitly indemnify 
the copyright holder against any liability he may incur as a result of you 
copying the code.

7. If you meet any of the contributors to the code you used from s-hull.org
	    in a pub or a bar, and you think the source code they contributed to is worth it,
	    you can buy them a beer.

	    If your principles run against beer a bacon-double-cheeseburger would do just as nicely
	    or you could email david@s-hull.org and arrange to make a donation of 10 of your local currancy units
	    to support s-hull.org.
	    

*/

struct Triad
{
  int a,b, c;
  int ab, bc, ac;  // adjacent edges index to neighbouring triangle.
  double ro, R,C;
  //std::set<int> idx;
  Triad() {};
Triad(int x, int y) : a(x), b(y),c(0), ab(-1), bc(-1), ac(-1), ro(-1), R(0), C(0) {};
Triad(int x, int y, int z) : a(x), b(y), c(z),  ab(-1), bc(-1), ac(-1), ro(-1), R(0), C(0) {};
Triad(const Triad &p) : a(p.a), b(p.b), c(p.c), ab(p.ab), bc(p.bc), ac(p.ac), ro(p.ro), R(p.R), C(p.C) {};

  Triad &operator=(const Triad &p)
  {
    a = p.a;
    b = p.b;
    c = p.c;

    ab = p.ab;
    bc = p.bc;
    ac = p.ac;

    ro = p.ro;
    R = p.R;
    C = p.C;

    return *this;
  };
  void prnt(){
      std::cerr << a << " " << b << " " << c << "   " << ab << " " << ac << " " << bc << std::endl;
  };
};



/* point structure for s_hull only.
   has to keep track of triangle ids as hull evolves.


*/


struct Shx
{
  int id, trid;
  double r,c , tr, tc;
  double ro;
  Shx() {};
  //ScenSim-Port://Shx(int a, int b) : r(a), c(b), ro(0), id(-1) {}; 
  //ScenSim-Port://Shx(int a, int b, double x) : r(a), c(b), ro(x), id(-1) {};
  Shx(const Shx &p) : id(p.id), trid(p.trid), r(p.r), c(p.c), tr(p.tr), tc(p.tc), ro(p.ro) {};

  Shx &operator=(const Shx &p)
  {
    id = p.id;
    trid = p.trid;
    r = p.r;
    c = p.c;
    tr = p.tr;
    tc = p.tc;
    ro = p.ro;
    return *this;
  };

};

//inline bool operator==(const Shx &a, const Shx &b) 
//{ 
//  return a.r == b.r && a.c == b.c;
//};

// sort into descending order (for use in corner responce ranking).
inline bool operator<(const Shx &a, const Shx &b) 
{ 
  if( a.ro == b.ro)
    return a.r < b.r;
  return a.ro <  b.ro;
};



// from s_hull.C


void s_hull_del_ray2( std::vector<Shx> &pts, std::vector<Triad> &triads);
void circle_cent2(double r1,double c1, double r2,double c2, double r3,double c3,double &r,double &c, double &ro2);
void circle_cent4(double r1,double c1, double r2,double c2, double r3,double c3,double &r,double &c, double &ro2);
void write_Shx(std::vector<Shx> &pts, char * fname);
void write_Triads(std::vector<Triad> &ts, char * fname);
void T_flip2( std::vector<Shx> &pts, std::vector<Triad> &triads, int *slump, int numt, int start);
void T_flip3( std::vector<Shx> &pts, std::vector<Triad> &triads, int *slump, int numt, int start,std::set<int> &ids );
void T_flip4( std::vector<Shx> &pts, std::vector<Triad> &triads, int *slump, std::set<int> &ids);


}//ScenSim-Port://

#endif
