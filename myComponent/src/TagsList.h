

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <qt4/QtCore/qmap.h>

class TagsList {
	  typedef struct {
	    int id;
	    float tx;
	    float ty;
	    float tz;
	    float rx;
	    float ry;
	    float rz;
	    QTime clock;
	  }Mark;
	    
	  QMap<int,Mark> mapa;
	  QMutex mutex;
	  QVec memory;
	  InnerModel *inner;


	  
public:
  bool inMemory;	
  int initMark;
  
  TagsList(InnerModel *inner_);
  void add(const RoboCompAprilTags::tag &t);
  Mark get(int id);
  bool existe(int id);
  float distancia(int initMark);
  void borraMarca(int id);
  int getM();
  void set(int init);
  
};