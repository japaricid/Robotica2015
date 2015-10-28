#include "TagsList.h"



TagsList(InnerModel *inner_) : inner(inner_){
	  inMemory=false;
	  initMark=0;
};

void TagsList::add(const RoboCompAprilTags::tag& t)
{
	  QMutexLocker ml(&mutex);
	  Mark marca;
	
	  marca.id = t.id;
	  marca.rx = t.rx;
	  marca.ry = t.ry;
	  marca.rz = t.rz;
	  marca.tx = t.tx*1000;
	  marca.ty = t.ty*1000;
	  marca.tz = t.tz*1000;
	  marca.clock=QTime::currentTime();
	  mapa.insert(t.id,marca);
	  if(initMark == marca.id){
	    memory = inner -> transform("world",QVec::vec3(marca.tx,0,marca.tz),"rgbd");
	    inMemory=true;
	  }
}
void TagsList::borraMarca(int id)
{
	  if(mapa.value(id).clock.elapsed()>300)
	    mapa.remove(id);
}
float TagsList::distancia(int initMark)
{
	  Mark m = get(initMark);
	   std::cout << m.tx <<" "<<m.tz<< std::endl;
	  QMutexLocker ml(&mutex);
	  borraMarca(initMark);
	  float d = sqrt(pow(m.tx,2) + pow(m.tz,2));
	  return d;
}
bool TagsList::existe(int id)
{
  QMutexLocker ml(&mutex);
  borraMarca(id);
  return mapa.contains(id) or inMemory;

}
TagsList::Mark TagsList::get(int id)
{
QMutexLocker ml(&mutex);
	  if (mapa.contains(id)){
	    return mapa.value(id);
	  }else{
	    Mark m;
	   QVec reality= inner ->transform("rgbd",memory,"world");
	    m.id=id;
	    m.tx=reality.x();
	    m.ty=reality.y();
	    m.tz=reality.z();
	    return m; 
	  }
}

int TagsList::getM()
{
return initMark;
}
void TagsList::set(int init)
{
  initMark = init;
}

