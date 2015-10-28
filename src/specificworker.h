/*
 *    Copyright (C) 2015 by CHIFRI Y CIU
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <qt4/QtCore/qmap.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void newAprilTag(const tagsList &tags);


public slots:
	void compute();
	void Move();
	void Buscar(int initMark);
	void wait();
	void wall();

private:
      
 
      struct MarksList{
	MarksList(InnerModel *inner_) : inner(inner_){
	  inMemory=false;
	  initMark=0;
	};
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
	bool inMemory;	
	int initMark;
	
	void add(const RoboCompAprilTags::tag &t)
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
	  
	};
	
	 Mark get(int id){
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
	  
	};
	
	bool existe(int id){
	  QMutexLocker ml(&mutex);
	  borraMarca(id);
	  return mapa.contains(id) or inMemory;
	};
	
	float distancia(int initMark) 
	{
	

	  Mark m = get(initMark);
	   std::cout << m.tx <<" "<<m.tz<< std::endl;
	  QMutexLocker ml(&mutex);
	  borraMarca(initMark);
	  float d = sqrt(pow(m.tx,2) + pow(m.tz,2));
	  return d;
	};
	
	void borraMarca(int id)
	{
	  if(mapa.value(id).clock.elapsed()>300)
	    mapa.remove(id);
	};
      };
   
    MarksList* MarkList;
    
    
    enum class State {INIT, MOVE, SEARCH, FINISH, WAIT, WALL};
    State estado = State::INIT;
   // int initId = 0;
    TLaserData ldata;
 
    InnerModel* inner;
  
    
};

#endif

