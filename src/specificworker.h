/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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
	void Buscar(int initId);
	

private:
      struct MarksList{
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
	
	
	void add(const RoboCompAprilTags::tag &t)
	{
	  QMutexLocker ml(&mutex);
	  Mark marca;
	  marca.id = t.id;
	  marca.rx = t.rx;
	  marca.ry = t.ry;
	  marca.rz = t.rz;
	  marca.tx = t.tx;
	  marca.ty = t.ty;
	  marca.tz = t.tz;
	  marca.clock=QTime::currentTime();
	  mapa.insert(t.id,marca);
	};
	
	 Mark get(int id){
	  QMutexLocker ml(&mutex);
	  return mapa.value(id);
	};
	
	bool existe(int id){
	  QMutexLocker ml(&mutex);
	  borraMarca(id);
	  return mapa.contains(id);
	};
	
	float distancia(int initId)
	{
	

	  Mark m = get(initId);
	   std::cout << m.tx <<" "<<m.tz<< std::endl;
	  QMutexLocker ml(&mutex);
	  float d = sqrt(pow(m.tx,2) + pow(m.tz,2));
	  return d;
	};
	void borraMarca(int id)
	{
	  if(mapa.value(id).clock.elapsed()>300)
	    mapa.remove(id);
	};
      };
   
    MarksList MarkList;
    
    
    enum class State {INIT, MOVE, SEARCH, FINISH};
    State estado = State::INIT;
    int initId = 0;
    
};

#endif

