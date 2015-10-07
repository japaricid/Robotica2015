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
	void Move();


public slots:
	void compute();
	

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
	}Mark;
	
	
	QMap<int,Mark> mapa;
	QMutex mutex;
	
	
	void add(const RoboCompAprilTags::tag &t)
	{
	  Mark marca;
	  marca.id = t.id;
	  marca.rx = t.rx;
	  marca.ry = t.ry;
	  marca.rz = t.rz;
	  marca.tx = t.tx;
	  marca.ty = t.ty;
	  marca.tz = t.tz;
	  mapa.insert(t.id,marca);
	};
	Mark get(){};
      };
   
    MarksList MarkList;
    
    
};

#endif

