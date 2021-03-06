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
#include <osgviewer/osgview.h>
#include <innermodel/innermodelviewer.h>
#include <innermodel/innermodeldraw.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	

	
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	float go(const TargetPose &target);
	NavState getState();
	float goBackwards(const TargetPose &target);
	float goReferenced(const TargetPose &target, const float xRef, const float zRef, const float threshold);
	float changeTarget(const TargetPose &target);
	void stop();
	void mapBasedTarget(const NavigationParameterMap &parameters);

public slots:
	void compute(); 	

private:
	
	typedef struct
	{
	  QVec target, subTarget;
	  float rot;
	  bool active, activeSub=false;
	} currentTarget;
	
	RoboCompTrajectoryRobot2D::NavState nState;
	InnerModel* inner;
	TLaserData ldata, ldataR;
	TBaseState bState;
	currentTarget ctarget;
	QGraphicsScene scene;
	
	void crearSubTarget();
	void irASubTarget();
	bool hayCamino();
	void irATarget();
	bool heLlegado();
	void pintar();
	void stopRobot();
	void drawTarget(const QVec& target);
	void undrawTarget(const QString &name);
	QTime elapsedTime;
	
	
	enum class State  {INIT, IDLE, WORKING, FINISH, TURN};
	State state = State::INIT;
	
	OsgView *osgView;
	InnerModelViewer *innerViewer;
	
};

#endif
