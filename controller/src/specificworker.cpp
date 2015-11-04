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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
   inner = new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
   state.state="IDLE";
  
 
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
 

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{



	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
 
  try
  {
     differentialrobot_proxy->getBaseState(bState);
     ldata = laser_proxy->getLaserData();
     inner->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);	//actualiza los valores del robot en el arbol de memoria
     
     if( state.state == "WORKING")
     {
	if( heLlegado() )
	{ qDebug()<<"he llegado";
	  differentialrobot_proxy->setSpeedBase(0,0);
	  state.state = "FINISH";
	  return;
	}
      
       else if(hayCamino())
       {
	   irATarget(); 
       }
       else if(ctarget.subtarget == true)
       {
	  irASubTarget(); 
       }
       else
       {
	 crearSubTarget();
       }
    }
  }
  catch(const Ice::Exception &e)
  {
    std::cout << "Error reading from Camera" << e << std::endl;
  }
}

void SpecificWorker::irASubTarget()
{
  qDebug()<<"ir Target";  
    QVec t = inner->transform("rgbd", ctarget.subtarget, "world");
    float alpha =atan2(t.x(), t.z());
    float r= 0.3*alpha;
    float d = t.norm2();
    if(d<100)
    {
      ctarget.activeSub=false;
    }else
    {
      if(d>400)d=400;
	differentialrobot_proxy->setSpeedBase(d,r);
    }
}


void SpecificWorker::crearSubTarget()
{
  qDebug()<<"creando Target";
   uint i;
  float dt;
  QVec t = inner->transform("rgbd", ctarget.target, "world");
  float d = t.norm2();
  float alpha =atan2(t.x(), t.z() );
  
  for(i = 0; i<ldata.size(); i++)
  {
      if(ldata[i].angle >= alpha)
      {
	if(d<ldata[i].dist)
	{
	  dt=ldata[i].dist;
	 break;
	}
      } 
  }
  
  for(uint j = i;j<ldata.size();j++){
      if(ldata[j].dist>dt+(dt*0.2))
      {
	ctarget.subtarget=inner->transform("world", QVec::vec3(ldata[j].dist *sin(ldata[j].angle),0, ldata[j].dist *cos(ldata[j].angle)), "laser");
	ctarget.activeSub=true;
      }
  }
}


bool SpecificWorker::heLlegado()
{
  QVec t = inner->transform("rgbd", ctarget.target, "world");
  qDebug()<< ctarget.target;
  float d = t.norm2();
  qDebug()<< "distancia: "<<d;
  if ( d < 400 ) 
    return true;
  else return false;
  
}


bool SpecificWorker::hayCamino()
{
 
  int i;
  
  QVec t = inner->transform("rgbd", ctarget.target, "world");
  float d = t.norm2();
  float alpha =atan2(t.x(), t.z() );
  
  for(i = 0; i<ldata.size(); i++)
  {
      if(ldata[i].angle >= alpha)
      {
	if( ldata[i].dist < d)
	{
	  return false;
	}
	else
	{
	  qDebug()<<"hay camino";
	  return true;
	}
      } 
  }
}

void SpecificWorker::irATarget()
{
   
  qDebug()<<"andar";

    QVec t = inner->transform("rgbd", ctarget.target, "world");
    float alpha =atan2(t.x(), t.z());
    float r= 0.3*alpha;
    float d = t.norm2();
    if(d>400)d=400;
    differentialrobot_proxy->setSpeedBase(d,r);
  
}



//////////////////////////////////////////777
////////////////////////////////////////////

float SpecificWorker::go(const TargetPose &target)
{
 qDebug()<<"GO";
 ctarget.target = QVec::vec3(target.x, target.y, target.z);
 ctarget.active = true;
 state.state = "WORKING";
 
}

NavState SpecificWorker::getState()
{
  return state;
}

void SpecificWorker::stop()
{

}






