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
     inner->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
     
     if( state.state == "WORKING")
     {
	if( heLlegado() )
	{ 
	  qDebug()<<"he llegado";
	  differentialrobot_proxy->setSpeedBase(0,0);
	  state.state = "FINISH";
	  sleep(2);
	   state.state = "IDLE";
	  return;
	}
      
       else if(hayCamino())
       {
	   irATarget(); 
       }
       else if(ctarget.activeSub == true)
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
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
}

void SpecificWorker::irASubTarget()
{
  qDebug()<<"ir subTarget";  
    QVec t = inner->transform("laser", ctarget.subtarget, "world");
    float alpha =atan2(t.x(), t.z());
    float r= 0.4*alpha;
    float d = t.norm2();
    if(d<100)
    {
      ctarget.activeSub=false;
      differentialrobot_proxy->setSpeedBase(0,0);
	  sleep(1);
    }else
    {
      if( fabs(r) > 0.2) d = 0;
      if(d>300)d=300;
	  differentialrobot_proxy->setSpeedBase(d,r);
       //differentialrobot_proxy->setSpeedBase(0,0);
    }
}


void SpecificWorker::crearSubTarget()
{
  
   uint i;
  float dt;
  QVec t = inner->transform("rgbd", ctarget.target, "world");
  float d = t.norm2();
  float alpha =atan2(t.x(), t.z() );

  /*
  for(i = 5; i<ldata.size()-5; i++)
  {
      if(ldata[i].angle < alpha)
      {
	if(d>ldata[i].dist)
	{
	
	  dt=ldata[i].dist;
	 break;
	}
      } 
  }
  
  for(uint j = i;j<ldata.size()-5;j++){
   
     
      if(ldata[j].dist> (dt+(dt*0.2)) and ldata[j].angle < 0)
      {
	qDebug()<<"creando subTarget";
	
	ctarget.subtarget=inner->transform("world", QVec::vec3(ldata[j].dist *sin(ldata[j].angle)-2000,0, ldata[j].dist *cos(ldata[j].angle)), "laser");
	ctarget.activeSub=true;
	break;
      }
      
  }*/
  for(i=5; i<ldata.size()-5;i++)
  {
    if(ldata[i-1].dist - ldata[i].dist > 400) 
    {
      ctarget.subtarget=inner->transform("world", QVec::vec3(ldata[i].dist *sin(ldata[i].angle),0, ldata[i].dist *cos(ldata[i].angle)), "laser");
      ctarget.activeSub=true;
      break;
    }
    
    if(ldata[i+1].dist - ldata[i].dist > 400)
    {
     
      ctarget.subtarget=inner->transform("world", QVec::vec3(ldata[i].dist *sin(ldata[i].angle),0, ldata[i].dist *cos(ldata[i].angle)), "laser");
      ctarget.activeSub=true;
      break;   
    }  
  }
  qDebug()<<ctarget.subtarget;
 // differentialrobot_proxy->setSpeedBase(0,0);
  //exit(-1);
}


bool SpecificWorker::heLlegado()
{
  QVec t = inner->transform("rgbd", ctarget.target, "world");
 // qDebug()<< ctarget.target;
  float d = t.norm2();
  qDebug()<< "distancia: "<<d;
  if ( d < 400 )
  {
     return true;
  }else
  {
     return false;
  }

}


bool SpecificWorker::hayCamino()
{
  
 
  uint i;
  
  QVec t = inner->transform("rgbd", ctarget.target, "world");
  float alpha =atan2(t.x(), t.z() );
  float d = t.norm2();
  float x, z;
 //int i = 50;
 for(uint i = 5; i<ldata.size()-5; i++)
  {
      if(ldata[i].angle < alpha)
      {
	if( ldata[i].dist < d)
	{
	  return false;
	}
	else
	{
	  ctarget.activeSub=false;
	  qDebug()<<"hay camino";
	  return true;
	}
      }
  }
  return false;
}

void SpecificWorker::irATarget()
{
   
     qDebug()<<  __FUNCTION__<<"andar"; 

    QVec t = inner->transform("rgbd", ctarget.target, "world");
    float alpha =atan2(t.x(), t.z());
    float r= 0.3*alpha;
    float d = 0.3*t.norm2();
    
    if(d<100)
    {
        ctarget.activeSub=false;
        differentialrobot_proxy->setSpeedBase(0,0);
	sleep(1);
      
    }else
    {
      if( fabs(r) > 0.2) d = 0;
      if(d>300)d=300;
      differentialrobot_proxy->setSpeedBase(d,r);
    }
  
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






