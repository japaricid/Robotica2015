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
	graphicsView->setScene(&scene);
	graphicsView->show();
	graphicsView->scale(3,3);
   
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
 

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	
	timer.start(1000);

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
     // reloj = QTime::currentTime();
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

//	pintar();
}


void SpecificWorker::pintar()
{
	static QGraphicsPolygonItem *p;
	static QGraphicsLineItem *l, *sr, *sl, *safety;
	const float R = 400; //Robot radius
	const float SAFETY = 600;

	scene.removeItem(p);
	scene.removeItem(l);
	scene.removeItem(sr);
	scene.removeItem(sl);
	scene.removeItem(safety);
	
	//Search the first increasing step from the center to the right
	uint i,j;
	for(i=ldata.size()/2; i>0; i--)
	{
		if( (ldata[i].dist - ldata[i-1].dist) < -R )
		{
			if(i<=1) 
			{ i=0; break;}
			uint k=i-2;
			while( (k >= 0) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[i-1].angle)) < R ))
			{ k--; }
			i=k;
			break;
		}
	}
	for(j=ldata.size()/2; j<ldata.size()-1; j++)
	{
		if( (ldata[j].dist - ldata[j+1].dist) < -R )
		{
			if(j>ldata.size()-2)
			{
				j=ldata.size()-1;
				break;
			}
			uint k=j+2;
			while( (k < ldata.size()) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[j+1].angle)) < R ))
			{ k++; }
			j=k;
			break;
		}
	}
	
	safety = scene.addLine(QLine(QPoint(0,-SAFETY/100),QPoint(ldata.size(),-SAFETY/100)), QPen(QColor(Qt::yellow)));
	sr = scene.addLine(QLine(QPoint(i,0),QPoint(i,-40)), QPen(QColor(Qt::blue)));
	sl = scene.addLine(QLine(QPoint(j,0),QPoint(j,-40)), QPen(QColor(Qt::magenta)));
	
	//DRAW		
	QPolygonF poly;
	int x=0;
	poly << QPointF(0, 0);
	
	for(auto d : ldata)
		poly << QPointF(++x, -d.dist/100); // << QPointF(x+5, d.dist) << QPointF(x+5, 0);
	poly << QPointF(x, 0);

	l = scene.addLine(QLine(QPoint(ldata.size()/2,0),QPoint(ldata.size()/2,-20)), QPen(QColor(Qt::red)));
	p = scene.addPolygon(poly, QPen(QColor(Qt::green)));
	
	scene.update();
	
	
}


void SpecificWorker::irASubTarget()
{
  qDebug()<<"ir subTarget";  
    QVec t = inner->transform("laser", ctarget.subtarget, "world");
    float alpha =atan2(t.x(), t.z());
    float r= 0.2*alpha;
    float d = t.norm2();
	qDebug()<<"distacia subTarget"<<d;
    if(d<400)
    {
      ctarget.activeSub=false;
      differentialrobot_proxy->setSpeedBase(0,0);
	  sleep(1);
    }else
    {
      if( fabs(r) > 0.1) d = 0;
      if(d>300)
		  d=150;
	  differentialrobot_proxy->setSpeedBase(d,r);
       //differentialrobot_proxy->setSpeedBase(0,0);
    }

}


void SpecificWorker::crearSubTarget()
{
  
 //  uint i;
  float dt;
  QVec t = inner->transform("rgbd", ctarget.target, "world");
  float d = t.norm2();
  float alpha =atan2(t.x(), t.z() );
  qDebug()<<"CREANDO SUBTARGET";

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
  
  //Search the first increasing step from the center to the right
	uint i,j;
	const int R =600;
	for(i=ldata.size()/2; i>5; i--)
	{
		if( (ldata[i].dist - ldata[i-1].dist) < -R )
		{
			if(i<=7) 
			{ 
				i=0; 
				break;
			}
			uint k=i-2;
			while( (k >= 0) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[i-1].angle)) < R ))
			{ k--; }
			i=k;
			break;
		}
	}
	for(j=ldata.size()/2-5; j<ldata.size()-1; j++)
	{
		if( (ldata[j].dist - ldata[j+1].dist) < -R )
		{
			if(j>ldata.size()-3)
			{
				j=ldata.size()-1;
				break;
			}
			uint k=j+2;
			while( (k < ldata.size()) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[j+1].angle)) < R ))
			{ k++; }
			j=k;
			break;
		}
	}
	
	QVec sI = inner->transform("world", QVec::vec3(ldata[j].dist *sin(ldata[j].angle),0, ldata[j].dist *cos(ldata[j].angle)), "laser");
	QVec sD = inner->transform("world", QVec::vec3(ldata[i].dist *sin(ldata[i].angle),0, ldata[i].dist *cos(ldata[i].angle)), "laser");
	
	if( (sI-ctarget.target).norm2() > (sD-ctarget.target).norm2() ) 
		ctarget.subtarget=sD;
	else
		ctarget.subtarget=sI;
		
	ctarget.activeSub=true;

  
  
//   for(i=5; i<ldata.size()-5;i++)
//   {
//     if(ldata[i-1].dist - ldata[i].dist > 400) 
//     {
//       ctarget.subtarget=inner->transform("world", QVec::vec3(ldata[i].dist *sin(ldata[i].angle),0, ldata[i].dist *cos(ldata[i].angle)), "laser");
//       ctarget.activeSub=true;
//       break;
//     }
//     
//     if(ldata[i+1].dist - ldata[i].dist > 400)
//     {
//      
//       ctarget.subtarget=inner->transform("world", QVec::vec3(ldata[i].dist *sin(ldata[i].angle),0, ldata[i].dist *cos(ldata[i].angle)), "laser");
//       ctarget.activeSub=true;
//       break;   
//     }
//     
//   }
  
  qDebug()<<ctarget.subtarget;
   // qDebug()<<"distancia subtarget"<<ldata[i].dist;
  
  
 // differentialrobot_proxy->setSpeedBase(0,0.5);
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
	
		if( ldata[i].dist < d )
		{
			return false;
		}
		else
		{
			ctarget.activeSub=false;
			qDebug()<<"hay camino"<<ldata[i].angle;
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
      if( fabs(r) > 0.1) d = 0;
      if(d>300)d=300;
      differentialrobot_proxy->setSpeedBase(d,r);
    }
  
}



//////////////////////////////////////////777
////////////////////////////////////////////

float SpecificWorker::go(const TargetPose &target)
{
 //QMutexLocker ml(&mutex);
 qDebug()<<"GO";
 ctarget.target = QVec::vec3(target.x, target.y, target.z);
 ctarget.active = true;
 state.state = "WORKING";
 return 0;
}

NavState SpecificWorker::getState()
{
	//QMutexLocker ml(&mutex);
  return state;
}

void SpecificWorker::stop()
{

}






