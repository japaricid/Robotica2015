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
#include "specificworker.h"

/**
* \brief Default constructor
*/

///*=================================================================================================================================================0
#include "specificworker.h"

/**
* \brief Default constructor
*/
///*=========================================================================================
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
  inner = new InnerModel ("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
  MarkList = new MarksList(inner);
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

void SpecificWorker::compute( )
{
   TBaseState bState;  
   differentialrobot_proxy -> getBaseState(bState);
   inner-> updateTransformValues("base",bState.x, 0, bState.z,0,bState.alpha,0);

  
    ldata = laser_proxy->getLaserData();  
   
    switch( estado )
    {
      case State::INIT:
	std::cout << "INIT" << std::endl;
	estado = State::SEARCH;
	break;
      case State::SEARCH:
	std::cout << "SEARCH" << std::endl;
	Buscar(MarkList->initMark);
	break;
      case State::MOVE:
	  std::cout << "MOVE" << std::endl;
	  Move();
	break;
	  case State::WALL:
	std::cout << "WALL" << std::endl;
	  wall();
	break;
	case State::WAIT:
	std::cout << "WAIT" << std::endl;
	  wait();
	  break;
      case State::FINISH:	
	std::cout << "FINISH" << std::endl;
	break;
    } 
    
}

void SpecificWorker::Buscar(int initMark)
{
  std::cout << "buscando" <<initMark<< std::endl;
   
  static bool firstTime=true;
  if(MarkList->existe(initMark))
  {
    try
    {
      differentialrobot_proxy->setSpeedBase(0,0);
    }
    catch(const Ice::Exception e){
      std::cout << e << std::endl;
    }
 
    estado = State::MOVE;
    firstTime=true;
    
    return;
  }
  
  if(firstTime)
  {
    try
    {
	differentialrobot_proxy->setSpeedBase(0, 0.5);
      

    }
    catch(const Ice::Exception e){
      std::cout << e << std::endl;
    }
    firstTime=false;
   }
  
}

void SpecificWorker::wait()
{
  static bool primeraVez=true;
  static QTime reloj;
  if(primeraVez){
    reloj = QTime::currentTime();
    primeraVez=false;
    MarkList->inMemory=false;
  }
  if(reloj.elapsed() > 5000){
    estado = State::SEARCH;
    primeraVez=true;
    return;
  } 
}


void SpecificWorker::newAprilTag(const tagsList& tags)
{
  for (auto t : tags){
    MarkList->add(t);
    qDebug() << t.id;
  }

}

void SpecificWorker::Move()
{
//const float threshold = 650; //millimeters
  //float rot = 0.9;  //rads per second
  const int offset = 20;
 // int v;
  //static float B=-(M_PI/4*M_PI/4)/log(0.3);

  bool giro =false;
  
   float distance= MarkList->distancia(MarkList->initMark);
      if(MarkList->existe(MarkList->initMark))
    {
      
      if(distance<400)
	
      {
	//parar robot
	differentialrobot_proxy->setSpeedBase(0,0);
	//std::cout << "buscando" <<initId<< std::endl;
	//qFatal("encontrado");
	
	MarkList->initMark = (MarkList->initMark + 1) % 4;
	estado = State::WAIT;
	return;
      }
    }
    else
    {
      estado = State::SEARCH;

      return;
    }
  
    try
    {
      
    //RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
    std::sort( ldata.begin()+offset, ldata.end()-offset, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
   
   
//     if((ldata.data()+offset)->angle>0){
// 	giro=false;
//       }else{
// 	giro=true;
//       }
    
    
    float angle=(ldata.data()+offset)->angle;
    float dist=(ldata.data()+offset)->dist;
    
    if(dist<450)
    {   
	estado = State::WALL;
	return;
    }
    else{
        float tx= MarkList->get(MarkList->initMark).tx;
	float tz= MarkList->get(MarkList->initMark).tz;
	float r= atan2(tx, tz);
	differentialrobot_proxy->setSpeedBase(150, 0.4*r);
    }
//     v=0.5*dist;
//     if(v>500)  v=500;
//     
//     rot=exp(-(angle*angle)/B)/(dist/500);
//     if(giro){
//       differentialrobot_proxy->setSpeedBase(v, rot);
//     }else{
//       differentialrobot_proxy->setSpeedBase(v, -rot);
//     }
//    // qDebug()<<v<<rot;
   
        
    }
      catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}

void SpecificWorker::wall()
{
    std::cout << "WALL" << std::endl;
    RoboCompLaser::TLaserData ldataCopy = ldata;
    int l = ldataCopy.size();
    float rot;
    const int offset = 30;
   
    std::sort( ldataCopy.begin()+offset, ldataCopy.end()-offset, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
    //std::sort( ldataCopy.begin()+l/2, ldataCopy.end()-5, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
    
    if((ldataCopy.data() + offset )->dist > 400)
    {
      estado = State::MOVE;
      return;
    }
    
      differentialrobot_proxy->setSpeedBase(40, -0.4);                  
      usleep(1000000);
}
/*
   if( (ldata.data()+20)->dist < threshold)
    {
     //   std::cout << ldata.front().dist << std::endl;
      if((ldata.data()+20)->angle>0){
	giro=false;
      }else{
	giro=true;
      }
      
	if(giro){
	differentialrobot_proxy->setSpeedBase(5, Vgiro);//girar derecha
	usleep(rand()%(1500000-100000 + 1) + 100000); 
	
	}
	else{
	  differentialrobot_proxy->setSpeedBase(5, -Vgiro);//girar izquierda
	  usleep(rand()%(1500000-100000 + 1) + 100000); 
	 
	}
    }
    
    else
    {
     
      if(((ldata.data()+20)->dist)>700){
	x=700;
      }else{
	 x=(ldata.data()+20)->dist;
      }
        differentialrobot_proxy->setSpeedBase(x, 0); // velocidad recta
	//giro=not giro;
	
    }   
    */




